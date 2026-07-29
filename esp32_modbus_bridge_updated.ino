// ================================================================
//  ESP32-ETH01  –  Modbus RTU Master  +  Modbus TCP Server
//  RTU  : Serial2 (RX=5, TX=17, DE=4) @ 115200  →  STM32 slave
//  TCP  : Ethernet, static 192.168.1.100:502
//  Register map:
//    0 – Vbatt  (uint16, mV)
//    1 – Ibat   (int16,  mA)
//    2 – SOC    (uint16, x100 → %)
//    3 – Rest   (uint16, seconds)
//    4 – Status (0=IDLE 1=CHARGING 2=DISCHARGING 3=OVERCHARGE 4=UNDERCHARGE)
// ================================================================

#include <ETH.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// ─── ETH01 pin map ─────────────────────────────────────────────
#define ETH_PHY_ADDR     1
#define ETH_PHY_MDC      23
#define ETH_PHY_MDIO     18
#define ETH_PHY_POWER    16
#define ETH_PHY_TYPE     ETH_PHY_LAN8720
#define ETH_CLK_MODE     ETH_CLOCK_GPIO0_IN

// ─── Static IP ─────────────────────────────────────────────────
IPAddress ip      (192, 168, 1, 100);
IPAddress gateway (192, 168, 1,   1);
IPAddress subnet  (255, 255, 255, 0);
IPAddress dns     (192, 168, 1,   1);

// ─── Modbus TCP ────────────────────────────────────────────────
#define MODBUS_TCP_PORT   502
WiFiServer mbTcpServer(MODBUS_TCP_PORT);

// ─── Modbus RTU (to STM32) ─────────────────────────────────────
#define MODBUS_UART       Serial2
#define MODBUS_BAUD       115200
#define MODBUS_TX_PIN     17
#define MODBUS_RX_PIN     5
#define MODBUS_DE_PIN     4
#define MODBUS_SLAVE_ID   1
#define MODBUS_TIMEOUT_MS 500
#define RTU_POLL_MS       1000

// ─── Cached register values (0–4) ──────────────────────────────
uint16_t mbRegs[5] = {0, 0, 0, 0, 0};
bool     ethReady  = false;

// ───────────────────────────────────────────────────────────────
//  RTU helpers
// ───────────────────────────────────────────────────────────────
uint16_t rtuCRC(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;
}

void rtuSetTX() { digitalWrite(MODBUS_DE_PIN, HIGH); }
void rtuSetRX() { digitalWrite(MODBUS_DE_PIN, LOW);  }

void rtuFlushRx() {
  while (MODBUS_UART.available()) MODBUS_UART.read();
}

bool rtuPoll() {
  uint8_t req[8];
  req[0] = MODBUS_SLAVE_ID;
  req[1] = 0x03;
  req[2] = 0x00; req[3] = 0x00;   // start reg 0
  req[4] = 0x00; req[5] = 0x05;   // count 5 (regs 0–4)
  uint16_t crc = rtuCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  rtuFlushRx();
  rtuSetTX();
  delayMicroseconds(500);
  MODBUS_UART.write(req, 8);
  MODBUS_UART.flush();
  delayMicroseconds(500);
  rtuSetRX();

  // Expected response: 1+1+1+10+2 = 15 bytes (5 registers)
  uint8_t  buf[32];
  uint8_t  n  = 0;
  uint32_t t0 = millis();

  while ((millis() - t0) < MODBUS_TIMEOUT_MS) {
    while (MODBUS_UART.available() && n < sizeof(buf)) {
      buf[n++] = MODBUS_UART.read();
      t0 = millis();
    }
  }

  if (n < 15) {
    Serial.print("RTU: short response (");
    Serial.print(n);
    Serial.println(" bytes)");
    return false;
  }

  uint16_t crc_rx   = buf[n-2] | ((uint16_t)buf[n-1] << 8);
  uint16_t crc_calc = rtuCRC(buf, n - 2);
  if (crc_rx != crc_calc) { Serial.println("RTU: CRC error"); return false; }
  if (buf[0] != MODBUS_SLAVE_ID || buf[1] != 0x03) { Serial.println("RTU: unexpected frame"); return false; }

  mbRegs[0] = ((uint16_t)buf[3]  << 8) | buf[4];
  mbRegs[1] = ((uint16_t)buf[5]  << 8) | buf[6];
  mbRegs[2] = ((uint16_t)buf[7]  << 8) | buf[8];
  mbRegs[3] = ((uint16_t)buf[9]  << 8) | buf[10];
  mbRegs[4] = ((uint16_t)buf[11] << 8) | buf[12];

  const char* statusStr[] = {"IDLE","CHARGING","DISCHARGING","OVERCHARGE","UNDERCHARGE"};
  Serial.print("RTU OK  Vbatt="); Serial.print(mbRegs[0]);
  Serial.print("mV  Ibat=");      Serial.print((int16_t)mbRegs[1]);
  Serial.print("mA  SOC=");       Serial.print(mbRegs[2] / 100.0f, 1);
  Serial.print("%  Rest=");       Serial.print(mbRegs[3]);
  Serial.print("s  Status=");     Serial.println(mbRegs[4] < 5 ? statusStr[mbRegs[4]] : "?");
  return true;
}

// ───────────────────────────────────────────────────────────────
//  Modbus TCP
// ───────────────────────────────────────────────────────────────
void handleTcpClient(WiFiClient &client) {
  uint8_t mbap[7];
  uint32_t t0 = millis();

  while (client.available() < 7 && (millis() - t0) < 200) delay(1);
  if (client.available() < 7) return;

  client.read(mbap, 7);

  uint16_t protoID = ((uint16_t)mbap[2] << 8) | mbap[3];
  uint16_t pduLen  = ((uint16_t)mbap[4] << 8) | mbap[5];

  if (protoID != 0) { client.stop(); return; }

  uint8_t pdu[256];
  uint8_t pduBytes = (uint8_t)(pduLen - 1);

  t0 = millis();
  while (client.available() < pduBytes && (millis() - t0) < 200) delay(1);
  if (client.available() < pduBytes) return;

  client.read(pdu, pduBytes);

  uint8_t  func     = pdu[0];
  uint16_t regStart = ((uint16_t)pdu[1] << 8) | pdu[2];
  uint16_t regCount = ((uint16_t)pdu[3] << 8) | pdu[4];

  uint8_t resp[256];
  uint8_t respPduLen;

  if (func != 0x03) {
    resp[0] = func | 0x80; resp[1] = 0x01; respPduLen = 2;
  } else if (regStart > 4 || (regStart + regCount) > 5 || regCount == 0) {
    resp[0] = func | 0x80; resp[1] = 0x02; respPduLen = 2;
  } else {
    uint8_t byteCount = regCount * 2;
    resp[0] = 0x03;
    resp[1] = byteCount;
    for (uint16_t i = 0; i < regCount; i++) {
      uint16_t val = mbRegs[regStart + i];
      resp[2 + i*2]     = val >> 8;
      resp[2 + i*2 + 1] = val & 0xFF;
    }
    respPduLen = 2 + byteCount;
  }

  uint8_t out[270];
  out[0] = mbap[0]; out[1] = mbap[1];
  out[2] = 0x00;    out[3] = 0x00;
  out[4] = (respPduLen + 1) >> 8;
  out[5] = (respPduLen + 1) & 0xFF;
  out[6] = mbap[6];
  memcpy(out + 7, resp, respPduLen);

  client.write(out, 7 + respPduLen);
}

// ───────────────────────────────────────────────────────────────
//  Ethernet event
// ───────────────────────────────────────────────────────────────
void onEthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: started");
      ETH.setHostname("bms-gateway");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH: cable connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
      ethReady = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH: disconnected");
      ethReady = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH: stopped");
      ethReady = false;
      break;
    default:
      break;
  }
}

// ───────────────────────────────────────────────────────────────
//  Setup & Loop
// ───────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== BMS Modbus Gateway ===");

  pinMode(MODBUS_DE_PIN, OUTPUT);
  rtuSetRX();
  MODBUS_UART.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  Serial.println("RTU UART ready");

  WiFi.onEvent(onEthEvent);
  ETH.begin(ETH_PHY_TYPE,
            ETH_PHY_ADDR,
            ETH_PHY_MDC,
            ETH_PHY_MDIO,
            ETH_PHY_POWER,
            ETH_CLK_MODE);

  ETH.config(ip, gateway, subnet, dns);

  Serial.print("Waiting for Ethernet");
  uint32_t t = millis();
  while (!ethReady && (millis() - t) < 5000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (ethReady) {
    mbTcpServer.begin();
    Serial.print("Modbus TCP listening on ");
    Serial.print(ETH.localIP());
    Serial.println(":502");
  } else {
    Serial.println("WARNING: Ethernet not ready – TCP server not started");
  }
}

void loop() {
  // RTU poll always runs regardless of TCP client
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll >= RTU_POLL_MS) {
    lastPoll = millis();
    rtuPoll();
  }

  if (ethReady) {
    WiFiClient client = mbTcpServer.accept();
    if (client) {
      Serial.print("TCP client: ");
      Serial.println(client.remoteIP());
      uint32_t t0 = millis();
      while (client.connected() && (millis() - t0) < 2000) {
        // RTU keeps polling while TCP client is connected
        if (millis() - lastPoll >= RTU_POLL_MS) {
          lastPoll = millis();
          rtuPoll();
        }
        if (client.available()) {
          handleTcpClient(client);
          t0 = millis();
        }
      }
      client.stop();
    }
  }
}
