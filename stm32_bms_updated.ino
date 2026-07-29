#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// ─── Pin & hardware ────────────────────────────────────────────
#define CHANNEL_VOLTAGE   3
#define CHANNEL_CURRENT   0

#define ADS_LSB_MV        0.125f
#define V_SHUNT_OHM       150.0f
#define I_SHUNT_OHM       150.0f
#define V_LOOP_GAIN       0.64f
#define V_LOOP_OFFSET     4.0f
#define I_LOOP_GAIN       0.08f
#define OVERSAMPLE_N      8

#define BATTERY_CAPACITY_AH   12.0
#define TIMER_FREQ_HZ         10.0
#define DELTA_T_S             0.1

#define REST_CURRENT_THRESHOLD_A   0.30f
#define REST_TIME_REQUIRED_S       1800.0f
#define VOLTAGE_STABLE_THRESHOLD   0.03f
#define CURRENT_DEADBAND_A         0.30f
#define ZERO_CALIBRATION_SAMPLES   100

// ─── EMA filter ────────────────────────────────────────────────
#define EMA_ALPHA_V   0.05f    // tune: lower = smoother, slower
#define EMA_ALPHA_I   0.05f

// ─── Battery status codes ──────────────────────────────────────
#define STATUS_IDLE         0
#define STATUS_CHARGING     1
#define STATUS_DISCHARGING  2
#define STATUS_OVERCHARGE   3
#define STATUS_UNDERCHARGE  4

// ─── Battery voltage limits ────────────────────────────────────
#define V_OVERCHARGE    14.4f
#define V_UNDERCHARGE   11.4f
#define SOC_OVERCHARGE  100.0
#define SOC_UNDERCHARGE 20.0

// ─── Modbus ────────────────────────────────────────────────────
#define MODBUS_UART       Serial1          // USART1 – PA9 TX / PA10 RX
#define MODBUS_BAUD       115200
#define MODBUS_DE_PIN     PA8
#define MODBUS_SLAVE_ID   1
#define MODBUS_BUF_SIZE   64

// ─── Debug serial (USART2 – PA2=TX, PA3=RX) ───────────────────
HardwareSerial DebugSerial(PA3, PA2);

// ─── Timer ─────────────────────────────────────────────────────
HardwareTimer *MyTimer = new HardwareTimer(TIM2);
volatile bool timerFlag = false;

// ─── BMS state ─────────────────────────────────────────────────
double Q_As = 0.0;
double Ilast = 0.0;
double SOC   = 0.0;

float Vbatt_V     = 0.0f;
float Vbatt_raw_V = 0.0f;
float Ibat_A      = 0.0f;
float Iloop_V_mA  = 0.0f;
float Iloop_I_mA  = 0.0f;
float I_LOOP_OFFSET_CAL = 12.0f;

// ─── EMA state ─────────────────────────────────────────────────
float emaVoltage     = 0.0f;
float emaCurrent     = 0.0f;
bool  emaVoltageInit = false;
bool  emaCurrentInit = false;

float  lastRestVoltage = 0.0f;
double restTimer_s     = 0.0;
bool   ocvCorrectionDone = false;
uint16_t battStatus    = STATUS_IDLE;

// ───────────────────────────────────────────────────────────────
//  Battery status
// ───────────────────────────────────────────────────────────────
uint16_t getBatteryStatus() {
  if (Vbatt_V > V_OVERCHARGE || SOC >= SOC_OVERCHARGE)
    return STATUS_OVERCHARGE;
  if (Vbatt_V < V_UNDERCHARGE || SOC <= SOC_UNDERCHARGE)
    return STATUS_UNDERCHARGE;
  if (Ibat_A > REST_CURRENT_THRESHOLD_A)
    return STATUS_CHARGING;
  if (Ibat_A < -REST_CURRENT_THRESHOLD_A)
    return STATUS_DISCHARGING;
  return STATUS_IDLE;
}

// ───────────────────────────────────────────────────────────────
//  Modbus RTU helpers
// ───────────────────────────────────────────────────────────────
uint16_t modbusCRC(uint8_t *buf, uint8_t len) {
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

void modbusSetTX() { digitalWrite(MODBUS_DE_PIN, HIGH); }
void modbusSetRX() { digitalWrite(MODBUS_DE_PIN, LOW);  }

void modbusSend(uint8_t *buf, uint8_t len) {
  modbusSetTX();
  delayMicroseconds(100);
  MODBUS_UART.write(buf, len);
  MODBUS_UART.flush();
  delayMicroseconds(100);
  modbusSetRX();
}

void modbusSendException(uint8_t func, uint8_t code) {
  uint8_t resp[5];
  resp[0] = MODBUS_SLAVE_ID;
  resp[1] = func | 0x80;
  resp[2] = code;
  uint16_t crc = modbusCRC(resp, 3);
  resp[3] = crc & 0xFF;
  resp[4] = crc >> 8;
  modbusSend(resp, 5);
}

uint16_t modbusGetRegister(uint16_t reg) {
  switch (reg) {
    case 0: return (uint16_t)(Vbatt_V * 1000.0f);
    case 1: return (uint16_t)((int16_t)(Ibat_A * 1000.0f));
    case 2: return (uint16_t)(SOC * 100.0);
    case 3: return (uint16_t)(restTimer_s);
    case 4: return battStatus;
    default: return 0;
  }
}

void modbusHandleFrame(uint8_t *buf, uint8_t len) {
  if (len < 6) return;

  uint16_t crc_rx   = buf[len-2] | ((uint16_t)buf[len-1] << 8);
  uint16_t crc_calc = modbusCRC(buf, len - 2);
  if (crc_rx != crc_calc) {
    DebugSerial.println("MODBUS: CRC error");
    return;
  }

  if (buf[0] != MODBUS_SLAVE_ID) return;

  uint8_t  func     = buf[1];
  uint16_t regStart = ((uint16_t)buf[2] << 8) | buf[3];
  uint16_t regCount = ((uint16_t)buf[4] << 8) | buf[5];

  DebugSerial.print("MODBUS RX: FC=");
  DebugSerial.print(func);
  DebugSerial.print(" reg=");
  DebugSerial.print(regStart);
  DebugSerial.print(" count=");
  DebugSerial.println(regCount);

  if (func != 0x03) {
    modbusSendException(func, 0x01);
    return;
  }

  if (regStart > 4 || (regStart + regCount) > 5 || regCount == 0) {
    modbusSendException(func, 0x02);
    return;
  }

  uint8_t byteCount = regCount * 2;
  uint8_t resp[5 + 64];
  resp[0] = MODBUS_SLAVE_ID;
  resp[1] = 0x03;
  resp[2] = byteCount;

  for (uint16_t i = 0; i < regCount; i++) {
    uint16_t val = modbusGetRegister(regStart + i);
    resp[3 + i*2]     = val >> 8;
    resp[3 + i*2 + 1] = val & 0xFF;
  }

  uint8_t respLen = 3 + byteCount;
  uint16_t crc = modbusCRC(resp, respLen);
  resp[respLen]     = crc & 0xFF;
  resp[respLen + 1] = crc >> 8;

  modbusSend(resp, respLen + 2);
  DebugSerial.println("MODBUS: response sent");
}

// ─── Modbus RX state machine ───────────────────────────────────
uint8_t  mbBuf[MODBUS_BUF_SIZE];
uint8_t  mbLen    = 0;
uint32_t mbLastRx = 0;
#define  MB_SILENCE_MS  4

void modbusPoll() {
  while (MODBUS_UART.available()) {
    if (mbLen < MODBUS_BUF_SIZE)
      mbBuf[mbLen++] = MODBUS_UART.read();
    else
      MODBUS_UART.read();
    mbLastRx = millis();
  }

  if (mbLen > 0 && (millis() - mbLastRx) >= MB_SILENCE_MS) {
    DebugSerial.print("MODBUS RAW (");
    DebugSerial.print(mbLen);
    DebugSerial.print("): ");
    for (uint8_t i = 0; i < mbLen; i++) {
      if (mbBuf[i] < 0x10) DebugSerial.print("0");
      DebugSerial.print(mbBuf[i], HEX);
      DebugSerial.print(" ");
    }
    DebugSerial.println();
    modbusHandleFrame(mbBuf, mbLen);
    mbLen = 0;
  }
}

// ───────────────────────────────────────────────────────────────
//  BMS functions
// ───────────────────────────────────────────────────────────────
void timerISR() { timerFlag = true; }

int32_t readADS_averaged(uint8_t channel, uint8_t samples) {
  int32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) sum += ads.readADC_SingleEnded(channel);
  return sum / samples;
}

float readCurrentLoop_mA() {
  int32_t raw = readADS_averaged(CHANNEL_CURRENT, OVERSAMPLE_N);
  return (raw * ADS_LSB_MV) / I_SHUNT_OHM;
}

float calibrateZeroCurrentOffset() {
  double sum = 0.0;
  for (int i = 0; i < ZERO_CALIBRATION_SAMPLES; i++) {
    sum += readCurrentLoop_mA();
    delay(10);
  }
  return sum / ZERO_CALIBRATION_SAMPLES;
}

float getBatteryVoltage() {
  int32_t raw = readADS_averaged(CHANNEL_VOLTAGE, OVERSAMPLE_N);
  float Vadc_mV = raw * ADS_LSB_MV;
  Iloop_V_mA  = Vadc_mV / V_SHUNT_OHM;
  Vbatt_raw_V = (Iloop_V_mA - V_LOOP_OFFSET) / V_LOOP_GAIN;
  float Vbatt = Vbatt_raw_V;
  if (Vbatt < 0.0f)  Vbatt = 0.0f;
  if (Vbatt > 15.0f) Vbatt = 15.0f;
  if (!emaVoltageInit) { emaVoltage = Vbatt; emaVoltageInit = true; }
  emaVoltage = EMA_ALPHA_V * Vbatt + (1.0f - EMA_ALPHA_V) * emaVoltage;
  return emaVoltage;
}

float getBatteryCurrent() {
  Iloop_I_mA = readCurrentLoop_mA();
  float Ibat = -((Iloop_I_mA - I_LOOP_OFFSET_CAL) / I_LOOP_GAIN);
  if (abs(Ibat) < CURRENT_DEADBAND_A) Ibat = 0.0f;
  if (Ibat >  100.0f) Ibat =  100.0f;
  if (Ibat < -100.0f) Ibat = -100.0f;
  if (!emaCurrentInit) { emaCurrent = Ibat; emaCurrentInit = true; }
  emaCurrent = EMA_ALPHA_I * Ibat + (1.0f - EMA_ALPHA_I) * emaCurrent;
  return emaCurrent;
}

double estimateSOCFromOCV(float v) {
  const int N = 11;
  float voltageTable[N] = { 11.40,11.51,11.66,11.81,11.96,12.10,12.24,12.37,12.50,12.62,12.73 };
  float socTable[N]     = { 0,10,20,30,40,50,60,70,80,90,100 };
  if (v <= voltageTable[0])   return 0.0;
  if (v >= voltageTable[N-1]) return 100.0;
  for (int i = 0; i < N-1; i++) {
    if (v >= voltageTable[i] && v <= voltageTable[i+1]) {
      return socTable[i] + ((v - voltageTable[i]) * (socTable[i+1] - socTable[i]))
                           / (voltageTable[i+1] - voltageTable[i]);
    }
  }
  return SOC;
}

double updateSOC_CoulombCounting(double Inew, double dt_s) {
  Q_As += 0.5 * (Ilast + Inew) * dt_s;
  Ilast = Inew;
  double Qmax_As = BATTERY_CAPACITY_AH * 3600.0;
  Q_As = constrain(Q_As, 0.0, Qmax_As);
  return constrain((Q_As / Qmax_As) * 100.0, 0.0, 100.0);
}

void correctSOCFromOCV(double SOC_ocv) {
  SOC  = SOC_ocv;
  Q_As = (SOC / 100.0) * BATTERY_CAPACITY_AH * 3600.0;
}

void checkRestAndCorrectSOC() {
  bool currentIsResting = abs(Ibat_A) < REST_CURRENT_THRESHOLD_A;
  bool voltageIsStable  = abs(Vbatt_V - lastRestVoltage) < VOLTAGE_STABLE_THRESHOLD;
  if (currentIsResting && voltageIsStable) restTimer_s += DELTA_T_S;
  else { restTimer_s = 0.0; ocvCorrectionDone = false; }
  lastRestVoltage = Vbatt_V;
  if (restTimer_s >= REST_TIME_REQUIRED_S && !ocvCorrectionDone) {
    double SOC_ocv = estimateSOCFromOCV(Vbatt_V);
    correctSOCFromOCV(SOC_ocv);
    ocvCorrectionDone = true;
    DebugSerial.print("OCV_CORRECTION_APPLIED\tSOC_OCV:");
    DebugSerial.println(SOC_ocv, 2);
  }
}

// ───────────────────────────────────────────────────────────────
//  Setup & Loop
// ───────────────────────────────────────────────────────────────
void setup() {
  DebugSerial.begin(115200);
  delay(1000);
  DebugSerial.println("BMS + MODBUS RTU SLAVE");

  pinMode(MODBUS_DE_PIN, OUTPUT);
  modbusSetRX();
  MODBUS_UART.begin(MODBUS_BAUD, SERIAL_8N1);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(400000);

  if (!ads.begin(0x48)) {
    DebugSerial.println("ERROR: ADS1115 not detected");
    while (1) delay(1000);
  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  DebugSerial.println("Keep current = 0 A during calibration...");
  delay(2000);

  I_LOOP_OFFSET_CAL = calibrateZeroCurrentOffset();

  DebugSerial.print("Zero offset = ");
  DebugSerial.print(I_LOOP_OFFSET_CAL, 4);
  DebugSerial.println(" mA");

  Vbatt_V = getBatteryVoltage();
  Ibat_A  = getBatteryCurrent();

  SOC  = estimateSOCFromOCV(Vbatt_V);
  Q_As = (SOC / 100.0) * BATTERY_CAPACITY_AH * 3600.0;
  Ilast = Ibat_A;
  lastRestVoltage = Vbatt_V;
  battStatus = getBatteryStatus();

  MyTimer->setOverflow(TIMER_FREQ_HZ, HERTZ_FORMAT);
  MyTimer->attachInterrupt(timerISR);
  MyTimer->resume();

  DebugSerial.println("Ready - Vbatt\tIbat\tSOC\tRest\tStatus");
}

void loop() {
  modbusPoll();

  if (!timerFlag) return;
  timerFlag = false;

  Vbatt_V = getBatteryVoltage();
  Ibat_A  = getBatteryCurrent();

  SOC = updateSOC_CoulombCounting(Ibat_A, DELTA_T_S);
  checkRestAndCorrectSOC();
  battStatus = getBatteryStatus();

  DebugSerial.print("Vbatt:"); DebugSerial.print(Vbatt_V, 3);
  DebugSerial.print("\tIbat:"); DebugSerial.print(Ibat_A, 3);
  DebugSerial.print("\tSOC:"); DebugSerial.print(SOC, 2);
  DebugSerial.print("\tRest:"); DebugSerial.print(restTimer_s, 1);
  DebugSerial.print("\tStatus:"); DebugSerial.println(battStatus);
}
