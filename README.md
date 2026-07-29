# Lead-Acid Battery Monitoring Unit with Hybrid SOC Estimator

Master's thesis project - Computer Engineering, IGEE (ex-INELEC), University of Boumerdes
Supervised by Pr. H. Bentarzi & Dr. A. Zitouni

A complete embedded battery monitoring system for 12V lead-acid batteries, combining
industrial-grade signal acquisition with a hybrid State-of-Charge (SOC) estimation algorithm,
communicating over Modbus RTU/TCP for remote supervision.

## Overview

The system acquires battery voltage, current, and temperature through three independent
4-20 mA signal conditioning channels, estimates SOC in real time using a hybrid algorithm,
and exposes all measurements to a SCADA/PC dashboard via Modbus.

Key results: current acquisition validated over a +/-19A test range, achieving
R-squared = 0.99996 with a mean absolute reconstruction error of 0.11A.

## Features

- 3-channel 4-20 mA acquisition: voltage (0-25V), current (+/-100A), temperature (0-100C), IEC 60381-compliant
- Hybrid SOC estimation: OCV-based initialization + real-time trapezoidal Coulomb Counting, with periodic OCV re-correction after rest periods
- EMA filtering for noise suppression on all acquisition channels
- 8-condition alarm/protection module (over/undervoltage, overcurrent, overcharge, undercharge, etc.)
- Modbus RTU over RS-485 between STM32 and host
- ESP32-ETH01 Modbus RTU-to-TCP gateway for remote/Ethernet access
- Python live dashboard (pymodbus) for real-time visualization

## Hardware / Tech Stack

- MCU: STM32F103C8T6
- ADC: ADS1115 (I2C)
- Communication: Modbus RTU (RS-485), Modbus TCP (Ethernet via ESP32-ETH01, LAN8720 PHY)
- Firmware: Embedded C
- Gateway: ESP32-ETH01 (Arduino core)
- Dashboard: Python, pymodbus

## Repository Structure

```
/firmware       STM32 acquisition, SOC estimation, and Modbus RTU firmware
/gateway        ESP32-ETH01 Modbus RTU-to-TCP bridge
/dashboard      Python live monitoring dashboard (pymodbus)
/hardware       Circuit diagrams (TikZ) and calibration notes
/docs           Defense slides / supporting documentation
```

## SOC Estimation Method

1. Initialization: Open Circuit Voltage (OCV) reading mapped to SOC via a calibrated OCV-SOC curve at rest.
2. Runtime tracking: Trapezoidal Coulomb Counting integrates current over time to update SOC.
3. Drift correction: after any 30-minute rest period, SOC is re-initialized from a fresh OCV reading to correct for Coulomb Counting drift.

## Validation

Current acquisition channel tested against a reference load across +/-19A:

- R-squared = 0.99996
- Mean absolute error = 0.11A

## Authors

- Messelem Rayane
- Meriane Ramdane

## Supervisors

- Pr. H. Bentarzi
- Dr. A. Zitouni

## License

MIT (or adjust based on your university's IP policy - confirm before publishing)
