# ESPHome RF Doorbell - Custom Components

This directory contains custom ESPHome components for the smart doorbell project featuring:
- **ESP32-C6-MINI-1** microcontroller (Rev3)
- **LTC4079** battery charger IC (Rev3 - component pending)
- **MAX17260** fuel gauge IC
- **Wio-SX1262** LoRa transceiver (Rev3 - component pending)

> **Note:** The firmware is currently in transition from Rev2 to Rev3 hardware. Some components
> reference legacy ICs (BQ25628E, RFM69W) and need updating for Rev3 hardware (LTC4079, Wio-SX1262).

## Project Structure

```
esphome/
├── doorbell.yaml              # Main ESPHome configuration
├── secrets.yaml.example       # Template for secrets
└── components/
    ├── bq25628e/             # Battery charger component (LEGACY - needs LTC4079)
    │   ├── __init__.py
    │   ├── bq25628e.h
    │   ├── bq25628e.cpp
    │   └── sensor.py
    ├── max17260/             # Fuel gauge component (working)
    │   ├── __init__.py
    │   ├── max17260.h
    │   ├── max17260.cpp
    │   └── sensor.py
    └── rfm69/                # RF transceiver component (LEGACY - needs SX1262)
        ├── __init__.py
        ├── rfm69.h
        └── rfm69.cpp
```

## Components Overview

### 1. LTC4079 Battery Charger (Rev3 - Pending)

The LTC4079 is a standalone linear Li-Ion battery charger with thermal regulation.

**Features:**
- Programmable charge current (via external resistor or ISL23315 digipot)
- Maximum 250mA charge current
- Thermal regulation (reduces current at high temp)
- Charge status indication (CHRG pin)
- No I2C interface (unlike BQ25628E)

**Hardware Notes:**
- Charge current set by resistor on PROG pin (or ISL23315 digital pot at U10)
- CHRG pin active-low during charging
- CE pin enables charging when low
- Connected to I2C bus at GPIO15/16 via ISL23315 for current control

> **TODO:** Create LTC4079 ESPHome component that controls charging via ISL23315 digital potentiometer

### 2. MAX17260 Fuel Gauge (Working)

The MAX17260 component interfaces with the Maxim Integrated MAX17260 fuel gauge for accurate battery monitoring.

**Features:**
- Battery voltage measurement
- State of charge (0-100%)
- Battery current (charge/discharge)
- Battery temperature
- Time to empty (minutes)
- Time to full (minutes)
- ModelGauge m5 algorithm for accuracy

**I2C Configuration:**
- Default address: `0x36`
- Connected to I2C Bus 2 (SDA2/SCL2)
- GPIOs: SDA=GPIO36, SCL=GPIO35

**Usage Example:**
```yaml
max17260:
  id: fuel_gauge
  i2c_id: i2c_bus2
  address: 0x36
  update_interval: 30s

sensor:
  - platform: max17260
    max17260_id: fuel_gauge
    voltage:
      name: "Battery Voltage (FG)"
    state_of_charge:
      name: "Battery Percentage"
    current:
      name: "Battery Current"
    temperature:
      name: "Battery Temperature"
    time_to_empty:
      name: "Battery Time to Empty"
```

### 3. Wio-SX1262 LoRa Transceiver (Rev3 - Pending)

The Wio-SX1262 is a LoRa module based on the Semtech SX1262 chip.

**Features:**
- Frequency range: 150-960 MHz (configured for 868 MHz EU or 915 MHz US)
- LoRa modulation for long range
- Low power consumption
- SPI interface

**SPI Configuration (Rev3):**
- Connected to SPI bus
- GPIOs:
  - CLK: GPIO8
  - MOSI: GPIO10
  - MISO: GPIO9
  - CS: GPIO11
  - Reset: GPIO7
  - DIO1: GPIO6 (interrupt)
  - BUSY: GPIO5

> **TODO:** Create SX1262 ESPHome component (or adapt existing LoRa libraries)

### 4. Legacy BQ25628E Component (Rev2 only)

**Note:** This component is for Rev2 hardware only. Rev3 uses LTC4079.

The BQ25628E component provides I2C control and monitoring of the Texas Instruments BQ25628E battery charger.

**Features:**
- Bus voltage monitoring (VBUS)
- Battery voltage monitoring
- Charge current monitoring
- Configurable charge parameters via I2C

## GPIO Pin Mapping (Rev3 - ESP32-C6-MINI-1)

Based on the schematic analysis, here's the complete GPIO mapping:

| Function | GPIO | Label | Description |
|----------|------|-------|-------------|
| I2C1 SDA | GPIO16 | SDA1 | LTC4079 charger (via ISL23315) |
| I2C1 SCL | GPIO15 | SCL1 | LTC4079 charger (via ISL23315) |
| I2C2 SDA | GPIO36 | SDA2 | MAX17260 fuel gauge |
| I2C2 SCL | GPIO35 | SCL2 | MAX17260 fuel gauge |
| SPI CLK | GPIO8 | RF.SCLK | Wio-SX1262 SPI clock |
| SPI MOSI | GPIO10 | RF.MOSI | Wio-SX1262 SPI MOSI |
| SPI MISO | GPIO9 | RF.MISO | Wio-SX1262 SPI MISO |
| SPI CS | GPIO11 | RF.CE | Wio-SX1262 chip select |
| RF Reset | GPIO7 | RF.Reset | Wio-SX1262 reset pin |
| RF DIO1 | GPIO6 | RF.DIO1 | Wio-SX1262 interrupt |
| RF BUSY | GPIO5 | RF.BUSY | Wio-SX1262 busy status |
| Battery Alert | GPIO21 | BAT_ALERT | Fuel gauge alert |
| Bell Signal | GPIO13 | ~BELL_SIGNAL | Doorbell button input |
| Bell Mute | GPIO12 | BELL_OFF | G3VM-63ER control |
| Door Opener | GPIO14 | OPEN_BUZZER | G3VM-31DR control |
| RF Power | GPIO33 | RF_PWR_EN | Wio-SX1262 LDO enable |
| Charger CE | GPIO18 | CHG_CE | LTC4079 enable (active low) |

## Deep Sleep Configuration

The system is configured for deep sleep to maximize battery life:

1. **Wake-up triggers:**
   - Wio-SX1262 DIO1 on GPIO6 (LoRa packet received)
   - Bell signal on GPIO13 (doorbell pressed)
   - Timer-based wake (every 60 seconds for status update)

2. **Sleep behavior:**
   - TPS63900 maintains 3.3V rail with <400nA quiescent
   - ESP32-C6 enters deep sleep between wake events
   - Configurable sleep duration via Home Assistant

3. **Power consumption:**
   - Active: ~25mA (WiFi on)
   - Deep sleep: <10uA (TPS63900 quiescent)

## Setup Instructions

1. **Copy secrets template:**
   ```bash
   cd esphome
   cp secrets.yaml.example secrets.yaml
   ```

2. **Edit secrets.yaml with your credentials:**
   - WiFi SSID and password
   - API encryption key
   - OTA password
   - LoRa encryption key (if using encryption)

3. **Compile and upload:**
   ```bash
   esphome compile doorbell.yaml
   esphome upload doorbell.yaml
   ```

4. **Monitor logs:**
   ```bash
   esphome logs doorbell.yaml
   ```

## LoRa Remote Pairing (Pending SX1262 Component)

To pair a new LoRa remote:

1. Call the service in Home Assistant:
   ```yaml
   service: esphome.esphome_rf_doorbell_enter_pairing_mode
   ```

2. Press the button on the remote you want to pair

3. The device ID will be automatically added to the paired devices list

4. Exit pairing mode:
   ```yaml
   service: esphome.esphome_rf_doorbell_exit_pairing_mode
   ```

## Home Assistant Integration

Once configured, the following entities will be available in Home Assistant:

**Sensors:**
- Battery Voltage (from fuel gauge)
- Battery Percentage
- Battery Current
- Battery Temperature
- Time to Empty
- Time to Full
- WiFi Signal
- Uptime

**Binary Sensors:**
- Battery Alert
- Bell Signal

**Switches:**
- Bell Mute
- Door Opener
- RF Power Enable

**Buttons:**
- Restart
- Trigger Deep Sleep

**Events:**
- esphome.doorbell_ring

## Troubleshooting

### MAX17260 not responding
- Check I2C connections (SDA2=GPIO36, SCL2=GPIO35)
- Verify I2C address (default 0x36)
- Check battery connection

### Wio-SX1262 not working (once component exists)
- Verify SPI connections
- Check reset pin connection (GPIO7)
- Check BUSY pin state (GPIO5)
- Ensure antenna is properly connected
- Verify frequency matches your region (868 MHz for EU, 915 MHz for US)

### Deep sleep issues
- Check wake-up pin configuration (GPIO6 for LoRa, GPIO13 for bell)
- Verify interrupt is properly connected
- Monitor logs for sleep/wake events

### Charger not working
- Check CE pin state (GPIO18 should be LOW to enable)
- Verify ISL23315 digital potentiometer is responding on I2C
- Check VBUS input voltage

## Hardware Revision Compatibility

| Component | Rev2 (ESP32-S2) | Rev3 (ESP32-C6) |
|-----------|-----------------|-----------------|
| bq25628e/ | Working | N/A (use LTC4079) |
| max17260/ | Working | Working |
| rfm69/ | Working | N/A (use SX1262) |
| ltc4079/ | N/A | Pending |
| sx1262/ | N/A | Pending |

## References

- [LTC4079 Datasheet](https://www.analog.com/en/products/ltc4079.html)
- [MAX17260 Datasheet](https://www.analog.com/en/products/max17260.html)
- [SX1262 Datasheet](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262)
- [Wio-SX1262 Wiki](https://wiki.seeedstudio.com/Wio-SX1262/)
- [ESP32-C6 Documentation](https://www.espressif.com/en/products/socs/esp32-c6)
- [ESPHome Documentation](https://esphome.io)

## License

This project is open source. See the main repository LICENSE file for details.
