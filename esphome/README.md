# ESPHome RF Doorbell - Custom Components

This directory contains custom ESPHome components for the smart doorbell project featuring:
- **ESP32-S2** microcontroller
- **BQ25628E** battery charger IC
- **MAX17260** fuel gauge IC
- **RFM69W** sub-GHz transceiver

## Project Structure

```
esphome/
├── doorbell.yaml              # Main ESPHome configuration
├── secrets.yaml.example       # Template for secrets
└── components/
    ├── bq25628e/             # Battery charger component
    │   ├── __init__.py
    │   ├── bq25628e.h
    │   ├── bq25628e.cpp
    │   └── sensor.py
    ├── max17260/             # Fuel gauge component
    │   ├── __init__.py
    │   ├── max17260.h
    │   ├── max17260.cpp
    │   └── sensor.py
    └── rfm69/                # RF transceiver component
        ├── __init__.py
        ├── rfm69.h
        └── rfm69.cpp
```

## Components Overview

### 1. BQ25628E Battery Charger

The BQ25628E component provides I2C control and monitoring of the Texas Instruments BQ25628E battery charger.

**Features:**
- Bus voltage monitoring (VBUS)
- Battery voltage monitoring
- Charge current monitoring
- System voltage monitoring
- Configurable charge current limit (0.05A - 3.0A)
- Configurable charge voltage limit (3.84V - 4.624V)
- Configurable input current limit (0.1A - 3.2A)
- Charger status reporting
- Fault detection and reporting

**I2C Configuration:**
- Default address: `0x6B`
- Connected to I2C Bus 1 (SDA1/SCL1)
- GPIOs: SDA=GPIO8, SCL=GPIO9

**Usage Example:**
```yaml
bq25628e:
  id: battery_charger
  i2c_id: i2c_bus1
  address: 0x6B
  update_interval: 60s
  charge_current_limit: 1.0    # 1A charging current
  charge_voltage_limit: 4.2    # 4.2V for Li-Ion
  input_current_limit: 0.5     # 500mA USB input

sensor:
  - platform: bq25628e
    bq25628e_id: battery_charger
    bus_voltage:
      name: "Charger Input Voltage"
    battery_voltage:
      name: "Battery Voltage"
    charge_current:
      name: "Charge Current"
```

### 2. MAX17260 Fuel Gauge

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
- GPIOs: SDA=GPIO5, SCL=GPIO6

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

### 3. RFM69 Transceiver

The RFM69 component provides wireless communication capabilities using the HopeRF RFM69W sub-GHz transceiver module.

**Features:**
- Frequency range: 290-1020 MHz (configured for 868 MHz EU band)
- Packet-based communication
- AES-128 encryption support
- Device pairing mechanism
- RSSI measurement
- Address filtering
- CRC checking
- Interrupt-driven packet reception

**SPI Configuration:**
- Connected to SPI bus
- GPIOs:
  - CLK: GPIO12 (RF.SCLK)
  - MOSI: GPIO11 (RF.MOSI)
  - MISO: GPIO13 (RF.MISO)
  - CS: GPIO15 (RF.CE)
  - Reset: GPIO14 (RF.Reset)
  - Interrupt: GPIO7 (RF.IO0/DIO0)

**Usage Example:**
```yaml
rfm69:
  id: rf_transceiver
  spi_id: spi_bus
  cs_pin: GPIO15
  reset_pin: GPIO14
  interrupt_pin: GPIO7
  frequency: 868.0              # 868 MHz (EU ISM band)
  node_id: 1                    # This device's ID
  network_id: 100               # Network identifier
  encryption_key: !secret rf_encryption_key
  is_high_power: false          # RFM69W (not RFM69HW)
  on_packet_received:
    then:
      - logger.log: "RF packet received"
      - script.execute: handle_rf_packet
```

## GPIO Pin Mapping

Based on the schematic analysis, here's the complete GPIO mapping:

| Function | GPIO | Label | Description |
|----------|------|-------|-------------|
| I2C1 SDA | GPIO8 | SDA1 | BQ25628E battery charger |
| I2C1 SCL | GPIO9 | SCL1 | BQ25628E battery charger |
| I2C2 SDA | GPIO5 | SDA2 | MAX17260 fuel gauge |
| I2C2 SCL | GPIO6 | SCL2 | MAX17260 fuel gauge |
| SPI CLK | GPIO12 | RF.SCLK | RFM69 SPI clock |
| SPI MOSI | GPIO11 | RF.MOSI | RFM69 SPI MOSI |
| SPI MISO | GPIO13 | RF.MISO | RFM69 SPI MISO |
| SPI CS | GPIO15 | RF.CE | RFM69 chip select |
| RF Reset | GPIO14 | RF.Reset | RFM69 reset pin |
| RF Int | GPIO7 | RF.IO0 | RFM69 interrupt (DIO0) |
| Battery Alert | GPIO21 | BAT_ALERT | Fuel gauge alert |
| Bell Signal | GPIO1 | ~BELL_SIGNAL | Doorbell button input |
| Bell Mute | GPIO3 | BELL_OFF | Mute bell output |
| Door Opener | GPIO2 | OPEN_BUZZER | Door buzzer control |
| RF Power | GPIO10 | RF_PWR_EN | RFM69 power enable |

## Deep Sleep Configuration

The system is configured for deep sleep to maximize battery life:

1. **Wake-up triggers:**
   - RFM69 interrupt on GPIO7 (packet received)
   - Timer-based wake (every 5 minutes for status update)

2. **Sleep behavior:**
   - Enters deep sleep after 30 seconds of inactivity
   - Prevents sleep when handling doorbell press or RF packet
   - Configurable via Home Assistant

3. **Power consumption:**
   - Active: ~80mA (WiFi on)
   - Deep sleep: <1mA

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
   - RF encryption key (exactly 16 characters)

3. **Compile and upload:**
   ```bash
   esphome compile doorbell.yaml
   esphome upload doorbell.yaml
   ```

4. **Monitor logs:**
   ```bash
   esphome logs doorbell.yaml
   ```

## RF Remote Pairing

To pair a new RF remote:

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
- Bus Voltage
- Battery Voltage (from charger and fuel gauge)
- Battery Percentage
- Battery Current
- Charge Current
- System Voltage
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

**Text Sensors:**
- Charger Status
- Charger Fault
- ESPHome Version

**Buttons:**
- Restart
- Trigger Deep Sleep
- Prevent Deep Sleep

**Events:**
- esphome.doorbell_ring

## Troubleshooting

### BQ25628E not responding
- Check I2C connections (SDA1=GPIO8, SCL1=GPIO9)
- Verify I2C address (default 0x6B)
- Check power supply to BQ25628E

### MAX17260 not responding
- Check I2C connections (SDA2=GPIO5, SCL2=GPIO6)
- Verify I2C address (default 0x36)
- Check battery connection

### RFM69 not working
- Verify SPI connections
- Check reset pin connection
- Ensure antenna is properly connected
- Verify frequency matches your region (868 MHz for EU)
- Check encryption key is exactly 16 characters

### Deep sleep issues
- Check wake-up pin configuration (GPIO7)
- Verify interrupt is properly connected
- Monitor logs for sleep/wake events

## Advanced Configuration

### Custom Charge Parameters

Adjust charging for different battery types:

```yaml
bq25628e:
  charge_current_limit: 0.8    # For 800mAh battery
  charge_voltage_limit: 4.35   # For Li-HV batteries
  input_current_limit: 1.0     # For 1A wall adapter
```

### RF Network Configuration

Create multiple networks:

```yaml
rfm69:
  network_id: 200              # Different network
  node_id: 5                   # Different node
  frequency: 915.0             # US ISM band
  is_high_power: true          # If using RFM69HW
```

## References

- [BQ25628E Datasheet](https://www.ti.com/lit/gpn/bq25628e)
- [MAX17260 Datasheet](https://www.analog.com/en/products/max17260.html)
- [RFM69 Datasheet](https://www.hoperf.com/modules/rf_transceiver/RFM69W.html)
- [ESPHome Documentation](https://esphome.io)
- [ESP32-S2 Documentation](https://www.espressif.com/en/products/socs/esp32-s2)

## License

This project is open source. See the main repository LICENSE file for details.
