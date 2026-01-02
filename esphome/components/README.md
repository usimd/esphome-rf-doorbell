# ESPHome Custom Components for RF Doorbell

This directory contains custom ESPHome components for the RF doorbell project.

## Components

### bq25628e - Battery Charger Driver
**I2C Address**: 0x6B (hardware-dependent, Adafruit library defaults to 0x6A)  
**Bus**: I2C Bus 1 (GPIO16/GPIO15)  
**Library**: Adafruit_BQ25628E v1.0.0

Wrapper for the official Adafruit BQ25628E library. Provides complete control over charging parameters and monitoring for the TI BQ25628E battery charger IC.

#### Features:
- Configurable charge current (0.05 - 3.0 A)
- Configurable charge voltage (3.84 - 4.624 V)
- Configurable input current limit (0.1 - 3.2 A)
- ADC monitoring (VBUS, VBAT, VSYS, ICHG)
- Fault detection and reporting
- Runtime parameter updates from Home Assistant
- Uses battle-tested Adafruit library for reliability

#### Usage:
```yaml
bq25628e:
  id: battery_charger
  i2c_id: i2c_bus1
  address: 0x6B
  update_interval: 60s
  charge_current_limit: 1.0  # Amps
  charge_voltage_limit: 4.2  # Volts
  input_current_limit: 0.5   # Amps
```

---

### max17260 - Fuel Gauge Driver
**I2C Address**: 0x36  
**Bus**: I2C Bus 2 (GPIO36/GPIO35)  
**Library**: Custom (no official Arduino library available)

Driver for the Maxim MAX17260 fuel gauge implementing ModelGauge m5 algorithm for accurate battery state estimation.

#### Features:
- State of charge (SOC) percentage
- Battery voltage, current, temperature
- Time to empty (TTE) / Time to full (TTF)
- Cycle counting
- Alert on low battery (GPIO37)
- **Note**: No official library exists; custom implementation based on datasheet

#### Usage:
```yaml
max17260:
  id: fuel_gauge
  i2c_id: i2c_bus2
  address: 0x36
  update_interval: 30s
```

---

### rfm69 - RF Transceiver Driver
**Communication**: SPI (GPIO8/10/9/11)  
**Interrupt**: GPIO6 (DIO0/PayloadReady)  
**Frequency**: 433 MHz ISM band  
**Protocol**: RadioHead-compatible packet format

Custom RF transceiver driver implementing secure communication with hardware AES encryption, challenge-response authentication, and device pairing.

#### Features:
- **AES-128 Hardware Encryption** - Built into RFM69 chip (registers 0x3E-0x4D)
- **Challenge-Response Protocol** - ESP32 hardware RNG generates 32-bit challenges
- **Device Pairing** - Serial number-based device registration (up to 10 devices)
- **One-Time Challenge Use** - Each challenge invalidated after use
- **Configurable Timeout** - Challenges expire after configurable period (default 30s)
- **Wake-on-Interrupt** - DIO0 (PayloadReady) triggers ESP32 wake from deep sleep
- **Temperature Sensor** - Built-in RFM69 temperature reading
- **Debug Register Dump** - Full register state logging for diagnostics

#### Protocol Commands:
| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| PING | 0x01 | Remote→Receiver | Request challenge |
| CHALLENGE | 0x02 | Receiver→Remote | 4-byte random challenge |
| DOOR_OPEN | 0x03 | Remote→Receiver | Open request with challenge |
| ACK | 0x04 | Receiver→Remote | Response status |
| PAIR_REQUEST | 0x05 | Remote→Receiver | Request pairing |
| PAIR_CONFIRM | 0x06 | Receiver→Remote | Pairing result |
| DOORBELL_RING | 0x10 | Remote→Receiver | Doorbell button press |

#### Security Layers:
1. **AES-128 Encryption** - All packets encrypted with 16-byte key
2. **Challenge-Response** - Remote must echo receiver's challenge
3. **One-Time Use** - Challenge invalidated immediately after successful use
4. **Timeout Enforcement** - Challenges expire after configurable period
5. **Device Whitelist** - Only paired serial numbers can open door

#### Usage:
```yaml
rfm69:
  id: rf_transceiver
  spi_id: spi_bus
  cs_pin: GPIO11
  reset_pin: GPIO7
  interrupt_pin: GPIO6
  frequency: 433.0  # MHz
  node_id: 1
  network_id: 100
  encryption_key: !secret rf_encryption_key  # Exactly 16 characters
  is_high_power: false  # RFM69W (not RFM69HW)
  tx_power: 13  # dBm (-18 to +13 for RFM69W)
  challenge_timeout: 30s

  on_device_paired:
    - logger.log:
        format: "Device paired: 0x%08X"
        args: ['serial_number']

  on_door_open_request:
    - if:
        condition:
          lambda: 'return authorized;'
        then:
          - switch.turn_on: door_opener

  on_packet_received:
    - logger.log:
        format: "Packet from 0x%02X, RSSI -%d dBm"
        args: ['from_address', 'rssi']
```

#### Home Assistant Integration:
```yaml
# Enable pairing mode (auto-disables after 60s)
switch:
  - platform: template
    name: "RF Pairing Mode"
    turn_on_action:
      - lambda: 'id(rf_transceiver).set_pairing_mode(true);'
    turn_off_action:
      - lambda: 'id(rf_transceiver).set_pairing_mode(false);'

# Clear all paired devices
button:
  - platform: template
    name: "Clear Paired Remotes"
    on_press:
      - lambda: 'id(rf_transceiver).clear_all_paired_devices();'
```

---

## Loading Components

### From Local Path (Development)
```yaml
external_components:
  - source:
      type: local
      path: components
    components: [bq25628e, max17260, rfm69]
```

### From GitHub (Production)
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/usimd/esphome-rf-doorbell
      ref: main
    components: [bq25628e, max17260, rfm69]
    refresh: 1d
```

---

## GPIO Pin Mapping

All GPIO assignments verified against KiCAD schematic:

| Pin | Function | Component | Description |
|-----|----------|-----------|-------------|
| GPIO1 | ~BELL_SIGNAL | Input | Doorbell button press |
| GPIO2 | OPEN_BUZZER | Output | Door opener trigger |
| GPIO3 | BELL_OFF | Output | Bell mute control |
| GPIO5 | SDA2 | I2C | MAX17260 fuel gauge data |
| GPIO6 | SCL2 | I2C | MAX17260 fuel gauge clock |
| GPIO7 | RF_INT | Input | RFM69 interrupt / wake source |
| GPIO8 | SDA1 | I2C | BQ25628E charger data |
| GPIO9 | SCL1 | I2C | BQ25628E charger clock |
| GPIO10 | RF_PWR_EN | Output | RFM69 power enable |
| GPIO11 | RF.MOSI | SPI | RFM69 SPI data out |
| GPIO12 | RF.SCLK | SPI | RFM69 SPI clock |
| GPIO13 | RF.MISO | SPI | RFM69 SPI data in |
| GPIO14 | RF_RESET | Output | RFM69 reset |
| GPIO15 | RF.CE | Output | RFM69 chip select |
| GPIO21 | BAT_ALERT | Input | Battery alert from MAX17260 |

---

## Dependencies

### External Libraries (Auto-installed by ESPHome)
- **Adafruit BusIO** v1.16.1 - I2C/SPI abstraction layer
- **Adafruit_BQ25628E** v1.0.0 - Battery charger driver

### ESPHome Components Used
- `esphome/core` - Component base classes
- `esphome/components/i2c` - I2C communication
- `esphome/components/spi` - SPI communication  
- `esphome/components/sensor` - Sensor platform
- `esphome/components/text_sensor` - Text sensor platform
- `esphome/components/binary_sensor` - Binary sensor platform

---

## Using Official Libraries

This project uses official, well-maintained libraries where available:

✅ **BQ25628E**: Adafruit_BQ25628E (official Adafruit library)  
⚠️ **RFM69**: Custom implementation (direct register access, RadioHead-compatible)  
⚠️ **MAX17260**: Custom implementation (no official library exists)

---

## Development

### File Structure
```
components/
├── bq25628e/
│   ├── __init__.py       # Component registration and schema
│   ├── bq25628e.h        # C++ header with class definition
│   ├── bq25628e.cpp      # C++ implementation
│   └── sensor.py         # Sensor platform integration
├── max17260/
│   ├── __init__.py
│   ├── max17260.h
│   ├── max17260.cpp
│   └── sensor.py
└── rfm69/
    ├── __init__.py
    ├── rfm69.h
    └── rfm69.cpp
```

### Adding New Features

1. **Modify component code** in respective directory
2. **Test locally** using `type: local` external_components
3. **Commit and push** to GitHub repository
4. **Update ref** in external_components (or use `@always` for latest)

### Debugging

Enable DEBUG logging in your ESPHome config:
```yaml
logger:
  level: DEBUG
  logs:
    bq25628e: DEBUG
    max17260: DEBUG
    rfm69: DEBUG
```

---

## References

- **BQ25628E Datasheet**: TI Document SLUSF36
- **MAX17260 Datasheet**: Maxim Integrated 19-100365
- **RFM69 Datasheet**: HopeRF Document
- **LowPowerLab Library**: https://github.com/LowPowerLab/RFM69
- **ESPHome Custom Components**: https://esphome.io/custom/custom_component.html

---

## License

See project root LICENSE file.

---

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test thoroughly on hardware
4. Submit pull request with:
   - Description of changes
   - Test results
   - Updated documentation

---

*For complete project documentation, see the root README.md*
