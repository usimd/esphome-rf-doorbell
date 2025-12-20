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
**Communication**: SPI (GPIO8/10/9/15)  
**Frequency**: 433 MHz ISM band  
**Library**: LowPowerLab RFM69 v1.5.3

Secure RF communication driver using the industry-standard LowPowerLab library with custom challenge-response protocol for replay attack protection.

#### Features:
- AES-128 encryption (via LowPowerLab)
- Challenge-response authentication
- Rolling code protection
- Nonce tracking (replay protection)
- Device pairing mode
- Wake-on-interrupt (GPIO7)

#### Security Layers:
1. **AES-128 Encryption** - Prevents eavesdropping
2. **Challenge-Response** - Prevents replay attacks
3. **Rolling Codes** - Sequential authentication
4. **Nonce Tracking** - Challenge uniqueness enforcement

#### Usage:
```yaml
rfm69:
  id: rf_transceiver
  spi_id: spi_bus
  cs_pin: GPIO15
  reset_pin: GPIO14
  interrupt_pin: GPIO7
  frequency: 433.0  # MHz
  node_id: 1
  network_id: 100
  encryption_key: !secret rf_encryption_key  # 16 characters
  is_high_power: false  # RFM69W variant
  use_challenge_response: true
  challenge_timeout: 30s
  on_packet_received:
    then:
      - script.execute: handle_rf_packet
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
- **LowPowerLab RFM69** v1.5.3 - RF transceiver driver

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
✅ **RFM69**: LowPowerLab RFM69 (industry standard)  
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
