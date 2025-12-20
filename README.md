# ESPHome RF Doorbell

A smart doorbell system for Home Assistant using ESPHome, featuring battery power management, RF remote control, and deep sleep optimization.

## 🌟 Features

- **ESP32-S2** microcontroller with WiFi connectivity
- **Battery Management:**
  - BQ25628E intelligent battery charger with I2C control
  - MAX17260 fuel gauge for accurate battery monitoring
  - Li-Ion battery support with USB-C charging
- **RF Communication:**
  - RFM69W sub-GHz transceiver (868/915 MHz)
  - AES-128 encryption
  - Remote pairing system (like garage door opener)
- **Deep Sleep:**
  - Wake on RF packet or doorbell press
  - Battery life optimization
  - Configurable sleep intervals
- **Home Assistant Integration:**
  - Real-time battery monitoring
  - Doorbell notifications
  - Remote door opener control
  - Bell mute functionality

## 📁 Project Structure

```
esphome-rf-doorbell/
├── base/                     # Main doorbell PCB
│   ├── esphome-rf-doorbell.kicad_pcb
│   ├── esphome-rf-doorbell.kicad_sch
│   └── bom/
├── remote/                   # Optional RF remote PCB
│   ├── remote.kicad_pcb
│   ├── remote.kicad_sch
│   └── firmware/             # PIC microcontroller firmware
├── lib/                      # KiCad component libraries
└── esphome/                  # ESPHome firmware
    ├── doorbell.yaml         # Main configuration
    ├── secrets.yaml.example  # Template for credentials
    ├── GETTING_STARTED.md    # Setup guide
    ├── README.md            # Component documentation
    └── components/          # Custom components
        ├── bq25628e/        # Battery charger driver
        ├── max17260/        # Fuel gauge driver
        └── rfm69/           # RF transceiver driver

```

## 🔧 Hardware

### Main PCB Components

- **ESP32-S2-MINI-2:** Main microcontroller
- **BQ25628E:** Li-Ion battery charger with I2C interface
- **MAX17260:** Battery fuel gauge
- **RFM69W:** Sub-GHz RF transceiver module
- **TPS629206:** Buck converter for efficient 3.3V power
- **Optocouplers:** For isolated doorbell/buzzer control

### RF Remote (Optional)

- **PIC12F microcontroller:** Low-power remote controller
- **RFM69W:** Matching RF transceiver
- **CR2032 battery:** Long-lasting coin cell power

## 🚀 Quick Start

1. **Clone the repository:**
   ```bash
   git clone https://github.com/usimd/esphome-rf-doorbell.git
   cd esphome-rf-doorbell
   ```

2. **Follow the setup guide:**
   - See [esphome/GETTING_STARTED.md](esphome/GETTING_STARTED.md) for detailed instructions
   - Component documentation in [esphome/README.md](esphome/README.md)

3. **Build the PCB:**
   - Open the KiCad project in `base/`
   - Generate BOM from `base/bom/ibom.html`
   - Order PCB from your preferred manufacturer

4. **Program the firmware:**
   ```bash
   cd esphome
   cp secrets.yaml.example secrets.yaml
   # Edit secrets.yaml with your credentials
   esphome run doorbell.yaml
   ```

## 📊 Pin Mapping

| Function | ESP32 GPIO | Connected To |
|----------|------------|--------------|
| I2C1 SDA | GPIO8 | BQ25628E (Charger) |
| I2C1 SCL | GPIO9 | BQ25628E (Charger) |
| I2C2 SDA | GPIO5 | MAX17260 (Fuel Gauge) |
| I2C2 SCL | GPIO6 | MAX17260 (Fuel Gauge) |
| SPI CLK | GPIO12 | RFM69 |
| SPI MOSI | GPIO11 | RFM69 |
| SPI MISO | GPIO13 | RFM69 |
| SPI CS | GPIO15 | RFM69 |
| RF Reset | GPIO14 | RFM69 Reset |
| RF Interrupt | GPIO7 | RFM69 DIO0 |
| Battery Alert | GPIO21 | MAX17260 Alert |
| Bell Input | GPIO1 | Doorbell Button |
| Bell Mute | GPIO3 | Bell Disable Output |
| Door Opener | GPIO2 | Buzzer/Opener Control |
| RF Power | GPIO10 | RFM69 Power Enable |

## 🔋 Power Consumption

| Mode | Current Draw | Notes |
|------|--------------|-------|
| Active (WiFi) | ~80mA | Normal operation |
| Idle (WiFi) | ~50mA | Connected, no activity |
| Deep Sleep | <1mA | Wake on interrupt/timer |
| Charging | 50mA - 1A | Configurable |

**Estimated battery life** (with 1000mAh battery):
- Always on: ~12-20 hours
- With deep sleep (5 min intervals): Several days to weeks

## 📱 Home Assistant Integration

Once configured, you'll have access to:

**Sensors:**
- Battery voltage, percentage, temperature
- Charge current and status
- Time to empty/full
- WiFi signal strength

**Controls:**
- Door opener button
- Bell mute switch
- Deep sleep management
- RF pairing mode

**Automations:**
- Doorbell press notifications
- Low battery alerts
- Automatic door opening via RF remote

## 🛠️ Development

### Custom Components

All custom ESPHome components are located in `esphome/components/`:

- **bq25628e:** Full I2C control of battery charger
- **max17260:** Fuel gauge with ModelGauge m5 algorithm
- **rfm69:** Complete RF transceiver with encryption and pairing

### Building from Source

```bash
# Install ESPHome
pip install esphome

# Compile firmware
cd esphome
esphome compile doorbell.yaml

# Upload OTA (after first flash)
esphome upload doorbell.yaml --device esphome-rf-doorbell.local
```

## 📖 Documentation

- [Getting Started Guide](esphome/GETTING_STARTED.md) - Setup and installation
- [Component Documentation](esphome/README.md) - Detailed component reference
- [KiCad Schematics](base/esphome-rf-doorbell.kicad_sch) - Hardware design
- [BOM](base/bom/ibom.html) - Interactive bill of materials

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for:
- Bug fixes
- Feature enhancements
- Documentation improvements
- Hardware design optimizations

## 📜 License

This project is open source. See LICENSE file for details.

## 🙏 Acknowledgments

- ESPHome community for excellent framework
- Home Assistant for smart home integration
- Texas Instruments for BQ25628E datasheet and reference design
- Maxim Integrated for MAX17260 documentation
- HopeRF for RFM69 modules

## ⚠️ Disclaimer

This project involves:
- Mains voltage wiring (for doorbell/buzzer)
- Li-Ion battery handling
- RF transmission (comply with local regulations)

**Use at your own risk.** Ensure proper safety measures and follow local electrical codes.
