# ESPHome RF Doorbell

A smart doorbell system for Home Assistant using ESPHome, featuring battery power management, LoRa remote control, and deep sleep optimization.

## Features

- **ESP32-C6-MINI-1** microcontroller with WiFi/BLE/Zigbee connectivity
- **Battery Management:**
  - LTC4079 Li-Ion battery charger (250mA max, thermal regulation)
  - MAX17260 fuel gauge for accurate battery monitoring (ModelGauge m5)
  - Li-Ion battery support with USB-C or doorbell transformer charging
- **LoRa Communication:**
  - Wio-SX1262 LoRa module (868/915 MHz)
  - Long-range wireless communication
  - Remote pairing system (like garage door opener)
- **Ultra-Low Power:**
  - TPS63900 buck-boost converter (400nA quiescent)
  - Deep sleep with wake on LoRa packet or doorbell press
  - Battery life optimization
- **Home Assistant Integration:**
  - Real-time battery monitoring
  - Doorbell notifications
  - Remote door opener control
  - Bell mute functionality

## Project Structure

```
esphome-rf-doorbell/
├── base/                     # Main doorbell PCB (Rev3)
│   ├── esphome-rf-doorbell.kicad_pcb
│   ├── esphome-rf-doorbell.kicad_sch
│   └── bom/
├── remote/                   # Optional RF remote PCB
│   ├── remote.kicad_pcb
│   ├── remote.kicad_sch
│   └── firmware/             # PIC microcontroller firmware
├── lib/                      # KiCad component libraries
├── datasheets/               # IC datasheets and context
└── esphome/                  # ESPHome firmware
    ├── doorbell.yaml         # Main configuration
    ├── secrets.yaml.example  # Template for credentials
    ├── GETTING_STARTED.md    # Setup guide
    ├── README.md             # Component documentation
    └── components/           # Custom components
        ├── bq25628e/         # Battery charger driver (legacy - needs LTC4079)
        ├── max17260/         # Fuel gauge driver
        └── rfm69/            # RF transceiver driver (legacy - needs SX1262)
```

## Hardware (Rev3)

### Main PCB Components

| Ref | Part Number | Function |
|-----|-------------|----------|
| U1 | LTC4079 | Li-Ion Battery Charger |
| U3 | TPS63900 | Buck-Boost Converter (3.3V output) |
| U4 | MAX17260 | Battery Fuel Gauge |
| U5 | LTV-354T | Optocoupler (bell detection) |
| U6 | G3VM-31DR | PhotoMOS Relay (SPST-NO, door opener) |
| U7 | Wio-SX1262 | LoRa Module (868/915 MHz) |
| U8 | G3VM-63ER | PhotoMOS Relay (SPST-NC, bell mute) |
| U9 | ESP32-C6-MINI-1 | WiFi/BLE/Zigbee MCU |
| U10 | ISL23315 | Digital Potentiometer (I2C, charge current) |

### PCB Specifications

- **Layers:** 4-layer stackup
- **Size:** 62.3 x 25.9 mm
- **Design:** KiCad 8.x

### RF Remote (Optional)

- **PIC12F microcontroller:** Low-power remote controller
- **Wio-SX1262:** Matching LoRa transceiver
- **CR2032 battery:** Long-lasting coin cell power

## Quick Start

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

## Pin Mapping (ESP32-C6-MINI-1)

| Function | GPIO | Connected To |
|----------|------|--------------|
| I2C SDA | GPIO16 | LTC4079 (Charger) |
| I2C SCL | GPIO15 | LTC4079 (Charger) |
| I2C2 SDA | GPIO36 | MAX17260 (Fuel Gauge) |
| I2C2 SCL | GPIO35 | MAX17260 (Fuel Gauge) |
| SPI CLK | GPIO8 | Wio-SX1262 |
| SPI MOSI | GPIO10 | Wio-SX1262 |
| SPI MISO | GPIO9 | Wio-SX1262 |
| SPI CS | GPIO11 | Wio-SX1262 |
| LoRa Reset | GPIO7 | Wio-SX1262 Reset |
| LoRa DIO1 | GPIO6 | Wio-SX1262 DIO1 |
| LoRa Busy | GPIO5 | Wio-SX1262 BUSY |
| Battery Alert | GPIO21 | MAX17260 Alert |
| Bell Input | GPIO13 | Doorbell Button (via optocoupler) |
| Bell Mute | GPIO12 | G3VM-63ER (NC relay) |
| Door Opener | GPIO14 | G3VM-31DR (NO relay) |
| RF Power | GPIO33 | Wio-SX1262 LDO Enable |
| Charger CE | GPIO18 | LTC4079 CE (active low) |

## Power Consumption

| Mode | Current Draw | Notes |
|------|--------------|-------|
| Active (WiFi) | ~25mA | ESP32-C6 with WiFi active |
| Deep Sleep | <10uA | TPS63900 ultra-low quiescent |
| Charging | up to 250mA | LTC4079 programmable |

**Estimated battery life** (with 350mAh battery):
- Always on: ~14 hours
- With deep sleep (60s intervals): Weeks to months

## Home Assistant Integration

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
- LoRa pairing mode

**Automations:**
- Doorbell press notifications
- Low battery alerts
- Automatic door opening via LoRa remote

## Development

### Custom Components

Custom ESPHome components are located in `esphome/components/`:

- **bq25628e:** Legacy component - needs replacement with LTC4079 driver
- **max17260:** Fuel gauge with ModelGauge m5 algorithm (working)
- **rfm69:** Legacy component - needs replacement with SX1262 driver

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

## Documentation

- [Getting Started Guide](esphome/GETTING_STARTED.md) - Setup and installation
- [Component Documentation](esphome/README.md) - Detailed component reference
- [KiCad Schematics](base/esphome-rf-doorbell.kicad_sch) - Hardware design
- [Datasheet Context](datasheets/DATASHEET_CONTEXT.md) - IC inventory and references
- [BOM](base/bom/ibom.html) - Interactive bill of materials

## Hardware Revision History

| Rev | MCU | Charger | RF Module | Buck/Boost | Notes |
|-----|-----|---------|-----------|------------|-------|
| Rev1 | ESP32-S2 | BQ25628E | RFM69W | TPS629206 | Initial design |
| Rev2 | ESP32-S2 | BQ25628E | RFM69W | TPS629206 | Bug fixes |
| **Rev3** | **ESP32-C6-MINI-1** | **LTC4079** | **Wio-SX1262** | **TPS63900** | Current (WIP) |

## Known Issues (Rev3 WIP)

- [ ] ESPHome firmware still references BQ25628E (needs LTC4079 component)
- [ ] ESPHome firmware still references RFM69 (needs SX1262 component)
- [ ] Optocoupler may need active-high modification (see SCHEMATIC_MODIFICATIONS_ACTIVE_HIGH.md)
- [ ] Pin mappings in doorbell.yaml need verification against schematic

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for:
- Bug fixes
- Feature enhancements
- Documentation improvements
- Hardware design optimizations

## License

This project is open source. See LICENSE file for details.

## Acknowledgments

- ESPHome community for excellent framework
- Home Assistant for smart home integration
- Analog Devices for LTC4079 and MAX17260
- Texas Instruments for TPS63900
- Seeed Studio for Wio-SX1262 module
- Espressif for ESP32-C6

## Disclaimer

This project involves:
- Mains voltage wiring (for doorbell/buzzer)
- Li-Ion battery handling
- RF transmission (comply with local regulations)

**Use at your own risk.** Ensure proper safety measures and follow local electrical codes.
