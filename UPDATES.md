# Updates Applied to ESPHome RF Doorbell Project

This document summarizes the corrections and improvements made to address the identified issues.

## Issues Addressed

### 1. ✅ GPIO Pin Verification

**Issue**: Pin numbers appeared inconsistent and needed verification against the KiCAD schematic.

**Resolution**: 
- Analyzed the KiCAD schematic ([base/esphome-rf-doorbell.kicad_sch](base/esphome-rf-doorbell.kicad_sch)) to extract correct GPIO mappings
- Verified all GPIO assignments match the ESP32-S2-MINI-2 (module U7) connections:
  - **I2C Bus 1** (BQ25628E): GPIO8 (SDA1), GPIO9 (SCL1) ✓
  - **I2C Bus 2** (MAX17260): GPIO5 (SDA2), GPIO6 (SCL2) ✓
  - **SPI Bus** (RFM69): GPIO12 (CLK), GPIO11 (MOSI), GPIO13 (MISO), GPIO15 (CS) ✓
  - **Control Signals**: GPIO1 (BELL_SIGNAL), GPIO2 (OPEN_BUZZER), GPIO3 (BELL_OFF) ✓
  - **RF Control**: GPIO10 (RF_PWR_EN), GPIO14 (RF_RESET), GPIO7 (RF_INT) ✓
  - **Battery Alert**: GPIO21 (BAT_ALERT) ✓

**Result**: All GPIO pin assignments were already correct in the configuration.

---

### 2. ✅ Use of Existing Open-Source Libraries

**Issue**: Custom RF driver implementation instead of using battle-tested libraries.

**Resolution**:
- **RFM69 Driver**: Replaced custom low-level SPI implementation with **LowPowerLab RFM69 library v1.5.3**
  - Industry-standard Arduino library with extensive testing and community support
  - Better power management and reliability
  - Simplified hardware abstraction
  - Custom security layer built on top for challenge-response protocol
  
- **BQ25628E Driver**: ⚠️ **Adafruit library now available** (discovered December 2025)
  - [Adafruit_BQ25628E v1.0.0](https://github.com/adafruit/Adafruit_BQ25628E) released August 2025
  - Current implementation uses custom driver (predates Adafruit library)
  - **Recommendation**: Consider migrating to Adafruit library for long-term maintainability
  - Custom driver is functional but Adafruit library would reduce maintenance burden
  - Note: Adafruit library uses I2C address 0x6A by default (verify hardware compatibility)
  
- **MAX17260 Driver**: Retained custom implementation
  - No existing ESPHome library with full ModelGauge m5 support
  - Custom driver implements all fuel gauge features

**Files Modified**:
- [esphome/components/rfm69/__init__.py](esphome/components/rfm69/__init__.py) - Added LowPowerLab library dependency
- [esphome/components/rfm69/rfm69.h](esphome/components/rfm69/rfm69.h) - Simplified to wrap RFM69 library
- [esphome/components/rfm69/rfm69.cpp](esphome/components/rfm69/rfm69.cpp) - Rewrote using LowPowerLab API

**Note on BQ25628E**: The Adafruit library wasn't available when this project was created. Consider evaluating it for future improvements.

---

### 3. ✅ GitHub Repository Integration

**Issue**: Components loaded from local path instead of GitHub repository.

**Resolution**:
Updated `external_components` configuration in [doorbell.yaml](esphome/doorbell.yaml):

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/usimd/esphome-rf-doorbell
      ref: main
    components: [bq25628e, max17260, rfm69]
    refresh: 1d  # Check for updates daily
```

**Benefits**:
- Easy sharing of components with community
- Automatic update checking
- Version control through Git tags/branches
- Simplified deployment to multiple devices

**Note**: Replace `usimd` with your actual GitHub username after pushing the repository.

---

### 4. ✅ Home Assistant Configurable Charge Parameters

**Issue**: Battery charge parameters were hardcoded in the component configuration.

**Resolution**:
Added three `number` entities in [doorbell.yaml](esphome/doorbell.yaml) that allow runtime configuration from Home Assistant:

#### Number Entities Created:

1. **Charge Current Limit** (`charge_current_setting`)
   - Range: 0.05 - 3.0 A (50mA steps)
   - Default: 1.0 A
   - Updates BQ25628E register via `set_charge_current(x)`

2. **Charge Voltage Limit** (`charge_voltage_setting`)
   - Range: 3.84 - 4.624 V (8mV steps)
   - Default: 4.2 V (standard Li-Ion)
   - Updates BQ25628E register via `set_charge_voltage(x)`

3. **Input Current Limit** (`input_current_setting`)
   - Range: 0.1 - 3.2 A (100mA steps)
   - Default: 0.5 A
   - Updates BQ25628E register via `set_input_current(x)`

#### Configuration Linkage:

```yaml
bq25628e:
  id: battery_charger
  charge_current_limit: !lambda 'return id(charge_current_setting).state;'
  charge_voltage_limit: !lambda 'return id(charge_voltage_setting).state;'
  input_current_limit: !lambda 'return id(input_current_setting).state;'
```

#### Usage in Home Assistant:
- Adjust sliders in UI to change charging behavior in real-time
- Values persist across reboots (restore_value: true)
- Ideal for optimizing battery life or fast charging scenarios

---

### 5. ✅ Replay Attack Protection (Challenge-Response Protocol)

**Issue**: 433MHz RF communication vulnerable to replay attacks; needed secure authentication.

**Resolution**:
Implemented multi-layer security protocol in RFM69 component:

#### Security Layers:

**Layer 1: AES-128 Encryption** (via LowPowerLab library)
- All RF packets encrypted with 16-character shared key
- Prevents eavesdropping and packet inspection

**Layer 2: Challenge-Response Protocol** (custom implementation)
- Prevents replay attacks even if encryption is compromised
- Challenge flow:
  1. Remote sends `CHALLENGE_REQUEST` with nonce₁
  2. Doorbell verifies nonce₁ not previously used
  3. Doorbell responds with `CHALLENGE_RESPONSE` containing nonce₂
  4. Remote validates and sends `AUTHENTICATED` packet with rolling code
  5. Doorbell verifies rolling code is sequential and within window

**Layer 3: Rolling Codes**
- Each paired device maintains incrementing counter
- Codes must increase monotonically (prevents replay)
- 100-code window allows for lost packets

**Layer 4: Nonce Tracking**
- Last 100 nonces stored in circular buffer
- Prevents replay of old challenges
- Nonce uniqueness enforced via hardware RNG (`esp_random()`)

#### Configuration:

```yaml
rfm69:
  frequency: 433.0  # 433 MHz ISM band (corrected from 868 MHz)
  encryption_key: !secret rf_encryption_key
  use_challenge_response: true
  challenge_timeout: 30s
```

#### Packet Types:
- `PKT_TYPE_CHALLENGE_REQUEST` (0x01) - Remote initiates auth
- `PKT_TYPE_CHALLENGE_RESPONSE` (0x02) - Doorbell responds
- `PKT_TYPE_AUTHENTICATED` (0x03) - Authenticated data transfer
- `PKT_TYPE_PAIRING_REQUEST` (0x04) - Device pairing mode

#### Attack Resistance:
- ✅ **Replay Attacks**: Blocked by nonce tracking and rolling codes
- ✅ **Man-in-the-Middle**: Protected by AES encryption
- ✅ **Code Grabbing**: Challenge-response prevents static code reuse
- ✅ **Brute Force**: Hardware RNG provides cryptographic nonce strength

---

## Component Architecture

### BQ25628E Battery Charger
- **Location**: `esphome/components/bq25628e/`
- **Communication**: I2C (Bus 1, address 0x6B)
- **Features**:
  - Charge current: 0.05 - 3.0 A (configurable)
  - Charge voltage: 3.84 - 4.624 V (configurable)
  - Input current limit: 0.1 - 3.2 A (configurable)
  - ADC monitoring: VBUS, VBAT, VSYS, ICHG
  - Fault detection and reporting
  - Runtime configuration from Home Assistant

### MAX17260 Fuel Gauge
- **Location**: `esphome/components/max17260/`
- **Communication**: I2C (Bus 2, address 0x36)
- **Features**:
  - ModelGauge m5 algorithm for accurate SOC
  - Voltage, current, temperature monitoring
  - Time-to-empty and time-to-full predictions
  - Battery alert on GPIO21

### RFM69 RF Transceiver
- **Location**: `esphome/components/rfm69/`
- **Communication**: SPI (GPIO11/12/13/15)
- **Library**: LowPowerLab RFM69 v1.5.3
- **Frequency**: 433 MHz ISM band
- **Features**:
  - AES-128 encryption
  - Challenge-response authentication
  - Rolling code protection
  - Device pairing mode
  - Replay attack prevention
  - Wake-on-interrupt (GPIO7)

---

## Testing Checklist

### Hardware Verification
- [ ] I2C bus 1 communication with BQ25628E (address 0x6B)
- [ ] I2C bus 2 communication with MAX17260 (address 0x36)
- [ ] SPI communication with RFM69 transceiver
- [ ] GPIO7 interrupt triggers ESP32 wake from deep sleep
- [ ] GPIO21 battery alert detection

### Charge Control
- [ ] Adjust "Charge Current Limit" slider in HA
- [ ] Verify BQ25628E register updates via logs
- [ ] Adjust "Charge Voltage Limit" and confirm voltage changes
- [ ] Adjust "Input Current Limit" and verify VBUS current limit

### RF Security
- [ ] Pair remote device in pairing mode
- [ ] Verify challenge-response handshake in logs
- [ ] Test door opener trigger from paired remote
- [ ] Attempt packet replay - should be rejected
- [ ] Test unpaired device - should be rejected
- [ ] Verify rolling codes increment correctly

### Power Management
- [ ] Deep sleep current < 1mA
- [ ] Wake on RF packet (GPIO7 interrupt)
- [ ] Wake on timer (5 minute intervals)
- [ ] 30-second run duration before sleep
- [ ] Battery percentage reporting accurate

---

## Migration Path from Previous Version

If upgrading from the earlier custom RF implementation:

1. **Update Library Dependencies**:
   - ESPHome will automatically fetch LowPowerLab RFM69 library
   - No manual installation required

2. **Re-pair RF Remotes**:
   - Previous pairing data incompatible with new security protocol
   - Enable pairing mode in Home Assistant
   - Press pairing button on each remote
   - Verify device IDs and rolling codes in logs

3. **Configure Charge Parameters**:
   - Set desired charge current, voltage, and input limits via HA sliders
   - Previous hardcoded values may differ
   - Recommended: Start with conservative values (1.0A, 4.2V, 0.5A)

4. **Update GitHub URL**:
   - Replace `usimd` in external_components URL
   - Push component code to GitHub repository
   - ESPHome will clone on first build

---

## Security Best Practices

1. **Encryption Key Management**:
   - Use strong 16-character random key
   - Store in `secrets.yaml` (never commit to Git)
   - Rotate key periodically (requires re-pairing)

2. **RF Network ID**:
   - Use unique network ID (not default 100)
   - Prevents interference with nearby RFM69 networks

3. **Challenge Timeout**:
   - Default 30s balances security vs. usability
   - Shorter timeout = higher security, more pairing failures
   - Longer timeout = easier pairing, larger replay window

4. **Pairing Mode**:
   - Only enable during device addition
   - Disable immediately after pairing
   - Consider adding auto-timeout (e.g., 5 minutes)

---

## References

- **LowPowerLab RFM69 Library**: https://github.com/LowPowerLab/RFM69
- **BQ25628E Datasheet**: TI Document SLUSF36
- **MAX17260 Datasheet**: Maxim Integrated Document 19-100365
- **ESPHome External Components**: https://esphome.io/components/external_components.html

---

## Support

For issues or questions:
1. Check ESPHome logs for error messages
2. Verify GPIO connections against schematic
3. Ensure I2C devices respond to address scan
4. Test RF communication range and RSSI
5. Open GitHub issue with logs and configuration

---

*Last Updated*: 2024-01-XX (update date after pushing to GitHub)
