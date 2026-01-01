#include "bq25628e.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

namespace esphome {
namespace bq25628e {

static const char *const TAG = "bq25628e";

void BQ25628EComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BQ25628E...");
  
  // Feed watchdog before long configuration sequence
  App.feed_wdt();
  
  if (!this->configure_charger_()) {
    ESP_LOGE(TAG, "Failed to configure BQ25628E");
    this->mark_failed();
    return;
  }
  
  // Feed watchdog after configuration
  App.feed_wdt();
  
  // Mark component as ready for public API calls
  this->is_ready_ = true;
  
  ESP_LOGCONFIG(TAG, "BQ25628E setup complete");
}

void BQ25628EComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BQ25628E:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with BQ25628E failed!");
  }
  
  // Read and publish device identification (REG 0x38)
  uint8_t part_info;
  if (this->get_part_information_reg_(part_info)) {
    uint8_t part_number = (part_info & PART_INFO_PN_MASK) >> PART_INFO_PN_SHIFT;
    uint8_t device_rev = (part_info & PART_INFO_DEV_REV_MASK) >> PART_INFO_DEV_REV_SHIFT;
    
    ESP_LOGCONFIG(TAG, "  Part Number: 0x%X (BQ2562%dE)", part_number, part_number + 4);
    ESP_LOGCONFIG(TAG, "  Device Revision: %d", device_rev);
    
    // Publish to text sensors if configured
    if (this->part_number_sensor_ != nullptr) {
      char pn_str[16];
      snprintf(pn_str, sizeof(pn_str), "BQ2562%dE", part_number + 4);
      this->part_number_sensor_->publish_state(pn_str);
    }
    
    if (this->device_revision_sensor_ != nullptr) {
      char rev_str[8];
      snprintf(rev_str, sizeof(rev_str), "Rev %d", device_rev);
      this->device_revision_sensor_->publish_state(rev_str);
    }
  }
  
  ESP_LOGCONFIG(TAG, "  Charge Current Limit: %.2f A", this->charge_current_limit_);
  ESP_LOGCONFIG(TAG, "  Charge Voltage Limit: %.2f V", this->charge_voltage_limit_);
  ESP_LOGCONFIG(TAG, "  Input Current Limit: %.2f A", this->input_current_limit_);
  ESP_LOGCONFIG(TAG, "  Input Voltage Limit: %.2f V", this->input_voltage_limit_);
  ESP_LOGCONFIG(TAG, "  Minimum System Voltage: %.2f V", this->minimum_system_voltage_);
  ESP_LOGCONFIG(TAG, "  Pre-charge Current: %.3f A", this->precharge_current_);
  ESP_LOGCONFIG(TAG, "  Termination Current: %.3f A", this->termination_current_);
  ESP_LOGCONFIG(TAG, "  Termination Enabled: %s", this->termination_enabled_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  VINDPM Battery Tracking: %s", this->vindpm_battery_tracking_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Thermal Regulation: %dC", this->thermal_regulation_threshold_);
  
  LOG_SENSOR("  ", "Bus Voltage", this->bus_voltage_sensor_);
  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_sensor_);
  LOG_SENSOR("  ", "Charge Current", this->charge_current_sensor_);
  LOG_SENSOR("  ", "System Voltage", this->system_voltage_sensor_);
  LOG_SENSOR("  ", "Input Current", this->input_current_sensor_);
  LOG_SENSOR("  ", "TS Temperature", this->ts_temperature_sensor_);
  LOG_SENSOR("  ", "Die Temperature", this->die_temperature_sensor_);
  LOG_TEXT_SENSOR("  ", "Part Number", this->part_number_sensor_);
  LOG_TEXT_SENSOR("  ", "Device Revision", this->device_revision_sensor_);
}

void BQ25628EComponent::update() {
  // Guard against calls before setup() completes
  if (!this->is_ready_) {
    return;
  }
  
  // Reset watchdog to prevent expiration (which disables charging)
  this->reset_watchdog();
  
  // Read ADC values
  if (!this->read_adc_values_()) {
    ESP_LOGW(TAG, "Failed to read ADC values");
    return;
  }
}

/**
 * Unified charger configuration using YAML values
 * 
 * Optimized for minimal I2C transactions at 1MHz:
 * - Uses atomic 16-bit writes where possible  
 * - Batches register writes for adjacent registers
 * - No unnecessary read-modify-write cycles for registers we fully control
 * 
 * Configuration from YAML:
 * - ICHG: charge_current_limit_
 * - VREG: charge_voltage_limit_
 * - IINDPM: input_current_limit_
 * - VINDPM: input_voltage_limit_
 * - VSYSMIN: minimum_system_voltage_
 * - IPRECHG: precharge_current_
 * - ITERM: termination_current_
 * - EN_TERM: termination_enabled_
 * - VINDPM_BAT_TRACK: vindpm_battery_tracking_
 * - VRECHG: recharge_threshold_
 * - WATCHDOG: watchdog_timeout_
 * - TREG: thermal_regulation_threshold_
 * - TS_IGNORE: !ts_monitoring_enabled_
 */
bool BQ25628EComponent::configure_charger_() {
  ESP_LOGCONFIG(TAG, "Configuring BQ25628E...");
  
  // ===========================================================================
  // 16-BIT REGISTER PAIRS - Using atomic setters for proper bit packing
  // ===========================================================================
  
  // REG 0x02-0x03: ICHG (Charge Current)
  if (!this->set_charge_current_reg_(this->charge_current_limit_)) {
    ESP_LOGE(TAG, "Failed to set ICHG");
    return false;
  }
  
  // REG 0x04-0x05: VREG (Charge Voltage)
  if (!this->set_charge_voltage_reg_(this->charge_voltage_limit_)) {
    ESP_LOGE(TAG, "Failed to set VREG");
    return false;
  }
  
  // REG 0x06-0x07: IINDPM (Input Current Limit)
  if (!this->set_input_current_limit_reg_(this->input_current_limit_)) {
    ESP_LOGE(TAG, "Failed to set IINDPM");
    return false;
  }
  
  // REG 0x08-0x09: VINDPM (Input Voltage Limit)
  // Uses set_input_voltage_limit_reg_() for correct encoding: code = voltage_mV / 40
  // (NO offset subtraction - datasheet shows 3800mV = 0x5F = 95, and 95 * 40 = 3800)
  if (!this->set_input_voltage_limit_reg_(this->input_voltage_limit_)) {
    ESP_LOGE(TAG, "Failed to set VINDPM");
    return false;
  }
  
  // REG 0x0E-0x0F: VSYSMIN (Minimum System Voltage)
  if (!this->set_minimum_system_voltage_reg_(this->minimum_system_voltage_)) {
    ESP_LOGE(TAG, "Failed to set VSYSMIN");
    return false;
  }
  
  // Feed watchdog between register groups
  App.feed_wdt();
  
  // REG 0x10: IPRECHG (Pre-charge Current)
  if (!this->set_precharge_current_reg_(this->precharge_current_)) {
    ESP_LOGE(TAG, "Failed to set IPRECHG");
    return false;
  }
  
  // REG 0x12: ITERM (Termination Current)
  if (!this->set_termination_current_reg_(this->termination_current_)) {
    ESP_LOGE(TAG, "Failed to set ITERM");
    return false;
  }
  
  // Feed watchdog again
  App.feed_wdt();
  
  // ===========================================================================
  // 8-BIT CONTROL REGISTERS - Direct writes (no RMW needed, we control all bits)
  // ===========================================================================
  
  // REG 0x14: CHG_CTRL - Charge termination and recharge settings
  {
    uint8_t chg_ctrl = 0;
    if (this->termination_enabled_) chg_ctrl |= CHG_CTRL_EN_TERM;
    if (this->vindpm_battery_tracking_) chg_ctrl |= CHG_CTRL_VINDPM_BAT_TRACK;
    if (this->recharge_threshold_ > 0.15f) chg_ctrl |= CHG_CTRL_VRECHG_MASK;  // 200mV vs 100mV
    if (!this->write_register_byte_(BQ25628E_REG_CHG_CTRL, chg_ctrl)) {
      ESP_LOGE(TAG, "Failed to write CHG_CTRL");
      return false;
    }
  }
  
  // REG 0x15: CHG_TMR_CTRL - Timer settings
  {
    uint8_t tmr_ctrl = CHG_TMR_CTRL_TMR2X_EN;  // Enable 2X timer during DPM/thermal
    if (!this->write_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, tmr_ctrl)) {
      ESP_LOGE(TAG, "Failed to write CHG_TMR_CTRL");
      return false;
    }
  }
  
  // REG 0x16: CHARGER_CTRL_0 - Enable charging, disable HIZ, set watchdog
  {
    uint8_t ctrl0 = CHARGER_CTRL_0_EN_CHG;  // EN_CHG=1, EN_HIZ=0
    ctrl0 |= ((this->watchdog_timeout_ & 0x03) << CHARGER_CTRL_0_WATCHDOG_SHIFT);
    if (!this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, ctrl0)) {
      ESP_LOGE(TAG, "Failed to write CHARGER_CTRL_0");
      return false;
    }
  }
  
  // REG 0x17: CHARGER_CTRL_1 - Thermal regulation, VBUS OVP
  {
    uint8_t ctrl1 = CHARGER_CTRL_1_CONV_STRN_STRONG | CHARGER_CTRL_1_VBUS_OVP_18V;
    if (this->thermal_regulation_threshold_ > 90) {
      ctrl1 |= CHARGER_CTRL_1_TREG;  // 120°C
    }
    if (!this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, ctrl1)) {
      ESP_LOGE(TAG, "Failed to write CHARGER_CTRL_1");
      return false;
    }
  }
  
  // REG 0x1A: NTC_CTRL_0 - TS monitoring
  {
    uint8_t ntc_ctrl = 0;
    if (!this->ts_monitoring_enabled_) {
      ntc_ctrl |= NTC_CTRL_0_TS_IGNORE;
      ESP_LOGW(TAG, "TS monitoring DISABLED");
    }
    if (!this->write_register_byte_(BQ25628E_REG_NTC_CTRL_0, ntc_ctrl)) {
      ESP_LOGE(TAG, "Failed to write NTC_CTRL_0");
      return false;
    }
  }
  
  // REG 0x26: ADC_CTRL - Enable continuous ADC with averaging
  // Bit 7: ADC_EN, Bit 6: ADC_RATE (continuous), Bits 5:4: ADC_SAMPLE (10-bit), Bit 3: ADC_AVG
  {
    uint8_t adc_ctrl = 0x80 | 0x40 | (0x02 << 4) | 0x08;  // 0xE8
    if (!this->write_register_byte_(BQ25628E_REG_ADC_CTRL, adc_ctrl)) {
      ESP_LOGE(TAG, "Failed to write ADC_CTRL");
      return false;
    }
  }
  
  // REG 0x27: ADC_DIS - Enable all ADC channels
  if (!this->write_register_byte_(BQ25628E_REG_ADC_DIS, 0x00)) {
    ESP_LOGE(TAG, "Failed to write ADC_DIS");
    return false;
  }
  
  // Feed watchdog before interrupt mask configuration
  App.feed_wdt();
  
  // ===========================================================================
  // INTERRUPT MASKS - Batch write (registers 0x23-0x25)
  // ===========================================================================
  if (!this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, 0x40) ||  // Mask ADC_DONE
      !this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_1, 0x00) ||  // Enable CHG/VBUS interrupts
      !this->write_register_byte_(BQ25628E_REG_FAULT_MASK_0, 0x00)) {    // Enable fault interrupts
    ESP_LOGE(TAG, "Failed to configure interrupt masks");
    return false;
  }
  
  ESP_LOGCONFIG(TAG, "BQ25628E configured: ICHG=%.0fmA, VREG=%.0fmV, IINDPM=%.0fmA, VINDPM=%.0fmV, TREG=%d°C",
                this->charge_current_limit_ * 1000, this->charge_voltage_limit_ * 1000,
                this->input_current_limit_ * 1000, this->input_voltage_limit_ * 1000,
                this->thermal_regulation_threshold_ > 90 ? 120 : 60);
  
  return true;
}

bool BQ25628EComponent::read_adc_values_() {
  // Read and publish VBUS (input voltage) - datasheet: 3.97mV per LSB, bits 14:2
  if (this->bus_voltage_sensor_ != nullptr) {
    uint16_t raw_vbus;
    if (this->read_register_word_(BQ25628E_REG_VBUS_ADC, raw_vbus)) {
      uint16_t vbus_adc = (raw_vbus >> 2) & 0x1FFF;  // Bits 14:2, 13-bit value
      float vbus = vbus_adc * VBUS_ADC_STEP;
      ESP_LOGD(TAG, "VBUS raw: 0x%04X, ADC: %d, value: %.2f V", raw_vbus, vbus_adc, vbus);
      this->bus_voltage_sensor_->publish_state(vbus);
    } else {
      ESP_LOGW(TAG, "Failed to read VBUS");
    }
  }
  
  // Read and publish VBAT (battery voltage) - datasheet: 1.99mV per LSB, bits 12:1
  if (this->battery_voltage_sensor_ != nullptr) {
    uint16_t raw_vbat;
    if (this->read_register_word_(BQ25628E_REG_VBAT_ADC, raw_vbat)) {
      uint16_t vbat_adc = (raw_vbat >> 1) & 0x0FFF;  // Bits 12:1, 12-bit value
      float vbat = vbat_adc * VBAT_ADC_STEP;
      ESP_LOGD(TAG, "VBAT raw: 0x%04X, ADC: %d, value: %.2f V", raw_vbat, vbat_adc, vbat);
      this->battery_voltage_sensor_->publish_state(vbat);
    } else {
      ESP_LOGW(TAG, "Failed to read VBAT");
    }
  }
  
  // Read and publish VSYS (system voltage) - datasheet: 1.99mV per LSB, bits 12:1
  if (this->system_voltage_sensor_ != nullptr) {
    uint16_t raw_vsys;
    if (this->read_register_word_(BQ25628E_REG_VSYS_ADC, raw_vsys)) {
      uint16_t vsys_adc = (raw_vsys >> 1) & 0x0FFF;  // Bits 12:1, 12-bit value
      float vsys = vsys_adc * VSYS_ADC_STEP;
      ESP_LOGD(TAG, "VSYS raw: 0x%04X, ADC: %d, value: %.2f V", raw_vsys, vsys_adc, vsys);
      this->system_voltage_sensor_->publish_state(vsys);
    } else {
      ESP_LOGW(TAG, "Failed to read VSYS");
    }
  }
  
  // Read and publish IBAT (charge current) - datasheet: 4mA per LSB, bits 15:2, signed
  if (this->charge_current_sensor_ != nullptr) {
    uint16_t raw_ibat;
    if (this->read_register_word_(BQ25628E_REG_IBAT_ADC, raw_ibat)) {
      // Bits 15:2, 14-bit value, then sign-extend to 16-bit signed
      uint16_t ibat_adc = (raw_ibat >> 2) & 0x3FFF;
      int16_t signed_ibat = (int16_t)(ibat_adc << 2) >> 2;  // Sign extend from bit 13
      float ibat = signed_ibat * IBAT_ADC_STEP;
      ESP_LOGD(TAG, "IBAT raw: 0x%04X, ADC: %d, signed: %d, value: %.3f A", raw_ibat, ibat_adc, signed_ibat, ibat);
      this->charge_current_sensor_->publish_state(ibat);
    } else {
      ESP_LOGW(TAG, "Failed to read IBAT");
    }
  }
  
  // Read and publish IBUS (input current) - datasheet: 2mA per LSB, bits 15:1, signed
  if (this->input_current_sensor_ != nullptr) {
    uint16_t raw_ibus;
    if (this->read_register_word_(BQ25628E_REG_IBUS_ADC, raw_ibus)) {
      // Bits 15:1, 15-bit value, then sign-extend to 16-bit signed
      uint16_t ibus_adc = (raw_ibus >> 1) & 0x7FFF;
      int16_t signed_ibus = (int16_t)(ibus_adc << 1) >> 1;  // Sign extend from bit 14
      float ibus = signed_ibus * IBUS_ADC_STEP;
      ESP_LOGD(TAG, "IBUS raw: 0x%04X, ADC: %d, signed: %d, value: %.3f A", raw_ibus, ibus_adc, signed_ibus, ibus);
      this->input_current_sensor_->publish_state(ibus);
    } else {
      ESP_LOGW(TAG, "Failed to read IBUS");
    }
  }
  
  // Read and publish TS temperature/voltage - datasheet: 0.9765625mV per LSB, bits 11:0
  if (this->ts_temperature_sensor_ != nullptr || this->ts_voltage_sensor_ != nullptr) {
    uint16_t raw_ts;
    if (this->read_register_word_(BQ25628E_REG_TS_ADC, raw_ts)) {
      uint16_t ts_adc = raw_ts & 0x0FFF;  // Bits 11:0, 12-bit value
      float ts_voltage = ts_adc * TS_ADC_STEP;
      ESP_LOGD(TAG, "TS raw: 0x%04X, ADC: %d, voltage: %.3f V", raw_ts, ts_adc, ts_voltage);
      if (this->ts_temperature_sensor_ != nullptr) {
        this->ts_temperature_sensor_->publish_state(ts_voltage);
      }
      if (this->ts_voltage_sensor_ != nullptr) {
        this->ts_voltage_sensor_->publish_state(ts_voltage);
      }
    } else {
      ESP_LOGW(TAG, "Failed to read TS");
    }
  }
  
  // Read and publish die temperature - datasheet: 0.5°C per LSB, 2's complement, bits 11:0
  // POR: 0°C = 0h, Range: -40°C (0xFB0) to 140°C (0x118)
  if (this->die_temperature_sensor_ != nullptr) {
    uint16_t raw_tdie;
    if (this->read_register_word_(BQ25628E_REG_TDIE_ADC, raw_tdie)) {
      uint16_t tdie_adc = raw_tdie & 0x0FFF;  // Bits 11:0, 12-bit value
      // Sign-extend 12-bit 2's complement to signed int16_t
      int16_t tdie_signed = (tdie_adc & 0x800) ? (tdie_adc | 0xF000) : tdie_adc;
      float tdie = tdie_signed * TDIE_ADC_STEP;
      ESP_LOGD(TAG, "TDIE raw: 0x%04X, ADC: %d, signed: %d, temperature: %.1f C", raw_tdie, tdie_adc, tdie_signed, tdie);
      this->die_temperature_sensor_->publish_state(tdie);
    } else {
      ESP_LOGW(TAG, "Failed to read TDIE");
    }
  }
  
  // Read charger status register 0x1D for DPM and regulation info
  uint16_t status_0_reg;
  if (this->read_register_word_(BQ25628E_REG_CHG_STATUS_0, status_0_reg)) {
    uint8_t status_0_byte = status_0_reg & 0xFF;
    bool adc_done_stat = status_0_byte & 0x40;  // Bit 6: ADC conversion complete (one-shot mode)
    bool treg_stat = status_0_byte & 0x20;      // Bit 5: Thermal regulation active
    bool vsys_stat = status_0_byte & 0x10;      // Bit 4: VSYSMIN regulation (BAT < VSYSMIN)
    bool iindpm_stat = status_0_byte & 0x08;    // Bit 3: IINDPM/ILIM regulation active
    bool vindpm_stat = status_0_byte & 0x04;    // Bit 2: VINDPM regulation active
    bool safety_tmr_stat = status_0_byte & 0x02;// Bit 1: Safety timer expired
    bool wd_stat = status_0_byte & 0x01;        // Bit 0: Watchdog expired
    ESP_LOGD(TAG, "Status0 [0x1D=0x%02X]: ADC_DONE:%d TREG:%d VSYS:%d IINDPM:%d VINDPM:%d SAFETY_TMR:%d WD:%d",
             status_0_byte, adc_done_stat, treg_stat, vsys_stat, iindpm_stat, vindpm_stat, safety_tmr_stat, wd_stat);
    ESP_LOGW(TAG, "⚠️ THERMAL_REG=%d WATCHDOG=%d SAFETY_TMR=%d - Check for faults!", treg_stat, wd_stat, safety_tmr_stat);
  }

  // Read charger status register (REG0x1E) for VBUS and CHG status
  uint16_t status_reg_word;
  if (this->read_register_word_(BQ25628E_REG_CHG_STATUS_1, status_reg_word)) {
    uint8_t status_byte = status_reg_word & 0xFF;
    uint8_t vbus_stat = status_byte & 0x07;  // Bits 2:0: VBUS status
    uint8_t chg_stat = (status_byte >> 3) & 0x03;  // Bits 4:3: Charge status
    const char* vbus_status[] = {"No Power", "Reserved", "Reserved", "Reserved", 
                                  "Unknown Adapter", "Reserved", "Reserved", "Reserved"};
    const char* chg_status[] = {"Not Charging", "CC/Trickle/Precharge", "CV Taper", "Top-off"};
    ESP_LOGD(TAG, "Status1 [0x1E]: VBUS=%s, CHG=%s", vbus_status[vbus_stat], chg_status[chg_stat]);
    
    // Detect poor source lockout and provide detailed diagnosis
    if (vbus_stat == 0) {  // VBUS_STAT = 000b = "No Power"
      ESP_LOGW(TAG, "⚠️ POOR SOURCE LOCKOUT! VBUS_STAT=000b (Not powered from VBUS)");
      
      // Read VBUS and VBAT ADC to diagnose the root cause
      uint16_t raw_vbus, raw_vbat;
      if (this->read_register_word_(BQ25628E_REG_VBUS_ADC, raw_vbus) &&
          this->read_register_word_(BQ25628E_REG_VBAT_ADC, raw_vbat)) {
        uint16_t vbus_adc = (raw_vbus >> 2) & 0x1FFF;
        uint16_t vbat_adc = (raw_vbat >> 1) & 0x0FFF;
        float vbus_v = vbus_adc * VBUS_ADC_STEP;
        float vbat_v = vbat_adc * VBAT_ADC_STEP;
        float delta = vbus_v - vbat_v;
        
        ESP_LOGW(TAG, "   VBUS_ADC=%.2fV, VBAT_ADC=%.2fV, Delta=%.0fmV", vbus_v, vbat_v, delta * 1000.0f);
        ESP_LOGW(TAG, "   VSLEEPZ threshold: 115-340mV (typ 220mV) for REGN to turn on");
        
        if (delta < 0.340f) {
          ESP_LOGW(TAG, "   ⚠️ SLEEP COMPARATOR: Delta %.0fmV may be < VSLEEPZ threshold!", delta * 1000.0f);
          ESP_LOGW(TAG, "   REGN LDO won't turn on if VBUS < VBAT + VSLEEPZ");
        }
        if (vbus_v < 3.5f) {
          ESP_LOGW(TAG, "   ⚠️ VBUS_UVLO: VBUS %.2fV < 3.5V (VVBUS_UVLOZ threshold)", vbus_v);
        }
        if (vbus_v < 3.6f) {
          ESP_LOGW(TAG, "   ⚠️ POOR_SOURCE: VBUS %.2fV < 3.6V (VPOORSRC threshold)", vbus_v);
        }
        ESP_LOGW(TAG, "   Possible causes: High resistance in power path, polyfuse tripped, bad solder");
        ESP_LOGW(TAG, "   Measure VBUS pin with multimeter while system running to compare with ADC");
      }
    }
  }

  // Read fault status register for debugging (REG0x1F)
  uint16_t fault_reg;
  if (this->read_register_word_(BQ25628E_REG_FAULT_STATUS_0, fault_reg)) {
    uint8_t fault_byte = fault_reg & 0xFF;  // LSB contains fault bits
    uint8_t ts_stat = fault_byte & 0x07;  // Bits 2:0: TS temperature zone
    if (fault_byte != 0) {
      ESP_LOGW(TAG, "Fault Status: 0x%02X - VBUS_FAULT:%d BAT_FAULT:%d SYS_FAULT:%d TSHUT:%d TS_ZONE:%d",
               fault_byte,
               (fault_byte & 0x80) ? 1 : 0,  // Bit 7: VBUS fault (OVP or sleep)
               (fault_byte & 0x40) ? 1 : 0,  // Bit 6: Battery fault (OCP or OVP)
               (fault_byte & 0x20) ? 1 : 0,  // Bit 5: VSYS fault (short or OVP)
               (fault_byte & 0x08) ? 1 : 0,  // Bit 3: Thermal shutdown
               ts_stat);  // Bits 2:0: Temperature zone (0=normal, 1=cold, 2=hot, etc)
      // Decode TS_STAT zone
      const char* ts_zone[] = {"NORMAL", "COLD/NO_BIAS", "HOT", "COOL", "WARM", "PRECOOL", "PREWARM", "BIAS_FAULT"};
      ESP_LOGW(TAG, "TS Zone: %s", ts_zone[ts_stat]);
    }
  }
  
  return true;
}

bool BQ25628EComponent::set_charging_enabled(bool enabled) {
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "set_charging_enabled called before setup complete, ignoring");
    return false;
  }
  
  uint8_t reg_val;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg_val)) {
    return false;
  }
  
  // Bit 5 of REG0x16 (Charger Control 0) controls charging enable
  // 1 = enable, 0 = disable
  if (enabled) {
    reg_val |= CHARGER_CTRL_0_EN_CHG;  // Set bit 5 to enable
  } else {
    reg_val &= ~CHARGER_CTRL_0_EN_CHG;  // Clear bit 5 to disable
  }
  
  bool success = this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg_val);
  if (success) {
    ESP_LOGI(TAG, "Charging %s", enabled ? "enabled" : "disabled");
  }
  return success;
}

bool BQ25628EComponent::set_charge_current(float current_amps) {
  // Guard against calls before setup() completes
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "set_charge_current called before setup complete, ignoring");
    return false;
  }
  
  if (current_amps < ICHG_MIN || current_amps > ICHG_MAX) {
    ESP_LOGE(TAG, "Charge current out of range: %.2f A", current_amps);
    return false;
  }
  
  // Use the proper _reg_ function with correct encoding
  bool success = this->set_charge_current_reg_(current_amps);
  if (success) {
    this->charge_current_limit_ = current_amps;
  }
  return success;
}

bool BQ25628EComponent::set_charge_voltage(float voltage_volts) {
  // Guard against calls before setup() completes
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "set_charge_voltage called before setup complete, ignoring");
    return false;
  }
  
  if (voltage_volts < VBAT_MIN || voltage_volts > VBAT_MAX) {
    ESP_LOGE(TAG, "Charge voltage out of range: %.2f V", voltage_volts);
    return false;
  }
  
  // Use the proper _reg_ function with correct encoding
  bool success = this->set_charge_voltage_reg_(voltage_volts);
  if (success) {
    this->charge_voltage_limit_ = voltage_volts;
  }
  return success;
}

bool BQ25628EComponent::set_input_current(float current_amps) {
  // Guard against calls before setup() completes
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "set_input_current called before setup complete, ignoring");
    return false;
  }
  
  if (current_amps < IINDPM_MIN || current_amps > IINDPM_MAX) {
    ESP_LOGE(TAG, "Input current out of range: %.2f A", current_amps);
    return false;
  }
  
  // Use the proper _reg_ function with correct encoding
  bool success = this->set_input_current_limit_reg_(current_amps);
  if (success) {
    this->input_current_limit_ = current_amps;
  }
  return success;
}

bool BQ25628EComponent::set_input_voltage(float voltage_volts) {
  // Guard against calls before setup() completes
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "set_input_voltage called before setup complete, ignoring");
    return false;
  }
  
  if (voltage_volts < VINDPM_MIN || voltage_volts > VINDPM_MAX) {
    ESP_LOGE(TAG, "Input voltage limit out of range: %.2f V", voltage_volts);
    return false;
  }
  
  // Use the proper _reg_ function with correct encoding
  bool success = this->set_input_voltage_limit_reg_(voltage_volts);
  if (success) {
    this->input_voltage_limit_ = voltage_volts;
  }
  return success;
}

bool BQ25628EComponent::set_hiz_mode(bool enabled) {
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "set_hiz_mode called before setup complete, ignoring");
    return false;
  }
  
  uint8_t reg_val;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg_val)) {
    return false;
  }
  
  if (enabled) {
    reg_val |= CHARGER_CTRL_0_EN_HIZ;
  } else {
    reg_val &= ~CHARGER_CTRL_0_EN_HIZ;
  }
  
  bool success = this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg_val);
  if (success) {
    ESP_LOGI(TAG, "HIZ mode %s", enabled ? "enabled" : "disabled");
  }
  return success;
}

bool BQ25628EComponent::reset_watchdog() {
  if (!this->is_ready_) {
    return false;
  }
  
  uint8_t reg_val;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg_val)) {
    return false;
  }
  
  reg_val |= CHARGER_CTRL_0_WD_RST;  // Set watchdog reset bit
  bool success = this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg_val);
  if (success) {
    ESP_LOGD(TAG, "Watchdog timer reset");
  }
  return success;
}

bool BQ25628EComponent::reset_registers() {
  if (!this->is_ready_) {
    ESP_LOGD(TAG, "reset_registers called before setup complete, ignoring");
    return false;
  }
  
  uint8_t reg_val;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg_val)) {
    return false;
  }
  
  reg_val |= CHARGER_CTRL_1_REG_RST;  // Set register reset bit
  bool success = this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg_val);
  if (success) {
    ESP_LOGI(TAG, "Registers reset to defaults");
    delay(10);  // Wait for reset to complete
    // Reconfigure with current settings
    this->configure_charger_();
  }
  return success;
}

uint8_t BQ25628EComponent::get_charge_status() {
  if (!this->is_ready_) {
    return 0xFF;  // Not ready, return error value
  }
  
  uint8_t status_reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_STATUS_1, status_reg)) {
    return 0xFF;  // Error value
  }
  
  uint8_t chg_stat = ((status_reg & 0xFF) & CHG_STATUS_1_CHG_STAT_MASK) >> 3;
  return chg_stat;
}

bool BQ25628EComponent::is_in_thermal_regulation() {
  if (!this->is_ready_) return false;
  
  uint16_t status_reg;
  if (!this->read_register_word_(BQ25628E_REG_CHG_STATUS_0, status_reg)) {
    return false;
  }
  return ((status_reg & 0xFF) & 0x20) != 0;  // Bit 5: TREG_STAT
}

bool BQ25628EComponent::is_in_vindpm_regulation() {
  if (!this->is_ready_) return false;
  
  uint16_t status_reg;
  if (!this->read_register_word_(BQ25628E_REG_CHG_STATUS_0, status_reg)) {
    return false;
  }
  return ((status_reg & 0xFF) & 0x04) != 0;  // Bit 2: VINDPM_STAT
}

bool BQ25628EComponent::is_in_iindpm_regulation() {
  if (!this->is_ready_) return false;
  
  uint16_t status_reg;
  if (!this->read_register_word_(BQ25628E_REG_CHG_STATUS_0, status_reg)) {
    return false;
  }
  return ((status_reg & 0xFF) & 0x08) != 0;  // Bit 3: IINDPM_STAT
}

bool BQ25628EComponent::has_fault() {
  if (!this->is_ready_) return false;
  
  uint16_t fault_reg;
  if (!this->read_register_word_(BQ25628E_REG_FAULT_STATUS_0, fault_reg)) {
    return false;
  }
  return (fault_reg & 0xFF) != 0;  // Any non-zero bit indicates a fault
}

// ============================================================================
// Register-specific setter methods with validation
// ============================================================================

bool BQ25628EComponent::set_charge_current_reg_(float current) {
  // REG 0x02/0x03 - ICHG: Charge Current Regulation Limit
  // Range: 40mA - 2000mA, Step: 40mA
  // ICHG[2:0] in REG0x02[7:5], ICHG[5:3] in REG0x03[2:0]
  // POR: 320mA (0x08)
  // NOTE: When Q4_FULLON=1, minimum is 80mA

  const float MIN = 0.040f;  // 40mA
  const float MAX = 2.000f;  // 2000mA
  const float STEP = 0.040f; // 40mA

  if (current < MIN || current > MAX) {
    ESP_LOGE(TAG, "Charge current %.3fA out of range [%.3fA - %.3fA]", current, MIN, MAX);
    return false;
  }

  // Encoding: code = current_mA / 40 (NO offset subtraction!)
  // Datasheet: 40mA = code 1, 2000mA = code 50 (0x32)
  uint8_t ichg_val = static_cast<uint8_t>((current * 1000.0f) / 40.0f + 0.5f);
  
  // Clamp to valid range (0x01-0x32)
  if (ichg_val < 0x01) ichg_val = 0x01;
  if (ichg_val > 0x32) ichg_val = 0x32;

  // Bit-pack: ICHG[2:0] to REG0x02[7:5]
  uint8_t reg02 = (ichg_val & 0x07) << 5;  // Bits [4:0] reserved, write 0
  
  // Read REG0x03 to preserve undefined bits [7:3]
  uint8_t reg03;
  if (!this->read_register_byte_(BQ25628E_REG_ICHG_CTRL + 1, reg03)) {
    ESP_LOGE(TAG, "Failed to read REG0x03 for ICHG configuration");
    return false;
  }
  
  // Bit-pack: ICHG[5:3] to REG0x03[2:0], preserve [7:3]
  reg03 = (reg03 & 0xF8) | ((ichg_val >> 3) & 0x07);
  
  float actual = ichg_val * STEP;  // code × 40mA, no offset
  ESP_LOGD(TAG, "Setting charge current: %.3fA → %.3fA (0x%02X), REG0x02=0x%02X, REG0x03=0x%02X", 
           current, actual, ichg_val, reg02, reg03);
  
  if (!this->write_register_byte_(BQ25628E_REG_ICHG_CTRL, reg02)) {
    ESP_LOGE(TAG, "Failed to write REG0x02 (ICHG)");
    return false;
  }
  
  if (!this->write_register_byte_(BQ25628E_REG_ICHG_CTRL + 1, reg03)) {
    ESP_LOGE(TAG, "Failed to write REG0x03 (ICHG)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charge current set to %.3fA", actual);
  return true;
}

bool BQ25628EComponent::set_charge_voltage_reg_(float voltage) {
  // REG 0x04/0x05 - VREG: Charge Voltage Regulation Limit
  // VREG[4:0] in REG0x04[7:3], VREG[8:5] in REG0x05[3:0]
  // POR: 4200mV (0x1A4), Range: 3500mV-4800mV (0x15E-0x1E0), Step: 10mV
  
  const float MIN = 3.500f;  // 3500mV
  const float MAX = 4.800f;  // 4800mV
  const float STEP = 0.010f; // 10mV
  const uint16_t MIN_CODE = 0x15E;  // 350
  const uint16_t MAX_CODE = 0x1E0;  // 480
  
  if (voltage < MIN || voltage > MAX) {
    ESP_LOGE(TAG, "Charge voltage %.3fV out of range [%.3fV - %.3fV]", voltage, MIN, MAX);
    return false;
  }
  
  // Encoding: code = voltage_mV / 10
  uint16_t vreg_val = static_cast<uint16_t>((voltage * 1000.0f) / 10.0f + 0.5f);
  
  // Clamp to valid range
  if (vreg_val < MIN_CODE) vreg_val = MIN_CODE;
  if (vreg_val > MAX_CODE) vreg_val = MAX_CODE;
  
  // Read both registers to preserve reserved bits
  uint8_t reg04, reg05;
  if (!this->read_register_byte_(BQ25628E_REG_VBAT_CTRL, reg04)) {
    ESP_LOGE(TAG, "Failed to read REG0x04 for VREG configuration");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_VBAT_CTRL + 1, reg05)) {
    ESP_LOGE(TAG, "Failed to read REG0x05 for VREG configuration");
    return false;
  }
  
  // Bit-pack: VREG[4:0] to REG0x04[7:3], preserve [2:0]
  reg04 = (reg04 & 0x07) | ((vreg_val & 0x1F) << 3);
  
  // Bit-pack: VREG[8:5] to REG0x05[3:0], preserve [7:4]
  reg05 = (reg05 & 0xF0) | ((vreg_val >> 5) & 0x0F);
  
  float actual = (vreg_val * 10.0f) / 1000.0f;
  ESP_LOGD(TAG, "Setting charge voltage: %.3fV → %.3fV (0x%03X), REG0x04=0x%02X, REG0x05=0x%02X", 
           voltage, actual, vreg_val, reg04, reg05);
  
  if (!this->write_register_byte_(BQ25628E_REG_VBAT_CTRL, reg04)) {
    ESP_LOGE(TAG, "Failed to write REG0x04 (VREG)");
    return false;
  }
  
  if (!this->write_register_byte_(BQ25628E_REG_VBAT_CTRL + 1, reg05)) {
    ESP_LOGE(TAG, "Failed to write REG0x05 (VREG)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charge voltage set to %.3fV", actual);
  return true;
}

bool BQ25628EComponent::set_input_current_limit_reg_(float current) {
  // REG 0x06/0x07 - IINDPM: Input Current Regulation Limit
  // IINDPM[3:0] in REG0x06[7:4], IINDPM[7:4] in REG0x07[3:0]
  // POR: 3200mA (0xA0), Range: 100mA-3200mA (0x05-0xA0), Step: 20mA
  // NOTE: Reset to POR on adapter removal
  
  const float MIN = 0.100f;  // 100mA
  const float MAX = 3.200f;  // 3200mA
  const float STEP = 0.020f; // 20mA
  const uint8_t MIN_CODE = 0x05;
  const uint8_t MAX_CODE = 0xA0;
  
  if (current < MIN || current > MAX) {
    ESP_LOGE(TAG, "Input current limit %.3fA out of range [%.3fA - %.3fA]", current, MIN, MAX);
    return false;
  }
  
  // Encoding: code = current_mA / 20
  uint8_t iindpm_val = static_cast<uint8_t>((current * 1000.0f) / 20.0f + 0.5f);
  
  // Clamp to valid range
  if (iindpm_val < MIN_CODE) iindpm_val = MIN_CODE;
  if (iindpm_val > MAX_CODE) iindpm_val = MAX_CODE;
  
  // Read both registers to preserve reserved bits
  uint8_t reg06, reg07;
  if (!this->read_register_byte_(BQ25628E_REG_IINDPM_CTRL, reg06)) {
    ESP_LOGE(TAG, "Failed to read REG0x06 for IINDPM configuration");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_IINDPM_CTRL + 1, reg07)) {
    ESP_LOGE(TAG, "Failed to read REG0x07 for IINDPM configuration");
    return false;
  }
  
  // Bit-pack: IINDPM[3:0] to REG0x06[7:4], preserve [3:0]
  reg06 = (reg06 & 0x0F) | ((iindpm_val & 0x0F) << 4);
  
  // Bit-pack: IINDPM[7:4] to REG0x07[3:0], preserve [7:4]
  reg07 = (reg07 & 0xF0) | ((iindpm_val >> 4) & 0x0F);
  
  float actual = (iindpm_val * 20.0f) / 1000.0f;
  ESP_LOGD(TAG, "Setting input current limit: %.3fA → %.3fA (0x%02X), REG0x06=0x%02X, REG0x07=0x%02X", 
           current, actual, iindpm_val, reg06, reg07);
  
  if (!this->write_register_byte_(BQ25628E_REG_IINDPM_CTRL, reg06)) {
    ESP_LOGE(TAG, "Failed to write REG0x06 (IINDPM)");
    return false;
  }
  
  if (!this->write_register_byte_(BQ25628E_REG_IINDPM_CTRL + 1, reg07)) {
    ESP_LOGE(TAG, "Failed to write REG0x07 (IINDPM)");
    return false;
  }
  
  ESP_LOGI(TAG, "Input current limit set to %.3fA", actual);
  return true;
}

bool BQ25628EComponent::set_input_voltage_limit_reg_(float voltage) {
  // REG 0x08/0x09 - VINDPM: Input Voltage Regulation Limit
  // VINDPM[2:0] in REG0x08[7:5], VINDPM[8:3] in REG0x09[5:0]
  // POR: 4600mV (0x73), Range: 3800mV-16800mV (0x5F-0x1A4), Step: 40mV
  
  const float MIN = 3.800f;  // 3800mV
  const float MAX = 16.800f; // 16800mV
  const float STEP = 0.040f; // 40mV
  const uint16_t MIN_CODE = 0x5F;  // 95
  const uint16_t MAX_CODE = 0x1A4; // 420
  
  if (voltage < MIN || voltage > MAX) {
    ESP_LOGE(TAG, "Input voltage limit %.3fV out of range [%.3fV - %.3fV]", voltage, MIN, MAX);
    return false;
  }
  
  // Encoding: code = voltage_mV / 40
  uint16_t vindpm_val = static_cast<uint16_t>((voltage * 1000.0f) / 40.0f + 0.5f);
  
  // Clamp to valid range
  if (vindpm_val < MIN_CODE) vindpm_val = MIN_CODE;
  if (vindpm_val > MAX_CODE) vindpm_val = MAX_CODE;
  
  // Read both registers to preserve reserved bits
  uint8_t reg08, reg09;
  if (!this->read_register_byte_(BQ25628E_REG_VINDPM_CTRL, reg08)) {
    ESP_LOGE(TAG, "Failed to read REG0x08 for VINDPM configuration");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_VINDPM_CTRL + 1, reg09)) {
    ESP_LOGE(TAG, "Failed to read REG0x09 for VINDPM configuration");
    return false;
  }
  
  // Bit-pack: VINDPM[2:0] to REG0x08[7:5], preserve [4:0]
  reg08 = (reg08 & 0x1F) | ((vindpm_val & 0x07) << 5);
  
  // Bit-pack: VINDPM[8:3] to REG0x09[5:0], preserve [7:6]
  reg09 = (reg09 & 0xC0) | ((vindpm_val >> 3) & 0x3F);
  
  float actual = (vindpm_val * 40.0f) / 1000.0f;
  ESP_LOGD(TAG, "Setting input voltage limit: %.3fV → %.3fV (0x%03X), REG0x08=0x%02X, REG0x09=0x%02X", 
           voltage, actual, vindpm_val, reg08, reg09);
  
  if (!this->write_register_byte_(BQ25628E_REG_VINDPM_CTRL, reg08)) {
    ESP_LOGE(TAG, "Failed to write REG0x08 (VINDPM)");
    return false;
  }
  
  if (!this->write_register_byte_(BQ25628E_REG_VINDPM_CTRL + 1, reg09)) {
    ESP_LOGE(TAG, "Failed to write REG0x09 (VINDPM)");
    return false;
  }
  
  ESP_LOGI(TAG, "Input voltage limit set to %.3fV", actual);
  return true;
}

bool BQ25628EComponent::set_minimum_system_voltage_reg_(float voltage) {
  // REG 0x0E/0x0F - VSYSMIN: Minimum System Voltage
  // VSYSMIN[1:0] in REG0x0E[7:6], VSYSMIN[5:2] in REG0x0F[3:0]
  // POR: 3520mV (0x2C), Range: 2560mV-3840mV (0x20-0x30), Step: 80mV
  
  const float MIN = 2.560f;  // 2560mV
  const float MAX = 3.840f;  // 3840mV
  const float STEP = 0.080f; // 80mV
  const uint8_t MIN_CODE = 0x20; // 32
  const uint8_t MAX_CODE = 0x30; // 48
  
  if (voltage < MIN || voltage > MAX) {
    ESP_LOGE(TAG, "Minimum system voltage %.3fV out of range [%.3fV - %.3fV]", voltage, MIN, MAX);
    return false;
  }
  
  // Encoding: code = voltage_mV / 80
  uint8_t vsysmin_val = static_cast<uint8_t>((voltage * 1000.0f) / 80.0f + 0.5f);
  
  // Clamp to valid range
  if (vsysmin_val < MIN_CODE) vsysmin_val = MIN_CODE;
  if (vsysmin_val > MAX_CODE) vsysmin_val = MAX_CODE;
  
  // Read both registers to preserve reserved bits
  uint8_t reg0e, reg0f;
  if (!this->read_register_byte_(BQ25628E_REG_VSYSMIN_CTRL, reg0e)) {
    ESP_LOGE(TAG, "Failed to read REG0x0E for VSYSMIN configuration");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_VSYSMIN_CTRL + 1, reg0f)) {
    ESP_LOGE(TAG, "Failed to read REG0x0F for VSYSMIN configuration");
    return false;
  }
  
  // Bit-pack: VSYSMIN[1:0] to REG0x0E[7:6], preserve [5:0]
  reg0e = (reg0e & 0x3F) | ((vsysmin_val & 0x03) << 6);
  
  // Bit-pack: VSYSMIN[5:2] to REG0x0F[3:0], preserve [7:4]
  reg0f = (reg0f & 0xF0) | ((vsysmin_val >> 2) & 0x0F);
  
  float actual = (vsysmin_val * 80.0f) / 1000.0f;
  ESP_LOGD(TAG, "Setting minimum system voltage: %.3fV → %.3fV (0x%02X), REG0x0E=0x%02X, REG0x0F=0x%02X", 
           voltage, actual, vsysmin_val, reg0e, reg0f);
  
  if (!this->write_register_byte_(BQ25628E_REG_VSYSMIN_CTRL, reg0e)) {
    ESP_LOGE(TAG, "Failed to write REG0x0E (VSYSMIN)");
    return false;
  }
  
  if (!this->write_register_byte_(BQ25628E_REG_VSYSMIN_CTRL + 1, reg0f)) {
    ESP_LOGE(TAG, "Failed to write REG0x0F (VSYSMIN)");
    return false;
  }
  
  ESP_LOGI(TAG, "Minimum system voltage set to %.3fV", actual);
  return true;
}

bool BQ25628EComponent::set_precharge_current_reg_(float current) {
  // REG 0x10/0x11 - IPRECHG: Pre-charge Current Limit
  // IPRECHG[4:0] in REG0x10[7:3] (only uses REG0x10)
  // POR: 30mA (0x03), Range: 10mA-310mA (0x01-0x1F), Step: 10mA
  // NOTE: When Q4_FULLON=1, minimum is 80mA
  
  const float MIN = 0.010f;  // 10mA
  const float MAX = 0.310f;  // 310mA
  const float STEP = 0.010f; // 10mA
  const uint8_t MIN_CODE = 0x01;
  const uint8_t MAX_CODE = 0x1F; // 31
  
  if (current < MIN || current > MAX) {
    ESP_LOGE(TAG, "Pre-charge current %.3fA out of range [%.3fA - %.3fA]", current, MIN, MAX);
    return false;
  }
  
  // Encoding: code = current_mA / 10
  uint8_t iprechg_val = static_cast<uint8_t>((current * 1000.0f) / 10.0f + 0.5f);
  
  // Clamp to valid range
  if (iprechg_val < MIN_CODE) iprechg_val = MIN_CODE;
  if (iprechg_val > MAX_CODE) iprechg_val = MAX_CODE;
  
  // Read register to preserve reserved bits
  uint8_t reg10;
  if (!this->read_register_byte_(BQ25628E_REG_IPRECHG_CTRL, reg10)) {
    ESP_LOGE(TAG, "Failed to read REG0x10 for IPRECHG configuration");
    return false;
  }
  
  // Bit-pack: IPRECHG[4:0] to REG0x10[7:3], preserve [2:0]
  reg10 = (reg10 & 0x07) | ((iprechg_val & 0x1F) << 3);
  
  float actual = (iprechg_val * 10.0f) / 1000.0f;
  ESP_LOGD(TAG, "Setting pre-charge current: %.3fA → %.3fA (0x%02X), REG0x10=0x%02X", 
           current, actual, iprechg_val, reg10);
  
  if (!this->write_register_byte_(BQ25628E_REG_IPRECHG_CTRL, reg10)) {
    ESP_LOGE(TAG, "Failed to write REG0x10 (IPRECHG)");
    return false;
  }
  
  ESP_LOGI(TAG, "Pre-charge current set to %.3fA", actual);
  return true;
}

bool BQ25628EComponent::set_termination_current_reg_(float current) {
  // REG 0x12/0x13 - ITERM: Termination Current Threshold
  // ITERM[5:0] in REG0x12[7:2] (only uses REG0x12)
  // POR: 20mA (0x04), Range: 5mA-310mA (0x01-0x3E), Step: 5mA
  // NOTE: When Q4_FULLON=1, minimum is 60mA
  
  const float MIN = 0.005f;  // 5mA
  const float MAX = 0.310f;  // 310mA
  const float STEP = 0.005f; // 5mA
  const uint8_t MIN_CODE = 0x01;
  const uint8_t MAX_CODE = 0x3E; // 62
  
  if (current < MIN || current > MAX) {
    ESP_LOGE(TAG, "Termination current %.3fA out of range [%.3fA - %.3fA]", current, MIN, MAX);
    return false;
  }
  
  // Encoding: code = current_mA / 5
  uint8_t iterm_val = static_cast<uint8_t>((current * 1000.0f) / 5.0f + 0.5f);
  
  // Clamp to valid range
  if (iterm_val < MIN_CODE) iterm_val = MIN_CODE;
  if (iterm_val > MAX_CODE) iterm_val = MAX_CODE;
  
  // Read register to preserve reserved bits
  uint8_t reg12;
  if (!this->read_register_byte_(BQ25628E_REG_ITERM_CTRL, reg12)) {
    ESP_LOGE(TAG, "Failed to read REG0x12 for ITERM configuration");
    return false;
  }
  
  // Bit-pack: ITERM[5:0] to REG0x12[7:2], preserve [1:0]
  reg12 = (reg12 & 0x03) | ((iterm_val & 0x3F) << 2);
  
  float actual = (iterm_val * 5.0f) / 1000.0f;
  ESP_LOGD(TAG, "Setting termination current: %.3fA → %.3fA (0x%02X), REG0x12=0x%02X", 
           current, actual, iterm_val, reg12);
  
  if (!this->write_register_byte_(BQ25628E_REG_ITERM_CTRL, reg12)) {
    ESP_LOGE(TAG, "Failed to write REG0x12 (ITERM)");
    return false;
  }
  
  ESP_LOGI(TAG, "Termination current set to %.3fA", actual);
  return true;
}

// ============================================================================
// Register-specific getters
// ============================================================================

bool BQ25628EComponent::get_charge_current_reg_(float &current) {
  // REG 0x02/0x03 - ICHG
  const float MIN = 0.040f;
  const float STEP = 0.040f;
  
  uint8_t reg02, reg03;
  if (!this->read_register_byte_(BQ25628E_REG_ICHG_CTRL, reg02)) {
    ESP_LOGE(TAG, "Failed to read REG0x02 (ICHG)");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_ICHG_CTRL + 1, reg03)) {
    ESP_LOGE(TAG, "Failed to read REG0x03 (ICHG)");
    return false;
  }
  
  // Extract: ICHG[2:0] from REG0x02[7:5], ICHG[5:3] from REG0x03[2:0]
  uint8_t ichg_val = ((reg02 >> 5) & 0x07) | (((reg03 & 0x07) << 3) & 0x38);
  current = MIN + (ichg_val * STEP);
  
  ESP_LOGD(TAG, "Read charge current: 0x%02X → %.3fA", ichg_val, current);
  return true;
}

bool BQ25628EComponent::get_charge_voltage_reg_(float &voltage) {
  // REG 0x04/0x05 - VREG
  // VREG[4:0] in REG0x04[7:3], VREG[8:5] in REG0x05[3:0]
  
  uint8_t reg04, reg05;
  if (!this->read_register_byte_(BQ25628E_REG_VBAT_CTRL, reg04)) {
    ESP_LOGE(TAG, "Failed to read REG0x04 (VREG)");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_VBAT_CTRL + 1, reg05)) {
    ESP_LOGE(TAG, "Failed to read REG0x05 (VREG)");
    return false;
  }
  
  // Extract: VREG[4:0] from REG0x04[7:3], VREG[8:5] from REG0x05[3:0]
  uint16_t vreg_val = ((reg04 >> 3) & 0x1F) | (((reg05 & 0x0F) << 5) & 0x1E0);
  voltage = (vreg_val * 10.0f) / 1000.0f;
  
  ESP_LOGD(TAG, "Read charge voltage: 0x%03X → %.3fV", vreg_val, voltage);
  return true;
}

bool BQ25628EComponent::get_input_current_limit_reg_(float &current) {
  // REG 0x06/0x07 - IINDPM
  // IINDPM[3:0] in REG0x06[7:4], IINDPM[7:4] in REG0x07[3:0]
  
  uint8_t reg06, reg07;
  if (!this->read_register_byte_(BQ25628E_REG_IINDPM_CTRL, reg06)) {
    ESP_LOGE(TAG, "Failed to read REG0x06 (IINDPM)");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_IINDPM_CTRL + 1, reg07)) {
    ESP_LOGE(TAG, "Failed to read REG0x07 (IINDPM)");
    return false;
  }
  
  // Extract: IINDPM[3:0] from REG0x06[7:4], IINDPM[7:4] from REG0x07[3:0]
  uint8_t iindpm_val = ((reg06 >> 4) & 0x0F) | (((reg07 & 0x0F) << 4) & 0xF0);
  current = (iindpm_val * 20.0f) / 1000.0f;
  
  ESP_LOGD(TAG, "Read input current limit: 0x%02X → %.3fA", iindpm_val, current);
  return true;
}

bool BQ25628EComponent::get_input_voltage_limit_reg_(float &voltage) {
  // REG 0x08/0x09 - VINDPM
  // VINDPM[2:0] in REG0x08[7:5], VINDPM[8:3] in REG0x09[5:0]
  
  uint8_t reg08, reg09;
  if (!this->read_register_byte_(BQ25628E_REG_VINDPM_CTRL, reg08)) {
    ESP_LOGE(TAG, "Failed to read REG0x08 (VINDPM)");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_VINDPM_CTRL + 1, reg09)) {
    ESP_LOGE(TAG, "Failed to read REG0x09 (VINDPM)");
    return false;
  }
  
  // Extract: VINDPM[2:0] from REG0x08[7:5], VINDPM[8:3] from REG0x09[5:0]
  uint16_t vindpm_val = ((reg08 >> 5) & 0x07) | (((reg09 & 0x3F) << 3) & 0x1F8);
  voltage = (vindpm_val * 40.0f) / 1000.0f;
  
  ESP_LOGD(TAG, "Read input voltage limit: 0x%03X → %.3fV", vindpm_val, voltage);
  return true;
}

bool BQ25628EComponent::get_minimum_system_voltage_reg_(float &voltage) {
  // REG 0x0E/0x0F - VSYSMIN
  // VSYSMIN[1:0] in REG0x0E[7:6], VSYSMIN[5:2] in REG0x0F[3:0]
  
  uint8_t reg0e, reg0f;
  if (!this->read_register_byte_(BQ25628E_REG_VSYSMIN_CTRL, reg0e)) {
    ESP_LOGE(TAG, "Failed to read REG0x0E (VSYSMIN)");
    return false;
  }
  if (!this->read_register_byte_(BQ25628E_REG_VSYSMIN_CTRL + 1, reg0f)) {
    ESP_LOGE(TAG, "Failed to read REG0x0F (VSYSMIN)");
    return false;
  }
  
  // Extract: VSYSMIN[1:0] from REG0x0E[7:6], VSYSMIN[5:2] from REG0x0F[3:0]
  uint8_t vsysmin_val = ((reg0e >> 6) & 0x03) | (((reg0f & 0x0F) << 2) & 0x3C);
  voltage = (vsysmin_val * 80.0f) / 1000.0f;
  
  ESP_LOGD(TAG, "Read minimum system voltage: 0x%02X → %.3fV", vsysmin_val, voltage);
  return true;
}

bool BQ25628EComponent::get_precharge_current_reg_(float &current) {
  // REG 0x10/0x11 - IPRECHG
  // IPRECHG[4:0] in REG0x10[7:3]
  
  uint8_t reg10;
  if (!this->read_register_byte_(BQ25628E_REG_IPRECHG_CTRL, reg10)) {
    ESP_LOGE(TAG, "Failed to read REG0x10 (IPRECHG)");
    return false;
  }
  
  // Extract: IPRECHG[4:0] from REG0x10[7:3]
  uint8_t iprechg_val = (reg10 >> 3) & 0x1F;
  current = (iprechg_val * 10.0f) / 1000.0f;
  
  ESP_LOGD(TAG, "Read pre-charge current: 0x%02X → %.3fA", iprechg_val, current);
  return true;
}

bool BQ25628EComponent::get_termination_current_reg_(float &current) {
  // REG 0x12/0x13 - ITERM
  // ITERM[5:0] in REG0x12[7:2]
  
  uint8_t reg12;
  if (!this->read_register_byte_(BQ25628E_REG_ITERM_CTRL, reg12)) {
    ESP_LOGE(TAG, "Failed to read REG0x12 (ITERM)");
    return false;
  }
  
  // Extract: ITERM[5:0] from REG0x12[7:2]
  uint8_t iterm_val = (reg12 >> 2) & 0x3F;
  current = (iterm_val * 5.0f) / 1000.0f;
  
  ESP_LOGD(TAG, "Read termination current: 0x%02X → %.3fA", iterm_val, current);
  return true;
}

// ============================================================================
// REG 0x14 CHG_CTRL - Charge Control Register
// ============================================================================

bool BQ25628EComponent::set_charge_control_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charge control register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHG_CTRL, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x14 (CHG_CTRL)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charge control register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charge_control_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x14 (CHG_CTRL)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charge control register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_q1_fullon_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHG_CTRL_Q1_FULLON;
  } else {
    reg &= ~CHG_CTRL_Q1_FULLON;
  }
  
  ESP_LOGD(TAG, "Setting Q1_FULLON (RBFET low resistance): %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg);
}

bool BQ25628EComponent::set_q4_fullon_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHG_CTRL_Q4_FULLON;
  } else {
    reg &= ~CHG_CTRL_Q4_FULLON;
  }
  
  ESP_LOGD(TAG, "Setting Q4_FULLON (BATFET low resistance): %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg);
}

bool BQ25628EComponent::set_trickle_current_(bool high) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  
  if (high) {
    reg |= CHG_CTRL_ITRICKLE;
  } else {
    reg &= ~CHG_CTRL_ITRICKLE;
  }
  
  ESP_LOGD(TAG, "Setting trickle current: %s", high ? "40mA" : "10mA");
  return this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg);
}

bool BQ25628EComponent::set_topoff_timer_(TopOffTimer timer) {
  if (timer > TOPOFF_52MIN) {
    ESP_LOGE(TAG, "Invalid top-off timer value: %d", timer);
    return false;
  }
  
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  
  reg = (reg & ~CHG_CTRL_TOPOFF_TMR_MASK) | ((timer << CHG_CTRL_TOPOFF_TMR_SHIFT) & CHG_CTRL_TOPOFF_TMR_MASK);
  
  const char* timer_str[] = {"Disabled", "17min", "35min", "52min"};
  ESP_LOGD(TAG, "Setting top-off timer: %s", timer_str[timer]);
  return this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg);
}

bool BQ25628EComponent::set_termination_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHG_CTRL_EN_TERM;
  } else {
    reg &= ~CHG_CTRL_EN_TERM;
  }
  
  ESP_LOGD(TAG, "Setting termination: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg);
}

bool BQ25628EComponent::set_vindpm_bat_track_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHG_CTRL_VINDPM_BAT_TRACK;
  } else {
    reg &= ~CHG_CTRL_VINDPM_BAT_TRACK;
  }
  
  ESP_LOGD(TAG, "Setting VINDPM battery tracking: %s", enable ? "VBAT+400mV" : "register value");
  return this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg);
}

bool BQ25628EComponent::set_recharge_threshold_(bool high) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  
  if (high) {
    reg |= CHG_CTRL_VRECHG;
  } else {
    reg &= ~CHG_CTRL_VRECHG;
  }
  
  ESP_LOGD(TAG, "Setting recharge threshold: %s below VREG", high ? "200mV" : "100mV");
  return this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg);
}

bool BQ25628EComponent::get_q1_fullon_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  enabled = (reg & CHG_CTRL_Q1_FULLON) != 0;
  return true;
}

bool BQ25628EComponent::get_q4_fullon_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  enabled = (reg & CHG_CTRL_Q4_FULLON) != 0;
  return true;
}

bool BQ25628EComponent::get_trickle_current_(bool &high) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  high = (reg & CHG_CTRL_ITRICKLE) != 0;
  return true;
}

bool BQ25628EComponent::get_topoff_timer_(TopOffTimer &timer) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  timer = static_cast<TopOffTimer>((reg & CHG_CTRL_TOPOFF_TMR_MASK) >> CHG_CTRL_TOPOFF_TMR_SHIFT);
  return true;
}

bool BQ25628EComponent::get_termination_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  enabled = (reg & CHG_CTRL_EN_TERM) != 0;
  return true;
}

bool BQ25628EComponent::get_vindpm_bat_track_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  enabled = (reg & CHG_CTRL_VINDPM_BAT_TRACK) != 0;
  return true;
}

bool BQ25628EComponent::get_recharge_threshold_(bool &high) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg)) {
    return false;
  }
  high = (reg & CHG_CTRL_VRECHG) != 0;
  return true;
}

// ============================================================================
// REG 0x15 CHG_TMR_CTRL - Charge Timer Control Register
// ============================================================================

bool BQ25628EComponent::set_charge_timer_control_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charge timer control register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x15 (CHG_TMR_CTRL)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charge timer control register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charge_timer_control_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x15 (CHG_TMR_CTRL)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charge timer control register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_stat_pin_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  
  // Note: DIS_STAT bit is inverted logic (1=disable, 0=enable)
  if (enable) {
    reg &= ~CHG_TMR_CTRL_DIS_STAT;  // Clear bit to enable
  } else {
    reg |= CHG_TMR_CTRL_DIS_STAT;   // Set bit to disable
  }
  
  ESP_LOGD(TAG, "Setting STAT pin: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg);
}

bool BQ25628EComponent::set_timer_2x_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHG_TMR_CTRL_TMR2X_EN;
  } else {
    reg &= ~CHG_TMR_CTRL_TMR2X_EN;
  }
  
  ESP_LOGD(TAG, "Setting 2X timer during DPM/thermal regulation: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg);
}

bool BQ25628EComponent::set_safety_timers_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHG_TMR_CTRL_EN_SAFETY_TMRS;
  } else {
    reg &= ~CHG_TMR_CTRL_EN_SAFETY_TMRS;
  }
  
  ESP_LOGD(TAG, "Setting safety timers: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg);
}

bool BQ25628EComponent::set_precharge_timer_(bool long_timer) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  
  if (long_timer) {
    reg |= CHG_TMR_CTRL_PRECHG_TMR;
  } else {
    reg &= ~CHG_TMR_CTRL_PRECHG_TMR;
  }
  
  ESP_LOGD(TAG, "Setting precharge timer: %s", long_timer ? "0.62hrs" : "2.5hrs");
  return this->write_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg);
}

bool BQ25628EComponent::set_fast_charge_timer_(bool long_timer) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  
  if (long_timer) {
    reg |= CHG_TMR_CTRL_CHG_TMR;
  } else {
    reg &= ~CHG_TMR_CTRL_CHG_TMR;
  }
  
  ESP_LOGD(TAG, "Setting fast charge timer: %s", long_timer ? "28hrs" : "14.5hrs");
  return this->write_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg);
}

bool BQ25628EComponent::get_stat_pin_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  // Inverted logic: bit clear = enabled
  enabled = (reg & CHG_TMR_CTRL_DIS_STAT) == 0;
  return true;
}

bool BQ25628EComponent::get_timer_2x_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  enabled = (reg & CHG_TMR_CTRL_TMR2X_EN) != 0;
  return true;
}

bool BQ25628EComponent::get_safety_timers_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  enabled = (reg & CHG_TMR_CTRL_EN_SAFETY_TMRS) != 0;
  return true;
}

bool BQ25628EComponent::get_precharge_timer_(bool &long_timer) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  long_timer = (reg & CHG_TMR_CTRL_PRECHG_TMR) != 0;
  return true;
}

bool BQ25628EComponent::get_fast_charge_timer_(bool &long_timer) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_TMR_CTRL, reg)) {
    return false;
  }
  long_timer = (reg & CHG_TMR_CTRL_CHG_TMR) != 0;
  return true;
}

// ============================================================================
// REG 0x16 CHARGER_CTRL_0 - Charger Control 0 Register
// ============================================================================

bool BQ25628EComponent::set_charger_ctrl_0_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charger control 0 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x16 (CHARGER_CTRL_0)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charger control 0 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charger_ctrl_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x16 (CHARGER_CTRL_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger control 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_auto_ibatdis_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHARGER_CTRL_0_EN_AUTO_IBATDIS;
  } else {
    reg &= ~CHARGER_CTRL_0_EN_AUTO_IBATDIS;
  }
  
  ESP_LOGD(TAG, "Setting auto battery discharge during OVP: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg);
}

bool BQ25628EComponent::set_force_ibatdis_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHARGER_CTRL_0_FORCE_IBATDIS;
  } else {
    reg &= ~CHARGER_CTRL_0_FORCE_IBATDIS;
  }
  
  ESP_LOGD(TAG, "Setting force battery discharge (~30mA): %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg);
}

bool BQ25628EComponent::set_charging_enabled_ctrl_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHARGER_CTRL_0_EN_CHG;
  } else {
    reg &= ~CHARGER_CTRL_0_EN_CHG;
  }
  
  ESP_LOGD(TAG, "Setting charge enable: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg);
}

bool BQ25628EComponent::set_hiz_mode_ctrl_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHARGER_CTRL_0_EN_HIZ;
  } else {
    reg &= ~CHARGER_CTRL_0_EN_HIZ;
  }
  
  ESP_LOGD(TAG, "Setting HIZ mode: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg);
}

bool BQ25628EComponent::set_force_pmid_discharge_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHARGER_CTRL_0_FORCE_PMID_DIS;
  } else {
    reg &= ~CHARGER_CTRL_0_FORCE_PMID_DIS;
  }
  
  ESP_LOGD(TAG, "Setting force PMID discharge (~30mA): %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg);
}

bool BQ25628EComponent::reset_watchdog_ctrl_() {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  
  reg |= CHARGER_CTRL_0_WD_RST;  // Set WD_RST bit (self-clearing)
  
  ESP_LOGD(TAG, "Resetting watchdog timer");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg);
}

bool BQ25628EComponent::set_watchdog_timer_(WatchdogTimer timer) {
  if (timer > WATCHDOG_200S) {
    ESP_LOGE(TAG, "Invalid watchdog timer value: %d", timer);
    return false;
  }
  
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  
  reg = (reg & ~CHARGER_CTRL_0_WATCHDOG_MASK) | ((timer << CHARGER_CTRL_0_WATCHDOG_SHIFT) & CHARGER_CTRL_0_WATCHDOG_MASK);
  
  const char* timer_str[] = {"Disabled", "50s", "100s", "200s"};
  ESP_LOGD(TAG, "Setting watchdog timer: %s", timer_str[timer]);
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg);
}

bool BQ25628EComponent::get_auto_ibatdis_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  enabled = (reg & CHARGER_CTRL_0_EN_AUTO_IBATDIS) != 0;
  return true;
}

bool BQ25628EComponent::get_force_ibatdis_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  enabled = (reg & CHARGER_CTRL_0_FORCE_IBATDIS) != 0;
  return true;
}

bool BQ25628EComponent::get_charging_enabled_ctrl_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  enabled = (reg & CHARGER_CTRL_0_EN_CHG) != 0;
  return true;
}

bool BQ25628EComponent::get_hiz_mode_ctrl_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  enabled = (reg & CHARGER_CTRL_0_EN_HIZ) != 0;
  return true;
}

bool BQ25628EComponent::get_force_pmid_discharge_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  enabled = (reg & CHARGER_CTRL_0_FORCE_PMID_DIS) != 0;
  return true;
}

bool BQ25628EComponent::get_watchdog_timer_(WatchdogTimer &timer) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_0, reg)) {
    return false;
  }
  timer = static_cast<WatchdogTimer>((reg & CHARGER_CTRL_0_WATCHDOG_MASK) >> CHARGER_CTRL_0_WATCHDOG_SHIFT);
  return true;
}

// ============================================================================
// REG 0x17 CHARGER_CTRL_1 - Charger Control 1 Register
// ============================================================================

bool BQ25628EComponent::set_charger_ctrl_1_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charger control 1 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x17 (CHARGER_CTRL_1)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charger control 1 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charger_ctrl_1_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x17 (CHARGER_CTRL_1)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger control 1 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::reset_registers_ctrl_() {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  
  reg |= CHARGER_CTRL_1_REG_RST;  // Set REG_RST bit (self-clearing)
  
  ESP_LOGI(TAG, "Resetting all registers to default values");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg);
}

bool BQ25628EComponent::set_thermal_threshold_(ThermalThreshold threshold) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  
  if (threshold == THERMAL_120C) {
    reg |= CHARGER_CTRL_1_TREG;
  } else {
    reg &= ~CHARGER_CTRL_1_TREG;
  }
  
  ESP_LOGD(TAG, "Setting thermal regulation threshold: %s", threshold == THERMAL_120C ? "120C" : "60C");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg);
}

bool BQ25628EComponent::set_switching_frequency_(SwitchingFrequency freq) {
  if (freq == FREQ_RESERVED) {
    ESP_LOGE(TAG, "FREQ_RESERVED is not a valid switching frequency");
    return false;
  }
  
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  
  reg = (reg & ~CHARGER_CTRL_1_CONV_FREQ_MASK) | ((freq << CHARGER_CTRL_1_CONV_FREQ_SHIFT) & CHARGER_CTRL_1_CONV_FREQ_MASK);
  
  const char* freq_str[] = {"1.5MHz (nominal)", "1.35MHz (-10%)", "1.65MHz (+10%)", "RESERVED"};
  ESP_LOGD(TAG, "Setting switching frequency: %s", freq_str[freq]);
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg);
}

bool BQ25628EComponent::set_drive_strength_(DriveStrength strength) {
  if (strength == DRIVE_RESERVED) {
    ESP_LOGE(TAG, "DRIVE_RESERVED is not a valid drive strength");
    return false;
  }
  
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  
  reg = (reg & ~CHARGER_CTRL_1_CONV_STRN_MASK) | ((strength << CHARGER_CTRL_1_CONV_STRN_SHIFT) & CHARGER_CTRL_1_CONV_STRN_MASK);
  
  const char* strength_str[] = {"weak", "normal", "RESERVED", "strong"};
  ESP_LOGD(TAG, "Setting drive strength: %s", strength_str[strength]);
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg);
}

bool BQ25628EComponent::set_vbus_ovp_threshold_(VbusOvpThreshold threshold) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  
  if (threshold == VBUS_OVP_18_5V) {
    reg |= CHARGER_CTRL_1_VBUS_OVP;
  } else {
    reg &= ~CHARGER_CTRL_1_VBUS_OVP;
  }
  
  ESP_LOGD(TAG, "Setting VBUS OVP threshold: %s", threshold == VBUS_OVP_18_5V ? "18.5V" : "6.3V");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg);
}

bool BQ25628EComponent::get_thermal_threshold_(ThermalThreshold &threshold) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  threshold = (reg & CHARGER_CTRL_1_TREG) ? THERMAL_120C : THERMAL_60C;
  return true;
}

bool BQ25628EComponent::get_switching_frequency_(SwitchingFrequency &freq) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  freq = static_cast<SwitchingFrequency>((reg & CHARGER_CTRL_1_CONV_FREQ_MASK) >> CHARGER_CTRL_1_CONV_FREQ_SHIFT);
  return true;
}

bool BQ25628EComponent::get_drive_strength_(DriveStrength &strength) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  strength = static_cast<DriveStrength>((reg & CHARGER_CTRL_1_CONV_STRN_MASK) >> CHARGER_CTRL_1_CONV_STRN_SHIFT);
  return true;
}

bool BQ25628EComponent::get_vbus_ovp_threshold_(VbusOvpThreshold &threshold) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_1, reg)) {
    return false;
  }
  threshold = (reg & CHARGER_CTRL_1_VBUS_OVP) ? VBUS_OVP_18_5V : VBUS_OVP_6_3V;
  return true;
}

// ============================================================================
// REG 0x18 CHARGER_CTRL_2 - Charger Control 2 Register
// ============================================================================

bool BQ25628EComponent::set_charger_ctrl_2_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charger control 2 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x18 (CHARGER_CTRL_2)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charger control 2 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charger_ctrl_2_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x18 (CHARGER_CTRL_2)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger control 2 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_pfm_forward_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= CHARGER_CTRL_2_PFM_FWD_DIS;
  } else {
    reg &= ~CHARGER_CTRL_2_PFM_FWD_DIS;
  }
  
  ESP_LOGD(TAG, "Setting PFM in forward buck mode: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg);
}

bool BQ25628EComponent::set_batfet_ctrl_wvbus_(bool allow_without_vbus) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  
  if (allow_without_vbus) {
    reg |= CHARGER_CTRL_2_BATFET_CTRL_WVBUS;
  } else {
    reg &= ~CHARGER_CTRL_2_BATFET_CTRL_WVBUS;
  }
  
  ESP_LOGD(TAG, "Setting BATFET control: %s", 
           allow_without_vbus ? "allow without VBUS check" : "only if VBUS < UVLO");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg);
}

bool BQ25628EComponent::set_batfet_delay_(BatfetDelay delay) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  
  if (delay == BATFET_DELAY_12_5S) {
    reg |= CHARGER_CTRL_2_BATFET_DLY;
  } else {
    reg &= ~CHARGER_CTRL_2_BATFET_DLY;
  }
  
  ESP_LOGD(TAG, "Setting BATFET delay: %s", delay == BATFET_DELAY_12_5S ? "12.5s" : "25ms");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg);
}

bool BQ25628EComponent::set_batfet_control_(BatfetControl mode) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  
  reg = (reg & ~CHARGER_CTRL_2_BATFET_CTRL_MASK) | ((mode << CHARGER_CTRL_2_BATFET_CTRL_SHIFT) & CHARGER_CTRL_2_BATFET_CTRL_MASK);
  
  const char* mode_str[] = {"Normal", "Shutdown", "Ship Mode", "System Reset"};
  ESP_LOGD(TAG, "Setting BATFET control mode: %s", mode_str[mode]);
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg);
}

bool BQ25628EComponent::get_pfm_forward_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  disabled = (reg & CHARGER_CTRL_2_PFM_FWD_DIS) != 0;
  return true;
}

bool BQ25628EComponent::get_batfet_ctrl_wvbus_(bool &allow_without_vbus) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  allow_without_vbus = (reg & CHARGER_CTRL_2_BATFET_CTRL_WVBUS) != 0;
  return true;
}

bool BQ25628EComponent::get_batfet_delay_(BatfetDelay &delay) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  delay = (reg & CHARGER_CTRL_2_BATFET_DLY) ? BATFET_DELAY_12_5S : BATFET_DELAY_25MS;
  return true;
}

bool BQ25628EComponent::get_batfet_control_(BatfetControl &mode) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_2, reg)) {
    return false;
  }
  mode = static_cast<BatfetControl>((reg & CHARGER_CTRL_2_BATFET_CTRL_MASK) >> CHARGER_CTRL_2_BATFET_CTRL_SHIFT);
  return true;
}

// ============================================================================
// REG 0x19 CHARGER_CTRL_3 - Charger Control 3 Register
// ============================================================================

bool BQ25628EComponent::set_charger_ctrl_3_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charger control 3 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x19 (CHARGER_CTRL_3)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charger control 3 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charger_ctrl_3_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x19 (CHARGER_CTRL_3)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger control 3 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_ibat_peak_threshold_(IbatPkThreshold threshold) {
  // Only 6A and 12A are valid
  if (threshold != IBAT_PK_6A && threshold != IBAT_PK_12A) {
    ESP_LOGE(TAG, "Invalid IBAT_PK threshold. Only 6A or 12A supported");
    return false;
  }
  
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  
  reg = (reg & ~CHARGER_CTRL_3_IBAT_PK_MASK) | ((threshold << CHARGER_CTRL_3_IBAT_PK_SHIFT) & CHARGER_CTRL_3_IBAT_PK_MASK);
  
  const char* threshold_str[] = {"", "", "6A", "12A"};
  ESP_LOGD(TAG, "Setting battery peak current threshold: %s", threshold_str[threshold]);
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg);
}

bool BQ25628EComponent::set_vbat_uvlo_threshold_(VbatUvloThreshold threshold) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  
  if (threshold == VBAT_UVLO_1_8V) {
    reg |= CHARGER_CTRL_3_VBAT_UVLO;
  } else {
    reg &= ~CHARGER_CTRL_3_VBAT_UVLO;
  }
  
  ESP_LOGD(TAG, "Setting VBAT UVLO threshold: %s (VBAT_SHORT: %s)", 
           threshold == VBAT_UVLO_1_8V ? "1.8V" : "2.2V",
           threshold == VBAT_UVLO_1_8V ? "1.85V" : "2.05V");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg);
}

bool BQ25628EComponent::set_external_ilim_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= CHARGER_CTRL_3_EN_EXTILIM;
  } else {
    reg &= ~CHARGER_CTRL_3_EN_EXTILIM;
  }
  
  ESP_LOGD(TAG, "Setting external ILIM pin: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg);
}

bool BQ25628EComponent::set_charge_rate_(ChargeRate rate) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  
  reg = (reg & ~CHARGER_CTRL_3_CHG_RATE_MASK) | ((rate << CHARGER_CTRL_3_CHG_RATE_SHIFT) & CHARGER_CTRL_3_CHG_RATE_MASK);
  
  const char* rate_str[] = {"1C", "2C", "4C", "6C"};
  ESP_LOGD(TAG, "Setting charge rate: %s", rate_str[rate]);
  return this->write_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg);
}

bool BQ25628EComponent::get_ibat_peak_threshold_(IbatPkThreshold &threshold) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  threshold = static_cast<IbatPkThreshold>((reg & CHARGER_CTRL_3_IBAT_PK_MASK) >> CHARGER_CTRL_3_IBAT_PK_SHIFT);
  return true;
}

bool BQ25628EComponent::get_vbat_uvlo_threshold_(VbatUvloThreshold &threshold) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  threshold = (reg & CHARGER_CTRL_3_VBAT_UVLO) ? VBAT_UVLO_1_8V : VBAT_UVLO_2_2V;
  return true;
}

bool BQ25628EComponent::get_external_ilim_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  enabled = (reg & CHARGER_CTRL_3_EN_EXTILIM) != 0;
  return true;
}

bool BQ25628EComponent::get_charge_rate_(ChargeRate &rate) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_CTRL_3, reg)) {
    return false;
  }
  rate = static_cast<ChargeRate>((reg & CHARGER_CTRL_3_CHG_RATE_MASK) >> CHARGER_CTRL_3_CHG_RATE_SHIFT);
  return true;
}

// ============================================================================
// REG 0x1A NTC_CTRL_0 - NTC Control 0 Register
// ============================================================================

bool BQ25628EComponent::set_ntc_ctrl_0_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting NTC control 0 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_NTC_CTRL_0, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x1A (NTC_CTRL_0)");
    return false;
  }
  
  ESP_LOGI(TAG, "NTC control 0 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_ntc_ctrl_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x1A (NTC_CTRL_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read NTC control 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_ts_ignore_(bool ignore) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg)) {
    return false;
  }
  
  if (ignore) {
    reg |= NTC_CTRL_0_TS_IGNORE;
  } else {
    reg &= ~NTC_CTRL_0_TS_IGNORE;
  }
  
  ESP_LOGD(TAG, "Setting TS ignore: %s", ignore ? "ignore TS" : "monitor TS");
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg);
}

bool BQ25628EComponent::set_ts_warm_current_(TsCurrentSetting setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_0_TS_ISET_WARM_MASK) | ((setting << NTC_CTRL_0_TS_ISET_WARM_SHIFT) & NTC_CTRL_0_TS_ISET_WARM_MASK);
  
  const char* setting_str[] = {"Suspend", "20% ICHG", "40% ICHG", "ICHG unchanged"};
  ESP_LOGD(TAG, "Setting TS warm current: %s", setting_str[setting]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg);
}

bool BQ25628EComponent::set_ts_cool_current_(TsCurrentSetting setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_0_TS_ISET_COOL_MASK) | ((setting << NTC_CTRL_0_TS_ISET_COOL_SHIFT) & NTC_CTRL_0_TS_ISET_COOL_MASK);
  
  const char* setting_str[] = {"Suspend", "20% ICHG", "40% ICHG", "ICHG unchanged"};
  ESP_LOGD(TAG, "Setting TS cool current: %s", setting_str[setting]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg);
}

bool BQ25628EComponent::get_ts_ignore_(bool &ignore) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg)) {
    return false;
  }
  ignore = (reg & NTC_CTRL_0_TS_IGNORE) != 0;
  return true;
}

bool BQ25628EComponent::get_ts_warm_current_(TsCurrentSetting &setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg)) {
    return false;
  }
  setting = static_cast<TsCurrentSetting>((reg & NTC_CTRL_0_TS_ISET_WARM_MASK) >> NTC_CTRL_0_TS_ISET_WARM_SHIFT);
  return true;
}

bool BQ25628EComponent::get_ts_cool_current_(TsCurrentSetting &setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_0, reg)) {
    return false;
  }
  setting = static_cast<TsCurrentSetting>((reg & NTC_CTRL_0_TS_ISET_COOL_MASK) >> NTC_CTRL_0_TS_ISET_COOL_SHIFT);
  return true;
}

// ============================================================================
// REG 0x1B NTC_CTRL_1 - NTC Control 1 Register
// ============================================================================

bool BQ25628EComponent::set_ntc_ctrl_1_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting NTC control 1 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_NTC_CTRL_1, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x1B (NTC_CTRL_1)");
    return false;
  }
  
  ESP_LOGI(TAG, "NTC control 1 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_ntc_ctrl_1_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_1, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x1B (NTC_CTRL_1)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read NTC control 1 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_ts_cold_thresholds_(TsColdThresholds thresholds) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_1_TS_TH123_MASK) | ((thresholds << NTC_CTRL_1_TS_TH123_SHIFT) & NTC_CTRL_1_TS_TH123_MASK);
  
  const char* threshold_str[] = {
    "TH1:0°C/TH2:5°C/TH3:15°C",
    "TH1:0°C/TH2:10°C/TH3:15°C",
    "TH1:0°C/TH2:15°C/TH3:20°C",
    "TH1:0°C/TH2:20°C/TH3:20°C",
    "TH1:-5°C/TH2:5°C/TH3:15°C",
    "TH1:-5°C/TH2:10°C/TH3:15°C",
    "TH1:-5°C/TH2:10°C/TH3:20°C",
    "TH1:0°C/TH2:10°C/TH3:20°C"
  };
  ESP_LOGD(TAG, "Setting TS cold thresholds: %s", threshold_str[thresholds]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg);
}

bool BQ25628EComponent::set_ts_hot_thresholds_(TsHotThresholds thresholds) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_1_TS_TH456_MASK) | ((thresholds << NTC_CTRL_1_TS_TH456_SHIFT) & NTC_CTRL_1_TS_TH456_MASK);
  
  const char* threshold_str[] = {
    "TH4:35°C/TH5:40°C/TH6:60°C",
    "TH4:35°C/TH5:45°C/TH6:60°C",
    "TH4:35°C/TH5:50°C/TH6:60°C",
    "TH4:40°C/TH5:55°C/TH6:60°C",
    "TH4:35°C/TH5:40°C/TH6:50°C",
    "TH4:35°C/TH5:45°C/TH6:50°C",
    "TH4:40°C/TH5:45°C/TH6:60°C",
    "TH4:40°C/TH5:50°C/TH6:60°C"
  };
  ESP_LOGD(TAG, "Setting TS hot thresholds: %s", threshold_str[thresholds]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg);
}

bool BQ25628EComponent::set_ts_warm_voltage_(TsWarmVoltageSetting setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_1_TS_VSET_WARM_MASK) | ((setting << NTC_CTRL_1_TS_VSET_WARM_SHIFT) & NTC_CTRL_1_TS_VSET_WARM_MASK);
  
  const char* setting_str[] = {"VREG-300mV", "VREG-200mV", "VREG-100mV", "VREG unchanged"};
  ESP_LOGD(TAG, "Setting TS warm voltage: %s", setting_str[setting]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg);
}

bool BQ25628EComponent::get_ts_cold_thresholds_(TsColdThresholds &thresholds) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg)) {
    return false;
  }
  thresholds = static_cast<TsColdThresholds>((reg & NTC_CTRL_1_TS_TH123_MASK) >> NTC_CTRL_1_TS_TH123_SHIFT);
  return true;
}

bool BQ25628EComponent::get_ts_hot_thresholds_(TsHotThresholds &thresholds) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg)) {
    return false;
  }
  thresholds = static_cast<TsHotThresholds>((reg & NTC_CTRL_1_TS_TH456_MASK) >> NTC_CTRL_1_TS_TH456_SHIFT);
  return true;
}

bool BQ25628EComponent::get_ts_warm_voltage_(TsWarmVoltageSetting &setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_1, reg)) {
    return false;
  }
  setting = static_cast<TsWarmVoltageSetting>((reg & NTC_CTRL_1_TS_VSET_WARM_MASK) >> NTC_CTRL_1_TS_VSET_WARM_SHIFT);
  return true;
}

// ============================================================================
// REG 0x1C NTC_CTRL_2 - NTC Control 2 Register
// ============================================================================

bool BQ25628EComponent::set_ntc_ctrl_2_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting NTC control 2 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_NTC_CTRL_2, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x1C (NTC_CTRL_2)");
    return false;
  }
  
  ESP_LOGI(TAG, "NTC control 2 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_ntc_ctrl_2_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x1C (NTC_CTRL_2)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read NTC control 2 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_ts_voltage_symmetric_(bool symmetric) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  
  if (symmetric) {
    reg |= NTC_CTRL_2_TS_VSET_SYM;
  } else {
    reg &= ~NTC_CTRL_2_TS_VSET_SYM;
  }
  
  ESP_LOGD(TAG, "Setting TS voltage symmetric: %s", symmetric ? "TS_COOL matches TS_WARM" : "VREG unchanged");
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg);
}

bool BQ25628EComponent::set_ts_prewarm_voltage_(TsWarmVoltageSetting setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_2_TS_VSET_PREWARM_MASK) | ((setting << NTC_CTRL_2_TS_VSET_PREWARM_SHIFT) & NTC_CTRL_2_TS_VSET_PREWARM_MASK);
  
  const char* setting_str[] = {"VREG-300mV", "VREG-200mV", "VREG-100mV", "VREG unchanged"};
  ESP_LOGD(TAG, "Setting TS prewarm voltage: %s", setting_str[setting]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg);
}

bool BQ25628EComponent::set_ts_prewarm_current_(TsCurrentSetting setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_2_TS_ISET_PREWARM_MASK) | ((setting << NTC_CTRL_2_TS_ISET_PREWARM_SHIFT) & NTC_CTRL_2_TS_ISET_PREWARM_MASK);
  
  const char* setting_str[] = {"Suspend", "20% ICHG", "40% ICHG", "ICHG unchanged"};
  ESP_LOGD(TAG, "Setting TS prewarm current: %s", setting_str[setting]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg);
}

bool BQ25628EComponent::set_ts_precool_current_(TsCurrentSetting setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  
  reg = (reg & ~NTC_CTRL_2_TS_ISET_PRECOOL_MASK) | ((setting << NTC_CTRL_2_TS_ISET_PRECOOL_SHIFT) & NTC_CTRL_2_TS_ISET_PRECOOL_MASK);
  
  const char* setting_str[] = {"Suspend", "20% ICHG", "40% ICHG", "ICHG unchanged"};
  ESP_LOGD(TAG, "Setting TS precool current: %s", setting_str[setting]);
  return this->write_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg);
}

bool BQ25628EComponent::get_ts_voltage_symmetric_(bool &symmetric) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  symmetric = (reg & NTC_CTRL_2_TS_VSET_SYM) != 0;
  return true;
}

bool BQ25628EComponent::get_ts_prewarm_voltage_(TsWarmVoltageSetting &setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  setting = static_cast<TsWarmVoltageSetting>((reg & NTC_CTRL_2_TS_VSET_PREWARM_MASK) >> NTC_CTRL_2_TS_VSET_PREWARM_SHIFT);
  return true;
}

bool BQ25628EComponent::get_ts_prewarm_current_(TsCurrentSetting &setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  setting = static_cast<TsCurrentSetting>((reg & NTC_CTRL_2_TS_ISET_PREWARM_MASK) >> NTC_CTRL_2_TS_ISET_PREWARM_SHIFT);
  return true;
}

bool BQ25628EComponent::get_ts_precool_current_(TsCurrentSetting &setting) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_NTC_CTRL_2, reg)) {
    return false;
  }
  setting = static_cast<TsCurrentSetting>((reg & NTC_CTRL_2_TS_ISET_PRECOOL_MASK) >> NTC_CTRL_2_TS_ISET_PRECOOL_SHIFT);
  return true;
}

// ============================================================================
// REG 0x1D CHARGER_STATUS_0 - Charger Status 0 Register (Read-only)
// ============================================================================

bool BQ25628EComponent::get_charger_status_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x1D (CHARGER_STATUS_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger status 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_adc_conversion_done_(bool &done) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, reg)) {
    return false;
  }
  done = (reg & CHARGER_STATUS_0_ADC_DONE) != 0;
  return true;
}

bool BQ25628EComponent::get_thermal_regulation_status_(bool &active) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, reg)) {
    return false;
  }
  active = (reg & CHARGER_STATUS_0_TREG_STAT) != 0;
  return true;
}

bool BQ25628EComponent::get_vsys_regulation_status_(bool &active) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, reg)) {
    return false;
  }
  active = (reg & CHARGER_STATUS_0_VSYS_STAT) != 0;
  return true;
}

bool BQ25628EComponent::get_iindpm_regulation_status_(bool &active) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, reg)) {
    return false;
  }
  active = (reg & CHARGER_STATUS_0_IINDPM_STAT) != 0;
  return true;
}

bool BQ25628EComponent::get_vindpm_regulation_status_(bool &active) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, reg)) {
    return false;
  }
  active = (reg & CHARGER_STATUS_0_VINDPM_STAT) != 0;
  return true;
}

bool BQ25628EComponent::get_safety_timer_expired_(bool &expired) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, reg)) {
    return false;
  }
  expired = (reg & CHARGER_STATUS_0_SAFETY_TMR) != 0;
  return true;
}

bool BQ25628EComponent::get_watchdog_timer_expired_(bool &expired) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_0, reg)) {
    return false;
  }
  expired = (reg & CHARGER_STATUS_0_WD_STAT) != 0;
  return true;
}

// ============================================================================
// REG 0x1E CHARGER_STATUS_1 - Charger Status 1 Register (Read-only)
// ============================================================================

bool BQ25628EComponent::get_charger_status_1_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_1, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x1E (CHARGER_STATUS_1)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger status 1 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charge_status_(ChargeStatus &status) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_1, reg)) {
    return false;
  }
  status = static_cast<ChargeStatus>((reg & CHARGER_STATUS_1_CHG_STAT_MASK) >> CHARGER_STATUS_1_CHG_STAT_SHIFT);
  return true;
}

bool BQ25628EComponent::get_vbus_status_(VbusStatus &status) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_STATUS_1, reg)) {
    return false;
  }
  uint8_t vbus_val = (reg & CHARGER_STATUS_1_VBUS_STAT_MASK) >> CHARGER_STATUS_1_VBUS_STAT_SHIFT;
  
  // Map known values, return as-is for unknown (datasheet incomplete)
  if (vbus_val == 0) {
    status = VBUS_NOT_POWERED;
  } else if (vbus_val == 4) {
    status = VBUS_UNKNOWN_ADAPTER;
  } else {
    // Unknown value - log and return raw value cast to enum
    ESP_LOGW(TAG, "Unknown VBUS_STAT value: 0x%02X", vbus_val);
    status = static_cast<VbusStatus>(vbus_val);
  }
  return true;
}

// ============================================================================
// REG 0x1F FAULT_STATUS_0 - Fault Status 0 Register (Read-only)
// ============================================================================

bool BQ25628EComponent::get_fault_status_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_STATUS_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x1F (FAULT_STATUS_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read fault status 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_vbus_fault_status_(bool &fault) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_STATUS_0, reg)) {
    return false;
  }
  fault = (reg & FAULT_STATUS_0_VBUS_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_battery_fault_status_(bool &fault) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_STATUS_0, reg)) {
    return false;
  }
  fault = (reg & FAULT_STATUS_0_BAT_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_system_fault_status_(bool &fault) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_STATUS_0, reg)) {
    return false;
  }
  fault = (reg & FAULT_STATUS_0_SYS_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_thermal_shutdown_status_(bool &shutdown) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_STATUS_0, reg)) {
    return false;
  }
  shutdown = (reg & FAULT_STATUS_0_TSHUT_STAT) != 0;
  return true;
}

bool BQ25628EComponent::get_ts_temperature_zone_(TsTemperatureZone &zone) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_STATUS_0, reg)) {
    return false;
  }
  zone = static_cast<TsTemperatureZone>((reg & FAULT_STATUS_0_TS_STAT_MASK) >> FAULT_STATUS_0_TS_STAT_SHIFT);
  return true;
}

// ============================================================================
// REG 0x20 CHARGER_FLAG_0 - Charger Flag 0 Register (Read-only)
// ============================================================================

bool BQ25628EComponent::get_charger_flag_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x20 (CHARGER_FLAG_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger flag 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_adc_conversion_done_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_0_ADC_DONE) != 0;
  return true;
}

bool BQ25628EComponent::get_thermal_regulation_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_0_TREG_FLAG) != 0;
  return true;
}

bool BQ25628EComponent::get_vsys_regulation_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_0_VSYS_FLAG) != 0;
  return true;
}

bool BQ25628EComponent::get_iindpm_regulation_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_0_IINDPM_FLAG) != 0;
  return true;
}

bool BQ25628EComponent::get_vindpm_regulation_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_0_VINDPM_FLAG) != 0;
  return true;
}

bool BQ25628EComponent::get_safety_timer_expired_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_0_SAFETY_TMR) != 0;
  return true;
}

bool BQ25628EComponent::get_watchdog_timer_expired_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_0_WD_FLAG) != 0;
  return true;
}

// ============================================================================
// REG 0x21 CHARGER_FLAG_1 - Charger Flag 1 Register (Read-only)
// ============================================================================

bool BQ25628EComponent::get_charger_flag_1_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_1, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x21 (CHARGER_FLAG_1)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger flag 1 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charge_status_changed_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_1, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_1_CHG_FLAG) != 0;
  return true;
}

bool BQ25628EComponent::get_vbus_status_changed_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_FLAG_1, reg)) {
    return false;
  }
  flag = (reg & CHARGER_FLAG_1_VBUS_FLAG) != 0;
  return true;
}

// ============================================================================
// REG 0x22 FAULT_FLAG_0 - Fault Flag 0 Register (Read-only)
// ============================================================================

bool BQ25628EComponent::get_fault_flag_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_FLAG_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x22 (FAULT_FLAG_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read fault flag 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_vbus_fault_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & FAULT_FLAG_0_VBUS_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_battery_fault_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & FAULT_FLAG_0_BAT_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_system_fault_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & FAULT_FLAG_0_SYS_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_thermal_shutdown_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & FAULT_FLAG_0_TSHUT_FLAG) != 0;
  return true;
}

bool BQ25628EComponent::get_ts_status_changed_flag_(bool &flag) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_FLAG_0, reg)) {
    return false;
  }
  flag = (reg & FAULT_FLAG_0_TS_FLAG) != 0;
  return true;
}

// ============================================================================
// REG 0x23 CHARGER_MASK_0 - Charger Mask 0 Register
// ============================================================================

bool BQ25628EComponent::set_charger_mask_0_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charger mask 0 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x23 (CHARGER_MASK_0)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charger mask 0 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charger_mask_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x23 (CHARGER_MASK_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger mask 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_adc_conversion_done_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_0_ADC_DONE;
  } else {
    reg &= ~CHARGER_MASK_0_ADC_DONE;
  }
  
  ESP_LOGD(TAG, "Setting ADC conversion done mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg);
}

bool BQ25628EComponent::set_thermal_regulation_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_0_TREG_MASK;
  } else {
    reg &= ~CHARGER_MASK_0_TREG_MASK;
  }
  
  ESP_LOGD(TAG, "Setting thermal regulation mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg);
}

bool BQ25628EComponent::set_vsys_regulation_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_0_VSYS_MASK;
  } else {
    reg &= ~CHARGER_MASK_0_VSYS_MASK;
  }
  
  ESP_LOGD(TAG, "Setting VSYS regulation mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg);
}

bool BQ25628EComponent::set_iindpm_regulation_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_0_IINDPM_MASK;
  } else {
    reg &= ~CHARGER_MASK_0_IINDPM_MASK;
  }
  
  ESP_LOGD(TAG, "Setting IINDPM/ILIM mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg);
}

bool BQ25628EComponent::set_vindpm_regulation_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_0_VINDPM_MASK;
  } else {
    reg &= ~CHARGER_MASK_0_VINDPM_MASK;
  }
  
  ESP_LOGD(TAG, "Setting VINDPM mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg);
}

bool BQ25628EComponent::set_safety_timer_expired_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_0_SAFETY_TMR;
  } else {
    reg &= ~CHARGER_MASK_0_SAFETY_TMR;
  }
  
  ESP_LOGD(TAG, "Setting safety timer mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg);
}

bool BQ25628EComponent::set_watchdog_timer_expired_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_0_WD_MASK;
  } else {
    reg &= ~CHARGER_MASK_0_WD_MASK;
  }
  
  ESP_LOGD(TAG, "Setting watchdog timer mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg);
}

bool BQ25628EComponent::get_adc_conversion_done_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_0_ADC_DONE) != 0;
  return true;
}

bool BQ25628EComponent::get_thermal_regulation_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_0_TREG_MASK) != 0;
  return true;
}

bool BQ25628EComponent::get_vsys_regulation_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_0_VSYS_MASK) != 0;
  return true;
}

bool BQ25628EComponent::get_iindpm_regulation_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_0_IINDPM_MASK) != 0;
  return true;
}

bool BQ25628EComponent::get_vindpm_regulation_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_0_VINDPM_MASK) != 0;
  return true;
}

bool BQ25628EComponent::get_safety_timer_expired_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_0_SAFETY_TMR) != 0;
  return true;
}

bool BQ25628EComponent::get_watchdog_timer_expired_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_0, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_0_WD_MASK) != 0;
  return true;
}

// ============================================================================
// REG 0x24 CHARGER_MASK_1 - Charger Mask 1 Register
// ============================================================================

bool BQ25628EComponent::set_charger_mask_1_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting charger mask 1 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_1, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x24 (CHARGER_MASK_1)");
    return false;
  }
  
  ESP_LOGI(TAG, "Charger mask 1 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_charger_mask_1_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_1, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x24 (CHARGER_MASK_1)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read charger mask 1 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_charge_status_changed_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_1, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_1_CHG_MASK;
  } else {
    reg &= ~CHARGER_MASK_1_CHG_MASK;
  }
  
  ESP_LOGD(TAG, "Setting charge status change mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_1, reg);
}

bool BQ25628EComponent::set_vbus_status_changed_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_1, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= CHARGER_MASK_1_VBUS_MASK;
  } else {
    reg &= ~CHARGER_MASK_1_VBUS_MASK;
  }
  
  ESP_LOGD(TAG, "Setting VBUS status change mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_CHARGER_MASK_1, reg);
}

bool BQ25628EComponent::get_charge_status_changed_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_1, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_1_CHG_MASK) != 0;
  return true;
}

bool BQ25628EComponent::get_vbus_status_changed_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_CHARGER_MASK_1, reg)) {
    return false;
  }
  mask = (reg & CHARGER_MASK_1_VBUS_MASK) != 0;
  return true;
}

// ============================================================================
// REG 0x25 FAULT_MASK_0 - Fault Mask 0 Register
// ============================================================================

bool BQ25628EComponent::set_fault_mask_0_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting fault mask 0 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_FAULT_MASK_0, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x25 (FAULT_MASK_0)");
    return false;
  }
  
  ESP_LOGI(TAG, "Fault mask 0 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_fault_mask_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x25 (FAULT_MASK_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read fault mask 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_vbus_fault_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= FAULT_MASK_0_VBUS_FAULT;
  } else {
    reg &= ~FAULT_MASK_0_VBUS_FAULT;
  }
  
  ESP_LOGD(TAG, "Setting VBUS fault mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg);
}

bool BQ25628EComponent::set_battery_fault_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= FAULT_MASK_0_BAT_FAULT;
  } else {
    reg &= ~FAULT_MASK_0_BAT_FAULT;
  }
  
  ESP_LOGD(TAG, "Setting battery fault mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg);
}

bool BQ25628EComponent::set_system_fault_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= FAULT_MASK_0_SYS_FAULT;
  } else {
    reg &= ~FAULT_MASK_0_SYS_FAULT;
  }
  
  ESP_LOGD(TAG, "Setting system fault mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg);
}

bool BQ25628EComponent::set_thermal_shutdown_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= FAULT_MASK_0_TSHUT_MASK;
  } else {
    reg &= ~FAULT_MASK_0_TSHUT_MASK;
  }
  
  ESP_LOGD(TAG, "Setting thermal shutdown mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg);
}

bool BQ25628EComponent::set_ts_temperature_zone_mask_(bool mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  
  if (mask) {
    reg |= FAULT_MASK_0_TS_MASK;
  } else {
    reg &= ~FAULT_MASK_0_TS_MASK;
  }
  
  ESP_LOGD(TAG, "Setting TS temperature zone mask: %s", mask ? "masked" : "not masked");
  return this->write_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg);
}

bool BQ25628EComponent::get_vbus_fault_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  mask = (reg & FAULT_MASK_0_VBUS_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_battery_fault_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  mask = (reg & FAULT_MASK_0_BAT_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_system_fault_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  mask = (reg & FAULT_MASK_0_SYS_FAULT) != 0;
  return true;
}

bool BQ25628EComponent::get_thermal_shutdown_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  mask = (reg & FAULT_MASK_0_TSHUT_MASK) != 0;
  return true;
}

bool BQ25628EComponent::get_ts_temperature_zone_mask_(bool &mask) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_FAULT_MASK_0, reg)) {
    return false;
  }
  mask = (reg & FAULT_MASK_0_TS_MASK) != 0;
  return true;
}

// ============================================================================
// REG 0x26 ADC_CONTROL - ADC Control Register
// ============================================================================

bool BQ25628EComponent::set_adc_control_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting ADC control register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_ADC_CONTROL, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x26 (ADC_CONTROL)");
    return false;
  }
  
  ESP_LOGI(TAG, "ADC control register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_adc_control_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x26 (ADC_CONTROL)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read ADC control register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_adc_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= ADC_CONTROL_ADC_EN;
  } else {
    reg &= ~ADC_CONTROL_ADC_EN;
  }
  
  ESP_LOGD(TAG, "Setting ADC: %s", enable ? "enabled" : "disabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_CONTROL, reg);
}

bool BQ25628EComponent::set_adc_rate_oneshot_(bool oneshot) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  
  if (oneshot) {
    reg |= ADC_CONTROL_ADC_RATE;
  } else {
    reg &= ~ADC_CONTROL_ADC_RATE;
  }
  
  ESP_LOGD(TAG, "Setting ADC rate: %s", oneshot ? "one-shot" : "continuous");
  return this->write_register_byte_(BQ25628E_REG_ADC_CONTROL, reg);
}

bool BQ25628EComponent::set_adc_sample_speed_(AdcSampleSpeed speed) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  
  reg = (reg & ~ADC_CONTROL_ADC_SAMPLE_MASK) | ((speed << ADC_CONTROL_ADC_SAMPLE_SHIFT) & ADC_CONTROL_ADC_SAMPLE_MASK);
  
  const char* speed_str[] = {"12-bit", "11-bit", "10-bit", "9-bit"};
  ESP_LOGD(TAG, "Setting ADC sample speed: %s", speed_str[speed]);
  return this->write_register_byte_(BQ25628E_REG_ADC_CONTROL, reg);
}

bool BQ25628EComponent::set_adc_average_enabled_(bool enable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  
  if (enable) {
    reg |= ADC_CONTROL_ADC_AVG;
  } else {
    reg &= ~ADC_CONTROL_ADC_AVG;
  }
  
  ESP_LOGD(TAG, "Setting ADC average: %s", enable ? "running average" : "single value");
  return this->write_register_byte_(BQ25628E_REG_ADC_CONTROL, reg);
}

bool BQ25628EComponent::set_adc_average_init_new_(bool use_new) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  
  if (use_new) {
    reg |= ADC_CONTROL_ADC_AVG_INIT;
  } else {
    reg &= ~ADC_CONTROL_ADC_AVG_INIT;
  }
  
  ESP_LOGD(TAG, "Setting ADC average init: %s", use_new ? "new conversion" : "existing value");
  return this->write_register_byte_(BQ25628E_REG_ADC_CONTROL, reg);
}

bool BQ25628EComponent::get_adc_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  enabled = (reg & ADC_CONTROL_ADC_EN) != 0;
  return true;
}

bool BQ25628EComponent::get_adc_rate_oneshot_(bool &oneshot) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  oneshot = (reg & ADC_CONTROL_ADC_RATE) != 0;
  return true;
}

bool BQ25628EComponent::get_adc_sample_speed_(AdcSampleSpeed &speed) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  speed = static_cast<AdcSampleSpeed>((reg & ADC_CONTROL_ADC_SAMPLE_MASK) >> ADC_CONTROL_ADC_SAMPLE_SHIFT);
  return true;
}

bool BQ25628EComponent::get_adc_average_enabled_(bool &enabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  enabled = (reg & ADC_CONTROL_ADC_AVG) != 0;
  return true;
}

bool BQ25628EComponent::get_adc_average_init_new_(bool &use_new) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_CONTROL, reg)) {
    return false;
  }
  use_new = (reg & ADC_CONTROL_ADC_AVG_INIT) != 0;
  return true;
}

// ============================================================================
// REG 0x27 ADC_FUNCTION_DISABLE_0 - ADC Function Disable Register
// ============================================================================

bool BQ25628EComponent::set_adc_function_disable_0_reg_(uint8_t value) {
  ESP_LOGD(TAG, "Setting ADC function disable 0 register: 0x%02X", value);
  
  if (!this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, value)) {
    ESP_LOGE(TAG, "Failed to write REG0x27 (ADC_FUNCTION_DISABLE_0)");
    return false;
  }
  
  ESP_LOGI(TAG, "ADC function disable 0 register set to 0x%02X", value);
  return true;
}

bool BQ25628EComponent::get_adc_function_disable_0_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, value)) {
    ESP_LOGE(TAG, "Failed to read REG0x27 (ADC_FUNCTION_DISABLE_0)");
    return false;
  }
  
  ESP_LOGD(TAG, "Read ADC function disable 0 register: 0x%02X", value);
  return true;
}

bool BQ25628EComponent::set_ibus_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_IBUS;
  } else {
    reg &= ~ADC_FUNCTION_DIS_IBUS;
  }
  
  ESP_LOGD(TAG, "Setting IBUS ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::set_ibat_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_IBAT;
  } else {
    reg &= ~ADC_FUNCTION_DIS_IBAT;
  }
  
  ESP_LOGD(TAG, "Setting IBAT ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::set_vbus_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_VBUS;
  } else {
    reg &= ~ADC_FUNCTION_DIS_VBUS;
  }
  
  ESP_LOGD(TAG, "Setting VBUS ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::set_vbat_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_VBAT;
  } else {
    reg &= ~ADC_FUNCTION_DIS_VBAT;
  }
  
  ESP_LOGD(TAG, "Setting VBAT ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::set_vsys_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_VSYS;
  } else {
    reg &= ~ADC_FUNCTION_DIS_VSYS;
  }
  
  ESP_LOGD(TAG, "Setting VSYS ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::set_ts_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_TS;
  } else {
    reg &= ~ADC_FUNCTION_DIS_TS;
  }
  
  ESP_LOGD(TAG, "Setting TS ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::set_tdie_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_TDIE;
  } else {
    reg &= ~ADC_FUNCTION_DIS_TDIE;
  }
  
  ESP_LOGD(TAG, "Setting TDIE ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::set_vpmid_adc_disabled_(bool disable) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  
  if (disable) {
    reg |= ADC_FUNCTION_DIS_VPMID;
  } else {
    reg &= ~ADC_FUNCTION_DIS_VPMID;
  }
  
  ESP_LOGD(TAG, "Setting VPMID ADC: %s", disable ? "disabled" : "enabled");
  return this->write_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg);
}

bool BQ25628EComponent::get_ibus_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_IBUS) != 0;
  return true;
}

bool BQ25628EComponent::get_ibat_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_IBAT) != 0;
  return true;
}

bool BQ25628EComponent::get_vbus_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_VBUS) != 0;
  return true;
}

bool BQ25628EComponent::get_vbat_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_VBAT) != 0;
  return true;
}

bool BQ25628EComponent::get_vsys_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_VSYS) != 0;
  return true;
}

bool BQ25628EComponent::get_ts_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_TS) != 0;
  return true;
}

bool BQ25628EComponent::get_tdie_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_TDIE) != 0;
  return true;
}

bool BQ25628EComponent::get_vpmid_adc_disabled_(bool &disabled) {
  uint8_t reg;
  if (!this->read_register_byte_(BQ25628E_REG_ADC_FUNCTION_DISABLE_0, reg)) {
    return false;
  }
  disabled = (reg & ADC_FUNCTION_DIS_VPMID) != 0;
  return true;
}

// ============================================================================
// REG 0x28-0x29 IBUS_ADC - IBUS ADC Reading (Read-only, 16-bit)
// ============================================================================

bool BQ25628EComponent::get_ibus_adc_(int16_t &current_ma) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x28) and LSB (REG0x29)
  if (!this->read_register_byte_(BQ25628E_REG_IBUS_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read IBUS_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_IBUS_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read IBUS_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value (2's complement)
  int16_t raw_value = (static_cast<int16_t>(msb) << 8) | lsb;
  
  // Convert to mA: 2mA per LSB
  current_ma = raw_value * 2;
  
  ESP_LOGD(TAG, "Read IBUS ADC: %d mA (raw: 0x%04X)", current_ma, static_cast<uint16_t>(raw_value));
  return true;
}

// ============================================================================
// REG 0x2A-0x2B IBAT_ADC - IBAT ADC Reading (Read-only, 16-bit, bits 15:2)
// ============================================================================

bool BQ25628EComponent::get_ibat_adc_(int16_t &current_ma) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x2A) and LSB (REG0x2B)
  if (!this->read_register_byte_(BQ25628E_REG_IBAT_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read IBAT_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_IBAT_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read IBAT_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value
  uint16_t raw_16bit = (static_cast<uint16_t>(msb) << 8) | lsb;
  
  // Check for polarity change error (0x8000 in full register = 0x2000 in IBAT_ADC field)
  if (raw_16bit == 0x8000) {
    ESP_LOGW(TAG, "IBAT ADC polarity change during measurement (0x8000)");
    current_ma = 0;
    return false;
  }
  
  // Extract bits 15:2 (14-bit field) and sign-extend to 16-bit
  int16_t ibat_field = static_cast<int16_t>(raw_16bit) >> 2;
  
  // Convert to mA: 4mA per LSB
  current_ma = ibat_field * 4;
  
  ESP_LOGD(TAG, "Read IBAT ADC: %d mA (raw: 0x%04X, field: 0x%04X)", 
           current_ma, raw_16bit, static_cast<uint16_t>(ibat_field));
  return true;
}

// ============================================================================
// REG 0x2C-0x2D VBUS_ADC - VBUS ADC Reading (Read-only, 16-bit, bits 14:2)
// ============================================================================

bool BQ25628EComponent::get_vbus_adc_(uint16_t &voltage_mv) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x2C) and LSB (REG0x2D)
  if (!this->read_register_byte_(BQ25628E_REG_VBUS_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read VBUS_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_VBUS_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read VBUS_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value
  uint16_t raw_16bit = (static_cast<uint16_t>(msb) << 8) | lsb;
  
  // Extract bits 14:2 (13-bit field)
  uint16_t vbus_field = (raw_16bit >> 2) & 0x1FFF;
  
  // Convert to mV: 3.97mV per LSB (using integer approximation: multiply by 397, divide by 100)
  voltage_mv = (vbus_field * 397) / 100;
  
  ESP_LOGD(TAG, "Read VBUS ADC: %u mV (raw: 0x%04X, field: 0x%04X)", 
           voltage_mv, raw_16bit, vbus_field);
  return true;
}

// ============================================================================
// REG 0x2E-0x2F VPMID_ADC - VPMID ADC Reading (Read-only, 16-bit, bits 14:2)
// ============================================================================

bool BQ25628EComponent::get_vpmid_adc_(uint16_t &voltage_mv) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x2E) and LSB (REG0x2F)
  if (!this->read_register_byte_(BQ25628E_REG_VPMID_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read VPMID_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_VPMID_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read VPMID_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value
  uint16_t raw_16bit = (static_cast<uint16_t>(msb) << 8) | lsb;
  
  // Extract bits 14:2 (13-bit field)
  uint16_t vpmid_field = (raw_16bit >> 2) & 0x1FFF;
  
  // Convert to mV: 3.97mV per LSB (using integer approximation: multiply by 397, divide by 100)
  voltage_mv = (vpmid_field * 397) / 100;
  
  ESP_LOGD(TAG, "Read VPMID ADC: %u mV (raw: 0x%04X, field: 0x%04X)", 
           voltage_mv, raw_16bit, vpmid_field);
  return true;
}

// ============================================================================
// REG 0x30-0x31 VBAT_ADC - VBAT ADC Reading (Read-only, 16-bit, bits 12:1)
// ============================================================================

bool BQ25628EComponent::get_vbat_adc_(uint16_t &voltage_mv) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x30) and LSB (REG0x31)
  if (!this->read_register_byte_(BQ25628E_REG_VBAT_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read VBAT_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_VBAT_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read VBAT_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value
  uint16_t raw_16bit = (static_cast<uint16_t>(msb) << 8) | lsb;
  
  // Extract bits 12:1 (12-bit field)
  uint16_t vbat_field = (raw_16bit >> 1) & 0x0FFF;
  
  // Convert to mV: 1.99mV per LSB (using integer approximation: multiply by 199, divide by 100)
  voltage_mv = (vbat_field * 199) / 100;
  
  ESP_LOGD(TAG, "Read VBAT ADC: %u mV (raw: 0x%04X, field: 0x%04X)", 
           voltage_mv, raw_16bit, vbat_field);
  return true;
}

// ============================================================================
// REG 0x32-0x33 VSYS_ADC - VSYS ADC Reading (Read-only, 16-bit, bits 12:1)
// ============================================================================

bool BQ25628EComponent::get_vsys_adc_(uint16_t &voltage_mv) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x32) and LSB (REG0x33)
  if (!this->read_register_byte_(BQ25628E_REG_VSYS_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read VSYS_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_VSYS_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read VSYS_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value
  uint16_t raw_16bit = (static_cast<uint16_t>(msb) << 8) | lsb;
  
  // Extract bits 12:1 (12-bit field)
  uint16_t vsys_field = (raw_16bit >> 1) & 0x0FFF;
  
  // Convert to mV: 1.99mV per LSB (using integer approximation: multiply by 199, divide by 100)
  voltage_mv = (vsys_field * 199) / 100;
  
  ESP_LOGD(TAG, "Read VSYS ADC: %u mV (raw: 0x%04X, field: 0x%04X)", 
           voltage_mv, raw_16bit, vsys_field);
  return true;
}

// ============================================================================
// REG 0x34-0x35 TS_ADC - TS ADC Reading (Read-only, 16-bit, bits 11:0)
// ============================================================================

bool BQ25628EComponent::get_ts_adc_(float &percentage) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x34) and LSB (REG0x35)
  if (!this->read_register_byte_(BQ25628E_REG_TS_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read TS_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_TS_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read TS_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value
  uint16_t raw_16bit = (static_cast<uint16_t>(msb) << 8) | lsb;
  
  // Extract bits 11:0 (12-bit field)
  uint16_t ts_field = raw_16bit & 0x0FFF;
  
  // Convert to percentage: 0.0961% per LSB
  percentage = ts_field * 0.0961f;
  
  ESP_LOGD(TAG, "Read TS ADC: %.2f %% (raw: 0x%04X, field: 0x%04X)", 
           percentage, raw_16bit, ts_field);
  return true;
}

// ============================================================================
// REG 0x36-0x37 TDIE_ADC - TDIE ADC Reading (Read-only, 16-bit, bits 11:0)
// ============================================================================

bool BQ25628EComponent::get_tdie_adc_(float &temperature_c) {
  uint8_t msb, lsb;
  
  // Read MSB (REG0x36) and LSB (REG0x37)
  if (!this->read_register_byte_(BQ25628E_REG_TDIE_ADC, msb)) {
    ESP_LOGE(TAG, "Failed to read TDIE_ADC MSB");
    return false;
  }
  
  if (!this->read_register_byte_(BQ25628E_REG_TDIE_ADC + 1, lsb)) {
    ESP_LOGE(TAG, "Failed to read TDIE_ADC LSB");
    return false;
  }
  
  // Combine into 16-bit value
  uint16_t raw_16bit = (static_cast<uint16_t>(msb) << 8) | lsb;
  
  // Extract bits 11:0 (12-bit field)
  uint16_t tdie_field = raw_16bit & 0x0FFF;
  
  // Sign-extend from 12-bit to 16-bit (2's complement)
  int16_t tdie_signed;
  if (tdie_field & 0x0800) {
    // Negative number - set upper 4 bits to 1
    tdie_signed = static_cast<int16_t>(tdie_field | 0xF000);
  } else {
    // Positive number
    tdie_signed = static_cast<int16_t>(tdie_field);
  }
  
  // Convert to temperature: 0.5°C per LSB
  temperature_c = tdie_signed * 0.5f;
  
  ESP_LOGD(TAG, "Read TDIE ADC: %.1f °C (raw: 0x%04X, field: 0x%04X)", 
           temperature_c, raw_16bit, tdie_field);
  return true;
}

/******************************************************************************
 * REG 0x38 - PART_INFORMATION (Read-only)
 ******************************************************************************/
// Full register getter
bool BQ25628EComponent::get_part_information_reg_(uint8_t &value) {
  if (!this->read_register_byte_(BQ25628E_REG_PART_INFORMATION, value)) {
    ESP_LOGE(TAG, "Failed to read PART_INFORMATION register");
    return false;
  }
  ESP_LOGD(TAG, "PART_INFORMATION: 0x%02X", value);
  return true;
}

// Bits 5:3: Part number (4h = BQ25628E)
bool BQ25628EComponent::get_part_number_(uint8_t &part_number) {
  uint8_t reg_value;
  if (!get_part_information_reg_(reg_value)) {
    return false;
  }
  part_number = (reg_value & PART_INFO_PN_MASK) >> PART_INFO_PN_SHIFT;
  ESP_LOGD(TAG, "Part Number: 0x%01X (4h=BQ25628E)", part_number);
  return true;
}

// Bits 2:0: Device revision
bool BQ25628EComponent::get_device_revision_(uint8_t &revision) {
  uint8_t reg_value;
  if (!get_part_information_reg_(reg_value)) {
    return false;
  }
  revision = (reg_value & PART_INFO_DEV_REV_MASK) >> PART_INFO_DEV_REV_SHIFT;
  ESP_LOGD(TAG, "Device Revision: 0x%01X", revision);
  return true;
}

// ============================================================================
// I2C register access helper methods
// ============================================================================

bool BQ25628EComponent::write_register_word_(uint8_t reg, uint16_t value) {
  // Write 16-bit register in SINGLE I2C transaction (little-endian: LSB first)
  uint8_t data[2] = {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
  ESP_LOGD(TAG, "write_register_word_: reg=0x%02X, value=0x%04X (LSB=0x%02X, MSB=0x%02X)", reg, value, data[0], data[1]);
  if (!this->write_bytes(reg, data, 2)) {
    ESP_LOGE(TAG, "Failed to write 16-bit register 0x%02X", reg);
    return false;
  }
  ESP_LOGD(TAG, "Successfully wrote 0x%04X to 16-bit register 0x%02X", value, reg);
  return true;
}

bool BQ25628EComponent::write_register_byte_(uint8_t reg, uint8_t value) {
  if (!this->write_byte(reg, value)) {
    ESP_LOGW(TAG, "Failed to write register 0x%02X", reg);
    return false;
  }
  return true;
}

bool BQ25628EComponent::read_register_byte_(uint8_t reg, uint8_t &value) {
  if (!this->read_byte(reg, &value)) {
    ESP_LOGW(TAG, "Failed to read register 0x%02X", reg);
    return false;
  }
  return true;
}

bool BQ25628EComponent::read_register_word_(uint8_t reg, uint16_t &value) {
  // BQ25628E requires atomic 16-bit read in single I2C transaction
  // Separate read_byte calls may read wrong registers on some I2C implementations
  uint8_t data[2];
  if (!this->read_bytes(reg, data, 2)) {
    ESP_LOGW(TAG, "Failed to read 16-bit register 0x%02X", reg);
    return false;
  }
  // Little-endian: LSB at data[0], MSB at data[1]
  value = (static_cast<uint16_t>(data[1]) << 8) | data[0];
  return true;
}

}  // namespace bq25628e
}  // namespace esphome

