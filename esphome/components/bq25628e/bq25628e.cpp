#include "bq25628e.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace bq25628e {

static const char *const TAG = "bq25628e";

void BQ25628EComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BQ25628E...");
  
  // Configure the charger with initial settings
  if (!this->configure_charger_()) {
    ESP_LOGE(TAG, "Failed to configure BQ25628E");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "BQ25628E setup complete");
}

void BQ25628EComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BQ25628E:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with BQ25628E failed!");
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
}

void BQ25628EComponent::update() {
  // Reset watchdog to prevent expiration (which disables charging)
  this->reset_watchdog();
  
  // Read ADC values
  if (!this->read_adc_values_()) {
    ESP_LOGW(TAG, "Failed to read ADC values");
    return;
  }
}

bool BQ25628EComponent::configure_charger_() {
  ESP_LOGD(TAG, "Starting configure_charger_()");
  
  // Set charge current limit (datasheet: 40mA to 2000mA, step 40mA, LSB of 16-bit register)
  uint8_t ichg_val = (uint8_t)((this->charge_current_limit_ - ICHG_MIN) / ICHG_STEP);
  uint16_t ichg_word = ichg_val;  // MSB = 0x00, LSB = ichg_val
  ESP_LOGD(TAG, "About to write ICHG_CTRL: reg=0x%02X, value=0x%04X (%d), calculated current=%.2f A", 
           BQ25628E_REG_ICHG_CTRL, ichg_word, ichg_val, this->charge_current_limit_);
  if (!this->write_register_word_(BQ25628E_REG_ICHG_CTRL, ichg_word)) {
    ESP_LOGE(TAG, "Failed to set charge current limit (reg 0x%02X)", BQ25628E_REG_ICHG_CTRL);
    return false;
  }
  ESP_LOGD(TAG, "Charge current limit set to %.2f A", this->charge_current_limit_);
  
  // Set charge voltage limit (datasheet: 3.5V to 4.8V, step 10mV, LSB of 16-bit register)
  uint8_t vbat_val = (uint8_t)((this->charge_voltage_limit_ - VBAT_MIN) / VBAT_STEP);
  uint16_t vbat_word = vbat_val;  // MSB = 0x00, LSB = vbat_val
  if (!this->write_register_word_(BQ25628E_REG_VBAT_CTRL, vbat_word)) {
    ESP_LOGE(TAG, "Failed to set charge voltage limit");
    return false;
  }
  ESP_LOGD(TAG, "Charge voltage limit set to %.2f V", this->charge_voltage_limit_);
  
  // Set input current limit (datasheet: 100mA to 3200mA, step 20mA, LSB of 16-bit register)
  uint8_t iindpm_val = (uint8_t)((this->input_current_limit_ - IINDPM_MIN) / IINDPM_STEP);
  uint16_t iindpm_word = iindpm_val;  // MSB = 0x00, LSB = iindpm_val
  if (!this->write_register_word_(BQ25628E_REG_IINDPM_CTRL, iindpm_word)) {
    ESP_LOGE(TAG, "Failed to set input current limit");
    return false;
  }
  ESP_LOGD(TAG, "Input current limit set to %.2f A", this->input_current_limit_);
  
  // Set input voltage limit (VINDPM) (datasheet: 3.8V to 16.8V, step 40mV, LSB of 16-bit register)
  uint8_t vindpm_val = (uint8_t)((this->input_voltage_limit_ - VINDPM_MIN) / VINDPM_STEP);
  uint16_t vindpm_word = vindpm_val;  // MSB = 0x00, LSB = vindpm_val
  if (!this->write_register_word_(BQ25628E_REG_VINDPM_CTRL, vindpm_word)) {
    ESP_LOGE(TAG, "Failed to set input voltage limit");
    return false;
  }
  ESP_LOGD(TAG, "Input voltage limit set to %.2f V", this->input_voltage_limit_);
  
  // Set VBUS overvoltage protection threshold (REG0x0A: 6V to 16.76V, step 40mV, LSB of 16-bit register)
  // Chip masks lower 4 bits (16-step granularity), so use 0xFF for maximum threshold
  // 0xFF → chip accepts 0xF0 → 6.0 + (240 × 0.04) = 15.6V
  // Try 0xFF (maximum) to test if fault clears
  uint16_t vbusovp_word = 0x00FF;  // Maximum value
  if (!this->write_register_word_(BQ25628E_REG_VBUSOVP_CTRL, vbusovp_word)) {
    ESP_LOGE(TAG, "Failed to set VBUS OVP threshold");
    return false;
  }
  ESP_LOGD(TAG, "VBUS OVP threshold set to maximum (wrote 0x%04X)", vbusovp_word);
  
  // Read back to verify and decode actual threshold
  uint16_t vbusovp_readback = 0;
  if (this->read_register_word_(BQ25628E_REG_VBUSOVP_CTRL, vbusovp_readback)) {
    uint8_t threshold_val = vbusovp_readback & 0xFF;  // LSB contains threshold
    float actual_threshold = 6.0f + (threshold_val * 0.04f);
    ESP_LOGD(TAG, "VBUS OVP readback: 0x%04X → %.2fV threshold", vbusovp_readback, actual_threshold);
    if (vbusovp_readback != vbusovp_word) {
      ESP_LOGW(TAG, "VBUS OVP mismatch (chip masks bits): wrote 0x%04X, read 0x%04X", vbusovp_word, vbusovp_readback);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read back VBUS OVP register");
  }
  
  // Diagnostic: Read registers 0x0B, 0x0C, 0x0D to identify potential VAC_OVP or other OVP settings
  uint16_t reg_0x0B = 0, reg_0x0C = 0, reg_0x0D = 0;
  if (this->read_register_word_(0x0B, reg_0x0B)) {
    ESP_LOGD(TAG, "Register 0x0B readback: 0x%04X", reg_0x0B);
  }
  if (this->read_register_word_(0x0C, reg_0x0C)) {
    ESP_LOGD(TAG, "Register 0x0C readback: 0x%04X", reg_0x0C);
  }
  if (this->read_register_word_(0x0D, reg_0x0D)) {
    ESP_LOGD(TAG, "Register 0x0D readback: 0x%04X", reg_0x0D);
  }
  
  // Set minimum system voltage (datasheet: 2.56V to 3.84V, step 80mV, 16-bit little-endian)
  uint16_t vsysmin_val = (uint16_t)((this->minimum_system_voltage_ - VSYSMIN_MIN) / VSYSMIN_STEP);
  vsysmin_val = (vsysmin_val << 6) & 0x0FC0;  // VSYSMIN is bits 11:6 in 16-bit register
  if (!this->write_register_word_(BQ25628E_REG_VSYSMIN_CTRL, vsysmin_val)) {
    ESP_LOGE(TAG, "Failed to set minimum system voltage");
    return false;
  }
  ESP_LOGD(TAG, "Minimum system voltage set to %.2f V", this->minimum_system_voltage_);
  
  // Set pre-charge current (datasheet: 10mA to 310mA, step 10mA, 16-bit little-endian)
  uint16_t iprechg_val = (uint16_t)((this->precharge_current_ - IPRECHG_MIN) / IPRECHG_STEP);
  iprechg_val = (iprechg_val << 3) & 0x00F8;  // IPRECHG is bits 7:3
  if (!this->write_register_word_(BQ25628E_REG_IPRECHG_CTRL, iprechg_val)) {
    ESP_LOGE(TAG, "Failed to set pre-charge current");
    return false;
  }
  ESP_LOGD(TAG, "Pre-charge current set to %.3f A", this->precharge_current_);
  
  // Set termination current (datasheet: 5mA to 310mA, step 5mA, 16-bit little-endian)
  uint16_t iterm_val = (uint16_t)((this->termination_current_ - ITERM_MIN) / ITERM_STEP);
  iterm_val = (iterm_val << 2) & 0x00FC;  // ITERM is bits 7:2
  if (!this->write_register_word_(BQ25628E_REG_ITERM_CTRL, iterm_val)) {
    ESP_LOGE(TAG, "Failed to set termination current");
    return false;
  }
  ESP_LOGD(TAG, "Termination current set to %.3f A", this->termination_current_);
  
  // Configure charge control register (REG0x14)
  uint8_t chg_ctrl = 0;
  if (this->termination_enabled_) {
    chg_ctrl |= CHG_CTRL_EN_TERM;
  }
  if (this->vindpm_battery_tracking_) {
    chg_ctrl |= CHG_CTRL_VINDPM_BAT_TRACK;
  }
  // VRECHG bit: 0 = 100mV, 1 = 200mV (use recharge_threshold_)
  if (this->recharge_threshold_ > 0.15f) {
    chg_ctrl |= CHG_CTRL_VRECHG_MASK;
  }
  if (!this->write_register_word_(BQ25628E_REG_CHG_CTRL, (uint16_t)chg_ctrl)) {
    ESP_LOGE(TAG, "Failed to configure charge control");
    return false;
  }
  ESP_LOGD(TAG, "Charge control configured: term=%d, vindpm_track=%d, vrechg=%.0fmV",
           this->termination_enabled_, this->vindpm_battery_tracking_,
           this->recharge_threshold_ * 1000);
  
  // Configure charger control 0 (REG0x16) - enable charging, set watchdog
  uint8_t charger_ctrl_0 = CHARGER_CTRL_0_EN_CHG;  // Enable charging by default
  charger_ctrl_0 |= (this->watchdog_timeout_ & 0x03);  // Bits 1:0 = watchdog timeout
  if (!this->write_register_word_(BQ25628E_REG_CHARGER_CTRL_0, (uint16_t)charger_ctrl_0)) {
    ESP_LOGE(TAG, "Failed to configure charger control 0");
    return false;
  }
  ESP_LOGD(TAG, "Charger control 0 configured: watchdog=%d", this->watchdog_timeout_);
  
  // Configure charger control 1 (REG0x17) - thermal regulation and TS monitoring
  uint8_t charger_ctrl_1 = 0;
  if (this->thermal_regulation_threshold_ > 90) {
    charger_ctrl_1 |= CHARGER_CTRL_1_TREG;  // 120C
  }
  // Disable TS monitoring if configured (to bypass thermal faults)
  if (!this->ts_monitoring_enabled_) {
    charger_ctrl_1 |= CHARGER_CTRL_1_TS_IGNORE;  // Ignore TS function
    ESP_LOGW(TAG, "TS monitoring DISABLED - thermal protection bypassed!");
  }
  // Default switching frequency and strength (bits 5:2)
  charger_ctrl_1 |= 0x0C;  // SET_CONV_STRN = 11b (strong)
  if (!this->write_register_word_(BQ25628E_REG_CHARGER_CTRL_1, (uint16_t)charger_ctrl_1)) {
    ESP_LOGE(TAG, "Failed to configure charger control 1");
    return false;
  }
  ESP_LOGD(TAG, "Thermal regulation threshold: %dC, TS monitoring: %s", 
           this->thermal_regulation_threshold_, this->ts_monitoring_enabled_ ? "enabled" : "DISABLED");
  
  // Enable ADC: bit 7 = ADC_EN, bit 6 = ADC continuous mode
  if (!this->write_register_word_(BQ25628E_REG_ADC_CTRL, 0xC0)) {
    ESP_LOGW(TAG, "Failed to enable ADC");
    return false;
  }
  
  // Enable all ADC channels (write 0x00 to disable register)
  if (!this->write_register_word_(BQ25628E_REG_ADC_DIS, 0x00)) {
    ESP_LOGW(TAG, "Failed to configure ADC channels");
    return false;
  }
  
  // Wait for ADC to be ready
  delay(50);
  
  ESP_LOGD(TAG, "ADC enabled and configured");
  
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
  
  // Read and publish die temperature - datasheet: 0.5C per LSB, -40C offset, bits 11:0
  if (this->die_temperature_sensor_ != nullptr) {
    uint16_t raw_tdie;
    if (this->read_register_word_(BQ25628E_REG_TDIE_ADC, raw_tdie)) {
      uint16_t tdie_adc = raw_tdie & 0x0FFF;  // Bits 11:0, 12-bit value
      float tdie = (tdie_adc * TDIE_ADC_STEP) - TDIE_ADC_OFFSET;
      ESP_LOGD(TAG, "TDIE raw: 0x%04X, ADC: %d, temperature: %.1f C", raw_tdie, tdie_adc, tdie);
      this->die_temperature_sensor_->publish_state(tdie);
    } else {
      ESP_LOGW(TAG, "Failed to read TDIE");
    }
  }
  
  // Read charger status register 0x1D for DPM and regulation info
  // CORRECTED per datasheet Table 8-23: bit positions were completely wrong!
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
    
    // Detect poor source lockout and warn user
    if (vbus_stat == 0) {  // VBUS_STAT = 000b = "No Power"
      ESP_LOGW(TAG, "⚠️ POOR SOURCE LOCKOUT! VBUS failed qualification (voltage drop <3.6V when loaded)");
      ESP_LOGW(TAG, "   Possible causes: Polyfuse tripped, weak supply, bad connection");
      ESP_LOGW(TAG, "   Try: Press 'Recover from Poor Source Lockout' button or replace polyfuse F1");
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
  uint16_t reg_val;
  if (!this->read_register_word_(BQ25628E_REG_CHARGER_CTRL_0, reg_val)) {
    return false;
  }
  
  // Bit 5 of REG0x16 (Charger Control 0) controls charging enable
  // 1 = enable, 0 = disable
  if (enabled) {
    reg_val |= CHARGER_CTRL_0_EN_CHG;  // Set bit 5 to enable
  } else {
    reg_val &= ~CHARGER_CTRL_0_EN_CHG;  // Clear bit 5 to disable
  }
  
  bool success = this->write_register_word_(BQ25628E_REG_CHARGER_CTRL_0, reg_val);
  if (success) {
    ESP_LOGI(TAG, "Charging %s", enabled ? "enabled" : "disabled");
  }
  return success;
}

bool BQ25628EComponent::set_charge_current(float current_amps) {
  if (current_amps < ICHG_MIN || current_amps > ICHG_MAX) {
    ESP_LOGE(TAG, "Charge current out of range: %.2f A", current_amps);
    return false;
  }
  
  uint8_t ichg_val = (uint8_t)((current_amps - ICHG_MIN) / ICHG_STEP);
  uint16_t ichg_word = ichg_val;
  bool success = this->write_register_word_(BQ25628E_REG_ICHG_CTRL, ichg_word);
  if (success) {
    this->charge_current_limit_ = current_amps;
    ESP_LOGI(TAG, "Charge current set to %.2f A", current_amps);
  }
  return success;
}

bool BQ25628EComponent::set_charge_voltage(float voltage_volts) {
  if (voltage_volts < VBAT_MIN || voltage_volts > VBAT_MAX) {
    ESP_LOGE(TAG, "Charge voltage out of range: %.2f V", voltage_volts);
    return false;
  }
  
  uint8_t vbat_val = (uint8_t)((voltage_volts - VBAT_MIN) / VBAT_STEP);
  uint16_t vbat_word = vbat_val;
  bool success = this->write_register_word_(BQ25628E_REG_VBAT_CTRL, vbat_word);
  if (success) {
    this->charge_voltage_limit_ = voltage_volts;
    ESP_LOGI(TAG, "Charge voltage set to %.2f V", voltage_volts);
  }
  return success;
}

bool BQ25628EComponent::set_input_current(float current_amps) {
  if (current_amps < IINDPM_MIN || current_amps > IINDPM_MAX) {
    ESP_LOGE(TAG, "Input current out of range: %.2f A", current_amps);
    return false;
  }
  
  uint8_t iindpm_val = (uint8_t)((current_amps - IINDPM_MIN) / IINDPM_STEP);
  uint16_t iindpm_word = iindpm_val;
  bool success = this->write_register_word_(BQ25628E_REG_IINDPM_CTRL, iindpm_word);
  if (success) {
    this->input_current_limit_ = current_amps;
    ESP_LOGI(TAG, "Input current limit set to %.2f A", current_amps);
  }
  return success;
}

bool BQ25628EComponent::set_input_voltage(float voltage_volts) {
  if (voltage_volts < VINDPM_MIN || voltage_volts > VINDPM_MAX) {
    ESP_LOGE(TAG, "Input voltage limit out of range: %.2f V", voltage_volts);
    return false;
  }
  
  uint8_t vindpm_val = (uint8_t)((voltage_volts - VINDPM_MIN) / VINDPM_STEP);
  uint16_t vindpm_word = vindpm_val;
  bool success = this->write_register_word_(BQ25628E_REG_VINDPM_CTRL, vindpm_word);
  if (success) {
    this->input_voltage_limit_ = voltage_volts;
    ESP_LOGI(TAG, "Input voltage limit set to %.2f V", voltage_volts);
  }
  return success;
}

bool BQ25628EComponent::set_hiz_mode(bool enabled) {
  uint16_t reg_val;
  if (!this->read_register_word_(BQ25628E_REG_CHARGER_CTRL_0, reg_val)) {
    return false;
  }
  
  if (enabled) {
    reg_val |= CHARGER_CTRL_0_EN_HIZ;
  } else {
    reg_val &= ~CHARGER_CTRL_0_EN_HIZ;
  }
  
  bool success = this->write_register_word_(BQ25628E_REG_CHARGER_CTRL_0, reg_val);
  if (success) {
    ESP_LOGI(TAG, "HIZ mode %s", enabled ? "enabled" : "disabled");
  }
  return success;
}

bool BQ25628EComponent::reset_watchdog() {
  uint16_t reg_val;
  if (!this->read_register_word_(BQ25628E_REG_CHARGER_CTRL_0, reg_val)) {
    return false;
  }
  
  reg_val |= CHARGER_CTRL_0_WD_RST;  // Set watchdog reset bit
  bool success = this->write_register_word_(BQ25628E_REG_CHARGER_CTRL_0, reg_val);
  if (success) {
    ESP_LOGD(TAG, "Watchdog timer reset");
  }
  return success;
}

bool BQ25628EComponent::reset_registers() {
  uint16_t reg_val;
  if (!this->read_register_word_(BQ25628E_REG_CHARGER_CTRL_1, reg_val)) {
    return false;
  }
  
  reg_val |= CHARGER_CTRL_1_REG_RST;  // Set register reset bit
  bool success = this->write_register_word_(BQ25628E_REG_CHARGER_CTRL_1, reg_val);
  if (success) {
    ESP_LOGI(TAG, "Registers reset to defaults");
    delay(10);  // Wait for reset to complete
    // Reconfigure with current settings
    this->configure_charger_();
  }
  return success;
}

uint8_t BQ25628EComponent::get_charge_status() {
  uint16_t status_reg;
  if (!this->read_register_word_(BQ25628E_REG_CHG_STATUS_1, status_reg)) {
    return 0xFF;  // Error value
  }
  
  uint8_t chg_stat = ((status_reg & 0xFF) & CHG_STATUS_1_CHG_STAT_MASK) >> 3;
  return chg_stat;
}

bool BQ25628EComponent::is_in_thermal_regulation() {
  uint16_t status_reg;
  if (!this->read_register_word_(BQ25628E_REG_CHG_STATUS_0, status_reg)) {
    return false;
  }
  return ((status_reg & 0xFF) & 0x20) != 0;  // Bit 5: TREG_STAT
}

bool BQ25628EComponent::is_in_vindpm_regulation() {
  uint16_t status_reg;
  if (!this->read_register_word_(BQ25628E_REG_CHG_STATUS_0, status_reg)) {
    return false;
  }
  return ((status_reg & 0xFF) & 0x04) != 0;  // Bit 2: VINDPM_STAT
}

bool BQ25628EComponent::is_in_iindpm_regulation() {
  uint16_t status_reg;
  if (!this->read_register_word_(BQ25628E_REG_CHG_STATUS_0, status_reg)) {
    return false;
  }
  return ((status_reg & 0xFF) & 0x08) != 0;  // Bit 3: IINDPM_STAT
}

bool BQ25628EComponent::has_fault() {
  uint16_t fault_reg;
  if (!this->read_register_word_(BQ25628E_REG_FAULT_STATUS_0, fault_reg)) {
    return false;
  }
  return (fault_reg & 0xFF) != 0;  // Any non-zero bit indicates a fault
}

bool BQ25628EComponent::write_register_word_(uint8_t reg, uint16_t value) {
  // Write little-endian (LSB first)
  uint8_t lsb = value & 0xFF;
  uint8_t msb = (value >> 8) & 0xFF;
  ESP_LOGD(TAG, "write_register_word_: reg=0x%02X, lsb=0x%02X, msb=0x%02X", reg, lsb, msb);
  if (!this->write_byte(reg, lsb)) {
    ESP_LOGE(TAG, "Failed to write LSB to register 0x%02X", reg);
    return false;
  }
  if (!this->write_byte(reg + 1, msb)) {
    ESP_LOGE(TAG, "Failed to write MSB to register 0x%02X", reg + 1);
    return false;
  }
  ESP_LOGD(TAG, "Successfully wrote 0x%04X to register 0x%02X", value, reg);
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

