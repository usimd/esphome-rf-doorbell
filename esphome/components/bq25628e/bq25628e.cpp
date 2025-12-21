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
  
  LOG_SENSOR("  ", "Bus Voltage", this->bus_voltage_sensor_);
  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_sensor_);
  LOG_SENSOR("  ", "Charge Current", this->charge_current_sensor_);
  LOG_SENSOR("  ", "System Voltage", this->system_voltage_sensor_);
}

void BQ25628EComponent::update() {
  // Read ADC values
  if (!this->read_adc_values_()) {
    ESP_LOGW(TAG, "Failed to read ADC values");
    return;
  }
}

bool BQ25628EComponent::configure_charger_() {
  // Set charge current limit (datasheet: 50mA to 3000mA, step 50mA)
  uint8_t ichg_val = (uint8_t)((this->charge_current_limit_ - ICHG_MIN) / ICHG_STEP);
  if (!this->write_register_byte_(BQ25628E_REG_ICHG_CTRL, ichg_val)) {
    ESP_LOGE(TAG, "Failed to set charge current limit");
    return false;
  }
  ESP_LOGD(TAG, "Charge current limit set to %.2f A", this->charge_current_limit_);
  
  // Set charge voltage limit (datasheet: 3.84V to 4.624V, step 8mV)
  uint8_t vbat_val = (uint8_t)((this->charge_voltage_limit_ - VBAT_MIN) / VBAT_STEP);
  if (!this->write_register_byte_(BQ25628E_REG_VBAT_CTRL, vbat_val)) {
    ESP_LOGE(TAG, "Failed to set charge voltage limit");
    return false;
  }
  ESP_LOGD(TAG, "Charge voltage limit set to %.2f V", this->charge_voltage_limit_);
  
  // Set input current limit (datasheet: 100mA to 3200mA, step 100mA)
  uint8_t iindpm_val = (uint8_t)((this->input_current_limit_ - IINDPM_MIN) / IINDPM_STEP);
  if (!this->write_register_byte_(BQ25628E_REG_IINDPM_CTRL, iindpm_val)) {
    ESP_LOGE(TAG, "Failed to set input current limit");
    return false;
  }
  ESP_LOGD(TAG, "Input current limit set to %.2f A", this->input_current_limit_);
  
  // Enable ADC: bit 7 = ADC_EN, bit 6 = ADC continuous mode
  if (!this->write_register_byte_(BQ25628E_REG_ADC_CTRL, 0xC0)) {
    ESP_LOGW(TAG, "Failed to enable ADC");
    return false;
  }
  
  // Enable all ADC channels (write 0x00 to disable register)
  if (!this->write_register_byte_(BQ25628E_REG_ADC_DIS, 0x00)) {
    ESP_LOGW(TAG, "Failed to configure ADC channels");
    return false;
  }
  
  // Wait for ADC to be ready
  delay(50);
  
  ESP_LOGD(TAG, "ADC enabled and configured");
  
  return true;
}

bool BQ25628EComponent::read_adc_values_() {
  // Read and publish VBUS (bus voltage) - datasheet: 1mV per LSB, 16-bit
  if (this->bus_voltage_sensor_ != nullptr) {
    uint16_t raw_vbus;
    if (this->read_register_word_(BQ25628E_REG_VBUS_ADC, raw_vbus)) {
      float vbus = raw_vbus * VBUS_ADC_STEP;
      this->bus_voltage_sensor_->publish_state(vbus);
      ESP_LOGD(TAG, "VBUS: %.2f V", vbus);
    }
  }
  
  // Read and publish VBAT (battery voltage) - datasheet: 1mV per LSB, 16-bit
  if (this->battery_voltage_sensor_ != nullptr) {
    uint16_t raw_vbat;
    if (this->read_register_word_(BQ25628E_REG_VBAT_ADC, raw_vbat)) {
      float vbat = raw_vbat * VBAT_ADC_STEP;
      this->battery_voltage_sensor_->publish_state(vbat);
      ESP_LOGD(TAG, "VBAT: %.2f V", vbat);
    }
  }
  
  // Read and publish VSYS (system voltage) - datasheet: 1mV per LSB, 16-bit
  if (this->system_voltage_sensor_ != nullptr) {
    uint16_t raw_vsys;
    if (this->read_register_word_(BQ25628E_REG_VSYS_ADC, raw_vsys)) {
      float vsys = raw_vsys * VSYS_ADC_STEP;
      this->system_voltage_sensor_->publish_state(vsys);
      ESP_LOGD(TAG, "VSYS: %.2f V", vsys);
    }
  }
  
  // Read and publish IBAT (charge current) - datasheet: 1mA per LSB, 16-bit, signed
  if (this->charge_current_sensor_ != nullptr) {
    uint16_t raw_ibat;
    if (this->read_register_word_(BQ25628E_REG_IBAT_ADC, raw_ibat)) {
      // Signed 16-bit value
      int16_t signed_ibat = (int16_t)raw_ibat;
      float ibat = signed_ibat * IBAT_ADC_STEP;
      this->charge_current_sensor_->publish_state(ibat);
      ESP_LOGD(TAG, "IBAT: %.3f A", ibat);
    }
  }
  
  return true;
}

bool BQ25628EComponent::set_charging_enabled(bool enabled) {
  uint8_t reg_val;
  if (!this->read_register_byte_(BQ25628E_REG_CHG_CTRL, reg_val)) {
    return false;
  }
  
  // Bit 5 of CHG_CTRL register controls charging enable (datasheet: 1 = disable, 0 = enable)
  if (enabled) {
    reg_val &= ~(1 << 5);  // Clear bit 5 to enable
  } else {
    reg_val |= (1 << 5);   // Set bit 5 to disable
  }
  
  bool success = this->write_register_byte_(BQ25628E_REG_CHG_CTRL, reg_val);
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
  bool success = this->write_register_byte_(BQ25628E_REG_ICHG_CTRL, ichg_val);
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
  bool success = this->write_register_byte_(BQ25628E_REG_VBAT_CTRL, vbat_val);
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
  bool success = this->write_register_byte_(BQ25628E_REG_IINDPM_CTRL, iindpm_val);
  if (success) {
    this->input_current_limit_ = current_amps;
    ESP_LOGI(TAG, "Input current limit set to %.2f A", current_amps);
  }
  return success;
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
  uint8_t lsb, msb;
  if (!this->read_byte(reg, &lsb) || !this->read_byte(reg + 1, &msb)) {
    ESP_LOGW(TAG, "Failed to read 16-bit register 0x%02X", reg);
    return false;
  }
  // LSB first (little-endian per datasheet)
  value = (msb << 8) | lsb;
  return true;
}

}  // namespace bq25628e
}  // namespace esphome

