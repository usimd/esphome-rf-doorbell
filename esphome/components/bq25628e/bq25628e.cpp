#include "bq25628e.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace bq25628e {

static const char *const TAG = "bq25628e";

void BQ25628EComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BQ25628E using Adafruit library...");
  
  // Create Adafruit BQ25628E instance
  this->bq_ = new Adafruit_BQ25628E();
  
  // Initialize with I2C address (note: Adafruit defaults to 0x6A, but we use 0x6B)
  Wire.begin(this->parent_->get_sda_pin(), this->parent_->get_scl_pin());
  if (!this->bq_->begin(this->address_, &Wire)) {
    ESP_LOGE(TAG, "Failed to initialize BQ25628E at address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  
  ESP_LOGD(TAG, "BQ25628E initialized successfully");
  
  // Configure the charger with initial settings
  if (!this->configure_charger_()) {
    ESP_LOGE(TAG, "Failed to configure BQ25628E");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "BQ25628E setup complete");
}

void BQ25628EComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BQ25628E (Adafruit Library):");
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
  LOG_TEXT_SENSOR("  ", "Charger Status", this->charger_status_sensor_);
  LOG_TEXT_SENSOR("  ", "Fault Status", this->fault_status_sensor_);
}

void BQ25628EComponent::update() {
  if (this->bq_ == nullptr) {
    return;
  }
  
  // Read ADC values
  if (!this->read_adc_values_()) {
    ESP_LOGW(TAG, "Failed to read ADC values");
    return;
  }
  
  // Read and publish charger status
  if (this->charger_status_sensor_ != nullptr) {
    uint8_t status = this->bq_->getChargeStatus();
    this->charger_status_sensor_->publish_state(this->get_charger_status_text_(status));
  }
  
  // Read and publish fault status
  if (this->fault_status_sensor_ != nullptr) {
    uint8_t fault = this->bq_->getFaultStatus();
    if (fault != 0) {
      this->fault_status_sensor_->publish_state(this->get_fault_status_text_(fault));
    } else {
      this->fault_status_sensor_->publish_state("None");
    }
  }
}

bool BQ25628EComponent::configure_charger_() {
  if (this->bq_ == nullptr) {
    return false;
  }
  
  // Set charge current limit
  if (!this->bq_->setChargeCurrent(this->charge_current_limit_ * 1000)) {  // Convert A to mA
    ESP_LOGE(TAG, "Failed to set charge current limit");
    return false;
  }
  ESP_LOGD(TAG, "Charge current limit set to %.2f A", this->charge_current_limit_);
  
  // Set charge voltage limit
  if (!this->bq_->setChargeVoltage(this->charge_voltage_limit_ * 1000)) {  // Convert V to mV
    ESP_LOGE(TAG, "Failed to set charge voltage limit");
    return false;
  }
  ESP_LOGD(TAG, "Charge voltage limit set to %.2f V", this->charge_voltage_limit_);
  
  // Set input current limit
  if (!this->bq_->setInputCurrentLimit(this->input_current_limit_ * 1000)) {  // Convert A to mA
    ESP_LOGE(TAG, "Failed to set input current limit");
    return false;
  }
  ESP_LOGD(TAG, "Input current limit set to %.2f A", this->input_current_limit_);
  
  // Enable ADC for continuous conversion
  this->bq_->enableADC(true);
  
  return true;
}

bool BQ25628EComponent::read_adc_values_() {
  if (this->bq_ == nullptr) {
    return false;
  }
  
  // Read and publish VBUS (bus voltage)
  if (this->bus_voltage_sensor_ != nullptr) {
    float vbus = this->bq_->getVbusVoltage() / 1000.0;  // Convert mV to V
    this->bus_voltage_sensor_->publish_state(vbus);
    ESP_LOGD(TAG, "VBUS: %.2f V", vbus);
  }
  
  // Read and publish VBAT (battery voltage)
  if (this->battery_voltage_sensor_ != nullptr) {
    float vbat = this->bq_->getBatteryVoltage() / 1000.0;  // Convert mV to V
    this->battery_voltage_sensor_->publish_state(vbat);
    ESP_LOGD(TAG, "VBAT: %.2f V", vbat);
  }
  
  // Read and publish VSYS (system voltage)
  if (this->system_voltage_sensor_ != nullptr) {
    float vsys = this->bq_->getSystemVoltage() / 1000.0;  // Convert mV to V
    this->system_voltage_sensor_->publish_state(vsys);
    ESP_LOGD(TAG, "VSYS: %.2f V", vsys);
  }
  
  // Read and publish ICHG (charge current)
  if (this->charge_current_sensor_ != nullptr) {
    float ichg = this->bq_->getChargeCurrent() / 1000.0;  // Convert mA to A
    this->charge_current_sensor_->publish_state(ichg);
    ESP_LOGD(TAG, "ICHG: %.3f A", ichg);
  }
  
  return true;
}

bool BQ25628EComponent::set_charging_enabled(bool enabled) {
  if (this->bq_ == nullptr) {
    return false;
  }
  
  bool success = this->bq_->enableCharging(enabled);
  if (success) {
    ESP_LOGI(TAG, "Charging %s", enabled ? "enabled" : "disabled");
  }
  return success;
}

bool BQ25628EComponent::set_charge_current(float current_amps) {
  if (this->bq_ == nullptr) {
    return false;
  }
  
  bool success = this->bq_->setChargeCurrent(current_amps * 1000);  // Convert A to mA
  if (success) {
    this->charge_current_limit_ = current_amps;
    ESP_LOGI(TAG, "Charge current set to %.2f A", current_amps);
  }
  return success;
}

bool BQ25628EComponent::set_charge_voltage(float voltage_volts) {
  if (this->bq_ == nullptr) {
    return false;
  }
  
  bool success = this->bq_->setChargeVoltage(voltage_volts * 1000);  // Convert V to mV
  if (success) {
    this->charge_voltage_limit_ = voltage_volts;
    ESP_LOGI(TAG, "Charge voltage set to %.2f V", voltage_volts);
  }
  return success;
}

bool BQ25628EComponent::set_input_current(float current_amps) {
  if (this->bq_ == nullptr) {
    return false;
  }
  
  bool success = this->bq_->setInputCurrentLimit(current_amps * 1000);  // Convert A to mA
  if (success) {
    this->input_current_limit_ = current_amps;
    ESP_LOGI(TAG, "Input current limit set to %.2f A", current_amps);
  }
  return success;
}

const char *BQ25628EComponent::get_charger_status_text_(uint8_t status) {
  switch (status) {
    case 0: return "Not Charging";
    case 1: return "Pre-Charge";
    case 2: return "Fast Charging";
    case 3: return "Charge Termination Done";
    default: return "Unknown";
  }
}

const char *BQ25628EComponent::get_fault_status_text_(uint8_t fault) {
  if (fault == 0) {
    return "None";
  }
  
  // Decode fault bits (simplified - Adafruit library may provide better methods)
  if (fault & 0x80) return "Watchdog Fault";
  if (fault & 0x40) return "Input Fault";
  if (fault & 0x20) return "Thermal Shutdown";
  if (fault & 0x10) return "Battery Fault";
  if (fault & 0x08) return "Safety Timer Fault";
  
  return "Fault";
}

}  // namespace bq25628e
}  // namespace esphome
