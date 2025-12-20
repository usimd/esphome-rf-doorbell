#include "max17260.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace max17260 {

static const char *const TAG = "max17260";

void MAX17260Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX17260...");
  
  // Verify device presence by reading STATUS register
  uint16_t status;
  if (!this->read_register_16_(MAX17260_REG_STATUS, &status)) {
    ESP_LOGE(TAG, "Failed to communicate with MAX17260");
    this->mark_failed();
    return;
  }
  
  ESP_LOGD(TAG, "Status: 0x%04X", status);
  
  // Check if POR (Power-On Reset) bit is set
  if (status & 0x0002) {
    ESP_LOGI(TAG, "Power-On Reset detected, initializing fuel gauge");
    
    // Clear POR bit
    if (!this->write_register_16_(MAX17260_REG_STATUS, status & ~0x0002)) {
      ESP_LOGW(TAG, "Failed to clear POR bit");
    }
    
    // Note: Full initialization would require battery capacity configuration
    // For now, we assume the fuel gauge retains its configuration or uses defaults
  }
  
  ESP_LOGCONFIG(TAG, "MAX17260 setup complete");
}

void MAX17260Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX17260:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX17260 failed!");
  }
  
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "State of Charge", this->state_of_charge_sensor_);
  LOG_SENSOR("  ", "Time to Empty", this->time_to_empty_sensor_);
  LOG_SENSOR("  ", "Time to Full", this->time_to_full_sensor_);
}

void MAX17260Component::update() {
  uint16_t raw_value;
  
  // Read and publish battery voltage (VCELL register)
  if (this->read_register_16_(MAX17260_REG_VCELL, &raw_value)) {
    if (this->voltage_sensor_ != nullptr) {
      // VCELL LSB = 78.125 μV
      float voltage = (raw_value * 78.125) / 1000000.0;
      this->voltage_sensor_->publish_state(voltage);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read voltage");
  }
  
  // Read and publish state of charge (RepSOC register)
  if (this->read_register_16_(MAX17260_REG_REPSOC, &raw_value)) {
    if (this->state_of_charge_sensor_ != nullptr) {
      // RepSOC LSB = 1/256 %
      float soc = raw_value / 256.0;
      this->state_of_charge_sensor_->publish_state(soc);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read state of charge");
  }
  
  // Read and publish current (Current register)
  if (this->read_register_16_(MAX17260_REG_CURRENT, &raw_value)) {
    if (this->current_sensor_ != nullptr) {
      // Current is a signed 16-bit value
      // LSB = 156.25 μV / R_sense (typically 10mΩ = 0.01Ω)
      // Current [A] = raw_value * 156.25μV / 10mΩ = raw_value * 0.015625 mA
      int16_t signed_current = (int16_t)raw_value;
      float current = (signed_current * 0.15625) / 1000.0;  // Convert to Amps
      this->current_sensor_->publish_state(current);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read current");
  }
  
  // Read and publish temperature (Temp register)
  if (this->read_register_16_(MAX17260_REG_TEMP, &raw_value)) {
    if (this->temperature_sensor_ != nullptr) {
      // Temperature LSB = 1/256 °C
      int16_t signed_temp = (int16_t)raw_value;
      float temperature = signed_temp / 256.0;
      this->temperature_sensor_->publish_state(temperature);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read temperature");
  }
  
  // Read and publish time to empty (TTE register)
  if (this->read_register_16_(MAX17260_REG_TTE, &raw_value)) {
    if (this->time_to_empty_sensor_ != nullptr) {
      // TTE LSB = 5.625 seconds
      float tte_minutes = (raw_value * 5.625) / 60.0;
      this->time_to_empty_sensor_->publish_state(tte_minutes);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read time to empty");
  }
  
  // Read and publish time to full (TTF register)
  if (this->read_register_16_(MAX17260_REG_TTF, &raw_value)) {
    if (this->time_to_full_sensor_ != nullptr) {
      // TTF LSB = 5.625 seconds
      float ttf_minutes = (raw_value * 5.625) / 60.0;
      this->time_to_full_sensor_->publish_state(ttf_minutes);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read time to full");
  }
}

bool MAX17260Component::read_register_16_(uint8_t reg, uint16_t *value) {
  uint8_t data[2];
  if (!this->read_bytes(reg, data, 2)) {
    return false;
  }
  
  // MAX17260 uses little-endian byte order
  *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  return true;
}

bool MAX17260Component::write_register_16_(uint8_t reg, uint16_t value) {
  uint8_t data[2];
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
  return this->write_bytes(reg, data, 2);
}

}  // namespace max17260
}  // namespace esphome
