#include "max17260.h"
#include "esphome/core/log.h"

namespace esphome {
namespace max17260 {

static const char *const TAG = "max17260";

void MAX17260Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX17260...");
  
  // Verify device is responding
  uint16_t status;
  if (!this->read_register_word_(MAX17260_REG_STATUS, status)) {
    ESP_LOGE(TAG, "Failed to communicate with MAX17260");
    this->mark_failed();
    return;
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
  LOG_SENSOR("  ", "State of Charge", this->soc_sensor_);
  LOG_SENSOR("  ", "Time to Empty", this->tte_sensor_);
  LOG_SENSOR("  ", "Time to Full", this->ttf_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void MAX17260Component::update() {
  // Read and publish voltage (datasheet: 78.125µV per LSB)
  if (this->voltage_sensor_ != nullptr) {
    uint16_t raw_vcell;
    if (this->read_register_word_(MAX17260_REG_AVGVCELL, raw_vcell)) {
      float voltage = raw_vcell * VCELL_SCALE;
      this->voltage_sensor_->publish_state(voltage);
      ESP_LOGD(TAG, "Voltage: %.3f V", voltage);
    }
  }
  
  // Read and publish current (datasheet: depends on sense resistor, assuming 5mΩ)
  if (this->current_sensor_ != nullptr) {
    uint16_t raw_current;
    if (this->read_register_word_(MAX17260_REG_AVGCURRENT, raw_current)) {
      // Signed 16-bit value
      int16_t signed_current = (int16_t)raw_current;
      float current = signed_current * CURRENT_SCALE_5mOhm;
      this->current_sensor_->publish_state(current);
      ESP_LOGD(TAG, "Current: %.3f A", current);
    }
  }
  
  // Read and publish state of charge (datasheet: 1% per LSB, use upper byte)
  if (this->soc_sensor_ != nullptr) {
    uint16_t raw_soc;
    if (this->read_register_word_(MAX17260_REG_REPSOC, raw_soc)) {
      float soc = (raw_soc >> 8) * PERCENT_SCALE;  // Upper byte only
      this->soc_sensor_->publish_state(soc);
      ESP_LOGD(TAG, "SOC: %.1f %%", soc);
    }
  }
  
  // Read and publish time to empty (datasheet: 5.625s per LSB)
  if (this->tte_sensor_ != nullptr) {
    uint16_t raw_tte;
    if (this->read_register_word_(MAX17260_REG_TTE, raw_tte)) {
      float tte_minutes = (raw_tte * TIME_SCALE) / 60.0f;
      this->tte_sensor_->publish_state(tte_minutes);
      ESP_LOGD(TAG, "TTE: %.1f min", tte_minutes);
    }
  }
  
  // Read and publish time to full (datasheet: 5.625s per LSB)
  if (this->ttf_sensor_ != nullptr) {
    uint16_t raw_ttf;
    if (this->read_register_word_(MAX17260_REG_TTF, raw_ttf)) {
      float ttf_minutes = (raw_ttf * TIME_SCALE) / 60.0f;
      this->ttf_sensor_->publish_state(ttf_minutes);
      ESP_LOGD(TAG, "TTF: %.1f min", ttf_minutes);
    }
  }
  
  // Read and publish temperature (datasheet: 1/256°C per LSB)
  if (this->temperature_sensor_ != nullptr) {
    uint16_t raw_temp;
    if (this->read_register_word_(MAX17260_REG_TEMP, raw_temp)) {
      // Signed 16-bit value
      int16_t signed_temp = (int16_t)raw_temp;
      float temperature = signed_temp * TEMP_SCALE;
      this->temperature_sensor_->publish_state(temperature);
      ESP_LOGD(TAG, "Temperature: %.1f °C", temperature);
    }
  }
}

bool MAX17260Component::read_register_word_(uint8_t reg, uint16_t &value) {
  uint8_t lsb, msb;
  if (!this->read_byte(reg, &lsb) || !this->read_byte(reg + 1, &msb)) {
    ESP_LOGW(TAG, "Failed to read 16-bit register 0x%02X", reg);
    return false;
  }
  // LSB first (little-endian)
  value = (msb << 8) | lsb;
  return true;
}

}  // namespace max17260
}  // namespace esphome
