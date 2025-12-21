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
  
  // Check if initialization is needed (bit 1 of Status = POR)
  if (status & 0x0002) {
    ESP_LOGI(TAG, "Power-on reset detected, initializing fuel gauge...");
    
    // Wait for FSTAT.DNR to clear (model loading complete)
    uint16_t fstat;
    int timeout = 100;
    do {
      delay(10);
      if (!this->read_register_word_(0x3D, fstat)) {  // FSTAT register
        ESP_LOGE(TAG, "Failed to read FSTAT");
        break;
      }
    } while ((fstat & 0x0001) && timeout-- > 0);
    
    if (timeout <= 0) {
      ESP_LOGW(TAG, "Timeout waiting for model load");
    }
    
    // Store original HibCFG value
    uint16_t hib_cfg;
    if (!this->read_register_word_(0xBA, hib_cfg)) {  // HibCFG
      ESP_LOGE(TAG, "Failed to read HibCFG");
      this->mark_failed();
      return;
    }
    
    // Exit hibernate mode to enable writes
    this->write_register_word_(0x60, 0x90);  // Soft-wakeup
    this->write_register_word_(0xBA, 0x0000);  // Clear HibCFG
    this->write_register_word_(0x60, 0x0000);  // Clear soft-wakeup
    
    // Write design capacity (330mAh with 25mΩ sense resistor)
    if (!this->write_register_word_(0x18, DESIGN_CAP)) {
      ESP_LOGE(TAG, "Failed to write DesignCap");
    } else {
      ESP_LOGI(TAG, "DesignCap set to %d (330mAh)", DESIGN_CAP);
    }
    
    // Write IChgTerm (charge termination current: 20mA)
    if (!this->write_register_word_(0x1E, ICHG_TERM)) {
      ESP_LOGE(TAG, "Failed to write IChgTerm");
    } else {
      ESP_LOGI(TAG, "IChgTerm set to %d (20mA)", ICHG_TERM);
    }
    
    // Write VEmpty (empty voltage)
    if (!this->write_register_word_(0x3A, VEMPTY)) {
      ESP_LOGE(TAG, "Failed to write VEmpty");
    } else {
      ESP_LOGI(TAG, "VEmpty set to 0x%04X", VEMPTY);
    }
    
    // Write ModelCFG to refresh model
    if (!this->write_register_word_(0xDB, 0x8400)) {  // Refresh=1, R100=4 (for 330mAh)
      ESP_LOGE(TAG, "Failed to write ModelCFG");
    } else {
      ESP_LOGI(TAG, "ModelCFG refresh initiated");
    }
    
    // Wait for model refresh to complete (up to 710ms)
    timeout = 100;
    uint16_t model_cfg;
    do {
      delay(10);
      if (!this->read_register_word_(0xDB, model_cfg)) {
        break;
      }
    } while ((model_cfg & 0x8000) && timeout-- > 0);
    
    if (timeout > 0) {
      ESP_LOGI(TAG, "Model refresh complete");
    } else {
      ESP_LOGW(TAG, "Model refresh timeout");
    }
    
    // Restore original HibCFG
    this->write_register_word_(0xBA, hib_cfg);
    
    // Clear POR bit
    this->write_register_word_(MAX17260_REG_STATUS, status & 0xFFFD);
    
    ESP_LOGI(TAG, "MAX17260 initialization complete");
  } else {
    ESP_LOGI(TAG, "MAX17260 already initialized");
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
  
  // Read and publish current (datasheet: depends on sense resistor, 25mΩ)
  if (this->current_sensor_ != nullptr) {
    uint16_t raw_current;
    if (this->read_register_word_(MAX17260_REG_AVGCURRENT, raw_current)) {
      // Signed 16-bit value
      int16_t signed_current = (int16_t)raw_current;
      float current = signed_current * CURRENT_SCALE_25mOhm;
      this->current_sensor_->publish_state(current);
      ESP_LOGD(TAG, "Current: %.3f A", current);
    }
  }
  
  // Read and publish state of charge (datasheet: 1% per LSB, use upper byte)
  if (this->soc_sensor_ != nullptr) {
    uint16_t raw_soc;
    if (this->read_register_word_(MAX17260_REG_REPSOC, raw_soc)) {
      float soc = (raw_soc >> 8) * PERCENT_SCALE;  // Upper byte only
      ESP_LOGD(TAG, "SOC raw: 0x%04X, upper byte: %d, value: %.1f %%", raw_soc, (raw_soc >> 8), soc);
      this->soc_sensor_->publish_state(soc);
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
      ESP_LOGD(TAG, "Temperature raw: 0x%04X, signed: %d, value: %.1f °C", raw_temp, signed_temp, temperature);
      this->temperature_sensor_->publish_state(temperature);
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

bool MAX17260Component::write_register_word_(uint8_t reg, uint16_t value) {
  uint8_t lsb = value & 0xFF;
  uint8_t msb = (value >> 8) & 0xFF;
  if (!this->write_byte(reg, lsb) || !this->write_byte(reg + 1, msb)) {
    ESP_LOGW(TAG, "Failed to write 16-bit register 0x%02X", reg);
    return false;
  }
  return true;
}

}  // namespace max17260
}  // namespace esphome
