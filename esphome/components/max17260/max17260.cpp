#include "max17260.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

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
    
    // Wait for model to load
    if (!this->wait_for_dnr_clear_()) {
      ESP_LOGW(TAG, "Model loading timeout - continuing anyway");
    }
    
    // Store original HibCFG value
    uint16_t hib_cfg;
    if (!this->read_register_word_(MAX17260_REG_HIBCFG, hib_cfg)) {
      ESP_LOGE(TAG, "Failed to read HibCFG");
      this->mark_failed();
      return;
    }
    
    // Exit hibernate mode to enable writes
    this->exit_hibernate_mode_();
    
    // Write design capacity (330mAh with 25mΩ sense resistor)
    if (!this->write_register_word_(MAX17260_REG_DESIGNCAP, DESIGN_CAP)) {
      ESP_LOGE(TAG, "Failed to write DesignCap");
    } else {
      ESP_LOGI(TAG, "DesignCap set to %d (330mAh)", DESIGN_CAP);
    }
    
    // Write IChgTerm (charge termination current: 20mA)
    if (!this->write_register_word_(MAX17260_REG_ICHGTERM, ICHG_TERM)) {
      ESP_LOGE(TAG, "Failed to write IChgTerm");
    } else {
      ESP_LOGI(TAG, "IChgTerm set to %d (20mA)", ICHG_TERM);
    }
    
    // Write VEmpty (empty voltage)
    if (!this->write_register_word_(MAX17260_REG_VEMPTY, VEMPTY)) {
      ESP_LOGE(TAG, "Failed to write VEmpty");
    } else {
      ESP_LOGI(TAG, "VEmpty set to 0x%04X", VEMPTY);
    }
    
    // Write ModelCFG to refresh model (Refresh=1, R100=4 for 330mAh)
    if (!this->write_register_word_(MAX17260_REG_MODELCFG, 0x8400)) {
      ESP_LOGE(TAG, "Failed to write ModelCFG");
    } else {
      ESP_LOGI(TAG, "ModelCFG refresh initiated");
    }
    
    // Wait for model refresh to complete (up to 710ms)
    int timeout = 100;
    uint16_t model_cfg;
    do {
      delay(10);
      if (!this->read_register_word_(MAX17260_REG_MODELCFG, model_cfg)) {
        break;
      }
    } while ((model_cfg & 0x8000) && timeout-- > 0);
    
    if (timeout > 0) {
      ESP_LOGI(TAG, "Model refresh complete");
    } else {
      ESP_LOGW(TAG, "Model refresh timeout");
    }
    
    // Restore original HibCFG
    this->restore_hibernate_mode_(hib_cfg);
    
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
  
  ESP_LOGCONFIG(TAG, "  Battery Configuration:");
  ESP_LOGCONFIG(TAG, "    Design Capacity: 330 mAh");
  ESP_LOGCONFIG(TAG, "    Sense Resistor: 25 mΩ");
  ESP_LOGCONFIG(TAG, "    Charge Termination: 20 mA");
  
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
  LOG_SENSOR("  ", "State of Charge", this->soc_sensor_);
  LOG_SENSOR("  ", "Remaining Capacity", this->remaining_capacity_sensor_);
  LOG_SENSOR("  ", "Full Capacity", this->full_capacity_sensor_);
  LOG_SENSOR("  ", "Cycle Count", this->cycle_count_sensor_);
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
      ESP_LOGD(TAG, "Voltage: %.3f V (raw 0x%04X)", voltage, raw_vcell);
    }
  }
  
  // Read and publish current (datasheet: 1.5625μV/RSENSE per LSB, RSENSE=25mΩ)
  if (this->current_sensor_ != nullptr) {
    uint16_t raw_current;
    if (this->read_register_word_(MAX17260_REG_AVGCURRENT, raw_current)) {
      // Signed 16-bit value: 1.5625μV / 25mΩ = 0.0625mA = 0.0000625A per LSB
      int16_t signed_current = static_cast<int16_t>(raw_current);
      float current = signed_current * CURRENT_SCALE;
      this->current_sensor_->publish_state(current);
      ESP_LOGD(TAG, "Current: %.4f A (raw 0x%04X = %d)", current, raw_current, signed_current);
    }
  }
  
  // Read and publish state of charge (datasheet: 1/256% per LSB, full 16-bit resolution)
  if (this->soc_sensor_ != nullptr) {
    uint16_t raw_soc;
    if (this->read_register_word_(MAX17260_REG_REPSOC, raw_soc)) {
      // Full 16-bit resolution: 1/256% per LSB
      float soc = raw_soc * PERCENT_SCALE;
      ESP_LOGD(TAG, "SOC: %.2f %% (raw 0x%04X)", soc, raw_soc);
      this->soc_sensor_->publish_state(soc);
    }
  }
  
  // Read and publish remaining capacity (mAh)
  if (this->remaining_capacity_sensor_ != nullptr) {
    uint16_t raw_repcap;
    if (this->read_register_word_(MAX17260_REG_REPCAP, raw_repcap)) {
      float capacity = raw_repcap * CAPACITY_SCALE;  // 5μVh/RSENSE, RSENSE=25mΩ -> 0.2mAh/LSB
      this->remaining_capacity_sensor_->publish_state(capacity);
      ESP_LOGD(TAG, "Remaining Capacity: %.1f mAh (raw 0x%04X)", capacity, raw_repcap);
    }
  }
  
  // Read and publish full capacity (mAh)
  if (this->full_capacity_sensor_ != nullptr) {
    uint16_t raw_fullcap;
    if (this->read_register_word_(MAX17260_REG_FULLCAPREP, raw_fullcap)) {
      float fullcap = raw_fullcap * CAPACITY_SCALE;
      this->full_capacity_sensor_->publish_state(fullcap);
      ESP_LOGD(TAG, "Full Capacity: %.1f mAh (raw 0x%04X)", fullcap, raw_fullcap);
    }
  }
  
  // Read and publish cycle count
  if (this->cycle_count_sensor_ != nullptr) {
    uint16_t raw_cycles;
    if (this->read_register_word_(MAX17260_REG_CYCLES, raw_cycles)) {
      float cycles = raw_cycles * CYCLES_SCALE;  // 1% cycle per LSB
      this->cycle_count_sensor_->publish_state(cycles);
      ESP_LOGD(TAG, "Cycle Count: %.2f cycles (raw 0x%04X)", cycles, raw_cycles);
    }
  }
  
  // Read and publish time to empty (datasheet: 5.625s per LSB)
  if (this->tte_sensor_ != nullptr) {
    uint16_t raw_tte;
    if (this->read_register_word_(MAX17260_REG_TTE, raw_tte)) {
      // 0xFFFF = no estimate available
      if (raw_tte == 0xFFFF) {
        ESP_LOGD(TAG, "TTE: Not available (not discharging)");
      } else {
        float tte_minutes = (raw_tte * TIME_SCALE) / 60.0f;
        this->tte_sensor_->publish_state(tte_minutes);
        ESP_LOGD(TAG, "TTE: %.1f min (raw 0x%04X)", tte_minutes, raw_tte);
      }
    }
  }
  
  // Read and publish time to full (datasheet: 5.625s per LSB)
  if (this->ttf_sensor_ != nullptr) {
    uint16_t raw_ttf;
    if (this->read_register_word_(MAX17260_REG_TTF, raw_ttf)) {
      // 0xFFFF = no estimate available
      if (raw_ttf == 0xFFFF) {
        ESP_LOGD(TAG, "TTF: Not available (not charging)");
      } else {
        float ttf_minutes = (raw_ttf * TIME_SCALE) / 60.0f;
        this->ttf_sensor_->publish_state(ttf_minutes);
        ESP_LOGD(TAG, "TTF: %.1f min (raw 0x%04X)", ttf_minutes, raw_ttf);
      }
    }
  }
  
  // Read and publish temperature (datasheet: 1/256°C per LSB, signed)
  if (this->temperature_sensor_ != nullptr) {
    uint16_t raw_temp;
    if (this->read_register_word_(MAX17260_REG_TEMP, raw_temp)) {
      // Signed 16-bit value: 1/256°C per LSB
      int16_t signed_temp = static_cast<int16_t>(raw_temp);
      float temperature = signed_temp * TEMP_SCALE;
      this->temperature_sensor_->publish_state(temperature);
      ESP_LOGD(TAG, "Temperature: %.1f °C (raw 0x%04X = %d)", temperature, raw_temp, signed_temp);
    }
  }
}

bool MAX17260Component::read_register_word_(uint8_t reg, uint16_t &value) {
  // MAX17260 does NOT auto-increment register address!
  // Must read 16-bit word in single I2C transaction
  uint8_t data[2];
  if (!this->read_bytes(reg, data, 2)) {
    ESP_LOGW(TAG, "Failed to read 16-bit register 0x%02X", reg);
    return false;
  }
  // Little-endian: LSB at data[0], MSB at data[1]
  value = (static_cast<uint16_t>(data[1]) << 8) | data[0];
  return true;
}

bool MAX17260Component::write_register_word_(uint8_t reg, uint16_t value) {
  // Write 16-bit word in single I2C transaction (little-endian)
  uint8_t data[2] = {
    static_cast<uint8_t>(value & 0xFF),        // LSB
    static_cast<uint8_t>((value >> 8) & 0xFF)  // MSB
  };
  if (!this->write_bytes(reg, data, 2)) {
    ESP_LOGW(TAG, "Failed to write 16-bit register 0x%02X", reg);
    return false;
  }
  return true;
}

bool MAX17260Component::wait_for_dnr_clear_() {
  // Wait for FSTAT.DNR to clear (model loading complete)
  uint16_t fstat;
  int timeout = 100;
  do {
    delay(10);
    if (!this->read_register_word_(MAX17260_REG_FSTAT, fstat)) {
      ESP_LOGE(TAG, "Failed to read FSTAT");
      return false;
    }
  } while ((fstat & 0x0001) && timeout-- > 0);
  
  if (timeout <= 0) {
    ESP_LOGW(TAG, "Timeout waiting for DNR clear");
    return false;
  }
  return true;
}

bool MAX17260Component::exit_hibernate_mode_() {
  // Exit hibernate mode per datasheet section 3.4
  this->write_register_word_(MAX17260_REG_SOFTWAKEUP, 0x90);
  this->write_register_word_(MAX17260_REG_HIBCFG, 0x0000);
  this->write_register_word_(MAX17260_REG_SOFTWAKEUP, 0x0000);
  return true;
}

bool MAX17260Component::restore_hibernate_mode_(uint16_t hib_cfg) {
  return this->write_register_word_(MAX17260_REG_HIBCFG, hib_cfg);
}

}  // namespace max17260
}  // namespace esphome
