/*
 * MAX17260 Fuel Gauge Component for ESPHome
 * 
 * This implementation follows the Maxim Integrated application notes:
 * - UG6595: "ModelGauge m5 Host Side Software Implementation Guide"
 * - UG6597: "MAX1726x ModelGauge m5 EZ User Guide"
 * 
 * INITIALIZATION SEQUENCE (per UG6595 Section 2):
 * ================================================
 * Step 0: Check POR bit (Status register bit 1)
 *         - If POR == 0, device already initialized, skip to monitoring
 *         - If POR == 1, proceed with full initialization
 * 
 * Step 1: Wait for DNR (Data Not Ready) to clear
 *         - Poll FSTAT.DNR (register 0x3D, bit 0) every 10ms
 *         - Must be 0 before proceeding with configuration
 * 
 * Step 2: Initialize fuel gauge
 *   2.0: Save original HibCFG register value (0xBA)
 *   2.1: Exit hibernate mode (enable register writes)
 *        - Write SoftWakeup (0x60) = 0x90
 *        - Write HibCFG (0xBA) = 0x0
 *        - Write SoftWakeup (0x60) = 0x0
 *   2.2: EZ Config - Write battery characterization parameters
 *        - DesignCap (0x18): Battery design capacity in register units
 *        - IChgTerm (0x1E): Charge termination current
 *        - VEmpty (0x3A): Empty voltage thresholds
 *        - ModelCFG (0xDB): Model configuration with Refresh bit
 *          * Bit 15 (Refresh) = 1 to initiate model loading
 *          * Bit 10 (VChg) = 1 if charge voltage > 4.275V, else 0
 *   2.3: Wait for model refresh to complete
 *        - Poll ModelCFG bit 15 every 10ms (up to 710ms)
 *        - Refresh bit will clear when model loading is complete
 *   2.4: Restore original HibCFG value
 * 
 * Step 3: Clear POR bit
 *         - Use WriteAndVerifyRegister to ensure POR bit is reliably cleared
 *         - Critical: Simple write is insufficient, verification required
 * 
 * MONITORING LOOP (per UG6595 Section 3):
 * =======================================
 * Step 3.1: Check POR bit periodically during update()
 *           - If POR bit is set, re-run full initialization sequence
 *           - Detects unexpected power cycles or resets
 * 
 * Step 3.2: Read RepCap and RepSOC for normal operation
 * 
 * Step 3.3: Read TTE (Time to Empty) register if needed
 * 
 * Step 3.4: Save learned parameters every 64% cycle change
 *           - RCOMP0 (0x38): Internal resistance compensation
 *           - TempCo (0x39): Temperature compensation
 *           - FullCapRep (0x10): Full capacity (reported)
 *           - Cycles (0x17): Cycle count
 *           - FullCapNom (0x23): Full capacity (nominal)
 *           - TODO: Requires non-volatile storage (ESPHome preferences)
 * 
 * REGISTER UNITS (per UG6597 Tables 3-4):
 * =======================================
 * Capacity: 5.0μVh / RSENSE per LSB (with 25mΩ: 0.2mAh per LSB)
 * SOC: 1/256% per LSB (full 16-bit resolution)
 * Voltage: 1.25mV/16 = 0.078125mV per LSB
 * Current: 1.5625μV / RSENSE per LSB (with 25mΩ: 62.5μA per LSB)
 * Temperature: 1/256°C per LSB (signed)
 * Time: 5.625s per LSB
 * Cycles: 1% per LSB (0-655.35 range)
 * 
 * CRITICAL IMPLEMENTATION DETAILS:
 * ================================
 * - WriteAndVerifyRegister required for POR bit clearing (3 retries, 1ms delays)
 * - DNR wait is mandatory before any configuration writes
 * - HibCFG must be saved/restored during initialization
 * - ModelCFG refresh can take up to 710ms - must wait for completion
 * - ModelCFG.VChg bit depends on charge voltage (>4.275V = 1, else 0)
 * - Periodic POR monitoring prevents stale data after unexpected resets
 * - All register values are little-endian (LSB at lower address)
 * 
 * BATTERY CONFIGURATION:
 * =====================
 * - Design Capacity: 330mAh
 * - Sense Resistor: 25mΩ
 * - Charge Termination: 20mA
 * - Empty Voltage: 3.3V (recovery at 3.88V)
 * - Charge Voltage: 4.2V (standard Li-ion)
 * 
 * Author: ESPHome MAX17260 Component
 * References: UG6595 Rev 3, UG6597 Rev 7
 */

#include "max17260.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace max17260 {

static const char *const TAG = "max17260";

void MAX17260Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX17260...");
  
  // === STEP 0: Check POR bit (UG6595 Section 2, Step 0) ===
  if (!this->check_por_bit_()) {
    // Already initialized, skip initialization
    ESP_LOGI(TAG, "MAX17260 already initialized (POR bit clear)");
    this->initialized_ = true;
    return;
  }
  
  ESP_LOGI(TAG, "Power-on reset detected, performing full initialization per UG6595...");
  
  // === STEP 1: Wait for DNR to clear (UG6595 Section 2, Step 1) ===
  // Must wait while FSTAT.DNR == 1 before proceeding with initialization
  if (!this->wait_for_dnr_clear_()) {
    ESP_LOGE(TAG, "DNR wait timeout - initialization failed");
    this->mark_failed();
    return;
  }
  
  // === STEP 2: Initialize (UG6595 Section 2, Step 2) ===
  
  // Step 2.0: Save original HibCFG value
  uint16_t saved_hib_cfg;
  if (!this->exit_hibernate_mode_(saved_hib_cfg)) {
    ESP_LOGE(TAG, "Failed to exit hibernate mode");
    this->mark_failed();
    return;
  }
  
  // Step 2.1: EZ Config - Write battery characterization parameters
  // Determine charge voltage for ModelCFG.VChg bit (UG6595 Section 2.1)
  // Assuming 4.2V charge voltage (typical for Li-ion), use 0x8000
  // If charge voltage > 4.275V, use 0x8400
  float charge_voltage = 4.2f;  // TODO: Make this configurable
  if (!this->perform_ez_config_(charge_voltage)) {
    ESP_LOGE(TAG, "EZ Config failed");
    this->mark_failed();
    return;
  }
  
  // Step 2.2: Wait for model refresh to complete (UG6595 Section 2.1)
  // ModelCFG.Refresh bit must clear (can take up to 710ms)
  if (!this->wait_model_refresh_()) {
    ESP_LOGW(TAG, "Model refresh timeout - continuing anyway");
  }
  
  // Step 2.3: Restore original HibCFG value
  if (!this->restore_hibernate_mode_(saved_hib_cfg)) {
    ESP_LOGW(TAG, "Failed to restore HibCFG");
  }
  
  // === STEP 3: Clear POR bit (UG6595 Section 2, Step 3) ===
  // Must use WriteAndVerifyRegister to ensure POR bit is properly cleared
  if (!this->clear_por_bit_()) {
    ESP_LOGE(TAG, "Failed to clear POR bit");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  
  // Load learned parameters from NVS if available (UG6595 Step 3.4)
  // This allows the fuel gauge to resume with previously learned values
  if (this->load_learned_parameters_()) {
    ESP_LOGI(TAG, "Loaded learned parameters from NVS - fuel gauge will use saved calibration");
  } else {
    ESP_LOGD(TAG, "No saved learned parameters found - fuel gauge will learn from scratch");
  }
  
  // Read current cycle count to track for future saves
  uint16_t current_cycles;
  if (this->read_register_word_(MAX17260_REG_CYCLES, current_cycles)) {
    this->last_cycles_ = current_cycles;
    ESP_LOGD(TAG, "Starting cycle tracking at %d", current_cycles);
  }
  
  ESP_LOGI(TAG, "MAX17260 initialization complete per UG6595");
}

void MAX17260Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX17260:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  
  // Setup NVS preference for learned parameters (UG6595 Step 3.4)
  this->learned_params_pref_ = global_preferences->make_preference<LearnedParameters>(
    fnv1_hash("max17260_learned"));
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX17260 failed!");
  }
  
  // Read and log device identification
  uint16_t dev_name;
  if (this->read_register_word_(MAX17260_REG_DEVNAME, dev_name)) {
    ESP_LOGCONFIG(TAG, "  Device Name: 0x%04X", dev_name);
  }
  
  // Read 32-bit serial number from registers 0xD4-0xD5 (Table 16)
  // Per MAX17260 datasheet Table 16: Serial number registers are shadowed by
  // MaxPeakPower/SusPeakPower. Must clear Config2.AtRateEn (bit 10) AND Config2.DPEn (bit 15)
  // to access the 128-bit serial number (we read first 32 bits: Word0 and Word1)
  
  // Save Config2 and temporarily disable both AtRateEn AND DPEn
  uint16_t config2_orig;
  bool config2_saved = this->read_register_word_(MAX17260_REG_CONFIG2, config2_orig);
  ESP_LOGD(TAG, "SN Read: Config2 original = 0x%04X (DPEn=%d, AtRateEn=%d)", 
           config2_orig, 
           (config2_orig & 0x8000) ? 1 : 0,  // bit 15
           (config2_orig & 0x0400) ? 1 : 0); // bit 10
  
  if (config2_saved) {
    // Clear BOTH AtRateEn (bit 10) AND DPEn (bit 15) - BOTH must be 0
    uint16_t config2_temp = config2_orig & ~0x8400;  // Clear bits 15 and 10
    if (config2_temp != config2_orig) {
      this->write_register_word_(MAX17260_REG_CONFIG2, config2_temp);
      
      // Verify the write worked
      uint16_t config2_verify;
      this->read_register_word_(MAX17260_REG_CONFIG2, config2_verify);
      ESP_LOGD(TAG, "SN Read: Config2 modified = 0x%04X (DPEn=%d, AtRateEn=%d), verify = 0x%04X", 
               config2_temp,
               (config2_temp & 0x8000) ? 1 : 0,
               (config2_temp & 0x0400) ? 1 : 0,
               config2_verify);
      
      // Per app note: Poll Status2.SNReady (bit 7) until set
      bool sn_ready = false;
      for (int i = 0; i < 50; i++) {  // Poll for up to 500ms
        uint16_t status2;
        if (this->read_register_word_(MAX17260_REG_STATUS2, status2)) {
          if (status2 & 0x0080) {  // Bit 7 = SNReady
            sn_ready = true;
            ESP_LOGD(TAG, "SN Read: Status2.SNReady set after %dms (Status2=0x%04X)", i * 10, status2);
            break;
          }
        }
        delay(10);
      }
      if (!sn_ready) {
        ESP_LOGW(TAG, "SN Read: Timeout waiting for Status2.SNReady");
      }
    }
  }
  
  // Now read serial number from 0xD4 (Word0) and 0xD5 (Word1)
  uint16_t sn_word0, sn_word1;
  if (this->read_register_word_(MAX17260_REG_SN_WORD0, sn_word0) &&
      this->read_register_word_(MAX17260_REG_SN_WORD1, sn_word1)) {
    ESP_LOGCONFIG(TAG, "  Serial Number: %04X%04X (raw: Word1=0x%04X, Word0=0x%04X)", 
                  sn_word1, sn_word0, sn_word1, sn_word0);
    
    // Publish to text sensor if configured
    if (this->serial_number_sensor_ != nullptr) {
      char serial_str[9];
      snprintf(serial_str, sizeof(serial_str), "%04X%04X", sn_word1, sn_word0);
      this->serial_number_sensor_->publish_state(serial_str);
    }
  } else {
    ESP_LOGW(TAG, "  Failed to read serial number from 0xD4/0xD5");
  }
  
  // IMPORTANT: Restore original Config2 to re-enable DPEn and AtRateEn
  if (config2_saved) {
    this->write_register_word_(MAX17260_REG_CONFIG2, config2_orig);
    delay(20);  // Wait for shadow registers to revert
    
    // Verify restoration
    uint16_t config2_restored;
    this->read_register_word_(MAX17260_REG_CONFIG2, config2_restored);
    ESP_LOGD(TAG, "SN Read: Config2 restored to 0x%04X, verify = 0x%04X", config2_orig, config2_restored);
  }
  
  // Publish device name if configured
  if (this->device_name_sensor_ != nullptr && this->read_register_word_(MAX17260_REG_DEVNAME, dev_name)) {
    char name_str[3];
    name_str[0] = (dev_name >> 8) & 0xFF;
    name_str[1] = dev_name & 0xFF;
    name_str[2] = '\0';
    this->device_name_sensor_->publish_state(name_str);
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
  LOG_TEXT_SENSOR("  ", "Device Name", this->device_name_sensor_);
  LOG_TEXT_SENSOR("  ", "Serial Number", this->serial_number_sensor_);
}

void MAX17260Component::update() {
  // === STEP 3.1: Periodic POR monitoring (UG6595 Section 3, Step 3.1) ===
  // Check if POR bit has been set (indicates unexpected reset)
  // If POR is set, re-run initialization sequence
  if (this->initialized_ && this->check_por_bit_()) {
    ESP_LOGW(TAG, "POR bit detected during monitoring - reinitializing fuel gauge");
    this->setup();  // Re-run full initialization
    return;  // Skip this update cycle
  }
  
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
      
      // === STEP 3.4: Save learned parameters every 64% cycle change (UG6595 Section 3, Step 3.4) ===
      // Save RCOMP0, TempCo, FullCapRep, Cycles, FullCapNom to NVS
      // These parameters represent the fuel gauge's learned knowledge about the battery
      if (abs(static_cast<int>(raw_cycles) - static_cast<int>(this->last_cycles_)) >= 64) {
        ESP_LOGI(TAG, "Cycle count changed by 64%% or more (was %d, now %d) - saving learned parameters to NVS",
                 this->last_cycles_, raw_cycles);
        
        if (this->save_learned_parameters_()) {
          this->last_cycles_ = raw_cycles;
          ESP_LOGI(TAG, "Learned parameters saved successfully");
        } else {
          ESP_LOGW(TAG, "Failed to save learned parameters to NVS");
        }
      }
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

bool MAX17260Component::write_and_verify_register(uint8_t reg, uint16_t value) {
  // WriteAndVerifyRegister per UG6595 pseudocode
  // Attempt to write and verify up to 3 times with 1ms delays
  for (int attempt = 0; attempt < 3; attempt++) {
    // Write the register
    if (!this->write_register_word_(reg, value)) {
      ESP_LOGW(TAG, "Write failed on attempt %d for register 0x%02X", attempt + 1, reg);
      continue;
    }
    
    // Wait 1ms per UG6595 specification
    delay(1);
    
    // Read back and verify
    uint16_t read_value;
    if (this->read_register_word_(reg, read_value)) {
      if (read_value == value) {
        // Verification successful
        return true;
      } else {
        ESP_LOGD(TAG, "Verify mismatch on attempt %d for register 0x%02X: wrote 0x%04X, read 0x%04X",
                 attempt + 1, reg, value, read_value);
      }
    }
  }
  
  // All 3 attempts failed
  ESP_LOGE(TAG, "WriteAndVerifyRegister failed after 3 attempts for register 0x%02X", reg);
  return false;
}

bool MAX17260Component::check_por_bit_() {
  // Step 0: Check if POR bit is set (UG6595 Section 2, Step 0)
  uint16_t status;
  if (!this->read_register_word_(MAX17260_REG_STATUS, status)) {
    ESP_LOGE(TAG, "Failed to read Status register");
    return false;  // Assume not initialized if can't read
  }
  
  // Check bit 1 (POR bit)
  bool por_set = (status & STATUS_POR_BIT) != 0;
  
  if (por_set) {
    ESP_LOGD(TAG, "POR bit is set (Status=0x%04X), initialization required", status);
  } else {
    ESP_LOGD(TAG, "POR bit is clear (Status=0x%04X), already initialized", status);
  }
  
  return por_set;
}

bool MAX17260Component::wait_for_dnr_clear_() {
  // Step 1: Wait while FSTAT.DNR == 1 (UG6595 Section 2, Step 1)
  // DNR = Data Not Ready, bit 0 of FSTAT register
  // Polling interval: 10ms per UG6595 pseudocode
  // Timeout: ~10 seconds (1000 iterations)
  
  ESP_LOGD(TAG, "Waiting for FSTAT.DNR to clear...");
  
  for (int i = 0; i < 1000; i++) {
    uint16_t fstat;
    if (!this->read_register_word_(MAX17260_REG_FSTAT, fstat)) {
      ESP_LOGW(TAG, "Failed to read FSTAT during DNR wait");
      return false;
    }
    
    // Check bit 0 (DNR)
    if ((fstat & FSTAT_DNR_BIT) == 0) {
      ESP_LOGD(TAG, "FSTAT.DNR cleared after %d ms", i * 10);
      return true;
    }
    
    delay(10);  // 10ms delay per UG6595
  }
  
  ESP_LOGE(TAG, "Timeout waiting for FSTAT.DNR to clear");
  return false;
}

bool MAX17260Component::exit_hibernate_mode_(uint16_t &saved_hib_cfg) {
  // Step 2.0: Save original HibCFG and exit hibernate mode (UG6595 Section 2, Step 2)
  // Exit hibernate sequence (UG6595 pseudocode):
  //   HibCFG = ReadRegister(0xBA)
  //   WriteRegister(0x60, 0x90)
  //   WriteRegister(0xBA, 0x0)
  //   WriteRegister(0x60, 0x0)
  
  if (!this->read_register_word_(MAX17260_REG_HIBCFG, saved_hib_cfg)) {
    ESP_LOGE(TAG, "Failed to read HibCFG");
    return false;
  }
  
  ESP_LOGD(TAG, "Saved HibCFG: 0x%04X", saved_hib_cfg);
  
  // Exit hibernate sequence
  if (!this->write_register_word_(MAX17260_REG_SOFTWAKEUP, 0x90)) {
    ESP_LOGE(TAG, "Failed to write SoftWakeup (step 1)");
    return false;
  }
  
  if (!this->write_register_word_(MAX17260_REG_HIBCFG, 0x0000)) {
    ESP_LOGE(TAG, "Failed to write HibCFG=0");
    return false;
  }
  
  if (!this->write_register_word_(MAX17260_REG_SOFTWAKEUP, 0x0000)) {
    ESP_LOGE(TAG, "Failed to write SoftWakeup (step 2)");
    return false;
  }
  
  ESP_LOGD(TAG, "Exited hibernate mode");
  return true;
}

bool MAX17260Component::restore_hibernate_mode_(uint16_t hib_cfg) {
  // Step 2.3: Restore original HibCFG value (UG6595 Section 2.3.9)
  if (!this->write_register_word_(MAX17260_REG_HIBCFG, hib_cfg)) {
    ESP_LOGE(TAG, "Failed to restore HibCFG to 0x%04X", hib_cfg);
    return false;
  }
  
  ESP_LOGD(TAG, "Restored HibCFG to 0x%04X", hib_cfg);
  return true;
}

bool MAX17260Component::perform_ez_config_(float charge_voltage) {
  // Step 2.1: EZ Config (UG6595 Section 2.1)
  // Write DesignCap, IChgTerm, VEmpty, and ModelCFG
  
  ESP_LOGD(TAG, "Performing EZ Config...");
  
  // Write DesignCap (330mAh battery with 25mΩ sense resistor)
  // LSB = 5.0μVh/RSENSE = 5.0μVh/0.025Ω = 0.2mAh
  // DesignCap = 330mAh / 0.2mAh = 1650
  if (!this->write_register_word_(MAX17260_REG_DESIGNCAP, DESIGN_CAP)) {
    ESP_LOGE(TAG, "Failed to write DesignCap");
    return false;
  }
  ESP_LOGD(TAG, "  DesignCap = %d (330mAh)", DESIGN_CAP);
  
  // Write IChgTerm (20mA charge termination current)
  // LSB = 1.5625μV/RSENSE = 1.5625μV/0.025Ω = 62.5μA
  // IChgTerm = 20mA / 0.0625mA = 320
  if (!this->write_register_word_(MAX17260_REG_ICHGTERM, ICHG_TERM)) {
    ESP_LOGE(TAG, "Failed to write IChgTerm");
    return false;
  }
  ESP_LOGD(TAG, "  IChgTerm = %d (20mA)", ICHG_TERM);
  
  // Write VEmpty (0xA561: VE=3.3V, VR=3.88V per datasheet default)
  if (!this->write_register_word_(MAX17260_REG_VEMPTY, VEMPTY)) {
    ESP_LOGE(TAG, "Failed to write VEmpty");
    return false;
  }
  ESP_LOGD(TAG, "  VEmpty = 0x%04X (VE=3.3V, VR=3.88V)", VEMPTY);
  
  // Write ModelCFG with voltage-dependent VChg bit (UG6595 Section 2.1)
  // Bit 15 (Refresh) = 1 to initiate model refresh
  // Bit 10 (VChg) = 1 if charge voltage > 4.275V, else 0
  // ModelID bits = 0 for standard Li-ion
  uint16_t model_cfg;
  if (charge_voltage > 4.275f) {
    model_cfg = 0x8400;  // Refresh=1, VChg=1
    ESP_LOGD(TAG, "  ModelCFG = 0x%04X (high voltage >4.275V)", model_cfg);
  } else {
    model_cfg = 0x8000;  // Refresh=1, VChg=0
    ESP_LOGD(TAG, "  ModelCFG = 0x%04X (standard voltage 4.2V)", model_cfg);
  }
  
  if (!this->write_register_word_(MAX17260_REG_MODELCFG, model_cfg)) {
    ESP_LOGE(TAG, "Failed to write ModelCFG");
    return false;
  }
  
  ESP_LOGD(TAG, "EZ Config complete");
  return true;
}

bool MAX17260Component::wait_model_refresh_() {
  // Step 2.2: Wait for ModelCFG.Refresh bit to clear (UG6595 Section 2.1)
  // Poll ModelCFG bit 15 every 10ms, timeout after 710ms (71 iterations)
  
  ESP_LOGD(TAG, "Waiting for model refresh to complete (up to 710ms)...");
  
  for (int i = 0; i < 71; i++) {
    delay(10);
    
    uint16_t model_cfg;
    if (!this->read_register_word_(MAX17260_REG_MODELCFG, model_cfg)) {
      ESP_LOGW(TAG, "Failed to read ModelCFG during refresh wait");
      return false;
    }
    
    // Check bit 15 (Refresh)
    if ((model_cfg & MODELCFG_REFRESH_BIT) == 0) {
      ESP_LOGD(TAG, "Model refresh complete after %d ms (ModelCFG=0x%04X)", i * 10, model_cfg);
      return true;
    }
  }
  
  ESP_LOGW(TAG, "Model refresh timeout after 710ms");
  return false;
}

bool MAX17260Component::clear_por_bit_() {
  // Step 3: Clear POR bit using WriteAndVerifyRegister (UG6595 Section 2, Step 3)
  // Must use WriteAndVerifyRegister to ensure POR bit is reliably cleared
  
  uint16_t status;
  if (!this->read_register_word_(MAX17260_REG_STATUS, status)) {
    ESP_LOGE(TAG, "Failed to read Status before clearing POR");
    return false;
  }
  
  ESP_LOGD(TAG, "Clearing POR bit (Status before: 0x%04X)", status);
  
  // Clear bit 1 (POR) while preserving other bits
  uint16_t new_status = status & ~STATUS_POR_BIT;
  
  if (!this->write_and_verify_register(MAX17260_REG_STATUS, new_status)) {
    ESP_LOGE(TAG, "Failed to clear POR bit with WriteAndVerifyRegister");
    return false;
  }
  
  // Verify POR bit is cleared
  if (!this->read_register_word_(MAX17260_REG_STATUS, status)) {
    ESP_LOGW(TAG, "Failed to verify POR bit clear");
    return false;
  }
  
  if (status & STATUS_POR_BIT) {
    ESP_LOGE(TAG, "POR bit still set after clear attempt (Status=0x%04X)", status);
    return false;
  }
  
  ESP_LOGD(TAG, "POR bit cleared successfully (Status after: 0x%04X)", status);
  return true;
}

uint32_t MAX17260Component::calculate_checksum_(const LearnedParameters &params) {
  // Simple checksum: XOR all words together
  uint32_t checksum = 0;
  checksum ^= params.rcomp0;
  checksum ^= params.tempco;
  checksum ^= params.fullcaprep;
  checksum ^= params.cycles;
  checksum ^= params.fullcapnom;
  return checksum;
}

bool MAX17260Component::load_learned_parameters_() {
  // Load learned parameters from NVS (UG6595 Step 3.4)
  // These parameters represent the fuel gauge's learned knowledge about the battery
  
  LearnedParameters params;
  if (!this->learned_params_pref_.load(&params)) {
    ESP_LOGD(TAG, "No learned parameters found in NVS");
    return false;
  }
  
  // Validate checksum
  uint32_t expected_checksum = this->calculate_checksum_(params);
  if (params.checksum != expected_checksum) {
    ESP_LOGW(TAG, "Learned parameters checksum mismatch (expected 0x%08X, got 0x%08X) - ignoring",
             expected_checksum, params.checksum);
    return false;
  }
  
  ESP_LOGD(TAG, "Loading learned parameters from NVS:");
  ESP_LOGD(TAG, "  RCOMP0: 0x%04X", params.rcomp0);
  ESP_LOGD(TAG, "  TempCo: 0x%04X", params.tempco);
  ESP_LOGD(TAG, "  FullCapRep: 0x%04X", params.fullcaprep);
  ESP_LOGD(TAG, "  Cycles: 0x%04X (%.2f cycles)", params.cycles, params.cycles * CYCLES_SCALE);
  ESP_LOGD(TAG, "  FullCapNom: 0x%04X", params.fullcapnom);
  
  // Write learned parameters to fuel gauge registers
  // These will be used by the ModelGauge m5 algorithm instead of defaults
  bool success = true;
  
  if (!this->write_register_word_(MAX17260_REG_RCOMP0, params.rcomp0)) {
    ESP_LOGW(TAG, "Failed to write RCOMP0 from NVS");
    success = false;
  }
  
  if (!this->write_register_word_(MAX17260_REG_TEMPCO, params.tempco)) {
    ESP_LOGW(TAG, "Failed to write TempCo from NVS");
    success = false;
  }
  
  if (!this->write_register_word_(MAX17260_REG_FULLCAPREP, params.fullcaprep)) {
    ESP_LOGW(TAG, "Failed to write FullCapRep from NVS");
    success = false;
  }
  
  if (!this->write_register_word_(MAX17260_REG_FULLCAPNOM, params.fullcapnom)) {
    ESP_LOGW(TAG, "Failed to write FullCapNom from NVS");
    success = false;
  }
  
  // Note: We don't write Cycles register as it's managed by the fuel gauge
  // We only save it for tracking 64% changes
  
  return success;
}

bool MAX17260Component::save_learned_parameters_() {
  // Save learned parameters to NVS (UG6595 Step 3.4)
  // Per UG6595: Save RCOMP0, TempCo, FullCapRep, Cycles, FullCapNom every 64% cycle change
  
  LearnedParameters params;
  
  // Read current values from fuel gauge registers
  if (!this->read_register_word_(MAX17260_REG_RCOMP0, params.rcomp0)) {
    ESP_LOGW(TAG, "Failed to read RCOMP0 for saving");
    return false;
  }
  
  if (!this->read_register_word_(MAX17260_REG_TEMPCO, params.tempco)) {
    ESP_LOGW(TAG, "Failed to read TempCo for saving");
    return false;
  }
  
  if (!this->read_register_word_(MAX17260_REG_FULLCAPREP, params.fullcaprep)) {
    ESP_LOGW(TAG, "Failed to read FullCapRep for saving");
    return false;
  }
  
  if (!this->read_register_word_(MAX17260_REG_CYCLES, params.cycles)) {
    ESP_LOGW(TAG, "Failed to read Cycles for saving");
    return false;
  }
  
  if (!this->read_register_word_(MAX17260_REG_FULLCAPNOM, params.fullcapnom)) {
    ESP_LOGW(TAG, "Failed to read FullCapNom for saving");
    return false;
  }
  
  // Calculate checksum
  params.checksum = this->calculate_checksum_(params);
  
  ESP_LOGD(TAG, "Saving learned parameters to NVS:");
  ESP_LOGD(TAG, "  RCOMP0: 0x%04X", params.rcomp0);
  ESP_LOGD(TAG, "  TempCo: 0x%04X", params.tempco);
  ESP_LOGD(TAG, "  FullCapRep: 0x%04X (%.1f mAh)", params.fullcaprep, params.fullcaprep * CAPACITY_SCALE);
  ESP_LOGD(TAG, "  Cycles: 0x%04X (%.2f cycles)", params.cycles, params.cycles * CYCLES_SCALE);
  ESP_LOGD(TAG, "  FullCapNom: 0x%04X (%.1f mAh)", params.fullcapnom, params.fullcapnom * CAPACITY_SCALE);
  ESP_LOGD(TAG, "  Checksum: 0x%08X", params.checksum);
  
  // Save to NVS
  if (!this->learned_params_pref_.save(&params)) {
    ESP_LOGE(TAG, "Failed to save learned parameters to NVS");
    return false;
  }
  
  return true;
}

}  // namespace max17260
}  // namespace esphome
