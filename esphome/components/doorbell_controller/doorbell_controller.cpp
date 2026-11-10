#include "doorbell_controller.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h"
#endif

namespace esphome {
namespace doorbell_controller {

static const char *const TAG = "doorbell_controller";

// =============================================================================
// DoorbellControllerComponent Implementation
// =============================================================================

void DoorbellControllerComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Doorbell Controller...");

  // The GPIO binary sensors already configured these inputs at HARDWARE priority.
  if (esp_sleep_enable_gpio_wakeup() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable light-sleep GPIO wakeup");
    this->mark_failed();
    return;
  }
  this->wakeup_sources_ready_ = true;
  this->refresh_wakeup_sources();

  // Initialize charge disable pin if configured
  if (this->charge_disable_pin_ != nullptr) {
    this->charge_disable_pin_->setup();
    this->charge_disable_pin_->pin_mode(gpio::FLAG_INPUT);  // High-Z default
    ESP_LOGD(TAG, "Charge disable pin configured (high-Z mode)");
  }

  // Initialize supply sense enable pin if configured
  if (this->supply_sense_enable_pin_ != nullptr) {
    this->supply_sense_enable_pin_->setup();
    this->supply_sense_enable_pin_->digital_write(false);  // Off by default
    ESP_LOGD(TAG, "Supply sense enable pin configured");
  }

  // Read current wiper value from ISL23315
  uint8_t wiper;
  if (this->read_wiper(wiper)) {
    this->current_wiper_ = wiper;
    ESP_LOGI(TAG, "ISL23315 wiper value: %d (%.1fV VINDPM threshold)", 
             wiper, this->wiper_to_voltage(wiper));
    this->update_wiper_sensor_();
  } else {
    ESP_LOGW(TAG, "Failed to read ISL23315 wiper value, using default");
    this->current_wiper_ = ISL23315_WIPER_DEFAULT;
  }
}

void DoorbellControllerComponent::update() {
  this->update_power_sensors_();
}

void DoorbellControllerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Doorbell Controller:");
  ESP_LOGCONFIG(TAG, "  VINDPM Divider: R_high=%.0fΩ, R_low=%.0fΩ, R_pot=%.0fΩ", 
                this->r_high_, this->r_low_, this->r_pot_);
  ESP_LOGCONFIG(TAG, "  Supply Divider Ratio: %.1f", this->supply_divider_ratio_);
  ESP_LOGCONFIG(TAG, "  R_PROG: %.0fΩ (max charge current: %.1fmA)", 
                this->r_prog_, 250.0f * LTC4079_VEN_THRESHOLD / this->r_prog_ * 1000.0f);
  ESP_LOGCONFIG(TAG, "  R_SERIES_VPROG: %.0fΩ (ADC isolation resistor)", this->r_series_vprog_);
  ESP_LOGCONFIG(TAG, "  Current wiper: %d (VINDPM: %.2fV)", 
                this->current_wiper_, this->wiper_to_voltage(this->current_wiper_));
  LOG_PIN("  Bell Wakeup Pin: ", this->bell_wakeup_pin_);
  LOG_PIN("  Battery Alert Wakeup Pin: ", this->battery_alert_wakeup_pin_);
  
  if (this->charge_disable_pin_ != nullptr) {
    LOG_PIN("  Charge Disable Pin: ", this->charge_disable_pin_);
  }
  if (this->supply_sense_enable_pin_ != nullptr) {
    LOG_PIN("  Supply Sense Enable Pin: ", this->supply_sense_enable_pin_);
  }
  ESP_LOGCONFIG(TAG, "  Supply ADC Pin: GPIO%d", this->supply_adc_pin_);
  ESP_LOGCONFIG(TAG, "  V_PROG ADC Pin: GPIO%d", this->vprog_adc_pin_);
}

// =============================================================================
// ISL23315 Digipot Control
// =============================================================================

bool DoorbellControllerComponent::read_wiper(uint8_t &value) {
  uint8_t cmd = ISL23315_WIPER_INSTRUCTION;
  
  // Use write_read() for consecutive write and read operations
  i2c::ErrorCode err = this->write_read(&cmd, 1, &value, 1);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "ISL23315 read wiper failed: %d", err);
    return false;
  }
  
  return true;
}

bool DoorbellControllerComponent::write_wiper(uint8_t value) {
  uint8_t data[2] = {ISL23315_WIPER_INSTRUCTION, value};
  
  i2c::ErrorCode err = this->write(data, 2);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "ISL23315 write wiper failed: %d", err);
    return false;
  }
  
  this->current_wiper_ = value;
  this->update_wiper_sensor_();
  
  ESP_LOGD(TAG, "ISL23315 wiper set to %d (VINDPM: %.2fV)", 
           value, this->wiper_to_voltage(value));
  return true;
}

// =============================================================================
// VINDPM Threshold Calculation
// =============================================================================

float DoorbellControllerComponent::wiper_to_voltage(uint8_t wiper_value) {
  float r_wiper = (static_cast<float>(wiper_value) / 255.0f) * this->r_pot_;
  float r_total = this->r_high_ + this->r_low_ + r_wiper;
  float r_bottom = this->r_low_ + r_wiper;
  return LTC4079_VEN_THRESHOLD * r_total / r_bottom;
}

uint8_t DoorbellControllerComponent::voltage_to_wiper(float voltage) {
  float ven = LTC4079_VEN_THRESHOLD;
  float r_wiper = (ven * (this->r_high_ + this->r_low_) - voltage * this->r_low_) / (voltage - ven);
  
  if (r_wiper < 0) r_wiper = 0;
  if (r_wiper > this->r_pot_) r_wiper = this->r_pot_;
  
  return static_cast<uint8_t>((r_wiper / this->r_pot_) * 255.0f);
}

bool DoorbellControllerComponent::set_vindpm_threshold(float voltage) {
  uint8_t wiper = this->voltage_to_wiper(voltage);
  
  if (!this->write_wiper(wiper)) {
    return false;
  }
  
  ESP_LOGI(TAG, "VINDPM threshold set to %.2fV (wiper=%d)", voltage, wiper);
  return true;
}

bool DoorbellControllerComponent::get_vindpm_threshold(float &voltage) {
  uint8_t wiper;
  if (!this->read_wiper(wiper)) {
    return false;
  }
  
  voltage = this->wiper_to_voltage(wiper);
  return true;
}

// =============================================================================
// Charging Control
// =============================================================================

bool DoorbellControllerComponent::enable_charging() {
  ESP_LOGI(TAG, "Enabling charging (wiper=255, VINDPM ~%.1fV)", 
           this->wiper_to_voltage(ISL23315_WIPER_MAX));
  return this->write_wiper(ISL23315_WIPER_MAX);
}

bool DoorbellControllerComponent::disable_charging() {
  ESP_LOGI(TAG, "Disabling charging via VINDPM (wiper=0, VINDPM ~%.1fV)", 
           this->wiper_to_voltage(ISL23315_WIPER_MIN));
  return this->write_wiper(ISL23315_WIPER_MIN);
}

void DoorbellControllerComponent::set_charge_override(bool disable) {
  if (this->charge_disable_pin_ == nullptr) {
    ESP_LOGW(TAG, "Charge disable pin not configured");
    return;
  }
  
  if (disable) {
    this->charge_disable_pin_->pin_mode(gpio::FLAG_OUTPUT);
    this->charge_disable_pin_->digital_write(false);
    ESP_LOGI(TAG, "Charge override ACTIVE - charging force disabled");
  } else {
    this->charge_disable_pin_->pin_mode(gpio::FLAG_INPUT);
    ESP_LOGI(TAG, "Charge override RELEASED - VINDPM controls charging");
  }
  
  this->charge_override_active_ = disable;
}

// =============================================================================
// Supply Voltage Measurement
// =============================================================================

#ifdef USE_ESP32
float DoorbellControllerComponent::read_adc_voltage_(uint8_t pin) {
  // Use ESP-IDF ADC oneshot driver for accurate reading
  adc_oneshot_unit_handle_t adc_handle;
  adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  
  if (adc_oneshot_new_unit(&init_config, &adc_handle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC unit");
    return NAN;
  }
  
  adc_oneshot_chan_cfg_t chan_config = {
    .atten = ADC_ATTEN_DB_12,  // Full 0-3.3V range
    .bitwidth = ADC_BITWIDTH_12,
  };
  
  // Map GPIO to ADC channel (ESP32-C6 specific)
  adc_channel_t channel = static_cast<adc_channel_t>(pin);
  
  if (adc_oneshot_config_channel(adc_handle, channel, &chan_config) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure ADC channel");
    adc_oneshot_del_unit(adc_handle);
    return NAN;
  }
  
  int raw_value;
  if (adc_oneshot_read(adc_handle, channel, &raw_value) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read ADC");
    adc_oneshot_del_unit(adc_handle);
    return NAN;
  }

  adc_oneshot_del_unit(adc_handle);

  return (static_cast<float>(raw_value) / 4095.0f) * 3.3f;
}
#endif

float DoorbellControllerComponent::read_supply_voltage() {
#ifdef USE_ESP32
  if (this->supply_sense_enable_pin_ == nullptr) {
    ESP_LOGW(TAG, "Supply sense enable pin not configured");
    return NAN;
  }
  
  // Enable voltage divider
  this->supply_sense_enable_pin_->digital_write(true);
  delay(1);  // Allow settling
  
  // Read ADC
  float adc_voltage = this->read_adc_voltage_(this->supply_adc_pin_);
  
  // Disable voltage divider to save power
  this->supply_sense_enable_pin_->digital_write(false);
  
  if (std::isnan(adc_voltage)) {
    return NAN;
  }
  
  // Apply divider ratio
  float supply_voltage = adc_voltage * this->supply_divider_ratio_;
  ESP_LOGD(TAG, "Supply voltage: %.2fV (ADC: %.3fV)", supply_voltage, adc_voltage);
  
  return supply_voltage;
#else
  return NAN;
#endif
}

float DoorbellControllerComponent::read_vprog_voltage() {
#ifdef USE_ESP32
  float adc_voltage = this->read_adc_voltage_(this->vprog_adc_pin_);
  if (std::isnan(adc_voltage)) {
    return NAN;
  }
  
  // Compensate for voltage divider formed by series resistor and ADC input impedance
  // ESP32 ADC input impedance at 12dB attenuation is approximately 2MΩ
  // V_PROG_actual = V_ADC * (R_ADC + R_series) / R_ADC
  static const float ADC_INPUT_IMPEDANCE = 2000000.0f;  // ~2MΩ
  float compensation = (ADC_INPUT_IMPEDANCE + this->r_series_vprog_) / ADC_INPUT_IMPEDANCE;
  float vprog = adc_voltage * compensation;
  
  ESP_LOGV(TAG, "V_PROG: ADC=%.3fV, compensated=%.3fV (factor=%.3f)", 
           adc_voltage, vprog, compensation);
  return vprog;
#else
  return NAN;
#endif
}

float DoorbellControllerComponent::read_charge_current() {
#ifdef USE_ESP32
  float vprog = this->read_vprog_voltage();
  if (std::isnan(vprog)) {
    return NAN;
  }
  
  // LTC4079 datasheet: I_BAT / I_PROG = 250, with I_PROG = V_PROG / R_PROG
  float current_ma = (250.0f * vprog / this->r_prog_) * 1000.0f;
  ESP_LOGD(TAG, "Charge current: %.1fmA (V_PROG: %.3fV, R_PROG: %.0fΩ)", 
           current_ma, vprog, this->r_prog_);
  return current_ma;
#else
  return NAN;
#endif
}

bool DoorbellControllerComponent::is_charging() {
  float current = this->read_charge_current();
  return !std::isnan(current) && current > CHARGE_CURRENT_THRESHOLD;
}

// =============================================================================
// Light-Sleep Wakeup
// =============================================================================

bool DoorbellControllerComponent::accept_bell_press() {
  if (!this->bell_wakeup_.accept_press()) {
    ESP_LOGD(TAG, "Ignoring bell press during 30s cooldown");
    return false;
  }
  this->refresh_wakeup_sources();
  this->set_timeout("bell_cooldown", 30000, [this]() {
    this->bell_wakeup_.finish_cooldown();
    this->refresh_wakeup_sources();
  });
  return true;
}

void DoorbellControllerComponent::refresh_wakeup_sources() {
  if (!this->wakeup_sources_ready_)
    return;

  auto configure = [this](InternalGPIOPin *pin, BellWakeup::Level level) {
    auto gpio_num = static_cast<gpio_num_t>(pin->get_pin());
    esp_err_t err;
    if (level == BellWakeup::DISABLED) {
      err = gpio_wakeup_disable(gpio_num);
    } else {
      bool high = (level == BellWakeup::HIGH) != pin->is_inverted();
      err = gpio_wakeup_enable(gpio_num, high ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL);
    }
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure GPIO%d wakeup: %s", pin->get_pin(), esp_err_to_name(err));
      this->status_set_error(LOG_STR("GPIO wakeup configuration failed"));
    }
  };
  configure(this->bell_wakeup_pin_, this->bell_wakeup_.wake_level(this->bell_wakeup_pin_->digital_read()));
  // An asserted alert must not prevent sleep if the gauge cannot clear it.
  configure(this->battery_alert_wakeup_pin_,
            this->battery_alert_wakeup_pin_->digital_read() ? BellWakeup::LOW : BellWakeup::HIGH);
}

// =============================================================================
// Internal Helpers
// =============================================================================

void DoorbellControllerComponent::update_wiper_sensor_() {
  if (this->wiper_value_sensor_ != nullptr) {
    this->wiper_value_sensor_->publish_state(this->current_wiper_);
  }
}

void DoorbellControllerComponent::update_power_sensors_() {
  // Read and publish supply voltage
  if (this->supply_voltage_sensor_ != nullptr) {
    float supply = this->read_supply_voltage();
    if (!std::isnan(supply)) {
      this->supply_voltage_sensor_->publish_state(supply);
    }
  }
  
  // Read and publish charge current
  if (this->charge_current_sensor_ != nullptr) {
    float current = this->read_charge_current();
    if (!std::isnan(current)) {
      this->charge_current_sensor_->publish_state(current);
    }
  }
  
  // Update charging binary sensor
  if (this->charging_binary_sensor_ != nullptr) {
    this->charging_binary_sensor_->publish_state(this->is_charging());
  }
}

// =============================================================================
// VINDPMThresholdNumber Implementation
// =============================================================================

void VINDPMThresholdNumber::setup() {
  float voltage;
  if (this->parent_->get_vindpm_threshold(voltage)) {
    this->publish_state(voltage);
  }
}

void VINDPMThresholdNumber::dump_config() {
  LOG_NUMBER("", "VINDPM Threshold", this);
}

void VINDPMThresholdNumber::control(float value) {
  if (this->parent_->set_vindpm_threshold(value)) {
    this->publish_state(value);
  }
}

// =============================================================================
// ChargingEnabledSwitch Implementation
// =============================================================================

void ChargingEnabledSwitch::setup() {
  this->publish_state(true);
}

void ChargingEnabledSwitch::dump_config() {
  LOG_SWITCH("", "Charging Enabled", this);
}

void ChargingEnabledSwitch::write_state(bool state) {
  if (state) {
    if (this->parent_->enable_charging()) {
      this->publish_state(true);
    }
  } else {
    if (this->parent_->disable_charging()) {
      this->publish_state(false);
    }
  }
}

// =============================================================================
// ChargeDisableSwitch Implementation  
// =============================================================================

void ChargeDisableSwitch::setup() {
  this->publish_state(false);
}

void ChargeDisableSwitch::dump_config() {
  LOG_SWITCH("", "Charge Disable Override", this);
}

void ChargeDisableSwitch::write_state(bool state) {
  this->parent_->set_charge_override(state);
  this->publish_state(state);
}

// =============================================================================
// BellMuteSwitch Implementation
// =============================================================================

void BellMuteSwitch::setup() {
  if (this->pin_ == nullptr) {
    ESP_LOGE(TAG, "Bell mute pin not configured");
    this->mark_failed();
    return;
  }

  bool initial_state = false;
  auto restored_state = this->get_initial_state_with_restore_mode();
  if (restored_state.has_value()) {
    initial_state = restored_state.value();
  }

  this->pin_->setup();
  this->pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->apply_output_(initial_state);
  this->publish_state(initial_state);
}

void BellMuteSwitch::dump_config() {
  LOG_SWITCH("", "Bell Mute", this);
  if (this->pin_ != nullptr) {
    LOG_PIN("  Pin: ", this->pin_);
  }
}

void BellMuteSwitch::write_state(bool state) {
  if (this->pin_ == nullptr) {
    return;
  }

  this->apply_output_(state);
  this->publish_state(state);
}

void BellMuteSwitch::apply_output_(bool state) {
  this->pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_->digital_write(state);
}

}  // namespace doorbell_controller
}  // namespace esphome
