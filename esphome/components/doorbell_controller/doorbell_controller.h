#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/deep_sleep/deep_sleep_component.h"

#ifdef USE_ESP32
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#endif

namespace esphome {
namespace doorbell_controller {

// Forward declarations
class DoorbellControllerComponent;

// =============================================================================
// ISL23315 Digital Potentiometer Constants
// =============================================================================
static const uint8_t ISL23315_DEFAULT_ADDRESS = 0x50;
static const uint8_t ISL23315_WIPER_INSTRUCTION = 0x00;
static const uint8_t ISL23315_WIPER_DEFAULT = 128;
static const uint8_t ISL23315_WIPER_MIN = 0;
static const uint8_t ISL23315_WIPER_MAX = 255;

// =============================================================================
// LTC4079 Charger Constants  
// =============================================================================
static const float LTC4079_VEN_THRESHOLD = 1.190f;  // V - charging enable threshold
static const float LTC4079_VEN_SHUTDOWN = 0.750f;   // V - shutdown threshold
static const float CHARGE_CURRENT_THRESHOLD = 1.0f; // mA - above this = charging active

// =============================================================================
// VINDPM Threshold Number Component
// =============================================================================
class VINDPMThresholdNumber : public number::Number, public Component {
 public:
  void set_parent(DoorbellControllerComponent *parent) { this->parent_ = parent; }
  void setup() override;
  void dump_config() override;
  
 protected:
  void control(float value) override;
  DoorbellControllerComponent *parent_{nullptr};
};

// =============================================================================
// Sleep Duration Number Component
// =============================================================================
class SleepDurationNumber : public number::Number, public Component {
 public:
  void set_parent(DoorbellControllerComponent *parent) { this->parent_ = parent; }
  void setup() override;
  void dump_config() override;
  
 protected:
  void control(float value) override;
  DoorbellControllerComponent *parent_{nullptr};
};

// =============================================================================
// Charging Enabled Switch Component
// =============================================================================
class ChargingEnabledSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(DoorbellControllerComponent *parent) { this->parent_ = parent; }
  void setup() override;
  void dump_config() override;
  
 protected:
  void write_state(bool state) override;
  DoorbellControllerComponent *parent_{nullptr};
};

// =============================================================================
// Charge Disable Override Switch Component
// =============================================================================
class ChargeDisableSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(DoorbellControllerComponent *parent) { this->parent_ = parent; }
  void setup() override;
  void dump_config() override;
  
 protected:
  void write_state(bool state) override;
  DoorbellControllerComponent *parent_{nullptr};
};

// =============================================================================
// Bell Mute Switch Component
// =============================================================================
class BellMuteSwitch : public switch_::Switch, public Component {
 public:
  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void on_safe_shutdown() override;

 protected:
  void write_state(bool state) override;
  void apply_output_(bool state);
  void enable_hold_();
  void release_hold_(bool state);

  InternalGPIOPin *pin_{nullptr};
};

// =============================================================================
// Main Doorbell Controller Component
// =============================================================================
class DoorbellControllerComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;  // Called on wakeup / periodic
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // =============================================================================
  // Configuration Setters
  // =============================================================================
  
  // VINDPM voltage divider
  void set_r_high(float r_high) { r_high_ = r_high; }
  void set_r_low(float r_low) { r_low_ = r_low; }
  void set_digipot_resistance(float r_pot) { r_pot_ = r_pot; }
  
  // Supply voltage divider ratio
  void set_supply_divider_ratio(float ratio) { supply_divider_ratio_ = ratio; }
  
  // LTC4079 R_PROG for charge current calculation
  void set_r_prog(float r_prog) { r_prog_ = r_prog; }
  
  // Series resistor on V_PROG ADC input for isolation/compensation
  void set_r_series_vprog(float r_series) { r_series_vprog_ = r_series; }
  
  // Hardware pins
  void set_charge_disable_pin(GPIOPin *pin) { charge_disable_pin_ = pin; }
  void set_supply_sense_enable_pin(GPIOPin *pin) { supply_sense_enable_pin_ = pin; }
  void set_supply_adc_pin(uint8_t pin) { supply_adc_pin_ = pin; }
  void set_vprog_adc_pin(uint8_t pin) { vprog_adc_pin_ = pin; }
  
  // Deep sleep reference
  void set_deep_sleep(deep_sleep::DeepSleepComponent *deep_sleep) { deep_sleep_ = deep_sleep; }

  // =============================================================================
  // Sub-component Setters
  // =============================================================================
  void set_vindpm_threshold_number(VINDPMThresholdNumber *num) { vindpm_threshold_number_ = num; }
  void set_sleep_duration_number(SleepDurationNumber *num) { sleep_duration_number_ = num; }
  void set_charging_enabled_switch(ChargingEnabledSwitch *sw) { charging_enabled_switch_ = sw; }
  void set_charge_disable_switch(ChargeDisableSwitch *sw) { charge_disable_switch_ = sw; }
  void set_wiper_value_sensor(sensor::Sensor *sensor) { wiper_value_sensor_ = sensor; }
  void set_supply_voltage_sensor(sensor::Sensor *sensor) { supply_voltage_sensor_ = sensor; }
  void set_charge_current_sensor(sensor::Sensor *sensor) { charge_current_sensor_ = sensor; }
  void set_charging_binary_sensor(binary_sensor::BinarySensor *sensor) { charging_binary_sensor_ = sensor; }

  // =============================================================================
  // ISL23315 Digipot Control Methods
  // =============================================================================
  bool read_wiper(uint8_t &value);
  bool write_wiper(uint8_t value);

  // =============================================================================
  // VINDPM Threshold Methods
  // =============================================================================
  float wiper_to_voltage(uint8_t wiper_value);
  uint8_t voltage_to_wiper(float voltage);
  bool set_vindpm_threshold(float voltage);
  bool get_vindpm_threshold(float &voltage);

  // =============================================================================
  // Charging Control Methods
  // =============================================================================
  bool enable_charging();
  bool disable_charging();
  void set_charge_override(bool disable);

  // =============================================================================
  // Supply Voltage Measurement
  // =============================================================================
  
  /**
   * Read supply voltage from ADC
   * Enables sense circuit, reads ADC, disables to save power
   * @return Supply voltage in volts, or NaN on failure
   */
  float read_supply_voltage();
  
  /**
   * Read V_PROG voltage from ADC (internal use)
   * @return V_PROG voltage in volts, or NaN on failure
   */
  float read_vprog_voltage();
  
  /**
   * Read charge current from V_PROG
   * Calculated as I_CHG = 250 * V_PROG / R_PROG
   * @return Charge current in mA, or NaN on failure
   */
  float read_charge_current();
  
  /**
   * Check if battery is currently charging
   * Based on charge current threshold
   */
  bool is_charging();

  // =============================================================================
  // Sleep Duration Control
  // =============================================================================
  
  /**
   * Set deep sleep duration
   * @param seconds Sleep duration in seconds
   */
  void set_sleep_duration(uint32_t seconds);
  
  /**
   * Get current sleep duration setting
   * @return Sleep duration in seconds
   */
  uint32_t get_sleep_duration() const { return sleep_duration_; }

  // =============================================================================
  // Deep Sleep / Wakeup Handling
  // =============================================================================
  
#ifdef USE_ESP32
  static esp_sleep_wakeup_cause_t get_wakeup_cause();
  static uint64_t get_ext1_wakeup_pins();
  static uint64_t get_gpio_wakeup_pins();
  static bool is_gpio_wakeup(uint8_t gpio);
#endif

 protected:
  // VINDPM voltage divider configuration
  float r_high_{820000.0f};   // R5 in schematic (820k)
  float r_low_{120000.0f};    // R6 in schematic (120k)
  float r_pot_{100000.0f};    // ISL23315T total resistance (100k)
  
  // Supply voltage divider ratio
  float supply_divider_ratio_{11.0f};  // (R10+R11)/R11 = 11.0
  
  // LTC4079 R_PROG for charge current calculation
  float r_prog_{30000.0f};    // R11 in schematic (30k) -> I_CHG = 250 * V_PROG / R_PROG
  
  // Series resistor on V_PROG ADC input for isolation
  // Forms voltage divider with ADC input impedance (~2MΩ for ESP32 at 12dB atten)
  float r_series_vprog_{47000.0f};  // 47k series resistor
  
  // Hardware pins
  GPIOPin *charge_disable_pin_{nullptr};
  GPIOPin *supply_sense_enable_pin_{nullptr};
  uint8_t supply_adc_pin_{0};  // GPIO0 default
  uint8_t vprog_adc_pin_{1};   // GPIO1 default
  
  // Deep sleep reference
  deep_sleep::DeepSleepComponent *deep_sleep_{nullptr};
  uint32_t sleep_duration_{300};  // Match the firmware deep_sleep default

  // Sub-components
  VINDPMThresholdNumber *vindpm_threshold_number_{nullptr};
  SleepDurationNumber *sleep_duration_number_{nullptr};
  ChargingEnabledSwitch *charging_enabled_switch_{nullptr};
  ChargeDisableSwitch *charge_disable_switch_{nullptr};
  sensor::Sensor *wiper_value_sensor_{nullptr};
  sensor::Sensor *supply_voltage_sensor_{nullptr};
  sensor::Sensor *charge_current_sensor_{nullptr};
  binary_sensor::BinarySensor *charging_binary_sensor_{nullptr};

  // State
  uint8_t current_wiper_{ISL23315_WIPER_DEFAULT};
  bool charge_override_active_{false};

  // Internal helpers
  void update_wiper_sensor_();
  void update_power_sensors_();
  
#ifdef USE_ESP32
  float read_adc_voltage_(uint8_t pin);
#endif
};

}  // namespace doorbell_controller
}  // namespace esphome
