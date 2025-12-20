#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <Adafruit_BQ25628E.h>  // Adafruit library

namespace esphome {
namespace bq25628e {

class BQ25628EComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_bus_voltage_sensor(sensor::Sensor *sensor) { bus_voltage_sensor_ = sensor; }
  void set_battery_voltage_sensor(sensor::Sensor *sensor) { battery_voltage_sensor_ = sensor; }
  void set_charge_current_sensor(sensor::Sensor *sensor) { charge_current_sensor_ = sensor; }
  void set_system_voltage_sensor(sensor::Sensor *sensor) { system_voltage_sensor_ = sensor; }
  void set_charger_status_sensor(text_sensor::TextSensor *sensor) { charger_status_sensor_ = sensor; }
  void set_fault_status_sensor(text_sensor::TextSensor *sensor) { fault_status_sensor_ = sensor; }

  void set_charge_current_limit(float limit) { charge_current_limit_ = limit; }
  void set_charge_voltage_limit(float limit) { charge_voltage_limit_ = limit; }
  void set_input_current_limit(float limit) { input_current_limit_ = limit; }

  // Public methods for external control
  bool set_charging_enabled(bool enabled);
  bool set_charge_current(float current_amps);
  bool set_charge_voltage(float voltage_volts);
  bool set_input_current(float current_amps);

 protected:
  Adafruit_BQ25628E *bq_;  // Adafruit library instance
  
  sensor::Sensor *bus_voltage_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *charge_current_sensor_{nullptr};
  sensor::Sensor *system_voltage_sensor_{nullptr};
  text_sensor::TextSensor *charger_status_sensor_{nullptr};
  text_sensor::TextSensor *fault_status_sensor_{nullptr};

  float charge_current_limit_;
  float charge_voltage_limit_;
  float input_current_limit_;

  bool read_adc_values_();
  bool configure_charger_();
  const char *get_charger_status_text_(uint8_t status);
  const char *get_fault_status_text_(uint8_t fault);
};

}  // namespace bq25628e
}  // namespace esphome
