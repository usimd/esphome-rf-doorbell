#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace max17260 {

// MAX17260 Register Map
static const uint8_t MAX17260_REG_STATUS = 0x00;
static const uint8_t MAX17260_REG_VCELL = 0x09;
static const uint8_t MAX17260_REG_REPSOC = 0x06;
static const uint8_t MAX17260_REG_REPCAP = 0x05;
static const uint8_t MAX17260_REG_TEMP = 0x08;
static const uint8_t MAX17260_REG_CURRENT = 0x0A;
static const uint8_t MAX17260_REG_AVGCURRENT = 0x0B;
static const uint8_t MAX17260_REG_TTE = 0x11;
static const uint8_t MAX17260_REG_TTF = 0x20;
static const uint8_t MAX17260_REG_VFOCV = 0xFB;
static const uint8_t MAX17260_REG_DESIGNCAP = 0x18;
static const uint8_t MAX17260_REG_ICHGTERM = 0x1E;
static const uint8_t MAX17260_REG_MODELCFG = 0xDB;
static const uint8_t MAX17260_REG_HIBCFG = 0xBA;

class MAX17260Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_voltage_sensor(sensor::Sensor *sensor) { voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { current_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }
  void set_state_of_charge_sensor(sensor::Sensor *sensor) { state_of_charge_sensor_ = sensor; }
  void set_time_to_empty_sensor(sensor::Sensor *sensor) { time_to_empty_sensor_ = sensor; }
  void set_time_to_full_sensor(sensor::Sensor *sensor) { time_to_full_sensor_ = sensor; }

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *state_of_charge_sensor_{nullptr};
  sensor::Sensor *time_to_empty_sensor_{nullptr};
  sensor::Sensor *time_to_full_sensor_{nullptr};

  bool read_register_16_(uint8_t reg, uint16_t *value);
  bool write_register_16_(uint8_t reg, uint16_t value);
};

}  // namespace max17260
}  // namespace esphome
