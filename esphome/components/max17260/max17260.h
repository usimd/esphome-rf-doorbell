#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace max17260 {

// MAX17260 Register Map - from datasheet
static const uint8_t MAX17260_REG_STATUS = 0x00;
static const uint8_t MAX17260_REG_VCELL = 0x09;       // Cell voltage
static const uint8_t MAX17260_REG_AVGVCELL = 0x19;    // Average cell voltage
static const uint8_t MAX17260_REG_CURRENT = 0x0A;     // Instantaneous current
static const uint8_t MAX17260_REG_AVGCURRENT = 0x0B;  // Average current
static const uint8_t MAX17260_REG_REPSOC = 0x06;      // Reported state of charge
static const uint8_t MAX17260_REG_TTE = 0x11;         // Time to empty
static const uint8_t MAX17260_REG_TTF = 0x20;         // Time to full
static const uint8_t MAX17260_REG_TEMP = 0x08;        // Temperature

// Scaling factors from datasheet
static const float VCELL_SCALE = 0.078125e-3f;  // 78.125µV per LSB
static const float CURRENT_SCALE_25mOhm = 0.0625e-3f;  // 1.5625μV/RSENSE, RSENSE=25mΩ -> 0.0625mA per LSB
static const float PERCENT_SCALE = 1.0f;                // 1% per LSB (upper byte)
static const float TIME_SCALE = 5.625f;                 // 5.625s per LSB
static const float TEMP_SCALE = 1.0f / 256.0f;          // 1/256°C per LSB

// Battery configuration
static const uint16_t DESIGN_CAP = 1650;  // 330mAh / (5μVh/25mΩ) = 1650
static const uint16_t ICHG_TERM = 320;    // 20mA × 25mΩ / 1.5625μV = 320
static const uint16_t VEMPTY = 0xA561;    // VE=3.3V, VR=3.88V (default for Li-ion)

class MAX17260Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_voltage_sensor(sensor::Sensor *sensor) { voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { current_sensor_ = sensor; }
  void set_state_of_charge_sensor(sensor::Sensor *sensor) { soc_sensor_ = sensor; }
  void set_time_to_empty_sensor(sensor::Sensor *sensor) { tte_sensor_ = sensor; }
  void set_time_to_full_sensor(sensor::Sensor *sensor) { ttf_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *tte_sensor_{nullptr};
  sensor::Sensor *ttf_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  bool read_register_word_(uint8_t reg, uint16_t &value);
  bool write_register_word_(uint8_t reg, uint16_t value);
};

}  // namespace max17260
}  // namespace esphome
