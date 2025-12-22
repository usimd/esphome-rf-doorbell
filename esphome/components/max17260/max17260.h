#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace max17260 {

// MAX17260 Register Map - from datasheet Table 2
static const uint8_t MAX17260_REG_STATUS = 0x00;
static const uint8_t MAX17260_REG_REPCAP = 0x05;      // Reported remaining capacity
static const uint8_t MAX17260_REG_REPSOC = 0x06;      // Reported state of charge  
static const uint8_t MAX17260_REG_TEMP = 0x08;        // Temperature
static const uint8_t MAX17260_REG_VCELL = 0x09;       // Cell voltage
static const uint8_t MAX17260_REG_CURRENT = 0x0A;     // Instantaneous current
static const uint8_t MAX17260_REG_AVGCURRENT = 0x0B;  // Average current
static const uint8_t MAX17260_REG_FULLCAPREP = 0x10;  // Full capacity (reported)
static const uint8_t MAX17260_REG_TTE = 0x11;         // Time to empty
static const uint8_t MAX17260_REG_CYCLES = 0x17;      // Cycle counter
static const uint8_t MAX17260_REG_DESIGNCAP = 0x18;   // Design capacity
static const uint8_t MAX17260_REG_AVGVCELL = 0x19;    // Average cell voltage
static const uint8_t MAX17260_REG_ICHGTERM = 0x1E;    // Charge termination current
static const uint8_t MAX17260_REG_TTF = 0x20;         // Time to full
static const uint8_t MAX17260_REG_VEMPTY = 0x3A;      // Empty voltage
static const uint8_t MAX17260_REG_FSTAT = 0x3D;       // Fuel gauge status
static const uint8_t MAX17260_REG_HIBCFG = 0xBA;      // Hibernate config
static const uint8_t MAX17260_REG_MODELCFG = 0xDB;    // Model config
static const uint8_t MAX17260_REG_SOFTWAKEUP = 0x60;  // Soft-wakeup

// Scaling factors from datasheet Table 3
static const float VCELL_SCALE = 0.078125e-3f;         // 78.125µV per LSB
static const float CURRENT_SCALE = 1.5625e-6f / 0.025f; // 1.5625μV/RSENSE, RSENSE=25mΩ -> 0.0625mA per LSB
static const float CAPACITY_SCALE = 5.0e-6f / 0.025f;  // 5μVh/RSENSE, RSENSE=25mΩ -> 0.2mAh per LSB  
static const float PERCENT_SCALE = 1.0f / 256.0f;      // 1/256% per LSB (full 16-bit resolution)
static const float TIME_SCALE = 5.625f;                // 5.625s per LSB
static const float TEMP_SCALE = 1.0f / 256.0f;         // 1/256°C per LSB
static const float CYCLES_SCALE = 1.0f / 100.0f;       // 1% cycle per LSB

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
  void set_remaining_capacity_sensor(sensor::Sensor *sensor) { remaining_capacity_sensor_ = sensor; }
  void set_full_capacity_sensor(sensor::Sensor *sensor) { full_capacity_sensor_ = sensor; }
  void set_cycle_count_sensor(sensor::Sensor *sensor) { cycle_count_sensor_ = sensor; }

  // Public register access for advanced features
  bool read_register(uint8_t reg, uint16_t &value) { return read_register_word_(reg, value); }
  bool write_register(uint8_t reg, uint16_t value) { return write_register_word_(reg, value); }

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *tte_sensor_{nullptr};
  sensor::Sensor *ttf_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *remaining_capacity_sensor_{nullptr};
  sensor::Sensor *full_capacity_sensor_{nullptr};
  sensor::Sensor *cycle_count_sensor_{nullptr};

  bool read_register_word_(uint8_t reg, uint16_t &value);
  bool write_register_word_(uint8_t reg, uint16_t value);
  bool wait_for_dnr_clear_();
  bool exit_hibernate_mode_();
  bool restore_hibernate_mode_(uint16_t hib_cfg);
};

}  // namespace max17260
}  // namespace esphome
