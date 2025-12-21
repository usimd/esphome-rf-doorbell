#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bq25628e {

// BQ25628E Register Map - from SLUSFA4C datasheet
static const uint8_t BQ25628E_REG_ICHG_CTRL = 0x02;  // Charge Current Limit
static const uint8_t BQ25628E_REG_VBAT_CTRL = 0x04;  // Charge Voltage Limit
static const uint8_t BQ25628E_REG_IINDPM_CTRL = 0x06;  // Input Current Limit
static const uint8_t BQ25628E_REG_VINDPM_CTRL = 0x08;  // Input Voltage Limit
static const uint8_t BQ25628E_REG_CHG_CTRL = 0x14;  // Charge Control
static const uint8_t BQ25628E_REG_ADC_CTRL = 0x26;  // ADC Control
static const uint8_t BQ25628E_REG_ADC_DIS = 0x27;  // ADC Function Disable
static const uint8_t BQ25628E_REG_IBUS_ADC = 0x28;  // Input Current ADC (2 bytes)
static const uint8_t BQ25628E_REG_IBAT_ADC = 0x2A;  // Battery Current ADC (2 bytes)
static const uint8_t BQ25628E_REG_VBUS_ADC = 0x2C;  // Input Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_VBAT_ADC = 0x30;  // Battery Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_VSYS_ADC = 0x32;  // System Voltage ADC (2 bytes)

// ADC scaling factors from datasheet
static const float VBUS_ADC_STEP = 0.00397f;  // 3.97mV per LSB (REG0x2C)
static const float VBAT_ADC_STEP = 0.00199f;  // 1.99mV per LSB (REG0x30)
static const float VSYS_ADC_STEP = 0.00199f;  // 1.99mV per LSB (REG0x32)
static const float IBAT_ADC_STEP = 0.004f;    // 4mA per LSB (REG0x2A)
static const float IBUS_ADC_STEP = 0.002f;    // 2mA per LSB (REG0x28)

// Charge current limit: 40mA to 2000mA, step 40mA
static const float ICHG_STEP = 0.04f;  // 40mA
static const float ICHG_MIN = 0.04f;   // 40mA
static const float ICHG_MAX = 2.0f;    // 2000mA

// Charge voltage limit: 3500mV to 4800mV, step 10mV
static const float VBAT_STEP = 0.01f;  // 10mV
static const float VBAT_MIN = 3.5f;
static const float VBAT_MAX = 4.8f;

// Input current limit: 100mA to 3200mA, step 20mA
static const float IINDPM_STEP = 0.02f;  // 20mA
static const float IINDPM_MIN = 0.1f;    // 100mA
static const float IINDPM_MAX = 3.2f;    // 3200mA

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

  void set_charge_current_limit(float limit) { charge_current_limit_ = limit; }
  void set_charge_voltage_limit(float limit) { charge_voltage_limit_ = limit; }
  void set_input_current_limit(float limit) { input_current_limit_ = limit; }

  // Public methods for external control
  bool set_charging_enabled(bool enabled);
  bool set_charge_current(float current_amps);
  bool set_charge_voltage(float voltage_volts);
  bool set_input_current(float current_amps);

 protected:
  sensor::Sensor *bus_voltage_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *charge_current_sensor_{nullptr};
  sensor::Sensor *system_voltage_sensor_{nullptr};

  float charge_current_limit_;
  float charge_voltage_limit_;
  float input_current_limit_;

  bool read_adc_values_();
  bool configure_charger_();
  bool write_register_byte_(uint8_t reg, uint8_t value);
  bool read_register_byte_(uint8_t reg, uint8_t &value);
  bool read_register_word_(uint8_t reg, uint16_t &value);
};

}  // namespace bq25628e
}  // namespace esphome
