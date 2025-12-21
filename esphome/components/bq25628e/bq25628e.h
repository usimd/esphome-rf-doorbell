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
static const uint8_t BQ25628E_REG_VSYSMIN_CTRL = 0x0E;  // Minimal System Voltage (2 bytes)
static const uint8_t BQ25628E_REG_IPRECHG_CTRL = 0x10;  // Pre-charge Control (2 bytes)
static const uint8_t BQ25628E_REG_ITERM_CTRL = 0x12;  // Termination Control (2 bytes)
static const uint8_t BQ25628E_REG_CHG_CTRL = 0x14;  // Charge Control
static const uint8_t BQ25628E_REG_CHG_TMR_CTRL = 0x15;  // Charge Timer Control
static const uint8_t BQ25628E_REG_CHARGER_CTRL_0 = 0x16;  // Charger Control 0
static const uint8_t BQ25628E_REG_CHARGER_CTRL_1 = 0x17;  // Charger Control 1
static const uint8_t BQ25628E_REG_CHARGER_CTRL_2 = 0x18;  // Charger Control 2
static const uint8_t BQ25628E_REG_CHARGER_CTRL_3 = 0x19;  // Charger Control 3
static const uint8_t BQ25628E_REG_CHG_STATUS_0 = 0x1D;  // Charger Status 0
static const uint8_t BQ25628E_REG_CHG_STATUS_1 = 0x1E;  // Charger Status 1
static const uint8_t BQ25628E_REG_FAULT_STATUS_0 = 0x1F;  // FAULT Status 0
static const uint8_t BQ25628E_REG_ADC_CTRL = 0x26;  // ADC Control
static const uint8_t BQ25628E_REG_ADC_DIS = 0x27;  // ADC Function Disable
static const uint8_t BQ25628E_REG_IBUS_ADC = 0x28;  // Input Current ADC (2 bytes)
static const uint8_t BQ25628E_REG_IBAT_ADC = 0x2A;  // Battery Current ADC (2 bytes)
static const uint8_t BQ25628E_REG_VBUS_ADC = 0x2C;  // Input Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_VBAT_ADC = 0x30;  // Battery Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_VSYS_ADC = 0x32;  // System Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_TS_ADC = 0x34;  // TS Pin ADC (2 bytes)
static const uint8_t BQ25628E_REG_TDIE_ADC = 0x36;  // Die Temperature ADC (2 bytes)
static const uint8_t BQ25628E_REG_PART_INFO = 0x38;  // Part Information

// ADC scaling factors from datasheet
static const float VBUS_ADC_STEP = 0.00397f;  // 3.97mV per LSB (REG0x2C)
static const float VBAT_ADC_STEP = 0.00199f;  // 1.99mV per LSB (REG0x30)
static const float VSYS_ADC_STEP = 0.00199f;  // 1.99mV per LSB (REG0x32)
static const float IBAT_ADC_STEP = 0.004f;    // 4mA per LSB (REG0x2A)
static const float IBUS_ADC_STEP = 0.002f;    // 2mA per LSB (REG0x28)
static const float TS_ADC_STEP = 0.0009765625f;  // 0.976562mV per LSB (REG0x34)
static const float TDIE_ADC_STEP = 0.5f;      // 0.5°C per LSB (REG0x36)
static const float TDIE_ADC_OFFSET = 40.0f;   // -40°C offset

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

// Input voltage limit (VINDPM): 3800mV to 16800mV, step 40mV
static const float VINDPM_STEP = 0.04f;  // 40mV
static const float VINDPM_MIN = 3.8f;    // 3800mV
static const float VINDPM_MAX = 16.8f;   // 16800mV

// Minimal system voltage: 2560mV to 3840mV, step 80mV
static const float VSYSMIN_STEP = 0.08f;  // 80mV
static const float VSYSMIN_MIN = 2.56f;   // 2560mV
static const float VSYSMIN_MAX = 3.84f;   // 3840mV
static const float VSYSMIN_DEFAULT = 3.52f;  // 3520mV (POR)

// Pre-charge current: 10mA to 310mA, step 10mA
static const float IPRECHG_STEP = 0.01f;  // 10mA
static const float IPRECHG_MIN = 0.01f;   // 10mA
static const float IPRECHG_MAX = 0.31f;   // 310mA
static const float IPRECHG_DEFAULT = 0.03f;  // 30mA (POR)

// Termination current: 5mA to 310mA, step 5mA
static const float ITERM_STEP = 0.005f;  // 5mA
static const float ITERM_MIN = 0.005f;   // 5mA
static const float ITERM_MAX = 0.31f;    // 310mA
static const float ITERM_DEFAULT = 0.02f;  // 20mA (POR)

// REG0x14 Charge Control bit masks
static const uint8_t CHG_CTRL_EN_TERM = 0x04;  // Bit 2: Enable termination
static const uint8_t CHG_CTRL_VINDPM_BAT_TRACK = 0x02;  // Bit 1: VINDPM battery tracking
static const uint8_t CHG_CTRL_VRECHG_MASK = 0x01;  // Bit 0: Recharge threshold

// REG0x16 Charger Control 0 bit masks
static const uint8_t CHARGER_CTRL_0_EN_CHG = 0x20;  // Bit 5: Charge enable
static const uint8_t CHARGER_CTRL_0_EN_HIZ = 0x10;  // Bit 4: HIZ mode enable
static const uint8_t CHARGER_CTRL_0_WD_RST = 0x02;  // Bit 1: Watchdog reset

// REG0x17 Charger Control 1 bit masks
static const uint8_t CHARGER_CTRL_1_REG_RST = 0x80;  // Bit 7: Register reset
static const uint8_t CHARGER_CTRL_1_TREG = 0x40;  // Bit 6: Thermal regulation (0=60C, 1=120C)

// REG0x1E Charger Status 1 bit masks
static const uint8_t CHG_STATUS_1_CHG_STAT_MASK = 0x18;  // Bits 4:3
static const uint8_t CHG_STATUS_1_VBUS_STAT_MASK = 0x07;  // Bits 2:0

// Charging status values
enum ChargeStatus {
  CHARGE_STATUS_NOT_CHARGING = 0,
  CHARGE_STATUS_FAST_CHARGING = 1,
  CHARGE_STATUS_TAPER_CHARGING = 2,
  CHARGE_STATUS_TOPOFF_CHARGING = 3
};

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
  void set_input_current_sensor(sensor::Sensor *sensor) { input_current_sensor_ = sensor; }
  void set_ts_temperature_sensor(sensor::Sensor *sensor) { ts_temperature_sensor_ = sensor; }
  void set_die_temperature_sensor(sensor::Sensor *sensor) { die_temperature_sensor_ = sensor; }

  void set_charge_current_limit(float limit) { charge_current_limit_ = limit; }
  void set_charge_voltage_limit(float limit) { charge_voltage_limit_ = limit; }
  void set_input_current_limit(float limit) { input_current_limit_ = limit; }
  void set_input_voltage_limit(float limit) { input_voltage_limit_ = limit; }
  void set_minimum_system_voltage(float voltage) { minimum_system_voltage_ = voltage; }
  void set_precharge_current(float current) { precharge_current_ = current; }
  void set_termination_current(float current) { termination_current_ = current; }
  void set_termination_enabled(bool enabled) { termination_enabled_ = enabled; }
  void set_vindpm_battery_tracking(bool enabled) { vindpm_battery_tracking_ = enabled; }
  void set_recharge_threshold(float voltage) { recharge_threshold_ = voltage; }
  void set_watchdog_timeout(uint8_t timeout) { watchdog_timeout_ = timeout; }
  void set_thermal_regulation_threshold(uint8_t threshold) { thermal_regulation_threshold_ = threshold; }

  // Public methods for external control
  bool set_charging_enabled(bool enabled);
  bool set_charge_current(float current_amps);
  bool set_charge_voltage(float voltage_volts);
  bool set_input_current(float current_amps);
  bool set_input_voltage(float voltage_volts);
  bool set_hiz_mode(bool enabled);
  bool reset_watchdog();
  bool reset_registers();

  // Status reading methods
  uint8_t get_charge_status();
  bool is_in_thermal_regulation();
  bool is_in_vindpm_regulation();
  bool is_in_iindpm_regulation();
  bool has_fault();

 protected:
  sensor::Sensor *bus_voltage_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *charge_current_sensor_{nullptr};
  sensor::Sensor *system_voltage_sensor_{nullptr};
  sensor::Sensor *input_current_sensor_{nullptr};
  sensor::Sensor *ts_temperature_sensor_{nullptr};
  sensor::Sensor *die_temperature_sensor_{nullptr};

  float charge_current_limit_;
  float charge_voltage_limit_;
  float input_current_limit_;
  float input_voltage_limit_;
  float minimum_system_voltage_;
  float precharge_current_;
  float termination_current_;
  float recharge_threshold_;
  bool termination_enabled_;
  bool vindpm_battery_tracking_;
  uint8_t watchdog_timeout_;
  uint8_t thermal_regulation_threshold_;

  bool read_adc_values_();
  bool configure_charger_();
  bool write_register_byte_(uint8_t reg, uint8_t value);
  bool read_register_byte_(uint8_t reg, uint8_t &value);
  bool read_register_word_(uint8_t reg, uint16_t &value);
  bool write_register_word_(uint8_t reg, uint16_t value);
};

}  // namespace bq25628e
}  // namespace esphome
