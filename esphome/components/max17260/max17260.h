#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace max17260 {

// MAX17260 Register Map - Complete from UG6595 and UG6597
// Page 0h
static const uint8_t MAX17260_REG_STATUS = 0x00;       // Status flags
static const uint8_t MAX17260_REG_VALRTTH = 0x01;      // Voltage alert thresholds
static const uint8_t MAX17260_REG_TALRTTH = 0x02;      // Temperature alert thresholds
static const uint8_t MAX17260_REG_SALRTTH = 0x03;      // SOC alert thresholds
static const uint8_t MAX17260_REG_ATRATE = 0x04;       // AtRate for hypothetical load prediction
static const uint8_t MAX17260_REG_REPCAP = 0x05;       // Reported remaining capacity
static const uint8_t MAX17260_REG_REPSOC = 0x06;       // Reported state of charge  
static const uint8_t MAX17260_REG_AGE = 0x07;          // Age (FullCapRep/DesignCap * 100%)
static const uint8_t MAX17260_REG_TEMP = 0x08;         // Temperature
static const uint8_t MAX17260_REG_VCELL = 0x09;        // Cell voltage
static const uint8_t MAX17260_REG_CURRENT = 0x0A;      // Instantaneous current
static const uint8_t MAX17260_REG_AVGCURRENT = 0x0B;   // Average current
static const uint8_t MAX17260_REG_QRESIDUAL = 0x0C;    // Residual capacity
static const uint8_t MAX17260_REG_MIXSOC = 0x0D;       // Mixed SOC (OCV + coulomb count)
static const uint8_t MAX17260_REG_AVSOC = 0x0E;        // Available SOC
static const uint8_t MAX17260_REG_MIXCAP = 0x0F;       // Mixed capacity

// Page 1h
static const uint8_t MAX17260_REG_FULLCAPREP = 0x10;   // Full capacity (reported)
static const uint8_t MAX17260_REG_TTE = 0x11;          // Time to empty
static const uint8_t MAX17260_REG_QRTABLE00 = 0x12;    // QRTable entry 0
static const uint8_t MAX17260_REG_FULLSOCTHR = 0x13;   // Full SOC threshold for charge detection
static const uint8_t MAX17260_REG_RCELL = 0x14;        // Cell internal resistance
static const uint8_t MAX17260_REG_AVGTA = 0x16;        // Average temperature
static const uint8_t MAX17260_REG_CYCLES = 0x17;       // Cycle counter
static const uint8_t MAX17260_REG_DESIGNCAP = 0x18;    // Design capacity
static const uint8_t MAX17260_REG_AVGVCELL = 0x19;     // Average cell voltage
static const uint8_t MAX17260_REG_MAXMINTEMP = 0x1A;   // Max/min temperature
static const uint8_t MAX17260_REG_MAXMINVOLT = 0x1B;   // Max/min voltage
static const uint8_t MAX17260_REG_MAXMINCURR = 0x1C;   // Max/min current
static const uint8_t MAX17260_REG_CONFIG = 0x1D;       // Configuration register
static const uint8_t MAX17260_REG_ICHGTERM = 0x1E;     // Charge termination current
static const uint8_t MAX17260_REG_AVCAP = 0x1F;        // Available capacity

// Page 2h
static const uint8_t MAX17260_REG_TTF = 0x20;          // Time to full
static const uint8_t MAX17260_REG_DEVNAME = 0x21;      // Device name (0x4D41 = "MA")
static const uint8_t MAX17260_REG_QRTABLE10 = 0x22;    // QRTable entry 1
static const uint8_t MAX17260_REG_FULLCAPNOM = 0x23;   // Full capacity nominal
static const uint8_t MAX17260_REG_AIN = 0x27;          // Thermistor ADC input (ratiometric)
static const uint8_t MAX17260_REG_LEARNCFG = 0x28;     // Learning configuration
static const uint8_t MAX17260_REG_FILTERCFG = 0x29;    // Filter configuration
static const uint8_t MAX17260_REG_RELAXCFG = 0x2A;     // Relaxation configuration
static const uint8_t MAX17260_REG_MISCCFG = 0x2B;      // Miscellaneous configuration
static const uint8_t MAX17260_REG_TGAIN = 0x2C;        // Thermistor gain
static const uint8_t MAX17260_REG_TOFF = 0x2D;         // Thermistor offset
static const uint8_t MAX17260_REG_CGAIN = 0x2E;        // Current gain
static const uint8_t MAX17260_REG_COFF = 0x2F;         // Current offset

// Page 3h
static const uint8_t MAX17260_REG_QRTABLE20 = 0x32;    // QRTable entry 2
static const uint8_t MAX17260_REG_DIETEMP = 0x34;      // Die temperature
static const uint8_t MAX17260_REG_FULLCAP = 0x35;      // Full capacity (present conditions)
static const uint8_t MAX17260_REG_RCOMP0 = 0x38;       // RComp0 (characterization)
static const uint8_t MAX17260_REG_TEMPCO = 0x39;       // TempCo (temperature compensation)
static const uint8_t MAX17260_REG_VEMPTY = 0x3A;       // Empty voltage
static const uint8_t MAX17260_REG_FSTAT = 0x3D;        // Fuel gauge status
static const uint8_t MAX17260_REG_TIMER = 0x3E;        // Timer LSW
static const uint8_t MAX17260_REG_SHDNTIMER = 0x3F;    // Shutdown timer

// Page 4h
static const uint8_t MAX17260_REG_QRTABLE30 = 0x42;    // QRTable entry 3
static const uint8_t MAX17260_REG_RGAIN = 0x43;        // Resistance gain for Dynamic Power
static const uint8_t MAX17260_REG_DQACC = 0x45;        // dQAcc accumulator
static const uint8_t MAX17260_REG_DPACC = 0x46;        // dPAcc accumulator
static const uint8_t MAX17260_REG_CONVGCFG = 0x49;     // Convergence configuration
static const uint8_t MAX17260_REG_VFREMCAP = 0x4A;     // Voltage filtered remaining capacity

// Page Bh
static const uint8_t MAX17260_REG_STATUS2 = 0xB0;      // Status 2 (includes SNReady)
static const uint8_t MAX17260_REG_POWER = 0xB1;        // Power
static const uint8_t MAX17260_REG_AVGPOWER = 0xB3;     // Average power
static const uint8_t MAX17260_REG_IALRTTH = 0xB4;      // Current alert thresholds
static const uint8_t MAX17260_REG_TTFCFG = 0xB5;       // Time to full configuration
static const uint8_t MAX17260_REG_CVMIXCAP = 0xB6;     // CV mix capacity
static const uint8_t MAX17260_REG_CVHALFTIME = 0xB7;   // CV half time
static const uint8_t MAX17260_REG_CGTEMPC0 = 0xB8;     // CGain temperature coefficient
static const uint8_t MAX17260_REG_CURVE = 0xB9;        // Temperature and current curve
static const uint8_t MAX17260_REG_HIBCFG = 0xBA;       // Hibernate config
static const uint8_t MAX17260_REG_CONFIG2 = 0xBB;      // Configuration 2
static const uint8_t MAX17260_REG_VRIPPLE = 0xBC;      // Voltage ripple
static const uint8_t MAX17260_REG_RIPPLECFG = 0xBD;    // Ripple configuration
static const uint8_t MAX17260_REG_TIMERH = 0xBE;       // Timer MSW

// Page Dh
static const uint8_t MAX17260_REG_RSENSE = 0xD0;       // Sense resistor (MAX17262 only)
static const uint8_t MAX17260_REG_SCOCVLIM = 0xD1;     // ScOcvLim for LiFePO4
static const uint8_t MAX17260_REG_VGAIN = 0xD2;        // Voltage gain
static const uint8_t MAX17260_REG_SOCHOLD = 0xD3;      // SOC hold configuration
static const uint8_t MAX17260_REG_MAXPEAKPOWER = 0xD4; // Max peak power (10ms) / SN Word0
static const uint8_t MAX17260_REG_SUSPEAKPOWER = 0xD5; // Sustained peak power (10s) / SN Word1
static const uint8_t MAX17260_REG_PACKRESISTANCE = 0xD6; // Pack resistance for Dynamic Power
static const uint8_t MAX17260_REG_SYSRESISTANCE = 0xD7;  // System resistance for Dynamic Power
static const uint8_t MAX17260_REG_MINSYSVOLTAGE = 0xD8;  // Minimum system voltage for Dynamic Power
static const uint8_t MAX17260_REG_MPPCURRENT = 0xD9;     // Max peak current (10ms) / SN Word2
static const uint8_t MAX17260_REG_SPPCURRENT = 0xDA;     // Sustained peak current (10s) / SN Word3
static const uint8_t MAX17260_REG_MODELCFG = 0xDB;       // Model config
static const uint8_t MAX17260_REG_ATQRESIDUAL = 0xDC;    // AtRate QResidual / SN Word4
static const uint8_t MAX17260_REG_ATTTE = 0xDD;          // AtRate time to empty / SN Word5
static const uint8_t MAX17260_REG_ATAVSOC = 0xDE;        // AtRate available SOC / SN Word6
static const uint8_t MAX17260_REG_ATAVCAP = 0xDF;        // AtRate available capacity / SN Word7

// Command register
static const uint8_t MAX17260_REG_SOFTWAKEUP = 0x60;   // Soft-wakeup command

// Serial number registers (accessible when Config2.AtRateEn=0 && Config2.DPEn=0)
// These overlap with Dynamic Power and AtRate registers (see UG6597 Table 9)
static const uint8_t MAX17260_REG_SN1 = 0xCE;  // Serial number register 1
static const uint8_t MAX17260_REG_SN2 = 0xCF;  // Serial number register 2
// Note: 0xD4 and 0xD5 are MaxPeakPower/SusPeakPower registers, NOT serial number

// Status register bit masks
static const uint16_t STATUS_POR_BIT = 0x0002;  // Power-On Reset bit (bit 1)
static const uint16_t STATUS_BR_BIT = 0x8000;   // Battery removal
static const uint16_t STATUS_BI_BIT = 0x0800;   // Battery insertion

// FSTAT register bit masks
static const uint16_t FSTAT_DNR_BIT = 0x0001;   // Data Not Ready bit (bit 0)

// ModelCfg register bit masks
static const uint16_t MODELCFG_REFRESH_BIT = 0x8000;  // Refresh bit (bit 15)
static const uint16_t MODELCFG_VCHG_BIT = 0x0400;     // VChg bit (bit 10) - set for >4.25V

// Scaling factors from datasheet Table 3
static const float VCELL_SCALE = 0.078125e-3f;         // 78.125µV per LSB
static const float CURRENT_SCALE = 1.5625e-6f / 0.025f; // 1.5625μV/RSENSE, RSENSE=25mΩ -> 62.5μA per LSB
static const float CAPACITY_SCALE = (5.0e-6f / 0.025f) * 1000.0f;  // 5μVh/RSENSE, RSENSE=25mΩ -> 0.2mAh per LSB  
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
  
  void set_device_name_sensor(text_sensor::TextSensor *sensor) { device_name_sensor_ = sensor; }
  void set_serial_number_sensor(text_sensor::TextSensor *sensor) { serial_number_sensor_ = sensor; }

  // Public register access for advanced features
  bool read_register(uint8_t reg, uint16_t &value) { return read_register_word_(reg, value); }
  bool write_register(uint8_t reg, uint16_t value) { return write_register_word_(reg, value); }
  
  // Write and verify register with retry (per UG6595 WriteAndVerifyRegister)
  bool write_and_verify_register(uint8_t reg, uint16_t value);

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
  
  text_sensor::TextSensor *device_name_sensor_{nullptr};
  text_sensor::TextSensor *serial_number_sensor_{nullptr};

  // Low-level I2C functions
  bool read_register_word_(uint8_t reg, uint16_t &value);
  bool write_register_word_(uint8_t reg, uint16_t value);
  
  // Initialization helper functions (per UG6595)
  bool check_por_bit_();
  bool wait_for_dnr_clear_();
  bool exit_hibernate_mode_(uint16_t &saved_hib_cfg);
  bool restore_hibernate_mode_(uint16_t hib_cfg);
  bool perform_ez_config_(float charge_voltage);
  bool wait_model_refresh_();
  bool clear_por_bit_();
  
  // Monitoring and maintenance
  uint16_t last_cycles_{0};  // For tracking learned parameter saving
  bool initialized_{false};   // Tracks if initialization completed successfully
  
  // Learned parameters structure for NVS storage (UG6595 Step 3.4)
  struct LearnedParameters {
    uint16_t rcomp0;        // Internal resistance compensation (0x38)
    uint16_t tempco;        // Temperature compensation (0x39)
    uint16_t fullcaprep;    // Full capacity reported (0x10)
    uint16_t cycles;        // Cycle count (0x17)
    uint16_t fullcapnom;    // Full capacity nominal (0x23)
    uint32_t checksum;      // Simple checksum for validation
  };
  
  ESPPreferenceObject learned_params_pref_;
  
  // Helper functions for learned parameter storage
  bool load_learned_parameters_();
  bool save_learned_parameters_();
  uint32_t calculate_checksum_(const LearnedParameters &params);
};

}  // namespace max17260
}  // namespace esphome
