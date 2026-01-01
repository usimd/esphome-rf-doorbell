#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bq25628e {

// BQ25628E Register Map - from SLUSFA4C datasheet
static const uint8_t BQ25628E_REG_ICHG_CTRL = 0x02;  // Charge Current Limit
static const uint8_t BQ25628E_REG_VBAT_CTRL = 0x04;  // Charge Voltage Limit
static const uint8_t BQ25628E_REG_IINDPM_CTRL = 0x06;  // Input Current Limit
static const uint8_t BQ25628E_REG_VINDPM_CTRL = 0x08;  // Input Voltage Limit
// NOTE: Registers 0x0A-0x0D do not exist in BQ25628E (gap in register map)
// VBUS OVP is controlled by bit 0 of register 0x17 (CHARGER_CTRL_1)
static const uint8_t BQ25628E_REG_VSYSMIN_CTRL = 0x0E;  // Minimal System Voltage (2 bytes)
static const uint8_t BQ25628E_REG_IPRECHG_CTRL = 0x10;  // Pre-charge Control (2 bytes)
static const uint8_t BQ25628E_REG_ITERM_CTRL = 0x12;  // Termination Control (2 bytes)
static const uint8_t BQ25628E_REG_CHG_CTRL = 0x14;  // Charge Control
static const uint8_t BQ25628E_REG_CHG_TMR_CTRL = 0x15;  // Charge Timer Control
static const uint8_t BQ25628E_REG_CHARGER_CTRL_0 = 0x16;  // Charger Control 0
static const uint8_t BQ25628E_REG_CHARGER_CTRL_1 = 0x17;  // Charger Control 1
static const uint8_t BQ25628E_REG_CHARGER_CTRL_2 = 0x18;  // Charger Control 2
static const uint8_t BQ25628E_REG_CHARGER_CTRL_3 = 0x19;  // Charger Control 3
static const uint8_t BQ25628E_REG_NTC_CTRL_0 = 0x1A;  // NTC Control 0
static const uint8_t BQ25628E_REG_NTC_CTRL_1 = 0x1B;  // NTC Control 1
static const uint8_t BQ25628E_REG_NTC_CTRL_2 = 0x1C;  // NTC Control 2
static const uint8_t BQ25628E_REG_CHG_STATUS_0 = 0x1D;  // Charger Status 0
static const uint8_t BQ25628E_REG_CHARGER_STATUS_0 = 0x1D;  // Charger Status 0 (alias)
static const uint8_t BQ25628E_REG_CHG_STATUS_1 = 0x1E;  // Charger Status 1
static const uint8_t BQ25628E_REG_CHARGER_STATUS_1 = 0x1E;  // Charger Status 1 (alias)
static const uint8_t BQ25628E_REG_FAULT_STATUS_0 = 0x1F;  // FAULT Status 0
static const uint8_t BQ25628E_REG_ADC_CTRL = 0x26;  // ADC Control
static const uint8_t BQ25628E_REG_ADC_CONTROL = 0x26;  // ADC Control (alias)
static const uint8_t BQ25628E_REG_ADC_DIS = 0x27;  // ADC Function Disable
static const uint8_t BQ25628E_REG_ADC_FUNCTION_DISABLE_0 = 0x27;  // ADC Function Disable (alias)
static const uint8_t BQ25628E_REG_IBUS_ADC = 0x28;  // Input Current ADC (2 bytes)
static const uint8_t BQ25628E_REG_IBAT_ADC = 0x2A;  // Battery Current ADC (2 bytes)
static const uint8_t BQ25628E_REG_VBUS_ADC = 0x2C;  // Input Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_VBAT_ADC = 0x30;  // Battery Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_VSYS_ADC = 0x32;  // System Voltage ADC (2 bytes)
static const uint8_t BQ25628E_REG_TS_ADC = 0x34;  // TS Pin ADC (2 bytes)
static const uint8_t BQ25628E_REG_TDIE_ADC = 0x36;  // Die Temperature ADC (2 bytes)
static const uint8_t BQ25628E_REG_VPMID_ADC = 0x38;  // PMID Voltage ADC (2 bytes, overlaps with PART_INFO)
static const uint8_t BQ25628E_REG_PART_INFO = 0x38;  // Part Information
static const uint8_t BQ25628E_REG_PART_INFORMATION = 0x38;  // Part Information (alias)

// Mask registers for interrupt control
static const uint8_t BQ25628E_REG_CHARGER_MASK_0 = 0x23;  // Charger Status Mask 0
static const uint8_t BQ25628E_REG_CHARGER_MASK_1 = 0x24;  // Charger Status Mask 1  
static const uint8_t BQ25628E_REG_FAULT_MASK_0 = 0x25;    // Fault Status Mask 0

// Flag registers for interrupt status
static const uint8_t BQ25628E_REG_CHARGER_FLAG_0 = 0x20;  // Charger Status Flag 0
static const uint8_t BQ25628E_REG_CHARGER_FLAG_1 = 0x21;  // Charger Status Flag 1
static const uint8_t BQ25628E_REG_FAULT_FLAG_0 = 0x22;    // Fault Status Flag 0

// ADC scaling factors from datasheet
static const float VBUS_ADC_STEP = 0.00397f;  // 3.97mV per LSB (REG0x2C)
static const float VBAT_ADC_STEP = 0.00199f;  // 1.99mV per LSB (REG0x30)
static const float VSYS_ADC_STEP = 0.00199f;  // 1.99mV per LSB (REG0x32)
static const float IBAT_ADC_STEP = 0.004f;    // 4mA per LSB (REG0x2A)
static const float IBUS_ADC_STEP = 0.002f;    // 2mA per LSB (REG0x28)
static const float TS_ADC_STEP = 0.0009765625f;  // 0.976562mV per LSB (REG0x34)
static const float TDIE_ADC_STEP = 0.5f;      // 0.5°C per LSB (REG0x36)
// Note: TDIE has NO offset - POR 0h = 0°C, 2's complement, range -40°C to 140°C (Table 8-41)

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
static const uint8_t CHG_CTRL_Q1_FULLON = 0x80;         // Bit 7: Force RBFET low resistance
static const uint8_t CHG_CTRL_Q4_FULLON = 0x40;         // Bit 6: Force BATFET low resistance
static const uint8_t CHG_CTRL_ITRICKLE = 0x20;          // Bit 5: Trickle current (0=10mA, 1=40mA)
static const uint8_t CHG_CTRL_TOPOFF_TMR_MASK = 0x18;   // Bits 4:3: Top-off timer
static const uint8_t CHG_CTRL_TOPOFF_TMR_SHIFT = 3;
static const uint8_t CHG_CTRL_EN_TERM = 0x04;           // Bit 2: Enable termination
static const uint8_t CHG_CTRL_VINDPM_BAT_TRACK = 0x02;  // Bit 1: VINDPM battery tracking
static const uint8_t CHG_CTRL_VRECHG = 0x01;            // Bit 0: Recharge threshold (0=100mV, 1=200mV)
static const uint8_t CHG_CTRL_VRECHG_MASK = 0x03;       // Bits 1:0: Recharge threshold mask
static const uint8_t CHG_CTRL_VRECHG_SHIFT = 0;

// Top-off timer values
enum TopOffTimer {
  TOPOFF_DISABLED = 0,  // 00b
  TOPOFF_17MIN = 1,     // 01b
  TOPOFF_35MIN = 2,     // 10b
  TOPOFF_52MIN = 3      // 11b
};

// REG0x15 Charge Timer Control bit masks
static const uint8_t CHG_TMR_CTRL_DIS_STAT = 0x80;          // Bit 7: Disable STAT pin
static const uint8_t CHG_TMR_CTRL_TMR2X_EN = 0x08;          // Bit 3: 2X timer during DPM/thermal reg
static const uint8_t CHG_TMR_CTRL_EN_SAFETY_TMRS = 0x04;    // Bit 2: Enable safety timers
static const uint8_t CHG_TMR_CTRL_PRECHG_TMR = 0x02;        // Bit 1: Precharge timer (0=2.5hrs, 1=0.62hrs)
static const uint8_t CHG_TMR_CTRL_CHG_TMR = 0x01;           // Bit 0: Fast charge timer (0=14.5hrs, 1=28hrs)

// Charger Status 1 bit masks (forward declarations for compatibility)
static const uint8_t CHARGER_STATUS_1_CHG_STAT_MASK = 0x38;   // Bits 5:3: Charge status
static const uint8_t CHARGER_STATUS_1_CHG_STAT_SHIFT = 3;
static const uint8_t CHG_STATUS_1_CHG_STAT_MASK = 0x38;  // Alias for compatibility

// REG0x16 Charger Control 0 bit masks
static const uint8_t CHARGER_CTRL_0_EN_AUTO_IBATDIS = 0x80; // Bit 7: Auto battery discharge during OVP
static const uint8_t CHARGER_CTRL_0_FORCE_IBATDIS = 0x40;   // Bit 6: Force battery discharge (~30mA)
static const uint8_t CHARGER_CTRL_0_EN_CHG = 0x20;          // Bit 5: Charge enable
static const uint8_t CHARGER_CTRL_0_EN_HIZ = 0x10;          // Bit 4: HIZ mode enable
static const uint8_t CHARGER_CTRL_0_FORCE_PMID_DIS = 0x08;  // Bit 3: Force PMID discharge (~30mA)
static const uint8_t CHARGER_CTRL_0_WD_RST = 0x04;          // Bit 2: Watchdog reset
static const uint8_t CHARGER_CTRL_0_WATCHDOG_MASK = 0x03;   // Bits 1:0: Watchdog timer
static const uint8_t CHARGER_CTRL_0_WATCHDOG_SHIFT = 0;

// Watchdog timer values
enum WatchdogTimer {
  WATCHDOG_DISABLED = 0,  // 00b
  WATCHDOG_50S = 1,       // 01b (default)
  WATCHDOG_100S = 2,      // 10b
  WATCHDOG_200S = 3       // 11b
};

// REG0x17 Charger Control 1 bit masks
static const uint8_t CHARGER_CTRL_1_REG_RST = 0x80;           // Bit 7: Register reset
static const uint8_t CHARGER_CTRL_1_TREG = 0x40;              // Bit 6: Thermal regulation (0=60C, 1=120C)
static const uint8_t CHARGER_CTRL_1_CONV_FREQ_MASK = 0x30;    // Bits 5:4: Switching frequency
static const uint8_t CHARGER_CTRL_1_CONV_FREQ_SHIFT = 4;
static const uint8_t CHARGER_CTRL_1_CONV_STRN_MASK = 0x0C;    // Bits 3:2: Drive strength
static const uint8_t CHARGER_CTRL_1_CONV_STRN_SHIFT = 2;
static const uint8_t CHARGER_CTRL_1_CONV_STRN_STRONG = 0x0C;  // 11b = strong
static const uint8_t CHARGER_CTRL_1_VBUS_OVP = 0x01;          // Bit 0: VBUS OVP (0=6.3V, 1=18.5V)
static const uint8_t CHARGER_CTRL_1_VBUS_OVP_18V = 0x01;      // 18.5V threshold

// Switching frequency values
enum SwitchingFrequency {
  FREQ_NOMINAL = 0,    // 00b = 1.5 MHz (default)
  FREQ_MINUS_10 = 1,   // 01b = 1.35 MHz (-10%)
  FREQ_PLUS_10 = 2,    // 10b = 1.65 MHz (+10%)
  FREQ_RESERVED = 3    // 11b = RESERVED
};

// Drive strength values
enum DriveStrength {
  DRIVE_WEAK = 0,      // 00b
  DRIVE_NORMAL = 1,    // 01b
  DRIVE_RESERVED = 2,  // 10b
  DRIVE_STRONG = 3     // 11b (default)
};

// Thermal regulation threshold values
enum ThermalThreshold {
  THERMAL_60C = 0,   // 0b
  THERMAL_120C = 1   // 1b (default)
};

// VBUS OVP threshold values
enum VbusOvpThreshold {
  VBUS_OVP_6_3V = 0,   // 0b
  VBUS_OVP_18_5V = 1   // 1b (default)
};

// REG0x18 Charger Control 2 bit masks
static const uint8_t CHARGER_CTRL_2_PFM_FWD_DIS = 0x10;       // Bit 4: Disable PFM in forward buck
static const uint8_t CHARGER_CTRL_2_BATFET_CTRL_WVBUS = 0x08; // Bit 3: BATFET control with VBUS
static const uint8_t CHARGER_CTRL_2_BATFET_DLY = 0x04;        // Bit 2: BATFET delay (0=25ms, 1=12.5s)
static const uint8_t CHARGER_CTRL_2_BATFET_CTRL_MASK = 0x03;  // Bits 1:0: BATFET control mode
static const uint8_t CHARGER_CTRL_2_BATFET_CTRL_SHIFT = 0;

// BATFET control modes
enum BatfetControl {
  BATFET_NORMAL = 0,        // 00b (default)
  BATFET_SHUTDOWN = 1,      // 01b - Shutdown Mode
  BATFET_SHIP = 2,          // 10b - Ship Mode
  BATFET_SYSTEM_RESET = 3   // 11b - System Power Reset
};

// BATFET delay values
enum BatfetDelay {
  BATFET_DELAY_25MS = 0,    // 0b
  BATFET_DELAY_12_5S = 1    // 1b (default)
};

// REG 0x19 - CHARGER_CTRL_3 bit masks
static const uint8_t CHARGER_CTRL_3_IBAT_PK_MASK = 0xC0;  // bits 7:6
static const uint8_t CHARGER_CTRL_3_IBAT_PK_SHIFT = 6;
static const uint8_t CHARGER_CTRL_3_VBAT_UVLO = 0x20;     // bit 5
static const uint8_t CHARGER_CTRL_3_EN_EXTILIM = 0x04;    // bit 2
static const uint8_t CHARGER_CTRL_3_CHG_RATE_MASK = 0x03; // bits 1:0
static const uint8_t CHARGER_CTRL_3_CHG_RATE_SHIFT = 0;

enum IbatPkThreshold {
  IBAT_PK_6A = 2,   // 10b
  IBAT_PK_12A = 3   // 11b (default)
};

enum VbatUvloThreshold {
  VBAT_UVLO_2_2V = 0,  // 2.2V UVLO, 2.05V SHORT (default)
  VBAT_UVLO_1_8V = 1   // 1.8V UVLO, 1.85V SHORT
};

enum ChargeRate {
  CHG_RATE_1C = 0,  // 00b (default)
  CHG_RATE_2C = 1,  // 01b
  CHG_RATE_4C = 2,  // 10b
  CHG_RATE_6C = 3   // 11b
};

// REG 0x1A - NTC_CTRL_0 bit masks
static const uint8_t NTC_CTRL_0_TS_IGNORE = 0x80;       // Bit 7: Ignore TS function
static const uint8_t NTC_CTRL_0_TS_ISET_WARM_MASK = 0x0C;  // Bits 3:2: TS warm current setting
static const uint8_t NTC_CTRL_0_TS_ISET_WARM_SHIFT = 2;
static const uint8_t NTC_CTRL_0_TS_ISET_COOL_MASK = 0x03;  // Bits 1:0: TS cool current setting
static const uint8_t NTC_CTRL_0_TS_ISET_COOL_SHIFT = 0;

enum TsCurrentSetting {
  TS_CURRENT_SUSPEND = 0,      // 00b - Charge suspend
  TS_CURRENT_ICHG_20PCT = 1,   // 01b - Set ICHG to 20%
  TS_CURRENT_ICHG_40PCT = 2,   // 10b - Set ICHG to 40%
  TS_CURRENT_ICHG_UNCHANGED = 3 // 11b - ICHG unchanged
};

// REG 0x1B - NTC_CTRL_1 bit masks
static const uint8_t NTC_CTRL_1_TS_TH123_MASK = 0xE0;  // Bits 7:5: TH1/TH2/TH3 cold thresholds
static const uint8_t NTC_CTRL_1_TS_TH123_SHIFT = 5;
static const uint8_t NTC_CTRL_1_TS_TH456_MASK = 0x1C;  // Bits 4:2: TH4/TH5/TH6 hot thresholds
static const uint8_t NTC_CTRL_1_TS_TH456_SHIFT = 2;
static const uint8_t NTC_CTRL_1_TS_VSET_WARM_MASK = 0x03;  // Bits 1:0: TS warm voltage setting
static const uint8_t NTC_CTRL_1_TS_VSET_WARM_SHIFT = 0;

enum TsColdThresholds {
  TS_COLD_0C_5C_15C = 0,    // 000b - TH1:0°C, TH2:5°C, TH3:15°C
  TS_COLD_0C_10C_15C = 1,   // 001b - TH1:0°C, TH2:10°C, TH3:15°C (default)
  TS_COLD_0C_15C_20C = 2,   // 010b - TH1:0°C, TH2:15°C, TH3:20°C
  TS_COLD_0C_20C_20C = 3,   // 011b - TH1:0°C, TH2:20°C, TH3:20°C
  TS_COLD_M5C_5C_15C = 4,   // 100b - TH1:-5°C, TH2:5°C, TH3:15°C
  TS_COLD_M5C_10C_15C = 5,  // 101b - TH1:-5°C, TH2:10°C, TH3:15°C
  TS_COLD_M5C_10C_20C = 6,  // 110b - TH1:-5°C, TH2:10°C, TH3:20°C
  TS_COLD_0C_10C_20C = 7    // 111b - TH1:0°C, TH2:10°C, TH3:20°C
};

enum TsHotThresholds {
  TS_HOT_35C_40C_60C = 0,   // 000b - TH4:35°C, TH5:40°C, TH6:60°C
  TS_HOT_35C_45C_60C = 1,   // 001b - TH4:35°C, TH5:45°C, TH6:60°C (default)
  TS_HOT_35C_50C_60C = 2,   // 010b - TH4:35°C, TH5:50°C, TH6:60°C
  TS_HOT_40C_55C_60C = 3,   // 011b - TH4:40°C, TH5:55°C, TH6:60°C
  TS_HOT_35C_40C_50C = 4,   // 100b - TH4:35°C, TH5:40°C, TH6:50°C
  TS_HOT_35C_45C_50C = 5,   // 101b - TH4:35°C, TH5:45°C, TH6:50°C
  TS_HOT_40C_45C_60C = 6,   // 110b - TH4:40°C, TH5:45°C, TH6:60°C
  TS_HOT_40C_50C_60C = 7    // 111b - TH4:40°C, TH5:50°C, TH6:60°C
};

enum TsWarmVoltageSetting {
  TS_WARM_VREG_MINUS_300MV = 0,  // 00b - VREG-300mV
  TS_WARM_VREG_MINUS_200MV = 1,  // 01b - VREG-200mV (default)
  TS_WARM_VREG_MINUS_100MV = 2,  // 10b - VREG-100mV
  TS_WARM_VREG_UNCHANGED = 3     // 11b - VREG unchanged
};

// REG 0x1C - NTC_CTRL_2 bit masks
static const uint8_t NTC_CTRL_2_TS_VSET_SYM = 0x40;          // Bit 6: Symmetric voltage setting
static const uint8_t NTC_CTRL_2_TS_VSET_PREWARM_MASK = 0x30; // Bits 5:4: TS prewarm voltage
static const uint8_t NTC_CTRL_2_TS_VSET_PREWARM_SHIFT = 4;
static const uint8_t NTC_CTRL_2_TS_ISET_PREWARM_MASK = 0x0C; // Bits 3:2: TS prewarm current
static const uint8_t NTC_CTRL_2_TS_ISET_PREWARM_SHIFT = 2;
static const uint8_t NTC_CTRL_2_TS_ISET_PRECOOL_MASK = 0x03; // Bits 1:0: TS precool current
static const uint8_t NTC_CTRL_2_TS_ISET_PRECOOL_SHIFT = 0;

// REG 0x1D - CHARGER_STATUS_0 bit masks (read-only)
static const uint8_t CHARGER_STATUS_0_ADC_DONE = 0x40;      // Bit 6: ADC conversion done
static const uint8_t CHARGER_STATUS_0_TREG_STAT = 0x20;     // Bit 5: Thermal regulation
static const uint8_t CHARGER_STATUS_0_VSYS_STAT = 0x10;     // Bit 4: VSYS regulation
static const uint8_t CHARGER_STATUS_0_IINDPM_STAT = 0x08;   // Bit 3: IINDPM/ILIM regulation
static const uint8_t CHARGER_STATUS_0_VINDPM_STAT = 0x04;   // Bit 2: VINDPM regulation
static const uint8_t CHARGER_STATUS_0_SAFETY_TMR = 0x02;    // Bit 1: Safety timer expired
static const uint8_t CHARGER_STATUS_0_WD_STAT = 0x01;       // Bit 0: Watchdog timer expired

// REG 0x1E - CHARGER_STATUS_1 bit masks (read-only)
// Note: Using values from earlier definition (0x38 mask, shift 3)
static const uint8_t CHARGER_STATUS_1_VBUS_STAT_MASK = 0x07;  // Bits 2:0: VBUS status
static const uint8_t CHARGER_STATUS_1_VBUS_STAT_SHIFT = 0;

// Charging status values
enum ChargeStatus {
  CHARGE_STATUS_NOT_CHARGING = 0,      // 00b - Not charging or terminated
  CHARGE_STATUS_TRICKLE_PRECHARGE = 1, // 01b - Trickle/Pre/Fast charge (CC mode)
  CHARGE_STATUS_TAPER_CHARGE = 2,      // 10b - Taper charge (CV mode)
  CHARGE_STATUS_TOPOFF_TIMER = 3       // 11b - Top-off timer active
};

// VBUS status values (partial - datasheet may have more)
enum VbusStatus {
  VBUS_NOT_POWERED = 0,     // 000b - Not powered from VBUS
  VBUS_UNKNOWN_ADAPTER = 4  // 100b - Unknown adapter (default IINDPM)
};

// REG 0x1F - FAULT_STATUS_0 bit masks (read-only)
static const uint8_t FAULT_STATUS_0_VBUS_FAULT = 0x80;   // Bit 7: VBUS fault
static const uint8_t FAULT_STATUS_0_BAT_FAULT = 0x40;    // Bit 6: Battery fault
static const uint8_t FAULT_STATUS_0_SYS_FAULT = 0x20;    // Bit 5: System fault
static const uint8_t FAULT_STATUS_0_TSHUT_STAT = 0x08;   // Bit 3: Thermal shutdown
static const uint8_t FAULT_STATUS_0_TS_STAT_MASK = 0x07; // Bits 2:0: TS temperature zone
static const uint8_t FAULT_STATUS_0_TS_STAT_SHIFT = 0;

enum TsTemperatureZone {
  TS_ZONE_NORMAL = 0,      // 000b - Normal temperature
  TS_ZONE_COLD = 1,        // 001b - Cold or TS power rail not available
  TS_ZONE_HOT = 2,         // 010b - Hot
  TS_ZONE_COOL = 3,        // 011b - Cool
  TS_ZONE_WARM = 4,        // 100b - Warm
  TS_ZONE_PRECOOL = 5,     // 101b - Pre-cool
  TS_ZONE_PREWARM = 6,     // 110b - Pre-warm
  TS_ZONE_BIAS_FAULT = 7   // 111b - TS pin bias reference fault
};

// REG 0x20 - CHARGER_FLAG_0 bit masks (read-only)
static const uint8_t CHARGER_FLAG_0_ADC_DONE = 0x40;      // Bit 6: ADC conversion done flag
static const uint8_t CHARGER_FLAG_0_TREG_FLAG = 0x20;     // Bit 5: Thermal regulation flag
static const uint8_t CHARGER_FLAG_0_VSYS_FLAG = 0x10;     // Bit 4: VSYS regulation flag
static const uint8_t CHARGER_FLAG_0_IINDPM_FLAG = 0x08;   // Bit 3: IINDPM/ILIM regulation flag
static const uint8_t CHARGER_FLAG_0_VINDPM_FLAG = 0x04;   // Bit 2: VINDPM regulation flag
static const uint8_t CHARGER_FLAG_0_SAFETY_TMR = 0x02;    // Bit 1: Safety timer flag
static const uint8_t CHARGER_FLAG_0_WD_FLAG = 0x01;       // Bit 0: Watchdog timer flag

// REG 0x21 - CHARGER_FLAG_1 bit masks (read-only)
static const uint8_t CHARGER_FLAG_1_CHG_FLAG = 0x08;      // Bit 3: Charge status changed flag
static const uint8_t CHARGER_FLAG_1_VBUS_FLAG = 0x01;     // Bit 0: VBUS status changed flag

// REG 0x22 - FAULT_FLAG_0 bit masks (read-only)
static const uint8_t FAULT_FLAG_0_VBUS_FAULT = 0x80;      // Bit 7: VBUS fault flag
static const uint8_t FAULT_FLAG_0_BAT_FAULT = 0x40;       // Bit 6: Battery fault flag
static const uint8_t FAULT_FLAG_0_SYS_FAULT = 0x20;       // Bit 5: System fault flag
static const uint8_t FAULT_FLAG_0_TSHUT_FLAG = 0x08;      // Bit 3: Thermal shutdown flag
static const uint8_t FAULT_FLAG_0_TS_FLAG = 0x01;         // Bit 0: TS status change flag

// REG 0x23 - CHARGER_MASK_0 bit masks (read/write)
static const uint8_t CHARGER_MASK_0_ADC_DONE = 0x40;      // Bit 6: ADC conversion done mask
static const uint8_t CHARGER_MASK_0_TREG_MASK = 0x20;     // Bit 5: Thermal regulation mask
static const uint8_t CHARGER_MASK_0_VSYS_MASK = 0x10;     // Bit 4: VSYS regulation mask
static const uint8_t CHARGER_MASK_0_IINDPM_MASK = 0x08;   // Bit 3: IINDPM/ILIM mask
static const uint8_t CHARGER_MASK_0_VINDPM_MASK = 0x04;   // Bit 2: VINDPM mask
static const uint8_t CHARGER_MASK_0_SAFETY_TMR = 0x02;    // Bit 1: Safety timer mask
static const uint8_t CHARGER_MASK_0_WD_MASK = 0x01;       // Bit 0: Watchdog timer mask

// REG 0x24 - CHARGER_MASK_1 bit masks (read/write)
static const uint8_t CHARGER_MASK_1_CHG_MASK = 0x08;      // Bit 3: Charge status change mask
static const uint8_t CHARGER_MASK_1_VBUS_MASK = 0x01;     // Bit 0: VBUS status change mask

// REG 0x25 - FAULT_MASK_0 bit masks (read/write)
static const uint8_t FAULT_MASK_0_VBUS_FAULT = 0x80;      // Bit 7: VBUS fault mask
static const uint8_t FAULT_MASK_0_BAT_FAULT = 0x40;       // Bit 6: Battery fault mask
static const uint8_t FAULT_MASK_0_SYS_FAULT = 0x20;       // Bit 5: System fault mask
static const uint8_t FAULT_MASK_0_TSHUT_MASK = 0x08;      // Bit 3: Thermal shutdown mask
static const uint8_t FAULT_MASK_0_TS_MASK = 0x01;         // Bit 0: TS temperature zone mask

// REG 0x26 - ADC_CONTROL bit masks (read/write)
static const uint8_t ADC_CONTROL_ADC_EN = 0x80;           // Bit 7: ADC enable
static const uint8_t ADC_CONTROL_ADC_RATE = 0x40;         // Bit 6: ADC conversion rate
static const uint8_t ADC_CONTROL_ADC_SAMPLE_MASK = 0x30;  // Bits 5:4: ADC sample speed
static const uint8_t ADC_CONTROL_ADC_SAMPLE_SHIFT = 4;
static const uint8_t ADC_CONTROL_ADC_AVG = 0x08;          // Bit 3: ADC average control
static const uint8_t ADC_CONTROL_ADC_AVG_INIT = 0x04;     // Bit 2: ADC average init

enum AdcSampleSpeed {
  ADC_SAMPLE_12BIT = 0,  // 00b - 12-bit effective resolution
  ADC_SAMPLE_11BIT = 1,  // 01b - 11-bit effective resolution
  ADC_SAMPLE_10BIT = 2,  // 10b - 10-bit effective resolution
  ADC_SAMPLE_9BIT = 3    // 11b - 9-bit effective resolution (default)
};

// REG 0x27 - ADC_FUNCTION_DISABLE_0 bit masks (read/write)
static const uint8_t ADC_FUNCTION_DIS_IBUS = 0x80;   // Bit 7: IBUS ADC disable
static const uint8_t ADC_FUNCTION_DIS_IBAT = 0x40;   // Bit 6: IBAT ADC disable
static const uint8_t ADC_FUNCTION_DIS_VBUS = 0x20;   // Bit 5: VBUS ADC disable
static const uint8_t ADC_FUNCTION_DIS_VBAT = 0x10;   // Bit 4: VBAT ADC disable
static const uint8_t ADC_FUNCTION_DIS_VSYS = 0x08;   // Bit 3: VSYS ADC disable
static const uint8_t ADC_FUNCTION_DIS_TS = 0x04;     // Bit 2: TS ADC disable
static const uint8_t ADC_FUNCTION_DIS_TDIE = 0x02;   // Bit 1: TDIE ADC disable
static const uint8_t ADC_FUNCTION_DIS_VPMID = 0x01;  // Bit 0: VPMID ADC disable

// REG 0x38 - PART_INFORMATION bit masks (read-only)
static const uint8_t PART_INFO_PN_MASK = 0x38;       // Bits 5:3: Part number
static const uint8_t PART_INFO_PN_SHIFT = 3;
static const uint8_t PART_INFO_DEV_REV_MASK = 0x07;  // Bits 2:0: Device revision
static const uint8_t PART_INFO_DEV_REV_SHIFT = 0;
static const uint8_t PART_INFO_PN_BQ25628E = 0x04;   // 4h = BQ25628E

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
  void set_ts_voltage_sensor(sensor::Sensor *sensor) { ts_voltage_sensor_ = sensor; }
  void set_die_temperature_sensor(sensor::Sensor *sensor) { die_temperature_sensor_ = sensor; }
  void set_ts_monitoring_enabled(bool enabled) { ts_monitoring_enabled_ = enabled; }
  
  void set_part_number_sensor(text_sensor::TextSensor *sensor) { part_number_sensor_ = sensor; }
  void set_device_revision_sensor(text_sensor::TextSensor *sensor) { device_revision_sensor_ = sensor; }

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
  
  // Public register access for YAML lambdas
  bool read_register_byte(uint8_t reg, uint8_t *value) { return read_register_byte_(reg, *value); }
  bool read_register_word(uint8_t reg, uint16_t *value) { return read_register_word_(reg, *value); }

 protected:
  sensor::Sensor *bus_voltage_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *charge_current_sensor_{nullptr};
  sensor::Sensor *system_voltage_sensor_{nullptr};
  sensor::Sensor *input_current_sensor_{nullptr};
  sensor::Sensor *ts_temperature_sensor_{nullptr};
  sensor::Sensor *ts_voltage_sensor_{nullptr};
  sensor::Sensor *die_temperature_sensor_{nullptr};
  
  text_sensor::TextSensor *part_number_sensor_{nullptr};
  text_sensor::TextSensor *device_revision_sensor_{nullptr};

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
  bool ts_monitoring_enabled_{true};

  bool read_adc_values_();
  bool configure_charger_();
  bool configure_charger_optimized_();  // Optimized boot configuration with minimal I2C ops
  bool write_register_byte_(uint8_t reg, uint8_t value);
  bool read_register_byte_(uint8_t reg, uint8_t &value);
  bool read_register_word_(uint8_t reg, uint16_t &value);
  bool write_register_word_(uint8_t reg, uint16_t value);

  // Protected register-specific setters with validation
  bool set_charge_current_reg_(float current);      // REG 0x02/0x03 ICHG (40mA-2000mA, 40mA steps)
  bool set_charge_voltage_reg_(float voltage);      // REG 0x04/0x05 VBAT (varies by step)
  bool set_input_current_limit_reg_(float current); // REG 0x06/0x07 IINDPM (varies by step)
  bool set_input_voltage_limit_reg_(float voltage); // REG 0x08/0x09 VINDPM (varies by step)
  bool set_minimum_system_voltage_reg_(float voltage); // REG 0x0E/0x0F VSYSMIN (varies by step)
  bool set_precharge_current_reg_(float current);   // REG 0x10/0x11 IPRECHG (varies by step)
  bool set_termination_current_reg_(float current); // REG 0x12/0x13 ITERM (varies by step)
  
  // REG 0x14 CHG_CTRL - Charge Control Register
  bool set_charge_control_reg_(uint8_t value);      // Write full register
  bool get_charge_control_reg_(uint8_t &value);     // Read full register
  bool set_q1_fullon_(bool enable);                 // Bit 7: Force RBFET low resistance
  bool set_q4_fullon_(bool enable);                 // Bit 6: Force BATFET low resistance
  bool set_trickle_current_(bool high);             // Bit 5: 0=10mA, 1=40mA
  bool set_topoff_timer_(TopOffTimer timer);        // Bits 4:3: Top-off timer
  bool set_termination_enabled_(bool enable);       // Bit 2: Enable termination
  bool set_vindpm_bat_track_(bool enable);          // Bit 1: VINDPM battery tracking
  bool set_recharge_threshold_(bool high);          // Bit 0: 0=100mV, 1=200mV
  bool get_q1_fullon_(bool &enabled);
  bool get_q4_fullon_(bool &enabled);
  bool get_trickle_current_(bool &high);
  bool get_topoff_timer_(TopOffTimer &timer);
  bool get_termination_enabled_(bool &enabled);
  bool get_vindpm_bat_track_(bool &enabled);
  bool get_recharge_threshold_(bool &high);

  // REG 0x15 CHG_TMR_CTRL - Charge Timer Control Register
  bool set_charge_timer_control_reg_(uint8_t value);   // Write full register
  bool get_charge_timer_control_reg_(uint8_t &value);  // Read full register
  bool set_stat_pin_enabled_(bool enable);             // Bit 7: Enable/disable STAT pin
  bool set_timer_2x_enabled_(bool enable);             // Bit 3: 2X timer during DPM/thermal
  bool set_safety_timers_enabled_(bool enable);        // Bit 2: Enable safety timers
  bool set_precharge_timer_(bool long_timer);          // Bit 1: 0=2.5hrs, 1=0.62hrs
  bool set_fast_charge_timer_(bool long_timer);        // Bit 0: 0=14.5hrs, 1=28hrs
  bool get_stat_pin_enabled_(bool &enabled);
  bool get_timer_2x_enabled_(bool &enabled);
  bool get_safety_timers_enabled_(bool &enabled);
  bool get_precharge_timer_(bool &long_timer);
  bool get_fast_charge_timer_(bool &long_timer);

  // REG 0x16 CHARGER_CTRL_0 - Charger Control 0 Register
  bool set_charger_ctrl_0_reg_(uint8_t value);         // Write full register
  bool get_charger_ctrl_0_reg_(uint8_t &value);        // Read full register
  bool set_auto_ibatdis_enabled_(bool enable);         // Bit 7: Auto discharge during OVP
  bool set_force_ibatdis_(bool enable);                // Bit 6: Force battery discharge
  bool set_charging_enabled_ctrl_(bool enable);        // Bit 5: Charge enable
  bool set_hiz_mode_ctrl_(bool enable);                // Bit 4: HIZ mode
  bool set_force_pmid_discharge_(bool enable);         // Bit 3: Force PMID discharge
  bool reset_watchdog_ctrl_();                         // Bit 2: Reset watchdog
  bool set_watchdog_timer_(WatchdogTimer timer);       // Bits 1:0: Watchdog timer
  bool get_auto_ibatdis_enabled_(bool &enabled);
  bool get_force_ibatdis_(bool &enabled);
  bool get_charging_enabled_ctrl_(bool &enabled);
  bool get_hiz_mode_ctrl_(bool &enabled);
  bool get_force_pmid_discharge_(bool &enabled);
  bool get_watchdog_timer_(WatchdogTimer &timer);

  // REG 0x17 CHARGER_CTRL_1 - Charger Control 1 Register
  bool set_charger_ctrl_1_reg_(uint8_t value);         // Write full register
  bool get_charger_ctrl_1_reg_(uint8_t &value);        // Read full register
  bool reset_registers_ctrl_();                        // Bit 7: Reset registers to default
  bool set_thermal_threshold_(ThermalThreshold threshold); // Bit 6: 60C or 120C
  bool set_switching_frequency_(SwitchingFrequency freq);  // Bits 5:4: Converter frequency
  bool set_drive_strength_(DriveStrength strength);    // Bits 3:2: Drive strength
  bool set_vbus_ovp_threshold_(VbusOvpThreshold threshold); // Bit 0: 6.3V or 18.5V
  bool get_thermal_threshold_(ThermalThreshold &threshold);
  bool get_switching_frequency_(SwitchingFrequency &freq);
  bool get_drive_strength_(DriveStrength &strength);
  bool get_vbus_ovp_threshold_(VbusOvpThreshold &threshold);

  // REG 0x18 CHARGER_CTRL_2 - Charger Control 2 Register
  bool set_charger_ctrl_2_reg_(uint8_t value);         // Write full register
  bool get_charger_ctrl_2_reg_(uint8_t &value);        // Read full register
  bool set_pfm_forward_disabled_(bool disable);        // Bit 4: Disable PFM in forward buck
  bool set_batfet_ctrl_wvbus_(bool allow_without_vbus); // Bit 3: BATFET control with/without VBUS
  bool set_batfet_delay_(BatfetDelay delay);           // Bit 2: BATFET delay
  bool set_batfet_control_(BatfetControl mode);        // Bits 1:0: BATFET control mode
  bool get_pfm_forward_disabled_(bool &disabled);
  bool get_batfet_ctrl_wvbus_(bool &allow_without_vbus);
  bool get_batfet_delay_(BatfetDelay &delay);
  bool get_batfet_control_(BatfetControl &mode);

  // REG 0x19 CHARGER_CTRL_3 - Charger Control 3 Register
  bool set_charger_ctrl_3_reg_(uint8_t value);         // Write full register
  bool get_charger_ctrl_3_reg_(uint8_t &value);        // Read full register
  bool set_ibat_peak_threshold_(IbatPkThreshold threshold);  // Bits 7:6: Battery peak current threshold
  bool set_vbat_uvlo_threshold_(VbatUvloThreshold threshold); // Bit 5: VBAT UVLO threshold
  bool set_external_ilim_enabled_(bool enable);        // Bit 2: External ILIM pin enable
  bool set_charge_rate_(ChargeRate rate);              // Bits 1:0: Charge rate
  bool get_ibat_peak_threshold_(IbatPkThreshold &threshold);
  bool get_vbat_uvlo_threshold_(VbatUvloThreshold &threshold);
  bool get_external_ilim_enabled_(bool &enabled);
  bool get_charge_rate_(ChargeRate &rate);

  // REG 0x1A NTC_CTRL_0 - NTC Control 0 Register
  bool set_ntc_ctrl_0_reg_(uint8_t value);             // Write full register
  bool get_ntc_ctrl_0_reg_(uint8_t &value);            // Read full register
  bool set_ts_ignore_(bool ignore);                    // Bit 7: Ignore TS function
  bool set_ts_warm_current_(TsCurrentSetting setting); // Bits 3:2: TS warm current setting
  bool set_ts_cool_current_(TsCurrentSetting setting); // Bits 1:0: TS cool current setting
  bool get_ts_ignore_(bool &ignore);
  bool get_ts_warm_current_(TsCurrentSetting &setting);
  bool get_ts_cool_current_(TsCurrentSetting &setting);

  // REG 0x1B NTC_CTRL_1 - NTC Control 1 Register
  bool set_ntc_ctrl_1_reg_(uint8_t value);             // Write full register
  bool get_ntc_ctrl_1_reg_(uint8_t &value);            // Read full register
  bool set_ts_cold_thresholds_(TsColdThresholds thresholds);  // Bits 7:5: TH1/TH2/TH3
  bool set_ts_hot_thresholds_(TsHotThresholds thresholds);    // Bits 4:2: TH4/TH5/TH6
  bool set_ts_warm_voltage_(TsWarmVoltageSetting setting);    // Bits 1:0: TS warm voltage
  bool get_ts_cold_thresholds_(TsColdThresholds &thresholds);
  bool get_ts_hot_thresholds_(TsHotThresholds &thresholds);
  bool get_ts_warm_voltage_(TsWarmVoltageSetting &setting);

  // REG 0x1C NTC_CTRL_2 - NTC Control 2 Register
  bool set_ntc_ctrl_2_reg_(uint8_t value);             // Write full register
  bool get_ntc_ctrl_2_reg_(uint8_t &value);            // Read full register
  bool set_ts_voltage_symmetric_(bool symmetric);      // Bit 6: Symmetric voltage setting
  bool set_ts_prewarm_voltage_(TsWarmVoltageSetting setting);  // Bits 5:4: TS prewarm voltage
  bool set_ts_prewarm_current_(TsCurrentSetting setting);      // Bits 3:2: TS prewarm current
  bool set_ts_precool_current_(TsCurrentSetting setting);      // Bits 1:0: TS precool current
  bool get_ts_voltage_symmetric_(bool &symmetric);
  bool get_ts_prewarm_voltage_(TsWarmVoltageSetting &setting);
  bool get_ts_prewarm_current_(TsCurrentSetting &setting);
  bool get_ts_precool_current_(TsCurrentSetting &setting);

  // REG 0x1D CHARGER_STATUS_0 - Charger Status 0 Register (Read-only)
  bool get_charger_status_0_reg_(uint8_t &value);      // Read full register
  bool get_adc_conversion_done_(bool &done);           // Bit 6: ADC conversion complete
  bool get_thermal_regulation_status_(bool &active);   // Bit 5: Thermal regulation active
  bool get_vsys_regulation_status_(bool &active);      // Bit 4: VSYS regulation active
  bool get_iindpm_regulation_status_(bool &active);    // Bit 3: IINDPM/ILIM regulation active
  bool get_vindpm_regulation_status_(bool &active);    // Bit 2: VINDPM regulation active
  bool get_safety_timer_expired_(bool &expired);       // Bit 1: Safety timer expired
  bool get_watchdog_timer_expired_(bool &expired);     // Bit 0: Watchdog timer expired

  // REG 0x1E CHARGER_STATUS_1 - Charger Status 1 Register (Read-only)
  bool get_charger_status_1_reg_(uint8_t &value);      // Read full register
  bool get_charge_status_(ChargeStatus &status);       // Bits 4:3: Charge status
  bool get_vbus_status_(VbusStatus &status);           // Bits 2:0: VBUS status

  // REG 0x1F FAULT_STATUS_0 - Fault Status 0 Register (Read-only)
  bool get_fault_status_0_reg_(uint8_t &value);        // Read full register
  bool get_vbus_fault_status_(bool &fault);            // Bit 7: VBUS fault
  bool get_battery_fault_status_(bool &fault);         // Bit 6: Battery fault
  bool get_system_fault_status_(bool &fault);          // Bit 5: System fault
  bool get_thermal_shutdown_status_(bool &shutdown);   // Bit 3: Thermal shutdown
  bool get_ts_temperature_zone_(TsTemperatureZone &zone); // Bits 2:0: TS temperature zone

  // REG 0x20 CHARGER_FLAG_0 - Charger Flag 0 Register (Read-only)
  bool get_charger_flag_0_reg_(uint8_t &value);        // Read full register
  bool get_adc_conversion_done_flag_(bool &flag);      // Bit 6: ADC conversion done flag
  bool get_thermal_regulation_flag_(bool &flag);       // Bit 5: Thermal regulation flag
  bool get_vsys_regulation_flag_(bool &flag);          // Bit 4: VSYS regulation flag
  bool get_iindpm_regulation_flag_(bool &flag);        // Bit 3: IINDPM/ILIM regulation flag
  bool get_vindpm_regulation_flag_(bool &flag);        // Bit 2: VINDPM regulation flag
  bool get_safety_timer_expired_flag_(bool &flag);     // Bit 1: Safety timer flag
  bool get_watchdog_timer_expired_flag_(bool &flag);   // Bit 0: Watchdog timer flag

  // REG 0x21 CHARGER_FLAG_1 - Charger Flag 1 Register (Read-only)
  bool get_charger_flag_1_reg_(uint8_t &value);        // Read full register
  bool get_charge_status_changed_flag_(bool &flag);    // Bit 3: Charge status changed
  bool get_vbus_status_changed_flag_(bool &flag);      // Bit 0: VBUS status changed

  // REG 0x22 FAULT_FLAG_0 - Fault Flag 0 Register (Read-only)
  bool get_fault_flag_0_reg_(uint8_t &value);          // Read full register
  bool get_vbus_fault_flag_(bool &flag);               // Bit 7: VBUS fault flag
  bool get_battery_fault_flag_(bool &flag);            // Bit 6: Battery fault flag
  bool get_system_fault_flag_(bool &flag);             // Bit 5: System fault flag
  bool get_thermal_shutdown_flag_(bool &flag);         // Bit 3: Thermal shutdown flag
  bool get_ts_status_changed_flag_(bool &flag);        // Bit 0: TS status change flag

  // REG 0x23 CHARGER_MASK_0 - Charger Mask 0 Register
  bool set_charger_mask_0_reg_(uint8_t value);         // Write full register
  bool get_charger_mask_0_reg_(uint8_t &value);        // Read full register
  bool set_adc_conversion_done_mask_(bool mask);       // Bit 6: ADC conversion mask
  bool set_thermal_regulation_mask_(bool mask);        // Bit 5: Thermal regulation mask
  bool set_vsys_regulation_mask_(bool mask);           // Bit 4: VSYS regulation mask
  bool set_iindpm_regulation_mask_(bool mask);         // Bit 3: IINDPM/ILIM mask
  bool set_vindpm_regulation_mask_(bool mask);         // Bit 2: VINDPM mask
  bool set_safety_timer_expired_mask_(bool mask);      // Bit 1: Safety timer mask
  bool set_watchdog_timer_expired_mask_(bool mask);    // Bit 0: Watchdog timer mask
  bool get_adc_conversion_done_mask_(bool &mask);
  bool get_thermal_regulation_mask_(bool &mask);
  bool get_vsys_regulation_mask_(bool &mask);
  bool get_iindpm_regulation_mask_(bool &mask);
  bool get_vindpm_regulation_mask_(bool &mask);
  bool get_safety_timer_expired_mask_(bool &mask);
  bool get_watchdog_timer_expired_mask_(bool &mask);

  // REG 0x24 CHARGER_MASK_1 - Charger Mask 1 Register
  bool set_charger_mask_1_reg_(uint8_t value);         // Write full register
  bool get_charger_mask_1_reg_(uint8_t &value);        // Read full register
  bool set_charge_status_changed_mask_(bool mask);     // Bit 3: Charge status change mask
  bool set_vbus_status_changed_mask_(bool mask);       // Bit 0: VBUS status change mask
  bool get_charge_status_changed_mask_(bool &mask);
  bool get_vbus_status_changed_mask_(bool &mask);

  // REG 0x25 FAULT_MASK_0 - Fault Mask 0 Register
  bool set_fault_mask_0_reg_(uint8_t value);           // Write full register
  bool get_fault_mask_0_reg_(uint8_t &value);          // Read full register
  bool set_vbus_fault_mask_(bool mask);                // Bit 7: VBUS fault mask
  bool set_battery_fault_mask_(bool mask);             // Bit 6: Battery fault mask
  bool set_system_fault_mask_(bool mask);              // Bit 5: System fault mask
  bool set_thermal_shutdown_mask_(bool mask);          // Bit 3: Thermal shutdown mask
  bool set_ts_temperature_zone_mask_(bool mask);       // Bit 0: TS temperature zone mask
  bool get_vbus_fault_mask_(bool &mask);
  bool get_battery_fault_mask_(bool &mask);
  bool get_system_fault_mask_(bool &mask);
  bool get_thermal_shutdown_mask_(bool &mask);
  bool get_ts_temperature_zone_mask_(bool &mask);

  // REG 0x26 ADC_CONTROL - ADC Control Register
  bool set_adc_control_reg_(uint8_t value);            // Write full register
  bool get_adc_control_reg_(uint8_t &value);           // Read full register
  bool set_adc_enabled_(bool enable);                  // Bit 7: ADC enable
  bool set_adc_rate_oneshot_(bool oneshot);            // Bit 6: ADC rate (0=continuous, 1=one-shot)
  bool set_adc_sample_speed_(AdcSampleSpeed speed);    // Bits 5:4: ADC sample speed
  bool set_adc_average_enabled_(bool enable);          // Bit 3: ADC average control
  bool set_adc_average_init_new_(bool use_new);        // Bit 2: ADC average init
  bool get_adc_enabled_(bool &enabled);
  bool get_adc_rate_oneshot_(bool &oneshot);
  bool get_adc_sample_speed_(AdcSampleSpeed &speed);
  bool get_adc_average_enabled_(bool &enabled);
  bool get_adc_average_init_new_(bool &use_new);

  // REG 0x27 ADC_FUNCTION_DISABLE_0 - ADC Function Disable Register
  bool set_adc_function_disable_0_reg_(uint8_t value);  // Write full register
  bool get_adc_function_disable_0_reg_(uint8_t &value); // Read full register
  bool set_ibus_adc_disabled_(bool disable);            // Bit 7: IBUS ADC disable
  bool set_ibat_adc_disabled_(bool disable);            // Bit 6: IBAT ADC disable
  bool set_vbus_adc_disabled_(bool disable);            // Bit 5: VBUS ADC disable
  bool set_vbat_adc_disabled_(bool disable);            // Bit 4: VBAT ADC disable
  bool set_vsys_adc_disabled_(bool disable);            // Bit 3: VSYS ADC disable
  bool set_ts_adc_disabled_(bool disable);              // Bit 2: TS ADC disable
  bool set_tdie_adc_disabled_(bool disable);            // Bit 1: TDIE ADC disable
  bool set_vpmid_adc_disabled_(bool disable);           // Bit 0: VPMID ADC disable
  bool get_ibus_adc_disabled_(bool &disabled);
  bool get_ibat_adc_disabled_(bool &disabled);
  bool get_vbus_adc_disabled_(bool &disabled);
  bool get_vbat_adc_disabled_(bool &disabled);
  bool get_vsys_adc_disabled_(bool &disabled);
  bool get_ts_adc_disabled_(bool &disabled);
  bool get_tdie_adc_disabled_(bool &disabled);
  bool get_vpmid_adc_disabled_(bool &disabled);

  // REG 0x28-0x29 IBUS_ADC - IBUS ADC Reading (Read-only, 16-bit)
  bool get_ibus_adc_(int16_t &current_ma);  // Read IBUS current in mA (2's complement, 2mA/LSB)

  // REG 0x2A-0x2B IBAT_ADC - IBAT ADC Reading (Read-only, 16-bit, bits 15:2)
  bool get_ibat_adc_(int16_t &current_ma);  // Read IBAT current in mA (2's complement, 4mA/LSB, bits 15:2)

  // REG 0x2C-0x2D VBUS_ADC - VBUS ADC Reading (Read-only, 16-bit, bits 14:2)
  bool get_vbus_adc_(uint16_t &voltage_mv);  // Read VBUS voltage in mV (unsigned, 3.97mV/LSB, bits 14:2)

  // REG 0x2E-0x2F VPMID_ADC - VPMID ADC Reading (Read-only, 16-bit, bits 14:2)
  bool get_vpmid_adc_(uint16_t &voltage_mv);  // Read VPMID voltage in mV (unsigned, 3.97mV/LSB, bits 14:2)

  // REG 0x30-0x31 VBAT_ADC - VBAT ADC Reading (Read-only, 16-bit, bits 12:1)
  bool get_vbat_adc_(uint16_t &voltage_mv);  // Read VBAT voltage in mV (unsigned, 1.99mV/LSB, bits 12:1)

  // REG 0x32-0x33 VSYS_ADC - VSYS ADC Reading (Read-only, 16-bit, bits 12:1)
  bool get_vsys_adc_(uint16_t &voltage_mv);  // Read VSYS voltage in mV (unsigned, 1.99mV/LSB, bits 12:1)

  // REG 0x34-0x35 TS_ADC - TS ADC Reading (Read-only, 16-bit, bits 11:0)
  bool get_ts_adc_(float &percentage);  // Read TS percentage (unsigned, 0.0961%/LSB, bits 11:0)

  // REG 0x36-0x37 TDIE_ADC - TDIE ADC Reading (Read-only, 16-bit, bits 11:0)
  bool get_tdie_adc_(float &temperature_c);  // Read die temperature in °C (2's complement, 0.5°C/LSB, bits 11:0)

  // REG 0x38 PART_INFORMATION - Part Information Register (Read-only)
  bool get_part_information_reg_(uint8_t &value);  // Read full register
  bool get_part_number_(uint8_t &part_number);     // Bits 5:3: Part number
  bool get_device_revision_(uint8_t &revision);    // Bits 2:0: Device revision

  // Protected register-specific getters
  bool get_charge_current_reg_(float &current);      // REG 0x02/0x03 ICHG
  bool get_charge_voltage_reg_(float &voltage);      // REG 0x04/0x05 VBAT
  bool get_input_current_limit_reg_(float &current); // REG 0x06/0x07 IINDPM
  bool get_input_voltage_limit_reg_(float &voltage); // REG 0x08/0x09 VINDPM
  bool get_minimum_system_voltage_reg_(float &voltage); // REG 0x0E/0x0F VSYSMIN
  bool get_precharge_current_reg_(float &current);   // REG 0x10/0x11 IPRECHG
  bool get_termination_current_reg_(float &current); // REG 0x12/0x13 ITERM
};

}  // namespace bq25628e
}  // namespace esphome
