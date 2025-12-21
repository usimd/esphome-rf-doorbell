import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["i2c"]
CODEOWNERS = ["@usimd"]

CONF_BQ25628E_ID = "bq25628e_id"
CONF_BUS_VOLTAGE = "bus_voltage"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_CHARGE_CURRENT = "charge_current"
CONF_SYSTEM_VOLTAGE = "system_voltage"
CONF_INPUT_CURRENT = "input_current"
CONF_TS_TEMPERATURE = "ts_temperature"
CONF_DIE_TEMPERATURE = "die_temperature"
CONF_CHARGE_CURRENT_LIMIT = "charge_current_limit"
CONF_CHARGE_VOLTAGE_LIMIT = "charge_voltage_limit"
CONF_INPUT_CURRENT_LIMIT = "input_current_limit"
CONF_INPUT_VOLTAGE_LIMIT = "input_voltage_limit"
CONF_MINIMUM_SYSTEM_VOLTAGE = "minimum_system_voltage"
CONF_PRECHARGE_CURRENT = "precharge_current"
CONF_TERMINATION_CURRENT = "termination_current"
CONF_TERMINATION_ENABLED = "termination_enabled"
CONF_VINDPM_BATTERY_TRACKING = "vindpm_battery_tracking"
CONF_RECHARGE_THRESHOLD = "recharge_threshold"
CONF_WATCHDOG_TIMEOUT = "watchdog_timeout"
CONF_THERMAL_REGULATION_THRESHOLD = "thermal_regulation_threshold"

bq25628e_ns = cg.esphome_ns.namespace("bq25628e")
BQ25628EComponent = bq25628e_ns.class_(
    "BQ25628EComponent", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BQ25628EComponent),
            # Sensor definitions
            cv.Optional(CONF_BUS_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SYSTEM_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_INPUT_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TS_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_DIE_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            # Configuration parameters with defaults from datasheet
            cv.Optional(CONF_CHARGE_CURRENT_LIMIT, default=1.0): cv.float_range(
                min=0.04, max=2.0
            ),
            cv.Optional(CONF_CHARGE_VOLTAGE_LIMIT, default=4.2): cv.float_range(
                min=3.5, max=4.8
            ),
            cv.Optional(CONF_INPUT_CURRENT_LIMIT, default=0.5): cv.float_range(
                min=0.1, max=3.2
            ),
            cv.Optional(CONF_INPUT_VOLTAGE_LIMIT, default=4.6): cv.float_range(
                min=3.8, max=16.8
            ),
            cv.Optional(CONF_MINIMUM_SYSTEM_VOLTAGE, default=3.52): cv.float_range(
                min=2.56, max=3.84
            ),
            cv.Optional(CONF_PRECHARGE_CURRENT, default=0.03): cv.float_range(
                min=0.01, max=0.31
            ),
            cv.Optional(CONF_TERMINATION_CURRENT, default=0.02): cv.float_range(
                min=0.005, max=0.31
            ),
            cv.Optional(CONF_TERMINATION_ENABLED, default=True): cv.boolean,
            cv.Optional(CONF_VINDPM_BATTERY_TRACKING, default=True): cv.boolean,
            cv.Optional(CONF_RECHARGE_THRESHOLD, default=0.1): cv.float_range(
                min=0.1, max=0.2
            ),
            cv.Optional(CONF_WATCHDOG_TIMEOUT, default=1): cv.int_range(
                min=0, max=3
            ),
            cv.Optional(CONF_THERMAL_REGULATION_THRESHOLD, default=120): cv.one_of(
                60, 120, int=True
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x6A))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Register sensors
    if CONF_BUS_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BUS_VOLTAGE])
        cg.add(var.set_bus_voltage_sensor(sens))

    if CONF_BATTERY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE])
        cg.add(var.set_battery_voltage_sensor(sens))

    if CONF_CHARGE_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_CHARGE_CURRENT])
        cg.add(var.set_charge_current_sensor(sens))

    if CONF_SYSTEM_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_SYSTEM_VOLTAGE])
        cg.add(var.set_system_voltage_sensor(sens))

    if CONF_INPUT_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_INPUT_CURRENT])
        cg.add(var.set_input_current_sensor(sens))

    if CONF_TS_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TS_TEMPERATURE])
        cg.add(var.set_ts_temperature_sensor(sens))

    if CONF_DIE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_DIE_TEMPERATURE])
        cg.add(var.set_die_temperature_sensor(sens))

    # Set configuration parameters
    cg.add(var.set_charge_current_limit(config[CONF_CHARGE_CURRENT_LIMIT]))
    cg.add(var.set_charge_voltage_limit(config[CONF_CHARGE_VOLTAGE_LIMIT]))
    cg.add(var.set_input_current_limit(config[CONF_INPUT_CURRENT_LIMIT]))
    cg.add(var.set_input_voltage_limit(config[CONF_INPUT_VOLTAGE_LIMIT]))
    cg.add(var.set_minimum_system_voltage(config[CONF_MINIMUM_SYSTEM_VOLTAGE]))
    cg.add(var.set_precharge_current(config[CONF_PRECHARGE_CURRENT]))
    cg.add(var.set_termination_current(config[CONF_TERMINATION_CURRENT]))
    cg.add(var.set_termination_enabled(config[CONF_TERMINATION_ENABLED]))
    cg.add(var.set_vindpm_battery_tracking(config[CONF_VINDPM_BATTERY_TRACKING]))
    cg.add(var.set_recharge_threshold(config[CONF_RECHARGE_THRESHOLD]))
    cg.add(var.set_watchdog_timeout(config[CONF_WATCHDOG_TIMEOUT]))
    cg.add(var.set_thermal_regulation_threshold(config[CONF_THERMAL_REGULATION_THRESHOLD]))
