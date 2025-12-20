import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_VOLTAGE,
    CONF_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
)

DEPENDENCIES = ["i2c"]
CODEOWNERS = ["@usimd"]

# Add Adafruit BQ25628E library dependency
AUTO = True

CONF_BQ25628E_ID = "bq25628e_id"
CONF_BUS_VOLTAGE = "bus_voltage"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_CHARGE_CURRENT = "charge_current"
CONF_SYSTEM_VOLTAGE = "system_voltage"
CONF_CHARGE_CURRENT_LIMIT = "charge_current_limit"
CONF_CHARGE_VOLTAGE_LIMIT = "charge_voltage_limit"
CONF_INPUT_CURRENT_LIMIT = "input_current_limit"

bq25628e_ns = cg.esphome_ns.namespace("bq25628e")
BQ25628EComponent = bq25628e_ns.class_(
    "BQ25628EComponent", cg.PollingComponent, i2c.I2CDevice
)

BQ25628EVoltage = bq25628e_ns.class_(
    "BQ25628EVoltage", sensor.Sensor, cg.Component
)
BQ25628ECurrent = bq25628e_ns.class_(
    "BQ25628ECurrent", sensor.Sensor, cg.Component
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BQ25628EComponent),
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
            cv.Optional(CONF_CHARGE_CURRENT_LIMIT, default=1.0): cv.float_range(
                min=0.05, max=3.0
            ),
            cv.Optional(CONF_CHARGE_VOLTAGE_LIMIT, default=4.2): cv.float_range(
                min=3.84, max=4.624
            ),
            cv.Optional(CONF_INPUT_CURRENT_LIMIT, default=0.5): cv.float_range(
                min=0.1, max=3.2
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x6B))
)


async def to_code(config):
    # Add Adafruit BQ25628E library from GitHub (not yet in PlatformIO registry)
    cg.add_library("adafruit/Adafruit BusIO", "1.16.1")
    cg.add_library("https://github.com/adafruit/Adafruit_BQ25628E.git", "1.0.0")
    
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

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

    cg.add(var.set_charge_current_limit(config[CONF_CHARGE_CURRENT_LIMIT]))
    cg.add(var.set_charge_voltage_limit(config[CONF_CHARGE_VOLTAGE_LIMIT]))
    cg.add(var.set_input_current_limit(config[CONF_INPUT_CURRENT_LIMIT]))
