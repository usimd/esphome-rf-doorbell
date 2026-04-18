import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    deep_sleep,
    esp32,
    i2c,
    number,
    sensor,
    switch,
)
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_SECOND,
    UNIT_MILLIAMP,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome import pins

DEPENDENCIES = ["i2c"]
CODEOWNERS = ["@usimd"]
AUTO_LOAD = ["sensor", "switch", "number", "binary_sensor"]

# Configuration keys - Hardware pins
CONF_CHARGE_DISABLE_PIN = "charge_disable_pin"
CONF_SUPPLY_SENSE_ENABLE_PIN = "supply_sense_enable_pin"
CONF_SUPPLY_ADC_PIN = "supply_adc_pin"
CONF_VPROG_ADC_PIN = "vprog_adc_pin"

# Voltage divider configuration for VINDPM calculation
CONF_R_HIGH = "r_high"  # R5 in schematic (820k default)
CONF_R_LOW = "r_low"  # R6 in schematic (120k default)
CONF_DIGIPOT_RESISTANCE = "digipot_resistance"  # 100k for ISL23315T

# Supply voltage divider (R10/R11)
CONF_SUPPLY_DIVIDER_RATIO = "supply_divider_ratio"

# LTC4079 R_PROG resistor for charge current calculation
CONF_R_PROG = "r_prog"  # R11 in schematic (30k default)

# Series resistor on V_PROG ADC input for isolation
CONF_R_SERIES_VPROG = "r_series_vprog"  # 47k default

# Deep sleep component reference for runtime sleep duration updates
CONF_DEEP_SLEEP_ID = "deep_sleep_id"

# Sub-component keys
CONF_VINDPM_THRESHOLD = "vindpm_threshold"
CONF_CHARGING_ENABLED = "charging_enabled"
CONF_CHARGE_DISABLE = "charge_disable"
CONF_WIPER_VALUE = "wiper_value"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_CHARGE_CURRENT = "charge_current"
CONF_CHARGING = "charging"
CONF_SLEEP_DURATION = "sleep_duration"

doorbell_controller_ns = cg.esphome_ns.namespace("doorbell_controller")
DoorbellControllerComponent = doorbell_controller_ns.class_(
    "DoorbellControllerComponent", cg.PollingComponent, i2c.I2CDevice
)
VINDPMThresholdNumber = doorbell_controller_ns.class_(
    "VINDPMThresholdNumber", number.Number, cg.Component
)
SleepDurationNumber = doorbell_controller_ns.class_(
    "SleepDurationNumber", number.Number, cg.Component
)
ChargingEnabledSwitch = doorbell_controller_ns.class_(
    "ChargingEnabledSwitch", switch.Switch, cg.Component
)
ChargeDisableSwitch = doorbell_controller_ns.class_(
    "ChargeDisableSwitch", switch.Switch, cg.Component
)

# Schema for VINDPM threshold number
VINDPM_THRESHOLD_SCHEMA = number.number_schema(
    VINDPMThresholdNumber,
    icon="mdi:flash-triangle-outline",
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(cv.COMPONENT_SCHEMA)

# Schema for sleep duration number
SLEEP_DURATION_SCHEMA = number.number_schema(
    SleepDurationNumber,
    icon="mdi:sleep",
    unit_of_measurement=UNIT_SECOND,
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(cv.COMPONENT_SCHEMA)

# Schema for charging enabled switch
CHARGING_ENABLED_SCHEMA = switch.switch_schema(
    ChargingEnabledSwitch,
    icon="mdi:battery-charging",
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(cv.COMPONENT_SCHEMA)

# Schema for charge disable override switch
CHARGE_DISABLE_SCHEMA = switch.switch_schema(
    ChargeDisableSwitch,
    icon="mdi:battery-off",
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(cv.COMPONENT_SCHEMA)

# Schema for wiper value sensor
WIPER_VALUE_SCHEMA = sensor.sensor_schema(
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    icon="mdi:resistor",
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

# Schema for supply voltage sensor (diagnostic - intercom transformer power)
SUPPLY_VOLTAGE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_VOLTAGE,
    state_class=STATE_CLASS_MEASUREMENT,
    icon="mdi:flash",
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

# Schema for charge current sensor (calculated from 250 * V_PROG / R_PROG)
CHARGE_CURRENT_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_MILLIAMP,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_CURRENT,
    state_class=STATE_CLASS_MEASUREMENT,
    icon="mdi:battery-charging-outline",
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

# Schema for charging binary sensor
CHARGING_SCHEMA = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_BATTERY,
    icon="mdi:battery-charging",
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DoorbellControllerComponent),
            # Hardware pin configuration
            cv.Optional(CONF_CHARGE_DISABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SUPPLY_SENSE_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SUPPLY_ADC_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_VPROG_ADC_PIN): pins.internal_gpio_input_pin_number,
            # VINDPM voltage divider resistor values
            cv.Optional(CONF_R_HIGH, default=820000): cv.resistance,
            cv.Optional(CONF_R_LOW, default=120000): cv.resistance,
            cv.Optional(CONF_DIGIPOT_RESISTANCE, default=100000): cv.resistance,
            # Supply voltage divider ratio (R10+R11)/R11 = 11.0 for 1M/100k
            cv.Optional(CONF_SUPPLY_DIVIDER_RATIO, default=11.0): cv.float_range(
                min=1.0, max=100.0
            ),
            # LTC4079 R_PROG for charge current calculation: I_CHG = 250 * V_PROG / R_PROG
            cv.Optional(CONF_R_PROG, default=30000): cv.resistance,
            # Series resistor on V_PROG ADC for isolation (compensated via ADC input impedance)
            cv.Optional(CONF_R_SERIES_VPROG, default=47000): cv.resistance,
            cv.Optional(CONF_DEEP_SLEEP_ID): cv.use_id(deep_sleep.DeepSleepComponent),
            # Sub-components - Numbers
            cv.Optional(CONF_VINDPM_THRESHOLD): VINDPM_THRESHOLD_SCHEMA,
            cv.Optional(CONF_SLEEP_DURATION): SLEEP_DURATION_SCHEMA,
            # Sub-components - Switches
            cv.Optional(CONF_CHARGING_ENABLED): CHARGING_ENABLED_SCHEMA,
            cv.Optional(CONF_CHARGE_DISABLE): CHARGE_DISABLE_SCHEMA,
            # Sub-components - Sensors
            cv.Optional(CONF_WIPER_VALUE): WIPER_VALUE_SCHEMA,
            cv.Optional(CONF_SUPPLY_VOLTAGE): SUPPLY_VOLTAGE_SCHEMA,
            cv.Optional(CONF_CHARGE_CURRENT): CHARGE_CURRENT_SCHEMA,
            # Sub-components - Binary Sensors
            cv.Optional(CONF_CHARGING): CHARGING_SCHEMA,
        }
    )
    .extend(cv.polling_component_schema("never"))  # Manual/wakeup triggered
    .extend(i2c.i2c_device_schema(0x28))  # Default I2C address for ISL23315
)


async def to_code(config):
    esp32.include_builtin_idf_component("esp_adc")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Set voltage divider configuration
    cg.add(var.set_r_high(config[CONF_R_HIGH]))
    cg.add(var.set_r_low(config[CONF_R_LOW]))
    cg.add(var.set_digipot_resistance(config[CONF_DIGIPOT_RESISTANCE]))
    cg.add(var.set_supply_divider_ratio(config[CONF_SUPPLY_DIVIDER_RATIO]))
    cg.add(var.set_r_prog(config[CONF_R_PROG]))
    cg.add(var.set_r_series_vprog(config[CONF_R_SERIES_VPROG]))

    if CONF_DEEP_SLEEP_ID in config:
        deep_sleep_var = await cg.get_variable(config[CONF_DEEP_SLEEP_ID])
        cg.add(var.set_deep_sleep(deep_sleep_var))

    # Configure hardware pins
    if CONF_CHARGE_DISABLE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_CHARGE_DISABLE_PIN])
        cg.add(var.set_charge_disable_pin(pin))

    if CONF_SUPPLY_SENSE_ENABLE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_SUPPLY_SENSE_ENABLE_PIN])
        cg.add(var.set_supply_sense_enable_pin(pin))

    if CONF_SUPPLY_ADC_PIN in config:
        cg.add(var.set_supply_adc_pin(config[CONF_SUPPLY_ADC_PIN]))

    if CONF_VPROG_ADC_PIN in config:
        cg.add(var.set_vprog_adc_pin(config[CONF_VPROG_ADC_PIN]))

    # Configure VINDPM threshold number
    if CONF_VINDPM_THRESHOLD in config:
        conf = config[CONF_VINDPM_THRESHOLD]
        num = await number.new_number(
            conf,
            min_value=5.0,
            max_value=10.0,
            step=0.1,
        )
        await cg.register_component(num, conf)
        cg.add(num.set_parent(var))
        cg.add(var.set_vindpm_threshold_number(num))

    # Configure sleep duration number
    if CONF_SLEEP_DURATION in config:
        conf = config[CONF_SLEEP_DURATION]
        num = await number.new_number(
            conf,
            min_value=30.0,
            max_value=300.0,
            step=30.0,
        )
        await cg.register_component(num, conf)
        cg.add(num.set_parent(var))
        cg.add(var.set_sleep_duration_number(num))

    # Configure charging enabled switch
    if CONF_CHARGING_ENABLED in config:
        conf = config[CONF_CHARGING_ENABLED]
        sw = await switch.new_switch(conf)
        await cg.register_component(sw, conf)
        cg.add(sw.set_parent(var))
        cg.add(var.set_charging_enabled_switch(sw))

    # Configure charge disable switch
    if CONF_CHARGE_DISABLE in config:
        conf = config[CONF_CHARGE_DISABLE]
        sw = await switch.new_switch(conf)
        await cg.register_component(sw, conf)
        cg.add(sw.set_parent(var))
        cg.add(var.set_charge_disable_switch(sw))

    # Configure sensors
    if CONF_WIPER_VALUE in config:
        sens = await sensor.new_sensor(config[CONF_WIPER_VALUE])
        cg.add(var.set_wiper_value_sensor(sens))

    if CONF_SUPPLY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_SUPPLY_VOLTAGE])
        cg.add(var.set_supply_voltage_sensor(sens))

    if CONF_CHARGE_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_CHARGE_CURRENT])
        cg.add(var.set_charge_current_sensor(sens))

    # Configure binary sensors
    if CONF_CHARGING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CHARGING])
        cg.add(var.set_charging_binary_sensor(sens))
