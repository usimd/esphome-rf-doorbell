import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome import pins, automation
from esphome.const import (
    CONF_ID,
    CONF_FREQUENCY,
    CONF_CS_PIN,
)

DEPENDENCIES = ["spi"]
CODEOWNERS = ["@usimd"]

CONF_RESET_PIN = "reset_pin"
CONF_INTERRUPT_PIN = "interrupt_pin"
CONF_NODE_ID = "node_id"
CONF_NETWORK_ID = "network_id"
CONF_ENCRYPTION_KEY = "encryption_key"
CONF_IS_HIGH_POWER = "is_high_power"
CONF_TX_POWER = "tx_power"
CONF_CHALLENGE_TIMEOUT = "challenge_timeout"
CONF_ON_PACKET_RECEIVED = "on_packet_received"
CONF_ON_DEVICE_PAIRED = "on_device_paired"
CONF_ON_DOOR_OPEN_REQUEST = "on_door_open_request"

rfm69_ns = cg.esphome_ns.namespace("rfm69")
RFM69Component = rfm69_ns.class_(
    "RFM69Component", cg.Component, spi.SPIDevice
)

PacketReceivedTrigger = rfm69_ns.class_(
    "PacketReceivedTrigger", automation.Trigger.template(cg.uint8, cg.uint8, cg.std_vector.template(cg.uint8))
)

DevicePairedTrigger = rfm69_ns.class_(
    "DevicePairedTrigger", automation.Trigger.template(cg.uint32)
)

DoorOpenRequestTrigger = rfm69_ns.class_(
    "DoorOpenRequestTrigger", automation.Trigger.template(cg.uint32, cg.bool_)
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RFM69Component),
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,
            cv.Required(CONF_FREQUENCY): cv.float_range(min=290.0, max=1020.0),
            cv.Required(CONF_NODE_ID): cv.int_range(min=1, max=255),
            cv.Optional(CONF_NETWORK_ID, default=100): cv.int_range(min=0, max=255),
            cv.Optional(CONF_ENCRYPTION_KEY): cv.string_strict,
            cv.Optional(CONF_IS_HIGH_POWER, default=False): cv.boolean,
            cv.Optional(CONF_TX_POWER, default=13): cv.int_range(min=-18, max=20),
            cv.Optional(CONF_CHALLENGE_TIMEOUT, default="30s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_PACKET_RECEIVED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_ID): cv.declare_id(PacketReceivedTrigger),
                }
            ),
            cv.Optional(CONF_ON_DEVICE_PAIRED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_ID): cv.declare_id(DevicePairedTrigger),
                }
            ),
            cv.Optional(CONF_ON_DOOR_OPEN_REQUEST): automation.validate_automation(
                {
                    cv.GenerateID(CONF_ID): cv.declare_id(DoorOpenRequestTrigger),
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    # CS pin is handled by spi.register_spi_device

    reset = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(reset))

    interrupt = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
    cg.add(var.set_interrupt_pin(interrupt))

    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_node_id(config[CONF_NODE_ID]))
    cg.add(var.set_network_id(config[CONF_NETWORK_ID]))
    cg.add(var.set_high_power(config[CONF_IS_HIGH_POWER]))
    cg.add(var.set_tx_power(config[CONF_TX_POWER]))
    cg.add(var.set_challenge_timeout(config[CONF_CHALLENGE_TIMEOUT]))

    if CONF_ENCRYPTION_KEY in config:
        cg.add(var.set_encryption_key(config[CONF_ENCRYPTION_KEY]))

    for conf in config.get(CONF_ON_PACKET_RECEIVED, []):
        trigger = cg.new_Pvariable(conf[CONF_ID], var)
        await automation.build_automation(
            trigger, 
            [(cg.uint8, "from_address"), (cg.uint8, "rssi"), (cg.std_vector.template(cg.uint8), "data")], 
            conf
        )

    for conf in config.get(CONF_ON_DEVICE_PAIRED, []):
        trigger = cg.new_Pvariable(conf[CONF_ID], var)
        await automation.build_automation(
            trigger, 
            [(cg.uint32, "serial_number")], 
            conf
        )

    for conf in config.get(CONF_ON_DOOR_OPEN_REQUEST, []):
        trigger = cg.new_Pvariable(conf[CONF_ID], var)
        await automation.build_automation(
            trigger, 
            [(cg.uint32, "serial_number"), (cg.bool_, "authorized")], 
            conf
        )
