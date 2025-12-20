import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from . import MAX17260Component, CONF_MAX17260_ID

DEPENDENCIES = ["max17260"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MAX17260_ID): cv.use_id(MAX17260Component),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MAX17260_ID])
    return parent
