import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from . import BQ25628EComponent, CONF_BQ25628E_ID

DEPENDENCIES = ["bq25628e"]

CONF_BUS_VOLTAGE = "bus_voltage"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_CHARGE_CURRENT = "charge_current"
CONF_SYSTEM_VOLTAGE = "system_voltage"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ25628E_ID): cv.use_id(BQ25628EComponent),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BQ25628E_ID])
    return parent
