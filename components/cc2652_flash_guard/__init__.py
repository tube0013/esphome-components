"""Types for the shared CC2652 pause/flash/resume automation."""
import esphome.config_validation as cv

AUTO_LOAD = ["zigbee_firmware_baud"]

CONFIG_SCHEMA = cv.Schema({})


async def to_code(config):
    pass
