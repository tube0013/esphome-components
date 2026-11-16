"""Types for the shared CC2652 pause/flash/resume automation."""

from __future__ import annotations

import esphome.config_validation as cv
from esphome.types import ConfigType

DOMAIN = "cc2652_flash_guard"

AUTO_LOAD = ["zigbee_firmware_baud"]

CONFIG_SCHEMA = cv.Schema({})


async def to_code(config: ConfigType) -> None:
    """This helper exposes native types and needs no runtime instance."""
