"""Shared manifest/application UART baud policy for Zigbee flashers."""

from __future__ import annotations

import esphome.config_validation as cv

DOMAIN = "zigbee_firmware_baud"
AUTO_LOAD = ["json", "select"]
CONFIG_SCHEMA = cv.Schema({})
