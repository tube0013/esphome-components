"""Persistent Ethernet configuration with a confirmed/trial boot transaction."""

from __future__ import annotations

from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import automation
from esphome.components import ethernet, http_request, web_server_base
from esphome.components.http_request.update import HttpRequestUpdate
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
from esphome.const import CONF_ID
from esphome.types import ConfigType

CONF_ETHERNET = "ethernet"
CONF_ETHERNET_ID = "ethernet_id"
CONF_FIRMWARE_UPDATE_ID = "firmware_update_id"
CONF_HTTP_REQUEST_ID = "http_request_id"
CONF_ON_INTERNET_ENABLED = "on_internet_enabled"
CONF_WIFI = "wifi"


DOMAIN = "network_settings"

DEPENDENCIES = [CONF_ETHERNET, "web_server", "esp32"]
AUTO_LOAD = ["json"]

ns = cg.esphome_ns.namespace("network_settings")
NetworkSettings = ns.class_("NetworkSettings", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NetworkSettings),
            cv.GenerateID(CONF_HTTP_REQUEST_ID): cv.use_id(http_request.HttpRequestComponent),
            cv.Optional(CONF_ON_INTERNET_ENABLED): automation.validate_automation(single=True),
            cv.Required(CONF_FIRMWARE_UPDATE_ID): cv.use_id(HttpRequestUpdate),
            cv.Required(CONF_ETHERNET_ID): cv.use_id(ethernet.EthernetComponent),
            cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(web_server_base.WebServerBase),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_framework("esp-idf"),
)


def final_validate(config: ConfigType) -> None:
    full = fv.full_config.get()
    if "manual_ip" in full[CONF_ETHERNET]:
        raise cv.Invalid("network_settings owns Ethernet addressing; remove ethernet.manual_ip")
    if full[CONF_ETHERNET].get("enable_on_boot", True) is False:
        raise cv.Invalid("network_settings requires Ethernet enabled on boot")
    if CONF_WIFI in full:
        raise cv.Invalid("network_settings currently supports Ethernet-only devices")


FINAL_VALIDATE_SCHEMA = final_validate


async def to_code(config: ConfigType) -> None:
    dependencies = [
        await cg.get_variable(config[key])
        for key in [
            CONF_HTTP_REQUEST_ID,
            CONF_FIRMWARE_UPDATE_ID,
            CONF_ETHERNET_ID,
            CONF_WEB_SERVER_BASE_ID,
        ]
    ]
    var = cg.new_Pvariable(config[CONF_ID], *dependencies)
    await cg.register_component(var, config)
    if CONF_ON_INTERNET_ENABLED in config:
        await automation.build_callback_automation(
            var,
            "add_on_internet_enabled_callback",
            [],
            config[CONF_ON_INTERNET_ENABLED],
        )
    # Keep ESPHome's own static-IP path compiled in, even though DHCP is the
    # factory default. The saved static settings are assigned before Ethernet setup.
    cg.add_define("USE_ETHERNET_MANUAL_IP")
    page = (Path(__file__).parent / "network_settings.html").read_text()
    validation = (Path(__file__).parent / "network_validation.js").read_text()
    page = page.replace("/* NETWORK_VALIDATION */", validation)
    cg.add_global(
        cg.RawStatement(
            "namespace esphome::network_settings {\n"
            'extern const char NETWORK_PAGE[] = R"TZBNET(' + page + ')TZBNET";\n}'
        )
    )
