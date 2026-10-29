"""Local TubesZB frontend for single and dual radio gateways."""

from __future__ import annotations

import gzip
from typing import Any

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import (
    binary_sensor,
    button,
    ethernet,
    network_settings,
    select,
    switch,
    text,
    text_sensor,
    uart,
    web_server,
    web_server_base,
)
from esphome.components.http_request.update import HttpRequestUpdate
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
from esphome.const import CONF_ID
from esphome.core import CORE
from esphome.schema_extractors import SCHEMA_EXTRACT, schema_extractor
from esphome.types import ConfigType

from .assets import embedded_page

CONF_CHECK_BUTTON = "check_button"
CONF_FLASH_BUTTON = "flash_button"
CONF_PROBE_BUTTON = "probe_button"
CONF_RESET_BUTTON = "reset_button"
CONF_PAUSE_BUTTON = "pause_button"
CONF_RESUME_BUTTON = "resume_button"
CONF_BOOTLOADER_BUTTON = "bootloader_button"
CONF_FIRMWARE_SENSOR = "firmware_sensor"
CONF_LATEST_SENSOR = "latest_sensor"
CONF_BOARD_SENSOR = "board_sensor"
CONF_MANUFACTURER_SENSOR = "manufacturer_sensor"
CONF_IEEE_SENSOR = "ieee_sensor"
CONF_STATUS_SENSOR = "status_sensor"
CONF_ESP32 = "esp32"
CONF_BOARD = "board"
CONF_BAUD_SELECT = "baud_select"
CONF_BOOT_PROBE_SCRIPT = "boot_probe_script"
CONF_BOOTLOADER_SCRIPT = "bootloader_script"
CONF_BUSY_SENSOR = "busy_sensor"
CONF_CC2652 = "cc2652"
CONF_CC2652_FLASHER = "cc2652_flasher"
CONF_CUSTOM_URL_TEXT = "custom_url_text"
CONF_EFR32_FLASHER = "efr32_flasher"
CONF_EFR32_INFO = "efr32_info"
CONF_ETHERNET_ID = "ethernet_id"
CONF_FIRMWARE_UPDATE_ID = "firmware_update_id"
CONF_FLASH_SCRIPT = "flash_script"
CONF_GATEWAY_BETA_SWITCH = "gateway_beta_switch"
CONF_MODEL = "model"
CONF_NETWORK_SETTINGS_ID = "network_settings_id"
CONF_RADIO_MODEL = "radio_model"
CONF_SERIAL_RADIO = "serial_radio"
CONF_STREAM_SERVER = "stream_server"
CONF_STREAM_SERVER_ID = "stream_server_id"
CONF_UART_ID = "uart_id"
CONF_WEB_SERVER_ID = "web_server_id"
CONF_ZIGBEE = "zigbee"
CONF_ZIGBEE_BETA_SWITCH = "zigbee_beta_switch"
CONF_ZWAVE_PROXY = "zwave_proxy"
CONF_ZWAVE_UART_ID = "zwave_uart_id"


DOMAIN = "tubeszb_web"

DEPENDENCIES = [CONF_ESP32, "web_server", "network_settings", "api"]


def AUTO_LOAD(config: ConfigType) -> list[str]:
    # The serial backend supports an optional selector; fixed-baud profiles
    # still compile that backend without creating a select entity.
    return ["json", "select"] if CONF_SERIAL_RADIO in config else ["json"]


ns = cg.esphome_ns.namespace("tubeszb_web")
TubesZBWeb = ns.class_("TubesZBWeb", cg.Component)


@schema_extractor("schema")
def zigbee_schema(value: Any) -> Any:
    from esphome.components import script, stream_server

    fields = {
        cv.Required(CONF_STREAM_SERVER_ID): cv.use_id(stream_server.StreamServerComponent),
        cv.Required(CONF_BOOTLOADER_SCRIPT): cv.use_id(script.Script),
    }
    for key in [
        CONF_CHECK_BUTTON,
        CONF_FLASH_BUTTON,
        CONF_PROBE_BUTTON,
        CONF_RESET_BUTTON,
        CONF_PAUSE_BUTTON,
        CONF_RESUME_BUTTON,
        CONF_BOOTLOADER_BUTTON,
    ]:
        fields[cv.Required(key)] = cv.use_id(button.Button)
    for key in [
        CONF_FIRMWARE_SENSOR,
        CONF_LATEST_SENSOR,
        CONF_BOARD_SENSOR,
        CONF_MANUFACTURER_SENSOR,
        CONF_IEEE_SENSOR,
        CONF_STATUS_SENSOR,
    ]:
        fields[cv.Required(key)] = cv.use_id(text_sensor.TextSensor)
    fields[cv.Required(CONF_BUSY_SENSOR)] = cv.use_id(binary_sensor.BinarySensor)
    fields[cv.Required(CONF_GATEWAY_BETA_SWITCH)] = cv.use_id(switch.Switch)
    fields[cv.Optional(CONF_ZIGBEE_BETA_SWITCH)] = cv.use_id(switch.Switch)
    fields[cv.Required(CONF_BAUD_SELECT)] = cv.use_id(select.Select)
    fields[cv.Required(CONF_CUSTOM_URL_TEXT)] = cv.use_id(text.Text)
    schema = cv.Schema(fields)
    if value is SCHEMA_EXTRACT:
        return schema
    return schema(value)


@schema_extractor("schema")
def cc2652_schema(value: Any) -> Any:
    from esphome.components import script, stream_server

    fields = {
        cv.Required(CONF_STREAM_SERVER_ID): cv.use_id(stream_server.StreamServerComponent),
        cv.Required(CONF_FLASH_SCRIPT): cv.use_id(script.Script),
        cv.Required(CONF_BOOT_PROBE_SCRIPT): cv.use_id(script.Script),
    }
    for key in [
        CONF_CHECK_BUTTON,
        CONF_FLASH_BUTTON,
        CONF_PROBE_BUTTON,
        CONF_RESET_BUTTON,
        CONF_PAUSE_BUTTON,
        CONF_RESUME_BUTTON,
        CONF_BOOTLOADER_BUTTON,
    ]:
        fields[cv.Required(key)] = cv.use_id(button.Button)
    for key in [
        CONF_FIRMWARE_SENSOR,
        CONF_LATEST_SENSOR,
        CONF_BOARD_SENSOR,
        CONF_IEEE_SENSOR,
    ]:
        fields[cv.Required(key)] = cv.use_id(text_sensor.TextSensor)
    for key in [CONF_GATEWAY_BETA_SWITCH, CONF_ZIGBEE_BETA_SWITCH]:
        fields[cv.Required(key)] = cv.use_id(switch.Switch)
    schema = cv.Schema(fields)
    if value is SCHEMA_EXTRACT:
        return schema
    return schema(value)


@schema_extractor("schema")
def serial_radio_schema(value: Any) -> Any:
    from esphome.components import stream_server

    fields = {
        cv.Required(CONF_STREAM_SERVER_ID): cv.use_id(stream_server.StreamServerComponent),
        cv.Optional(CONF_BAUD_SELECT): cv.use_id(select.Select),
        cv.Optional(CONF_RADIO_MODEL, default="MGM21"): cv.string,
    }
    for key in [CONF_RESET_BUTTON, CONF_PAUSE_BUTTON, CONF_RESUME_BUTTON]:
        fields[cv.Required(key)] = cv.use_id(button.Button)
    schema = cv.Schema(fields)
    if value is SCHEMA_EXTRACT:
        return schema
    return schema(value)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TubesZBWeb),
            cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(web_server_base.WebServerBase),
            cv.GenerateID(CONF_WEB_SERVER_ID): cv.use_id(web_server.WebServer),
            cv.Required(CONF_NETWORK_SETTINGS_ID): cv.use_id(network_settings.NetworkSettings),
            cv.Required(CONF_ETHERNET_ID): cv.use_id(ethernet.EthernetComponent),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_FIRMWARE_UPDATE_ID): cv.use_id(HttpRequestUpdate),
            cv.Required(CONF_MODEL): cv.string,
            cv.Optional(CONF_ZWAVE_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_SERIAL_RADIO): serial_radio_schema,
            cv.Optional(CONF_ZIGBEE): zigbee_schema,
            cv.Optional(CONF_CC2652): cc2652_schema,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_framework("esp-idf"),
    cv.has_at_most_one_key(CONF_ZIGBEE, CONF_CC2652, CONF_SERIAL_RADIO),
)


def validate_dependencies(config: ConfigType) -> None:
    full = fv.full_config.get()
    if CONF_ZIGBEE in config:
        if CONF_EFR32_FLASHER not in full or CONF_EFR32_INFO not in full:
            raise cv.Invalid("Zigbee frontend requires the EFR32 flasher and information components")
    elif CONF_CC2652 in config:
        if CONF_CC2652_FLASHER not in full:
            raise cv.Invalid("CC2652 frontend requires the CC2652 flasher")
    elif CONF_SERIAL_RADIO in config:
        if CONF_STREAM_SERVER not in full:
            raise cv.Invalid("Serial radio frontend requires the existing Zigbee serial bridge")
    elif CONF_ZWAVE_PROXY not in full:
        raise cv.Invalid("Configure a Z-Wave proxy or the Zigbee frontend bindings")

    radio_key = next((k for k in (CONF_ZIGBEE, CONF_CC2652, CONF_SERIAL_RADIO) if k in config), None)
    if radio_key is not None:
        streams = full[CONF_STREAM_SERVER]
        if isinstance(streams, dict):
            streams = [streams]
        stream = next(s for s in streams if str(s[CONF_ID]) == str(config[radio_key][CONF_STREAM_SERVER_ID]))
        if str(stream[CONF_UART_ID]) != str(config[CONF_UART_ID]):
            raise cv.Invalid("The Zigbee frontend UART must match its serial bridge")
        flasher_key = {
            CONF_ZIGBEE: CONF_EFR32_FLASHER,
            CONF_CC2652: CONF_CC2652_FLASHER,
        }.get(radio_key)
        if flasher_key:
            flashers = full[flasher_key]
            if isinstance(flashers, dict):
                flashers = [flashers]
            if not any(str(f[CONF_UART_ID]) == str(config[CONF_UART_ID]) for f in flashers):
                raise cv.Invalid("The Zigbee frontend UART must match its native flasher")

    if radio_key is not None and CONF_ZWAVE_PROXY in full and CONF_ZWAVE_UART_ID not in config:
        raise cv.Invalid("Combined Zigbee/Z-Wave frontends require the separate zwave_uart_id binding")
    if CONF_ZWAVE_UART_ID in config:
        if CONF_ZWAVE_PROXY not in full or not any(k in config for k in (CONF_ZIGBEE, CONF_CC2652, CONF_SERIAL_RADIO)):
            raise cv.Invalid("Dual-radio frontend requires Zigbee and a Z-Wave proxy")
        if str(full[CONF_ZWAVE_PROXY][CONF_UART_ID]) != str(config[CONF_ZWAVE_UART_ID]):
            raise cv.Invalid("zwave_uart_id must match the Z-Wave proxy UART")
        if str(config[CONF_UART_ID]) == str(config[CONF_ZWAVE_UART_ID]):
            raise cv.Invalid("Zigbee and Z-Wave must use separate UARTs")


FINAL_VALIDATE_SCHEMA = validate_dependencies


async def to_code(config: ConfigType) -> None:
    dependencies = [
        await cg.get_variable(config[key])
        for key in [
            CONF_WEB_SERVER_BASE_ID,
            CONF_WEB_SERVER_ID,
            CONF_NETWORK_SETTINGS_ID,
            CONF_ETHERNET_ID,
            CONF_UART_ID,
            CONF_FIRMWARE_UPDATE_ID,
        ]
    ]
    var = cg.new_Pvariable(config[CONF_ID], *dependencies)
    await cg.register_component(var, config)
    cg.add_define("USE_OTA_STATE_LISTENER")
    if any(k in config for k in (CONF_ZIGBEE, CONF_CC2652, CONF_SERIAL_RADIO)):
        cg.add_define("USE_TUBESZB_ZIGBEE")
    if CONF_CC2652 in config:
        cg.add_define("USE_TUBESZB_CC2652")
        for key, value in config[CONF_CC2652].items():
            cg.add(getattr(var, "set_" + key)(await cg.get_variable(value)))
    if CONF_ZIGBEE in config:
        cg.add_define("USE_TUBESZB_EFR32")
        for key, value in config[CONF_ZIGBEE].items():
            cg.add(getattr(var, "set_" + key)(await cg.get_variable(value)))
    if CONF_SERIAL_RADIO in config:
        cg.add_define("USE_TUBESZB_SERIAL_RADIO")
        for key, value in config[CONF_SERIAL_RADIO].items():
            cg.add(getattr(var, "set_" + key)(value if key == CONF_RADIO_MODEL else await cg.get_variable(value)))
    if CONF_ZIGBEE in config or CONF_CC2652 in config:
        key = CONF_EFR32_FLASHER if CONF_ZIGBEE in config else CONF_CC2652_FLASHER
        flashers = CORE.config[key]
        if isinstance(flashers, dict):
            flashers = [flashers]
        flasher = next(f for f in flashers if str(f[CONF_UART_ID]) == str(config[CONF_UART_ID]))
        cg.add(var.set_radio_flasher(await cg.get_variable(flasher[CONF_ID])))
    if CONF_ZWAVE_UART_ID in config:
        cg.add_define("USE_TUBESZB_DUAL_RADIO")
        cg.add(var.set_zwave_uart(await cg.get_variable(config[CONF_ZWAVE_UART_ID])))
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_board(CORE.config[CONF_ESP32].get(CONF_BOARD, "ESP32")))
    page = gzip.compress(embedded_page().encode(), mtime=0)
    cg.add_global(
        cg.RawStatement(
            "namespace esphome::tubeszb_web {\nextern const uint8_t FRONTEND_GZ[] = {"
            + ",".join(str(byte) for byte in page)
            + "};\nextern const size_t FRONTEND_GZ_SIZE = sizeof(FRONTEND_GZ);\n}"
        )
    )
