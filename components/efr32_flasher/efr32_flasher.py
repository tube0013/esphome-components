import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch as switch_, text_sensor, binary_sensor
from esphome.const import CONF_ID, CONF_URL
from esphome.automation import register_action

DEPENDENCIES = ["uart", "switch"]
AUTO_LOAD = ["md5", "json"]

CONF_BL_SWITCH = "boot_switch"
CONF_RST_SWITCH = "rst_switch"
CONF_DEBUG = "debug"
CONF_SHOW_PROGRESS = "show_progress"
CONF_PROGRESS_STEP = "progress_step"
CONF_FW_VERSION_TEXT = "fw_version_text"
CONF_LATEST_FW_TEXT = "latest_fw_version_text"
CONF_EZSP_PROTOCOL_TEXT = "ezsp_protocol_text"
CONF_INSTALLED_STACK_TEXT = "installed_stack_text"
CONF_XNCP_VERSION_TEXT = "xncp_version_text"
CONF_IEEE_TEXT = "ieee_text"
CONF_MANUF_ID_TEXT = "manuf_id_text"
CONF_BOARD_NAME_TEXT = "board_name_text"
CONF_MFG_STRING_TEXT = "mfg_string_text"
CONF_CHIP_TEXT = "chip_text"
CONF_VARIANT = "variant"
CONF_BUSY_BINARY_SENSOR = "busy_binary_sensor"
efr32_ns = cg.esphome_ns.namespace("efr32_flasher")
EFR32Flasher = efr32_ns.class_("EFR32Flasher", cg.Component)
UpdateFirmwareAction = efr32_ns.class_("UpdateFirmwareAction")
CheckUpdateAction = efr32_ns.class_("CheckUpdateAction")

CONF_PAUSE_SWITCH = "pause_switch"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EFR32Flasher),
        cv.Required(CONF_URL): cv.string_strict,
        cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_BL_SWITCH): cv.use_id(switch_.Switch),
        cv.Required(CONF_RST_SWITCH): cv.use_id(switch_.Switch),
        cv.Optional(CONF_DEBUG, default=False): cv.boolean,
        cv.Optional(CONF_SHOW_PROGRESS, default=True): cv.boolean,
        cv.Optional(CONF_PROGRESS_STEP, default=5): cv.int_range(min=1, max=50),
        cv.Optional(CONF_FW_VERSION_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_LATEST_FW_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_EZSP_PROTOCOL_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_INSTALLED_STACK_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_XNCP_VERSION_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_IEEE_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_MANUF_ID_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_BOARD_NAME_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_MFG_STRING_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_CHIP_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_PAUSE_SWITCH): cv.use_id(switch_.Switch),
        cv.Optional(CONF_BUSY_BINARY_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        # variant: auto | MGM24 | BM24 (case-insensitive)
        cv.Optional(CONF_VARIANT, default="auto"): cv.one_of("auto", "mgm24", "bm24", lower=True),
    }
).extend(cv.COMPONENT_SCHEMA)


@register_action("efr32_flasher.update_firmware", UpdateFirmwareAction, cv.Schema({cv.Required(CONF_ID): cv.use_id(EFR32Flasher)}))
async def update_firmware_action_to_code(config, action_id, template_arg, args):
    act = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(act.set_parent(parent))
    return act


@register_action("efr32_flasher.check_update", CheckUpdateAction, cv.Schema({cv.Required(CONF_ID): cv.use_id(EFR32Flasher)}))
async def check_update_action_to_code(config, action_id, template_arg, args):
    act = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(act.set_parent(parent))
    return act


async def to_code(config):
    # Ensure our C++ header is visible to the generated build (local and external_components).
    cg.add_global(cg.RawStatement('#include "esphome/components/efr32_flasher/efr32_flasher.h"'), prepend=True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_update_url(config[CONF_URL]))

    uart_comp = await cg.get_variable(config["uart_id"])
    bl = await cg.get_variable(config[CONF_BL_SWITCH])
    rst = await cg.get_variable(config[CONF_RST_SWITCH])

    cg.add(var.set_uart(uart_comp))
    cg.add(var.set_bl_switch(bl))
    cg.add(var.set_rst_switch(rst))

    if CONF_DEBUG in config:
        cg.add(var.set_verbose(config[CONF_DEBUG]))
    if CONF_SHOW_PROGRESS in config:
        cg.add(var.set_show_progress(config[CONF_SHOW_PROGRESS]))
    if CONF_PROGRESS_STEP in config:
        cg.add(var.set_progress_step(config[CONF_PROGRESS_STEP]))

    if CONF_FW_VERSION_TEXT in config:
        fw_txt = await cg.get_variable(config[CONF_FW_VERSION_TEXT])
        cg.add(var.set_fw_text_sensor(fw_txt))
    if CONF_LATEST_FW_TEXT in config:
        latest_txt = await cg.get_variable(config[CONF_LATEST_FW_TEXT])
        cg.add(var.set_latest_text_sensor(latest_txt))

    if CONF_EZSP_PROTOCOL_TEXT in config:
        t = await cg.get_variable(config[CONF_EZSP_PROTOCOL_TEXT])
        cg.add(var.set_ezsp_protocol_text(t))
    if CONF_INSTALLED_STACK_TEXT in config:
        t = await cg.get_variable(config[CONF_INSTALLED_STACK_TEXT])
        cg.add(var.set_installed_stack_text(t))
    if CONF_XNCP_VERSION_TEXT in config:
        t = await cg.get_variable(config[CONF_XNCP_VERSION_TEXT])
        cg.add(var.set_xncp_version_text(t))
    if CONF_IEEE_TEXT in config:
        t = await cg.get_variable(config[CONF_IEEE_TEXT])
        cg.add(var.set_ieee_text(t))
    if CONF_MANUF_ID_TEXT in config:
        t = await cg.get_variable(config[CONF_MANUF_ID_TEXT])
        cg.add(var.set_manuf_id_text(t))
    if CONF_BOARD_NAME_TEXT in config:
        t = await cg.get_variable(config[CONF_BOARD_NAME_TEXT])
        cg.add(var.set_board_name_text(t))
    if CONF_MFG_STRING_TEXT in config:
        t = await cg.get_variable(config[CONF_MFG_STRING_TEXT])
        cg.add(var.set_mfg_string_text(t))
    if CONF_CHIP_TEXT in config:
        t = await cg.get_variable(config[CONF_CHIP_TEXT])
        cg.add(var.set_chip_text(t))

    if CONF_VARIANT in config:
        v = config[CONF_VARIANT]
        m = {"auto": 0, "mgm24": 1, "bm24": 2}
        cg.add(var.set_variant(m.get(v, 0)))
    if CONF_PAUSE_SWITCH in config:
        ps = await cg.get_variable(config[CONF_PAUSE_SWITCH])
        cg.add(var.set_pause_switch(ps))
    if CONF_BUSY_BINARY_SENSOR in config:
        bs = await cg.get_variable(config[CONF_BUSY_BINARY_SENSOR])
        cg.add(var.set_busy_sensor(bs))

# Old probe actions removed to keep component lean
