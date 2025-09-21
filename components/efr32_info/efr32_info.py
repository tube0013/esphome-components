import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.automation import register_action
from esphome.components import uart, text_sensor, binary_sensor
from esphome.const import CONF_ID

CONF_RAW_CAPTURE = "raw_capture"

try:
    from esphome.components import stream_server
except ImportError:
    stream_server = None

DEPENDENCIES = ["uart"]

CONF_UART_ID = "uart_id"
CONF_STREAM_SERVER_ID = "stream_server_id"
CONF_BUSY_BINARY_SENSOR = "busy_binary_sensor"
CONF_STATUS_TEXT = "status_text"
CONF_EZSP_VERSION_TEXT = "ezsp_version_text"
CONF_LIBRARY_STATUS_TEXT = "library_status_text"
CONF_STACK_VERSION_TEXT = "stack_version_text"
CONF_MANUFACTURER_TEXT = "manufacturer_text"
CONF_BOARD_NAME_TEXT = "board_name_text"
CONF_FIRMWARE_TEXT = "firmware_text"
CONF_IEEE_TEXT = "ieee_text"
CONF_CHANNEL_TEXT = "channel_text"
CONF_NODE_ID_TEXT = "node_id_text"
CONF_NETWORK_STATUS_TEXT = "network_status_text"
CONF_SECURITY_STATE_TEXT = "security_state_text"


efr32_info_ns = cg.esphome_ns.namespace("efr32_info")
EFR32InfoComponent = efr32_info_ns.class_("EFR32InfoComponent", cg.Component)
EFR32InfoProbeAction = efr32_info_ns.class_("EFR32InfoProbeAction")


def _maybe_stream_server_use_id(value):
    if stream_server is None:
        raise cv.Invalid("stream_server component not available")
    return cv.use_id(stream_server.StreamServerComponent)(value)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EFR32InfoComponent),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_STREAM_SERVER_ID): _maybe_stream_server_use_id,
        cv.Optional(CONF_BUSY_BINARY_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_STATUS_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_EZSP_VERSION_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_LIBRARY_STATUS_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_STACK_VERSION_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_MANUFACTURER_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_BOARD_NAME_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_FIRMWARE_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_IEEE_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_CHANNEL_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_NODE_ID_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_NETWORK_STATUS_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_SECURITY_STATE_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_RAW_CAPTURE, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


@register_action(
    "efr32_info.probe",
    EFR32InfoProbeAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(EFR32InfoComponent)}),
)
async def efr32_info_probe_to_code(config, action_id, template_arg, args):
    action = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(action.set_parent(parent))
    return action


async def to_code(config):
    cg.add_global(cg.RawStatement('#include "esphome/components/efr32_info/efr32_info.h"'), prepend=True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart(uart_comp))

    if CONF_STREAM_SERVER_ID in config:
        ss = await cg.get_variable(config[CONF_STREAM_SERVER_ID])
        cg.add(var.set_stream_server(ss))

    if CONF_BUSY_BINARY_SENSOR in config:
        busy = await cg.get_variable(config[CONF_BUSY_BINARY_SENSOR])
        cg.add(var.set_busy_sensor(busy))

    if CONF_STATUS_TEXT in config:
        sensor = await cg.get_variable(config[CONF_STATUS_TEXT])
        cg.add(var.set_status_sensor(sensor))

    if CONF_EZSP_VERSION_TEXT in config:
        sensor = await cg.get_variable(config[CONF_EZSP_VERSION_TEXT])
        cg.add(var.set_ezsp_version_sensor(sensor))

    if CONF_LIBRARY_STATUS_TEXT in config:
        sensor = await cg.get_variable(config[CONF_LIBRARY_STATUS_TEXT])
        cg.add(var.set_library_status_sensor(sensor))

    if CONF_STACK_VERSION_TEXT in config:
        sensor = await cg.get_variable(config[CONF_STACK_VERSION_TEXT])
        cg.add(var.set_stack_version_sensor(sensor))

    if CONF_MANUFACTURER_TEXT in config:
        sensor = await cg.get_variable(config[CONF_MANUFACTURER_TEXT])
        cg.add(var.set_manufacturer_sensor(sensor))

    if CONF_BOARD_NAME_TEXT in config:
        sensor = await cg.get_variable(config[CONF_BOARD_NAME_TEXT])
        cg.add(var.set_board_name_sensor(sensor))

    if CONF_FIRMWARE_TEXT in config:
        sensor = await cg.get_variable(config[CONF_FIRMWARE_TEXT])
        cg.add(var.set_firmware_sensor(sensor))

    if CONF_IEEE_TEXT in config:
        sensor = await cg.get_variable(config[CONF_IEEE_TEXT])
        cg.add(var.set_ieee_sensor(sensor))

    if CONF_CHANNEL_TEXT in config:
        sensor = await cg.get_variable(config[CONF_CHANNEL_TEXT])
        cg.add(var.set_channel_sensor(sensor))

    if CONF_NODE_ID_TEXT in config:
        sensor = await cg.get_variable(config[CONF_NODE_ID_TEXT])
        cg.add(var.set_node_id_sensor(sensor))

    if CONF_NETWORK_STATUS_TEXT in config:
        sensor = await cg.get_variable(config[CONF_NETWORK_STATUS_TEXT])
        cg.add(var.set_network_status_sensor(sensor))

    if CONF_SECURITY_STATE_TEXT in config:
        sensor = await cg.get_variable(config[CONF_SECURITY_STATE_TEXT])
        cg.add(var.set_security_state_sensor(sensor))

    if CONF_RAW_CAPTURE in config:
        cg.add(var.set_raw_capture(config[CONF_RAW_CAPTURE]))
