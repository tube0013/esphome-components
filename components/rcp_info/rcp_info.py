import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.automation import maybe_simple_id, register_action
from esphome.components import uart, text_sensor
from esphome.const import CONF_ID

try:
    from esphome.components import stream_server
except ImportError:
    stream_server = None

DEPENDENCIES = ["uart"]

CONF_UART_ID = "uart_id"
CONF_STREAM_SERVER_ID = "stream_server_id"
CONF_STATUS_TEXT = "status_text"
CONF_VERSION_TEXT = "version_text"
CONF_IEEE_TEXT = "ieee_text"
CONF_BAUD_RATES = "baud_rates"
CONF_PER_BAUD_TIMEOUT = "per_baud_timeout"

rcp_info_ns = cg.esphome_ns.namespace("rcp_info")
RCPInfoComponent = rcp_info_ns.class_("RCPInfoComponent", cg.Component)
RCPInfoProbeAction = rcp_info_ns.class_("RCPInfoProbeAction")


def _maybe_stream_server_use_id(value):
    if stream_server is None:
        raise cv.Invalid("stream_server component not available")
    return cv.use_id(stream_server.StreamServerComponent)(value)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RCPInfoComponent),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_STREAM_SERVER_ID): _maybe_stream_server_use_id,
        cv.Optional(CONF_STATUS_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_VERSION_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_IEEE_TEXT): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_BAUD_RATES, default=[921600, 460800, 115200]): cv.ensure_list(
            cv.int_range(min=1200, max=3000000)
        ),
        cv.Optional(
            CONF_PER_BAUD_TIMEOUT, default="2s"
        ): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)


@register_action(
    "rcp_info.probe",
    RCPInfoProbeAction,
    maybe_simple_id({cv.Required(CONF_ID): cv.use_id(RCPInfoComponent)}),
    synchronous=False,
)
async def rcp_info_probe_to_code(config, action_id, template_arg, args):
    action = cg.new_Pvariable(action_id, template_arg)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(action.set_parent(parent))
    return action


async def to_code(config):
    cg.add_global(cg.RawStatement('#include "esphome/components/rcp_info/rcp_info.h"'), prepend=True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart(uart_comp))

    if CONF_STREAM_SERVER_ID in config:
        ss = await cg.get_variable(config[CONF_STREAM_SERVER_ID])
        cg.add(var.set_stream_server(ss))

    if CONF_STATUS_TEXT in config:
        sensor = await cg.get_variable(config[CONF_STATUS_TEXT])
        cg.add(var.set_status_sensor(sensor))

    if CONF_VERSION_TEXT in config:
        sensor = await cg.get_variable(config[CONF_VERSION_TEXT])
        cg.add(var.set_version_sensor(sensor))

    if CONF_IEEE_TEXT in config:
        sensor = await cg.get_variable(config[CONF_IEEE_TEXT])
        cg.add(var.set_ieee_sensor(sensor))

    cg.add(var.set_baud_rates(config[CONF_BAUD_RATES]))
    cg.add(var.set_per_baud_timeout(config[CONF_PER_BAUD_TIMEOUT].total_milliseconds))
