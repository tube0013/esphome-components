import logging

from esphome import automation
import esphome.codegen as cg
from esphome.components import socket, uart
import esphome.config_validation as cv
from esphome.const import CONF_BUFFER_SIZE, CONF_ID, CONF_PORT

_LOGGER = logging.getLogger(__name__)

# ESPHome doesn't know the Stream abstraction yet, so hardcode to use a UART for now.

AUTO_LOAD = ["socket"]

DEPENDENCIES = ["uart", "network"]

MULTI_CONF = True

ns = cg.global_ns
StreamServerComponent = ns.class_("StreamServerComponent", cg.Component)
PauseAction = ns.class_("PauseAction", automation.Action)
ResumeAction = ns.class_("ResumeAction", automation.Action)

CONF_KEEP_ALIVE = "keep_alive"
CONF_IDLE_TIME = "idle_time"
CONF_INTERVAL = "interval"
CONF_COUNT = "count"


def validate_buffer_size(buffer_size):
    if buffer_size & (buffer_size - 1) != 0:
        raise cv.Invalid("Buffer size must be a power of two.")
    _LOGGER.warning(
        "stream_server: 'buffer_size' no longer has any effect (the ring buffer was "
        "replaced by fixed 512-byte staging buffers with TCP/UART backpressure) and "
        "can be removed. To absorb radio bursts while the network stalls, raise the "
        "uart 'rx_buffer_size' instead."
    )
    return buffer_size


def validate_keep_alive(value):
    # `keep_alive: false` disables TCP keepalive; omitting it uses the defaults
    # (10s idle, 5s interval, 3 probes) so half-open clients free the single slot.
    if value is False or (isinstance(value, str) and value.lower() == "false"):
        return False
    return cv.Schema(
        {
            cv.Required(CONF_IDLE_TIME): cv.positive_time_period_seconds,
            cv.Required(CONF_INTERVAL): cv.positive_time_period_seconds,
            cv.Required(CONF_COUNT): cv.positive_int,
        }
    )(value)


CONFIG_SCHEMA = cv.All(
    # register_action(synchronous=...) below requires ESPHome 2026.3.0.
    cv.require_esphome_version(2026, 3, 0),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(StreamServerComponent),
            cv.Optional(CONF_PORT, default=6638): cv.port,
            # Deprecated, accepted for existing configs; see validate_buffer_size.
            cv.Optional(CONF_BUFFER_SIZE): cv.All(
                cv.positive_int, validate_buffer_size
            ),
            cv.Optional("trace", default=False): cv.boolean,
            cv.Optional(CONF_KEEP_ALIVE): validate_keep_alive,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    # The pump runs in a FreeRTOS task and drives the ESP-IDF UART driver directly.
    cv.only_on_esp32,
    # Listener + client, plus one transient socket while a newcomer is refused.
    socket.consume_sockets(1, "stream_server", socket.SocketType.TCP_LISTEN),
    socket.consume_sockets(2, "stream_server", socket.SocketType.TCP),
)

# NOTE: no FINAL_VALIDATE_SCHEMA claiming the UART. The UART is deliberately
# shared with cc2652_flasher / efr32_flasher / efr32_info / rcp_info, which hand
# it over via stream_server.pause / stream_server.resume.


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_trace(config["trace"]))
    keep_alive = config.get(CONF_KEEP_ALIVE)
    if keep_alive is False:
        cg.add(var.set_keep_alive(0, 0, 0))
    elif keep_alive is not None:
        cg.add(
            var.set_keep_alive(
                keep_alive[CONF_IDLE_TIME].total_seconds,
                keep_alive[CONF_INTERVAL].total_seconds,
                keep_alive[CONF_COUNT],
            )
        )

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


@automation.register_action(
    "stream_server.pause",
    PauseAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(StreamServerComponent)}),
    synchronous=False,
)
async def stream_server_pause_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


@automation.register_action(
    "stream_server.resume",
    ResumeAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(StreamServerComponent)}),
    synchronous=False,
)
async def stream_server_resume_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
