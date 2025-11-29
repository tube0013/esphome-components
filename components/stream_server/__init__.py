from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PORT, CONF_BUFFER_SIZE

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
    return buffer_size


CONFIG_SCHEMA = cv.All(
    cv.require_esphome_version(2022, 3, 0),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(StreamServerComponent),
            cv.Optional(CONF_PORT, default=6638): cv.port,
            cv.Optional(CONF_BUFFER_SIZE, default=128): cv.All(
                cv.positive_int, validate_buffer_size
            ),
            cv.Optional("trace", default=False): cv.boolean,
            cv.Optional(CONF_KEEP_ALIVE): cv.Schema(
                {
                    cv.Required(CONF_IDLE_TIME): cv.positive_time_period_seconds,
                    cv.Required(CONF_INTERVAL): cv.positive_time_period_seconds,
                    cv.Required(CONF_COUNT): cv.positive_int,
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_buffer_size(config[CONF_BUFFER_SIZE]))
    if "trace" in config:
        cg.add(var.set_trace(config["trace"]))
    if CONF_KEEP_ALIVE in config:
        keep_alive_config = config[CONF_KEEP_ALIVE]
        cg.add(
            var.set_keep_alive(
                keep_alive_config[CONF_IDLE_TIME], keep_alive_config[CONF_INTERVAL], keep_alive_config[CONF_COUNT]
            )
        )

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


@automation.register_action(
    "stream_server.pause",
    PauseAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(StreamServerComponent)}),
)
async def stream_server_pause_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


@automation.register_action(
    "stream_server.resume",
    ResumeAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(StreamServerComponent)}),
)
async def stream_server_resume_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
