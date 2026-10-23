import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC, STATE_CLASS_TOTAL_INCREASING

from . import StreamServerComponent

CONF_STREAM_SERVER = "stream_server"
CONF_DISCONNECTS = "disconnects"
CONF_CONNECTION_COUNT = "connection_count"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_STREAM_SERVER): cv.use_id(StreamServerComponent),
        # Client sessions that ended other than by stream_server.pause (peer
        # close, reset, keepalive timeout, send/recv error). Counts since boot.
        cv.Optional(CONF_DISCONNECTS): sensor.sensor_schema(
            icon="mdi:lan-disconnect",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        # stream_server enforces a single client, so a count is redundant.
        cv.Optional(CONF_CONNECTION_COUNT): cv.invalid(
            "stream_server: connection_count sensor has been removed. "
            "Use the stream_server binary_sensor (connected) instead."
        ),
    }
)


async def to_code(config):
    server = await cg.get_variable(config[CONF_STREAM_SERVER])
    if conf := config.get(CONF_DISCONNECTS):
        sens = await sensor.new_sensor(conf)
        cg.add(server.set_disconnects_sensor(sens))
