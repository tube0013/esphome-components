import esphome.config_validation as cv

# The connection_count sensor has been removed. stream_server now enforces a single
# client connection, making a numeric count redundant. Use the binary_sensor
# (connected) platform instead — it reflects the same connected/disconnected state.
CONFIG_SCHEMA = cv.invalid(
    "stream_server: connection_count sensor has been removed. "
    "Use the stream_server binary_sensor (connected) instead."
)
