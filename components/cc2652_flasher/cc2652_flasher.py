import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, output
from esphome.const import CONF_ID

# Define configuration keys.
CONF_MANIFEST_URL = "manifest_url"
CONF_UART_ID = "uart_id"
CONF_BSL_OUTPUT = "bsl_output"
CONF_RESET_OUTPUT = "reset_output"
CONF_FLASHING_BAUD_RATE = "flashing_baud_rate"

DEPENDENCIES = ["uart", "output"]
AUTO_LOAD = ["uart", "output"]

# Create a namespace for our component.
cc2652_flasher_ns = cg.esphome_ns.namespace("cc2652_flasher")
CC2652FlasherComponent = cc2652_flasher_ns.class_(
    "CC2652FlasherComponent", cg.Component, uart.UARTDevice
)

# Define the configuration schema.
#
# Required configuration items:
#  - manifest_url: URL for the manifest JSON.
#  - uart_id: ID of the UART bus to use.
#  - bsl_output: ID of the GPIO output for the bootloader (BSL) signal.
#  - reset_output: ID of the GPIO output for the reset signal.
#
# Optional:
#  - flashing_baud_rate: The baud rate to switch to during flashing (default: 500000).
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CC2652FlasherComponent),
        cv.Required(CONF_MANIFEST_URL): cv.string,
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_BSL_OUTPUT): cv.use_id(output.GPIOBinaryOutput),
        cv.Required(CONF_RESET_OUTPUT): cv.use_id(output.GPIOBinaryOutput),
        cv.Optional(CONF_FLASHING_BAUD_RATE, default=500000): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

def to_code(config):
    # Create the component instance.
    var = cg.new_Pvariable(config[CONF_ID])
    # Set the manifest URL.
    cg.add(var.set_manifest_url(config[CONF_MANIFEST_URL]))
    # Set the flashing baud rate (for example, 500000).
    cg.add(var.set_flashing_baud_rate(config[CONF_FLASHING_BAUD_RATE]))
    # Bind the GPIO outputs.
    bsl = yield cg.get_variable(config[CONF_BSL_OUTPUT])
    cg.add(var.set_bsl_output(bsl))
    reset = yield cg.get_variable(config[CONF_RESET_OUTPUT])
    cg.add(var.set_reset_output(reset))
    # Register this component on the specified UART bus.
    yield uart.register_uart_device(var, config)
