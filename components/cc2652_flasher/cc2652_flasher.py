import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, gpio
from esphome.const import CONF_ID, CONF_UART_ID

# Define configuration keys.
CONF_MANIFEST_URL = "manifest_url"
CONF_BSL_OUTPUT = "bsl_output"
CONF_RESET_OUTPUT = "reset_output"
CONF_FLASHING_BAUD_RATE = "flashing_baud_rate"

DEPENDENCIES = ["uart", "gpio"]
AUTO_LOAD = ["uart", "gpio"]

# Create a namespace for our component.
cc2652_flasher_ns = cg.esphome_ns.namespace("cc2652_flasher")
CC2652FlasherComponent = cc2652_flasher_ns.class_(
    "CC2652FlasherComponent", cg.Component, uart.UARTDevice
)

# Define the configuration schema.
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CC2652FlasherComponent),
        cv.Required(CONF_MANIFEST_URL): cv.string,
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_BSL_OUTPUT): cv.use_id(gpio.GPIOBinaryOutput),
        cv.Required(CONF_RESET_OUTPUT): cv.use_id(gpio.GPIOBinaryOutput),
        cv.Optional(CONF_FLASHING_BAUD_RATE, default=500000): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

def to_code(config):
    # Create a new instance of the component.
    var = cg.new_Pvariable(config[CONF_ID])
    
    # Set the manifest URL.
    cg.add(var.set_manifest_url(config[CONF_MANIFEST_URL]))
    
    # Set the flashing baud rate (e.g., 500000).
    cg.add(var.set_flashing_baud_rate(config[CONF_FLASHING_BAUD_RATE]))
    
    # Resolve and assign the BSL GPIO output.
    bsl = yield cg.get_variable(config[CONF_BSL_OUTPUT])
    cg.add(var.set_bsl_output(bsl))
    
    # Resolve and assign the reset GPIO output.
    reset = yield cg.get_variable(config[CONF_RESET_OUTPUT])
    cg.add(var.set_reset_output(reset))
    
    # Register this component as a UART device, using the provided uart_id.
    yield uart.register_uart_device(var, config)
