import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import CONF_ID, CONF_UART_ID

# Define configuration keys.
CONF_MANIFEST_URL = "manifest_url"
CONF_BSL_OUTPUT = "bsl_output"
CONF_RESET_OUTPUT = "reset_output"
CONF_FLASHING_BAUD_RATE = "flashing_baud_rate"

DEPENDENCIES = ["uart", "switch"]
AUTO_LOAD = ["uart", "switch"]

# Create a namespace for the component.
cc2652_flasher_ns = cg.esphome_ns.namespace("cc2652_flasher")
CC2652FlasherComponent = cc2652_flasher_ns.class_(
    "CC2652FlasherComponent", cg.Component, uart.UARTDevice
)

# Note: Do not extend uart.UART_DEVICE_SCHEMA here.
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CC2652FlasherComponent),
    cv.Required("manifest_url"): cv.string,
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required("bsl_output"): cv.use_id(switch.Switch),
    cv.Required("reset_output"): cv.use_id(switch.Switch),
    cv.Optional("flashing_baud_rate", default=500000): cv.positive_int,
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_manifest_url(config["manifest_url"]))
    cg.add(var.set_flashing_baud_rate(config["flashing_baud_rate"]))
    bsl = yield cg.get_variable(config["bsl_output"])
    cg.add(var.set_bsl_output(bsl))
    reset = yield cg.get_variable(config["reset_output"])
    cg.add(var.set_reset_output(reset))
    # Register the UART device and get the pointer.
    uart_component = yield uart.register_uart_device(var, config)
    cg.add(var.set_uart_component(uart_component))
