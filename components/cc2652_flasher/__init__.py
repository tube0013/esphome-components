import esphome.config_validation as cv
from esphome.const import (
    CONF_ID
)
from esphome import core, automation
import esphome.codegen as cg

CODEOWNERS = ["@<your-github-user>"]
DEPENDENCIES = ["uart", "http_request"]
AUTO_LOAD = ["switch", "uart", "http_request"]

# Tell ESPHome we'll have a namespace `cc2652_flasher`
cc2652_flasher_ns = cg.esphome_ns.namespace('cc2652_flasher')
CC2652FlasherClass = cc2652_flasher_ns.class_('CC2652Flasher', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CC2652FlasherClass),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    # Create the flasher object
    var = cg.new_Pvariable(config[CONF_ID])
    # Register it so ESPHome calls setup(), loop() etc.
    cg.register_component(var, config)
