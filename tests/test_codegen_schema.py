"""Public schema extraction must include each supported radio's bindings."""
from pathlib import Path
import importlib
from esphome import loader
from esphome.schema_extractors import SCHEMA_EXTRACT

ROOT = Path(__file__).resolve().parents[1]


def test_dynamic_radio_schemas_are_extractable() -> None:
    loader.install_meta_finder(ROOT / "components")
    web = importlib.import_module("esphome.components.tubeszb_web")
    for name, keys in [
        ("zigbee_schema", {"flash_button", "busy_sensor", "firmware_sensor", "custom_url_text"}),
        ("cc2652_schema", {"flash_script", "boot_probe_script", "board_sensor", "flash_button"}),
        ("serial_radio_schema", {"stream_server_id", "reset_button", "baud_select", "radio_model"}),
    ]:
        schema = getattr(web, name)(SCHEMA_EXTRACT)
        assert keys <= {str(key) for key in schema.schema}
    for domain in ("network_settings", "tubeszb_web", "cc2652_flash_guard", "zigbee_firmware_baud", "cc2652_flasher", "efr32_flasher"):
        assert importlib.import_module("esphome.components." + domain).DOMAIN == domain
