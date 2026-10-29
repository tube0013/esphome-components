"""Production asset packaging and native web policy helpers."""
from component_paths import COMPONENTS_ROOT
from pathlib import Path
import re
import gzip
import importlib.util
from html.parser import HTMLParser
ROOT = Path(__file__).resolve().parents[1]

def source_text(relative: str) -> str:
    # These are binding checks, independent of formatting and explicit member qualification.
    return re.sub(r"\s+", " ", (COMPONENTS_ROOT / relative).read_text().replace("this->", ""))


def test_embedded_frontend_assets():
    spec = importlib.util.spec_from_file_location("tubeszb_assets", COMPONENTS_ROOT / "tubeszb_web/assets.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    page = module.embedded_page()
    assert "<!-- BRAND_SYMBOL -->" not in page
    assert "/* NETWORK_VALIDATION */" not in page
    assert "/* FRONTEND_SCRIPT */" not in page
    assert "Design preview" not in page
    assert "window.openai" not in page
    assert 'viewBox="43.4 0 17.1 20.6"' in page
    assert "Download logs" in page
    assert len(gzip.compress(page.encode(), mtime=0)) < 30000

    class Assets(HTMLParser):
        def handle_starttag(self, tag, attrs):
            attrs = dict(attrs)
            if tag in ("script", "img", "link"):
                assert not attrs.get("src", "").startswith(("https://", "http://"))
                assert not attrs.get("href", "").startswith(("https://", "http://"))

    Assets().feed(page)


def test_http_statuses_supported_by_esphome_idf():
    """Catch accepted actions becoming HTTP 500 in the real IDF response adapter."""
    import re
    import esphome

    adapter = Path(esphome.__file__).parent / "components/web_server_idf/web_server_idf.cpp"
    source = adapter.read_text().split("void AsyncWebServerRequest::init_response_(", 1)[1]
    source = source.split("httpd_resp_set_status(*this, status);", 1)[0]
    supported = {int(code) for code in re.findall(r"case (\d+):", source)} | {500}
    for relative in ("tubeszb_web/tubeszb_web.cpp", "network_settings/network_settings.cpp"):
        handlers = (COMPONENTS_ROOT / relative).read_text()
        emitted = {int(code) for code in re.findall(
            r"(?:send_json_|send_error_|error_|reject)\((?:request,\s*)?(\d+),", handlers
        )}
        assert emitted <= supported, f"{relative}: IDF maps {emitted - supported} to HTTP 500"


def test_custom_radio_url_validation(tmp_path):
    """Exercise the same URL policy the device uses before saving or flashing."""
    import subprocess
    target = tmp_path / "gbl-url-test"
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(COMPONENTS_ROOT.parent),
                    str(ROOT / "tests/gbl_url_test.cpp"), "-o", str(target)], check=True)
    subprocess.run([str(target)], check=True)


def test_cc2652_native_log_events(tmp_path):
    """Percentages and transfer messages cannot stand in for a native flash result."""
    import subprocess
    target = tmp_path / "cc2652-log-test"
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(COMPONENTS_ROOT.parent),
                    str(ROOT / "tests/cc2652_log_test.cpp"), "-o", str(target)], check=True)
    subprocess.run([str(target)], check=True)


def test_cc2652_flash_session(tmp_path):
    """Restore the original bridge state after success, failure and retry."""
    import subprocess
    namespace = tmp_path / "include/esphome/components"
    namespace.mkdir(parents=True)
    (namespace / "zigbee_firmware_baud").symlink_to(COMPONENTS_ROOT / "zigbee_firmware_baud", target_is_directory=True)
    target = tmp_path / "cc2652-flash-session-test"
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(COMPONENTS_ROOT.parent),
                    "-I", str(tmp_path / "include"), str(ROOT / "tests/cc2652_flash_session_test.cpp"), "-o", str(target)], check=True)
    subprocess.run([str(target)], check=True)



def test_native_operation_status_bindings():
    web=source_text('tubeszb_web/tubeszb_web.cpp')
    install=web.split('} else if (command == Command::COMMAND_INSTALL) {',1)[1].split('#ifdef USE_TUBESZB_ZIGBEE',1)[0]
    assert 'update_->state == update::UPDATE_STATE_AVAILABLE' in install
    assert 'valid_ota_url(update_->update_info.firmware_url)' in install
    assert 'gateway_start_.arm(now)' in install
    assert 'update_->on_ota_state(ota::OTA_ERROR' in web
    queued=web.split('} else if (url == "/tubeszb/install-update") {',1)[1].split('#ifdef USE_TUBESZB_ZIGBEE',1)[0]
    assert 'gateway_ota_state_ = 1' not in queued
    for family in ('efr32','cc2652'):
        support=source_text('tubeszb_web/'+family+'_support.cpp')
        assert 'radio_flasher_->set_flash_listener' in support
        native=(COMPONENTS_ROOT/(family+'_flasher')/(family+'_flasher.cpp')).read_text()
        assert 'FlashAttempt attempt(flash_status_)' in native
        assert 'if (ok) attempt.succeed();' in native
    efr=source_text('tubeszb_web/efr32_support.cpp')
    assert '>= 35000' in efr and 'Zigbee flash did not start' in efr


def test_restart_recovery_bypasses_queued_but_not_active_writes():
    source=source_text('tubeszb_web/tubeszb_web.cpp')
    handle=source.split('void TubesZBWeb::handleRequest(',1)[1]
    assert handle.index('if (!authorized_(request))') < handle.index('if (url == "/tubeszb/restart")')
    assert handle.index('if (url == "/tubeszb/restart")') < handle.index('if (restart_guard_.requested() || action_pending_')
    assert 'restart_guard_.request(gateway_ota_state_ == 1, radio_flash_active_)' in handle
    assert 'restart_guard_.ready(gateway_ota_state_ == 1, radio_flash_active_)' in source
    for family in ('efr32','cc2652'):
        callback=source_text('tubeszb_web/'+family+'_support.cpp')
        assert 'radio_flash_active_ = status.result == 1' in callback


def test_idle_sampling_does_not_recopy_sensor_strings():
    header=source_text('tubeszb_web/tubeszb_web.h')
    snapshot=header.split('struct Snapshot {',1)[1].split('} snapshot_;',1)[0]
    assert 'std::string' not in snapshot
    assert 'std::is_trivially_copyable_v<Snapshot>' in header
    for family in ('cc2652','efr32'):
        source=source_text('tubeszb_web/'+family+'_support.cpp')
        sample=source.split('void TubesZBWeb::sample_radio_()',1)[1].split('void TubesZBWeb::radio_json_',1)[0]
        for field in ('firmware','latest','board','manufacturer','ieee','url'):
            assert not re.search(r'radio_\.'+field+r'\s*=',sample)
        assert 'radio_source_key_cache_' in source
        key=source.split('TubesZBWeb::radio_source_key_()',1)[1].split('}',1)[0]
        assert 'return radio_source_key_cache_' in key and 'std::string(' not in key
        assert 'sensor->add_on_state_callback' in source and 'radio_.*field = sensor->state' in source


def test_loop_timing_uses_the_cached_component_tick():
    for domain,cls in [('tubeszb_web','TubesZBWeb'),('network_settings','NetworkSettings')]:
        source=source_text(domain+'/'+domain+'.cpp')
        loop=source.split('void '+cls+'::loop()',1)[1].split('void '+cls+'::dump_config()',1)[0]
        assert 'App.get_loop_component_start_time()' in loop
        assert not re.search(r'\bmillis\(\)',loop)


def test_baud_label_callback_matches_installed_select_api() -> None:
    import esphome
    select=(Path(esphome.__file__).parent/'components/select/select.h').read_text()
    assert 'LazyCallbackManager<void(size_t)> state_callback_' in select
    for name in ('efr32_support.cpp','serial_support.cpp'):
        source=source_text('tubeszb_web/'+name)
        assert 'baud_->add_on_state_callback([this](size_t)' in source
        assert 'radio_.selected_baud = baud_->current_option()' in source
