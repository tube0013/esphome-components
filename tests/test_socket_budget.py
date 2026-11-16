"""Socket accounting follows actual transports and does not count shared HTTP twice."""

import importlib
from pathlib import Path

import pytest
from esphome import loader
from esphome.components import socket
from esphome.core import CORE

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def registry(monkeypatch):
    monkeypatch.setattr(CORE, "data", {})
    loader.install_meta_finder(ROOT / "components")
    return CORE.data


@pytest.mark.parametrize("domain,boot_key", [("cc2652_flasher", "bsl_switch"), ("efr32_flasher", "boot_switch")])
def test_radio_flashers_register_one_tcp_connection_per_instance(registry, domain, boot_key):
    component = importlib.import_module("esphome.components." + domain)
    for instance in (1, 2):
        component.CONFIG_SCHEMA(
            {
                "id": f"flasher_{instance}",
                "url": "https://example.com/firmware.json",
                "uart_id": "radio_uart",
                "rst_switch": "reset0",
                boot_key: "boot0",
            }
        )
        assert registry[socket.KEY_SOCKET_CONSUMERS_TCP][domain] == instance
    assert "socket" in component.AUTO_LOAD
    assert socket.KEY_SOCKET_CONSUMERS_TCP_LISTEN not in registry


@pytest.mark.parametrize("native_http,native_control", [(0, 0), (0, 2), (1, 0), (1, 2), (3, 2)])
def test_shared_http_fills_only_missing_native_registrations(registry, native_http, native_control):
    budget = importlib.import_module("esphome.components.network_settings.socket_budget")
    if native_http:
        socket.consume_sockets(native_http, "http_request", socket.SocketType.TCP)({})
    if native_control:
        socket.consume_sockets(native_control, "web_server_idf", socket.SocketType.UDP)({})
    # Calling final validation again must not reserve the same shared sockets twice.
    budget.register_shared_http_sockets({})
    budget.register_shared_http_sockets({})
    counts = socket.get_socket_counts()
    assert counts.tcp == max(1, native_http) + 1
    assert registry[socket.KEY_SOCKET_CONSUMERS_TCP]["network_settings.connectivity_probe"] == 1
    assert counts.udp == 2
    assert counts.tcp_listen == 0
    if native_http:
        assert "network_settings.http_request" not in registry[socket.KEY_SOCKET_CONSUMERS_TCP]
    if native_control:
        assert "network_settings.httpd_control" not in registry[socket.KEY_SOCKET_CONSUMERS_UDP]
