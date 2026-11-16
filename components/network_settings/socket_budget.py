"""Account for shared HTTP sockets omitted by older ESPHome versions."""

from esphome.components import socket
from esphome.core import CORE
from esphome.types import ConfigType


def register_shared_http_sockets(config: ConfigType) -> None:
    """Run after all component schemas, so native registrations are already known.

    GuardedHttp delegates to the configured HttpRequestComponent: it does not
    create another client. Reserve one shared outbound connection only when
    http_request itself has not registered one, for manifest checks/gateway OTA.
    The SingleFlight-protected connectivity probe runs in a background task and
    can overlap a foreground request, so it needs its own TCP allowance.

    ESP-IDF's HTTP server also keeps two UDP control sockets open. ESPHome
    2026.10.x omits these; newer versions register them as web_server_idf.
    """
    tcp = CORE.data.setdefault(socket.KEY_SOCKET_CONSUMERS_TCP, {})
    probe = "network_settings.connectivity_probe"
    if not tcp.get(probe, 0):
        socket.consume_sockets(1, probe, socket.SocketType.TCP)(config)
    native_http = sum(
        count for name, count in tcp.items() if name == "http_request" or name.startswith("http_request.")
    )
    fallback_http = "network_settings.http_request"
    missing_http = max(0, 1 - native_http - tcp.get(fallback_http, 0))
    if missing_http:
        socket.consume_sockets(missing_http, fallback_http, socket.SocketType.TCP)(config)

    udp = CORE.data.setdefault(socket.KEY_SOCKET_CONSUMERS_UDP, {})
    fallback_control = "network_settings.httpd_control"
    missing_control = max(0, 2 - udp.get("web_server_idf", 0) - udp.get(fallback_control, 0))
    if missing_control:
        socket.consume_sockets(missing_control, fallback_control, socket.SocketType.UDP)(config)
