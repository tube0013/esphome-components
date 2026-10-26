# EFR32 Info (ESPHome External Component)

`efr32_info` is an ESPHome external component that talks ASH/EZSP over UART to an
EFR32 Zigbee NCP. It provides a non-blocking *probe* action that gathers key
radio details (firmware, stack version, IEEE address, manufacturing tokens, …)
and publishes them to text sensors.

The component is designed to run alongside the `stream_server` component so the
radio can continue serving the Serial/Socket interface used by Home Assistant or
other tooling. During a probe `efr32_info` briefly pauses the stream server, runs
the ASH/EZSP sequence, then resumes normal traffic.

## Features

* Runs the full ASH handshake (RST → RSTACK) and v4/v8 `getVersion` exchange.
* Collects EZSP state: IEEE address, stack version, firmware version, EZSP
  protocol version, library status, security/network status (when available).
* Reads firmware identity from Nabu Casa XNCP overrides when advertised, with
  per-field fallback to `MFG_STRING` / `MFG_BOARD_NAME` for older firmware.
  Blank/erased tokens are reported as unavailable rather than `FFFF...`.
* Reads full firmware/build information through `getValue(VERSION_INFO)`,
  including the Nabu Casa special version digit (for example `9.1.2.1`).
* Non-blocking probe loop – heavy work runs from `loop()` and feeds the ESP32
  watchdog.
* Optional raw ASH logging (`raw_capture: true`) for debugging.

## Requirements

* ESPHome 2025.8.0 or newer (tested on 2025.8.3).
* UART wired to an EFR32 Zigbee module running EZSP 7.x/8.x.
* `stream_server` component (pause/resume support recommended).
* ESPHome build framework: ESP-IDF.

## Configuration

```yaml
external_components:
  - source: github://youruser/yourrepo@latest
    components: [efr32_info]

uart:
  id: uart_bus
  rx_pin: GPIO36
  tx_pin: GPIO4
  rts_pin: GPIO2
  cts_pin: GPIO13
  baud_rate: 115200
  hw_flow_control: CTS_RTS

stream_server:
  id: stream_srv
  uart_id: uart_bus

binary_sensor:
  - platform: template
    id: ncp_busy

text_sensor:
  - platform: template
    id: board_name
  - platform: template
    id: mfg_string
  - platform: template
    id: ezsp_info_status
  - platform: template
    id: ezsp_ieee
  - platform: template
    id: ezsp_firmware

efr32_info:
  id: zigbee_info
  uart_id: uart_bus
  stream_server_id: stream_srv
  busy_binary_sensor: ncp_busy
  status_text: ezsp_info_status
  board_name_text: board_name
  manufacturer_text: mfg_string
  ieee_text: ezsp_ieee
  firmware_text: ezsp_firmware
  raw_capture: false    # set true for verbose ASH dumps
```

### Options

| Option                | Type                         | Description                                                 |
|-----------------------|------------------------------|-------------------------------------------------------------|
| `uart_id`             | `uart::UARTComponent`        | Required UART connected to the EFR32.                       |
| `stream_server_id`    | `StreamServerComponent`      | Optional; will be paused/resumed automatically.             |
| `busy_binary_sensor`  | `binary_sensor::BinarySensor`| Published `true` while a probe is running.                  |
| `*_text` sensors      | `text_sensor::TextSensor`    | Optional publishers – pass any combination you care about.  |
| `status_text`         | `text_sensor::TextSensor`    | Status string (`probing`, `ash-error`, `ok`, …).             |
| `raw_capture`         | `bool` (default `false`)     | Emit RAW ASH frames at VERY_VERBOSE logging level.          |

## Usage

* Call `efr32_info.probe` from an automation, script, or button:

```yaml
button:
  - platform: template
    name: "Probe EZSP Info"
    on_press:
      - efr32_info.probe: zigbee_info
```

* The component sets `ncp_busy` high during the probe so you can gate other
  traffic or pause external clients.

## Troubleshooting

* **Probe runs at boot**: add a small delay and ensure the stream server is ready
  before calling `efr32_info.probe`.
* **ASH errors**: enable `raw_capture: true` and set `logger.level: VERY_VERBOSE`
  to capture frames for analysis.
* **No board name/manufacturer**: check the radio’s manufacturing tokens; older
  firmware may not populate them.

## Development Notes

The ASH helpers are shared with `efr32_flasher` via `ash_util.h`. If you modify
them, keep both components in sync. The component avoids `external_components`
dependencies beyond standard ESPHome modules; it’s safe to publish from a GitHub
repo for reuse.

## Identity/version compatibility

The ASH reset, bootstrap, ACK handling, protocol negotiation, stream-server
pause/resume, and existing network/security queries are unchanged. After
negotiation, a read-only custom-frame feature query checks for the manufacturing
identity override bit. Only a successful, correctly framed response enables
XNCP identity reads. Each unavailable/empty override falls back independently to
its physical manufacturing token; firmware without XNCP therefore still works.

`getMfgToken` returns a length-prefixed byte string without a status byte.
Empty, erased, whitespace-only, non-ASCII, and truncated responses do not count
as identity. Both manufacturer and board must be present for `tokens=ok`.
Missing fields are cleared so a previous module's identity cannot remain shown.

The full version query is EZSP `0x00AA` with value ID `0x11`. Its payload starts
with a little-endian build number, then major/minor/patch/special; the optional
version-type byte is ignored. Status decoding handles one-byte statuses before
EZSP 14 and four-byte statuses in EZSP 14+. If full version info is unavailable,
the existing VERSION(min) result remains as a fallback. Identity/value replies
are decoded only for the matching request because they do not echo the selector.

### Local verification

```sh
c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -I. \
  tests/test_efr32_info_protocol.cpp -o /tmp/test-efr32-info-protocol
/tmp/test-efr32-info-protocol
uvx --from esphome==2026.10.0 esphome compile tests/efr32_info_compile.yaml
```

These check captured/constructed EZSP replies and compile the ESP32 component.
They do not replace testing a probe against physical NCP firmware. The YAML is
a compile-only fixture with placeholder Wi-Fi settings, not a deployment image.
