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
* Reads manufacturing tokens such as `MFG_BOARD_NAME` and `MFG_STRING` – used by
  `efr32_flasher` to pick the correct firmware variant automatically.
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
