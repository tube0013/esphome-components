# CC2652 Flasher (ESPHome external component)

Flash Texas Instruments CC2652 (P2/P7) Zigbee radios over UART using the ROM bootloader. Automatically detects the module variant (P2 vs P7), selects the right firmware from a JSON manifest, and provides buttons/actions + text sensors for visibility.

## Features
- Auto‑detect variant
  - Order: YAML `variant` override → ROM BSL probe → ZNP SYS_VERSION (if definitive)
- Manifest with variants (P2/P7) and fallbacks
- HEX and BIN streaming (Intel HEX parsed on‑device)
- Erase modes: `sector`, `bank` (default), `auto`
- Actions: update, erase, check update, detect variant
- Text sensors: current FW code revision, IEEE address, latest manifest version, detected variant

## Requirements
- ESPHome 2025.8.3
- ESP32 (ESP‑IDF) with a UART connected to the CC2652 and two GPIOs:
  - Reset line
  - BSL/BOOT line (GPIO that pulls the CC2652 boot pin)

## Install (external_components)
```yaml
external_components:
  # optional, for UART streaming/debug
  - source: github://oxan/esphome-stream-server
  # main component
  - source: github://tube0013/esphome-components
    components: [cc2652_flasher, zigbee_firmware_baud]
```

## YAML example
```yaml
esp32:
  board: esp32-poe
  framework:
    type: esp-idf   # This component is ESP-IDF only

uart:
  id: uart_bus
  rx_pin: GPIO36
  tx_pin: GPIO4
  baud_rate: 115200
  rx_buffer_size: 1024

switch:
  - platform: gpio
    id: zRST_gpio
    pin: 5
    inverted: true
    restore_mode: ALWAYS_OFF
  - platform: gpio
    id: zBSL
    pin: 16
    inverted: true
    restore_mode: ALWAYS_OFF

# Optional debug stream (remove if not needed)
stream_server:
  uart_id: uart_bus
  port: 6638

cc2652_flasher:
  id: zigbee_flasher
  uart_id: uart_bus
  url: https://example.com/path/to/manifest.json
  bsl_switch: zBSL
  rst_switch: zRST_gpio
  boot_baud_rate: 500000
  restore_baud_rate: 115200
  erase_mode: bank  # default; sector | bank | auto
  debug: false
  show_progress: true
  progress_step: 5
  fw_version_text: zb_fw_version
  ieee_text: zb_ieee
  latest_fw_version_text: zb_latest_fw
  variant_text: zb_variant
  check_on_boot: true
  check_interval_days: 7
  detect_on_boot: true
  detect_on_boot_delay_ms: 0
  # auto (default), p2 or p7; case-insensitive (P2/P7 also work).
  # Set the known hardware variant to skip the ROM detection probe.
  variant: auto

select:
  - platform: template
    name: CC2652 Variant Override
    id: cc2652_variant_override
    optimistic: true
    restore_value: true
    initial_option: "Auto"
    options: ["Auto", "P2", "P7"]
    # Restore the override before the flasher's boot detection (priority 800).
    setup_priority: 810
    on_value:
      then:
        - lambda: |-
            id(zigbee_flasher).set_variant(i == 1 ? 2 : i == 2 ? 7 : 0);

button:
  - platform: template
    name: Flash CC2652
    on_press:
      - cc2652_flasher.update_firmware: { id: zigbee_flasher }
  - platform: template
    name: Check CC2652 Update
    on_press:
      - cc2652_flasher.check_update: { id: zigbee_flasher }
  - platform: template
    name: Detect CC2652 Variant
    on_press:
      - cc2652_flasher.detect_variant: { id: zigbee_flasher }

text_sensor:
  - platform: template
    id: zb_fw_version
    name: Zigbee FW Version
  - platform: template
    id: zb_ieee
    name: Zigbee IEEE
  - platform: template
    id: zb_latest_fw
    name: Zigbee Latest FW
  - platform: template
    id: zb_variant
    name: Zigbee Variant
```

## Web UI variant override
With `web_server` enabled, the example's **CC2652 Variant Override** dropdown
offers **Auto** (the default), **P2**, and **P7**. If detection reports
`BSL probe: autobaud failed`, select the known hardware variant, then press
**Check CC2652 Update** or **Flash CC2652**. Changing the dropdown updates the
variant sensor and firmware selection immediately; it does not start flashing.
ESPHome saves the selection using its normal preferences mechanism and restores
it before boot detection, so a saved P2/P7 override also skips the boot probe.
Select **Auto** to clear the override and use ROM/ZNP detection again.

The TubesZB frontend exposes the same saved selector on **Firmware → Zigbee radio firmware**
when `tubeszb_web.cc2652.variant_select` is bound to `cc2652_variant_override`.
Changing either frontend updates the other and requires a fresh firmware check.

The optional YAML `variant` setting remains case-insensitive. When using the
dropdown, leave `variant: auto`: the dropdown's initial or restored selection
controls the runtime override. Keep the option order and `setup_priority` shown
above so saved values restore correctly before detection.

## Actions
- `cc2652_flasher.update_firmware` – Fetches manifest, selects variant, erases as needed, programs image, verifies MD5, resets target.
- `cc2652_flasher.erase_flash` – Erase only. Optional mode: `sector|bank|auto`.
- `cc2652_flasher.check_update` – Fetches manifest and publishes the latest version to `latest_fw_version_text`.
- `cc2652_flasher.detect_variant` – Publishes the configured override, or briefly enters ROM BSL to read chip IDs when `variant: auto`.

## Manifest formats
Preferred (object):
```json
{
  "version": "20250321",
  "variants": {
    "p2": { "fw_url": "https://…/cc2652p2.hex", "md5": "…", "size": 507877 },
    "p7": { "fw_url": "https://…/cc2652p7.hex", "md5": "…", "size": 508798 }
  }
}
```

Preferred (array):
```json
{
  "version": "20250321",
  "variants": [
    { "match": { "variant": "p2" }, "fw_url": "https://…/cc2652p2.hex" },
    { "match": { "variant": "p7" }, "fw_url": "https://…/cc2652p7.hex" }
  ]
}
```

Legacy supported:
- `fw_url_p2` / `fw_url_p7`
- Single `fw_url` / `firmware_url` / `url`
- MD5 fields: `md5`, `md5sum`, `md5_hex`
- Size fields can be number or string (`size` / `length`)
- Version fields: `version`, `code_revision`, or `rev`

## Notes
- Variant detection uses ROM BSL registers to differentiate P2 vs P7 when ZNP cannot.
- If probing fails, set `variant: p2` or `variant: p7` to select the known hardware variant. Values are case-insensitive; `P2`, `P7`, `CC2652P2` and `CC2652P7` are also accepted. Omit the option or use `auto` to retain automatic detection.
- A manual variant skips the ROM detection probe at boot and when the detect action is called. It takes priority over ZNP/ROM results for manifest selection and the variant text sensor, and is published at boot even with `detect_on_boot: false`.
- The override selects the firmware variant; flashing still requires a working UART/ROM bootloader connection.
- For local manifests, serve the JSON over HTTP and use its URL; the ESP cannot read files from your workstation.
- Links `ArduinoJson` and `MD5` automatically via AUTO_LOAD (no manual libs needed).
- Requires ESP-IDF framework for ESP32 (Arduino framework is not supported).

## License
This component is licensed under the GNU General Public License v3.0 (GPLv3).

- Portions inspired by work in the XZG project (Arduino CC2652 flashing utilities).
- You should have received a copy of the GNU General Public License
  along with this program. If not, see https://www.gnu.org/licenses/
