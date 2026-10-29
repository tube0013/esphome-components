# TubesZB ESPHome external components

Device configurations in TubesZB-ESPHome-Builder import these components from
GitHub. Component source, embedded frontend assets and native handlers live here.

- `network_settings`: persistent DHCP/static IPv4 trials and offline policy.
- `tubeszb_web`: branded management UI, logs, OTA/radio progress and radio controls.
- `cc2652_flash_guard`: native-result tracking for bridge ownership during flashing.
- `cc2652_flasher`, `efr32_flasher`, `zigbee_firmware_baud`: native flashing,
  persisted manifest application baud and EFR32 GPIO/EZSP bootloader entry.

Existing `stream_server` and `efr32_info` provide serial transport and identity.
See individual component READMEs and [radio behavior](docs/radio-flashing.md).
The paired builder PR runs its integration, browser and firmware checks against
this repository; its source reference temporarily targets this PR's head.

Minimum ESPHome version: **2026.10.1** (the frontend uses the public update source URL API).

Component host tests live here, including network transactions, IPv4 JavaScript/C++
parity, transport policy, web assets, baud persistence, and native radio protocols.
Run `python -m pytest -q tests` with ESPHome 2026.10.1, pytest, Node.js and
`ARDUINOJSON_INCLUDE` pointing at ArduinoJson 7.4.3's `src`. The Builder keeps
hardware configuration and browser integration tests; it runs this same suite
from its external checkout instead of maintaining duplicate native fixtures.

The Origin and per-boot UI nonce checks protect against accidental cross-site
mutations; they do not authenticate LAN clients or prevent DNS rebinding.
Configure ESPHome `web_server.auth` when access control is required. The shared
server applies that authentication to both management interfaces.

Restart is available while checks or unstarted firmware actions are pending.
Native OTA/radio flash events block restart during real firmware operations;
the reboot grace period rechecks those events before restarting.
