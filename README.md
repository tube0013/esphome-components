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
