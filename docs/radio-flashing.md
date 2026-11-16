# Application baud and software bootloader entry

CC2652 and EFR32 manifests may declare an integer `baudrate` on the selected
image, or at the root as a default. A missing field preserves the application
rate. Values must be 9600–921600 and, when `runtime_baud_select` is configured,
one of its options. Invalid types/rates reject the manifest before programming.

After a successful transfer the updater applies the target baud to the UART
hardware and synchronously saves a numeric preference before serial ownership
is released. Failed transfers retain the old rate. If saving fails after a
successful transfer, the new rate remains active for the installed image but
the operation reports failure. Numeric preferences are restored during setup,
seeded from legacy selector state on migration. Checks do not change/save baud.
Direct EFR32 `.gbl` URLs have no manifest metadata and retain the current rate.

For a bound template select, suppress its UART/pause/resume set_action while
`flasher.is_applying_manifest_baud()` is true. This updates the saved entity state
without resuming the bridge while native flashing owns it. Manual baud scripts
must also call `set_runtime_baud_rate()` so subsequent flash restores and numeric
preferences follow the user's choice. The builder's configurations implement
these ownership rules; the complete builder ships the same native source.

EFR32 `bootloader_entry` modes:

- `gpio` (default) requires `boot_switch` and retains the existing BL/reset path.
- `ezsp` omits `boot_switch` and uses the running NCP's ASH/EZSP command path.

The native `efr32_flasher.enter_bootloader` action is available for manual UI
buttons. Its caller keeps the bridge paused for an external flasher and may
inspect `bootloader_pending()` / `bootloader_succeeded()`. A hardware reset plus
`restore_runtime_baud()` exits manual mode and restores application baud.

EZSP entry discovers/negotiates protocol v8+, checks for a standalone bootloader,
sends frame 0x008F with normal mode 1 at application baud, then requires the
actual Gecko bootloader menu at bootloader baud. It can also recognize a
bootloader entered with a physical button. No transfer starts if entry fails.
The transport validates CRCs and sequence IDs, handles duplicates and bounded
NAK retries, ignores out-of-band XON/XOFF, and bounds receive memory/timeouts.
Unresponsive/non-EZSP applications require physical recovery.

`hardware_flow_control: false` preserves older boards' configured two-wire UART
pins. The default retains MG24 RTS/CTS. `variant: flat` requires a model-specific
flat manifest and rejects variant catalogs, avoiding an MG24-image fallback on
older silicon. Bootloader baud remains separate from application `baudrate`.

CC2652 now reloads actual UART settings when entering/restoring bootloader baud.
Its final success log follows UART restoration and persistence; the earlier
transport message is not a final success result. Pausing the bridge through
flash and post-flash identity reads is still the configuration's responsibility.

When restricting `external_components.components`, include
`zigbee_firmware_baud` alongside the flasher so ESPHome can load its shared
headers. EFR32 builds also configure `efr32_info` for the ASH utility dependency
and identity reader, as the builder profiles do.

## Validation

Run `ARDUINOJSON_INCLUDE=/path/to/ArduinoJson/src python -m pytest -q tests`.
CI supplies pinned ArduinoJson 7.4.3. Tests exercise numeric preferences/selector
migration, malformed JSON metadata, real ASH wire encoding and CRCs, negotiation,
launch rejection/timeouts, duplicate frames, NAK retries, oversized frames and
software flow control. Both CC2652 class declarations are checked for equal
member layout. Integration firmware builds and UI tests live in the paired
TubesZB-ESPHome-Builder change; BM24 forward/reverse baud transitions have been
hardware tested, while the new MGM21 software entry awaits physical testing.
