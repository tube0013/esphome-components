# rcp_info

Detects **OpenThread RCP firmware** on a Zigbee/Thread radio module by speaking
Spinel (over HDLC-lite framing) directly on the UART — the on-device equivalent
of the `rcp_check.py` QC script. It round-trips `PROP_NCP_VERSION` and
`PROP_HWADDR`, publishing the firmware version string and EUI-64.

The intended use is as a **fallback probe**: when a Zigbee firmware probe
(`efr32_info` EZSP, `cc2652_flasher` ZNP) comes up empty, a YAML automation
triggers `rcp_info.probe` to check whether the module is actually running RCP
firmware. This keeps a single ESPHome firmware usable for modules flashed with
either Zigbee or Thread RCP images. The component is fully standalone — it does
not modify or depend on the other components.

## How it works

- Probes each configured baud rate in order (default `921600, 460800, 115200`),
  temporarily switching the UART rate for each attempt.
- The **original UART baud rate is always restored** afterward. The detected
  rate is reported via the status text (`rcp@921600`) and the log — set your
  baud-rate select entity accordingly if you want to use the RCP afterward.
- If a `stream_server_id` is configured, the serial stream is paused for the
  duration of the probe and resumed afterward (an active stream competes for
  UART bytes and breaks the probe).
- This is a passive query: no RCP reset is performed.
- On success it publishes the version string to `version_text`, the EUI-64
  (colon-hex) to `ieee_text`, and `rcp@<baud>` to `status_text`. On failure it
  publishes only `not-found` to `status_text` — version/IEEE sensors are left
  untouched so values from a Zigbee probe are never clobbered.
- Worst-case duration is roughly `baud_rates × per_baud_timeout` (~6.5 s with
  defaults); a successful first-baud probe takes 2–4 s. The probe blocks the
  main loop while running (watchdog is fed).

## Configuration

```yaml
rcp_info:
  id: rcp_probe
  uart_id: uart_bus
  stream_server_id: stream_srv       # optional; pause stream during probe
  status_text: probe_status          # optional text_sensor IDs
  version_text: fw_version
  ieee_text: fw_ieee
  baud_rates: [921600, 460800, 115200]  # default
  per_baud_timeout: 2s                  # default
```

Trigger manually from anywhere (e.g. a button):

```yaml
button:
  - platform: template
    name: "Probe RCP"
    on_press:
      - rcp_info.probe: rcp_probe
```

## Fallback pattern: EFR32 (efr32_info)

`efr32_info` publishes `ash-sync-fail`, `v4-fail`, or `no-response` to its
status text sensor when the EZSP probe fails completely. Point `rcp_info` at
the **same** text sensors the EZSP probe already uses and trigger on those
values. The `rcp_info.probe` action only sets a flag, so the RCP probe runs on
a later loop pass — after `efr32_info` has finished and resumed the stream.

```yaml
rcp_info:
  id: rcp_probe
  uart_id: uart_bus
  stream_server_id: stream_srv
  status_text: ezsp_info_status
  version_text: ezsp_firmware
  ieee_text: ezsp_ieee

text_sensor:
  - platform: template
    name: "Zigbee Probe Status"
    id: ezsp_info_status
    # ...existing sensor config...
    on_value:
      - if:
          condition:
            lambda: 'return x == "ash-sync-fail" || x == "v4-fail" || x == "no-response";'
          then:
            - rcp_info.probe: rcp_probe
```

No re-trigger loop is possible: after the RCP probe the status becomes
`rcp@<baud>` or `not-found`, neither of which matches the failure condition.

## Fallback pattern: CC2652 (cc2652_flasher)

The ZNP probe publishes nothing on failure — its sensors simply stay empty. So
check for an empty firmware sensor a little while after boot (the flasher's
second ZNP attempt fires at 4.5 s):

```yaml
rcp_info:
  id: rcp_probe
  uart_id: uart_bus
  stream_server_id: stream_srv
  version_text: zb_fw_version
  ieee_text: zb_ieee

esphome:
  on_boot:
    priority: -100
    then:
      - delay: 15s
      - if:
          condition:
            lambda: 'return !id(zb_fw_version).has_state() || id(zb_fw_version).state.empty();'
          then:
            - rcp_info.probe: rcp_probe
```

## Notes

- The UART baud rate must be switchable at runtime; this uses the same
  `set_baud_rate()` + `load_settings()` mechanism as the flasher components
  (works on ESP-IDF and Arduino ESP32).
- A future opt-in `keep_detected_baud` option could leave the UART at the
  detected RCP rate instead of restoring — not implemented; the baud select
  entity in the device YAML owns the rate today.
- Cross-check results from a host with the builder repo's
  `scripts/rcp_check.py --host <device>` (remember to set the device baud
  select to the detected rate first).
