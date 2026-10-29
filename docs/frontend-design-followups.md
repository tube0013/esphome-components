# Radio API follow-up

The owner chose to keep the larger radio refactor out of the current draft PRs.
Group the following changes so they share one native operation contract:

- Extend typed status beyond flashing to CC2652 check/probe and EFR32 non-flash failures.
- Introduce small CC2652, EFR32 and serial-only adapters behind a common radio interface.
- Route frontend and legacy template-button actions through a common native controller.
  Preserve internet/network guards, prior manual pause state, bootloader sequencing,
  application baud persistence, and post-flash identity-readback ownership before
  removing YAML button/sensor bindings. Calling start_firmware_update directly
  without that orchestration would undo the tested UART ownership behavior.
- Move the remaining simple flasher Action forwarding classes to register_apply_action
  as part of those native controller/API changes.

## Completed in the cleanup pass

Periodic frontend snapshots contain only numeric state (enforced at compile time).
Callbacks seed and refresh sensor strings; source keys rebuild only on source changes.
Required invariant dependencies are constructor arguments. Network policy automations
use build_callback_automation and templated callback registration. Model names are
explicit in all 22 Builder profiles. HTTP nonce, Origin policy and JSON responses
share one helper. Saved network phase values and NVS record layout are unchanged.

## ESPHome upgrade gate

GuardedHttp is coupled to the ESPHome HTTP transport's protected perform override
and HttpRequestUpdate parent replacement API. The host suite checks the installed
ESPHome header contract in addition to the offline-policy transport tests.
The Component host tests workflow has an esphome_version manual input for upgrade
candidates; its default remains 2026.10.1. Before moving the Builder release pins,
also run Builder firmware compilation and offline-mode hardware acceptance against
that candidate. A host header check alone cannot establish runtime compatibility.
