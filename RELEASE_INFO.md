# Release and provenance information

## Source units

| Directory | License | Upstream | Baseline |
|---|---|---|---|
| `FAST_LIO` | GPL-2.0-only | <https://github.com/Ericsii/FAST_LIO.git> | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| `FAST_LIO_LOCALIZATION2` | GPL-2.0-only | <https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2.git> | `f04974907c8da976dd0495b272d18ac4c534d41f` |

Each program embeds a complete `ikd-Tree` source snapshot. Its GPLv2 text is
retained beside the source. Component `NOTICE` files record the corresponding
`ikd-Tree` baselines and the 2026 local modification scope.

## Local derivative scope

The source includes the ROS 2 Humble/aarch64 and RoboSense RSFAIRY adaptations
used by the navigation system, including mapping/map-save, gravity alignment,
initial-pose localization, transform fusion, base-yaw handling, and shutdown
safety changes. The v1.1.0 mapping update separates full-range export from
near-field EKF/ikd-tree registration, creates a separately bounded PCT source
PCD, and commits both PCD outputs transactionally. Refer to the component
notices for the authoritative summary.

## Source integrity

- The repository retains both complete derivative source trees and their
  embedded `ikd-Tree` sources.
- Baseline commits and local modification scopes are recorded in the component
  notices.
- Maps, logs, credentials, endpoint values, generated objects, and binaries are
  not corresponding source and are not included.

`SOURCE_INVENTORY.sha256` locks every distributed file except itself. Run
`./audit_source.sh` after cloning to verify the inventory, license declarations,
embedded source, privacy exclusions, path lengths, and generated-artifact ban.

## Redistribution

Keep the complete corresponding source, GPLv2 texts, copyright notices,
attributions, modification notices, and build files with any distributed
binaries. Do not describe this repository as permissively licensed.
