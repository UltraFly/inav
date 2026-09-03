# INAV Gyro Assist EdgeTX tool

`inav-gyro-assist` is the independent EdgeTX setup tool for INAV Gyro Assist. It is generic INAV functionality: it does not require a NEXUS receiver, an Avian ESC, TextGen, or Spektrum equipment.

Gyro Assist is an AUX modifier for fixed-wing MANUAL mode. Stick position remains a direct surface command, with bounded filtered gyro damping added to oppose disturbances. It is not a primary mode, an angular-rate target, or attitude hold. Unmodified MANUAL behavior must remain unchanged when the modifier is off.

The feature and its radio configuration are the INAV-native functional equivalent of receiver stabilization configured through Spektrum Forward Programming, but this tool neither implements that protocol nor configures Spektrum receivers.

## Requirements and installation

Use stable **EdgeTX 2.11.0 or newer** with Lua enabled. The full launcher and directory names rely on the long tool-filename fix included in EdgeTX 2.11.0. Release candidates are not part of the minimum-version requirement.

The distributable SD-card tree is:

```text
src/utils/edgetx/inav-gyro-assist/
└── SCRIPTS/TOOLS/
    ├── inav-gyro-assist.lua
    └── inav-gyro-assist/
        ├── main.lua
        └── msp.lua
```

Copy the launcher and matching directory to the radio without shortening or renaming them:

```text
/SCRIPTS/TOOLS/inav-gyro-assist.lua
/SCRIPTS/TOOLS/inav-gyro-assist/main.lua
/SCRIPTS/TOOLS/inav-gyro-assist/msp.lua
```

Restart the radio and open **INAV Gyro Assist 0.3** from the Tools page. Version 0.3 targets a 480 × 272 color display. A compact monochrome layout is not yet implemented.

## Version 0.3 interface

The home page exposes two views:

1. **Status and safety** shows INAV link state, armed state, whether all settings were discovered, and the fixed-wing MANUAL eligibility rule.
2. **Gyro Assist settings** discovers and edits all 17 `gyro_assist_*` settings. It presents per-axis gain, stick Priority, Stop Release, Stop Lock, and correction limits plus the shared low-pass and transition settings.

Priority reduces effective rate-damping gain as the corresponding stick moves away from center. Stop Release controls how quickly that gain falls, and Stop Lock controls how quickly it returns after recentering. They do not provide heading hold or self-leveling; those behaviors remain outside Gyro Assist and are available through other INAV flight modes.

The tool communicates through EdgeTX's CRSF telemetry API and the standard MSPv2 framing used by INAV. It does not add a feature-specific settings protocol.

## MSP interface

The settings workflow uses existing generic commands:

| Operation | Command |
| --- | --- |
| Read setting metadata, range, type, and current value | `MSP2_COMMON_SETTING_INFO` (`0x1007`) |
| Read back a setting | `MSP2_COMMON_SETTING` (`0x1003`) |
| Change a setting in RAM | `MSP2_COMMON_SET_SETTING` (`0x1004`) |
| Persist verified changes | `MSP_EEPROM_WRITE` (`0x00FA`) |
| Confirm disarmed state before writing and again before save | `MSP2_INAV_STATUS` (`0x2000`) |

Apply is deliberately transactional at the application level:

1. Refuse to start unless the FC is known to be disarmed.
2. Re-read the armed state.
3. Send each changed settings set (the current implementation sends the complete 17-setting snapshot).
4. Read every setting back and compare it with the requested value.
5. Re-read the armed state.
6. Issue EEPROM write only if every write and readback succeeded and the FC is still disarmed.

If a request is rejected, times out, reads back differently, or the FC becomes armed, the tool reports that partial RAM changes were **not saved**. Reconnect or reboot and reload the settings before trying again. INAV remains the authority for setting ranges and controller safety.

The AUX assignment deliberately remains in INAV Configurator's **Modes** tab. Assign `GYRO ASSIST` to a switch there; the Lua tool only configures the controller settings.

## Controls

On the home page, turn the roller to select a view, press `ENTER` to open it, and press `EXIT` to close the tool. On a view, `EXIT` returns home. Touch selection is supported on compatible radios.

In the settings view, turn the roller to select a row, press `ENTER` to enter or leave editing, and turn the roller to change the selected value. Select **Apply and save** to run the disarmed write/readback/save sequence. **Revert** discards local edits and restores the last values read from INAV.

## Safety and scope

- Gyro Assist is eligible only for fixed-wing MANUAL mode and is off by default.
- It adds bounded correction before the existing mixer and servo limits; it does not replace those limits.
- Stick Priority and Stop Release/Lock shape effective damping gain only. They do not change the configured correction limit or add attitude-hold behavior.
- Leaving MANUAL, disarm, failsafe, launch, navigation, autotrim, and stale or invalid gyro data follow firmware-defined handoff or immediate-inhibit rules.
- Setting ranges displayed by the radio come from INAV metadata. Firmware-side controller validation remains the safety authority.
- Initial hardware tests require propeller removal, verified correction polarity, low gains and limits, and an immediate return to unmodified MANUAL.

## Host checks

Run the shared Lua 5.2 transport suite against the Gyro Assist copy and run its application smoke test:

```sh
lua5.2 src/utils/edgetx/tests/msp_test.lua \
    src/utils/edgetx/inav-gyro-assist/SCRIPTS/TOOLS/inav-gyro-assist/msp.lua

lua5.2 src/utils/edgetx/inav-gyro-assist/tests/main_test.lua \
    src/utils/edgetx/inav-gyro-assist/SCRIPTS/TOOLS/inav-gyro-assist/main.lua \
    src/utils/edgetx/inav-gyro-assist/SCRIPTS/TOOLS/inav-gyro-assist/msp.lua
```

The application test checks status and generic setting discovery, disarmed-only apply, setting writes, readback, EEPROM save ordering, and absence of Avian coupling.

## Provenance

- [Spektrum AS3X+ Priority and Stop Lock/Release behavior](https://wiki.spektrumrc.com/spektrum/as3x-setup-guide)

- [EdgeTX long tool-filename fix](https://github.com/EdgeTX/edgetx/pull/6027)
- [EdgeTX 2.11.0 stable release](https://github.com/EdgeTX/edgetx/releases/tag/v2.11.0)
- [EdgeTX Crossfire Lua telemetry API](https://github.com/EdgeTX/edgetx/blob/main/radio/src/lua/api_general.cpp)
- [INAV MSP-over-telemetry framing](https://github.com/iNavFlight/inav/blob/master/src/main/telemetry/msp_shared.c)
- [Gyro Assist controller design](GYRO_ASSIST.md)
