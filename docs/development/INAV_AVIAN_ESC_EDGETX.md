# INAV Avian ESC EdgeTX tool

`inav-avian-esc` is the independent EdgeTX tool for Spektrum Avian ESC telemetry and TextGen programming. It has no dependency on Gyro Assist or a NEXUS receiver, so it remains useful with MANUAL, conventional stabilized modes, and autonomous/navigation configurations.

TextGen is the only Avian ESC-programming mechanism in project scope. Spektrum Forward Programming configures compatible receivers and is not implemented by this tool.

## Requirements and installation

Use stable **EdgeTX 2.11.0 or newer** with Lua enabled. The full launcher and directory names rely on the long tool-filename fix included in EdgeTX 2.11.0. Release candidates are not part of the minimum-version requirement.

The distributable SD-card tree is:

```text
src/utils/edgetx/inav-avian-esc/
└── SCRIPTS/TOOLS/
    ├── inav-avian-esc.lua
    └── inav-avian-esc/
        ├── main.lua
        └── msp.lua
```

Copy the launcher and matching directory to the radio without shortening or renaming them:

```text
/SCRIPTS/TOOLS/inav-avian-esc.lua
/SCRIPTS/TOOLS/inav-avian-esc/main.lua
/SCRIPTS/TOOLS/inav-avian-esc/msp.lua
```

Restart the radio and open **INAV Avian ESC 0.3** from the Tools page. Version 0.3 targets a 480 × 272 color display. A compact monochrome layout is not yet implemented.

## Version 0.3 capabilities

Implemented:

- Independent live-status, Avian telemetry, and Avian TextGen pages.
- Automatic discovery of standard EdgeTX and Spektrum Avian telemetry sensor names.
- Raw TextGen reception from a 4-in-1 Multi-Module through EdgeTX `multiBuffer()`.
- Bounded TextGen rows `0`–`8`, refresh row `254`, and clear row `255`.
- A two-step TextGen safety prompt that explicitly requires throttle cut, disarm, and the thrust-reverse switch in its normal position, plus automatic closure if the throttle stick rises.
- A simulator with representative ESC and TextGen data.
- A native MSPv2-over-CRSF client with request fragmentation, response reassembly, sequence checks, backpressure, size limits, and timeouts.
- Read-only `MSP2_INAV_STATUS` polling to show the FC link and armed state.
- TextGen entry rejection when INAV reports armed. If status is unavailable, the UI says so rather than assuming disarmed.

Not yet implemented:

- INAV MSP messages for the direct FC-to-ESC link, ESC telemetry, or TextGen sessions.
- Lua-generated TextGen navigation actions.
- Any ESC-setting write outside the ESC's TextGen menu.

Unavailable operations remain read-only or visibly pending.

## Controls

On the home page, turn the roller to select a page, press `ENTER` to open it, and press `EXIT` to close the tool. On a feature page, `EXIT` returns home. Touch selection is supported on compatible radios.

## Avian telemetry

The tool searches for the following EdgeTX telemetry names and uses the first available alias:

| Display value | Preferred name | Fallback names |
| --- | --- | --- |
| Electrical RPM | `Erpm` | `EscR`, `RPM` |
| Input voltage | `EVIN` | `EscV`, `VFAS`, `RxBt` |
| Motor current | `ECUR` | `EscA`, `Curr` |
| FET temperature | `TFET` | `EscT`, `Tmp1` |
| BEC temperature | `TBEC` | — |
| BEC current | `CBEC` | `BecA` |
| BEC voltage | `VBEC` | `BecV` |
| ESC throttle | `ETHR` | `Thr` |
| ESC output | `EOUT` | — |

EdgeTX already converts these sensor `0x20` values to display units; the tool does not decode or rescale them again. A positive voltage update is the prototype freshness marker until the INAV adapter exposes explicit validity and timestamps.

## TextGen over a 4-in-1 module

For sensor `0x0C`, EdgeTX's Spektrum raw bridge uses the `STR` marker in bytes `0`–`2`, the requested sensor ID in byte `3`, and byte `4` as a producer/consumer semaphore. Once byte `4` reports `0x0C`, the tool reads the instance, line number, and up to 13 text characters, then releases the frame.

The tool clears raw-buffer ownership on normal TextGen exit and tool exit. Capture starts only after two `ENTER` presses with the physical throttle stick fully low. Version 0.3 displays the menu but does not synthesize aileron, elevator, or throttle: with a Smart receiver and 4-in-1 module, the pilot continues to use the physical sticks as directed by the ESC.

Thrust reverse is configured in this same ESC menu. The captured Avian Lite 85A screen shows `THRUST REV = CH9`; the intended direct-INAV path forwards the configured receiver channels with their numbering intact so the ESC can observe the selected AUX channel. The ESC owns the selection and no duplicate INAV channel selector is required. The selected radio channel needs a verified non-reverse failsafe value, and the reverse switch must be normal before TextGen entry.

## ExpressLRS / CRSF plan

The NEXUS-XR internal ExpressLRS path cannot use `multiBuffer()`. The same TextGen display model, ESC state, and abstract navigation actions will therefore travel through MSP over CRSF. The radio sends abstract up/down/select/back actions; INAV owns bounded SRXL2 channel generation, zero-throttle enforcement, timeouts, and deterministic restoration of ordinary forwarding.

Project-specific MSP command IDs must be assigned and documented in INAV before this tool encodes them.

## Safety

- Remove the propeller before programming or initial bench testing.
- Enable throttle cut, lower the stick fully, confirm disarm, and put the thrust-reverse switch in its normal/non-reverse position before opening TextGen.
- The current tool verifies physical stick position and uses INAV's existing MSP armed status when available; it cannot yet verify the motor output, ESC-link guard state, or failsafe state.
- A power cycle may be required after saving ESC settings.
- Treat the display as information, never as authorization to arm.

## Host checks

Run the Lua 5.2 transport suite against both independent copies and the Avian application smoke test:

```sh
lua5.2 src/utils/edgetx/tests/msp_test.lua \
    src/utils/edgetx/inav-avian-esc/SCRIPTS/TOOLS/inav-avian-esc/msp.lua

lua5.2 src/utils/edgetx/inav-avian-esc/tests/main_test.lua \
    src/utils/edgetx/inav-avian-esc/SCRIPTS/TOOLS/inav-avian-esc/main.lua \
    src/utils/edgetx/inav-avian-esc/SCRIPTS/TOOLS/inav-avian-esc/msp.lua
```

The application test checks MSP status framing, armed-state rendering, and TextGen entry blocking while armed.

## Provenance

- [EdgeTX Spektrum telemetry implementation](https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/spektrum.cpp)
- [EdgeTX long tool-filename fix](https://github.com/EdgeTX/edgetx/pull/6027)
- [EdgeTX 2.11.0 stable release](https://github.com/EdgeTX/edgetx/releases/tag/v2.11.0)
- [EdgeTX Crossfire Lua telemetry API](https://github.com/EdgeTX/edgetx/blob/main/radio/src/lua/api_general.cpp)
- [INAV MSP-over-telemetry framing](https://github.com/iNavFlight/inav/blob/master/src/main/telemetry/msp_shared.c)
- [DSMTools](https://github.com/frankiearzu/DSMTools)
- [Spektrum Avian TextGen instructions](https://www.spektrumrc.com/on/demandware.static/-/Sites-horizon-master/default/dw71772f5d/Manuals/SPMXAE1160HV-SPMXAE1200HV-Manual-EN.pdf)

No DSMTools source was copied into this implementation.
