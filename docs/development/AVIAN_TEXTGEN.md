# Avian ESC TextGen design

Avian TextGen programming is a separate feature from ordinary ESC telemetry. Both use the upstream `motor_srxl2` driver merged in INAV PR #11947, but TextGen has its own display model, safety state, programming-only channel overrides, radio interface, and test plan. The TextGen branch is stacked directly on `feature/avian-esc-telemetry`; Gyro Assist remains independent of both Avian branches.

Within this project, TextGen is the only mechanism used to program an Avian ESC. Spektrum Forward Programming is a separate receiver-configuration protocol; implementing that protocol is out of scope. INAV Gyro Assist and its MSP configuration provide the project's INAV-native functional equivalent of Forward-Programming-configured receiver stabilization, but remain completely separate from ESC TextGen. The corresponding radio tool is `inav-avian-esc`; it does not require Gyro Assist.

This first firmware-side increment is hardware-independent. It decodes the 16-byte Spektrum TextGen sensor payload and models a bounded programming session; it does not yet connect the model to an SRXL2 UART, MSP, or an EdgeTX write interface.

## Payload model

Spektrum sensor `0x0C` is a fixed 16-byte payload:

| Offset | Meaning |
| --- | --- |
| 0 | Sensor ID `0x0C` |
| 1 | Instance/secondary ID |
| 2 | Line number |
| 3-15 | Thirteen text bytes, NUL-terminated when shorter than 13 characters |

Lines `0` through `8` form the display. Line `0` is the title and lines `1` through `8` are the menu body. Line `254` requests a refresh/backlight action and line `255` clears all display text.

`escTextGenDisplay_t` stores exactly nine lines of at most 13 printable ASCII characters, a valid-line mask, one selected instance, a revision counter, and a pending-refresh flag. The decoder rejects wrong lengths, unknown line numbers, control characters, and data from a second instance. A NUL terminator ends the visible string; unused bytes after it are ignored. Full-width 13-character lines remain NUL-terminated in INAV memory.

The TextGen decoder consumes only the sensor payload. The upstream driver already validates the SRXL2 envelope and CRC before `srxl2HandleTelemetry()` dispatches sensor `0x20`. The later integration will add sensor `0x0C` dispatch at that validated boundary. Invalid envelopes must never reach the TextGen decoder or refresh the session timeout. Live UART routing is still pending.

## Programming-session safety

A session can start only when all five entry conditions are true:

- the aircraft is disarmed;
- input throttle is confirmed low;
- the physical thrust-reverse switch is confirmed in its normal/non-reverse position;
- failsafe is inactive;
- the ESC link is available.

While active, the session produces a channel override with throttle fixed at 1000 microseconds and aileron/elevator centered at 1500 microseconds. A deliberate navigation request produces one bounded 350 ms pulse, matching the first real-ESC result from `smart-esc-tool`:

| Action | Aileron | Elevator |
| --- | ---: | ---: |
| Up | 1500 | 2000 |
| Down | 1500 | 1000 |
| Left/back | 1000 | 1500 |
| Right/select | 2000 | 1500 |

The throttle override remains at 1000 microseconds during every navigation pulse. A second request is rejected until the current pulse expires, after which the output automatically returns to neutral even if the caller has not mutated session state.

The session immediately releases all channel overrides and records a reason if the aircraft arms, throttle rises, the thrust-reverse switch leaves its normal position, failsafe begins, the link is lost, malformed TextGen traffic is reported, the user exits, or no valid TextGen activity arrives for 30 seconds. The longer inactivity bound accommodates the measured two-step menu-entry sequence, which holds each requested stick position for about eight seconds. Valid TextGen rows and accepted navigation actions refresh activity. All timers use wrap-safe unsigned arithmetic. The application can mark a setting change so the power-cycle-required warning survives session exit; starting a new session clears that warning.

The 350 ms pulse and approximately 250 ms neutral dwell were measured on an Avian 70A Smart Lite through `smart-esc-tool`. They remain unverified on this project's 85A, 100A, and 130A ESCs. The firmware session generates virtual SRXL2 channel values after the receiver/mixer path, so aircraft servo reversal must not alter menu navigation polarity.

## Thrust reverse channel

The captured Avian menu includes `THRUST REV = CH9`. Treat this as the ESC-side SRXL2 slot assignment used by upstream's `esc_srxl2_reverse_channel` setting, not as INAV reversible-motor mode and not as a live receiver-channel mapping.

During normal operation, INAV sends throttle plus the configured virtual reverse slot and drives that slot from the `THRUST REVERSE` mode. TextGen programming temporarily generates only virtual channels 1, 2, and 3 for safe throttle, aileron, and elevator navigation; it does not forward the live receiver vector. Entry requires the reverse mode to be inactive, and any abort restores the upstream driver's ordinary throttle/reverse frame construction. The radio UI should display INAV's configured slot alongside the value read from the ESC so a mismatch is visible.

## Planned MSP/EdgeTX interface

The NEXUS-XR radio path uses MSP over CRSF/ExpressLRS. It cannot consume EdgeTX's Multi-Module raw buffer directly. A minimal MSP interface should expose:

- session state and last stop reason;
- selected TextGen instance, valid-line mask, display revision, and nine fixed-width lines;
- start, stop, refresh acknowledgement, and four navigation commands;
- arming, throttle, failsafe, link, malformed-frame, timeout, and power-cycle-required status.

The EdgeTX Lua application should send actions rather than arbitrary channel values. INAV remains responsible for validating entry conditions, bounding pulse duration and amplitude, forcing safe throttle, and releasing overrides on every abort path.

An EdgeTX radio using a 4-in-1 Multi-Module and a compatible Smart receiver may receive native raw `0x0C` payloads through EdgeTX's Spektrum raw telemetry bridge. That is a separate radio topology from the direct NEXUS-XR-to-ESC bus; both can render the same nine-line model without sharing their transport implementation.

## Host coverage

Native tests currently cover:

- title, body, short, and full-width line decoding;
- spaces, NUL termination, invalid lines, invalid characters, wrong sensor IDs, and wrong lengths;
- instance isolation, refresh acknowledgement, and full display clearing;
- every entry interlock and runtime abort condition;
- safe neutral output, all four navigation directions, pulse bounding, and overlap rejection;
- activity timeout extension and 32-bit timer wrap;
- deterministic override release for explicit exit and malformed telemetry.

## Hardware-deferred validation

With the propeller removed, record and verify:

- the stick/channel sequence required to enter each supported Avian ESC's TextGen menu;
- up/down/left/right polarity, pulse amplitude, minimum pulse duration, repeat behavior, and neutral dwell;
- the selected thrust-reverse channel number, normal/reverse polarity, activation threshold, and safe value used during programming and failsafe;
- line order, update cadence, instance behavior, character set, refresh, and clear packets;
- timeout, exit, receiver loss, FC reboot, ESC brownout, and power-cycle-to-apply behavior.

Store the ESC model, ESC firmware, INAV commit, negotiated baud rate, channel map, and raw captures with the results. Do not enable user-facing write controls until those observations agree with the bounded session model.

## References

- [Spektrum telemetry sensor definitions](https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/spektrumTelemetrySensors.h)
- [EdgeTX Spektrum raw telemetry bridge](https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/spektrum.cpp)
- [DSMTools TextGen renderer and simulator fixtures](https://github.com/frankiearzu/DSMTools)
- [MrScothh Smart ESC tool and measured Avian menu behavior](https://github.com/MrScothh/smart-esc-tool)
