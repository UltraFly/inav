# Avian ESC TextGen design

Avian TextGen programming is a separate feature from ordinary ESC telemetry. Both use the same SRXL2 wire and polling infrastructure, but TextGen has its own display model, safety state, channel overrides, radio interface, and test plan.

Within this project, TextGen is the only mechanism used to program an Avian ESC. Spektrum Forward Programming is a separate receiver-configuration protocol; implementing that protocol is out of scope. INAV Gyro Assist and its MSP configuration provide the project's INAV-native functional equivalent of Forward-Programming-configured receiver stabilization, but remain completely separate from ESC TextGen. The corresponding radio tool is `inav-avian-esc`; it does not require Gyro Assist.

This first firmware-side increment is hardware-independent. It decodes the 16-byte Spektrum TextGen sensor payload and models a bounded programming session; it does not yet connect the model to an SRXL2 UART, MSP, or an EdgeTX write interface.

## Development branch boundary

`feature/avian-esc-textgen` is the later, stacked branch in the three-branch project layout:

```text
feature/gyro-assist                 (independent Gyro Assist and Lua tool)

feature/avian-esc-telemetry         (SRXL2 bus, throttle only, sensor 0x20)
└── feature/avian-esc-textgen       (this branch)
```

The parent telemetry branch is the only implementation of the shared SRXL2 bus. This branch inherits that transport and adds the riskier configuration behavior: full configured-channel forwarding, ordinary AUX thrust reverse, complete per-channel failsafes, sensor `0x0C`, programming-session and navigation overrides, the future TextGen MSP interface, and `inav-avian-esc`. It must not contain Gyro Assist or `inav-gyro-assist`.

Transport and sensor `0x20` corrections are made on `feature/avian-esc-telemetry`, then this branch is rebased onto the new parent tip. TextGen, reverse, navigation, and full-channel changes remain here. This branch is proposed upstream only after the telemetry/SRXL2 work has been accepted and the additional control paths have completed their own hardware safety validation.

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

The TextGen decoder consumes only the sensor payload. `srxl2EscDecodeTelemetryFrame()` validates the 22-byte SRXL2 envelope and CRC and exposes its 16-byte payload for dispatch to TextGen. Invalid envelopes must never reach the TextGen decoder or refresh the session timeout. Live UART frame routing is still pending.

## Programming-session safety

A session can start only when all five entry conditions are true:

- the aircraft is disarmed;
- input throttle is confirmed low;
- the physical thrust-reverse switch is confirmed in its normal/non-reverse position;
- failsafe is inactive;
- the ESC link is available.

While active, the session produces a channel override with throttle fixed at 1000 microseconds and aileron/elevator centered at 1500 microseconds. A deliberate navigation request produces one bounded 250 ms pulse:

| Action | Aileron | Elevator |
| --- | ---: | ---: |
| Up | 1500 | 2000 |
| Down | 1500 | 1000 |
| Left/back | 1000 | 1500 |
| Right/select | 2000 | 1500 |

The throttle override remains at 1000 microseconds during every navigation pulse. A second request is rejected until the current pulse expires, after which the output automatically returns to neutral even if the caller has not mutated session state.

The session immediately releases all channel overrides and records a reason if the aircraft arms, throttle rises, the thrust-reverse switch leaves its normal position, failsafe begins, the link is lost, malformed TextGen traffic is reported, the user exits, or no valid TextGen activity arrives for five seconds. All timers use wrap-safe unsigned arithmetic. The application can mark a setting change so the power-cycle-required warning survives session exit; starting a new session clears that warning.

The 250 ms pulse duration and direction polarity are safe, explicit initial values based on conventional stick navigation; they are not yet Avian-hardware validated. Confirm them against a supported ESC with the propeller removed before enabling write controls in the radio application. If a model reverses aileron or elevator, the final integration must define whether navigation is applied before or after that reversal and test both cases.

## Thrust reverse channel

The captured Avian menu includes `THRUST REV = CH9`. Treat this as an ESC-side channel assignment, not as INAV reversible-motor mode: the user selects the desired channel in TextGen, and the SRXL2 adapter preserves the same one-based transmitter channel number in its outgoing control mask (`CH9` is protocol channel index `8`).

During normal operation, the assigned AUX channel follows its receiver input. No duplicate channel assignment is required in INAV. Its radio/INAV failsafe value must be explicitly configured and verified as non-reverse, and the adapter must forward the defined failsafe vector rather than stale input. TextGen navigation temporarily overrides only throttle, aileron, and elevator; every other configured channel remains in the outgoing frame. Entry requires the physical reverse switch to be normal, while forced zero throttle prevents a direction request from producing thrust. The radio UI should display the selected assignment once firmware exposes it, but the ESC remains the authority for which channel it observes.

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
