# Avian SRXL2 bus-master design

This document defines the safety and integration contract for the direct INAV-to-Spektrum Avian ESC connection. The first implementation is intentionally split into a hardware-independent discovery state machine and a future target/UART driver. The state machine can be exercised on a host before any shared output pin is driven.

The implementation follows the public Spektrum SRXL2 Rev K specification and uses Spektrum's public protocol library as a behavioral reference. It is an independent INAV implementation rather than a copy of the closed master portion of that library.

## Scope

`io/esc_srxl2.c` owns packet encoding, packet validation, ESC telemetry decoding, and conversion of common values into INAV ESC sensor units.

`io/esc_srxl2_bus.c` owns the transport-independent safety states. It does not configure pins, start a timer, open a UART, or schedule tasks. A future hardware adapter will perform those operations and will use this state machine to decide when serial transmission is permitted.

`io/esc_srxl2_control.c` owns the transport-independent 11 ms control cadence, receiver and failsafe channel snapshots, safe-throttle substitution, stale-input handling, and bounded programming overrides. It accepts INAV's sequential transmitter-channel vector, preserves indices `0` through `31`, and deliberately omits only channels that cannot be represented by SRXL2's 32-bit mask.

The bus layer currently supports one directly connected ESC in the `0x40` device-ID range. It deliberately does not implement receiver-side SRXL2, DSM RF transport, or TextGen session behavior.

## Discovery states

| State | Permitted behavior |
| --- | --- |
| `DISABLED` | No serial control, handshake, or telemetry activity. |
| `LISTEN_GUARD` | Listen at 115200 8N1. The shared signal pin must remain undriven. |
| `SEND_DIRECTED_HANDSHAKE` | A CRC-valid ESC handshake has identified SRXL2, so a directed handshake may be sent. |
| `WAIT_HANDSHAKE_REPLY` | Listen for the matching ESC reply. Retry the directed handshake no faster than every 50 ms. |
| `SEND_FINAL_HANDSHAKE` | Send the broadcast handshake containing the baud rate supported by both endpoints. |
| `WAIT_FINAL_TX_COMPLETE` | Keep the UART at 115200 until the final byte has physically left the UART. |
| `RUNNING` | Control frames are permitted at the negotiated baud. |
| `PWM_FALLBACK` | No SRXL2 device was detected during the 200 ms guard; serial transmission remains prohibited and the hardware adapter may select PWM. |

A valid unprompted ESC handshake completes the listen-before-transmit guard early. Random UART traffic, a bad CRC, or a handshake from a non-ESC device cannot unlock transmission. If no valid ESC handshake arrives within 200 ms, the state machine returns no frame and selects `PWM_FALLBACK`; it never probes the shared pin by transmitting.

An ESC unprompted handshake received while running is treated as a probable ESC brownout. Control output is blocked immediately, the negotiated baud returns to 115200, telemetry becomes stale, and discovery restarts with a directed handshake.

## Hardware adapter contract

The NEXUS-X/XR ESC signal is PA9. That pin can be TIM1 channel 2 or USART1 TX, while the current target maps USART1 to PB6/PB7 for AUX/SBUS. Integration must therefore add an explicit alternate-function mapping for PA9 or initially use a documented dedicated UART pin. Timer and UART ownership must be mutually exclusive.

The adapter must obey all of the following rules:

1. Start with the signal pin in receive/high-impedance mode and the UART receiver at 115200 baud.
2. Do not enable PWM or UART transmission while `LISTEN_GUARD` is active.
3. On `PWM_FALLBACK`, close or release the UART before assigning the timer resource and producing a defined safe PWM throttle value.
4. Once a valid ESC handshake selects SRXL2, never assign the same pin to the timer until the bus has been disabled and a deliberate fallback transition has completed.
5. Call `srxl2EscBusOnFrameTransmitted()` only after UART transmission-complete, not merely after filling the transmit register or DMA buffer. This prevents switching to 400000 baud before the final 115200-baud handshake is complete.
6. Send scheduled channel frames only through `srxl2EscControlSchedulerBuildFrame()`. It delegates final bus-state gating to `srxl2EscBusBuildControlFrame()`, so neither layer permits control transmission outside `RUNNING`.
7. Feed the scheduler receiver channels in their original transmitter-number order, before any INAV logical-function remapping. It forwards every configured channel representable by SRXL2 (protocol indices `0` through `31`). This lets the ESC observe an auxiliary channel selected in TextGen, such as the captured `THRUST REV = CH9` assignment.
8. Configure a complete failsafe vector before enabling control transmission. The scheduler overrides the configured throttle channel to the safe minimum during disarm, failsafe, stale input, and TextGen programming. It refuses to send a stale/failsafe frame if any normally forwarded channel lacks an explicit failsafe value, preventing stale or undefined AUX values from reaching the ESC. The ESC-selected thrust-reverse channel must have a verified non-reverse failsafe value.
9. Use a zero reply ID in failsafe frames. The codec enforces this even if a caller supplies an ESC reply ID.
10. Mark telemetry fresh only after complete length and CRC validation. Expired data must not be published as current ESC sensor data.
11. Allow `srxl2EscBusRestartDiscovery()` from PWM fallback only after the aircraft is disarmed, throttle is at minimum, PWM has stopped, and the pin is back in receive mode.

The UART must use half-duplex single-wire behavior and leave the bus undriven while idle. Turnaround timing and the two-character inter-packet idle requirement still need to be implemented and verified in the hardware adapter.

## Host coverage

Native unit tests cover:

- silence for the entire startup guard and PWM fallback without a transmitted probe;
- CRC-invalid, wrong-device, and wrong-destination handshakes;
- directed and broadcast handshake contents;
- 115200/400000 negotiation and delayed baud switching;
- handshake retry timing;
- control-frame gating before and after discovery;
- an ESC brownout handshake while running;
- explicit late-ESC rediscovery from PWM fallback;
- telemetry freshness expiration, including a 32-bit clock wrap;
- disabled-bus behavior.

The control scheduler tests additionally cover:

- immediate first output followed by an 11 ms cadence without catch-up bursts;
- bus-state gating and deterministic timing reset;
- forwarding 9, 16, and up to 32 sequential transmitter channels while capping INAV's larger vector;
- preservation of the captured human `CH9` thrust-reverse assignment as protocol index `8`;
- safe throttle during disarm, programming, stale input, and explicit failsafe;
- complete failsafe-vector enforcement and telemetry-reply suppression;
- programming overrides that affect only requested channels and cannot override the throttle guard;
- unsafe or malformed programming requests falling back to failsafe;
- stale-input and cadence behavior across a 32-bit microsecond clock wrap.

The packet codec separately covers multi-channel ordering, the complete 32-channel mask, preservation of the human `CH9` thrust-reverse assignment as protocol channel index `8`, PWM range conversion, failsafe reply suppression, malformed telemetry, unavailable fields, field ranges, electrical-to-mechanical RPM conversion, and INAV unit conversion.

## Hardware-deferred validation

Do not consider the transport flight-ready until all of these have been demonstrated with the propeller removed:

- PA9 electrical idle level and half-duplex direction changes on an oscilloscope or logic analyzer;
- no edge or pulse emitted during the 200 ms guard with a PWM-only ESC attached;
- correct fallback to a safe PWM value for a non-SRXL2 ESC;
- Avian handshake at 115200 and, where supported, the transition to 400000;
- control cadence, two-character turnaround gaps, telemetry reply timing, and collision-free recovery;
- ESC late power-up and ESC brownout recovery;
- disarm, receiver failsafe, FC reboot, malformed traffic, and TextGen abort behavior;
- voltage, current, eRPM/RPM, temperature, BEC, throttle, and power-output comparison with a known reference.

Record the NEXUS hardware revision, INAV commit, ESC model and firmware, negotiated baud rate, channel map, and raw logic-analyzer captures with the test results.

## References

- [Spektrum SRXL2 protocol, Rev K specification, and public reference implementation](https://github.com/SpektrumRC/SRXL2)
- [Spektrum ESC telemetry structure](https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/spektrumTelemetrySensors.h)
- [NEXUS-X target documentation](https://github.com/iNavFlight/inav/blob/master/docs/boards/NEXUSX.md)
