# Avian SRXL2 bus-master design

This document defines the safety and integration contract for the direct INAV-to-Spektrum Avian ESC connection. The first implementation is intentionally split into a hardware-independent discovery state machine and a future target/UART driver. The state machine can be exercised on a host before any shared output pin is driven.

The implementation follows the public Spektrum SRXL2 Rev K specification and uses Spektrum's public protocol library as a behavioral reference. It is an independent INAV implementation rather than a copy of the closed master portion of that library.

## Development branch boundary

`feature/avian-esc-telemetry` is the Avian foundation and the second of three canonical project branches:

```text
feature/gyro-assist                 (independent Gyro Assist and Lua tool)

feature/avian-esc-telemetry         (this branch)
└── feature/avian-esc-textgen       (later configuration work)
```

This branch intentionally combines SRXL2 bus-master infrastructure with sensor `0x20` telemetry because direct telemetry polling cannot work without the FC owning that bus. Its control scheduler sends only the configured throttle channel. It excludes sensor `0x0C`, aileron/elevator/AUX forwarding, thrust reverse, TextGen sessions and navigation, TextGen MSP messages, and `inav-avian-esc`.

Shared Avian transport fixes are made here first. The TextGen branch must then be rebased onto this branch so it inherits the fix without creating a second bus implementation. TextGen-only behavior must never be back-ported here. This branch is the sole source for the planned SRXL2 bus-master and Avian telemetry upstream merge request; Gyro Assist is reviewed separately from `feature/gyro-assist`.

## Scope

`io/esc_srxl2.c` owns packet encoding, packet validation, ESC telemetry decoding, and conversion of common values into INAV ESC sensor units.

`io/esc_srxl2_bus.c` owns the transport-independent safety states. It does not configure pins, start a timer, open a UART, or schedule tasks. A future hardware adapter will perform those operations and will use this state machine to decide when serial transmission is permitted.

`io/esc_srxl2_control.c` owns the transport-independent 11 ms control cadence, current and failsafe throttle snapshots, safe-throttle substitution, and stale-input handling. The telemetry feature deliberately transmits only the configured throttle channel. Full receiver-channel forwarding, thrust reverse, and programming overrides belong to the later Avian TextGen feature.

The bus layer currently supports one directly connected ESC in the `0x40` device-ID range. It deliberately does not implement receiver-side SRXL2, DSM RF transport, or TextGen session behavior.

## Discovery states

| State | Permitted behavior |
| --- | --- |
| `DISABLED` | No serial control, handshake, or telemetry activity. |
| `LISTEN_GUARD` | Listen at 115200 8N1. The shared signal pin must remain undriven. |
| `SEND_DIRECTED_HANDSHAKE` | A CRC-valid ESC announcement identified SRXL2, or an explicitly configured SRXL2-only output completed its silent guard, so a directed handshake may be sent. |
| `WAIT_HANDSHAKE_REPLY` | Listen for the matching ESC reply. Retry after 50 ms for an announced ESC, or once per second while an SRXL2-only output is still discovering the default ESC address. |
| `SEND_FINAL_HANDSHAKE` | Send the broadcast handshake while continuing to advertise 115200-only operation. |
| `WAIT_FINAL_TX_COMPLETE` | Keep the UART at 115200 until the final byte has physically left the UART. |
| `RUNNING` | Control frames are permitted at 115200 baud. |
| `PWM_FALLBACK` | No SRXL2 device was detected during the 200 ms guard; serial transmission remains prohibited and the hardware adapter may select PWM. |

A valid unprompted ESC handshake completes the listen-before-transmit guard early. Random UART traffic, a bad CRC, or a handshake from a non-ESC device cannot unlock transmission. Discovery has two explicit policies:

- `PASSIVE_PWM_FALLBACK` preserves automatic PWM compatibility. If no valid ESC handshake arrives within 200 ms, the state machine returns no frame, selects `PWM_FALLBACK`, and never probes the shared pin by transmitting.
- `ACTIVE_SRXL2_ONLY` is permitted only when the output has been explicitly configured for SRXL2. It remains silent for the same 200 ms guard and then offers a directed handshake to default ESC address `0x40` once per second until the ESC responds. It never selects PWM. This allows discovery when the FC is powered before an ESC that does not emit an announcement after seeing an already-active master.

The FC uses SRXL2 receiver/master device ID `0x21`. It advertises no high-baud capability in both handshake responses and never switches away from 115200. A 400000-baud transition is unsupported and must be treated by the future hardware adapter as a link/configuration failure, not as an alternate operating mode or recovery path.

An ESC unprompted handshake received while running is treated as a probable ESC brownout. Control output is blocked immediately, telemetry becomes stale, and discovery restarts with a directed handshake at 115200.

## Hardware adapter contract

The NEXUS-X/XR ESC signal is PA9. That pin can be TIM1 channel 2 or USART1 TX, while the current target maps USART1 to PB6/PB7 for AUX/SBUS. Integration must therefore add an explicit alternate-function mapping for PA9 or initially use a documented dedicated UART pin. Timer and UART ownership must be mutually exclusive.

The adapter must obey all of the following rules:

1. Start with the signal pin in receive/high-impedance mode and the UART receiver at 115200 baud. The idle line must not be driven high. A third-party Firma 150A trace reports that idle-high at ESC power-up permanently selected PWM for that boot, while released/low selected serial detection; use a defined pull-down if PA9 otherwise floats, then verify this behavior on every Avian model.
2. Do not enable PWM or UART transmission while `LISTEN_GUARD` is active.
3. On `PWM_FALLBACK`, close or release the UART before assigning the timer resource and producing a defined safe PWM throttle value.
4. Once a valid ESC handshake selects SRXL2, never assign the same pin to the timer until the bus has been disabled and a deliberate fallback transition has completed.
5. Call `srxl2EscBusOnFrameTransmitted()` only after UART transmission-complete, not merely after filling the transmit register or DMA buffer. The bus must remain at 115200 after that final handshake.
6. Send scheduled channel frames only through `srxl2EscControlSchedulerBuildFrame()`. It delegates final bus-state gating to `srxl2EscBusBuildControlFrame()`, so neither layer permits control transmission outside `RUNNING`.
7. Feed the scheduler only the configured throttle channel. Do not forward aileron, elevator, thrust reverse, or other AUX channels in the telemetry feature.
8. Configure a safe throttle failsafe before enabling control transmission. The scheduler forces minimum throttle during disarm, failsafe, and stale input, and refuses to transmit a failsafe frame until that value is defined.
9. Request ESC telemetry only in every thirtieth successfully built control frame. After granting a reply, reserve a 20 ms receive window by skipping the next nominal 11 ms transmission slot. The resulting request rate is about 2.9 Hz. This conservative starting rate is consistent with ArduPilot's approximately 3 Hz scheduling of sensor `0x20` on its receiver-facing Spektrum telemetry link, although that is evidence of a useful presentation rate rather than a direct ESC polling requirement. The divider and receive window remain provisional hardware-test values: the 20 ms value is based on third-party Firma replies observed 4--16 ms after a grant and must be measured on all three Avian ESCs. Use a zero reply ID in intervening frames and every failsafe frame; the codec also enforces reply suppression for failsafe.
10. Encode the normal 1000--2000 microsecond channel range into `0x2AA0`--`0xD554`, preserving `0x8000` as center and clearing the two reserved low bits. Do not emit raw zero as minimum throttle.
11. Mark telemetry fresh only after complete length and CRC validation. Expired data must not be published as current ESC sensor data.
12. Allow `srxl2EscBusRestartDiscovery()` from PWM fallback only after the aircraft is disarmed, throttle is at minimum, PWM has stopped, and the pin is back in receive mode.

The UART must use 3.3 V half-duplex single-wire behavior, leave the bus undriven while idle, and provide the required idle pull-down without fighting the ESC. A reported ChibiOS Firma test required the equivalent of `HalfDuplex + Pulldown_TX`; that setting is evidence about the electrical contract, not an INAV configuration prescription. Turnaround timing and the two-character inter-packet idle requirement still need to be implemented and verified in the hardware adapter.

Moving the ESC signal from a PWM output to a UART-capable pin can also remove the normal BEC-to-servo-rail power path on some installations. The board integration documentation must explicitly state whether a split lead or separate rail-power connection is required; protocol success does not prove that the control surfaces are powered.

## Host coverage

Native unit tests cover:

- silence for the entire startup guard and PWM fallback without a transmitted probe;
- CRC-invalid, wrong-device, and wrong-destination handshakes;
- directed and broadcast handshake contents;
- fixed master device ID `0x21`, 115200-only advertisement, and no high-baud transition even when the ESC advertises high-baud capability;
- handshake retry timing;
- control-frame gating before and after discovery;
- an ESC brownout handshake while running;
- explicit late-ESC rediscovery from PWM fallback;
- telemetry freshness expiration, including a 32-bit clock wrap;
- disabled-bus behavior.

The control scheduler tests additionally cover:

- immediate first output followed by an 11 ms ordinary cadence without catch-up bursts;
- bus-state gating and deterministic timing reset;
- a one-bit channel mask containing only the configured throttle channel;
- one telemetry request every thirtieth control frame, with zero reply ID between requests;
- a 20 ms protected receive window after each telemetry grant, implemented by skipping one nominal control slot, plus request and skipped-slot counters for bench diagnostics;
- safe throttle during disarm, stale input, and explicit failsafe;
- safe-throttle failsafe enforcement and telemetry-reply suppression;
- stale-input and cadence behavior across a 32-bit microsecond clock wrap.

The packet codec separately covers protocol-level channel ordering and masks, PWM range conversion, failsafe reply suppression, malformed ESC telemetry, unavailable fields, field ranges, electrical-to-mechanical RPM conversion, and INAV unit conversion. Generic codec coverage does not enable multi-channel forwarding in the telemetry scheduler.

## Hardware-deferred validation

The direct-path test matrix is a NEXUS-XR with a RadioMaster TX16S MK3 ExpressLRS transmitter, tested separately against a 6S Avian Lite 85A ESC, a 100A Avian ESC, and a 130A Avian ESC. The existing TX16S MK2 internal-4-in-1, AR10360T+, and Avian Lite 85A chain remains the known-working Spektrum behavioral reference.

Do not consider the transport flight-ready until all of these have been demonstrated with the propeller removed:

- PA9 electrical idle level and half-duplex direction changes on an oscilloscope or logic analyzer;
- no edge or pulse emitted during the 200 ms guard with a PWM-only ESC attached;
- correct fallback to a safe PWM value for a non-SRXL2 ESC;
- Avian handshake and continuous operation at 115200, including confirmation that the ESC never attempts a 400000-baud transition;
- `0x2AA0` safe idle, `0x8000` center, and `0xD554` full travel compared with a known-working Spektrum receiver trace;
- one sensor `0x20` reply request every thirtieth emitted control frame (about 2.9 Hz with the protected receive slot), with request count, valid/invalid reply count, latency distribution, collisions, and bus loading recorded so both divider and window can be justified;
- operation at both standard 11 ms and 22 ms RC frame rates compared against the known Spektrum chain. The Firma/SR6100AT trace happened to use about 22 ms; that is an ordinary rate selection, not evidence of a Firma-specific cadence or a contradiction of the 11 ms implementation default;
- two-character turnaround gaps, telemetry replies arriving 4--16 ms after a grant, and collision-free recovery; treat that latency range as a Firma test hypothesis until Avian captures confirm or replace it;
- ESC late power-up and ESC brownout recovery;
- disarm, receiver failsafe, FC reboot, and malformed traffic;
- voltage, current, eRPM/RPM, temperature, BEC, throttle, and power-output comparison with a known reference.

For each ESC, record the NEXUS hardware revision, TX16S MK3 and ExpressLRS versions, INAV commit, exact ESC model and firmware, negotiated baud rate, throttle channel, telemetry poll divider and measured reply rate, and raw logic-analyzer captures with the test results.

## References

- [Spektrum SRXL2 protocol, Rev K specification, and public reference implementation](https://github.com/SpektrumRC/SRXL2)
- [Spektrum ESC telemetry structure](https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/spektrumTelemetrySensors.h)
- [NEXUS-X target documentation](https://github.com/iNavFlight/inav/blob/master/docs/boards/NEXUSX.md)
- [RobertoD91's INAV SRXL2 ESC proof of concept](https://github.com/RobertoD91/inav/tree/claude/esc-spektrum-protocol-check-i5b6jx) — source of the candidate `0x21`, 115200-only, normal-travel channel endpoints, and 1-in-10 polling behavior; all remain **NEEDS VERIFYING** on this project's hardware
- [MSRC issue #152: Adding Spektrum Avian Smart ESC Telemetry](https://github.com/dgatf/msrc/issues/152) — third-party bench history and interoperability evidence; **NEEDS VERIFYING**
- [ArduPilot issue #27603: Spektrum Smart ESC support](https://github.com/ArduPilot/ardupilot/issues/27603) — open feature request; as of this review there is no merged ArduPilot Smart-ESC bus master
- [Firma 150A bench and logic-analyzer report](https://github.com/ArduPilot/ardupilot/issues/27603#issuecomment-5024088801) — source of the idle pull-down, continuous discovery, approximately 22 ms reference cadence, 4--16 ms reply latency, collision, and sensor-scaling observations; unpublished third-party ArduPilot 4.6.3 work and **NEEDS VERIFYING** on Avian
- [ArduPilot integration notes](https://github.com/ArduPilot/ardupilot/issues/27603#issuecomment-5035083785) — independently supports receiver-role master ID `0x21` and a separate ESC-facing state machine; **NEEDS VERIFYING**
- [ArduPilot receiver-facing Spektrum telemetry scheduler](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RCTelemetry/AP_Spektrum_Telem.cpp) — schedules ESC sensor `0x20` at approximately 3 Hz; evidence that this update rate is useful, not evidence of the direct ESC's required polling cadence
- [Spektrum SRXL2 electrical clarification](https://github.com/SpektrumRC/SRXL2/issues/3#issuecomment-1172830140) — 3.3 V half-duplex, 115200 baud with optional 400000 baud, handshake timeout, and typical 11 ms receiver packet cadence
