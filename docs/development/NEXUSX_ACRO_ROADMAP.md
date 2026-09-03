# NEXUS-X/XR acro airplane roadmap

This roadmap keeps the work split into reviewable changes that can be proposed upstream independently.

## 1. Shared Spektrum Avian SRXL2 support

Avian Smart Throttle uses SRXL2 on the ESC signal wire. INAV already implements the non-master side of SRXL2 for Spektrum receivers, but an Avian ESC needs INAV to act as the bus master. The NEXUS-X/XR `ESC` connector is suitable because its PA9 signal pin can be either TIM1 channel 2 or half-duplex USART1 TX.

The current NEXUSX target maps USART1 to the separate PB6/PB7 AUX/SBUS pins, however. The transport patch must therefore add an explicit, conflict-checked PA9 alternate-function mapping, or initially document use of the AUX UART1 TX pin. It must not silently take ownership of PA9 from the timer output.

Implementation stages:

1. Add a transport-independent SRXL2 ESC codec with host unit tests.
2. Expand control framing from throttle-only to a channel mask containing throttle, aileron, elevator, and an optional reversing/auxiliary channel. TextGen programming relies on these ordinary channel values.
3. Add a single-wire, half-duplex bus-master state machine: listen before transmit, discover ESC device `0x40`, negotiate baud rate, send channel data, request telemetry, and recover after a bus reset.
4. Add an SRXL2 motor protocol and ensure timer/UART ownership is exclusive. Initially support one aircraft ESC only.

The shared layer ends at reliable bus operation and channel transport. Telemetry decoding and TextGen programming remain independent consumers so either feature can be reviewed, enabled, and tested without requiring the other.

Current status: the transport-independent codec builds handshake frames and variable-length channel frames, orders channel values by mask bit, clamps PWM inputs, suppresses telemetry requests during failsafe, and is covered by native host tests. UART ownership, discovery, baud negotiation, and the live bus scheduler remain to be implemented.

## 2. Avian ESC telemetry

Decode Spektrum ESC sensor `0x20` data and convert it to INAV's existing `escSensorData_t` units so battery metering, OSD, blackbox, and radio telemetry work without protocol-specific consumers.

Implementation stages:

1. Add valid, truncated, malformed, out-of-range, byte-order, and stale-frame fixtures to the host tests.
2. Map voltage, current, RPM, temperature, throttle, BEC, and other supported fields into existing INAV units with explicit unavailable-value handling.
3. Verify polling cadence, timeout behavior, late ESC power-up, CRC rejection, bus recovery, and PWM fallback.
4. Bench-test live values against a known reference with the propeller removed and record the ESC model, firmware, baud rate, and captures.

Current status: sensor `0x20` decoding, CRC and length validation, unavailable-value handling, throttle range validation, and conversion of RPM, voltage, FET temperature, and motor current into INAV ESC sensor units are covered by native host tests. Publishing those values to the live sensor subsystem awaits the bus transport; BEC, throttle, and power-output values remain available in the protocol-specific structure for a future MSP/radio status interface.

## 3. Avian ESC TextGen programming

TextGen is a separate user-facing feature built on the shared SRXL2 transport. Sensor `0x0C` supplies an instance byte, a line number, and 13 text characters. Line `254` requests a refresh/backlight action and line `255` clears the display. The menu is entered with throttle at zero and navigated using ordinary aileron and elevator channel positions; affected ESCs require a power cycle to apply changes.

EdgeTX, DIY Multi-Module, and DSMTools provide working behavioral references for Spektrum telemetry and raw TextGen access where the public documents stop at the generic transport. Multi-Module speaks the DSM RF side through a Smart receiver; it does not replace the direct SRXL2 bus-master implementation between NEXUS-XR and the ESC.

Implementation stages:

1. Decode TextGen frames into a bounded nine-line, 13-character display model with strict instance, line, length, and character validation.
2. Implement line `254` refresh/backlight and line `255` clear behavior with host fixtures based on documented or captured traffic.
3. Add an explicit programming-session state machine that requires disarm and zero throttle before entry.
4. Translate bounded navigation actions into the required aileron/elevator SRXL2 channel values without allowing throttle to rise.
5. Expose the display model, navigation state, errors, and power-cycle requirement through MSP for radio clients.
6. Restore normal channel forwarding deterministically on exit, timeout, malformed traffic, link loss, failsafe, or reboot.
7. Bench-test entry, navigation, setting changes, exit, persistence after power cycle, and every abort path with the propeller removed.

## 4. Surface-deflection stabilization

Add a separate, opt-in fixed-wing controller in which pilot stick position commands the baseline normalized control-surface deflection. Do not convert stick position into an angular-rate setpoint. Add filtered gyro feedback only as a bounded disturbance-compensation term, then pass the combined command through the existing mixer and servo limits. Keep the existing PIFF/rate controller unchanged.

The intended per-axis control form is:

`surfaceCommand = clamp(pilotDeflection + limitedGyroCorrection)`

Pilot authority must remain predictable at every stick position. The gyro correction must have an independently configured limit, use explicit polarity and filtering, and must not move the final output beyond mixer or servo constraints.

Implementation stages:

1. Specify the control law, gyro filtering, correction limiting, final saturation, and polarity conventions before implementation.
2. Define exactly which fixed-wing manual/stabilized modes may use it and how it interacts with launch, navigation, failsafe, autotrim, and invalid or stale gyro data.
3. Make entry and exit bumpless and ensure disarm, failsafe, and controller changes cannot produce a surface step.
4. Add pure host tests for neutral and full-stick commands, positive and negative gusts, correction polarity, configured correction limits, final saturation, reversed axes, stale gyro data, mode transitions, and failsafe.
5. Add per-axis settings for damping gain, correction limit, and filtering without changing existing controller defaults or behavior.
6. Add blackbox fields or debug modes that separately expose pilot deflection, gyro correction, and final command.
7. Perform servo-direction and endpoint checks on the bench, restrained airframe tests with the propeller removed, and conservative staged flight testing with an immediate conventional-mode fallback.

## 5. EdgeTX aircraft setup Lua

Build this as a separate radio artifact after the firmware settings stabilize. The script should present aircraft-oriented pages for receiver/output mapping, surface directions and limits, stabilization, Avian telemetry status, failsafe, and pre-flight validation.

The first read-only UI and simulator prototype is maintained on the separate `feature/edgetx-nexus-acro` branch. It already displays discovered ESC telemetry, EdgeTX outputs, simulated surface-controller data, and raw `0x0C` TextGen received through a 4-in-1 module. Aircraft-changing controls remain disabled until matching, safety-enforced INAV MSP messages exist.

Support the two EdgeTX topologies without confusing their transports:

1. EdgeTX with a 4-in-1 Multi-Module and Smart receiver can consume native Spektrum telemetry and TextGen through EdgeTX's raw Spektrum telemetry interface. DSMTools is the working reference for this path.
2. NEXUS-XR's internal ExpressLRS receiver uses CRSF, not the Multi-Module `multiBuffer` interface. Carry the same TextGen line model, navigation requests, ESC status, and INAV settings through MSP over CRSF. Reuse the documented behavior and user experience rather than the DSM RF transport.

For the NEXUS-XR path, add an explicit programming session that requires disarm and zero throttle, maps deliberate Lua navigation actions to bounded aileron/elevator SRXL2 values, restores normal channel forwarding on exit, and reports whether a power cycle is required.

## Safety and upstream criteria

- Never emit serial bytes on a shared PWM throttle pin until the SRXL2 discovery guard time has elapsed.
- Disarmed, programming, and failsafe throttle behavior must be explicit and covered by tests.
- Entering TextGen programming must require disarm and zero throttle; leaving it must restore channel forwarding deterministically.
- New behavior must be opt-in and leave existing motor protocols and fixed-wing controllers unchanged.
- Protocol code must cite the public Spektrum specifications where available. Constants derived from EdgeTX, Multi-Module, DSMTools, or captures must record their source and be independently implemented rather than copied wholesale.
- Hardware validation records should include firmware revision, ESC model/firmware, baud rate, captures of handshake/control/telemetry frames, and observed fallback behavior.

## References

- [Spektrum SRXL2 protocol and reference implementation](https://github.com/SpektrumRC/SRXL2)
- [Spektrum telemetry sensor structures](https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/spektrumTelemetrySensors.h)
- [Spektrum Avian TextGen programming instructions](https://www.spektrumrc.com/on/demandware.static/-/Sites-horizon-master/default/dw71772f5d/Manuals/SPMXAE1160HV-SPMXAE1200HV-Manual-EN.pdf)
- [EdgeTX Spektrum telemetry and raw Lua bridge](https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/spektrum.cpp)
- [DIY Multi-Module DSM transport](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Multiprotocol/DSM_cyrf6936.ino)
- [DSMTools telemetry, capture, simulator, and TextGen implementation](https://github.com/frankiearzu/DSMTools)
- [INAV NEXUSX target documentation](https://github.com/iNavFlight/inav/blob/master/docs/boards/NEXUSX.md)
- [RadioMaster NEXUS-XR product documentation](https://radiomasterrc.com/products/nexus-xr-control-system)
