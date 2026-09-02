# NEXUS-X/XR acro airplane roadmap

This roadmap keeps the work split into reviewable changes that can be proposed upstream independently.

## 1. Spektrum Avian Smart ESC support

Avian Smart Throttle uses SRXL2 on the ESC signal wire. INAV already implements the non-master side of SRXL2 for Spektrum receivers, but an Avian ESC needs INAV to act as the bus master. The NEXUS-X/XR `ESC` connector is suitable because its PA9 signal pin can be either TIM1 channel 2 or half-duplex USART1 TX.

The current NEXUSX target maps USART1 to the separate PB6/PB7 AUX/SBUS pins, however. The transport patch must therefore add an explicit, conflict-checked PA9 alternate-function mapping, or initially document use of the AUX UART1 TX pin. It must not silently take ownership of PA9 from the timer output.

Implementation stages:

1. Add a transport-independent SRXL2 ESC codec with host unit tests.
2. Add a single-wire, half-duplex bus-master state machine: listen before transmit, discover ESC device `0x40`, negotiate baud rate, send throttle channel data, request telemetry, and recover after a bus reset.
3. Add an SRXL2 motor protocol and ensure timer/UART ownership is exclusive. Initially support one aircraft ESC only.
4. Convert Spektrum ESC sensor `0x20` data to INAV's existing `escSensorData_t` units so battery metering, OSD, blackbox, and radio telemetry work without protocol-specific consumers.
5. Verify startup, disarm, failsafe, late ESC power-up, CRC rejection, and PWM fallback on real NEXUS-XR and Avian hardware with the propeller removed.

Forward Programming is a separate concern. The public SRXL2 specification describes pass-through framing but not Avian parameter identifiers. ESC settings should only be added after an authoritative definition or captured, repeatable transactions are available.

## 2. Surface-deflection stabilization

Add an opt-in fixed-wing controller in which pilot stick input directly commands normalized surface deflection and gyro feedback adds a bounded damping correction. Keep the existing PIFF rate controller unchanged.

Implementation stages:

1. Specify the control law, saturation/anti-windup behavior, mode transitions, and interaction with launch, navigation, failsafe, and autotrim.
2. Add pure control-law tests for neutral stick, full deflection, gust rejection, saturation, and sign conventions.
3. Add per-axis settings for damping gain, correction limit, and filtering, scoped to airplane manual/stabilized modes.
4. Add blackbox fields or debug modes needed for flight tuning, followed by staged bench and flight testing.

## 3. EdgeTX aircraft setup Lua

Build this as a separate radio artifact after the firmware settings stabilize. Use MSP over CRSF/ExpressLRS telemetry for NEXUS-XR rather than Spektrum TextGen. The script should present aircraft-oriented pages for receiver/output mapping, surface directions and limits, stabilization, Avian telemetry status, failsafe, and pre-flight validation.

TextGen remains useful for Spektrum transmitters and CMS, but it is not the configuration transport used by an EdgeTX/ExpressLRS installation.

## Safety and upstream criteria

- Never emit serial bytes on a shared PWM throttle pin until the SRXL2 discovery guard time has elapsed.
- Disarmed and failsafe throttle behavior must be explicit and covered by tests.
- New behavior must be opt-in and leave existing motor protocols and fixed-wing controllers unchanged.
- Protocol code must cite the public Spektrum SRXL2 and telemetry-structure specifications and avoid reverse-engineered constants where an authoritative definition is unavailable.
- Hardware validation records should include firmware revision, ESC model/firmware, baud rate, captures of handshake/control/telemetry frames, and observed fallback behavior.

## References

- [Spektrum SRXL2 protocol and reference implementation](https://github.com/SpektrumRC/SRXL2)
- [Spektrum telemetry sensor structures](https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/spektrumTelemetrySensors.h)
- [INAV NEXUSX target documentation](https://github.com/iNavFlight/inav/blob/master/docs/boards/NEXUSX.md)
- [RadioMaster NEXUS-XR product documentation](https://radiomasterrc.com/products/nexus-xr-control-system)
