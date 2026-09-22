# NEXUSX Avian Smart ESC validation

## Objective

Validate the Smart ESC implementation merged in INAV PR #11947 on a physical
NEXUS-XR, beginning with the Avian Lite 85A. Only after that path is repeatable
do the same interoperability checks on the available Avian 100A and 130A, then
begin the `inav-avian-esc` EdgeTX/MSP TextGen work.

This branch carries two upstream fixes while they remain outside the selected
`maintenance-10.x` base:

- PR #12002 fixes telemetry invalidation and link fallback caused by comparing
  a newly received frame with an older millisecond timestamp.
- PR #12003 fixes MSP serial-passthrough host ordering, `+++` exit, and baud
  mirroring.

The NEXUSX test build enables `USE_MOTOR_SRXL2` explicitly and advertises only
115200 baud through `MOTOR_SRXL2_115200_ONLY`.

## Safety and wiring gate

1. Remove the propeller. Keep it removed for every test in this document.
2. Record the NEXUS-XR, radio, INAV build, Configurator, and ESC firmware
   versions.
3. Verify ground continuity and the intended BEC/servo-rail power path before
   applying power. A UART signal connection alone does not power the rail.
4. Connect the ESC signal to an exposed UART TX pin, not automatically to the
   PA9 pad labelled `ESC`. Record the chosen UART and physical pin.
5. Start with the motor disconnected. Use a current-limited supply or smoke
   stopper for initial power-up.
6. Capture the single-wire signal passively. Record voltage levels, idle state,
   ESC announcements, directed and broadcast handshakes, control frames,
   telemetry replies, turnaround time, and any collision or UART echo.

## Avian Lite 85A sequence

Run each item as a separate recorded result. A successful later step does not
erase an earlier failure.

1. Build-size check and Configurator visibility on NEXUSX.
2. Cold power-up with FC and ESC together; confirm discovery, negotiated 115200
   baud, link status, and arming inhibition before the link becomes ready.
3. FC-only reboot while the ESC remains powered.
4. ESC-only power cycle while the FC remains powered.
5. Disconnect and reconnect only the signal wire; confirm the reported link
   drops in approximately the configured timeout and cannot remain falsely
   connected because of half-duplex echo.
6. Brownout/recovery simulation within the limits of the bench supply.
7. Sensor `0x20` field validation at rest: voltage, current, RPM, FET
   temperature, unavailable sentinels, and freshness.
8. Measure delivered ESC frames at each upstream telemetry-rate setting. Keep
   request rate and delivered sensor `0x20` rate separate in the notes.
9. Run endpoint calibration with the motor disconnected and record the full
   sequence and stored range.
10. Connect the motor, keep the propeller removed, and verify disarmed output,
    armed idle, several bounded throttle points, reported throttle, RPM, and
    link recovery.
11. Configure the ESC's actual thrust-reverse slot, verify the INAV setting
    matches it, then test reverse from rest and at a conservative running speed.
12. Exercise failsafe, disarm, stale telemetry, Configurator connection, and
    serial-passthrough entry/exit. Reverse must release on disarm.

## Additional ESCs

After the Avian Lite 85A sequence is understood, repeat at minimum cold
power-up, both one-sided reboot cases, calibration, bounded throttle,
sensor `0x20`, telemetry cadence, disconnect detection, and reverse on the 100A
and 130A ESCs. Do not infer baud capability, channel range, telemetry rotation,
or reboot recovery from another model.

## Exit criteria for radio-tool work

Begin the direct ExpressLRS/CRSF configuration-tool stage only when the Avian
Lite 85A can be discovered and safely controlled repeatedly on NEXUS-XR, its
sensor `0x20` stream is understood, link loss cannot leave a false connected
state, and ordinary control is restored after serial-passthrough testing.

The later TextGen branch will reuse the upstream motor driver. It will add a
disarmed MSP session that generates only bounded virtual throttle, aileron, and
elevator navigation values; it will not continuously forward live receiver
channels.
