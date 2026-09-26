# BEC voltage (VBEC)

VBEC measures a BEC-powered supply locally through the flight controller's ADC.
It is independent of the main battery voltage (VBAT). For example, a plane can
obtain flight-pack voltage from ESC telemetry while INAV measures the supply
reaching the servo connectors. A board with suitable sensing hardware can also
monitor a BEC that supplies electronics on another type of aircraft.

The board's wiring determines which supply is measured. A BEC output reported
by an ESC and the voltage measured at the flight controller are different
measurement points. VBEC uses the local ADC, regardless of ESC telemetry state.

## Configuration

The feature requires a board ADC input connected to the chosen supply through
an appropriate voltage divider. A connector marked `5V`, `BEC` or `VB2` does not
by itself establish that the servo or electronics supply is already wired to
that input. Consult the board's schematic or target documentation.

| Setting | Meaning |
| --- | --- |
| `vbec_adc_channel` | ADC channel 1–6. Default `0` disables VBEC on every target. Save and reboot after changing the channel. |
| `vbec_scale` | Voltage divider ratio multiplied by 100, using the same convention as `vbat_scale`. Generic default is 100; a target can provide its nominal divider scale. |

If the selected channel is already allocated to an active ADC function, VBEC
remains unavailable and the existing allocation is preserved. A channel not
implemented by the board also remains unavailable. The CLI `status` command
shows the configured and allocated BEC channel and the VBEC reading.

VBEC has its own persistent configuration group. Existing battery settings and
the four existing ADC assignments retain their storage layout and values.
VBAT source selection, cell detection, current integration and battery alarms
are unchanged. No BEC alarm or arming condition is added.

## RadioMaster NEXUS-X/XR

The `NEXUSX` target declares PC1 as ADC channel 2 for the BEC/servo supply.
The separate PC0 input is the external flight-battery voltage input.

```text
set vbec_adc_channel = 2
set vbec_scale = 620
save
```

The scale of 620 is nominal, not a measured calibration. It follows the
[Rotorflight NEXUS-XR target's BEC divider configuration](https://github.com/rotorflight/rotorflight-targets/blob/master/configs/RDMS-NEXUS_XR.config):
`100 × 1216 / 196 ≈ 620`. Confirm it against a voltmeter at the servo power pins.
Rotorflight also explicitly describes its
[BEC meter as servo-bus voltage](https://rotorflight.org/docs/2.1.0/configurator/tabs/power#voltage-meters).

After reboot, use `status` to read VBEC. Let the reading settle and adjust:

```text
new_scale = old_scale × voltmeter_voltage / reported_VBEC
```

Round to an integer, set `vbec_scale`, and save. Do not calibrate from a zero
or unavailable reading. Both measured and reported values must refer to the
same supply at the same time.

## Native CRSF / ExpressLRS telemetry

VBAT continues to use the existing CRSF Battery Sensor frame (`0x08`). VBEC uses
the standard
[Voltage Group frame (`0x0E`)](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x0e-voltages-or-voltage-group)
with values in millivolts. INAV's internal reading has 0.01 V resolution.

The proposed INAV convention is source ID `0xC8`, element 0 = local VBEC.
The value `0xC8` identifies the FC for this INAV mapping; CRSF does **not** reserve
it as a universal BEC voltage source ID. Other producers using the same source
and element would collide and need a different assignment.

In EdgeTX 2.11.2 or newer, sensor discovery initially names the measurement
`Volt`, with ID `C8FE`, instance 0. EdgeTX uses its internal `0xFE` voltage-array
type for general voltage groups; the on-wire CRSF frame type remains `0x0E`.
Rename it `VBEC` in the model's telemetry settings. This fits the four-character sensor
name limit. The voltage frame contains no text label; INAV cannot send the name
`VBEC` through it. Existing `RxBt` remains the usual flight-battery reading.

No Lua tool or dedicated Configurator page is required. These settings use
INAV's existing CLI and generic MSP settings access.

## Availability and verification

The measurement uses a 1 Hz low-pass filter and starts at the first valid
sample. An allocated, available ADC reading of zero is valid and is published.
Disabled, unallocated, unavailable, out-of-range, or more than 500 ms old
measurements are not published. The timeout measures time since the VBEC task
read the ADC; the existing ADC interface does not detect a DMA stream that
silently stops updating. Missing telemetry does not itself configure an alarm
on the radio.

Before considering a target validated:

1. Compare VBEC with a voltmeter at the monitored supply, and calibrate it.
2. Display flight-pack voltage and VBEC simultaneously on the radio.
3. Verify VBEC still works with main-battery monitoring disabled or its ESC
   source unavailable, without changing servo mappings.
4. With the FC powered independently, verify that removing the monitored
   supply produces a settled, valid zero; disabling VBEC instead stops updates.
5. Check channel conflicts and persistence across reboot, and verify existing
   battery/current/ADC settings survive the firmware update.

The initial implementation has host tests and build validation. NEXUS-XR meter
comparison and radio discovery are pending; no flight validation is claimed.

## Prior requests

INAV users have requested
[a second independent voltage/current readout](https://github.com/iNavFlight/inav/discussions/9679)
and monitoring of
[the separate supply powering flight controls and servos](https://github.com/iNavFlight/inav/issues/10613#issuecomment-3225434804).
Those requests concern OSD or generic ADC access. They establish the need for
independent supply monitoring, not acceptance of this particular CRSF mapping.
