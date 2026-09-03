# Fixed-wing surface-deflection stabilization

Surface-deflection stabilization is an opt-in fixed-wing controller for pilots who want the stick to command control-surface position directly while the gyro adds only a bounded disturbance correction. It is independent of Avian telemetry and TextGen.

This first increment implements and tests the controller kernel. It is deliberately not selectable in flight yet: it adds no mode ID, persistent setting, mixer hook, or default behavior. That keeps the existing PIFF/rate controller unchanged while the controller contract and safety behavior are reviewed.

## Control law

All commands inside the kernel use the logical normalized mixer range `[-1, 1]`. For each roll, pitch, and yaw axis:

1. Clamp the pilot command to `[-1, 1]`.
2. Low-pass filter the measured body-axis gyro rate in degrees per second. A zero cutoff bypasses the filter.
3. Calculate negative rate feedback: `raw correction = -filtered gyro rate * gyro gain`.
4. Limit the correction to the configured per-axis correction limit.
5. Limit it again to the pilot's remaining command headroom: `1 - abs(pilot command)`.
6. Add the correction to the pilot command.
7. Cross-fade between the existing controller command and the surface-deflection command during normal entry and exit.

The headroom limit is intentional. At neutral stick, the configured gyro correction is available. As the pilot moves toward full throw, less correction is allowed, and at full stick the correction is exactly zero. Gyro feedback therefore cannot take commanded throw away from the pilot or push the logical command beyond its normal range.

This is rate damping, not rate command. Pilot stick position is never converted to a desired angular rate and is not passed through PIFF. The gyro term reacts to measured rotation only; it does not integrate attitude or try to hold an angle.

## Existing-controller fallback and transitions

The caller supplies both the direct pilot command and the command that INAV would have used without this feature. The latter is called the fallback command. The final output is:

`fallback + blend * (surface deflection command - fallback)`

On entry, blend starts at zero, so the first output exactly matches the existing controller. It then rises to one over the configured transition time. A deliberate exit starts at the current blend and fades back to the live fallback command. Re-entering during an exit reverses the fade from its current value rather than restarting it.

The eventual mixer integration should derive the fallback before invoking this controller:

- manual fixed-wing fallback: current `rcCommand` behavior;
- stabilized fixed-wing fallback: current `axisPID` behavior;
- launch, navigation, failsafe, and any unsupported mode: the command already produced by that subsystem.

After conversion back to INAV mixer units, the result replaces only `INPUT_STABILIZED_ROLL`, `INPUT_STABILIZED_PITCH`, and `INPUT_STABILIZED_YAW`. Existing servo mixer weights, servo reversal, midpoint, rate, speed limiting, low-pass filtering, and min/max limits remain downstream and unchanged.

## Eligibility and safety policy

The kernel accepts explicit conditions rather than reading global flight state. This makes the policy host-testable and prevents a protocol or UI feature from silently enabling it.

| Condition | Behavior |
| --- | --- |
| Feature not requested | Remain inactive, or cross-fade out if already engaged |
| Fixed-wing pilot-control mode allowed | May enter when every safety condition is valid |
| Navigation active | Controlled exit to the navigation fallback |
| Launch active | Controlled exit to the launch fallback |
| Servo autotrim active | Controlled exit before autotrim operates |
| Unsupported mode | Controlled exit to the existing controller |
| Disarmed | Immediately discard correction and use fallback |
| Failsafe | Immediately discard correction and use failsafe fallback |
| Not a fixed wing | Immediately discard correction and use fallback |
| Invalid or stale gyro | Immediately discard correction and use fallback |
| Invalid input, timing, or configuration | Immediately discard correction and use a finite fallback, or zero if the fallback itself is invalid |

Normal mode changes cross-fade because both command sources are valid. Safety and data-validity faults do not continue using an old correction merely to make the transition smooth. A safety fallback can therefore contain a step if the two controllers disagreed at the instant of failure; that tradeoff is explicit.

When the feature is dormant, unrelated gyro or configuration faults are ignored and the fallback passes through. Once entry or exit has begun, validation remains mandatory until the existing controller has full authority again.

## Reversal and saturation

The kernel operates in INAV's logical roll, pitch, and yaw coordinate system. It must run before servo mixer weights and per-servo reversal. Reversing a servo or using a negative mixer weight therefore reverses the complete pilot-plus-correction command together. Applying reversal to the gyro term alone would change the feedback polarity and is not permitted.

The kernel clamps pilot, correction, surface, fallback, cross-fade, and final logical commands. The existing mixer must still apply each servo's configured output limits because multiple mixer rules can sum into one physical output.

## Proposed configuration and mode integration

Public settings and a permanent mode identifier should be selected with upstream maintainers rather than invented in this isolated kernel. A later integration patch should provide, at minimum:

- an explicit off-by-default selection or AUX mode;
- per-axis gyro gain;
- per-axis maximum correction;
- gyro low-pass cutoff;
- controller cross-fade time;
- gyro freshness timeout;
- debug/blackbox fields for pilot command, filtered gyro, correction, blend, final command, phase, and inhibit reason.

The conservative first live policy should allow only the dedicated fixed-wing pilot-control context and exclude Angle, Horizon, Angle Hold, Turn Assistant, Autotune, navigation, launch, autoland, failsafe, and servo autotrim until each interaction has its own transition test. Enabling an excluded mode should select that subsystem's already-computed command as fallback and begin a controlled exit.

Suggested initial values must come from simulator and propeller-removed bench work; the kernel deliberately provides no flight-ready gain defaults. A gain that is too high or a filter that is too fast can excite servo or airframe modes even though the correction is bounded.

## Host coverage

The native tests cover:

- valid and invalid configuration;
- neutral stick and still-air output;
- positive and negative correction polarity;
- gust response and per-axis correction limiting;
- full-stick pilot authority and near-full-stick headroom;
- normalized final-output saturation;
- bumpless entry, exit, and exit reversal;
- disarm, failsafe, invalid gyro, stale gyro, and timer wrap;
- navigation, launch, autotrim, and unsupported-mode exit;
- invalid timing and non-finite inputs;
- downstream mixer reversal of the complete logical command;
- low-pass attenuation of a gyro step.

## Hardware-deferred validation

Before flight, use the simulator and then a restrained, propeller-removed airframe to confirm:

- gyro sign and surface response for every mixed physical surface;
- all servo reversals, differential, elevon/V-tail mixing, and asymmetric endpoints;
- transition behavior when the pilot moves the stick during entry and exit;
- behavior on arm, disarm, receiver failsafe, FC reboot, gyro fault, and stale samples;
- handoff to Angle/Horizon if supported later, navigation, launch, autoland, and autotrim;
- correction limits at neutral, partial, and full stick;
- filter and servo resonance margins across expected airspeed;
- blackbox evidence that the gyro term opposes the measured disturbance and returns to zero.

Flight testing should begin with low gains and small correction limits, one axis at a time, with an immediate switch back to the unchanged controller.

## Relevant INAV paths

- `src/main/fc/fc_core.c`: creates pilot `rcCommand` values and applies failsafe input.
- `src/main/flight/pid.c`: converts stabilized stick input to rate targets and runs PIFF today.
- `src/main/flight/servos.c`: selects `rcCommand` in Manual mode or `axisPID` in assisted modes, then applies the servo mixer and physical limits.
- `src/main/flight/fw_surface_deflection.c`: transport-independent controller and transition kernel added by this branch.
