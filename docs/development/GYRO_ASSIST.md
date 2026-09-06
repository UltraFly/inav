# INAV Gyro Assist

Gyro Assist is an opt-in AUX modifier for fixed-wing MANUAL mode. The stick continues to command control-surface deflection directly while the gyro adds only a bounded disturbance correction. It is independent of Avian telemetry and TextGen.

The implementation adds a dedicated `GYRO ASSIST` mode-range entry, persistent `gyro_assist_*` settings, the MANUAL servo-mixer hook, debug fields, and a separate EdgeTX tool. The modifier is off by default. Its per-axis gains also default to zero, so an AUX assignment alone cannot produce a correction.

## Development branch boundary

`feature/gyro-assist` is one of three canonical project branches and is independent of the two Avian branches:

```text
feature/gyro-assist                 (this branch; independent)

feature/avian-esc-telemetry         (SRXL2 bus, throttle only, sensor 0x20)
└── feature/avian-esc-textgen       (full channels, reverse, sensor 0x0C, Lua tool)
```

This branch contains the Gyro Assist firmware and the independent `inav-gyro-assist` EdgeTX tool. It must not contain SRXL2 ESC transport, Avian telemetry, TextGen, thrust reverse, or `inav-avian-esc`. Conversely, neither Avian branch may contain Gyro Assist or `inav-gyro-assist`. The intended first upstream merge request is built from this branch alone.

## Control law

All commands inside the kernel use the logical normalized mixer range `[-1, 1]`. For each roll, pitch, and yaw axis:

1. Clamp the pilot command to `[-1, 1]`.
2. Low-pass filter the measured body-axis gyro rate in degrees per second. A zero cutoff bypasses the filter.
3. Convert absolute stick deflection and the configured per-axis Priority into a target gain scale.
4. Shape movement toward that target with separate Stop Release and Stop Lock time constants.
5. Calculate negative rate feedback: `raw correction = -filtered gyro rate * gyro gain * priority gain scale`.
6. Limit the correction to the fixed configured per-axis correction limit.
7. Add the correction to the pilot command and clamp the result to `[-1, 1]`.
8. Cross-fade between the existing controller command and the Gyro Assist command during normal entry and exit.

Stick movement changes effective gyro gain, not the configured correction limit. Priority `0` leaves gain independent of the stick. Priority `100` decreases gain linearly from 100% at center to zero at full stick. Above `100`, the zero-gain point moves inward; for example, Priority `140` reaches zero gain at 60% stick and remains zero beyond it. This follows the public Spektrum AS3X+ Priority examples without assuming its undisclosed internal implementation.

Stop Release is the response when stick movement asks for less gain; Stop Lock is the response when recentering asks for gain to return. Each is an explicit PT1 time constant in milliseconds: `0` follows the Priority target immediately, and a larger value makes that direction slower. These millisecond values are intentionally not claimed to be numerically equivalent to Spektrum's proprietary 1–100 Rate values.

The correction limit remains a fixed independent safety bound, and final command saturation remains independent of both Priority and shaping. With the default Priority of `100`, sustained full stick commands zero effective gyro gain. A non-zero Stop Release time deliberately allows some damping to remain briefly during a quick input, so initial testing must verify that this does not noticeably impede pilot commands.

This is rate damping, not rate command. Pilot stick position is never converted to a desired angular rate and is not passed through PIFF. The gyro term reacts to measured rotation only; it does not integrate attitude or try to hold an angle. Heading hold, self-leveling, envelope protection, and SAFE-like recovery are explicitly out of scope and remain the responsibility of INAV's existing assisted modes.

## Existing-controller fallback and transitions

The caller supplies both the direct pilot command and the command that INAV would have used without this feature. The latter is called the fallback command. The final output is:

`fallback + blend * (Gyro Assist command - fallback)`

On entry, blend starts at zero, so the first output exactly matches the existing controller. It then rises to one over the configured transition time. A deliberate exit starts at the current blend and fades back to the live fallback command. Re-entering during an exit reverses the fade from its current value rather than restarting it.

The mixer derives the fallback before invoking this controller:

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

## Configuration and mode integration

`GYRO ASSIST` is exposed as an off-by-default fixed-wing AUX modifier. The following master settings are available through the CLI, Configurator's generic settings interface, and generic MSPv2 setting commands:

| Setting | Units | Range | Default |
| --- | --- | ---: | ---: |
| `gyro_assist_roll_gain` | % full correction at 100 deg/s | 0–100 | 0 |
| `gyro_assist_pitch_gain` | % full correction at 100 deg/s | 0–100 | 0 |
| `gyro_assist_yaw_gain` | % full correction at 100 deg/s | 0–100 | 0 |
| `gyro_assist_roll_priority` | stick-priority strength | 0–200 | 100 |
| `gyro_assist_pitch_priority` | stick-priority strength | 0–200 | 100 |
| `gyro_assist_yaw_priority` | stick-priority strength | 0–200 | 100 |
| `gyro_assist_roll_stop_release_ms` | roll gain-reduction PT1 time | 0–2000 ms | 50 |
| `gyro_assist_pitch_stop_release_ms` | pitch gain-reduction PT1 time | 0–2000 ms | 50 |
| `gyro_assist_yaw_stop_release_ms` | yaw gain-reduction PT1 time | 0–2000 ms | 50 |
| `gyro_assist_roll_stop_lock_ms` | roll gain-return PT1 time | 0–2000 ms | 250 |
| `gyro_assist_pitch_stop_lock_ms` | pitch gain-return PT1 time | 0–2000 ms | 250 |
| `gyro_assist_yaw_stop_lock_ms` | yaw gain-return PT1 time | 0–2000 ms | 250 |
| `gyro_assist_roll_limit` | % full mixer command | 0–50 | 20 |
| `gyro_assist_pitch_limit` | % full mixer command | 0–50 | 20 |
| `gyro_assist_yaw_limit` | % full mixer command | 0–50 | 20 |
| `gyro_assist_lpf_hz` | Hz; 0 bypasses the extra filter | 0–100 | 20 |
| `gyro_assist_transition_ms` | ms | 20–2000 | 250 |

Settings use stable integer scaling on the wire. For example, a roll gain of 10 means 10% of full mixer correction at a measured roll rate of 100 deg/s, before stick-priority scaling and the fixed correction limit. Firmware validates the stored controller configuration before using it.

The modifier changes only `INPUT_STABILIZED_ROLL`, `INPUT_STABILIZED_PITCH`, and `INPUT_STABILIZED_YAW` after the normal MANUAL/fallback inputs have been selected. With the AUX modifier inactive and no exit transition in progress, the controller is not called and the existing MANUAL assignments pass through exactly. The PIFF controller, navigation controller, motor mixer, and throttle path are unchanged.

Set `debug_mode = GYRO_ASSIST` to record these debug values:

| Index | Value |
| ---: | --- |
| 0 | roll pilot command, mixer units |
| 1 | roll correction, mixer units |
| 2 | roll final command, mixer units |
| 3 | pitch pilot command, mixer units |
| 4 | pitch correction, mixer units |
| 5 | pitch final command, mixer units |
| 6 | yaw correction, mixer units |
| 7 | phase × 1000 + inhibit reason |

The conservative first live policy should allow only the dedicated fixed-wing pilot-control context and exclude Angle, Horizon, Angle Hold, Turn Assistant, Autotune, navigation, launch, autoland, failsafe, and servo autotrim until each interaction has its own transition test. Enabling an excluded mode should select that subsystem's already-computed command as fallback and begin a controlled exit.

Initial non-zero gains must come from simulator and propeller-removed bench work. A gain that is too high or a filter that is too fast can excite servo or airframe modes even though the correction is bounded.

## Host coverage

The native tests cover:

- valid and invalid configuration;
- neutral stick and still-air output;
- positive and negative correction polarity;
- gust response and per-axis correction limiting;
- per-axis Priority, full-stick gain removal, and the above-100 zero-gain point;
- independent Stop Release and Stop Lock shaping;
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

## First NEXUSX bench test

Back up the current configuration before flashing a development image. A motor is not required for this test; disconnect it or remove the propeller and otherwise make the power system safe.

1. Flash the matching NEXUSX development `.hex` with full chip erase, reconnect, and restore only configuration appropriate to that firmware build.
2. Confirm that `GYRO ASSIST` appears in Configurator's **Modes** tab. Assign it to a dedicated switch range while keeping MANUAL as the primary flight mode.
3. Leave all three gains at their zero defaults. Arm only if the receiver/servo bench setup is safe, select MANUAL, and toggle `GYRO ASSIST`. Servo output must be indistinguishable from ordinary MANUAL because zero gain produces no correction.
4. Disarm. Start with one axis only; for example set roll gain to `5`, roll limit to `10`, low-pass to `20`, and transition time to `250`. Keep the other gains at zero.
5. With MANUAL and the modifier selected, move the airframe briskly around that axis. The surface response must oppose the measured rotation. If it reinforces the motion, do not arm or fly; correct the FC orientation, mixer, or servo setup first rather than attempting to use a negative gain.
6. Confirm that effective correction gain reduces as the stick approaches full travel and, with Priority `100`, becomes zero at sustained full stick. Check quick inputs separately because Stop Release intentionally shapes rather than instantaneously removes gain.
7. Toggle the modifier, MANUAL, failsafe, and disarm paths while watching outputs. Normal entry/exit should blend; disarm and failsafe should immediately discard correction.
8. Repeat one axis at a time. Do not begin flight testing until every physical surface—including elevon, V-tail, differential, and reversed outputs—has the correct response across its full range.

For CLI-based setup, the initial roll-only example is:

```text
set gyro_assist_roll_gain = 5
set gyro_assist_pitch_gain = 0
set gyro_assist_yaw_gain = 0
set gyro_assist_roll_priority = 100
set gyro_assist_roll_stop_release_ms = 50
set gyro_assist_roll_stop_lock_ms = 250
set gyro_assist_roll_limit = 10
set gyro_assist_lpf_hz = 20
set gyro_assist_transition_ms = 250
save
```

These are conservative bench-start values, not flight-tested recommendations. The Lua tool can edit the same settings through generic MSPv2 while disarmed. AUX assignment remains in Configurator's Modes workflow.

## Relevant INAV paths

- `src/main/fc/fc_core.c`: creates pilot `rcCommand` values and applies failsafe input.
- `src/main/flight/pid.c`: converts stabilized stick input to rate targets and runs PIFF today.
- `src/main/flight/servos.c`: selects `rcCommand` in Manual mode or `axisPID` in assisted modes, then applies the servo mixer and physical limits.
- `src/main/flight/fw_gyro_assist.c`: transport-independent controller and transition kernel added by this branch.
- `src/main/flight/fw_gyro_assist_config.c`: persistent integer-scaled settings and safe defaults.
- `src/utils/edgetx/inav-gyro-assist/`: generic MSPv2 settings tool for EdgeTX 2.11.0 and newer.
