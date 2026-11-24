# Intake and ramp

The two simple actuators. Both only do what they are told; when they are told is decided by the coordinator ([mechanisms.md](mechanisms.md)).

Hardware: [robot_design/mechanisms.md](../robot_design/mechanisms.md).

## 1. Intake

**Interface.** A mode: OFF, IN or OUT. It takes effect at once and is re-applied every loop. A power scale (0 to 1, default 1) multiplies both powers.

| Mode | Motor power |
|---|---|
| IN | 1.0 |
| OUT | -0.5 |
| OFF | 0 (brake) |

**Current watch.** Added after the intake failed mechanically in a run and the log showed nothing.

- While the intake is commanded, its motor current is read every 100 ms. While it is OFF nothing is read and the value is 0.
- **High current:** 6 A or more for 0.5 s while commanded is reported as a possible jam or stall.
- **Low current:** under 0.4 A for 1.0 s while commanded IN is reported as a roller that is not turning or not coupled.
- Each stretch is reported when it begins and when it ends (with its length and peak), or when the intake is switched off during it.
- The reports go to the run log only. **Nothing acts on them**: there is no automatic reversal. The thresholds are first guesses.

**Actions for autos:** set a mode; run a mode for a given time and then switch off.

| Name | Value |
|---|---|
| IN power / OUT power | 1.0 / -0.5 |
| Current refresh | 100 ms |
| High current: at least / for | 6.0 A / 0.5 s |
| Low current: below / for | 0.4 A / 1.0 s |

## 2. Ramp

**Interface.** Two commands, *deploy* and *stow*. Each sets both servos straight to their positions from the chassis profile, with no rate limiting.

**Rules.**

- With no feedback, **settled** means the travel time (0.8 s) has passed since the last command.
- If any of the four positions is not set in the chassis profile, the ramp never moves.
- **Built for an auto or a test opmode:** the stowed positions are commanded at INIT. The robot must start stowed to fit the starting cube.
- **Built for the competition TeleOp:** nothing is commanded at INIT. The servos sag when an auto ends and the ramp may be stuck or down, and rule G403 forbids powered movement in the transition between auto and TeleOp. The position is reported as unknown until the first command, which is a stow at Start.

**Clear of a flower.** A helper answers whether the robot's origin is farther than the clear distance from all four flower centers. The coordinator can use it to delay a stow until the robot has driven out from under a flower. The competition TeleOp stows at once and does not use it.

| Name | Value |
|---|---|
| Travel time | 0.8 s |
| Clear distance | 24 in (a placeholder; the real ramp reach was never measured) |
| Flower centers used by the check | (-68.6, -24.0), (-24.0, 68.6), (24.0, -68.6), (68.6, 24.0) |

These flower centers were read from the manual's figures and differ by a fraction of an inch from the CAD values the route planner uses ([field.md](field.md)).
