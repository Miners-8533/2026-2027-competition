# Flywheel and launch speed

The launcher: one flywheel, its position servo, and the table that turns distance into speed. When a salvo may start, and which speed is asked for, is decided by the coordinator ([mechanisms.md](mechanisms.md)).

Hardware: [robot_design/mechanisms.md](../robot_design/mechanisms.md).

## 1. Flywheel speed control

**Interface.** Set a target speed in encoder ticks per second (0 or less means off). Update once per loop. Read: the speed, a filtered speed, *ready*, *in the launch band*, the last motor command, the motor current.

**Control law (bang-bang with a feedforward).** Each loop, with one fresh speed reading:

| Condition | Motor command |
|---|---|
| Target is 0 | 0 |
| Speed at or below the target | full power (1.0) |
| Speed above the target | the feedforward power |

- The feedforward must not exceed the power that holds the slowest launch speed, or the wheel creeps above its setpoint.
- The wheel is never braked. When the target drops, it only coasts down.
- **It does not wait for the speed to recover between elements.** Readiness is judged once, before the first element of a salvo; the pusher sets the spacing of the rest.
- The motor is written only when the command changes. The motor current is refreshed every 250 ms for display and logging.

**Filtered speed.** The mean of the last 5 loop readings. Readiness and the launch band use it.

| Test | True when |
|---|---|
| Ready | target above 0 and filtered speed at least 95 percent of it |
| In the launch band | target above 0 and filtered speed from 96 to 104 percent of it |

The upper limit of the band matters because a wheel still coasting down from a higher setpoint launches long.

## 2. Speed sensor failure

**Detection.** The encoder is judged lost when the motor has been commanded at 0.9 or more for 1.0 s while the speed never reached 50 ticks/s. Any reading of 50 or more restarts the window, so a slow spin-up never trips it. A reversed encoder (negative speed) trips it the same way.

**Lost mode.** The wheel runs open loop at a fixed power:

> power = base power + (target - base speed) / top speed

with base power 0.40 for base speed 1350 ticks/s and top speed 2200 ticks/s. There is no battery compensation, because the voltage reading is noisy and sags with the other motors. The drivers correct with the speed trim. *Ready* becomes a timer: 2.0 s after the switch. *In the launch band* equals *ready*.

**Recovery (added after Hartland, not run on the robot).** A hub that stops answering for a moment looks the same as a dead encoder. While in lost mode the speed is still read; when it has stayed at 50 percent of the target or more for 1.0 s, closed-loop control resumes. This happens at most 2 times per run, after which lost mode stays for the rest of the run.

**Manual open loop.** An older switch runs the wheel at power = target / top speed and treats it as ready 1.5 s after it is turned on. No control is bound to it at present.

**Reports.** The flywheel says once when it enters lost mode (with the numbers that tripped it) and once when it recovers; the coordinator logs both and raises a screen alert.

## 3. Parameters

| Name | Value |
|---|---|
| Feedforward power | 0.35 |
| Ready fraction | 0.95 |
| Launch band | 0.96 to 1.04 of the target |
| Filter length | 5 readings |
| Encoder sign | +1 |
| Lost detection: command at least / for / speed below | 0.9 / 1.0 s / 50 ticks/s |
| Lost mode: base power / base speed / ready after | 0.40 / 1350 ticks/s / 2.0 s |
| Recovery: speed at least / for / at most | 50 percent of target / 1.0 s / 2 times |
| Open loop top speed / ready after | 2200 ticks/s / 1.5 s |
| Current refresh | 250 ms |

## 4. Flywheel position servo

**Purpose.** Hold the flywheel in the position the launch speeds were measured at.

- Two named positions, POLLEN and NECTAR. **This branch only ever commands POLLEN.** There is no switch, toggle or automatic change.
- Nothing is commanded when the object is built: the servo's position is unknown until the first command.
- Who commands POLLEN first:
  - the autos and the tuning opmodes, when their mechanisms are built at INIT;
  - the competition TeleOp, at Start (nothing may move in its INIT, rule G403);
  - as a safety net, the first mechanisms update if nothing has.
- With no feedback, "in place" is a timer from the last command: 0.10 s for a move to POLLEN, 0.25 s for a move to NECTAR, and the longer one for the first command. A launch waits for it.
- If the servo is not configured, the robot runs without it.

| Name | Value |
|---|---|
| POLLEN position | 0.45 |
| NECTAR position | 0.30 |
| Travel to POLLEN / to NECTAR | 0.10 / 0.25 s |

**Status:** added after Hartland; the hold has not run on the robot. The switching between positions exists only on the `flywheel-servo-prototype` branch and is out of scope here.

## 5. Launch map (speed from distance)

**Purpose.** The flywheel speed for a launch from a given distance to the hive cell. The hood is fixed, so distance is the only input.

**Rule.** Linear interpolation between measured points; the first and last speeds are held outside the table.

| Distance to the cell (in) | Speed (ticks/s) |
|---|---|
| 28.6 | 1325 |
| 32.0 | 1350 |
| 35.8 | 1350 |
| 42.3 | 1350 |
| 46.7 | 1375 |
| 62.0 | 1450 |
| 82.2 | 1650 |

Measured 2026-11-01 with a charged battery and real pollen, four-element salvos, at one cell. The speed depends only on distance, so the table serves all four cells. Distance is from the robot's origin to the cell point.

- The operator's speed trim is added on top of the table's speed.
- If the table were empty, the coordinator would fall back to a manual setpoint of 1350 ticks/s.

A two-dimensional alternative (speed and aim per field position) exists behind a switch that is off; see [aiming.md](aiming.md).
