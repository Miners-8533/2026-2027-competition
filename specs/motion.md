# Motion: actions and closed-loop moves

The building blocks the autos (and a few TeleOp features) use to move the chassis under control. All of them command the drive in [drive.md](drive.md) and read its pose.

## 1. Action framework

**Purpose.** A unit of work that advances a little on every control loop, so that driving, mechanisms and logging all keep running together. This is the team's own small replacement for the Road Runner action classes.

**Contract.** An action has one operation, *run one loop*, which returns whether it needs another loop. It also receives the loop's Dashboard telemetry packet to put debug values in.

| Action | Behavior |
|---|---|
| Sequential | runs its children one after another; when a child finishes, the next starts **in the same loop**, so a run of instant steps takes one loop |
| Parallel | runs every unfinished child each loop; finished when all are |
| Sleep | finishes after a fixed time, counted from its first loop |
| Instant | runs a piece of code once and finishes in the same loop |

**Runner.** An auto runs one action to completion. Each loop: run the action, then run the opmode's *every-loop* work (the mechanisms update), then send the telemetry packet, then give the run log its tick. The runner stops as soon as the opmode stops.

## 2. Drive to pose

**Purpose.** Drive to a field pose and stop there. Every auto leg is one of these.

**Control law.**

- Three independent PID controllers on the field-x error, the field-y error and the heading error (wrapped to the short way round).
- The field-frame translation command is rotated into the robot frame by the current heading and sent to the drive with the turn command.
- Each axis is clamped to a maximum power separately. Consequence: a diagonal move with both axes saturated starts as a 45 degree line, not a straight line to the target. Paths that must clear an obstacle add a via point.
- The gains are re-read every loop, so Dashboard edits act at once.

**Finish.** Done when position and heading have both been inside tolerance for the settle time, or when the time cap passes. The drive is stopped on finish and one log line says `settled` or `TIMED OUT` with the time, the final error and the **overshoot**: the furthest the robot went past the target along its line of travel, and past the target heading in the direction it was turning. The same figures can be read from the leg by the opmode that ran it.

**Options a leg may set.**

| Option | Effect |
|---|---|
| Timeout | a fixed cap for this leg |
| Distance timeout | cap = max(minimum, base + distance / speed), plus an extra for a turn over 120 deg, never above a maximum; worked out when the leg starts |
| Tolerance | a coarser position and heading tolerance, for pass-through points |
| Settle time | 0 ends the leg the moment it is inside tolerance |
| Maximum power | a lower per-axis clamp, for a slow leg |
| Slow zone | within a given distance of the target the translation power magnitude is limited to a given value |
| Stiction | adds the static friction term below |

**Static friction term (opt-in, added after Hartland, not run on the robot).** Without it a small error commands a power below the drive's dead zone and the robot does not move. When on, per robot axis and only while that axis is still outside tolerance (position error on that axis at least 0.3 in), a constant is added in the direction of the command: a *start* value while the axis is nearly still, a smaller *running* value once it moves. A zero command stays zero.

**Parameters.**

| Name | Value |
|---|---|
| Axial (field x) kP / kI / kD | 0.05 / 0 / 0.006 |
| Lateral (field y) kP / kI / kD | 0.06 / 0 / 0.008 |
| Heading kP / kI / kD (per radian) | 1.5 / 0 / 0.08 |
| Per-axis maximum power | 0.6 |
| Integral clamp | 0.25 |
| Position tolerance | 1.0 in |
| Heading tolerance | 2.0 deg |
| Settle time | 0.20 s |
| Default time cap | 4.0 s |
| Stiction running: forward / strafe / turn | 0.065 / 0.11 / 0.09 |
| Stiction start: forward / strafe / turn | 0.10 / 0.15 / 0.14 |
| Counts as still below | 1.5 in/s, 6 deg/s |

**Known limit.** These gains were never tuned on `ROBOT_1` with the tuning opmodes. Without the stiction term a leg stalls 1.5 to 3 in short of its target, so the autos use coarse tolerances and time caps to move on.

### 2.1 Target T2: tuned, and proven before anything is built on it

Not in the code. See "As built and target" in the [README](README.md).

**The weakness.** Every auto leg is a drive to pose, and drive to pose does not reach its targets on this robot. The autos hide that with coarse tolerances and by letting legs run to their time caps, so their timing and their positions both depend on how a leg happens to stall.

**What has been measured.** On the morning of the Hartland Qualifier (2026-11-07) the team ran Drive Characterization, not the tuners. That run measured the drive itself: speeds, braking, coasting and dead zones ([robot_design/chassis.md](../robot_design/chassis.md)). The simulator, the grid routes and the static friction values are built on it. The gains in the parameter table above are still the starting values of 2026-09-14. Until 2026-11-17 the tuning opmodes wrote no log, so no tuning session could have left a record.

**Requirement.**

1. A leg that is meant to settle does settle. The time cap is a safety net.
2. The control law in use is the one that was tuned: the static friction term is part of it, not an option.
3. A tuned value is in the code, with the date and the log it came from.
4. Every number in the acceptance test is measured and reported by the opmode that runs it. Nobody works them out by hand. **Built 2026-11-17, not run on the robot:** the Drive Acceptance opmode ([support-opmodes.md](support-opmodes.md) section 4.2.1).

**Acceptance test** (on `ROBOT_1`, run by the Drive Acceptance opmode, [support-opmodes.md](support-opmodes.md) section 4.2.1, with the battery at least 12.0 V at rest). The values are proposed and are to be fixed at the tuning session.

| Test, 10 runs each | Pass |
|---|---|
| 48 in forward and back | every run `settled`, none `TIMED OUT` |
| 48 in strafe and back | the same |
| A 90 deg and a 180 deg turn in place | the same |
| Final error of every run | within 1.0 in and 2.0 deg (the ordinary tolerance) |
| Overshoot | at most 1.5 in and 5 deg |
| Time to settle | at most 1.0 s more than the distance at the leg's speed |
| A 12 in move and a 5 deg turn (the stall case) | settled within 1.5 s |

**Design consequences.**

- The static friction term loses its switch once it has passed the test.
- Every leg is declared as one of two kinds. A **settling** leg must end inside its tolerance. A **timed** leg ends on its clock by intent (the ram, the wall-squaring phases).
- A `TIMED OUT` on a settling leg is a fault. It is logged as such and the event-day report counts them per run.
- The distance-based time cap keeps its formula, with its speed taken from the measured drive performance in [robot_design/chassis.md](../robot_design/chassis.md), not from an estimate.

**Waits on** a tuning session on the robot. Nothing in the autos is reworked before it passes ([autos.md](autos.md) section 9).

## 3. Aim at a cell

**Purpose.** In an auto, turn in place onto a hive cell just before a launch. The leg before it already ends at the aim heading; this removes the last degree or two.

**Behavior.**

- Uses the same turn law, launcher offset and far-cell extra as the TeleOp target lock ([aiming.md](aiming.md)), with no translation.
- Takes an aim trim in degrees (positive counterclockwise) for that launch, and optionally an explicit sideways extra that replaces the far-cell rule.
- **Breakaway:** while the error is outside the turn law's dead band, the commanded turn is smaller than the breakaway power and the robot is turning slower than the stall rate, the command is raised to the breakaway power. Once the robot moves, the normal law takes over.
- Done when the error has been within the aligned angle and the turn rate below the target lock's aligned rate for the hold time, or when the time cap passes. The launch goes ahead either way.
- One log line: `AIM aligned` or `AIM TIMED OUT`, the error, the trim, how many loops used the breakaway, and the pose.

| Name | Value |
|---|---|
| Aligned angle | 1.0 deg |
| Hold time | 0.15 s |
| Time cap | 0.6 s |
| Breakaway power | 0.14 (0 = off) |
| Stall rate | 6 deg/s |

## 4. Square on a wall

**Purpose.** Press the robot into a corner so its heading and position are physically known, after the garden pickup.

**Behavior.**

1. **Side wall.** Strafe left (robot frame) at the push power while holding the starting heading with a small turn limit, so the wall can still square the robot. End when the robot has stopped (below the stop speed for the stopped-hold time, after the minimum time) or at the maximum time.
2. **Front wall.** Drive forward at the forward power while still leaning into the side wall at the side-hold power. Same stop test with its own minimum and maximum times.

The pose is **not** reseeded. Each phase writes a log line with the pose before and after and its difference from the pose a flush robot should read, so odometry drift can be read from the logs.

| Name | Value |
|---|---|
| Push power (side) | 0.5 |
| Side phase minimum / maximum time | 0.5 / 1.4 s |
| Forward power / side-hold power | 0.4 / 0.2 |
| Forward phase minimum / maximum time | 0.25 / 0.8 s |
| Stopped: speed below / for | 1.0 in/s / 0.15 s |
| Heading hold kP / turn limit | 1.5 / 0.15 |

## 5. Jostle

**Purpose.** Shake a flower so it gives up its pollen. A sustained push works where a single ram does not.

**Behavior.** The **contact point** is where the robot is when the jostle starts, pressed against the flower. Every stroke is measured from that anchor, so the robot cannot creep.

1. **Hold:** keep pressing forward for the start-hold time.
2. **Back:** reverse at the back power until the robot is the back distance behind the contact point, or the back time limit passes.
3. **Push:** drive forward until it is back within the contact tolerance (after the minimum push time) or the push time limit passes.
4. **Press:** keep pressing for the press-hold time. That completes one cycle; go to step 2.

The heading is held at its starting value with a small turn limit. All motion is along the robot's own forward axis, so it is the same for either alliance.

**It ends when** the caller's "done" condition is true (the robot is full), or the cycle limit is reached, or enough elements are held and nothing new has arrived for the quiet time, or the time limit given by the caller passes.

Log lines record the start, each back stroke (commanded power, time, distance reached, whether it ended by distance or by the time limit), each push, and why it ended.

| Name | Value |
|---|---|
| Start hold | 1.0 s |
| Forward power | 0.5 |
| Back distance / back power / back time limit | 0.5 in / 0.4 / 0.3 s |
| Push minimum / maximum time | 0.2 / 0.6 s |
| Press hold | 1.0 s |
| Cycle limit | 1 (0 = no limit) |
| Contact tolerance | 0.3 in |
| Early stop: held at least / quiet for | 5 (never, since a robot holds 4) / 0.8 s |
| Heading hold kP / turn limit | 1.5 / 0.15 |

## 6. Hold position

**Purpose.** Keep the robot where it is while it launches, so the salvo leaves from the pose the aim and speed were computed for, and so a push from another robot is resisted. Used by the TeleOp; it is a per-loop helper, not an action.

**Inputs each loop:** the pose and measured velocity; *wanted* (a launch is queued or a salvo is running, or a Park Dash is at its corner); *cancel* (the driver is moving a stick, or something else is driving); *heading free* (the target lock is not turning the robot); *brake*.

**Output each loop:** whether it is active, a robot-frame forward and left power, and optionally a turn power.

**States.**

| State | Behavior | Leaves when |
|---|---|---|
| Idle | no output | *wanted* becomes true: go to Waiting |
| Waiting | no output, or the brake (below) | the robot has been slower than the latch speed for the latch time: latch the pose, go to Holding |
| Holding | PID on the field position error from the latched pose, plus a static term along the error direction outside the dead band; if *heading free*, a PD on the heading error outside its dead band | *wanted* false for longer than the tail time, or *cancel*, or the sanity limit |

**Rules.**

- *Cancel* releases it at once. If the launch is still going when the sticks are let go, it waits and latches again.
- **There is no give-up limit on the error.** It must fight being shoved. The only guard is the sanity distance: an error that large is a pose jump or a reseed, not a push, and the hold lets go.
- Because it works on the error relative to the latched pose, absolute odometry error does not matter.
- **Brake** (only when the caller asks): while Waiting and the robot is faster than the latch speed, command a power opposite to the measured velocity, proportional to it, up to a maximum. This removes the coast when a salvo is fired while driving.
- Log lines: `HOLD latched`, `HOLD released` with the reason, duration, peak error and peak command, and `HOLD dropped` on the sanity limit.

| Name | Value |
|---|---|
| Latch: slower than / for | 3.0 in/s / 0.12 s |
| Tail after the launch | 0.2 s |
| Position kP / kI / kD / static | 0.06 / 0.03 / 0.006 / 0.04 |
| Most power the integral adds, per axis | 0.2 |
| Position dead band | 0.4 in |
| Maximum translation power | 0.7 |
| Heading kP / kD / dead band / turn limit | 1.5 / 0.08 / 1.0 deg / 0.4 |
| Sanity distance | 30 in |
| Brake gain / maximum | 0.02 power per in/s / 0.6 |

**Status:** hold position was in the Hartland build. The brake was added after Hartland and has run in practice, but not at speed.
