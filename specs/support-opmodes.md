# Support opmodes

Opmodes that are not run in matches: checks, tuners, studies and build aids. Each exists to produce a number, a table or a verdict that goes back into the code or into [robot_design/](../robot_design/README.md). They are specified by purpose, procedure and output, not control by control.

**Visibility.** An opmode can be hidden from the Driver Station list without removing its code. Hidden opmodes are brought back by removing one marker line and reinstalling. The list is kept short for competition.

| Opmode | Group | Shown | Log kind |
|---|---|---|---|
| Pit Check (robot on blocks) | Bring-up | hidden | pitcheck |
| Launch Tuner | Bring-up | yes | bringup |
| Launch Grid Tuner | Bring-up | yes | bringup |
| Launch Salvo Speed Study | Bring-up | yes | bringup |
| CS Pass Study | Bring-up | yes | bringup |
| Flywheel Step Study | Bring-up | yes | bringup |
| Color Sensor Study | Bring-up | hidden | bringup |
| Color Sensors Test (CS1-CS4) | Bring-up | hidden | none (device log) |
| Launcher Offset Finder | Bring-up | hidden | bringup |
| Pinpoint Push Test | Bring-up | hidden | bringup |
| Pinpoint Offset Check | Bring-up | hidden | bringup |
| Telemetry Font Test | Bring-up | hidden | none |
| Auto Waypoint Marker | Auto build | hidden | waypoints |
| Drive Acceptance | Tuning | yes | bringup |
| Tune - Axial, Lateral, Heading | Tuning | hidden | bringup |
| Drive Characterization | Tuning | hidden | drivetest |
| Auton Verify - Forward 48 and back, Lateral 48 and back | Auto verify | hidden | none |

**Common rules.** No opmode uses Start or Back for a function. Anything that moves waits for a button. Opmodes that seed the pose need the robot on the start spot and still at INIT.

## 1. Pit Check

**Purpose.** The one check to run after a repair, a rewiring or a hub swap, and before a match day. The running code does not watch for wiring and sign faults, so this is where they are found.

**Setup.** Robot on blocks, wheels clear, ramp free to move, track empty. Nothing moves until a button is pressed for that step; every move is short and slow.

**Controls.** One button runs a step or confirms "what I saw was right"; one says "wrong"; one skips; one stops everything.

| Step | Checked | Judged by |
|---|---|---|
| Configuration | every required device name and type; a stale second flywheel motor; the hub and port of each device; configured devices the code does not use | automatic |
| Battery | at least 12.0 V on every hub | automatic |
| Pinpoint | status READY and a sane device loop time | automatic |
| CS1 to CS4 | each answers and matches its stored empty reading | automatic |
| Flywheel | spins at low power: the encoder reads the right way and fast enough | encoder automatic; launching direction by eye |
| Pusher | turns about 60 deg forward and back: encoder sign, and return to the start | sign and return automatic; pushing direction by eye |
| Intake | pulses in, then out; the current is shown | by eye |
| Ramp | deploys and stows | by eye |
| Each drive wheel | one wheel at a time at low power | by eye: only that wheel turns, its top toward the front |

**Output.** Each failure names the fix. The result line is ALL PASS, FAILED or INCOMPLETE with the counts. A skipped step is not a pass. The ports line is the source for [robot_design/hub-configuration.md](../robot_design/hub-configuration.md).

**Limits.** It drives the raw hardware, not the subsystem classes, so it does not disturb the pusher's saved zero; it returns the arm to where it started and reports if it cannot. It does not check CS5 or the flywheel position servo. It does not test behavior that needs real elements.

## 2. Launch tuning

### 2.1 Launch Tuner

**Purpose.** Re-measure the launch map (speed from distance) after a flywheel change or a mechanical adjustment.

**Procedure.** Drive as in the TeleOp, with the intake always in. The cell follows the pose. The speed is the saved map's speed for the distance plus a **speed trim**; the aim is the target lock plus an **aim trim**. Fire, adjust the trims until the salvo scores, then **record** the point (distance, speed, both trims, pose, cell, battery). Recording resets both trims, and each point keeps its own. **Undo** removes the last point and restores the trims. The speed base can be switched between the saved map and the points recorded this session.

**Output.** One log line per salvo (launch speed, lowest speed and dip, battery, flywheel current, distance, pose, lock error, trims), and a table line ready to paste into the launch map.

**Controls** (from `design/opmodes.md`; not checked against the code).

| Who | Control | Effect |
|---|---|---|
| Driver | right bumper | target lock on / off |
| Driver | D-pad up / down | speed trim, 20 ticks/s a step |
| Driver | D-pad right / left | aim trim, 0.5 deg a step |
| Driver | A | fire |
| Operator | left trigger | intake OUT |
| Operator | D-pad | the TeleOp's landed low / high / left / right reports |
| Operator | A / B | record / undo |
| Operator | Y | fire |
| Operator | left bumper | speed base: the points recorded this session |

Log lines: `TUNER SALVO`, `TRIM`, `RECORD`, `UNDO`, `TUNER TABLE`.

### 2.2 Launch Grid Tuner

**Purpose.** Collect the data for the two-dimensional launch grid ([aiming.md](aiming.md)): aim and speed per field position for one hive cell.

**Procedure.** Choose the near or far cell at INIT; the alliance is assumed blue. While a trigger is held the robot drives itself to the next spot of the grid, already pointed at the cell with today's aim and at the launch map's speed. The operator loads, fires, and trims the angle (the robot re-aims by itself) and the speed until the salvo scores, then accepts the spot, or marks it as "no shot possible", or goes back one spot.

- The route to each spot comes from the dash planner; where the planner refuses a spot beside a flower stand, a short straight hop is used if it is clear, otherwise the operator drives by hand.
- Small re-aims use *drive to pose* with its static friction term, and each reports whether it turned or stuck.
- A sweep can be continued from a later spot, restricted to a list of spot numbers, or extended with extra positions, by settings read at INIT.

**Output.** One line per spot (speed split into map and trim, aim heading split likewise, distance, salvos fired, pose; a good spot with no salvo is marked unverified), a line per salvo, and a table at the end. A PC tool turns the log into the generated grid table.

**Controls** (from `design/opmodes.md`; not checked against the code). At INIT: NEAR is the audience cell, FAR the rear cell; the start is the usual audience-wall start. Hold the right trigger to drive to the next spot. D-pad left / right trims the angle by 0.5 deg, up / down the speed by 20 ticks/s. Y fires. A accepts the spot, B marks it as no shot, X goes back one spot. Log lines: `GRID POINT`, `FIRE`, `GRID TABLE`.

### 2.3 Launcher Offset Finder

**Purpose.** Measure the launcher's sideways offset, and separate it from any error in where the hive's cells are.

**Procedure.** No target lock. Drive by hand to a wall, line the launcher up with a cell's centerline by eye, launch, and **mark** a salvo as good, or as landed left or right. Repeat for all four cells, ideally at two distances each, reseeding the pose after long drives.

**Output.** Marks in the log with the pose at each launch. A PC tool computes, per cell, how far the cell center sat from the robot's front line; the offset is the mean of a hive's two cells, the hive error half their difference, and the trend with distance is an angle error.

## 3. Studies

Each study was built to answer one question from logged data.

| Study | Question | Method | Result so far |
|---|---|---|---|
| Launch Salvo Speed Study | How fast should the pusher turn a salvo? | A copy of the competition TeleOp in which the driver's D-pad changes the pusher's power live | 0.37 turned a salvo in 1.0 s against 1.2 s at 0.30 with no change in flywheel dip; the team chose 0.30 |
| Flywheel Step Study | How fast does the flywheel follow a setpoint change? | The flywheel alone steps through a list of setpoints; each change is timed until the speed is in the launch band and has stayed there | not yet run on the robot |
| CS Pass Study | Can CS5 count entering elements without slowing the loop? | The real mechanisms with the intake in; CS5 read on its own thread; the operator presses a button per element fed as ground truth | yes at a 20 to 25 ms period with a small read; this led to the entry counter |
| Color Sensor Study | What do CS1 to CS4 read after the track or lighting changes? | Four guided steps: empty baseline, an empty pusher sweep, a full baseline with 4 pollen, a slow reject | prints the new empty readings to paste into the element counter |

The Launch Salvo Speed Study is a **fork**: later changes to the competition TeleOp are not carried into it.

### 3.1 Target T6: the match loop exists once

Not in the code. See "As built and target" in the [README](README.md).

**The weakness.** The study is a copy of the competition TeleOp, about 830 lines, made to change one number from the gamepad. The TeleOp has moved on since (the FULL AUTO or COUNT ONLY choice, FULL arming the auto-launch, the launch grid switch, the flywheel position at Start, stopping all mechanisms at the end), so the study no longer measures the robot that plays matches, and nothing says so when it runs.

**Requirement.** The match loop is written once. No opmode copies it.

**Design.**

- **The Launch Salvo Speed Study is retired.** Its question is answered, and the number it varied, the pusher's salvo power, is a live tunable value: the same experiment is the competition TeleOp with that value edited in the Dashboard, and every run logs the value in force ([logging.md](logging.md)).
- A later study that needs match behavior with a variation is the competition TeleOp started with a **study option** chosen at INIT. The option may add bindings only on controls the TeleOp leaves free, add log columns, and change tunable values. It may not change the loop. The log's `SETUP` line names the option, so no study run can be mistaken for a match run.
- A study that needs only part of the robot (the Flywheel Step Study, the CS Pass Study, the Color Sensor Study) stays its own small opmode built on the subsystems, as now.

**What would behave differently.** Nothing in a match. One opmode leaves the Driver Station list.

**Color Sensors Test** is an older tool that measures the empty baseline and scores the classification thresholds against hand-set labels; the Color Sensor Study replaced it for baselines.

### 3.2 Procedure details of the studies

(from `design/opmodes.md`; not checked against the code.)

| Study | Details |
|---|---|
| Flywheel Step Study | Setpoints in ticks/s: 1350, 1650, 1350, 1450, 1350, 1375, 1325, 1650, 1450, 1650, 1325, the whole list repeated. The loop is padded to 28 ms, like the TeleOp's. A runs the sequence, B stops it; with it stopped the D-pad steps the setpoint by 100 or 25. One `STEP` line per change and a `STEP TABLE` at the end; a step that does not settle within its limit says `NEVER SETTLED`. **Not specified:** the settle time and the limit |
| CS Pass Study | Intake IN throughout, flywheel at 500 ticks/s, loop padded to 28 ms. CS5 on its own thread every 20 ms, one 15-byte burst read per cycle. B = one element fed (a `FED n` line, the ground truth); Y salvo; X automatic clear; A mark; D-pad speed. A `CS5 THREAD` line every 5 s |
| Color Sensor Study, Color Sensors Test | **Not specified:** the controls |

Also **not specified:** the controls of the Launcher Offset Finder, the Pinpoint Push Test and the Pinpoint Offset Check, which buttons Pit Check uses for its four functions, and which button is the tuners' kill switch.

## 4. Drive and odometry

### 4.1 Pinpoint Push Test and Pinpoint Offset Check

For when a pod or the Pinpoint is moved or rewired.

- **Push Test:** push the robot forward, sideways and around by hand; the opmode checks the sign of each axis and so the pod directions.
- **Offset Check:** spin the robot in place several turns; the wobble of the reported position and the per-axis error estimates give the pod offsets to put in the chassis profile.

### 4.2 Tune - Axial, Lateral, Heading; Auton Verify

- **Tuners:** each drives a repeating pattern (48 in forward and back, 48 in strafe, a turn pattern) with *drive to pose* so its gains can be adjusted live in the Dashboard. One button is a kill switch.
- **Verify autos:** drive 48 in and back, forward or sideways, to confirm the tuned gains by the final error.

These have **not** been used to tune `ROBOT_1`; the drive-to-pose gains are still starting values.

**Logging (built 2026-11-17, not run on the robot).** The three tuners write a run log, kind `bringup`: the gains at the start and again whenever a Dashboard edit has changed them (`TUNE gains` lines), every leg's `DRIVE` lines with its time, final error and overshoot, and a row every 50 ms with the pose and the command sent. The verify autos still write no log. They stay for what they are good at: comparing the Pinpoint's answer with a tape mark on the floor.

### 4.2.1 Drive Acceptance

**Status:** built 2026-11-17 for target T2 ([motion.md](motion.md) section 2.1). It compiles and has not run on the robot. It is shown on the Driver Station.

**Purpose.** Drive the acceptance test of *drive to pose* and report every number of it, so a tuning session ends with a verdict and a record. The tuners find the gains; this opmode judges them.

**Setup.** Any clear floor. The start is (0, 0, 0 deg) wherever the robot stands at INIT, and it must stand still from INIT to Start. It needs 48 in clear ahead and to its left, with a margin, and room to spin. No mechanism moves.

**Tests.** Each is one move driven 10 times, out and back in turn. Every run is one leg with the static friction term on, and is measured.

| # | Move | Time allowed |
|---|---|---|
| 1 | 48 in forward | 3.2 s (48 in at 22 in/s, plus 1.0 s) |
| 2 | 48 in strafe, to the robot's left | 3.8 s (a strafe is taken as 0.78 of that speed) |
| 3 | turn 90 deg | 1.75 s (at 120 deg/s, plus 1.0 s) |
| 4 | turn 180 deg | 2.5 s |
| 5 | 12 in forward (the stall case) | 1.5 s |
| 6 | turn 5 deg (the stall case) | 1.5 s |

**Controls (gamepad 1).**

| Control | Effect |
|---|---|
| D-pad up / down, in INIT | the first test, to go on from an earlier run |
| Hold right bumper | the robot drives only while it is held. Letting go stops it at once; the run in progress is void and is driven again |
| X | skip the current test. It is reported as SKIPPED, never as a pass |

**Behavior.**

- Before each run the robot rests 0.6 s and the battery is read at rest. A run below 12.0 V is driven and logged but **not counted**.
- If the robot is not within tolerance of a run's start, an unmeasured leg takes it there first, so every run starts from the same place.
- A leg's time cap is the larger of the default cap and its time allowed plus 1 s, so a slow leg is told apart from one that never settles.
- **Fence:** a robot more than 15 in outside the rectangle between a test's two poses ends that test as a FAIL and is driven back to the start.

**Verdict for each test.**

| Verdict | When |
|---|---|
| PASS | 10 counted runs, all settled, and the worst final error, overshoot and time are inside the limits |
| FAIL | any of those is outside its limit, or the fence ended the test. The reasons are listed |
| INCOMPLETE | fewer than 10 runs counted (the battery) |
| SKIPPED | the operator skipped it |

**Output.**

| Line | Content |
|---|---|
| `ACCEPT run` | one per run: settled or timed out, the time and its limit, the final error, the overshoot, the battery at rest |
| `ACCEPT TEST` | one per test: the verdict and its reasons; settled and timed-out counts; worst and mean final error; worst overshoot; worst time against its limit; lowest battery at rest; runs not counted and void |
| `ACCEPT RESULT` | ALL PASS, FAILED or INCOMPLETE, with the tests that are not a pass |
| `ACCEPT GAINS` | the gains and limits of *drive to pose* in force, as one line ready to paste into the code, with the hub's date |

The settings dump at Start records the gains and this opmode's limits. A row is written on every loop: the pose, the command sent, and the test, run and phase. The screen shows the current test and run, the last run's numbers and one line per finished test.

| Name | Value |
|---|---|
| Runs per test | 10 |
| Long move / short move / small turn | 48 in / 12 in / 5 deg |
| Final error limit | 1.0 in, 2.0 deg |
| Overshoot limit | 1.5 in, 5.0 deg |
| Time allowed: travel speed / strafe ratio / turn rate / margin | 22 in/s / 0.78 / 120 deg/s / 1.0 s |
| Time allowed, short tests | 1.5 s |
| Battery at rest, at least | 12.0 V |
| Rest before a run | 0.6 s |
| Fence | 15 in |
| Time cap of an unmeasured leg | 6.0 s |

All are tunable values with these proposed numbers as defaults. They are to be settled at the tuning session, and every log records what they were.

**Where it differs from the first proposal.** The log kind is `bringup`, not `drivetest`, because the drive model tools read the latest `drivetest` log. The verify autos are kept. The battery limit is Pit Check's 12.0 V at rest, not 12.5 V: a battery sags below 12.5 V within one session.

### 4.3 Drive Characterization

**Purpose.** Measure the drive itself (speed, acceleration, braking, coasting, turning, dead zones) to build the simulation model behind the grid routes.

**Procedure.** The robot drives 34 open-loop tests by itself on one alliance's half, along a lane parallel to the wall, each run starting where the last ended. The tests cover stops from speed (braking at zero power, coasting on a small power, dropping to a lower power, reversing), turning while rolling, direction changes, steady speeds forward and strafing, spins, creeping at small powers, and two random sequences as a check.

**Safety.**

- The driver must **hold** a bumper the whole time; letting go zeroes the power at once. Holding again restarts the current test.
- A **geofence** ends a run when the footprint, or the footprint where the robot would stop, would cross a wall, the center line, the hive or a flower.
- No mechanism moves.

**Output.** A log row on every loop (section 1.5 of [logging.md](logging.md)). PC tools fit the drive model from it.

**Controls** (from `design/opmodes.md`; not checked against the code). At INIT, D-pad right / left chooses the blue / red half and up / down the first test. The bumper to hold is the right one. X skips a test.

**Events the tools read:** `DRIVE TEST <n> of 34: <name>; start ...`, `DRIVE TEST <n> segment <k> of <m>: ... -> robot powers forward <f>, left <l>, turn <t>`, `DRIVE TEST <n> GEOFENCE (<reason>)`.

**Not specified:** the 34 tests themselves (each test's segments, powers and durations). They are in the code; the `DRIVE TEST` events of any `drivetest` log name them.

## 5. Auto Waypoint Marker

**Purpose.** Collect poses for writing autos.

**Procedure.** Choose a start at INIT, drive by hand, and press a button to **mark** the current pose. Each mark is logged and shown as a ready-to-paste pose with the ramp state. A second button logs a numbered **check** at a spot whose true position is known, to test the odometry. Two buttons deploy and stow the ramp; a held chord reseeds to the start. There is no undo.

**Controls** (from `design/opmodes.md`; not checked against the code). At INIT: the blue start (rear wall, y +16) or the red start (audience wall, y -16). A marks (a `MARK <n>: x, y, heading, ramp` line with the pose ready to paste), B logs a `CHECK <n>`, X / Y deploy / stow the ramp, holding left bumper + right bumper reseeds.

## 6. Telemetry Font Test

**Purpose.** Find out what the Driver Station screen can display.

**Procedure.** Pages of test patterns (sizes, headings, styles, colors, widths, line counts, and three mock-ups of the match screen) in each of the Driver Station's display formats (HTML, MONOSPACE, CLASSIC). No hardware is used. The D-pad changes the page and A the format (from `design/opmodes.md`; not checked against the code).

**Finding.** Only the HTML format formats text. Nested "big" and "small" tags change size; font-size attributes and background colors are ignored; about 15 to 16 lines of about 36 characters fit; longer lines wrap. The match screen is built on this ([teleop.md](teleop.md)).

## 7. Removed opmodes

Earlier bring-up opmodes for each actuator, a hardware check, a chassis info screen, a launch test and the pusher sensor tools were deleted once their job was done or absorbed by Pit Check. They are in the repository history, not in this specification.
