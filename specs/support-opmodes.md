# Support opmodes

Opmodes that are not run in matches: checks, tuners, studies and build aids. Each exists to produce a number, a table or a verdict that goes back into the code or into [robot_design/](../robot_design/README.md). They are specified by purpose, procedure and output, not control by control.

**Visibility.** An opmode can be hidden from the Driver Station list without removing its code. Hidden opmodes are brought back by removing one marker line and reinstalling. The list is kept short for competition.

| Opmode | Group | Shown | Log kind |
|---|---|---|---|
| Pit Check (robot on blocks) | Bring-up | yes | pitcheck |
| Launch Tuner | Bring-up | hidden | bringup |
| Launch Grid Tuner | Bring-up | hidden | bringup |
| CS Pass Study | Bring-up | hidden | bringup |
| Flywheel Step Study | Bring-up | hidden | bringup |
| Color Sensor Study | Bring-up | hidden | bringup |
| Color Sensors Test (CS1-CS4) | Bring-up | hidden | none (device log) |
| Pusher Calibration Test | Bring-up | yes | bringup |
| Launcher Offset Finder | Bring-up | hidden | bringup |
| Pinpoint Push Test | Bring-up | hidden | bringup |
| Pinpoint Offset Check | Bring-up | hidden | bringup |
| Telemetry Font Test | Bring-up | hidden | none |
| Auto Waypoint Marker | Auto build | hidden | waypoints |
| Drive Acceptance | Tuning | hidden | bringup |
| Tune - Axial, Lateral, Heading | Tuning | hidden | bringup |
| Drive Characterization | Tuning | hidden | drivetest |
| Auton Verify - Forward 48 and back, Lateral 48 and back | Auto verify | hidden | none |

**Common rules.** No opmode uses Start for a function. **Six support opmodes use Back,** which the match opmodes never do: Launch Tuner and Launch Grid Tuner (clear a pusher fault; held 1 s, reseed the pose), CS Pass Study (clear a pusher fault), Pit Check (stop everything and end), Pinpoint Push Test (apply the settings again and restart) and Color Sensors Test (clear the baseline). Anything that moves waits for a button. Opmodes that seed the pose need the robot on the start spot and still at INIT.

## 1. Pit Check

**Purpose.** The one check to run after a repair, a rewiring or a hub swap, and before a match day. The running code does not watch for wiring and sign faults, so this is where they are found.

**Setup.** Robot on blocks, wheels clear, ramp free to move, track empty. Nothing moves until a button is pressed for that step; every move is short and slow.

**Controls.** One button runs a step or confirms "what I saw was right"; one says "wrong"; one skips; one stops everything.

| Step | Checked | Judged by |
|---|---|---|
| Configuration | every required device name and type; a stale second flywheel motor; the hub and port of each device; configured devices the code does not use | automatic |
| Battery | at least 12.0 V on every hub | automatic |
| Pinpoint | status READY and a sane device loop time | automatic |
| CS1 to CS4 | each answers; CS1, CS3 and CS4 match their stored empty readings (CS2, on the feed ramp, has none) | automatic |
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

**Controls.**

| Who | Control | Effect |
|---|---|---|
| Driver | sticks; left trigger | drive as in the TeleOp; precision (30 percent) |
| Driver | right bumper | target lock on / off (a toggle here; held in the TeleOp) |
| Driver | D-pad up / down | speed trim, 20 ticks/s a step |
| Driver | D-pad right / left | aim trim, 0.5 deg a step (right aims left, counterclockwise) |
| Driver | A | fire. With the lock on it waits for aligned |
| Driver | Back | clear a pusher fault; held 1 s, reseed the pose to the start |
| Operator | left trigger | intake OUT |
| Operator | D-pad | the TeleOp's landed low / high / left / right reports, with these steps |
| Operator | A / B | record (it replaces a point within 1 in) / undo |
| Operator | Y | fire, without waiting for the lock |
| Operator | left bumper | speed base: the saved map / the points recorded this session |

Log lines: `TUNER SALVO`, `TRIM`, `RECORD`, `UNDO`, `TUNER TABLE`.

The alliance is assumed blue: the start is the blue default start against the rear wall, and the cell is the closer of blue's two.

### 2.2 Launch Grid Tuner

**Purpose.** Collect the data for the two-dimensional launch grid ([aiming.md](aiming.md)): aim and speed per field position for one hive cell.

**Procedure.** Choose the near or far cell at INIT; the alliance is assumed blue. While a trigger is held the robot drives itself to the next spot of the grid, already pointed at the cell with today's aim and at the launch map's speed. The operator loads, fires, and trims the angle (the robot re-aims by itself) and the speed until the salvo scores, then accepts the spot, or marks it as "no shot possible", or goes back one spot.

- The route to each spot comes from the dash planner; where the planner refuses a spot beside a flower stand, a short straight hop is used if it is clear, otherwise the operator drives by hand.
- Small re-aims use *drive to pose* with its static friction term, and each reports whether it turned or stuck.
- A sweep can be continued from a later spot, restricted to a list of spot numbers, or extended with extra positions, by settings read at INIT.

**Output.** One line per spot (speed split into map and trim, aim heading split likewise, distance, salvos fired, pose; a good spot with no salvo is marked unverified), a line per salvo, and a table at the end. A PC tool turns the log into the generated grid table.

**Controls** (gamepad 1). At INIT, D-pad left / right chooses NEAR (the rear cell) / FAR (the audience cell); the start is against the rear wall, at (-63.4, 16.0, 0 deg) (16.5 until 2026-11-18; the same y as the TeleOp and the autos).

| Control | Effect |
|---|---|
| Right trigger (hold) | drive to the next spot; letting go stops at once |
| Sticks; right bumper | drive by hand, robot-oriented, while the trigger is not held; precision (30 percent) |
| D-pad left / right | aim left / right by 0.5 deg; the robot turns by itself |
| D-pad up / down | speed, 20 ticks/s a step |
| Y | fire |
| A / B / X | accept the spot / no shot is possible here / back one spot |
| Left trigger (hold) | intake OUT |
| Back | clear a pusher fault; held 1 s, reseed the pose to the start |

**Settings read at INIT.** `START_INDEX`: start the sweep at this position in the list of spots that will be visited, counted from 0, so 0 is its first spot; a value past the end means the last spot. The earlier spots stay in the list, and X can go back to them. `RETEST_SPOTS`: visit only these spots, a comma list of spot numbers with ranges, such as `8-10,22-30`; the spots keep their numbers. `EXTRA_SPOTS`: extra positions as `x,y;x,y`, visited last and numbered from 101; each must pass the grid's rules. `CARRY_TRIMS` (off): start a spot with the previous spot's trims; otherwise they start at 0.

**Visiting order.** A serpentine over the columns (x), the column nearest the start first. Inside a column the spots are taken along y, beginning at the end nearer to where the column before finished.

**Driving.** At most 0.5 power to a spot. A spot driven to by hand counts as reached within 4 in. An angle trim is limited to 20 deg either way. A re-aim turns at no more than 0.3 power to within 0.25 deg, keeping the position within 1 in, for at most 2.5 s, and reports `RE-AIM OK` or `RE-AIM STUCK` (more than 0.4 deg left), with the degrees asked, turned and left. A (accept) is ignored, with a notice, unless the robot is at the spot; B (no shot) works from anywhere.

**The short straight hop** is clear when the robot's rectangle, grown by the planner's margin and turning steadily from its present heading to the spot's, touches no support and no flower stand (the stands not grown) at any half inch of the line, and stays inside the walls less 3 in.

**Log lines.**

- `FIRE spot <n> salvo <k>: pose (<x>, <y>, <deg> deg), aim heading <deg> deg (trim <t>), <d> in from <cell>, setpoint <s> (map <m>, trim <t>)`
- `GRID POINT <n> of <N>: spot (<x>, <y>)[ slid], <d> in, GOOD: speed <s> ticks/s (map <m> <t> trim), aim heading <deg> deg (geometric <g> <t> trim), <k> salvo(s) fired[ (UNVERIFIED)], pose (...)`, or with `NO SHOT possible` in place of the `GOOD: ...` part
- `GRID TABLE (x, y, distance in, speed ticks/s, angle trim deg, status G good / N no shot / - not done, salvos fired): {<x>, <y>, <d>, <speed>, <trim>, '<status>', <k>}, ...`
- `TRIM spot <n>: angle <t> deg, speed <t> ticks/s (aim heading <deg> deg)` at every trim

In the table, x and y are the spot's, and the speed is written only for a good spot; for any other it is `NaN`. `<N>` in `GRID POINT` is the number of spots that will be visited.

### 2.3 Launcher Offset Finder

**Purpose.** Measure the launcher's sideways offset, and separate it from any error in where the hive's cells are.

**Procedure.** No target lock. Drive by hand to a wall, line the launcher up with a cell's centerline by eye, launch, and **mark** a salvo as good, or as landed left or right. Repeat for all four cells, ideally at two distances each, reseeding the pose after long drives.

**Output.** Marks in the log with the pose at each launch. A PC tool computes, per cell, how far the cell center sat from the robot's front line; the offset is the mean of a hive's two cells, the hive error half their difference, and the trend with distance is an angle error.

## 3. Studies

Each study was built to answer one question from logged data.

| Study | Question | Method | Result so far |
|---|---|---|---|
| Launch Salvo Speed Study (removed 2026-11-17) | How fast should the pusher turn a salvo? | A copy of the competition TeleOp in which the driver's D-pad changed the pusher's power live | 0.37 turned a salvo in 1.0 s against 1.2 s at 0.30 with no change in flywheel dip; the team chose 0.30 |
| Flywheel Step Study | How fast does the flywheel follow a setpoint change? | The flywheel alone steps through a list of setpoints; each change is timed until the speed is in the launch band and has stayed there | not yet run on the robot |
| CS Pass Study | Can CS5 count entering elements without slowing the loop? | The real mechanisms with the intake in; CS5 read on its own thread; the operator presses a button per element fed as ground truth | yes at a 20 to 25 ms period with a small read; this led to the entry counter |
| Color Sensor Study | What do the track sensors CS1, CS3 and CS4 read after the track or lighting changes? | Four guided steps: empty baseline, an empty pusher sweep, a full baseline with 4 pollen, a slow reject | prints the new empty readings to paste into the element counter |

### 3.1 The match loop exists once

**The Launch Salvo Speed Study was removed on 2026-11-17** (target T6). It was a copy of the competition TeleOp, about 830 lines, made to change one number from the gamepad, and it had fallen behind the TeleOp: it lacked the FULL AUTO or COUNT ONLY choice, FULL arming the auto-launch, the launch grid switch, the flywheel position at Start and the stop of all mechanisms at the end. Its question is answered. The number it varied, the pusher's salvo power, is a live tunable value, so the same experiment is the competition TeleOp with that value edited in the Dashboard, and every run logs the value in force ([logging.md](logging.md)). The code is in the repository history.

**The rule from here on.** The match loop is written once and no opmode copies it.

- A study that needs match behavior with a variation is the competition TeleOp started with a **study option** chosen at INIT. The option may add bindings only on controls the TeleOp leaves free, add log columns, and change tunable values. It may not change the loop. The log's `SETUP` line names the option, so no study run can be mistaken for a match run. No study needs this today, so the option is not built.
- A study that needs only part of the robot (the Flywheel Step Study, the CS Pass Study, the Color Sensor Study) stays its own small opmode built on the subsystems.

**Color Sensors Test** is an older tool that measures the empty baseline and scores the classification thresholds against hand-set labels; the Color Sensor Study replaced it for baselines.

### 3.2 Controls and details of the studies and checks

All on gamepad 1.

| Opmode | Details |
|---|---|
| Flywheel Step Study | Setpoints in ticks/s: 1350, 1650, 1350, 1450, 1350, 1375, 1325, 1650, 1450, 1650, 1325; the list is run 2 times. The loop is padded to 28 ms, about the length of the TeleOp's loop (the TeleOp itself is not padded). A step has settled when the speed has stayed in the launch band 0.3 s; it ends 1.5 s after that, or after 8 s with `NEVER SETTLED`. A runs the sequence (again: restarts it); B stops it and the wheel. With it stopped, D-pad up / down steps the setpoint by 100 and right / left by 25; from off, any D-pad press spins up to 1350. One `STEP` line per change and a `STEP TABLE` at the end |
| CS Pass Study | Intake IN throughout, flywheel at 500 ticks/s, loop padded to 28 ms. CS5 on its own thread every 20 ms, one 15-byte burst read per cycle; a `CS5 THREAD` line every 5 s. Sticks drive robot-oriented, right bumper precision. B = one element fed (a `FED n` line, the ground truth). Y salvo; X automatic clear on / off (while on, a salvo fires by itself when the slot count reads 4 or more, and not within 2 s of the last salvo); A mark; D-pad up / down setpoint by 20; left trigger intake OUT; Back clears a pusher fault |
| Color Sensor Study | A starts the step shown, and stops a running one. D-pad left / right goes to the previous / next step, between steps only. Right bumper logs a note. Holding X runs the intake IN to load pollen, between steps only. The baselines hold still about 8 s; the sweep and the slow reject turn the pusher backward one full turn at 0.15 power, open loop, and are given up after 25 s, or after 2 s if the arm has not moved 5 deg. One sensor is read every 25 ms. The pusher must be at HOLD at INIT |
| Color Sensors Test | A captures the empty baseline; Back clears it. B turns the sensor LEDs on / off. X switches between one sensor per loop and all four every loop. D-pad up / down selects a sensor and left / right sets its label; left bumper sets every label to NOTHING. Y starts / stops recording against the labels; right bumper clears the counts |
| Launcher Offset Finder | At INIT, D-pad right / left chooses the BLUE / RED default start. Sticks drive as in the TeleOp; left bumper is precision. Right trigger: intake IN, even when full; left trigger: OUT. A launches. Y marks a good salvo; D-pad left / right marks one that landed left / right. X held 1 s reseeds. The speed is for the nearest of all four cells; the flywheel runs from Start; there is no speed trim. Each launch logs a `LAUNCH` line with the pose, the nearest cell, the distance, the bearing and how far to the left of the robot's front line the cell's center sits. A mark logs `MARK good`, `MARK landed LEFT` or `MARK landed RIGHT`, then ` | AT THE LAST LAUNCH: <that launch line> | NOW: <the same for the present pose>` |
| Pinpoint Push Test | Three guided pushes with the drive motors floating: forward about 6 in (expect +x), left about 6 in (expect +y), counterclockwise about 45 deg (expect + heading). A step is judged once the push reads 6 in, or the turn 45 deg. A skips a step; Back applies the settings again, zeroes the pose and restarts |
| Pinpoint Offset Check | A starts or reruns the spin; B stops at once. The robot spins 3 turns at 0.25 turn power (45 s at most) and the position is sampled at every half turn. Each sample taken half a turn into a revolution, less the mean of the samples before and after it (which removes a steady drift), is the far point of the offset error's circle, (-2 ey, 2 ex): ex is the error along the forward axis, which follows the x offset, and ey along the strafe axis, which follows the y offset. The net drift per full turn is reported apart. The sign of the correction has not been verified on hardware |
| Pit Check | A runs a step, or says what was seen was right; B says it was wrong; X skips; Back stops everything and ends |
| The tuners | B is the kill switch. Axial: (0, 0) and (48, 0) in turn. Lateral: (0, 0) and (0, 48). Heading: 0, 135, -135, 20, -20, 175, -175, 90, -90 deg in turn |

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

**Status:** built 2026-11-17 for target T2 ([motion.md](motion.md) section 2.1). **Run on the robot 2026-11-19 21:32 (build 60e5cbe, 119 s): ALL PASS, 6 tests of 10 runs, with the gains of 2026-09-14 and the static friction term on** (results in motion.md section 2.1). It is shown on the Driver Station.

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

**Controls.** At INIT, D-pad right / left chooses the blue / red half (nothing moves until one is chosen) and up / down the first test. The bumper to hold is the right one. X skips a test.

**Events the tools read:** `DRIVE TEST <n> of 34: <name>; start ...`, `DRIVE TEST <n> segment <k> of <m>: ... -> robot powers forward <f>, left <l>, turn <t>`, `DRIVE TEST <n> GEOFENCE (<reason>)`.

**The area** (blue frame; a red robot's pose is rotated first). A lane along x at y = 46, from x = -54 to +54: a run starts at the end the robot is nearer and drives to the other, so one test ends where the next starts. Spot tests are at x = +40 or -40 on the lane, whichever is nearer.

**A test:** drive to its start under position control (at most 0.6 power, to within 2 in and 2 deg), wait until the robot is still (slower than 2 in/s and 10 deg/s for 0.25 s; the wait is at most 1.5 s), then run its segments. The robot is seeded at the TeleOp's default start of the chosen half. At a spot it faces along the lane. Every test ends with a zero-power segment. When that is over the next test begins at once, with its own drive to the start and its own wait for stillness: there is no other pause between two tests. A geofence stop jumps to the test's closing zero-power segment, which is still run and logged, and the test counts as stopped. Letting go of the bumper pauses; holding it again resumes a drive to the start, and restarts from its start a test that was resting or running. The `phase` column is one of `MOVE`, `REST`, `RUN`, `PAUSED`, `DONE`. A segment is one constant command, turned into robot-frame powers once, when it starts. On the lane the command is (along the lane, across it toward the alliance wall, turn toward the alliance wall); at a spot it is the robot's own (forward, left, turn). A segment ends after its time (3.0 s unless stated), or earlier on its own condition.

**The geofence** stops a run at once when the footprint, or the robot where it would stop at 200 in/s², would cross a wall (68 in), the center line (y = 4), the hive (|x| under 22.5 with y under 28) or a flower stand grown by 3 in.

**The 34 tests**, most important first. *Heading* is the robot's heading against the direction of travel: 0 front first, 180 rear first, 90 or -90 strafing. A single number is the power along the lane. *Until the stop needs N* means until the lane left, less 6 in, is the distance to stop at N in/s². *Stop* is zero power.

| # | Test | Heading | Segments |
|---|---|---|---|
| 1 | stop: brake from 0.85, front first | 0 | 0.85 until the stop needs 250; stop 0.8 s |
| 2 | stop: brake from 0.85, rear first | 180 | as test 1 |
| 3 | stop: coast on 0.2 from 0.85 | 0 | 0.85 until the stop needs 80; 0.2 until slower than 12 in/s (0.1 to 2.0 s); stop 0.6 s |
| 4 | stop: coast on 0.2 from 0.85, rear first | 180 | as test 3 |
| 5 | stop: drop from 0.85 to 0.45 | 0 | 0.85 until the stop needs 55; 0.45 for 0.6 s; stop 0.6 s |
| 6 | stop: reverse 0.4 from 0.85 | 0 | 0.85 until the stop needs 160; -0.4 until slower than 10 in/s (0.05 to 0.6 s); stop 0.6 s |
| 7 | stop: brake from 0.85, strafing | -90 | as test 1 |
| 8 | stop: coast on 0.2 from 0.85, strafing | -90 | as test 3 |
| 9 | stop: brake from 0.6 | 0 | 0.6 until the stop needs 250; stop 0.8 s |
| 10 | stop: coast on 0.2 from 0.6 | 180 | 0.6 until the stop needs 80; 0.2 until slower than 12 in/s (0.1 to 2.0 s); stop 0.6 s |
| 11 | roll at 0.85 + turn 0.5 for 0.25 s | 0 | 0.85 for 80 in; (0.85, 0, -0.5) for 0.25 s; stop 0.8 s |
| 12 | the same, rear first | 180 | as test 11 |
| 13 | roll at 0.85 + turn 0.3 for 0.3 s | 0 | 0.85 for 80 in; (0.85, 0, -0.3) for 0.3 s; stop 0.8 s |
| 14 | roll at 0.85, then zero power + turn 0.5 | 0 | 0.85 for 80 in; (0, 0, -0.5) for 0.4 s; stop 0.8 s |
| 15 | roll at 0.85, then 50 deg toward the center line | 0 | 0.85 for 80 in; (0.55, -0.65, 0) for 0.3 s; stop 0.8 s |
| 16 | the same, rear first | 180 | as test 15 |
| 17 | roll at 0.85, then a dash corner | 0 | 0.85 for 80 in; (0.2, -0.3, -0.5) for 0.3 s; stop 0.8 s |
| 18 | step forward 0.3 | 0 | 0.3 for at most 3.0 s; stop 0.6 s |
| 19 | step forward 0.5 | 180 | 0.5 for at most 3.0 s; stop 0.6 s |
| 20 | step forward 0.7 | 0 | 0.7 for at most 3.0 s; stop 0.6 s |
| 21 | step forward 1.0 | 0 | 1.0 for at most 3.0 s; stop 0.8 s |
| 22 | step strafe 0.4 | -90 | 0.4 for at most 3.0 s; stop 0.6 s |
| 23 | step strafe 0.7 | 90 | 0.7 for at most 3.0 s; stop 0.6 s |
| 24 | step strafe 1.0 | -90 | 1.0 for at most 3.0 s; stop 0.8 s |
| 25 | step diagonal 0.7 | -45 | 0.7 for at most 0.8 s; stop 0.6 s |
| 26 | spin 0.3 | spot | turn 0.3 until turned 300 deg, at most 2.0 s; stop 0.6 s |
| 27 | spin 0.5 | spot | turn 0.5 until turned 400 deg, at most 1.5 s; stop 0.6 s |
| 28 | spin -0.7 | spot | turn -0.7 until turned 450 deg, at most 1.2 s; stop 0.8 s |
| 29 | spin 1.0 | spot | turn 1.0 until turned 500 deg, at most 1.0 s; stop 0.8 s |
| 30 | creep forward 0.08 / 0.12 / 0.16 | 0 | each power for 0.7 s; stop 0.4 s |
| 31 | creep strafe 0.08 / 0.12 / 0.16 | -90 | each power for 0.7 s; stop 0.4 s |
| 32 | creep turn 0.05 / 0.08 / 0.12 | spot | each turn power for 0.7 s; stop 0.4 s |
| 33 | random A | spot at x = +40 | 12 random commands of 0.25 s (along up to 0.5, across up to 0.3, turn up to 0.6 either way; random seed 11); stop 0.6 s |
| 34 | random B | spot at x = +40 | the same with random seed 12 |

In tests 11 to 25, 30 and 31 each moving segment also ends when the stop needs 250, so a run cannot reach the lane's end.

The random tests draw each command from Java's `Random` started with the seed, three uniform draws from 0 to 1 per command in the order along, across, turn: along = draw - 0.5, across = 0.6 x (draw - 0.5), turn = 1.2 x (draw - 0.5).

## 5. Auto Waypoint Marker

**Purpose.** Collect poses for writing autos.

**Procedure.** Choose a start at INIT, drive by hand, and press a button to **mark** the current pose. Each mark is logged and shown as a ready-to-paste pose with the ramp state. A second button logs a numbered **check** at a spot whose true position is known, to test the odometry. Two buttons deploy and stow the ramp; a held chord reseeds to the start. There is no undo.

**Controls** (gamepad 1). At INIT, D-pad right / left chooses the blue start (rear wall, y +16) / the red start (audience wall, y -16); nothing drives until one is chosen. Right trigger is precision. A marks: `MARK <n>: x <x>, y <y>, heading <deg> deg, ramp <state> | <the pose as a line to paste>`. B logs a `CHECK <n>` the same way. X / Y deploy / stow the ramp. Left bumper + right bumper held 1 s reseeds to the start.

## 6. Telemetry Font Test

**Purpose.** Find out what the Driver Station screen can display.

**Procedure.** Pages of test patterns (sizes, headings, styles, colors, widths, line counts, and three mock-ups of the match screen) in each of the Driver Station's display formats (HTML, MONOSPACE, CLASSIC). No hardware is used. D-pad right / left changes the page and A the format.

**Finding.** Only the HTML format formats text. Nested "big" and "small" tags change size; font-size attributes and background colors are ignored; about 15 to 16 lines of about 36 characters fit; longer lines wrap. The match screen is built on this ([teleop.md](teleop.md)).

## 7. Removed opmodes

The Launch Salvo Speed Study (section 3.1), earlier bring-up opmodes for each actuator, a hardware check, a chassis info screen, a launch test and the pusher sensor tools were deleted once their job was done or absorbed by Pit Check. They are in the repository history, not in this specification.
