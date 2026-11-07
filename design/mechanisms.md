# BIOBUZZ mechanisms design

**Naming (2026-10-31):** the mechanisms are called the **pusher** (formerly the gate or transport gate) and the **ramp** (formerly the depollinator). The hardware-configuration names did NOT change: the pusher motor is still `gate`, and the ramp servos are still `depolLeft` and `depolRight`.

Design notes for the four actuators the final robot adds on top of the verified chassis: intake, flywheel, pusher, and ramp. Worked out one actuator at a time with the team; each item is marked **Decided**, **Proposed** (my suggestion, not yet confirmed), or **Open**.

Build status: all four actuators are built as classes in `subsystems/` with a bring-up opmode each (group "Bring-up"), and all compile. **None has been run on hardware.** The mechanisms coordinator that enforces the cross-actuator rules below is not built, because those rules are still proposed.

Last updated 2026-10-22.

## Automatic pusher jam resolution (2026-11-06, built, not yet run on the robot)

The logs of 11-05 and 11-06 had 10 stuck pusher moves (8 salvos, 1 reject, 1 lock seek). A stuck forward move stops within 1 to 11 deg of where it started (the lock seek at 67 deg) and sits at 1.4 to 2.5 A with the speed at 0 until `Pusher.MOVE_TIMEOUT_S` (3 s); a moving arm draws a median 0.22 A (99th percentile 0.9 A, maximum 4.4 A at the start of a move). The pusher stalls at only about 2.5 A, so the old `STALL_CURRENT_A` 6 would never have seen it.

**Behavior (the user's design, 2026-11-06):** a forward move (salvo, return, a forward seek such as the lock) that has run `JAM_GRACE_S` 0.4 s and then sits under `JAM_MAX_TICKS_PER_SEC` 20 with more than `JAM_AMPS` 1.2 A for `JAM_TIME_S` 0.3 s is a jam. The arm backs off half a pollen (90 deg per pollen on the rotary transport is an assumption) at `JAM_BACKOFF_POWER` 0.25 and then goes on with the move it was doing; how far depends on the move (the user's rule, 2026-11-06): a jammed **lock** backs off to `JAM_BEHIND_HOLD_DEG` 45 deg BEHIND the HOLD position, on purpose, a run-up through HOLD (the nearest 'half a pollen before a HOLD' point behind the stuck arm: stuck 67 deg past a HOLD it backs 112 deg to 45 deg before that HOLD); a jammed **salvo** (or the return nudge) backs off `JAM_SALVO_BACKOFF_DEG` 45 deg from where it is stuck and finishes (stuck at 67 deg, the 8 logged salvo jams, it goes to 22 deg; a jam at 150 deg goes to 105 deg). At least `JAM_MIN_BACKOFF_DEG` 10 deg either way. Up to `JAM_RETRIES` 2 times per move (`JAM_RETRIES_AUTO` 1 in the autos, set by `Mechanisms.setJamRetries`). The move's own 3 s timer keeps running, so a jam never costs more than today. When the retries are used up the move gives up at once and the drivers see `PUSHER JAMMED` in red until the next move; when a retry works they see `Pusher jam cleared` for 4 s. Reverse moves (reject, unlock) are left alone: backing off from one would sweep pollen forward. The intake is not touched. `JAM_RESOLVE_ENABLED` false turns it all off.

Log lines: `PUSHER JAM: the SALVO is stuck 67 deg past HOLD (2.41 A, 0 ticks/s, 0.74 s into the move): backing off 45 deg, then going on (1 of 2)`, `PUSHER JAM RESOLVED: ...`, `PUSHER JAM NOT RESOLVED: ... giving up`.

## Rules that constrain the design

Section numbers are from the BIOBUZZ Competition Manual V1.

| Rule | What it says | Why it matters here |
|---|---|---|
| R102 | The starting configuration must fit an 18 x 18 x 18 in cube (preloaded elements may stick out). | The ramp, stowed, has to fit. |
| G407 | A robot may control at most 4 scoring elements at once. | The transport holds 4. The intake and ramp must stop feeding at 4. |
| G408 | A robot may not control the opponent's NECTAR. | The pusher reverse is how we reject one. |
| G418 | Elements enter a FLOWER only through its top, and only POLLEN may be removed, from its bottom. | The ramp takes pollen only. |

## Shared architecture

- **Decided:** each actuator is a small class with setters that record a target and one `update()` call per loop, the same pattern as last year's subsystems.
- **Proposed:** autos use the existing `Action` interface and its composition (`SequentialAction`, `ParallelAction`, `SleepAction`, `InstantAction`) in `teamcode.motion`, so each actuator offers action factories such as "spin up and finish when at speed".
- **Proposed:** values that differ per chassis (motor directions, servo positions) live in `ChassisProfile`, which is already chosen automatically by Control Hub. Tunable numbers use `@Config` so they can be edited live in the FTC Dashboard.
- **Proposed:** each actuator gets its own bring-up opmode (group "Bring-up") before it is integrated, in the style of the Pinpoint tests (the bring-up opmodes for the intake, flywheel, pusher and ramp were deleted 2026-10-31 once those were integrated; Launch Test and Comp TeleOp cover them). A combined mechanisms coordinator and an integrated TeleOp come last.

## 1. Intake

**Hardware (decided):** one motor, no encoder, open loop, mounted at the FRONT of the robot, the same end as the launcher (the robot launches out of the side it intakes; corrected 2026-11-01, it was written as the back). It takes both pollen (2.8 in) and nectar (3.6 in).

**Behavior**
- **Decided:** it runs only while the robot is actively intaking, never always-on, so it cannot pick up stray balls and push us over the carry limit.
- **Decided:** it switches on automatically when the ramp lowers.
- **Decided:** power is 1.0 to take elements in and -0.5 to spit out, as last year. Both values need real tuning.
- **Decided:** the pusher physically holds only 4 elements. Once the color sensors (future) see 4, a short outtake clears anything still in the intake, as last year.
- **Decided:** jams are handled by the driver for now.
- **Open, still being considered:** current-based jam detection. If the motor current stays high for about a quarter second, reverse briefly and automatically.
- **Open idea:** with one color sensor per slot, the robot could spot an opponent-color nectar (G408) and outtake it automatically.

**Status:** the `Intake` class (`subsystems/Intake.java`) are built. The motor direction is in `ChassisProfile.intakeDirection` (FORWARD on `ROBOT_1`, set 2026-10-31).

**Verification:** checked in TeleOp - Competition. The Intake Test (hold-to-run, current reading) was deleted; a jam threshold would need the motor current with and without an element, which would have to be read in Comp TeleOp.

## 2. Flywheel

**Hardware (decided):** an encoder motor (`fly_motor_enc`), very like last year's, plus an optional second motor (`fly_follow`, no encoder, last year's name) that gets the same power. With no `fly_follow` in the hub configuration it is a one-motor flywheel. Whether the second motor is needed is decided from rig data: how far the speed dips and how fast it recovers over a 4-ball burst, with and without it. The hood is a static angle.

**Control**
- **Decided:** bang-bang plus feedforward, as last year: full power below the setpoint, a single `feedForward` motor command (0 to 1) at or above it. The prototype rig on the `launcher-prototype` branch already works this way.
- **Decided:** one flywheel speed for both pollen and nectar. A team simulation says it works.
- **Decided:** simple first. No safety floor or droop compensation until the plain version has been tried.

**Setpoint (decided)**
- It comes from an interpolating launch map, like last year's launch map: robot distance to the open cell, to flywheel speed, with linear interpolation and the end values held. The hood is fixed, so there are only two columns.
- The driver tells the robot which cell is the open one for now.
- The target point for blue is the center line of the blue HIVE, Y = +12.75, with the cell opening about 21.5 in from the HIVE center in X: the rear side (X about -21.5) until the first tip, the audience side (X about +21.5) after it. The opening sits 53.5 to 65.6 in above the tiles. Distance is measured from the robot's center, and any offset to the real launcher exit is absorbed by measuring the table on the robot.
- There is no simulation output to seed the table, so it is built from measurements on the real robot. A tuner opmode that shows the distance and the map's speed, and prints a "distance, speed" line to paste in when a launch works, is proposed.

**Cadence (decided)**
- Do not wait for the flywheel between balls: launch a group as fast as possible. Last year's 90% rule is not assumed to transfer.
- The flywheel only needs to be ready before the first ball. The ball spacing comes from the pusher's sweep speed.
- It stops between salvos and spins up when needed. In autos the spin-up runs in parallel with the drive to the launching spot.

**Status:** built and compiling, not yet run on hardware. `subsystems/Flywheel.java` (bang-bang plus `FEED_FORWARD`, 0.35 since 2026-11-03 (0.2 before; the hold power at the slowest launch-map speed 1325 is about 0.38, 0.37 on a battery above 12.3 V, so 0.35 is at the edge: lower it if the speed creeps above the setpoint at the slowest entry); `READY_FRACTION` 0.95; 5-sample filter), `HiveTarget.java` (the four open-cell target points) and `LaunchMap.java` (interpolation; the saved `TABLE` is empty until measured). Directions are `ChassisProfile.flywheelDirection` and `flywheelFollowerDirection` (REVERSE, last year's pair), both placeholders. Both motors run without the hub's closed-loop encoder modes and receive the same power, so a power is a plain duty cycle on each (the closed-loop mode would make one number mean a speed target on one motor and a duty on the other, and runs away if an encoder counts opposite to its motor, as the pusher's did). `ENCODER_SIGN` (default +1) fixes a reversed encoder, and a speed that reads strongly negative while spinning stops the flywheel with a fault. The single-motor `Launcher` on the `launcher-prototype` branch is superseded by `Flywheel`.

**Verification:** the **Launch Map Tuner** (disabled) or the TeleOp (tune `FEED_FORWARD` through the Dashboard, and read the lowest speed in the `SALVO done` log line; the Launch Test and the Flywheel Test were removed) and **Launch Map Tuner** (drive to a distance, find a speed that scores, record the pair, test the interpolation live, and print lines for `LaunchMap.TABLE`).

## 3. Pusher

**Hardware (decided):** one pusher arm, with some thickness, that rotates around a curved linear track the elements ride along. The back of the arm limits how many elements can enter (the 4 cap). The end of the track feeds the flywheel. The arm passes through the ramp, so it can reach the last element and push the whole stack up toward the flywheel. One motor with a relative encoder. **There are no hard stops: the pusher spins continuously and almost always goes forward.**

**Position (proposed from the above)**
- Position is an angle on a circle: ticks divided by the ticks per revolution, with HOLD defined as 0 degrees. The arm's thickness means its pushing face and its limiting face sit at different angles.
- Moves are forward only. The error is the forward distance to the target modulo one revolution. A small overshoot is tolerated, so a pusher that stops a few degrees past a target does not command nearly a full extra turn.
- **Measured on the bench:** ticks per revolution is 751.8, goBILDA's published value for the 26.9:1 motor (ten mark-to-mark turns read 751.6). It is a tunable constant, re-measured over 1, 5 or 10 turns by a calibration step in the bring-up opmode.
- **Decided (bench):** the encoder on the bench pusher counts opposite to its motor's direction, so `ENCODER_SIGN` (-1 there) flips the ticks, and the pusher runs without the hub's closed-loop encoder modes (which run away when the polarity is opposed).

**Behavior**
- **Decided:** full salvos only for now, because we may miss. A salvo is a forward turn of `SALVO_DEG` (400 deg since 2026-11-01: a full revolution and 40 deg more, so the last element is surely out, and the pusher check brings the arm back to HOLD): from HOLD, sweep through the ramp pushing the stack to the flywheel, then keep going forward back to HOLD.
- **Decided:** control is plain thresholding on position, with no PID and no velocity loop (a PID and a velocity loop were tried on the bench and were jerky and hard to tune). A forward move runs at a calibratable `SWEEP_POWER`, drops to a gentler `APPROACH_POWER` for the last `APPROACH_DEG`, and cuts the power `STOP_EARLY_DEG` before the target; the brake does the rest. The power sets the spacing between balls. Targets are absolute multiples of one turn, so landing errors do not accumulate. If the arm comes to rest a little short of HOLD it nudges forward; nothing else holds it against a push.
- **Decided:** reverse is needed to reject an opponent's nectar (G408), together with an intake outtake. A reject reverses about 330 degrees and finishes on the nearest HOLD behind (a full reverse turn).
- **Decided:** assume a known starting position, with the pusher at HOLD at power-on.
- **Proposed:** auto zeroes the encoder at init. TeleOp after auto does not, and takes the reference from a static saved by auto, as the pose is. A fresh TeleOp-only session assumes the pusher is at HOLD.
- **Proposed:** stall and jam detection by high current with zero speed. A blocked pusher does not lose its position, because it simply does not move.
- **Open, later:** a hall sensor at HOLD to re-zero every revolution and check the start position, or the color sensors reading the pusher tip as it passes. Color sensors read over a slow bus, so a fast tip could pass between readings, and the tip would need a distinct color.

**Status:** built and compiling, not yet run on hardware. `subsystems/Pusher.java`: position thresholding with HOLD at 0 degrees (salvo = one forward turn, return = forward to the nearest HOLD, reject = reverse onto the nearest HOLD behind), stall and no-progress fault detection, and a zero reference saved in a static (`Reference.ASSUME_AT_HOLD` for auto, `CONTINUE` for TeleOp after auto). The encoder is never reset in code. The powers, `STOP_EARLY_DEG` and the stall thresholds are placeholders to calibrate on the rig, and the pusher direction is a `ChassisProfile` placeholder. The hardware name is `gate`. Ran on the bench pusher (branch `pusher-bench`) with the earlier PID version; this thresholding rewrite has not been run yet.

**Verification:** the TeleOp and the Launch Map Tuner run a salvo, a reject and a return to HOLD, and the log shows where the arm settled after every move (the Launch Test that marked HOLD was removed); the powers and the early-stop angle are Dashboard values to copy into `Pusher.java`. `TICKS_PER_REV` is confirmed from the Pusher Sensor Matrix logs. The Pusher Test was deleted.

## 4. Ramp

**Hardware (decided):** two goBILDA torque servos lower a ramp so pollen from a FLOWER's bottom opening rolls straight into the intake. The flower's retrieval opening is about 3.55 in tall and 3.57 in deep. The pollen sits in a 2.79 in hole in a ring 0.43 in thick.

**Behavior**
- **Decided:** two positions only, STOWED and DEPLOYED.
- **Decided:** the driver or operator decides when to deploy and stow, for now. An automatic approach to a flower pickup pose is the long-run goal.
- **Decided:** deploying switches the intake on. Stowing switches it off unless the driver is holding intake.
- **Decided:** the robot must drive out from under the flower before the ramp can stow.
- **Decided (rule 5):** a stow requested while still under the flower is remembered and happens once the pose, compared against the four known flower positions, shows the robot is clear.
- **Decided:** keep the class plain, like last year's hood and stop servos: each command sets both servos straight to their position, with no slew limiting. "Settled" is a fixed `TRAVEL_SEC` after the command, measured once.
- **Proposed:** each servo has its own stowed and deployed positions in `ChassisProfile`, mirrored by direction. Both servos must be set to the same mode (goBILDA dual-mode servos can be positional, continuous or multi-turn). The deployed position is commanded just short of contact, so a torque servo is not stalled against the floor.
- **Open:** whether the ramp lands too hard at the servo's full speed. If it does, put a slew limit back (rate-limited motion, as an earlier version had).
- **Open:** the stowed and deployed angles are not known yet. A bring-up opmode jogs each servo and records them.

**Rules:** the stowed ramp must fit the R102 cube. Only pollen is removed (G418).

**Status:** built and compiling, not yet run on hardware. `subsystems/Ramp.java` (about 110 lines: servos `depolLeft` and `depolRight`, per-servo positions in `ChassisProfile`, direct `deploy()` / `stow()`, `isSettled()` after `TRAVEL_SEC`, and an `isClearOfFlower` check; no per-loop `update()` is needed). **It will not move until the four positions are measured** (they are NaN in `ChassisProfile`). The coordinator holds a stow request until the robot is clear (rule 5). `TRAVEL_SEC` (0.6 s) and `CLEAR_DISTANCE_IN` (24 in) are placeholders: measure the first, and set the second from the real ramp length. It uses the plain `Servo` interface, as last year's hood and stop servos did, so there is no configurable pulse range and no way to switch a servo off. If either turns out to be needed (a goBILDA servo in positional mode may need a wider range than the SDK default, and a torque servo may need to be powered down), go back to `ServoImplEx`.

**Verification:** the Ramp Test found the stowed and deployed positions and `TRAVEL_SEC` on 2026-10-31 and was then deleted; the values are in `ChassisProfile` and `Ramp`.

## Cross-actuator rules (confirmed 2026-10-22)

1. **Removed (2026-10-31):** the intake was gated on the pusher being at HOLD (HOLD limits capacity to 4). The pusher rests a few degrees past HOLD after a salvo and nothing drives that back, so the check ("within 8 deg of HOLD") locked out both the intake and the next salvo. The intake is no longer gated on the pusher's position. Capacity then depends on the arm physically sitting at or near HOLD, which the pusher's landing accuracy (Phase 2.3 of the integration plan) has to keep true.
2. **Decided:** deploying the ramp turns the intake on. Stowing turns it off, unless the intake has been toggled on by the operator.
3. **Decided:** the flywheel must be up to speed before a salvo starts (first ball only). A Launch pressed too early is queued: it spins up if needed and launches automatically the moment the flywheel is ready and the pusher has stopped (a salvo is a full turn from the nearest HOLD, so it can start from wherever the pusher rests), with a timeout. One button does spin-up and launch.
4. **Decided:** a rejection is a pusher reverse plus an intake outtake, together. The outtake runs for the whole reject move (reverse, then forward back to HOLD), then the intake turns off.
5. **Decided:** a stow requested while the robot is still under a flower is remembered, and the ramp stows automatically once the pose shows the robot has driven out. A warning shows meanwhile.
6. **Decided (2026-10-31):** with the element counter, the intake stops by itself when the robot holds 4 elements (rule G407) and the operator's intake toggle is cleared, so they press it again after a salvo. The outtake still works. Opponent nectar in the robot only raises a warning with its sensor position; reject stays the operator's decision.
7. **Decided (2026-10-31):** overrides for a sensor that fails in a match, shown on screen and in the log, with no stick-button or Start controls on the driver's controller (too easy to press by accident); REMOVED from the TeleOp controls on 2026-11-02 (the code remains dormant in `Mechanisms`): the operator used the stick buttons: operator L3 ignored the element counter (no 4-element stop, no element warnings; a warning also appears by itself when the counter stops reading), operator R3 runs the flywheel open loop (power = target / 2200, ready after 1.5 s, speed sensor ignored), driver LB + X drives robot-oriented (ignores the heading), driver LB + B ignores the pose (the ramp stows without waiting to clear a flower). The flywheel's reversed-encoder fault was removed: wiring is the pit check's job.
8. **Decided (2026-11-01):** the pusher check. After a salvo, once the element counter reads 0 and launch, reject and the outtake are idle (the intake may be running), the coordinator runs `PusherCheck`: it reverses the arm about 40 deg to the measured arc center (33 deg behind HOLD), looks for the arm at CS4, and if it is not seen finds it with one continuous slow sweep (not giving up before a full turn), corrects the pusher's zero from a forward and a return crossing, verifies it, and returns the arm to HOLD. Not found gives a LOST warning. Any launch, reject, intake or ramp request aborts it. It can be turned off with `Mechanisms.AUTO_PUSHER_CHECK`. The operator can also start a calibration sweep by hand (hold the left bumper + Y): no parked look, straight to the sweep, then the same verify and return; refused unless the counter reads empty and the intake, launch and reject are idle; pressing it again stops it. The zero counts as verified once a check has seen the arm where it says (or corrected it); until then (after a power cycle or install, a zero set by hand, or a faked slip) the coordinator also runs one check right after Start, if the track reads empty within 2 s (`Mechanisms.AUTO_STARTUP_CHECK`), and warns "Pusher zero not verified against CS4 yet".
9. **Decided (2026-11-02):** target lock (`drive/TargetLock.java`). TeleOp - Competition engages it by itself from the moment the robot is full until its salvo is done (`TeleOpCompetition.AUTO_LOCK_WHEN_FULL`), or while the operator has switched it on by hand with A (a toggle; full or not; A also cancels an automatic lock until the robot is next full, the way out of a bad element sensor); a finished salvo releases both the automatic and the manual lock (the automatic one then waits until the robot is no longer full before it can engage again); the driver launches with A (any number of elements). While it is on, the TeleOp turns the robot to point its front (the intake and launcher end) at the nearest of its alliance's two cells (picked automatically from the pose, with a 6 in hysteresis; the operator no longer picks a cell, so the D-pad left/right is free) and the driver keeps the translation; the turn command replaces the turn stick, which nudges the aim up to 15 deg only while the left bumper is held. It uses the Pinpoint pose and the Pinpoint's angular velocity, so it does nothing in the backup mode. ALIGNED on the screen says it is on target (within 2 deg and nearly stopped); there is no rumble for it (no rumble anywhere since 2026-11-06). The gains and `AIM_OFFSET_DEG` (the launcher's line against the robot's forward) are Dashboard values to tune on the robot.
10. **CHANGED 2026-11-06 (user): the TeleOp intake is ALWAYS IN, whatever the element count and the pusher lock say; only the operator's left trigger (force OUT) and a reject run it OUT, and a manual pusher calibration sweep still holds it off. It no longer stops at 4 (a phantom full cannot kill it) and keeps running IN against a locked arm; a fifth pollen (rule G407) is the operator's to push out with the left trigger. The right trigger now only unlocks the pusher. Everything below this marker is the earlier design.** **Decided (2026-11-02):** in TeleOp - Competition the intake has no toggle and is never off (`Mechanisms.setIntakeAlwaysOn`): it runs IN while the robot holds fewer than 4 elements and STOPS once the (1000 ms confirmed since 2026-11-04, 500 before) count reaches 4 (changed from OUT on 2026-11-02: there is no mechanical detent yet to keep elements from moving back toward the intake; `Mechanisms.FULL_RUNS_OUT` = true restores the OUT). The operator overrides by holding the right trigger (IN even when full) or the left trigger (OUT). The autos keep the toggle and the stop at 4. Reject needs the operator's left bumper + B, and the ramp's stow is immediate (no flower check, no pose override). **From 2026-11-07 (user) the autos too:** `Mechanisms.setIntakeAlwaysOn(true)` at the start of `AutonSolo` and `AutonDuoBase` (all duo autos), so the intake runs IN for the whole auto, from before the preload salvo to the end, whatever the count says (the Q5, Q10 and Q17 logs showed it off 19 to 37 percent of the auto: from the start until the garden pickup, when full, and around the salvos); the autos' `setIntake(false)`, `forceIntakeIn(false)` and `Intake OFF` notes were removed; only a reject or an outtake runs it OUT.
11. **Decided (2026-11-02):** BACKUP MODE in TeleOp - Competition, one way until the opmode ends. Entered by the driver holding left bumper + X for 1 s or when the Pinpoint stays faulted (any status but READY) or throws for 0.5 s (`BACKUP_PINPOINT_FAULT_SEC`; shorter is a blip). Then the pose and heading are not trusted: robot-oriented drive, no target lock (auto or manual), no reseed, the pose is no longer polled or saved, and the flywheel speed is the launch map's speed for a fixed 42 in (`BACKUP_LAUNCH_DISTANCE_IN`, the start position's distance) plus the speed trim, so the driver drives to that spot to launch. The screen shows BACKUP MODE in red (the three rumble taps were removed 2026-11-06). Fixing it needs the robot serviced.
12. **Decided (2026-11-02):** a dead flywheel encoder is detected and handled by `Flywheel` (latched for the rest of the run). Criteria: the motor commanded at 0.9 or more (the bang-bang pinned at full power because the speed reads under the setpoint) for 1.0 s (`LOST_DETECT_SEC`) while the measured speed never reaches 50 ticks/s (`LOST_SPEED_MAX`); a reversed encoder (negative speed) trips it the same way, and any speed of 50 or more restarts the window, so a normal or slow spin-up never trips it. Then the flywheel runs open loop at `LOST_BASE_POWER` (0.40, the average `fly_power` of a steady spin at 1350 ticks/s over 14,709 rows of the 2026-11-02 TeleOp logs: 0.38 to 0.42 per run, higher at a lower battery voltage; the earlier placeholder 0.61 would have spun about 50 % too fast) for `LOST_BASE_SETPOINT` (1350) ticks/s with NO battery-voltage compensation (the reading is too noisy and sags with the other motors); the launch map is ignored and the speed trim moves the power (10 ticks/s, one D-pad report, is about 0.0045 power); ready is a timer (`LOST_READY_SEC`, 2 s). The screen warns (the two rumble taps were removed 2026-11-06) and the log has `FLYWHEEL ENCODER LOST` with the numbers. Motor current (`fly_amps`) is logged to tell a dead encoder from a dead or jammed motor, but is not used yet.
13. **Found 2026-11-02 (logs):** after a salvo the pusher arm rests 40 to 60 deg past HOLD, where CS4 reads "other" and the counter counted it as an element (1 in an empty robot, or a false 4th). It skipped about a fifth of the post-salvo pusher checks ("the robot did not read empty"), refused the manual calibration sweep ("1 element(s) held") and could stop the intake or engage the lock at 3 real elements. `Mechanisms.ARM_NEAR_AHEAD_DEG` is now 70 (was 25): CS4's "other" is ignored as the arm's edge while the arm is within 70 deg ahead of HOLD; pollen and nectar readings still count. Untested.

## Element counter (CS1 to CS4)

Four REV Color Sensor V3, one per I2C bus, named `CS1` to `CS4` and numbered along the track: intake, CS1, CS2, CS3, CS4, flywheel (CS4 is deepest and also sees the pusher arm). `ElementCounter` reads **one sensor every 40 ms** (each read is about 13 ms of blocking I2C, and the sensors refresh every 100 ms) and reports, per sensor, EMPTY, POLLEN, RED NECTAR, BLUE NECTAR, OTHER or PUSHER ARM.

- **Presence** comes from the shift of the color shares from each sensor's own empty baseline (the empty views differ per sensor and the contrast against a near wall is small), then alpha and distance, with hysteresis and a debounce.
- **Count** = sensors that see an element (not the arm), reported only after it has been steady (150 ms going up, 1000 ms for the fourth so FULL is certain (500 ms before 2026-11-04), 300 ms going down). A fourth element is reported about 1.3 s after it is first seen. An element without a neighbor toward the flywheel sets a *gap* flag but still counts.
- **Baseline:** the empty readings in `ElementCounter` were measured 2026-10-31. Lighting changes them, so re-measure in the pit with Color Sensors Test (it is now `@Disabled`; **A** with the track empty) and copy the BASELINE line into the table. It cannot be captured at match start, because the robot may already hold elements.
- **Measured on the robot:** four pollen read 100% correct on all four sensors (3,298 samples); blue nectar and the pusher arm classified correctly in the logs; red nectar worked up close and was loosened for distance (not yet re-run live). Thresholds are accepted for now.
- **Not done yet:** a live run of the counter inside TeleOp - Competition, and the pusher position check that uses `pusherArmSeen()` on CS4.

The flywheel is bang-bang only. A switchable feed-forward plus P controller (developed 2026-11-04, never run on the robot, with a simulation in `tools/flywheel/ffp_sim.py` and test row T8) was removed 2026-11-05 at the user's request: the bang-bang is tried and true for the team. It is in git history (commit 4001aba, with the code-review fix in 08f4815) if it is ever wanted again.

Decided (2026-11-04, from the 268 salvos of 11-02 and 11-03; tools/logs/launch_stability.py): a salvo only STARTS while the filtered flywheel speed is inside the launch band, 96 to 104 percent of the setpoint (`Flywheel.LAUNCH_BAND_LOW_FRACTION` / `LAUNCH_BAND_HIGH_FRACTION`); a queued launch waits for it (at most `LAUNCH_TIMEOUT_SEC`) and a salvo that has started is never interrupted by the speed, there is no check per pollen. Before this only the lower limit (READY_FRACTION 95 percent) existed, and 5 of 268 salvos started 5 to 16 percent over the setpoint, all just after the driver moved in and the setpoint dropped. While a launch is queued, a salvo runs and for 0.6 s after, the log samples on every loop (`BURST_LOG_SALVOS`).

**Active hold (2026-11-05, not yet run on the robot):** in the HOLD state the pusher motor used to be at 0 power with the brake, which resists motion but not a steady push, so the arm could creep (2026-11-05 logs: more than 10 deg off where it had stopped for 30 to 46 percent of the parked time, pusher check corrections of +94 and +155 deg after salvo-heavy stretches). Now `Pusher.HOLD_SERVO_ENABLED`: once the arm has been still for 0.3 s wherever it rests (HOLD, the 40 deg past it after a salvo, the lock) that angle is latched, and a P power (0.012 per deg, at least 0.05, at most 0.15, in 0.01 steps, deadband 2 deg) pushes it back when it drifts; if it needs the maximum for 2 s it lets go until the next move. Log: `PUSHER HOLD SERVO ...` lines and the `pusher_hold` column. Test row T15.

**OFF since 2026-11-06 (user): `Mechanisms.PUSHER_LOCK_ENABLED` is false. The TeleOp intake is always IN now, so the arm must not lock across the entrance; the lock comes back (set the Dashboard value true) when the element sensors are more reliable. Everything below describes the lock as built.** Decided (2026-11-04): the PUSHER LOCK. The end stop of the track that forces the pollen up into the flywheel is the **feed ramp** (not the ramp, the deployable depollinator). While a target lock or a Hive Dash is on (and something is held and the arm is at HOLD), the arm moves 90 deg forward (`Pusher.LOCK_DEG`) to rest across the intake entrance, so the pollen cannot roll back out and a fifth cannot get in; it moves the pollen forward a little and does not launch one. It is automatic only: no lock or unlock control and no retry wait (user, 2026-11-04; `Mechanisms.PUSHER_LOCK_ENABLED` is the off switch). **Changed 2026-11-05 (user):** it no longer locks when the robot is full (the count of 4 cannot be trusted, and a full robot's intake simply stops, as before the lock): the TeleOp asks for it with `Mechanisms.setPusherLockWanted(target lock on or Hive Dash running)`, the arm goes back to HOLD `LOCK_RELEASE_DELAY_SEC` (0.5 s) after the request ends without a launch, and the autos never request it. While locked: the element count is frozen (`ElementCounter.setFrozen`, the sensors still log), and the intake is OFF (it used to run OUT; changed 2026-11-05), also while the arm is moving to the lock, so no pollen is pushed against the arm; the operator's forced outtake and a reject still run it OUT. A salvo from the lock targets the nearest HOLD behind plus `SALVO_DEG`, so it turns 310 deg (the salvo releases the lock); a reject unlocks first (a 90 deg reverse to HOLD, never forward, which would sweep the pollen into the flywheel) and then starts by itself; the operator forcing the intake IN (right trigger) while locked unlocks it, the lock stays off while the trigger is held, and after it the robot locks again while the target lock or dash is still on. A lock that fails (the arm does not reach 90 deg) is logged and tried again whenever the conditions hold (the arm at HOLD, not busy). Open: what CS1 to CS3 read with the arm parked at +90 deg (the `slots` column in the log shows it); a false full (3 spread pollen) does not release by itself: force the intake IN.

Also decided: after a salvo finishes, the flywheel stops automatically a short moment later so the last ball clears, and spins up again only when the next Launch is requested. The integrated TeleOp splits the controls between a driver (gamepad 1, drives the chassis) and an operator (gamepad 2, runs the mechanisms), as last year.

**Status:** the coordinator (`subsystems/Mechanisms.java`) and the competition TeleOp (`opmodes/TeleOpCompetition.java`) implement these rules. Any mechanism the robot does not have yet is simply skipped, so the coordinator works on a partially built chassis. Not yet run on hardware.

## Still to measure or decide

- [ ] Pusher ticks per revolution (calibration step).
- [ ] Ramp servo angles, stowed and deployed, and `TRAVEL_SEC`. Watch whether the ramp lands too hard without a slew.
- [ ] Launch map: distance to speed, measured on the robot.
- [ ] Intake outtake power (starts at -0.5).
- [ ] Whether to add current-based jam detection to the intake.
- [ ] Hard interlock or warning for stowing under a flower.
- [ ] The color sensors: placement and what they read (element count, element color, possibly the pusher tip).
- [ ] Whether to add a hall sensor at the pusher's HOLD position.

## Proposed build order

1. Intake class and bring-up opmode.
2. Flywheel: single motor with an encoder, the launch map, and its tuner.
3. Pusher, including the ticks-per-revolution calibration.
4. Ramp, including measuring the servo angles.
5. A mechanisms coordinator that enforces the cross-actuator rules, then an integrated TeleOp. The color sensors plug into the coordinator later.
