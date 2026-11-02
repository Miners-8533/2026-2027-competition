# BIOBUZZ mechanisms design

**Naming (2026-10-31):** the mechanisms are called the **pusher** (formerly the gate or transport gate) and the **ramp** (formerly the depollinator). The hardware-configuration names did NOT change: the pusher motor is still `gate`, and the ramp servos are still `depolLeft` and `depolRight`.

Design notes for the four actuators the final robot adds on top of the verified chassis: intake, flywheel, pusher, and ramp. Worked out one actuator at a time with the team; each item is marked **Decided**, **Proposed** (my suggestion, not yet confirmed), or **Open**.

Build status: all four actuators are built as classes in `subsystems/` with a bring-up opmode each (group "Bring-up"), and all compile. **None has been run on hardware.** The mechanisms coordinator that enforces the cross-actuator rules below is not built, because those rules are still proposed.

Last updated 2026-10-22.

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
- **Proposed:** each actuator gets its own bring-up opmode (group "Bring-up") before it is integrated, in the style of the Pinpoint tests (the bring-up opmodes for the intake, flywheel, pusher and ramp were deleted 2026-10-31 once those were integrated; Shot Test and Comp TeleOp cover them). A combined mechanisms coordinator and an integrated TeleOp come last.

## 1. Intake

**Hardware (decided):** one motor, no encoder, open loop, mounted on the BACK of the robot. It takes both pollen (2.8 in) and nectar (3.6 in).

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
- **Decided:** bang-bang plus feedforward, as last year: full power below the setpoint, a single `feedForward` motor command (0 to 1) at or above it. The prototype rig on the `shooter-prototype` branch already works this way.
- **Decided:** one flywheel speed for both pollen and nectar. A team simulation says it works.
- **Decided:** simple first. No safety floor or droop compensation until the plain version has been tried.

**Setpoint (decided)**
- It comes from an interpolating shot map, like last year's launch map: robot distance to the open cell, to flywheel speed, with linear interpolation and the end values held. The hood is fixed, so there are only two columns.
- The driver tells the robot which cell is the open one for now.
- The target point for blue is the center line of the blue HIVE, Y = +12.75, with the cell opening about 21.5 in from the HIVE center in X: the rear side (X about -21.5) until the first tip, the audience side (X about +21.5) after it. The opening sits 53.5 to 65.6 in above the tiles. Distance is measured from the robot's center, and any offset to the real shooter exit is absorbed by measuring the table on the robot.
- There is no simulation output to seed the table, so it is built from measurements on the real robot. A tuner opmode that shows the distance and the map's speed, and prints a "distance, speed" line to paste in when a shot works, is proposed.

**Cadence (decided)**
- Do not wait for the flywheel between balls: fire a group as fast as possible. Last year's 90% rule is not assumed to transfer.
- The flywheel only needs to be ready before the first ball. The ball spacing comes from the pusher's sweep speed.
- It stops between salvos and spins up when needed. In autos the spin-up runs in parallel with the drive to the shooting spot.

**Status:** built and compiling, not yet run on hardware. `subsystems/Flywheel.java` (bang-bang plus `FEED_FORWARD`, placeholder 0.2; `READY_FRACTION` 0.95; 5-sample filter), `HiveTarget.java` (the four open-cell target points) and `ShotMap.java` (interpolation; the saved `TABLE` is empty until measured). Directions are `ChassisProfile.flywheelDirection` and `flywheelFollowerDirection` (REVERSE, last year's pair), both placeholders. Both motors run without the hub's closed-loop encoder modes and receive the same power, so a power is a plain duty cycle on each (the closed-loop mode would make one number mean a speed target on one motor and a duty on the other, and runs away if an encoder counts opposite to its motor, as the pusher's did). `ENCODER_SIGN` (default +1) fixes a reversed encoder, and a speed that reads strongly negative while spinning stops the flywheel with a fault. The single-motor `Shooter` on the `shooter-prototype` branch is superseded by `Flywheel`.

**Verification:** **Shot Test** (set a speed, tune `FEED_FORWARD` through the Dashboard, and read the lowest speed during each salvo; the Flywheel Test was deleted) and **Shot Map Tuner** (drive to a distance, find a speed that scores, record the pair, test the interpolation live, and print lines for `ShotMap.TABLE`).

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

**Verification:** **Shot Test** marks HOLD (jog, then B), runs a salvo, a reject and a return to HOLD, and shows where the arm settled after every move; the powers and the early-stop angle are Dashboard values to copy into `Pusher.java`. `TICKS_PER_REV` is confirmed from the Pusher Sensor Matrix logs. The Pusher Test was deleted.

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
3. **Decided:** the flywheel must be up to speed before a salvo starts (first ball only). A Fire pressed too early is queued: it spins up if needed and fires automatically the moment the flywheel is ready and the pusher has stopped (a salvo is a full turn from the nearest HOLD, so it can start from wherever the pusher rests), with a timeout. One button does spin-up and fire.
4. **Decided:** a rejection is a pusher reverse plus an intake outtake, together. The outtake runs for the whole reject move (reverse, then forward back to HOLD), then the intake turns off.
5. **Decided:** a stow requested while the robot is still under a flower is remembered, and the ramp stows automatically once the pose shows the robot has driven out. A warning shows meanwhile.
6. **Decided (2026-10-31):** with the element counter, the intake stops by itself when the robot holds 4 elements (rule G407) and the operator's intake toggle is cleared, so they press it again after a salvo. The outtake still works. Opponent nectar in the robot only raises a warning with its sensor position; reject stays the operator's decision.
7. **Decided (2026-10-31):** overrides for a sensor that fails in a match, shown on screen and in the log, with no stick-button or Start controls on the driver's controller (too easy to press by accident); the operator uses the stick buttons, the driver holds the left bumper as a guard: operator L3 ignores the element counter (no 4-element stop, no element warnings; a warning also appears by itself when the counter stops reading), operator R3 runs the flywheel open loop (power = target / 2200, ready after 1.5 s, speed sensor ignored), driver LB + X drives robot-oriented (ignores the heading), driver LB + B ignores the pose (the ramp stows without waiting to clear a flower). The flywheel's reversed-encoder fault was removed: wiring is the pit check's job.
8. **Decided (2026-11-01):** the pusher check. After a salvo, once the element counter reads 0 and fire, reject and the outtake are idle (the intake may be running), the coordinator runs `PusherCheck`: it reverses the arm about 40 deg to the measured arc center (33 deg behind HOLD), looks for the arm at CS4, and if it is not seen finds it with one continuous slow sweep (not giving up before a full turn), corrects the pusher's zero from a forward and a return crossing, verifies it, and returns the arm to HOLD. Not found gives a LOST warning. Any fire, reject, intake or ramp request aborts it. It can be turned off with `Mechanisms.AUTO_PUSHER_CHECK`. The zero counts as verified once a check has seen the arm where it says (or corrected it); until then (after a power cycle or install, a zero set by hand, or a faked slip) the coordinator also runs one check right after Start, if the track reads empty within 2 s (`Mechanisms.AUTO_STARTUP_CHECK`), and warns "Pusher zero not verified against CS4 yet".

## Element counter (CS1 to CS4)

Four REV Color Sensor V3, one per I2C bus, named `CS1` to `CS4` and numbered along the track: intake, CS1, CS2, CS3, CS4, flywheel (CS4 is deepest and also sees the pusher arm). `ElementCounter` reads **one sensor every 40 ms** (each read is about 13 ms of blocking I2C, and the sensors refresh every 100 ms) and reports, per sensor, EMPTY, POLLEN, RED NECTAR, BLUE NECTAR, OTHER or PUSHER ARM.

- **Presence** comes from the shift of the color shares from each sensor's own empty baseline (the empty views differ per sensor and the contrast against a near wall is small), then alpha and distance, with hysteresis and a debounce.
- **Count** = sensors that see an element (not the arm), reported only after it has been steady (150 ms going up, 300 ms going down). A fourth element is reported about 0.4 s after it is first seen. An element without a neighbor toward the flywheel sets a *gap* flag but still counts.
- **Baseline:** the empty readings in `ElementCounter` were measured 2026-10-31. Lighting changes them, so re-measure in the pit with Color Sensors Test (**A** with the track empty) and copy the BASELINE line into the table. It cannot be captured at match start, because the robot may already hold elements.
- **Measured on the robot:** four pollen read 100% correct on all four sensors (3,298 samples); blue nectar and the pusher arm classified correctly in the logs; red nectar worked up close and was loosened for distance (not yet re-run live). Thresholds are accepted for now.
- **Not done yet:** a live run of the counter inside TeleOp - Competition, and the pusher position check that uses `pusherArmSeen()` on CS4.

Also decided: after a salvo finishes, the flywheel stops automatically a short moment later so the last ball clears, and spins up again only when the next Fire is requested. The integrated TeleOp splits the controls between a driver (gamepad 1, drives the chassis) and an operator (gamepad 2, runs the mechanisms), as last year.

**Status:** the coordinator (`subsystems/Mechanisms.java`) and the competition TeleOp (`opmodes/TeleOpCompetition.java`) implement these rules. Any mechanism the robot does not have yet is simply skipped, so the coordinator works on a partially built chassis. Not yet run on hardware.

## Still to measure or decide

- [ ] Pusher ticks per revolution (calibration step).
- [ ] Ramp servo angles, stowed and deployed, and `TRAVEL_SEC`. Watch whether the ramp lands too hard without a slew.
- [ ] Shot map: distance to speed, measured on the robot.
- [ ] Intake outtake power (starts at -0.5).
- [ ] Whether to add current-based jam detection to the intake.
- [ ] Hard interlock or warning for stowing under a flower.
- [ ] The color sensors: placement and what they read (element count, element color, possibly the pusher tip).
- [ ] Whether to add a hall sensor at the pusher's HOLD position.

## Proposed build order

1. Intake class and bring-up opmode.
2. Flywheel: single motor with an encoder, the shot map, and its tuner.
3. Pusher, including the ticks-per-revolution calibration.
4. Ramp, including measuring the servo angles.
5. A mechanisms coordinator that enforces the cross-actuator rules, then an integrated TeleOp. The color sensors plug into the coordinator later.
