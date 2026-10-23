# BIOBUZZ mechanisms design

Design notes for the four actuators the final robot adds on top of the verified chassis: intake, flywheel, transport gate, and depollinator. Worked out one actuator at a time with the team; each item is marked **Decided**, **Proposed** (my suggestion, not yet confirmed), or **Open**. Nothing here is built yet.

Last updated 2026-10-22.

## Rules that constrain the design

Section numbers are from the BIOBUZZ Competition Manual V1.

| Rule | What it says | Why it matters here |
|---|---|---|
| R102 | The starting configuration must fit an 18 x 18 x 18 in cube (preloaded elements may stick out). | The depollinator ramp, stowed, has to fit. |
| G407 | A robot may control at most 4 scoring elements at once. | The transport holds 4. The intake and ramp must stop feeding at 4. |
| G408 | A robot may not control the opponent's NECTAR. | The gate reverse is how we reject one. |
| G418 | Elements enter a FLOWER only through its top, and only POLLEN may be removed, from its bottom. | The depollinator takes pollen only. |

## Shared architecture

- **Decided:** each actuator is a small class with setters that record a target and one `update()` call per loop, the same pattern as last year's subsystems.
- **Proposed:** autos use the existing `Action` interface and its composition (`SequentialAction`, `ParallelAction`, `SleepAction`, `InstantAction`) in `teamcode.motion`, so each actuator offers action factories such as "spin up and finish when at speed".
- **Proposed:** values that differ per chassis (motor directions, servo positions) live in `ChassisProfile`, which is already chosen automatically by Control Hub. Tunable numbers use `@Config` so they can be edited live in the FTC Dashboard.
- **Proposed:** each actuator gets its own bring-up opmode (group "Bring-up") before it is integrated, in the style of Motor Map Test and the Pinpoint tests. A combined mechanisms coordinator and an integrated TeleOp come last.

## 1. Intake

**Hardware (decided):** one motor, no encoder, open loop, mounted on the BACK of the robot. It takes both pollen (2.8 in) and nectar (3.6 in).

**Behavior**
- **Decided:** it runs only while the robot is actively intaking, never always-on, so it cannot pick up stray balls and push us over the carry limit.
- **Decided:** it switches on automatically when the depollinator ramp lowers.
- **Decided:** power is 1.0 to take elements in and -0.5 to spit out, as last year. Both values need real tuning.
- **Decided:** the gate physically holds only 4 elements. Once the color sensors (future) see 4, a short outtake clears anything still in the intake, as last year.
- **Decided:** jams are handled by the driver for now.
- **Open, still being considered:** current-based jam detection. If the motor current stays high for about a quarter second, reverse briefly and automatically.
- **Open idea:** with one color sensor per slot, the robot could spot an opponent-color nectar (G408) and outtake it automatically.

**Status:** the `Intake` class (`subsystems/Intake.java`) and the bring-up opmode **Intake Test** are built and compile, but have not been run on hardware. The motor direction is in `ChassisProfile.intakeDirection` and is a last-year placeholder (REVERSE) until verified.

**Verification:** Intake Test runs it only while a button is held, at 30% power to start (D-pad steps the scale), confirms direction, and shows the motor current with a peak reading. The peak current with and without an element in the roller is the data a jam threshold would come from.

## 2. Flywheel

**Hardware (decided):** one motor with an encoder (`fly_motor_enc`), very like last year's. A second follower motor with no encoder may be added later, decided from rig data: how far the speed dips and how fast it recovers over a 4-ball burst. The hood is a static angle.

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
- The flywheel only needs to be ready before the first ball. The ball spacing comes from the transport gate's sweep speed.
- It stops between salvos and spins up when needed. In autos the spin-up runs in parallel with the drive to the shooting spot.

**Verification:** `ShooterTest` on the prototype branch, then the shot-map tuner. A 4-ball burst measures the speed dip and recovery.

## 3. Transport gate

**Hardware (decided):** one pusher arm, with some thickness, that rotates around a curved linear track the elements ride along. The back of the arm limits how many elements can enter (the 4 cap). The end of the track feeds the flywheel. The arm passes through the ramp, so it can reach the last element and push the whole stack up toward the flywheel. One motor with a relative encoder. **There are no hard stops: the gate spins continuously and almost always goes forward.**

**Position (proposed from the above)**
- Position is an angle on a circle: ticks divided by the ticks per revolution, with HOLD defined as 0 degrees. The arm's thickness means its pushing face and its limiting face sit at different angles.
- Moves are forward only. The error is the forward distance to the target modulo one revolution. A small overshoot is tolerated, so a gate that stops a few degrees past a target does not command nearly a full extra turn.
- **Open (required):** ticks per revolution is not known yet. It is needed to finish the class. It is a tunable constant, measured by a one-revolution calibration step in the bring-up opmode.

**Behavior**
- **Decided:** full salvos only for now, because we may miss. A salvo is about one forward revolution: from HOLD, sweep through the ramp pushing the stack to the flywheel, then keep going forward back to HOLD.
- **Decided:** the sweep speed is tunable. It sets the spacing between balls. Using the encoder's velocity mode would keep it steady as the battery drains.
- **Decided:** reverse is needed to reject an opponent's nectar (G408), together with an intake outtake.
- **Decided:** assume a known starting position, with the gate at HOLD at power-on.
- **Proposed:** auto zeroes the encoder at init. TeleOp after auto does not, and takes the reference from a static saved by auto, as the pose is. A fresh TeleOp-only session assumes the gate is at HOLD.
- **Proposed:** stall and jam detection by high current with zero speed. A blocked gate does not lose its position, because it simply does not move.
- **Open, later:** a hall sensor at HOLD to re-zero every revolution and check the start position, or the color sensors reading the gate tip as it passes. Color sensors read over a slow bus, so a fast tip could pass between readings, and the tip would need a distinct color.

**Verification:** a bring-up opmode that jogs the gate slowly, shows ticks and current, measures ticks per revolution, then runs moves, a sweep and the stall check.

## 4. Depollinator

**Hardware (decided):** two goBILDA torque servos lower a ramp so pollen from a FLOWER's bottom opening rolls straight into the intake. The flower's retrieval opening is about 3.55 in tall and 3.57 in deep. The pollen sits in a 2.79 in hole in a ring 0.43 in thick.

**Behavior**
- **Decided:** two positions only, STOWED and DEPLOYED.
- **Decided:** the driver or operator decides when to deploy and stow, for now. An automatic approach to a flower pickup pose is the long-run goal.
- **Decided:** deploying switches the intake on. Stowing switches it off unless the driver is holding intake.
- **Decided:** the robot must drive out from under the flower before the ramp can stow.
- **Open:** whether a stow request made while still under the flower is ignored (hard interlock) or only warned about. The check would compare the Pinpoint pose against the four known flower positions.
- **Proposed:** each servo has its own stowed and deployed positions in `ChassisProfile`, mirrored by direction. Both servos must be set to the same mode (goBILDA dual-mode servos can be positional, continuous or multi-turn). The deployed position is commanded just short of contact, so a torque servo is not stalled against the floor. Motion is rate-limited so the ramp lowers gently, and the travel time can be computed without feedback.
- **Open:** the stowed and deployed angles are not known yet. A bring-up opmode jogs each servo and records them.

**Rules:** the stowed ramp must fit the R102 cube. Only pollen is removed (G418).

## Cross-actuator rules (proposed)

1. The intake runs only while the gate is at HOLD, because HOLD is what limits capacity to 4.
2. Deploying the depollinator turns the intake on. Stowing turns it off, unless the driver is holding intake.
3. The flywheel must be up to speed before a salvo starts (first ball only).
4. A rejection is a gate reverse plus an intake outtake, together.
5. Stowing is blocked or warned about while the robot is not clear of a flower.

## Still to measure or decide

- [ ] Gate ticks per revolution (calibration step).
- [ ] Depollinator servo angles, stowed and deployed.
- [ ] Shot map: distance to speed, measured on the robot.
- [ ] Intake outtake power (starts at -0.5).
- [ ] Whether to add current-based jam detection to the intake.
- [ ] Hard interlock or warning for stowing under a flower.
- [ ] The color sensors: placement and what they read (element count, element color, possibly the gate tip).
- [ ] Whether to add a hall sensor at the gate's HOLD position.

## Proposed build order

1. Intake class and bring-up opmode.
2. Flywheel: single motor with an encoder, the shot map, and its tuner.
3. Transport gate, including the ticks-per-revolution calibration.
4. Depollinator, including measuring the servo angles.
5. A mechanisms coordinator that enforces the cross-actuator rules, then an integrated TeleOp. The color sensors plug into the coordinator later.
