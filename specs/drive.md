# Drive base and localization

The lowest layer that moves the robot and knows where it is. Everything above it (the TeleOp, the autos, the dashes, the aim) reads one pose per loop from here and sends one drive command per loop to here.

Hardware: [robot_design/chassis.md](../robot_design/chassis.md). Frames: [field.md](field.md).

## 1. Chassis profile

**Purpose.** One build runs on every chassis the team has. Everything that differs between chassis is a named profile, chosen automatically from the identity of the Control Hub.

**A profile holds:** the drive motor directions (left side, right side); the two odometry pod directions and offsets; the footprint (depth, width, origin distance from the back edge); the intake, flywheel and pusher motor directions; the four ramp servo positions (stowed and deployed, left and right).

**Selection rules.**

1. Read the hub's hardware serial and its Android id.
2. If either matches an id registered to a profile, use that profile.
3. If no profile has any id registered at all (first-time setup), use the default profile and report "no hub registered yet" on INIT screens.
4. Otherwise stop the opmode with an error that lists the hub's ids. A known robot must never run with another chassis's directions.

Only `ROBOT_1` has ids registered, so rule 4 is in force for every other hub.

## 2. Mecanum drive

**Purpose.** Turn one robot-frame command into four wheel powers, and own the localizer.

**Command.** Three numbers from -1 to 1: forward, left, and counterclockwise turn. This is an open-loop power command; there is no wheel speed control.

**Mixing.** As in [robot_design/chassis.md](../robot_design/chassis.md): left front = f - l - t, left back = f + l - t, right back = f - l + t, right front = f + l + t. If the largest magnitude exceeds 1, all four are divided by it. A global power scale (default 1.0) multiplies the result.

**Hub write saving.** Each motor write blocks the loop for a few milliseconds, so a wheel power is sent only when it matters:

| Rule | Value |
|---|---|
| Skip a power within this of the last one sent to that wheel | 0.01 |
| Always send a stop (zero after non-zero) | yes |
| Re-send at least this often even if unchanged | every 200 ms |

**Other behavior.**

- All hubs are put in automatic bulk-read mode when the drive is built.
- The drive remembers the last command it was given, for the run log.
- The drive is built in one of two ways: *reset and seed* (the autos and test opmodes) or *adopt* (the TeleOp after an auto). See section 4.

## 3. Drive wheel check

**Purpose.** Notice a drive wheel that is not loaded (a broken or slipping belt). Added after a belt broke in a match and nothing on the robot showed it.

**Sampling.** Every 100 ms, read the motor current of **one** wheel, taking the wheels in turn and skipping any wheel commanded below 0.3 power. A robot standing still is never read. Each read costs 2 to 3 ms.

**Judgement, per wheel.**

1. Compute amps per unit of commanded power and keep a running average (a new sample weighs 0.3).
2. A wheel is judged only after 6 samples, against the median of the other wheels that also have 6.
3. A wheel whose average stays below 45 percent of that reference for 4 of its own samples in a row is **flagged**.
4. The flag clears when the wheel is back above 70 percent.

**Outputs.** A log line when a wheel is flagged and when it recovers; a short screen text naming the flagged wheels (for example `DRIVE WHEEL LB?`); the last current of each wheel as log columns.

Setting the sampling period to 0 turns the check off.

**Status:** in the Hartland build; the flagging has not been exercised with a real broken belt.

## 4. Localization

**Purpose.** One field pose and one field-frame velocity per loop, from the Pinpoint.

**Contract.** `update` is called exactly once per loop, before anything reads the pose. It returns the velocity (inches per second in field x and y, radians per second) and stores the pose (inches, heading wrapped to (-180, 180] degrees). `setPose` forces the estimate and writes it to the device.

The Pinpoint keeps its pose for as long as it is powered, across opmodes. That allows two ways of bringing it up.

### 4.1 Reset and seed

Used by every auto, every test opmode, and a TeleOp with no fresh pose from an auto. **The robot must be still.**

1. Write the pod offsets, pod type and pod directions from the chassis profile.
2. Command the device to zero its position and recalibrate its IMU.
3. Wait at least 0.3 s, then until the device reports READY, at most 2 s. (A pose written before the reset finishes is lost.)
4. Write the start pose and read it back; repeat up to 5 times until it reads within 0.5 in and 2 deg.
5. Report whether the seed was confirmed. An unconfirmed seed means the pose is not trustworthy and INIT screens say so.

### 4.2 Adopt

Used by the TeleOp when an auto saved a pose recently (section 5). **Nothing is written to the device**, so the pose it kept through the transition, including any bump, is used as it is and the IMU is not recalibrated.

The device's pose is adopted unless one of these holds, in which case the device is reset and seeded with the saved pose instead:

| Check | Meaning |
|---|---|
| Not READY within 0.5 s | the device is not answering |
| A reading is not a number | bad data |
| It reads within 1 in and 2 deg of (0, 0, 0) while the saved pose is more than 3 in or 5 deg from zero | the device restarted |
| It reads more than 6 in outside the field walls | nonsense |

The result is one of three modes, each with a one-line note for the INIT screen and the log: `RESET_AND_SEED`, `ADOPTED`, `RESEEDED_AFTER_FAILED_ADOPT`. The note carries the device status, how long READY took, the device pose, its loop rate and firmware, both encoder counts, and how far the saved pose differs.

**Status:** used at Hartland.

## 5. Pose hand-over between opmodes

**Purpose.** An auto hands its final pose and alliance to the TeleOp.

- One saved pose, one alliance flag and a save time, held in memory for the life of the robot app. An app restart empties it.
- A saved pose is used only while it is **fresh**; the caller states the oldest age it accepts (the TeleOp uses 300 s). A pose left from an hour-old practice run must not start a match.
- Autos save at their end. The TeleOp saves every loop, so a TeleOp restarted mid-match resumes where the robot is.
- The TeleOp stops saving in backup mode, when the pose is no longer trusted.

## 6. Parameters

| Name | Value | Meaning |
|---|---|---|
| Power scale | 1.0 | global multiplier on wheel powers |
| Write deadband | 0.01 | smallest change of a wheel power that is sent |
| Refresh | 200 ms | longest time a wheel goes without a write |
| Current check period | 100 ms | one wheel current per period; 0 = off |
| Check: minimum wheel power | 0.3 | below this a sample is ignored |
| Check: low fraction / recover fraction | 0.45 / 0.70 | of the other wheels' median |
| Check: samples before judging / to confirm | 6 / 4 | |
| Seed tolerance | 0.5 in, 2 deg | read-back after seeding |
| Seed attempts | 5 | |
| Reset wait | 0.3 s minimum, 2 s maximum | |
| Adopt READY wait | 0.5 s | |
| Pose freshness for the TeleOp | 300 s | |
