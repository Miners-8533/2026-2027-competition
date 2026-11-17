# Competition TeleOp

`TeleOp - Competition`: the driver-controlled period. A driver (gamepad 1) moves the chassis; an operator (gamepad 2) looks after the mechanisms. Most of what happens is automatic, and this page says what.

Building blocks: [drive.md](drive.md), [mechanisms.md](mechanisms.md), [aiming.md](aiming.md), [dashes.md](dashes.md), [motion.md](motion.md) (hold position).

## 1. INIT

Everything is built during INIT. **Nothing is driven**: no motor, no servo. Only the Pinpoint and the slot count are read.

### 1.1 Start pose

1. If a pose was saved in the last 300 s (by an auto, or by a TeleOp that was stopped), take the alliance from it and **adopt** the Pinpoint's own pose ([drive.md](drive.md)). The robot may be anywhere.
2. Otherwise **reset and seed** the default start for the alliance: blue (-63.4, 16.0, 0 deg), back against the rear wall; red (63.4, -16.0, 180 deg), back against the audience wall. The robot must be on that spot and still from INIT until Start.

The x of the default start is computed from the wall position and the robot's origin-to-back distance.

### 1.2 Controls during INIT (gamepad 1)

| Control | Effect |
|---|---|
| D-pad right / left | choose BLUE / RED. Only when no saved pose is in use; a saved pose fixes the alliance |
| D-pad up | discard the saved pose and use the default start (the alliance choice then works) |
| X | switch between **FULL AUTO** and **COUNT ONLY** (default FULL AUTO; kept until the app restarts) |

Changing the alliance or discarding the saved pose re-seeds the position on the device at once, without recalibrating its IMU.

### 1.3 INIT screen

It reports: where the start pose came from and how old a saved pose is; the Pinpoint pose and whether it is acceptable; which mechanisms were found; that the ramp position is unknown; what the flywheel position servo will do; the FULL mode; whether the slot count is reading; whether both gamepads are assigned; and a warning if no hub is registered.

The pose is **acceptable** when, after an adoption, the device reports READY, or, after a seed, the pose is within 1.5 in and 3 deg of the start pose and the seed was confirmed. After an adoption, a difference of more than 3 in or 5 deg from the saved pose is flagged for the drivers to check.

### 1.4 At Start

In this order: stow the ramp; command the flywheel position to POLLEN; set the target cell to the one nearest the start; switch on "target follows the pose", "intake always on" and "flywheel runs from Start"; allow the FULL actions if FULL AUTO was chosen; allow the launch grid (off unless switched on). The setup is logged: profile, alliance, start pose, mechanisms found, FULL mode, gamepad assignment.

## 2. Controls

Neither controller uses Start or Back. The driver's stick buttons are unused. Anything risky needs a second button held.

### 2.1 Driver (gamepad 1)

| Control | Effect |
|---|---|
| Left stick | drive, field-oriented: up is away from the driver, right is the driver's right |
| Right stick X | turn |
| Left trigger (hold) | precision: translation and turn scaled to 30 percent |
| A | launch a salvo. Also cancels a running reject |
| Right bumper (hold) | target lock |
| Left bumper + right stick X, while locked | nudge the aim, up to 15 deg either way |
| D-pad left / right (hold) | Hive Dash to the cell on the driver's left / right |
| D-pad up / down (hold) | Park Dash to the park corner on the driver's left / right |
| Left bumper + right trigger (hold 1 s) | reseed the pose to the default start |
| Left bumper + X (hold 1 s) | enter backup mode |

### 2.2 Operator (gamepad 2)

| Control | Effect |
|---|---|
| Left trigger (hold) | intake OUT |
| Right trigger (hold) | unlock a locked pusher and keep the intake IN through a FULL sequence |
| Left bumper + B | reject. Also cancels a queued launch or a running salvo |
| X | deploy the ramp |
| Y | stow the ramp, at once |
| Left bumper + Y | pusher calibration sweep; again to stop it |
| Right bumper | auto-launch on / off (off at Start) |
| D-pad left / right | report: the last salvo landed LEFT / RIGHT |
| D-pad down / up | report: the last salvo landed LOW / HIGH |
| Left bumper + D-pad left or right | clear the remembered aim of the last-launched cell |
| Left bumper + D-pad up or down | clear the speed trim |
| Left bumper + A | switch the launch grid on / off (a test control) |

There is no control for the intake (always in), the flywheel (always running), the target cell (always the nearer one), or the pusher lock.

## 3. Driving

**Field-oriented.** The stick is interpreted in the field frame using the direction the alliance's drivers face (blue -90 deg, red +90 deg), then rotated into the robot frame by the current heading. Turn stick power is scaled by 0.5. Precision mode scales translation and turn by a further 0.30.

**Who commands the chassis.** Each loop the translation and the turn are decided separately, later rows overriding earlier ones.

| Priority (low to high) | Translation | Turn |
|---|---|---|
| 1 | the driver's stick | the driver's turn stick |
| 2 | a Hive Dash that is driving | the Hive Dash, until the lock takes over |
| 3 | a Park Dash that is driving | the Park Dash |
| 4 | zero, inside an auto-launch zone | the target lock, whenever it is on |
| 5 | zero, during the drive hold of a salvo | |
| 6 | hold position, when active | hold position, only when the lock is off |

## 4. Target lock

The lock is on while any of these holds, and never in backup mode:

- the driver holds the right bumper;
- a Hive Dash asks for it;
- (switched off) the robot is full by the slot count.

While on, the robot aims at the current target cell as in [aiming.md](aiming.md), using the far-cell extra for that cell and the operator's remembered correction for it. The driver's turn stick only nudges, and only with the left bumper held.

Log lines mark the lock coming on (and why), becoming aligned, and going off.

## 5. Launching

There are four ways a salvo is requested. Each logs a full aim line: pose, cell, distance, bearing, predicted miss, setpoint and trim, lock state, the remembered correction, and what the launch grid would say.

1. **Driver A.** Launches at once, with any number of elements, locked or not. If the lock is on but not yet aligned, the launch **waits** for aligned, at most 1.5 s; it is dropped if the lock is released or the time passes.
2. **Hive Dash**, on arrival ([dashes.md](dashes.md)).
3. **Auto-launch**, when armed (section 6).
4. From a FULL robot, by way of auto-launch being armed (section 7).

What happens after the request is the coordinator's queued launch ([mechanisms.md](mechanisms.md)).

### 5.1 Operator reports

After a salvo the operator can report where it landed, as seen from behind the robot. A report applies to the cell of the **last launch**.

| Report | Effect |
|---|---|
| Landed LEFT | that cell's remembered aim turns 3 deg clockwise |
| Landed RIGHT | that cell's remembered aim turns 3 deg counterclockwise |
| Landed LOW | speed trim +10 ticks/s |
| Landed HIGH | speed trim -10 ticks/s |

- The remembered aim is kept per cell, limited to 10 deg either way, used by every later lock on that cell, and cleared for all cells by a driver reseed (it was correcting the drift of the old pose). This is the team's chosen answer to drift, which cannot be predicted in a match. Target T1 ([aiming.md](aiming.md) section 5) keeps the remembered aims through a TeleOp restart and shows both of the alliance's cells' trims on the match screen.
- The speed trim is one value shared by all cells.
- Every report is logged together with the aim line of the launch it refers to.

## 6. Auto-launch

**Armed** when the operator has switched it on, or when the robot is FULL in FULL AUTO mode. Never in backup mode.

While armed and the target lock is on:

1. Nothing happens if the robot reads empty (both counts read and both hold nothing).
2. When the robot enters the launch zone of the locked cell, the driver's translation is ignored and the chassis stops.
3. Once the lock is aligned and the robot has been slower than 4 in/s for 0.15 s, one salvo is requested.
4. If that has not happened 2.5 s after entering the zone, auto-launch gives up for this lock and the driver has the robot back.
5. It acts at most once per lock. Releasing the lock re-arms it.

## 7. FULL AUTO and COUNT ONLY

Chosen at INIT; not switchable after Start.

| | FULL AUTO | COUNT ONLY |
|---|---|---|
| Entry count shown and logged | yes | yes |
| FULL sequence (intake, pusher lock) | yes ([mechanisms.md](mechanisms.md)) | no; the intake stays IN |
| Auto-launch armed by FULL | yes | no |
| Operator's own auto-launch toggle | works | works |

COUNT ONLY reproduces how the robot behaved at Hartland.

## 8. Automatic motion control around a salvo

**Drive hold during the sweep.** From the start of a salvo until the pusher has turned 360 deg past the HOLD it started from (about 1.0 to 1.3 s):

- the driver's translation is ignored;
- hold position brakes the robot and then holds its pose, and the sticks do not cancel it;
- the turn is not affected, so the lock and the nudge keep working;
- a fail-safe releases it after 2 s, so a jammed pusher cannot hold the driver;
- it does not apply while a launch is only queued, nor in backup mode.

Reason: salvos fired while moving shifted the robot several inches during the launch, which is several degrees of aim. This is a build-time switch, currently on. A log line records how long it lasted, how far the robot moved and how hard the driver pushed.

**Hold position.** Wanted while a launch is queued or a salvo runs, or while a Park Dash is parked. Cancelled by the driver moving a stick (the turn stick counts only when the lock is off), by a dash driving, by a Pinpoint fault and in backup mode. It holds the heading too when the lock is off. See [motion.md](motion.md).

## 9. Reseed

Holding the driver's left bumper and right trigger for 1 s sets the pose to the alliance's default start pose. The robot should be on that spot. It also clears every remembered aim. The screen counts the hold down. It is the fix for a drifted pose or a wrong heading. Not available in backup mode.

## 10. Backup mode

For when the pose or heading cannot be trusted. **One way: it lasts until the opmode ends.**

**Entered** by the driver holding left bumper + X for 1 s, or automatically when the Pinpoint has reported a fault status, or thrown a read error, for 0.5 s. A shorter fault is logged as a blip and nothing changes.

**Effects.**

- Drive becomes robot-oriented: stick up is the robot's front.
- No target lock, no dashes, no auto-launch, no reseed, no drive hold, no hold position.
- The pose is no longer read or saved.
- The flywheel speed is the launch map's speed for a fixed 42 in plus the speed trim. The driver drives to that distance to launch.
- The screen shows BACKUP MODE in red.

## 11. Driver screen

Two screens, switched by a Dashboard value: the **match screen** (default) and a **verbose screen** with every value and the full mechanism readout, for the pit.

The match screen uses the Driver Station's HTML display mode, which is the only mode that formats text. It has about 15 lines of about 36 characters, and nothing may wrap. From the top:

| Line | Content |
|---|---|
| Alerts | at most three, red first. Blank when all is well |
| Alliance | `BLUE` or `RED`, the largest text, in the alliance color |
| Flywheel and count | `READY` (green), `SPIN UP` or `SLOWING` (yellow), `NO FLY` (red); then `N HELD`, green when full, or `? HELD` in yellow when CS5 is not answering |
| Activity | only when there is one (below) |
| Target | `REAR 36 in` or `AUDIENCE 34 in`; `FIXED 42 in` in backup mode |
| Pose | x, y, heading |
| Small lines | `N SHOT`; `PUSHER LOCKED`; `COUNT ONLY`; the speed trim when not zero |

- `READY` means the flywheel is in the launch band.
- `N HELD` is the CS5 entry count, never the slot count.

**Activity**, first match wins: `DASH: REAR` / `DASH: AUDIENCE`, `DASH: AIMING`, `DASH: LAUNCHING`; `PARK DASH`, `PARKED`; `LAUNCHING`; `WAITING FOR AIM`; `REJECTING`; `LOCKING`, `LOCK ALIGNED` (green).

**Alerts.**

| Color | Alerts |
|---|---|
| Red | `BACKUP MODE`, `PINPOINT FAULT`, `DRIVE WHEEL LB?` (a wheel drawing too little current), `FLYWHEEL LOST`, `FLY OPEN LOOP`, `PUSHER STALLED`, `PUSHER NOT CALIBRATED`, `PUSHER JAMMED` |
| Yellow | `BACKUP IN 0.6 s` and `RESEED IN 0.4 s` (hold countdowns), `DASH REFUSED`, `PARK DASH REFUSED`, `CS5 NOT ANSWERING`, `PUSHER ZERO?`, `OPPONENT NECTAR: REJECT`, `COUNTER NOT READING`, `LAUNCH MAP EMPTY`, `NO SHOT ZONE`, any current notice (cut to 26 characters), `CHECK ROBOT (verbose)` for a warning with no alert of its own |
| Cyan | `DRIVE HELD`, the FULL sequence step, `AUTO-LAUNCH ARMED`, `LAUNCH GRID ON` |

The alerts are chosen from the coordinator's warning **codes**, not their wording. Four warnings are deliberately not shown: full by the slot count, stow waiting, zero not verified yet, and the coordinator's own backup warning.

The controllers have no rumble, so every signal to the drivers is on this screen.

## 12. Loop order

Each loop, in this order:

1. Read the pose (unless in backup mode) and watch for a Pinpoint fault; save the pose.
2. Read the driver's sticks.
3. Update the Park Dash, then the Hive Dash; apply the Hive Dash's cell override and launch distance.
4. Decide the target lock and compute its turn; run auto-launch.
5. Decide the drive hold; update hold position.
6. Send the drive command.
7. Handle the hold chords (backup, reseed).
8. Read the operator's controls and the driver's launch; request launches, rejects, ramp moves.
9. Update the mechanisms.
10. Give the run log its tick; draw the screen.

On stop: stop the drive, then stop all mechanisms (not a final mechanisms update).

## 13. Logging

Kind TELEOP ([logging.md](logging.md)). Sample columns: pose; the driver's stick; the drive command; drive mode (field, robot or backup); lock error; the coordinator's columns; the four wheel currents; the launch grid's zone, speed and residual. A crash is written to the log before it is re-thrown.

## 14. Parameters

| Name | Value |
|---|---|
| Saved pose freshness | 300 s |
| Default start y / heading (blue) | 16.0 in / 0 deg |
| Driver forward direction: blue / red | -90 deg / +90 deg |
| Turn scale / precision scale | 0.5 / 0.30 |
| Trigger threshold | 0.3 |
| Launch waits for aligned, at most | 1.5 s |
| Report: aim step / aim limit / speed step | 3 deg / 10 deg / 10 ticks/s |
| Reseed hold / backup hold | 1.0 s / 1.0 s |
| Backup: Pinpoint fault for / fixed distance | 0.5 s / 42 in |
| Drive hold during the sweep / fail-safe | on / 2.0 s |
| Auto-launch armed by FULL | on |
| Automatic lock when full (slot count) | off |
| Sample period | 100 ms |

## 15. Status

Used at Hartland: field-oriented drive, the lock on the right bumper, launch waiting for aligned, the dashes, the operator's auto-launch, the reports, hold position, backup mode, reseed, the match screen, the Pinpoint adoption. Added after Hartland: the drive hold during the sweep (run in practice, not at speed), FULL AUTO with its auto-launch arming (the present sequence has not run), COUNT ONLY, the launch grid switch.
