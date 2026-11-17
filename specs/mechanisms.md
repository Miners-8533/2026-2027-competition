# Mechanisms coordinator

The one object every opmode talks to for the intake, flywheel, pusher, ramp and both element counts. It owns the rules that involve more than one actuator. The actuators themselves are specified in [intake-and-ramp.md](intake-and-ramp.md), [flywheel.md](flywheel.md) and [pusher.md](pusher.md); the counts in [element-counting.md](element-counting.md).

## 1. Shape

- Built from whatever the hub configuration contains. A missing mechanism is simply absent and every rule skips it, so the coordinator works on a partly built robot.
- An opmode issues **commands** (requests), then calls **update** once per loop with the current pose. Update decides what each actuator does this loop, then updates each actuator. In an auto, update is the every-loop work of the action runner.
- **Stop all** is for the end of an opmode. It drops anything queued or running and switches the intake, flywheel and pusher off at once. It replaces the last update: an update after it could restart the intake or start a queued salvo.

**Per-opmode settings.**

| Setting | Competition TeleOp | Autos | Default |
|---|---|---|---|
| Intake always on | yes | yes | no |
| Flywheel stops after a salvo | no (runs from Start) | no (runs from Start) | yes, 0.5 s after |
| Target cell follows the pose | yes | no (each launch names its cell) | no |
| FULL actions allowed | yes, if FULL AUTO was chosen at INIT | no | no |
| Launch grid allowed | yes | no | no |
| Jam retries | 2 | 1 | 2 |
| Ramp commanded at INIT | no | yes | yes |
| Flywheel position POLLEN commanded | at Start | at INIT | first update |

## 2. Launch speed

Each update computes one **setpoint** for the flywheel.

1. **Target cell.** If "follows the pose" is on: the nearer of the alliance's two cells, switching only when the other cell is 6 in closer (so it does not flip-flop). Otherwise the cell the opmode set.
2. **Distance.** In order of priority: the backup mode's fixed distance; the Hive Dash's launch-spot distance while a dash runs; else the distance from the pose to the target cell.
3. **Setpoint.** Launch map speed for that distance plus the operator's **speed trim**.
4. **Launch grid** (only when allowed and switched on, never in backup mode): if the grid has a value for the spot, its speed plus the trim replaces the map's. During a dash the grid is looked up for the launch spot.
5. **Flywheel encoder lost:** the setpoint is the flywheel's base speed plus the trim, whatever the distance.

Setting the dash distance at the start of a dash sends the flywheel straight to the speed it will need on arrival, instead of following the changing distance along the route and arriving above the launch band.

**The flywheel runs** while any of these holds: spin requested, a launch queued, a salvo running, or a stop pending. Otherwise its target is 0.

## 3. Launching a salvo

**Request.** A launch request is refused if the flywheel or pusher is missing or the pusher is faulted, and ignored if one is already queued or running. Otherwise:

- a running pusher check is aborted;
- a running reject is **cancelled** (this is the operator's way out of a jam in that direction);
- a reject waiting for an unlock is cancelled;
- the launch is **queued** and a 3 s timer starts.

**Queued.** Each update while queued:

1. Send the flywheel to the setpoint.
2. The salvo starts when **all** hold: the flywheel is in the launch band (96 to 104 percent, filtered), the pusher is stopped (wherever it rests), and the flywheel position servo has finished travelling.
3. If the 3 s pass first, the launch is dropped with a notice that says which condition failed.

The first time a queued launch waits on the band, a log line says whether the speed is above or below it.

**Start of a salvo.**

- The pusher lock is released (the salvo sweeps from the lock position).
- The pusher starts its salvo.
- The **launched tally** grows by the number taken to be in the robot: the higher of the CS5 entry count and the CS1 to CS4 count, at most 4. Both undercount and neither is trusted alone.
- The CS5 entry count is reset to 0, which ends FULL.

**The band is checked once.** A salvo that has started is never interrupted by the flywheel speed.

**End of a salvo.** When the pusher is stopped again:

- the salvo counter increases (the TeleOp watches it to release the target lock);
- a log line records the lowest flywheel speed and where the arm settled;
- a pusher check is requested (section 6);
- if "stops after a salvo" is on, the flywheel stops 0.5 s later.

**A salvo that did not complete** (the pusher gave up on a jam or the timeout, or faulted): the entry count and the launched tally are put back as if nothing left the robot, plus whatever entered since. A robot that shows too many is launched or rejected again; that is safer than one that shows 0 while jammed and keeps collecting.

While a launch is queued or a salvo runs, and for 0.6 s after, the run log writes a row on every loop.

## 4. Reject

A reject is a pusher reverse together with an intake outtake.

- A running pusher check is aborted.
- If the pusher is locked or locking: unlock first, cancel any queued launch, and start the reject by itself when the unlock finishes.
- A queued launch or a running salvo is **cancelled** by a reject (the operator's way out of a jam).
- The intake runs OUT for the whole reject move.
- When the pusher stops, the reject is done and the CS5 entry count is reset.

## 5. Intake

The intake mode is decided fresh each update. Later rules override earlier ones.

**With "intake always on" (TeleOp and autos).**

1. IN.
2. During the FULL sequence (section 7, TeleOp with FULL AUTO only): whatever the sequence says.
3. OUT while a reject runs or the manual outtake is held.
4. IN if an auto has forced the intake in (section 9), unless rule 3 applies.
5. OFF during an operator calibration sweep of the pusher.

**Without it (the default, used by some test opmodes).**

1. IN if the intake was requested or the ramp is deployed.
2. A full robot by the CS1 to CS4 count stops it, clears the request and raises a notice.
3. OUT while a reject runs or the manual outtake is held.
4. OFF while the pusher is locked or locking.
5. OFF during a calibration sweep.

Every change of mode is logged. The stretches during which the intake ran OUT are remembered so that elements leaving past CS5 are not counted as entries.

## 6. Automatic pusher check

**When it is wanted.**

- After every finished salvo.
- Once after the first update of an opmode, if the pusher's zero has not been verified since it was assumed (a power cycle, an install, or a zero set by hand).

**When it starts.** Within 2 s of being wanted, as soon as all hold: nothing else is busy (no launch queued, salvo, reject, manual outtake or counter override), the CS1 to CS4 count reads 0, the pusher is stopped, and 0.4 s have passed. Otherwise it is skipped with a log line that says why. It is also skipped if the pusher gave up on its last move.

**While it runs.**

- Anything busy, or an element appearing in the count, aborts it.
- CS4's reading is kept out of the element count; during a sweep all four sensors are (the arm passes each).
- For 0.8 s after it ends the sensors stay out of the count.

**Warnings.** "Zero not verified yet" until a check succeeds; "zero uncertain" after a check ends LOST.

**Operator calibration sweep.** Refused, with a notice saying why, unless the counter reads and holds nothing, nothing is busy, and the intake is not requested and the ramp not deployed. Pressing it again while it runs stops it. The intake is held off until it ends.

> In the always-on TeleOp the track rarely reads empty right after a salvo, so in practice the automatic check is often skipped and the arm rests 40 to 60 deg past HOLD between salvos.

## 7. FULL and the pusher lock

### 7.1 Declaring FULL

The robot is **FULL** when 4 elements have entered, by the CS5 entry count, since the track was last emptied. The count restarts at 0 when a salvo starts or a reject finishes. An entry whose peak fell while the intake was running OUT (or within 0.4 s after) is not counted.

The CS1 to CS4 count plays no part in declaring FULL: those sensors do not see every element when the track is crowded.

FULL is always declared and logged. Whether anything **acts** on it depends on the opmode: the competition TeleOp in FULL AUTO mode acts; COUNT ONLY mode and every other opmode do not.

### 7.2 The FULL sequence (TeleOp, FULL AUTO)

| Step | Intake | Pusher | Screen text |
|---|---|---|---|
| 0. For 0.1 s after FULL | IN | still | `FULL: INTAKE RUNNING ON` |
| 1. For the next 0.1 s | OUT | still | `FULL: REVERSING` |
| 2. Until the lock is reached | OFF | moves to the lock | `FULL: LOCKING` |
| 3. Locked | OUT | locked | `FULL: LOCKED, INTAKE OUT` |

- Step 0 lets the fourth element, which was counted as it passed the sensor, finish entering.
- Step 2: the lock goes forward from wherever the arm rests, as long as that is between 8 deg before a HOLD and 130 deg past it.
- **If the lock is refused, the intake goes back IN** (`FULL: LOCK REFUSED`). A stopped intake with the arm open lets the elements roll out.
- The operator's right trigger overrides the whole sequence: the intake runs IN and a locked pusher unlocks.
- A salvo (which goes from the lock) or a reject (which unlocks first) ends FULL.
- FULL also arms the TeleOp's auto-launch ([teleop.md](teleop.md)).

**Variant, switched off.** If "intake OUT while locked" is turned off, the intake stays stopped in step 3 and a *fifth-element watch* runs instead: once CS5 has read empty for 0.5 s (so an element resting there is one of the four), a reading 8 counts above empty for 0.3 s sends the intake OUT for at least 0.6 s, until CS5 has read within 3 counts of empty for 0.3 s; it gives up after 3 s and waits 2 s before trying again.

### 7.3 The pusher lock

Lock states: none, locking, locked, unlocking.

**A lock is refused when:** the pusher or the counter is missing; the robot is busy; the pusher is faulted or gave up on its last move; a pusher check is running or wanted; the lock has already failed 2 times since the last salvo, reject or new request; or the arm is not in a position to lock from.

**Locking.** The CS1 to CS4 count is frozen (the arm will sit in view of sensors). When the arm stops, it is *locked* if within tolerance of the lock angle; otherwise the attempt has failed, the count is unfrozen, and it may be tried again up to the limit.

**Unlocking** happens when: the reason for the lock has ended (after 0.5 s, and no launch is pending); the operator forces the intake in; or a reject is requested. When the arm stops, the count is unfrozen.

**A second reason to lock exists but is switched off:** locking whenever the TeleOp has a target lock or a Hive Dash active and something is held. It is disabled by a master switch because the TeleOp intake is always in.

**Preload pre-position (autos).** At the very start of an auto the arm is moved to the lock angle while the flywheel spins up, so the preload salvo turns 310 deg instead of 400 and finishes about 0.3 s sooner. This calls the pusher's lock move directly and is independent of the switches above.

## 8. Ramp

- **Deploy:** at once.
- **Stow now:** at once, with no check. The TeleOp and the autos use this.
- **Stow (checked):** remembered and carried out when the robot's origin is clear of every flower, or at once if the pose is not trusted. A warning shows while it waits. No competition opmode uses this at present.
- Deploying the ramp turns the intake on in the non-always-on mode.

## 9. Commands for autos

Each is an action ([motion.md](motion.md)).

| Action | Finishes when |
|---|---|
| Launch | the queued launch has completed or been dropped |
| Reject | the reject has completed |
| Set intake on or off | same loop |
| Force intake in, on or off | same loop. While on, the intake runs IN whatever any count says; a reject or outtake still wins |
| Deploy | the ramp has settled |
| Stow now | the ramp has settled |
| Stow (checked) | the stow has happened and settled |

## 10. Backup mode

Entered by the TeleOp when the pose cannot be trusted; one way until the opmode ends. The launch distance becomes a fixed value given by the TeleOp, the target cell stops following the pose, the launch grid is not used, and a warning shows. The speed trim still works.

## 11. Warnings and notices

- A **warning** is a standing condition, recomputed every update. Each has a code and a sentence. The codes are what other software acts on (the competition screen); the sentences are for the log and the verbose screen. Each warning is logged when it appears and when it clears.
- A **notice** is a one-off message shown for 4 s and logged.

| Warning code | Condition |
|---|---|
| Counter override | the element counter is being ignored |
| Counter not reading | a sensor of CS1 to CS4 has stopped answering |
| Full | 4 held by the CS1 to CS4 count |
| Opponent nectar | the other alliance's nectar is at a named sensor; reject is the operator's decision |
| Launch map empty | no speed source; the manual setpoint is in use |
| Flywheel encoder lost | open loop at a fixed power |
| Flywheel open loop | the manual open-loop switch is on |
| Ramp not configured | servo positions missing |
| Backup mode | no pose |
| Stow waiting | a checked stow is pending |
| Pusher not calibrated / stalled / jammed | as named |
| Pusher zero not verified / uncertain | see section 6 |

## 12. Dormant features

These exist in the coordinator but no control reaches them in the competition opmodes: the counter override, the pose override as a separate switch, the manual flywheel open loop, the manual setpoint (the launch map is never empty), the intake toggle, the spin toggle, and clearing a pusher fault (faults are off).

## 13. Parameters

| Name | Value |
|---|---|
| Launch timeout | 3.0 s |
| Flywheel stop delay after a salvo | 0.5 s |
| Notice duration | 4.0 s |
| Target cell hysteresis | 6 in |
| Burst logging after a salvo | 0.6 s |
| Empty-track health line | every 1.0 s |
| Pusher check: wait / settle / count hold-off after | 2.0 s / 0.4 s / 0.8 s |
| Arm near CS4: behind HOLD / ahead of HOLD | 70 deg / 70 deg |
| Entry capacity | 4 |
| FULL: intake runs on / reverses | 0.1 s / 0.1 s |
| Lock: release delay / maximum tries | 0.5 s / 2 |
| Lock for a target lock or dash | off |
| Fifth watch (variant, off): arm after / dwell / clear | 0.5 s / 8 counts for 0.3 s / 3 counts for 0.3 s |
| Fifth watch: minimum out / give up / retry after | 0.6 s / 3 s / 2 s |
| Manual setpoint (unused) | 1350 ticks/s |

## 14. Log columns

Every sample row of the run log carries the coordinator's state: setpoint, flywheel target, speed and ready flag; pusher state, angle, speed and current; intake mode; ramp target; CS1 to CS4 count and slots; launch state; number of warnings; overrides; pusher check status; flywheel power and current; pusher lock state; active hold power; intake current; CS5 total entries, held count, FULL flag, latest proximity and raw samples; launched tally. See [logging.md](logging.md).

## 15. Status

In the Hartland build: the queued launch with the launch band, reject, always-on intake, the pusher check, jam handling, the preload pre-position, backup mode. Added after Hartland: the CS5 entry count and FULL (the first version ran in practice; the present sequence and the edge counter have not run), the lock retry limit, the put-back after an incomplete salvo, stop all, the flywheel position hold, and the launch grid.
