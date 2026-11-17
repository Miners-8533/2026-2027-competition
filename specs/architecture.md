# Architecture

How the team's robot software is organized: its layers, the control loop, the conventions every part follows, and the constraints that shaped it. The FTC SDK underneath is not described.

## 1. Context

- The code runs as FTC **opmodes** on a REV Control Hub (Android), written in Java against FTC SDK 12.0.
- The only third-party library is **FTC Dashboard** (live tunable values, graphs, a field view). There is no path-planning or motion library: the team's own drive, localization and action code replaces the Road Runner classes used in earlier seasons.
- An opmode is a single thread running one loop. The loop reads the hubs, decides, and writes the hubs.
- The physical robot is described in [robot_design/](../robot_design/README.md).

## 2. Layers

```
opmodes      TeleOp - Competition · the autos · support opmodes · the match screen
   │
features     target lock · auto-launch zones · hold position · Hive Dash · Park Dash
   │         route planner · grid routes · launch grid
   │
actions      action framework · drive to pose · aim at a cell · square on a wall · jostle
   │
mechanisms   coordinator ── intake · flywheel (+ position servo) · pusher (+ check) · ramp
   │                        slot count (CS1 to CS4) · entry count (CS5) · launch map
   │
drive base   mecanum drive · localizer (Pinpoint) · pose hand-over · chassis profile · wheel check
   │
foundation   math (pose, vector, rotation) · PID · run log · loop profiler · settings dump
```

**Packages.** `math`, `motion` (actions and closed-loop moves), `drive` (drive base and the features that plan or aim), `subsystems` (mechanisms and counts), `opmodes`, `util` (logging and profiling).

**Dependencies as they are.** Only `math` is independent. `drive`, `motion`, `subsystems` and `util` refer to each other: the hive cell positions live with the mechanisms but are used by the aiming code; actions use both the drive and the mechanisms; the settings dump lists classes from every package including two opmodes. The diagram is the intended reading order, not an enforced rule. A rebuild could separate these more cleanly without changing behavior.

| Page | Covers |
|---|---|
| [field.md](field.md) | field frame, positions, rules |
| [drive.md](drive.md) | drive base, localization, pose hand-over |
| [motion.md](motion.md) | actions and closed-loop moves, hold position |
| [intake-and-ramp.md](intake-and-ramp.md), [flywheel.md](flywheel.md), [pusher.md](pusher.md) | the actuators |
| [element-counting.md](element-counting.md) | the two counts |
| [mechanisms.md](mechanisms.md) | the coordinator and its cross-actuator rules |
| [aiming.md](aiming.md) | target lock, launch zones, launch grid |
| [dashes.md](dashes.md) | route planning, Hive Dash, Park Dash, grid routes |
| [teleop.md](teleop.md), [autos.md](autos.md), [support-opmodes.md](support-opmodes.md) | the opmodes |
| [logging.md](logging.md), [tools.md](tools.md) | logs, profiling, PC tools |

## 3. The control loop

Every opmode follows the same shape.

1. **Read once.** The pose is read from the Pinpoint once, at the top of the loop. Each mechanism reads its own sensors once, at the top of its update. Everything else in the loop works from those values.
2. **Decide.** Features and rules run on the buffered values.
3. **Write once.** One drive command per loop. Each motor is written only when its command changes.
4. **Log.** The run log's tick, then the screen.

In the TeleOp the opmode owns the loop. In an auto the action runner owns it: one action step, then the mechanisms update, then the log.

### 3.1 The hub I/O budget

Every read or write of a hub is a blocking round trip of a few milliseconds. This is the dominant constraint on the whole design. The loop runs about 26 to 32 ms in the TeleOp and about 20 ms in an auto, and almost all of it is hub traffic.

Design rules that follow from it:

- All hubs use automatic bulk reads.
- Writes are skipped when nothing changed; drive writes within a small deadband of the last one are skipped, with a periodic refresh.
- Slow sensors are rationed: one color sensor per 40 ms; one drive wheel current per 100 ms; other currents a few times a second.
- Features that plan (the dashes) do their expensive work once, at the press, and are warmed up at INIT.
- Anything that must be sampled faster than the loop gets its own thread (section 6).
- The pusher's timing depends on the loop time, so its calibration is only valid in the loop it was done in.

## 4. Conventions

### 4.1 A subsystem

Each actuator is a small class: commands record a target, and one **update** per loop reads once and writes when needed. Subsystems also offer **actions** for autos (for example "run a salvo and finish when the pusher has stopped").

### 4.2 Pure logic

Planning and decision code touches no hardware: it takes a snapshot in and returns a result. The opmode gathers the snapshot and applies the result. This applies to the route planner, both dashes and their followers, hold position, the target lock, the drive wheel check, the entry counting rule and the launch grid. It is what makes PC checks and the dash simulator possible ([tools.md](tools.md)), and it must be preserved.

### 4.3 Tunable values

Almost every number is a tunable value shown in the FTC Dashboard and read live. Consequences:

- a value can be changed without a rebuild, and is lost when the app restarts;
- a tuned value survives only when written into the code;
- every run logs the values in force ([logging.md](logging.md));
- most features have an on/off switch among their tunables, which is how unproven features are shipped disabled.

A few switches are build-time constants instead, where a live change would be unsafe.

### 4.4 Optional hardware

The coordinator builds whatever the hub configuration contains and every rule tolerates a missing mechanism. Only the drive motors and the Pinpoint are required. See [robot_design/hub-configuration.md](../robot_design/hub-configuration.md).

### 4.5 Alliance symmetry

Everything is defined for blue and rotated 180 degrees about the field center for red. There are no red-specific numbers ([field.md](field.md)).

### 4.6 Naming

The team's words are pusher, ramp, launch, launcher, launch map, salvo, hive cell, flower, garden. The hub configuration keeps older device names for the pusher and ramp, and they are not to be renamed.

### 4.7 Operator interface rules

- Start and Back are never bound (the Driver Station uses Start to assign gamepads).
- The driver's stick buttons are never bound (too easy to press by accident).
- Risky actions need a second button held, and some need a one-second hold.
- The controllers have no rumble: every signal to the drivers goes on the screen.
- Only match-critical information goes on the match screen.

### 4.8 Failure philosophy

- **Degrade, do not stop.** A lost mechanism is skipped. A lost flywheel encoder switches to open loop. A lost Pinpoint switches the TeleOp to backup mode. A pusher that cannot finish a move gives up after a timeout; it does not fault.
- **No fault for the drivers to clear in a match.** Jams are resolved automatically or by pressing the opposite command (a launch cancels a reject and the reverse).
- **A wrong sensor must never block collecting or launching.** The operator can always force the intake and always launch; the autos always launch, whatever a count says.
- **Wiring and sign errors are found in the pit**, by Pit Check, not handled at run time.

## 5. State that outlives an opmode

Three things are kept in memory for as long as the robot app runs, so that the TeleOp can continue from an auto. All are lost on an app restart.

| State | Written by | Used by |
|---|---|---|
| Saved pose, alliance and its time | autos at their end; the TeleOp every loop | the TeleOp's INIT, if fresh |
| Pusher zero, and whether it has been verified | every opmode that builds the pusher; the pusher check | a TeleOp that continues from an auto |
| Tunable values as edited in the Dashboard | the Dashboard | every opmode |

The Pinpoint keeps its own pose in the device for as long as it is powered, which is what makes adoption possible ([drive.md](drive.md)).

## 6. Threads

| Thread | Purpose |
|---|---|
| The opmode thread | everything, unless listed below |
| Run log writer | writes queued lines to the file and flushes once a second |
| CS5 entry counter | samples CS5 every 25 ms and runs the counting rule; starts with the first mechanisms update, stops when updates stop |
| CS5 reader in the CS Pass Study | that study only |

The opmode thread never waits on these. Results cross over through small thread-safe queues and counters; the log is written only from the opmode thread's requests.

## 7. Opmode lifecycle

| Phase | Rule |
|---|---|
| INIT | Build everything. An auto resets and seeds the Pinpoint and commands the ramp stowed and the flywheel position. **The TeleOp moves nothing in INIT** (rule G403) and adopts the Pinpoint when it can. INIT screens verify the pose, the mechanisms found and the counts |
| Start | The TeleOp makes its first servo commands here |
| Run | The loop of section 3 |
| End | The TeleOp stops the drive, then stops all mechanisms at once, without a final update. An auto saves its pose and keeps reading the pose and updating the mechanisms until it is stopped. Both close the log on every way out |

## 8. Feature status at a glance

| Area | State |
|---|---|
| Drive, localization, adoption, hand-over | in the tested build |
| Queued launch, launch band, reject, pusher check, jam handling | in the tested build |
| Target lock, auto-launch, hold position, reports, backup mode | in the tested build |
| Hive Dash and Park Dash (hand-placed route points) | in the tested build; known to cut clearances fine at present speeds |
| Solo, Daryl, Cybugs autos | in the tested build |
| CS5 entry count, FULL sequence, COUNT ONLY choice | added later; the first version ran in practice, the present one has not |
| Drive hold during a salvo | added later; ran in practice, not at speed |
| Flywheel position hold; flywheel encoder recovery | added later; not run |
| Launch grid | added later; off; not run |
| Grid routes | built; off; not run on the robot |
| Pusher lock for a target lock or dash | built; off |
| Drive-to-pose static friction term | added later; opt-in; not run |
| Drive-to-pose gain tuning on this robot | never done |

"Tested build" means the build used at the Hartland Qualifier on 2026-11-07, which is the `main` branch. These specs describe the `integration` branch, which has all of the above.

## 9. Known structural weaknesses

Recorded so a rebuild can choose to fix them.

1. **Odometry drift** across the field is compensated in three unrelated places: the far-cell extra, the autos' -8 degree trims, and the operator's remembered aim.
2. **The autos rest on untuned drive gains**, loose tolerances and time caps.
3. **Two element counts** that disagree, with rules spread between the counter classes and the coordinator.
4. **The coordinator is large** and carries dormant features (overrides, the toggle intake, manual setpoint) beside live ones.
5. **Package dependencies are circular** (section 2).
6. **One study opmode is a fork** of the competition TeleOp and drifts from it.
7. **Field constants are duplicated** in a few places (flower centers; the duo path check's copy of the auto poses).
