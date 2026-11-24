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

**Dependencies as they are.** Only `math` is independent. `drive`, `motion`, `subsystems` and `util` refer to each other: the hive cell positions live with the mechanisms but are used by the aiming code; actions use both the drive and the mechanisms; the settings dump lists classes from every package including two opmodes. The diagram is the intended reading order, not an enforced rule.

### 2.1 Target T5: packages in layers

**The weakness.** The four packages above `math` form one cycle. It comes from six misplacements, not from the design:

| What | Lives in | Needed by |
|---|---|---|
| Hive cell positions | `subsystems` | the aiming and dash code in `drive`, and `motion` |
| Chassis profile | `drive` | every actuator in `subsystems` |
| Action framework | `motion`, beside the moves that need the drive | the actuators' own actions in `subsystems` |
| PID controller | `motion` | hold position in `drive` |
| Launch grid lookup | `drive` | the coordinator in `subsystems` |
| Settings dump's class list | `util` | names classes in every package, opmodes included |

**Requirement.** A package refers only to packages above it in the table below. No cycles. No behavior changes.

**Design.**

| Package | Holds | May use |
|---|---|---|
| `math` | pose, vector, rotation, PID controller | nothing |
| `util` | run log, loop profiler, settings dump (which discovers its classes, T8) | nothing |
| `field` | the field model (T7): frame, walls, hive cells and supports, flowers, zones, start poses, launch spots, park corners, route points | `math` |
| `robot` | the chassis profile: which robot, its device names, its dimensions | `math` |
| `actions` | the action framework: action, sequence, parallel, wait, instant | `util` |
| `drive` | mecanum drive, localizer, pose hand-over, wheel check | `math`, `util`, `robot` |
| `aiming` | aim heading, target lock, auto-launch zones, launch grid | `math`, `util`, `field` |
| `navigation` | route planner, followers, Hive Dash, Park Dash, grid routes, hold position | `math`, `util`, `field`, `robot` |
| `subsystems` | actuators, counts, launch map, coordinator | `math`, `util`, `field`, `robot`, `actions`, `aiming` |
| `motion` | the action runner; drive to pose, aim at a cell, square on a wall, jostle | everything above |
| `opmodes` | the opmodes and the match screen | everything above |

`field`, `aiming` and `navigation` are then pure-logic packages as a whole: apart from the log and the tunable-value marker they touch nothing from the SDK, so the PC checks ([tools.md](tools.md)) compile whole packages instead of hand-picked classes.

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

- Start and Back are never bound (the Driver Station uses Start to assign gamepads). This holds for the match opmodes; six support opmodes use Back ([support-opmodes.md](support-opmodes.md)).
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

Five things are kept in memory for as long as the robot app runs, so that the TeleOp can continue from an auto. All are lost on an app restart.

| State | Written by | Used by |
|---|---|---|
| Saved pose, alliance and its time | autos at their end; the TeleOp every loop | the TeleOp's INIT, if fresh |
| The operator's remembered aims (built 2026-11-17, not run) | the TeleOp, every loop; an auto drops them | a TeleOp that adopts the device's pose |
| CS5 held count and its time (built 2026-11-17, not run) | the autos and the TeleOp, every loop | the TeleOp's INIT, if fresh |
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
| End | The TeleOp and the autos stop the drive, then stop all mechanisms at once, without a final update (the autos from 2026-11-17, not run). An auto saves its pose and keeps reading the pose and updating the mechanisms until it is stopped. Both close the log on every way out |

### 7.1 Target T10: one way out

**The weakness.** The support opmodes each stop what they used in their own way, or leave it to the SDK, which cuts motor power when an opmode stops. It works, but "what is still commanded when an opmode ends" has a different answer per opmode. The TeleOp and, from 2026-11-17, the autos end the same way.

**Requirement and design.** Every opmode ends through the same shutdown, in this order: stop the drive; stop all mechanisms (with no update after it); save the pose if the opmode saves one; close the log. It runs on every way out, a crash included. An auto whose sequence finishes early still keeps updating until it is stopped, as now; the shutdown is for the stop itself.

No behavior the drivers can see changes.

## 8. Feature status at a glance

| Area | State |
|---|---|
| Drive, localization, adoption, hand-over | in the tested build |
| Queued launch, launch band, reject, pusher check, jam handling | in the tested build |
| Target lock, auto-launch, hold position, reports, backup mode | in the tested build |
| Hive Dash and Park Dash (hand-placed route points) | in the tested build; judged too slow to use as they are, and known to cut clearances fine at present speeds |
| Solo, Daryl, Cybugs autos, as at Hartland | in the tested build |
| Solo with the launch grid, the launch-ready pusher and one launch spot (2026-11-19, 20) | ran in six matches at the Novi Qualifier |
| Boomerang (2026-11-21) | ran in five playoff matches at the Novi Qualifier and finished every time. A launch spot 2 in nearer the wall is on the `integration` branch only, not run |
| Daryl's new finish; Blaser's Gambit (2026-11-21) | each run three times on 2026-11-21 outside a match; Daryl finished once, Blaser's Gambit has not finished its actions |
| Triple Tip (2026-11-20); Garden Side and Flower Side as rebuilt 2026-11-21 | built; not run |
| CS5 entry count, FULL sequence, COUNT ONLY choice | added later; ran in practice to 2026-11-19; **CS5 is switched off since 2026-11-20**: no held count is shown (`? HELD`) and nothing is ever FULL. CS2's recording is switched off too |
| Drive hold during a salvo | added later; ran in every match at the Novi Qualifier |
| Flywheel position hold; flywheel encoder recovery | added later; not run |
| Launch grid | on from Start in the TeleOp (2026-11-18) and in the shown autos (2026-11-19 to 21); used in every match at the Novi Qualifier |
| No launch from a no-shot zone of the grid (TeleOp) | built 2026-11-19; ran at the Novi Qualifier |
| Grid routes, driven as smooth paths | built; on since 2026-11-18 for both dashes; not run on the robot |
| Launch-ready pusher position | the autos' request ran in every auto at the Novi Qualifier; the TeleOp's (for a target lock or a dash) ran in two matches there and is off since 2026-11-21 |
| Pusher re-home after a skipped check | built 2026-11-19; ran after every salvo at the Novi Qualifier, where a stale CS3 baseline kept the check itself from running (re-measured 2026-11-21; the new values are on the `integration` branch only, not run) |
| Jostle strokes by time; a time limit for the whole wall squaring | built 2026-11-19, 20; ran at the Novi Qualifier |
| Drive-to-pose static friction term | added later; opt-in; not run |
| Drive-to-pose gain tuning on this robot | never done |
| CS5 held count carried between opmodes; an auto starts at 4 | built 2026-11-17; not run |
| The entry count alone for full and the tally (jostle end, warning, intake stop) | built 2026-11-17; on; not run |
| Autos: one aim trim per launch, no far-cell extra | built 2026-11-17; same aim by PC check; in every auto at the Novi Qualifier |
| Remembered aims kept through a TeleOp restart; aim trims on the match screen | built 2026-11-17; not run |
| Autos stop all mechanisms at their end | built 2026-11-17; in every auto at the Novi Qualifier |
| Drive Acceptance opmode; overshoot in the drive log line; tuners write a log | built 2026-11-17; not run |

"Tested build" means the build used at the Hartland Qualifier on 2026-11-07, which is the `main` branch. These specs describe the `integration` branch, which has all of the above.

## 9. Known weaknesses and their targets

Each weakness of the code as built has a target: what the software should do instead. The target text is on the page of the component it belongs to. None of it is in the code (see "As built and target" in the [README](README.md)).

| # | Weakness as built | Target | Specified in | Changes behavior | Waits on |
|---|---|---|---|---|---|
| T1 | **Odometry drift** was allowed for in four unrelated places, two of which worked against each other | Drift is accepted (team decision); the operator's trims are the answer; fixed allowances exist once | [aiming.md](aiming.md) section 5 | barely | **mostly built 2026-11-17, not run:** one trim per auto launch, the aims saved for a restart, the trims on the match screen. Left: the far launch spot worked out from the allowance (needs the route tables regenerated) |
| T2 | **The autos rest on untuned drive gains**, loose tolerances and time caps; the tuning opmodes leave no record | Drive to pose is tuned and passes an acceptance test whose numbers the opmode itself reports; a timed-out leg is a counted fault | [motion.md](motion.md) section 2.1, [support-opmodes.md](support-opmodes.md) section 4.2 | yes, after tuning | a tuning session on the robot. The acceptance opmode and the logging are built (2026-11-17), not run |
| T3 | **Two element counts** with different rules for "full"; the count started at 0 in every opmode | The CS5 entry count is the count (team decision); the slot sensors keep kinds, "reads empty" and the arm; the count is remembered between opmodes | [element-counting.md](element-counting.md) section 5 | yes | **built 2026-11-17, not run,** with a switch back to the slot count. Waits on the edge counter's first robot run |
| T4 | **The coordinator is large**, mixes live and dormant features, and can only be exercised on the robot | The same single object for opmodes, made of pure-logic parts; nothing unreachable (the pusher lock for a target lock or dash is kept, switched off) | [mechanisms.md](mechanisms.md) section 16 | no | target T3 |
| T5 | **Package dependencies are circular** | Packages in layers | section 2.1 | no | nothing |
| T6 | **One study opmode was a fork** of the competition TeleOp and had drifted from it | The match loop exists once; the fork is retired | [support-opmodes.md](support-opmodes.md) section 3.1 | no | **done 2026-11-17:** the study is removed |
| T7 | **Field constants are duplicated** (the path check's copy of the auto poses; helper points and park poses in several autos) | One field model that the robot code and the PC checks both read | [field.md](field.md) section 7 | no | nothing. The flower stands have one definition from 2026-11-17 |
| T8 | **The settings dump's class list is kept by hand** and is missing classes | The dump finds every tunable class itself | [logging.md](logging.md) section 2.1 | more lines in the log | nothing. The three missing match classes were added by hand on 2026-11-17 |
| T9 | **The dashes are too slow to use** (team judgement) and still cut clearances fine; their speed values were set by hand | A dash must beat hand driving and touch nothing, both proven before the drivers get it; the grid routes are the route source | [dashes.md](dashes.md) sections 6.1 and 6.2 | yes | **gates built and run in simulation 2026-11-17; route tables regenerated with a 3 in drift margin, a stop that does not slide, a move-clear leg in place of a refusal:** the safety gate passes at 4 in of drift and no dash is refused; the speed gate fails, the median trip being 0.26 s slower than the drivers. Next: the time (a nearer goal, a faster last approach), then the first robot run |
| T10 | **Opmodes end in different ways** | One shutdown for every opmode | section 7.1 | no | nothing. The autos end as the TeleOp does from 2026-11-17; the support opmodes are left |
| T11 | **Nothing sees an element leave**; CS2, moved to the top of the feed ramp on 2026-11-18, is still read as a place along the track | CS2 counts the elements each salvo launches (team decision): the true number of each load, the launched tally, what is still held after a salvo | [element-counting.md](element-counting.md) section 6 | yes | **stages 1 and 2 built 2026-11-18, not run:** CS2 has left the slot count and is recorded through every salvo. The rule waits on the recordings |

**Order.** T5, T7, T8, T10 and T6 are safe at any time and make the others easier. T2 comes before any rework of the autos. T3 comes before T4, which builds on it. T9 uses the accepted drift that T1 states. T11 stands apart from the others.

**Not targets, because no design change answers them:** the entry count and the FULL sequence need their first robot run; the launch grid needs data for the other cells; the duo programs' waits need tuning from logged runs.
