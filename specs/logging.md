# Logging and diagnostics

Every run of a competition or test opmode writes a text log to the Control Hub. The logs are the team's main tool: tuning, fault finding, match review and the drive model are all done from them. This page specifies the log, the settings dump, the loop profiler and the build stamp.

## 1. Run log

### 1.1 Files

Logs live in a `runlogs` folder under the FTC SDK's data folder on the Control Hub (`/sdcard/FIRST/runlogs`).

| Kind | File | Written by |
|---|---|---|
| auto | `auto.log` | every autonomous program |
| teleop | `teleop.log` | the competition TeleOp |
| pitcheck | `pitcheck.log` | Pit Check |
| bringup | `bringup.log` | the tuning and study opmodes |
| waypoints | `waypoints.log` | the Auto Waypoint Marker |
| drivetest | `drivetest.log` | Drive Characterization |

**Rotation.** Opening a log of a kind shifts that kind's earlier runs back one place (`teleop.log` to `teleop_prev.log`, then `teleop_prev2.log` and so on) and drops the oldest. The last **10** runs of each kind are kept. Kinds never touch each other's files, so an auto and the TeleOp that follows it are both kept.

- The first line of a new log says how the previous run of that kind ended: normally, with a crash, or with no end marker (the app stopped hard or power was lost).
- One run is discarded: an INIT that was stopped without ever starting and without crashing.
- If the log cannot be opened, the opmode runs without one. Every logging call does nothing while no log is open, so any code may log freely.

### 1.2 Lines

Plain text, one record per line. The first field says what it is.

| First field | Meaning |
|---|---|
| `#` | a comment: the opmode, the date and time by the hub's clock, how the previous run ended, the build stamp |
| `H` | the column names of the sample rows |
| `S` | a sample row |
| `E` | an event: seconds since the log opened, then free text |

**Layout.** Fields are separated by commas, with no quoting.

| Line | Layout |
|---|---|
| First line | `# <opmode name>, opened <YYYY-MM-DD> <HH:MM:SS> (hub clock). Previous <kind> run: <how it ended>` |
| Second line | `# Build: commit <short id> (<branch>), built <PC time> (PC clock), clean`, or in place of `clean`: `UNCOMMITTED changes: <n> file(s) changed, <m> new, fingerprint <8 hex digits>` |
| `H` | `H,t,loop_ms,loop_max_ms,volt,` then the opmode's column names |
| `S` | `S,` then one value per column of the `H` line, in its order |
| `E` | `E,<seconds>,<text>`. The text may contain commas: a reader splits on the first two only |

*How it ended* is one of: `none`; `ended normally`; `CRASHED (see the exception in <kind>_prev.log)`; `NO END MARKER: the app stopped hard or the robot lost power`; `init only, stopped before start`.

**Start and end of a run.**

- Start writes the event `RUN STARTED`, then the settings dump. The loop profiler counts from there.
- A crash writes `CRASH: <the exception>` and up to 60 lines of its stack trace, each as its own `E` line.
- Closing the log writes the whole-run loop profile and then, as the last line, `RUN ENDED (normal)` or `RUN ENDED (after a crash)`. A log without `RUN ENDED` stopped hard.
- The run that is discarded (an INIT that was stopped without starting or crashing) is recognized when the next log of its kind is opened: it has `RUN ENDED` and neither `RUN STARTED` nor `CRASH:`. It is deleted then, and the other runs are not shifted.

### 1.3 Contract for an opmode

1. **Open** the log first, before any hardware is touched, so a crash during initialization is captured.
2. Mark **started** when Start is pressed. Times keep counting from the open.
3. Register a **sampler**: the column names, a period, and a function that builds the row from values already read this loop. It must not read the hub.
4. Call **tick** once per loop. It writes a sample when one is due and notes a slow loop.
5. On an exception, record the **crash** and rethrow.
6. **Close** in every way out of the opmode.

### 1.4 Behavior

| Rule | Value |
|---|---|
| Writing | on a background thread; the loop only pays for building the line |
| Flush | about once a second, so a hard crash loses at most the last second |
| Normal sample period | 100 ms |
| **Burst** | while switched on, a sample row is written on every loop. The coordinator switches it on from a queued launch until 0.6 s after the salvo |
| Battery voltage | read once a second (the only hub read the log adds) |
| Slow loop | a loop over 80 ms is logged as an event, at most once every 0.5 s |
| Runs kept per kind | 10 |

### 1.5 Standard sample columns

| Opmode | Columns after the common four |
|---|---|
| Competition TeleOp | pose (x, y, heading); the driver's stick (up, right); the drive command (forward, left, turn); drive mode; lock error; the coordinator's columns; four wheel currents; launch grid zone, speed and residual |
| Autonomous programs | pose; the last drive command; the coordinator's columns; four wheel currents |
| Drive Characterization | every loop: pose, field velocity, the command, the four wheel powers sent, the battery every loop, one motor current per loop in turn, and the test, segment and phase |

The coordinator's columns are listed in [mechanisms.md](mechanisms.md). Study opmodes define their own rows.

**Column names.**

| Columns | Names |
|---|---|
| Pose | `x`, `y`, `hdg_deg` |
| The driver's stick | `stick_up`, `stick_right` |
| The drive command | `cmd_fwd`, `cmd_left`, `cmd_turn` |
| Drive mode, lock error | `drive_mode`, `lock_err_deg` |
| The four wheel currents | `a_lf`, `a_lb`, `a_rb`, `a_rf` |
| Launch grid | `grid_zone`, `grid_speed`, `grid_resid_deg` |
| Drive Characterization | `x`, `y`, `hdg_deg`, `vx`, `vy`, `w_dps`, `cmd_fwd`, `cmd_left`, `cmd_turn`, `p_lf`, `p_lb`, `p_rb`, `p_rf`, `vbat`, `a_lf`, `a_lb`, `a_rb`, `a_rf`, `test`, `seg`, `phase` |

The competition TeleOp's row is, in order: the pose, the stick, the drive command, `drive_mode`, `lock_err_deg`, the coordinator's columns, the wheel currents, the launch grid columns. An auto's row is: the pose, the drive command, the coordinator's columns, the wheel currents.

### 1.6 Event families

These prefixes are relied on by the analysis tools, so they are part of the interface. Beyond its prefix the wording of an event is not fixed, with two exceptions: the wordings in the second table below, and a line whose form is given on the page of its feature.

| Prefix | Source |
|---|---|
| `SETUP` | what a run started with: profile, alliance, start pose, mechanisms, modes, gamepads |
| `PINPOINT` | how the device was brought up, what was decided at INIT, the state at Start, the hand-over, faults and blips |
| `CONFIG`, `CONFIG CHANGED` | section 2 |
| `COMMAND` | every operator or program command to the mechanisms |
| `LAUNCH`, `SALVO`, `REJECT` | the life of a launch: requested (with the aim line), queued, held, released, started, done, dropped |
| `TARGET LOCK`, `AUTO LAUNCH`, `HOLD`, `DRIVE HELD` | the aiming and holding features |
| `HIVE DASH`, `PARK DASH` | start with the route, legs, via points, samples, arrival, launch, cancel, refusal |
| `PUSHER ...` | state changes, jam, lock, unlock, active hold, the pusher check and its crossings |
| `INTAKE`, `FLYWHEEL` | mode changes, current stretches, encoder lost and back |
| `ELEMENTS`, `COUNTER` | slot count changes, stored baselines, the once-a-second empty-track health line |
| `CS5 ENTRY`, `ENTRY` | each entry, FULL, resets, the thread's statistics and health |
| `CS5 KIND` | the kind of an entry, with its color shares, total rise and number of color values, and the load order after it; `CS5 KIND OFF` when the color reads were given up for the run. The form of these lines, and of the load order on the `ENTRY COUNT` and `SALVO started` lines, is given in [element-counting.md](element-counting.md) section 2.8 |
| `CS2 LAUNCH SENSOR` | the reader of CS2 at the top of the feed ramp (recorded only, from 2026-11-18): its settings at the start, and after every salvo the statistics of the fast stretch ([element-counting.md](element-counting.md) section 6.1) |
| `WARNING`, `WARNING cleared`, `NOTICE` | the coordinator's standing conditions and one-off messages |
| `REPORT` | the operator's reports of where a salvo landed |
| `REMEMBERED AIM` | the remembered aims continued from the last TeleOp at INIT, or cleared there |
| `ENTRY COUNT` | the held count's start value (with the load order it starts with) and its resets |
| `AUTO ...` | notes, `AUTO TIMING`, `AUTO WAIT`, `AUTO SKIP`, `AUTO SQUARE`, `AUTO JOSTLE` |
| `DRIVE start`, `DRIVE settled / TIMED OUT`, `AIM` | each closed-loop leg and aim |
| `ACCEPT run`, `ACCEPT TEST`, `ACCEPT RESULT`, `ACCEPT GAINS` | the Drive Acceptance opmode ([support-opmodes.md](support-opmodes.md)) |
| `TUNE gains` | the gains in force in a tuner, at its start and when a Dashboard edit changes them |
| `DRIVE WHEEL` | a wheel drawing too little current, and its recovery |
| `LOOP PROFILE` | section 3 |
| `BACKUP MODE`, `DRIVER`, `GAMEPAD`, `SLOW LOOP`, `CRASH:` | as named |

**Exact wording the pull tool matches** (a different wording leaves a field of the run index empty).

| Event text starts with | Used for |
|---|---|
| `RUN ENDED`, `CRASH:` | how the run ended. (The tool takes the last of the two it meets, and a crashed run writes `CRASH:` and then `RUN ENDED (after a crash)`, so it files a crashed run as ended normally) |
| `SETUP profile <name>, alliance <BLUE or RED>, start pose <default or from the AUTO> (<x>, <y>, <heading> deg), driver forward <deg> deg; ...` | the alliance and the start |
| `SALVO started` | the number of salvos |
| `MARK <n>`, `CHECK <n>` | the Auto Waypoint Marker's marks |
| `PUSHER CHECK` with `started`, `SEEK` or `OK` | the number of pusher checks |
| `WARNING` without `cleared` | the number of warnings |
| `SLOW LOOP` | the number of slow loops |
| `BACKUP MODE ENTERED (<why>)` | the backup flag |

Team vocabulary is fixed: logs say launch, launcher, pusher and ramp. Logs from before 2026-11-04 use older words and the tools accept both.

## 2. Settings dump

**Purpose.** Tunable values can be edited live in the FTC Dashboard, and edits are lost when the app restarts. Without a record, a log does not say which trim, speed or tolerance was in force.

**Behavior.** When a run starts:

- one `CONFIG <class> name=value, ...` event per tunable class;
- then one `CONFIG CHANGED <class.name>: <old> -> <new>` event for every value that differs from the previous run in the same app session.

The list of classes is maintained by hand, and a PC tool reports any tunable class missing from it.

**Known gap.** The list holds the classes of the match code and Drive Acceptance. It does not include the tunable values of the other support opmodes (the studies, Pit Check, Drive Characterization, the Launch Tuner and the others). Until 2026-11-17 it also missed the entry counter, the flywheel position servo and the drive wheel check.

### 2.1 Target T8: the dump finds its own classes

Not in the code. See "As built and target" in the [README](README.md).

**The weakness.** A new tunable class is logged only if someone remembers to add it to the list. Three that run in matches were forgotten until 2026-11-17. The list also makes the logging package name classes in every other package, opmodes included, which is part of the package cycle ([architecture.md](architecture.md) section 2.1).

**Requirement.** Every tunable class in the team's code is dumped, with no list to keep.

**Design.**

- The set of classes is found, not written: every class of the team's code that carries the tunable-value marker. It is found once, when the app starts, or generated at build time; either way it costs nothing at the start of a run.
- Every opmode dumps every class, so a support opmode's own values are on record with the run they shaped.
- The line format and the `CONFIG CHANGED` lines are unchanged, so the log tools keep working.
- The PC tool that looks for missing classes is no longer needed.

**What would behave differently.** More `CONFIG` lines at the start of each log: 44 classes carry the marker today and 29 are listed. Nothing on the robot.

## 3. Loop profiler

**Purpose.** Show where the loop time goes, permanently, at negligible cost.

**Behavior.**

- Named **sections** of the loop are timed. A section started inside another is nested: it is reported but not counted twice.
- Named **counters** count events per loop (for example drive writes sent and skipped).
- The log's tick marks the end of each loop.
- Every 5 s one `LOOP PROFILE window` line is written, and one `LOOP PROFILE RUN` line for the whole run when the log closes.

A line gives: the window length and number of loops; how many loops exceeded 40, 60 and 80 ms; the average and longest loop; the time outside every top-level section; then for each section its average per loop, its longest single call and its calls per loop; then the counters.

A section that runs only on some loops is averaged over all loops, so its longest call is the figure to read.

**Sections in use.** `pose` (the Pinpoint read), `drive_write`, `drive_amps`, `mechanisms` with `mech.counter`, `mech.intake`, `mech.flywheel` and `mech.pusher` inside it, `telemetry`, `park_dash`, `hive_dash`, `target_lock`, `hold`; in an auto `action` and `dash_packet`.

**Reference figures.** The competition TeleOp loop averages about 26 to 32 ms with roughly a tenth to a fifth of loops over 40 ms. The largest items are the color sensor read, the Pinpoint read and the pusher. Autos run about 20 to 22 ms.

## 4. Build stamp

**Purpose.** Tie every log to the exact code that produced it.

- At every build, a build step generates a small source file from the repository state: commit id, branch, whether the tree had uncommitted changes, how many files were changed or new, a fingerprint of those changes, and the build time by the PC's clock.
- The second line of every log is that stamp.
- A clean build is exactly that commit. For a build with uncommitted changes, the PC that built it saves the tracked difference as a patch named by commit and fingerprint, and appends one line per build to a build history file. Both stay on that PC and are not committed.
- The PC's clock and the hub's clock are different clocks. Only the order of builds against runs is meaningful.

## 5. Getting and reading logs

These are PC tools, outside the robot code ([tools.md](tools.md)), but they define how the logs are used.

### 5.1 Pulling the logs (hub on USB)

From the repo root:

```
python tools/logs/pull_logs.py
```

(or double-click `tools/logs/pull-logs.cmd`). It finds adb under the Android SDK's platform-tools, copies the logs off the hub and files **every** run. Then it removes from the hub each log that it has confirmed is stored and finished (an end marker, or idle for 5 minutes); a log that may still be running is left there. The hub keeps only the last 10 runs of each kind, so pull before an opmode has run an eleventh time.

```
robot-logs/                    (not in git)
  index.md                     one row per run, newest first: opmode, build, length, how it ended, alliance, salvos, minimum battery volts, worst loop time, slow loops, flags (crash, backup mode, flywheel encoder lost, Pinpoint faults)
  index.json                   the same for tools
  latest/teleop.log            the newest and second newest run of each kind under fixed names
  latest/teleop_prev.log          (also auto, bringup, pitcheck)
  2026-11-02/000341_teleop.log every run, by the date and time on its first line (the hub's clock)
  matchlogs/                   the SDK's own match logs, with --matchlogs
```

- Running it again is safe: runs already stored are skipped, and a run that was still being written when it was pulled is replaced by the longer copy.
- Options: `--list` (print the index only), `--from-dir <folder>` (file logs that were pulled earlier), `--keep-on-hub` (do not remove them from the hub), `--matchlogs`, `--serial <adb serial>`, `--dest <folder>`.
- The date and time of a run are the hub's clock, which can be wrong: at the Novi Qualifier it ran about 60 minutes ahead.
- After a crash, the first `#` line of the next log of that kind says how the previous run ended.

**Without the tool:** `adb pull /sdcard/FIRST/runlogs runlogs` copies the folder as it is. Two check opmodes also write to the robot's system log: `adb logcat -d -s PitCheck` (Pit Check's results) and `adb logcat -d -s ColorSensors` (the Color Sensors Test).

### 5.2 Event day

Pull the logs as usual, then label them with their matches and read the report:

```
python tools/logs/event_day.py label  --event "<event>" --since 2026-11-05                  # asks for the match of each unlabeled run, oldest first (free text: Q12, M3, practice)
python tools/logs/event_day.py label  --event "<event>" --set 2026-11-05/101530 Q12         # or one run at a time, no questions
python tools/logs/event_day.py report --event "<event>" --since 2026-11-05
python tools/logs/cycle_times.py --event "<event>"
```

- A match is an auto followed by a TeleOp within 90 s of its end; a lone run is its own group. `--auto-number R` labels every unlabeled run R1, R2, ... for a rehearsal.
- Everything goes to `robot-logs/events/<event>/`: `labels.csv` (edit it by hand to clean labels up; the raw logs are never changed), one `<label>.md` per match (flags, a table of numbers, the events worth reading, the settings that changed) and `day.md` (what stands out against the day's median once there are 4 or more matches, the hard faults, a table of every match).
- **Cycle times** writes `cycle_times.md` beside the reports. A TeleOp cycle is one salvo start to the next, split into the salvo, collect (salvo done to the last time the stack rose to the count it held at the lock), transit (that moment to the target lock coming on, with the distance moved) and lock (lock on to the salvo starting). It also gives pollen per minute, the cycles by pollen launched, the time before the first salvo and after the last, and the auto steps side by side. A lock the driver released again counts as collect, and so does a pickup made on the way to the launch spot, so transit is a floor.
- Match labels are never entered on the robot: only mission-critical input goes on the Driver Station (the user, 2026-11-04).

### 5.3 Other analyses

Loop profile tables, settings changes between runs, sensor baseline health, launch stability and the launcher offset: one tool each in `tools/logs/` ([tools.md](tools.md) section 4).

## 6. What is not logged

Raw color sensor readings in match opmodes (only the classified slots and, when the track is empty, the color shifts); motor currents other than those named above.

## 7. Dashboard

Every tunable value is exposed in the FTC Dashboard and read live, so most parameters in these specs can be changed without a rebuild. Such changes last until the app restarts. **A tuned value survives only if it is written into the code.**

The Dashboard is at `192.168.43.1:8080/dash` over the hub's Wi-Fi. Over USB: `adb forward tcp:8080 tcp:8080`, then `localhost:8080/dash`.

The Dashboard's field view also shows the dashes' routes and obstacles while a plan is on screen.
