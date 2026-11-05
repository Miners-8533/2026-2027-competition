# Run logs (the last auto and the last TeleOp)

`TeleOp - Competition` and the autos (`Auton - Solo`, and the duo autos on their branch) write a log of the run to the hub, so it can be read after a match. The code is `util/RunLog.java`.

## Files

They are in `/sdcard/FIRST/runlogs/` on the Control Hub:

| File | What it holds |
|---|---|
| `auto.log` | the latest auto run |
| `auto_prev.log` | the auto run before it |
| `teleop.log` | the latest TeleOp run |
| `teleop_prev.log` | the TeleOp run before it |
| `waypoints.log` (and `_prev` ...) | the Auto Waypoint Marker's run: its `MARK` lines are the points marked |
| `teleop_prev2.log` ... `teleop_prev9.log` | the runs before that, each one further back (the same for `auto`, `bringup` and `pitcheck`) |

Starting an opmode shifts its logs back one place (`*.log` to `*_prev.log`, `*_prev.log` to `*_prev2.log`, and so on, dropping the oldest) and starts a new one, so **the auto and the TeleOp never overwrite each other**. If an opmode crashes and is started again, the crash is in `*_prev.log` and the rest of the run is in `*.log`. The first line of the new log says how the previous run of that kind ended (`ended normally`, `CRASHED`, or `NO END MARKER`, meaning the app stopped hard or power was lost).

The one run that is discarded is an init that was stopped before Start without crashing.

Only the last 10 runs of each kind are kept (`RunLog.KEEP_RUNS`; a run is a few hundred KB), so pull them (tools/logs/pull_logs.py) before running an opmode a eleventh time.

## Getting it off the hub

With the hub on USB, from the repo root:

```bash
python tools/logs/pull_logs.py
```

(or double-click `tools/logs/pull-logs.cmd`). It copies the logs off the hub and files **every** run, since the hub keeps only the last 10 of each kind, and then removes them from the hub:

```
robot-logs/                    (not in git)
  index.md                     one row per run, newest first: opmode, length, how it ended, alliance, salvos, minimum battery volts, worst loop time, slow loops, flags (crash, backup mode, flywheel encoder lost, Pinpoint faults)
  index.json                   the same for tools
  latest/teleop.log            the newest and second newest run of each kind under fixed names
  latest/teleop_prev.log          (also auto, bringup, pitcheck)
  2026-11-02/000341_teleop.log every run, by the date and time on its first line (the hub's clock)
  matchlogs/                   the SDK's own match logs, with --matchlogs
```

Running it again is safe: runs already stored are skipped, a run that was still being written when it was pulled is replaced by the longer copy, and nothing is deleted from the hub or from `robot-logs`. Options: `--list` (print the index only), `--from-dir <folder>` (file logs you already pulled), `--matchlogs`, `--serial <adb serial>`, `--dest <folder>`.

Then ask Claude Code to read `robot-logs/latest/teleop.log` (or any file in the index).

## Format

Plain text, one line each, first field says what it is:

| First field | Meaning |
|---|---|
| `#` | comment: opmode, date (the hub's clock, which may be wrong), how the previous run ended |
| `H` | the column names of the `S` rows |
| `S` | a sample every 100 ms: seconds since the log opened, average and worst loop time since the last sample (ms), battery volts (refreshed once a second), then the opmode's columns |
| `E` | an event: seconds, then text |

TeleOp sample columns: pose (`x`, `y`, `hdg_deg`), the driver's left stick (`stick_up`, `stick_right`, after the precision scale), the drive command (`cmd_fwd`, `cmd_left`, `cmd_turn`), then the mechanisms' columns: `setpoint`, `fly_target`, `fly_vel`, `fly_ready`, `pusher_state`, `pusher_deg`, `pusher_vel`, `pusher_amps`, `intake_mode`, `ramp`, `count`, `slots` (CS1 to CS4: P pollen, R red nectar, B blue nectar, O other, A pusher arm, . empty), `launch`, `warnings` (how many are active), `overrides` (letters: C element counter ignored, P pose ignored, F flywheel open loop, - none), `check`, `fly_power` (the flywheel's motor command, 0 to 1: it switches between 1.0 and the feedforward, so average it over a steady spin to get the duty cycle that holds a speed), `fly_amps` (its motor current) (the pusher check: its stage while running, else the last result: ok, teeth+1, sweep, LOST, aborted, -). The TeleOp also has `drive_mode` (field or robot) after the drive command. Auto samples hold the pose only.

Events written when they happen:
- **Commands** (`COMMAND ...`, from the operator or an auto): intake toggle, launch queued, reject, ramp, flywheel, setpoint. Each notes the element count.
- **Salvos:** `SALVO started` and `SALVO done` (lowest flywheel speed, how far the pusher coasted and where it rested).
- **State changes:** pusher state, intake mode, element count, warnings that appear or clear.
- **Overrides** (`OVERRIDE ...`): each time one is switched on or off.
- **The pusher check** (`PUSHER CHECK ...`): started, arm seen (and at how many teeth from the expected center), the result with its reason, skipped or aborted.
- **Queued launch:** `LAUNCH queued until the lock is ALIGNED`, then `LAUNCH released: aligned after <s>` or `LAUNCH queue dropped` (released trigger, or `LAUNCH_ALIGN_WAIT_SEC` passed).
- **Auto-launch:** `COMMAND auto-launch -> ON / off`, `AUTO LAUNCH: entered the <cell> zone` (the chassis is held from then), `AUTO LAUNCH: <s> after entering the zone ... | <aim line>` when it launches, or `AUTO LAUNCH gave up` after `AutoLaunch.FREEZE_MAX_SEC`.
- **Park Dash:** `PARK DASH start (<corner>): from (x, y, deg) to (x, y), direct / via ..., power`, `PARK DASH leg <i> of <n>`, `via point (x, y) reached`, `sample` every 0.5 s, `parked after <s> s at (x, y, deg)`, `released` / `cancelled (<why>)`, `refused (<corner>): <why>`, and `dry run` with `ParkDash.DRY_RUN`.
- **Hive Dash:** `HIVE DASH start (<cell>): from (x, y, deg) to the launch spot (x, y), direct / via (x, y) ..., <n> in; under-hive routes on / off, power`, `HIVE DASH leg <i> of <n>: ... aligned, front / rear leading` or `short, not aligned`, `HIVE DASH via point (x, y) reached`, `HIVE DASH sample: ... speed <in/s>, travel <deg> off the front` every 0.25 s (the strafe-to-forward measurement), `HIVE DASH arrived`, `HIVE DASH launching`, `HIVE DASH finished: <why>`, `HIVE DASH cancelled (<why>) at (x, y, deg)`, `HIVE DASH refused (<cell>): <why>`, and `HIVE DASH dry run` with `HiveDash.DRY_RUN`; the launch itself is the usual `LAUNCH requested` line.
- **Dashboard values (from 2026-11-04):** when a run starts, one `CONFIG <class> name=value, ...` event per tunable class (`util/ConfigDump.java`), then `CONFIG CHANGED <class.name>: <old> -> <new>` for every value edited since the previous run in this app session (the Dashboard resets when the app restarts, so the first run of a session shows the compiled defaults unless edited before it). `python tools/logs/config_changes.py` prints what changed per run, `--value Flywheel.FEED_FORWARD` shows one value in every run; `python tools/logs/check_config_list.py` lists any `@Config` class missing from the dump.
- **Sensor baseline health (from 2026-11-04):** `COUNTER baselines: CS1 alpha ... dist ... shares ... ; ... | presence ON at shift 0.05, OFF below 0.03` once at the start, then `COUNTER EMPTY shift s1 s2 s3 s4 | alpha ... | dist ... | arm near CS4 / clear` once a second while the track reads empty (`Mechanisms.HEALTH_LOG_SEC`). `python tools/logs/baseline_health.py [--since YYYY-MM-DD]` reads them across runs (for example a competition day's logs after the matches) and gives each sensor OK / WATCH / RE-MEASURE from its median empty shift (a shift near the 0.03 OFF threshold is one lighting change from a phantom pollen) and, for older logs, from phantoms left after salvos (a count that stays above 0 on an empty track). Re-measure in the pit with the Color Sensor Study when it says so.
- **Hold position:** `HOLD latched at (x, y, deg) after the robot settled`, `HOLD released (the driver moved / the launch is done / pose jump) after <s> ... peak error <in>, peak command <power>` (the peak error says how hard it was pushed, the peak command how hard the motors worked), `HOLD dropped: <in> from the latched pose (a pose jump, not a push)`.
- **Pusher lock:** `PUSHER LOCK (full confirmed): ... | LOCK: seek ...`, `PUSHER LOCKED at <deg> past HOLD after <s>`, `PUSHER LOCK FAILED: the arm rests at <deg> ...`, `PUSHER UNLOCK (the operator forced the intake IN / a reject was requested)`, `PUSHER UNLOCKED: the arm is <deg> past HOLD`, `PUSHER LOCK released by the salvo`, `COMMAND reject waits for the pusher to unlock`; the sample rows have a `pusher_lock` column (`-`, `locking`, `LOCKED`, `unlocking`) and, while it is LOCKED, the `slots` column shows what each sensor reads with the arm parked.
- **Launch speed band:** `LAUNCH held: the flywheel (filtered <v> ticks/s) is ABOVE / BELOW the band <lo> to <hi>; ...` when a queued launch has to wait for the speed to come inside +-4 percent of the setpoint (`Flywheel.LAUNCH_BAND_LOW_FRACTION` / `HIGH_FRACTION`), then `LAUNCH released: the flywheel is inside the band after <s> s of waiting`; if it stays outside for `Mechanisms.LAUNCH_TIMEOUT_SEC` the launch is dropped with a notice. The band is checked only when the salvo would START.
- **Burst rows:** from when a launch is queued until 0.6 s after the salvo, the sample rows (`S,` lines) are written on EVERY loop (about 25 ms) instead of every 100 ms (`Mechanisms.BURST_LOG_SALVOS`), so the flywheel speed, pusher angle and element slots of a salvo have about 4 times the points. The hub's flywheel speed still moves in steps of 20 ticks/s (1.5 percent of 1350).
- **Launches:** `LAUNCH requested` and the operator's `REPORT landed LEFT / RIGHT` (remembered aim for the cell, before and after) and `REPORT landed LOW / UNDER / HIGH / OVER` (speed trim), each report repeating the aim of the last launch; `COMMAND remembered aim of <cell> cleared`, `DRIVER reseed cleared the remembered aim of every cell`. The aim line has the pose, target, bearing, the predicted miss from the launcher's offset (the far-cell extra is logged beside it and not included in the miss, so marks stay comparable) and the setpoint.
- **Auto timing (`Auton - Solo`):** an `AUTO TIMING <step>` line as each step ends (drives, aims, salvos, ramp deploy and stow, the flower pause), with its length and when it started and ended in seconds since Start, then one `AUTO TIMING total` line listing every step; the `DRIVE settled / TIMED OUT` lines say whether a leg arrived or hit its cap.
- **Pinpoint bring-up and hand-over (from 2026-11-05):** the TeleOp adopts the Pinpoint's own pose after an auto instead of resetting it. In order: `PINPOINT INIT decision: saved pose (x, y, deg, alliance) is <s> old, limit 300 s: adopt ...` (or `no saved pose` / `too old: reset, recalibrate and seed the default start`); `PINPOINT START mode <ADOPTED | RESET_AND_SEED | RESEEDED_AFTER_FAILED_ADOPT>, seedConfirmed, bring-up <ms>: <note>` where the note has the device status, how long it took to read READY, the device pose, loop time and rate, firmware, both encoder counts, and how far the saved pose differs (in, deg), or for a reset the seed and the read-back; `PINPOINT INIT manual reset: ...` when the driver pressed D-pad up (auto pose discarded) or changed the alliance in INIT (it says what the device read and what it was reseeded to; position only, no IMU recalibration); `PINPOINT at Start: CONTINUING from the auto / seeded ..., pose, moved <in> / <deg> since INIT, device status`. `Auton - Solo` writes `PINPOINT START ...` (its reset and seed) and `PINPOINT handed to the TeleOP: pose, alliance, device status` when it ends. Read these first for a start pose that looks wrong: `NOT adopted` lines say why the device was not trusted (not READY, NaN, `(0, 0, 0)` with the saved pose elsewhere = it restarted, outside the field). The event-day report flags a failed adoption and a manual reset.
- **Driver commands:** pose reseed, backup mode (`BACKUP MODE ENTERED`), Pinpoint faults and blips.
- **Gamepad assignment changes** (a controller that disconnects mid-match).
- **Slow loops** (`SLOW LOOP`, over 80 ms).
- **Auto:** each drive leg's start, and whether it `settled` or `TIMED OUT` with its final error.
- **A crash:** `CRASH:` and its stack trace.

## Cost and limits

- Lines are written by a background thread, so the loop only pays for building the line (about 10 times a second). The only hub read added is the battery voltage, once a second.
- The file is flushed about once a second, so a hard crash loses at most the last second.
- Not logged: raw color sensor readings (use Color Sensors Test, which is disabled), and motor currents other than the pusher's cached value.

## Competition day (2026-11-04)

Pull the logs as usual (`python tools/logs/pull_logs.py`), then label them with their matches and read the report:

```
python tools/logs/event_day.py label  --event "Regional 2026" --since 2026-11-05     # asks for the match of each unlabeled run, oldest first (free text: Q12, SF2-1, F1, practice)
python tools/logs/event_day.py label  --event "Regional 2026" --set 2026-11-05/101530 Q12   # or one run at a time, no questions
python tools/logs/event_day.py report --event "Regional 2026" --since 2026-11-05
```

A match is an auto followed by a TeleOp within 90 s of its end; a lone run is its own group. Everything goes to `robot-logs/events/<event>/`: `labels.csv` (edit it by hand to clean labels up; the raw logs are never changed), one `<label>.md` per match (flags, a table of numbers, the events worth reading, the Dashboard values that changed) and `day.md` (what stands out against the day's median once there are 4 or more matches, the hard faults, a table of every match). There is no label entry on the Driver Station: only mission-critical input goes there (user, 2026-11-04). `--auto-number R` labels every run R1, R2, ... for a rehearsal.
