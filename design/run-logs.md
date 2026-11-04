# Run logs (the last auto and the last TeleOp)

`TeleOp - Competition`, `Auton - Solo`, `Auton - Blue Students` and `Auton - Hive Volley (Blue)` write a log of the run to the hub, so it can be read after a match. The code is `util/RunLog.java`.

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
- **Shots:** `LAUNCH requested` and the operator's `REPORT landed LEFT / RIGHT` (remembered aim for the cell, before and after) and `REPORT landed LOW / UNDER / HIGH / OVER` (speed trim), each report repeating the aim of the last launch; `COMMAND remembered aim of <cell> cleared`, `DRIVER reseed cleared the remembered aim of every cell`. The aim line has the pose, target, bearing, the predicted miss from the shooter's offset (the far-cell extra is logged beside it and not included in the miss, so marks stay comparable) and the setpoint.
- **Auto timing (`Auton - Solo`):** an `AUTO TIMING <step>` line as each step ends (drives, aims, salvos, ramp deploy and stow, the flower pause), with its length and when it started and ended in seconds since Start, then one `AUTO TIMING total` line listing every step; the `DRIVE settled / TIMED OUT` lines say whether a leg arrived or hit its cap.
- **Driver commands:** pose reseed, backup mode (`BACKUP MODE ENTERED`), Pinpoint faults and blips.
- **Gamepad assignment changes** (a controller that disconnects mid-match).
- **Slow loops** (`SLOW LOOP`, over 80 ms).
- **Auto:** each drive leg's start, and whether it `settled` or `TIMED OUT` with its final error.
- **A crash:** `CRASH:` and its stack trace.

## Cost and limits

- Lines are written by a background thread, so the loop only pays for building the line (about 10 times a second). The only hub read added is the battery voltage, once a second.
- The file is flushed about once a second, so a hard crash loses at most the last second.
- Not logged: raw color sensor readings (use Color Sensors Test, which is disabled), and motor currents other than the pusher's cached value.
