# Run logs (the last auto and the last TeleOp)

`TeleOp - Competition`, `Auton - Blue Students` and `Auton - Hive Volley (Blue)` write a log of the run to the hub, so it can be read after a match. The code is `util/RunLog.java`.

## Files

They are in `/sdcard/FIRST/runlogs/` on the Control Hub:

| File | What it holds |
|---|---|
| `auto.log` | the latest auto run |
| `auto_prev.log` | the auto run before it |
| `teleop.log` | the latest TeleOp run |
| `teleop_prev.log` | the TeleOp run before it |

Starting an opmode moves its latest log to `*_prev.log` and starts a new one, so **the auto and the TeleOp never overwrite each other**. If an opmode crashes and is started again, the crash is in `*_prev.log` and the rest of the run is in `*.log`. The first line of the new log says how the previous run of that kind ended (`ended normally`, `CRASHED`, or `NO END MARKER`, meaning the app stopped hard or power was lost).

The one run that is discarded is an init that was stopped before Start without crashing.

Only the last two runs of each kind are kept, so look at them before running an opmode a third time.

## Getting it off the hub

With the hub on USB:

```bash
adb pull /sdcard/FIRST/runlogs <some-folder>
```

Then ask Claude Code to read the files in that folder.

## Format

Plain text, one line each, first field says what it is:

| First field | Meaning |
|---|---|
| `#` | comment: opmode, date (the hub's clock, which may be wrong), how the previous run ended |
| `H` | the column names of the `S` rows |
| `S` | a sample every 100 ms: seconds since the log opened, average and worst loop time since the last sample (ms), battery volts (refreshed once a second), then the opmode's columns |
| `E` | an event: seconds, then text |

TeleOp sample columns: pose (`x`, `y`, `hdg_deg`), the drive command (`cmd_fwd`, `cmd_left`, `cmd_turn`), then the mechanisms' columns: `setpoint`, `fly_target`, `fly_vel`, `fly_ready`, `pusher_state`, `pusher_deg`, `pusher_vel`, `pusher_amps`, `intake_mode`, `ramp`, `count`, `slots` (CS1 to CS4: P pollen, R red nectar, B blue nectar, O other, A pusher arm, . empty), `fire`, `warnings` (how many are active), `overrides` (letters: C element counter ignored, P pose ignored, F flywheel open loop, - none), `check` (the pusher check: its stage while running, else the last result: ok, teeth+1, sweep, LOST, aborted, -). The TeleOp also has `drive_mode` (field or robot) after the drive command. Auto samples hold the pose only.

Events written when they happen:
- **Commands** (`COMMAND ...`, from the operator or an auto): intake toggle, fire queued, reject, ramp, flywheel, setpoint. Each notes the element count.
- **Salvos:** `SALVO started` and `SALVO done` (lowest flywheel speed, how far the pusher coasted and where it rested).
- **State changes:** pusher state, intake mode, element count, warnings that appear or clear.
- **Overrides** (`OVERRIDE ...`): each time one is switched on or off.
- **The pusher check** (`PUSHER CHECK ...`): started, arm seen (and at how many teeth from the expected center), the result with its reason, skipped or aborted.
- **Driver commands:** heading reset, pose reseed.
- **Gamepad assignment changes** (a controller that disconnects mid-match).
- **Slow loops** (`SLOW LOOP`, over 80 ms).
- **Auto:** each drive leg's start, and whether it `settled` or `TIMED OUT` with its final error.
- **A crash:** `CRASH:` and its stack trace.

## Cost and limits

- Lines are written by a background thread, so the loop only pays for building the line (about 10 times a second). The only hub read added is the battery voltage, once a second.
- The file is flushed about once a second, so a hard crash loses at most the last second.
- Not logged: raw color sensor readings (use Color Sensors Test), and motor currents other than the pusher's cached value.
