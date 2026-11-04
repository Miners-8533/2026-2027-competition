# Pit check

The Pit Check is the only check opmode on the Driver Station. It took over the **Hardware Check** and **Chassis Info** opmodes (removed 2026-11-02). The calibration tools **Shot Map Tuner**, **Pinpoint Offset Check**, **Pinpoint Push Test** and **Color Sensors Test** still exist but are `@Disabled`; see [opmodes.md](opmodes.md).

`Pit Check (robot on blocks)` (opmode, group Bring-up) is the check to run after any repair, rewiring or hub swap, and before a match day. The code no longer watches for wiring and sign problems while running (the flywheel's reversed-encoder fault was removed), so this is where they are found.

**Setup:** the robot on blocks, wheels clear of the floor, the ramp free to swing, the element track **empty**. Nothing moves until you press A for that step, every move is short and slow, and Back stops everything.

## Gamepad 1

| Button | Meaning |
|---|---|
| A | run the step, or "what I saw was right" |
| B | "what I saw was wrong" |
| X | skip the step, or "I could not tell" |
| Back | stop everything and end |

## What it checks

| Step | How | Judged by |
|---|---|---|
| Configuration | every device name and type, a stale `fly_follow`, where each device is plugged in (hub and port, in the `Ports` line, for hub-configuration.md) and any configured device the code does not use | automatic |
| Battery | at least 12.0 V on every hub | automatic |
| Pinpoint | status READY, loop time 300-1500 us | automatic |
| Color sensors CS1-CS4 | each answers, and matches its empty baseline (so a changed lighting or something left in the track shows up) | automatic |
| Flywheel | spins at 0.3 power for 1.5 s: the encoder must read at least 100 ticks/s the right way | encoder automatic; the launching direction is yours to judge |
| Pusher | turns about 60 deg forward and back: the encoder sign and the return to the start | sign and return automatic; the pushing direction is yours to judge |
| Intake | pulses IN then OUT at half power; the motor current is shown | direction is yours to judge |
| Ramp | deploys and stows | yours to judge |
| Drive, each wheel | one wheel at a time at 0.3 power | yours to judge: only that wheel turns, its top moving toward the front |

Each failure says what to fix (for example "flip `Pusher.ENCODER_SIGN`" or "check the cable on the encoder port").

## Where the result goes

On screen, in the robot log (`adb logcat -s PitCheck`), and in `/sdcard/FIRST/runlogs/pitcheck.log` (with `pitcheck_prev.log` for the run before; see [run-logs.md](run-logs.md)). The last line is `PIT CHECK ALL PASS` or `FAILED` or `INCOMPLETE`, with the counts.

## Limits

- It uses the raw hardware, not the `Pusher` or `Flywheel` classes, so it does not change the pusher's saved zero. It returns the pusher to where it started; if it cannot, the report says by how much, and the arm must be put back at HOLD by hand before any other opmode runs.
- The drive wheels have no encoders, and on blocks the Pinpoint sees nothing, so wheel direction is judged by eye.
- It does not test the dynamic behaviors that need real elements (the intake auto-stop, the arm seen at CS4). Those belong to the Comp TeleOp tests and the pusher check.
