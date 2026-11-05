# Color Sensor Study (procedure)

Opmode **Color Sensor Study** (group Bring-up, `opmodes/bringup/ColorSensorStudy.java`). A guided check of the four element sensors CS1 to CS4 for after the rotary transport (or the lighting) changes: it measures the new empty baseline, shows what each sensor sees of the pusher arm, shows the full-load readings, and shows the pollen sliding past the sensors in a slow reject. Reworked 2026-11-05 from the longer study of 2026-11-03 (the jostle takes and the hand-set pollen count are gone). **First run 2026-11-05 19:32 (log `193256_bringup.log`):** the transport's new color put CS1 to CS3 over the 0.05 ON threshold on an empty track (shifts 0.10, 0.14, 0.06); the new baseline was pasted into `ElementCounter` the same day. The arm reads PUSHER_ARM at CS4 (and OTHER at CS1 to CS3), 4 pollen at HOLD read POLLEN on all four sensors, and CS4 shows a long OTHER stretch near its empty baseline in the slow reject.

Why: the transport's color changed (2026-11-05) and the empty readings are mostly the transport, so the stored baselines in `ElementCounter` (`BASE_ALPHA`, `BASE_DIST`, `BASE_SHARES`) are probably wrong. A wrong baseline gives a phantom element or a missed one (the false full of 2026-11-03 was exactly this).

## Run it

Initialize with the pusher at **HOLD** (its zero is assumed there), the track **empty**, and the robot still. Nothing else moves: no drive, no flywheel. Gamepad 1 (no Start or Back): **A** starts the step shown and stops a running one, **D-pad left / right** goes to the previous / next step (to repeat or skip one), **right bumper** logs a NOTE, and **holding X** runs the intake IN so pollen can be loaded (between steps only, off while a step runs; a loading aid, not part of a test).

| Step | You do | The opmode does | You get |
|---|---|---|---|
| 1. Empty baseline | track empty, still | records about 8 s | the median of each sensor = the new baseline, printed as the three lines to paste into `ElementCounter`, with the spread of each |
| 2. Pusher sweep, empty | track still empty | 2 s still, the pusher turns back one full turn slowly (power 0.15), 2 s still | every stretch where a sensor sees something (against the baseline just measured) with the pusher angle, and what the counter would call it (the arm should read PUSHER_ARM at CS4) |
| 3. Full baseline | load 4 pollen, check by eye | records about 8 s | what each sensor reads with 4 pollen at HOLD, and whether it counts as lit |
| 4. Full slow reject | the same 4 pollen | as 2, with the intake OUT | the lit stretches as in 2: the pollen and the arm passing each sensor. The pollen end up on the floor in front of the intake |

If a sweep is aborted (A, or the arm does not move: raise `SWEEP_POWER` on the Dashboard), the arm is left where it stopped: stop the opmode and initialize again with the arm at HOLD before the next sweep.

## After it

1. Pull the log (`python tools/logs/pull_logs.py`) and run `python tools/logs/sensor_study.py`: it prints each step with the BASELINE and RESULT lines.
2. Copy the three BASELINE lines into `ElementCounter` (`BASE_ALPHA`, `BASE_DIST`, `BASE_SHARES`), note the date in the comment above them, and rebuild. The screen shows the same lines, so it can be done in the pit without the log.
3. Look at steps 2 and 4: does CS4 still call the arm PUSHER_ARM (the thresholds `ARM_MIN_BLUE`, `ARM_MAX_RED`, `ARM_MAX_GREEN_BLUE_GAP`, `ARM_MIN_DIST_CM` assume the old colors)? Does pollen still read POLLEN (`POLLEN_MIN_GREEN_OVER_BLUE`, `POLLEN_MAX_RED`)? Do other sensors see the arm too (they should not count it)? In step 3, are all four sensors lit with four pollen?
4. Check the stored-baseline columns: in the log the `shift` and `on` columns are against the baseline stored in `ElementCounter`, so before the update they show how far off it is (any sensor lit on an empty track is a phantom element).

The log is the BRINGUP kind (`/sdcard/FIRST/runlogs/bringup.log`), about 20 rows a second: for each sensor the distance, alpha, red / green / blue shares, the shift from the stored baseline, whether it is lit, and a read counter (a changed counter means a fresh value); the counter's slot string, the lit count and the reported count; the pusher angle (also unwrapped, from HOLD), state, speed and current; the intake mode and current. The sensors are read one every 25 ms (each about every 100 ms, the rate at which they refresh) at the SDK's default settings; the robot's own counter reads one every 40 ms.

## Checking the baselines after a match

`python tools/logs/baseline_health.py --since <competition day>` reads the `COUNTER EMPTY` lines the TeleOp and autos write once a second while the track is empty (the colour shift of each sensor from its stored empty baseline) and says OK, WATCH or RE-MEASURE per sensor; re-measure with step 1 of this opmode when it says so (see design/run-logs.md).

Dashboard values on the opmode: `SENSOR_READ_PERIOD_MS`, `SAMPLE_PERIOD_MS`, `STILL_SEC`, `STILL_SETTLE_SEC`, `SWEEP_POWER`, `SWEEP_DEG`, `SWEEP_MAX_SEC`, `SWEEP_NO_MOVE_SEC`, `LEAD_SEC`, `TAIL_SEC`.
