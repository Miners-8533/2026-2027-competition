# Color Sensor Study (procedure)

Opmode **Color Sensor Study** (group Bring-up, `opmodes/bringup/ColorSensorStudy.java`). It records what the four element sensors CS1 to CS4 see, so the element count can be understood and improved (2026-11-03: the count reads 4 for stretches with only 3 inside, which trips a false target lock). It does not decide anything. Built 2026-11-03, **not yet run on the robot**.

## What it records

About 20 rows a second in the BRINGUP log (`/sdcard/FIRST/runlogs/bringup.log`, pulled with the log tool): for each sensor the distance, alpha, red / green / blue shares, the shift from its empty baseline, whether it is lit before the debounce, and a read counter (a changed counter means a fresh value); the counter's slot string, the number of sensors lit now, and the reported count; the pusher angle (also unwrapped, from HOLD), state, speed and current; intake mode and current; pose, speed, turn rate and the drive commands; and the take's labels. The sensors are read one every 25 ms (each about every 100 ms, the rate at which they refresh) at the SDK's default settings; the robot's own counter reads one every 40 ms. The counter's debounce and steady-count windows run as on the robot, so its count can be compared with the truth.

## Controls (gamepad 1; no Start or Back)

| Control | Does |
|---|---|
| A | start / stop a take (during a SWEEP it aborts) |
| D-pad up / down | take type: STILL, SWEEP, JOSTLE |
| D-pad left / right | the number of pollen **really inside**, 0 to 4. Set it before the take |
| X / Y / B | intake IN / OUT / OFF |
| Sticks | drive (not during a SWEEP) |
| Right bumper | log a NOTE with the time and take |

Initialize with the pusher at **HOLD** (its zero is assumed there) and the robot still. Count the pollen by eye and set the number before every take.

## Takes

1. **Empty, STILL** (known 0, intake OFF), about 20 s: the baseline.
2. **Empty, SWEEP** (known 0): the pusher arm's signature alone at each sensor.
3. **4 pollen at HOLD, STILL** (known 4), about 20 s: the full baseline.
4. **4 pollen, SWEEP** (known 4): load 4, check by eye, set 4, A. About 2 s still, then the pusher turns backward one full turn slowly with the intake OUT (the pollen leave in order past each sensor), then about 2 s still. The pollen end up on the floor in front of the intake: reload before the next take.
5. **JOSTLE** takes, one per known count. Load the known number, check by eye, set it, A, then drive: straight and brake, spin in place, a figure eight, a few bumps, about 60 s. Do 3 to 4 takes each at **3** and **4** pollen, and 2 each at 0, 1 and 2. For 3 pollen do some takes with the intake OFF and some with it IN (the situation of the false lock); other counts with it OFF.

Press A to stop each JOSTLE take. Between takes you may move pollen or reload freely (not recording).

## Reading the log

```
python tools/logs/sensor_study.py            # the newest bringup log
```

It prints each take (how often the sensors lit now and the reported count were below / equal to / above the known number), then the same pooled by known count with the false-FULL rate, the median reading per sensor for the STILL takes, and for the SWEEP takes every stretch where a sensor was lit with the pusher angle it covered (pollen and arm passes). Dashboard values on the opmode: `SENSOR_READ_PERIOD_MS`, `SAMPLE_PERIOD_MS`, `SWEEP_POWER` (0.15, backward, open loop; raise it if the arm does not move: the take aborts after 2 s), `SWEEP_DEG`, `LEAD_SEC`, `TAIL_SEC`, `DRIVE_SCALE`, `TURN_SCALE`.
