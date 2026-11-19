# Sensors (ROBOT_1)

Everything the software can measure. The wiring is in [hub-configuration.md](hub-configuration.md); the odometry is in [chassis.md](chassis.md).

| Sensor | Measures | Used for |
|---|---|---|
| goBILDA Pinpoint with two pods | field position, heading, velocity | all driving, aiming and distance to the hive |
| `CS1`, `CS3`, `CS4` (REV Color Sensor V3) | color and distance at three places along the track | which elements are held; where the pusher arm is |
| `CS2` (REV Color Sensor V3) | color and distance at the top of the feed ramp (moved there, reported 2026-11-18; it was the second place along the track) | meant to count the elements a salvo launches (team, 2026-11-18); for now its proximity is recorded through every salvo and nothing is counted |
| `CS5` (REV Color Sensor V3) | proximity at the intake entrance | counting elements as they enter |
| Pusher motor encoder | arm angle and speed | every pusher move |
| Flywheel motor encoder | wheel speed | launch readiness |
| Motor currents (pusher, flywheel, intake, each drive wheel) | amps | jam detection, fault diagnosis, broken-belt detection |
| Hub battery voltage | volts | logging and diagnosis |

There is no camera in use, no limit or home switch, and no encoder on the drive wheels. The servos have no feedback.

## The color sensors along the track: CS1 to CS4

Four REV Color Sensor V3. Until 2026-11-17 they were numbered in the order an element passes them. **`CS2` has since been moved to the feed ramp** (next section), so the order an element passes them is now:

```
intake -> CS1 -> CS3 -> CS4 -> feed ramp (CS2) -> flywheel
```

`CS4` is the deepest of the three left in the track. Where each sits along the track in inches or degrees: **NOT RECORDED**.

The rest of this section was measured with `CS2` in its old place, between `CS1` and `CS3`.

**How they behave.**

- Each gives color (red, green, blue, and a brightness called alpha) and a distance.
- With the SDK's standard settings a sensor produces a new value only every 100 ms, and one read takes about 13 ms. Reading all four in one loop would cost about 54 ms, so they must be read one at a time.
- The sensor's hardware gain is fixed at 3x; the software gain is 1.0 (a higher value only clips bright readings).
- `CS1` to `CS3` look at a near wall of the track. Brightness and distance change little when an element arrives, so **color is the reliable cue**, judged against each sensor's own empty reading.

**Empty-track readings** (measured 2026-11-05, after the track was recolored).

| Sensor | Alpha | Distance (cm) | Red / green / blue share |
|---|---|---|---|
| CS1 | 0.209 | 3.60 | 0.238 / 0.410 / 0.352 |
| CS2 (its old place; **not valid since the move**) | 0.109 | 5.20 | 0.220 / 0.448 / 0.332 |
| CS3 | 0.353 | 3.92 | 0.226 / 0.414 / 0.360 |
| CS4 | 0.061 | 7.67 | 0.240 / 0.408 / 0.352 |

These depend on the lighting and on the track's color. A stale empty reading shows up as a phantom element (this happened on 2026-11-03), so they are re-measured in the pit when the lighting changes. They cannot be captured at the start of a match, because the robot starts with elements in it.

**What each thing looks like** (red / green / blue share).

| In view | Signature |
|---|---|
| Pollen (yellow) | about 0.28 to 0.31 / 0.44 to 0.51 / 0.18 to 0.27; green at least 1.5 times blue; up close the brightness saturates |
| Red nectar | about 0.36 to 0.47 / 0.32 to 0.36 / 0.21 to 0.27 |
| Blue nectar | about 0.11 to 0.17 / 0.24 to 0.37 / 0.45 to 0.64 |
| Pusher arm (grey), at CS4 | about 0.23 / 0.38 / 0.39, green close to blue, about 5 cm away |

A dim blue nectar at CS4 is close to the arm's color; the arm is told apart by its green being nearly equal to its blue.

**The pusher arm at CS4.** CS4 sees the arm once per turn. This is the robot's only check of the arm's position.

| Quantity | Value |
|---|---|
| Center of the arm's arc | 326.7 deg forward of HOLD (33 deg behind it) |
| Arc width at half brightness | about 28 deg at rest, wider at speed (up to about 38 deg) |
| Window where it is classified as the arm when parked | about 320 to 332 deg |
| Sensor delay | about 15 ms: the arc appears later the faster the arm moves |
| Repeatability of the arc center | 0.07 deg at 124 deg/s, 0.4 deg at 340 deg/s, 1.0 deg at 742 deg/s |

- One skipped belt tooth (18 deg) moves the arm out of the parked window, so a parked look detects any slip.
- The arm's edge has a long tail toward HOLD: with the arm resting a few degrees to about 60 deg past HOLD, CS4 can read an unclassified object. After a salvo the arm rests in that zone (40 to 60 deg).
- `CS1` to `CS3` also see the arm as an unclassified object when it sweeps past them.

## CS2 on the feed ramp

`CS2` was moved from the track to the feed ramp, the fixed end of the track that turns the elements up into the flywheel. Reported 2026-11-18. In the ten runs of 2026-11-17 it was still in its old place: its empty reading matched the stored one in every run (alpha 0.114 to 0.118, 5.1 to 5.2 cm).

| Quantity | Value |
|---|---|
| Where | at the top of the feed ramp, past the HOLD position (team, 2026-11-18). Inches or degrees: **NOT RECORDED** |
| What it is for | counting the elements a salvo launches (team, 2026-11-18) |
| Whether anything rests in its view with the arm at HOLD and 4 held | taken to be nothing, since it sits past HOLD: elements pass it only on their way into the flywheel. **Not confirmed on the robot** |
| Empty reading in the new place | **NOT MEASURED** |
| Pollen, red nectar, blue nectar in the new place | **NOT MEASURED** |
| The pusher arm in its view: at which angles, and whether the arm rests there after a salvo (it rests 40 to 60 deg past HOLD) | **NOT MEASURED**; the arm sweeps through the feed ramp, so it must pass |
| How long one launched element is in view | **NOT MEASURED** |
| Hub, bus and name | unchanged: Expansion Hub, bus 2, `CS2` (team, 2026-11-18). Only its place changed |

- With the standard sensor settings (a new value every 100 ms) it cannot follow single elements through a salvo: a salvo launches four in about 1.2 s. From 2026-11-18 it is put in the fast proximity mode CS5 uses (8-bit, a new value every 6.25 ms) and is no longer read by the slot count. In that mode its distance reading means nothing.
- The software records it and counts nothing yet: [specs/element-counting.md](../specs/element-counting.md) sections 1.8 and 6. Every NOT MEASURED line above waits on the first recordings.

## The entrance sensor: CS5

A fifth REV Color Sensor V3 at the intake entrance, ahead of `CS1`. Its **proximity** counts the elements. Its **color** names each one (from 2026-11-17, not run on the robot; the color table is below).

| Quantity | Value |
|---|---|
| Sensor setting | fast proximity mode: 8-bit, a new value every 6.25 ms (the standard setting gives one every 100 ms, too slow) |
| Read cost | one 2-byte read, about 3 ms |
| Empty reading | 65 to 67 counts, flickering between two neighbouring counts |
| One pollen passing | a bump of 8 to 44 counts above empty, above the noise for about 80 to 120 ms |
| Hard arrivals in real driving | up to about 160 counts above empty, with ringing |
| Touching elements in a clump | the dips between them are only 2 to 4 counts above empty |

**CS5's color** (the raw 16-bit green, blue and red channels, without the SDK's channel factors; one run, 2026-11-14 13:27, 31 pollen, the robot standing still):

| Quantity | Value |
|---|---|
| Sensor setting | 16-bit, a new value every 25 ms |
| Read cost | one 15-byte read that also returns the proximity: 6.8 ms measured, against 5.6 to 6.4 ms for the 2-byte read in a match loop |
| Empty reading | green about 2385, blue about 1450, red about 1200; each wanders by about 15 counts within a run |
| One pollen passing | green rises 60 to 140 counts, red 40 to 90, blue under 20 |
| Pollen: shares of the rise | green 0.52 to 0.65, blue 0.00 to 0.13, red 0.31 to 0.46 |
| Timing against the proximity | the color starts to rise one or two samples before the proximity reaches the count's threshold, and takes two or three samples longer to fall back |
| Red nectar, blue nectar | **not measured** |

- The empty color level in a moving robot (other light, the field wall or a flower stand in view) has not been measured.
- `CS5` does **not** see the pusher arm: its reading stays at the empty level through salvos and rejects.
- An element parked under the sensor holds the reading up for as long as it stays.
- Only pollen has been characterized. Nectar may reflect differently.
- The signal depends on how the sensor is mounted.

## Encoders

| Encoder | Resolution | Notes |
|---|---|---|
| Pusher (`gate`) | 751.8 ticks per arm turn (0.48 deg per tick) | relative; counts down going forward; never reset by the software |
| Flywheel (`fly_motor_enc`) | ticks per wheel turn **NOT RECORDED** | the hub's speed reading moves in 20 ticks/s steps |

Both are read through the hub's bulk read, once per loop.

## Motor currents

Readable from the hubs, 2 to 3 ms per read, so they are sampled sparingly.

| Motor | Normal | Abnormal |
|---|---|---|
| Pusher | 0.22 A moving | 1.4 to 2.5 A at zero speed is a jam |
| Intake | 1.0 A in, 1.5 A out | sustained above 6 A or below 0.4 A while commanded (first guesses) |
| Each drive wheel | about 1.6 A while driven | one wheel far below the others is a broken or slipping belt |
| Flywheel | logged only | meant to tell a dead encoder from a dead or jammed motor |

## Battery voltage

Each hub reports its voltage. The reading is noisy and sags with every motor, so it is used for logs and diagnosis, not for control.
