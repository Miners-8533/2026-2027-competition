# Mechanisms (ROBOT_1)

Four actuators on top of the chassis: the intake, the pusher, the flywheel (with a position servo) and the ramp. This page gives their hardware and the physical constants. What the software does with them belongs in `specs/`.

## The element path

Everything enters and leaves at the **front** of the robot.

```
floor or flower -> [ramp] -> intake -> CS5 -> track: CS1, CS3, CS4 -> feed ramp (CS2) -> flywheel -> out the front
                                              (the pusher arm sweeps this track)
```

- The robot handles two elements: **pollen** (about 2.8 in) and **nectar** (about 3.6 in).
- The **track** is a curved channel the elements ride along. With the pusher arm at its HOLD position the track holds **4 pollen**, which is also the most a robot may control (rule G407).
- The stack fills from the flywheel end: the first element in comes to rest at CS4.
- The **feed ramp** is the fixed end of the track that turns the elements up into the flywheel. It is not the deployable ramp.
- There is **no mechanical detent** at the entrance. With the intake stopped and the arm not across the entrance, held elements can roll back out.
- The track was recolored on 2026-11-05, which changed what the color sensors read when it is empty.

**Starting configuration:** ramp stowed, pusher arm at HOLD, four pollen preloaded in the track, the whole robot inside the 18 in cube.

## Intake

| Item | Value |
|---|---|
| Actuator | one motor, `intake`, no encoder; model **NOT RECORDED** |
| Position | at the front, the same end as the launcher |
| Takes | pollen and nectar, from the floor and from the deployed ramp |
| Operating points | full power in; half power out |
| Free-running current | about 1.0 A in, about 1.5 A out |

The intake failed mechanically once (2026-11-05) with no sign in the data of the time; its current is now readable for that reason.

## Pusher

One arm, with some thickness, that rotates around the curved track and pushes the stack to the flywheel.

| Item | Value |
|---|---|
| Actuator | one motor, `gate`: goBILDA 5203 series, 26.9:1 |
| Drive | a belt; the pusher pulley has **20 teeth**, so one skipped tooth is 18 deg of arm rotation |
| Encoder | relative (no index), 751.8 ticks per arm revolution, confirmed to about 0.03 deg per turn |
| Encoder polarity | counts **down** when the arm moves forward |
| Hard stops | **none**: the arm turns continuously, almost always forward |
| Belt ratio | 1:1 is an inference (the motor's own 751.8 ticks per output turn equal one arm turn); **NOT RECORDED** |
| Arm length, track radius | **NOT RECORDED** |
| Arm color | grey (the color sensors tell it from the elements by this) |

**Angles.** The arm's position is an angle on a circle, measured forward from HOLD.

| Position | Angle | What it is |
|---|---|---|
| HOLD | 0 deg | the back of the arm limits the track to 4 elements; the arm must be here at power-up |
| Lock | 90 deg | the arm lies across the intake entrance: nothing rolls out and a fifth element cannot enter (the autos' preload position) |
| Launch ready | 141 deg (2026-11-19: 156 moved back 15 deg by the user, to be farther from the flywheel; it landed at 152 to 160 deg) | the TeleOp's lock position: behind the stack with the first pollen at the bottom of the feed ramp. The first pollen is in front of CS2 (top of the feed ramp) after 171 deg of arm travel, the next three at 226, 288 and 328 deg (192 salvos, 2026-11-18) |
| Center of CS4's view of the arm | 326.7 deg (33 deg behind HOLD) | where the arm is parked to check its position |
| Rest after a salvo | 40 to 60 deg | a salvo turns the arm a full turn and 40 deg more |

- The arm sweeps the whole track, through the feed ramp, so it reaches the last element.
- A full salvo from HOLD is 400 deg of travel; from the lock position it is 310 deg.
- HOLD placed by eye varies by about 5 deg.

**The encoder is the only position reference.** There is no home switch. The software assumes the arm is at HOLD when the robot is initialized, and CS4 seeing the arm is the only way to detect and correct a wrong zero or a skipped belt tooth.

**Measured behavior.**

| Quantity | Value |
|---|---|
| Speed at power 0.30 | about 354 deg/s; a salvo takes about 1.2 s |
| Speed at power 0.37 | about 433 deg/s; a salvo takes about 1.0 s |
| Travel to the lock position | 0.3 to 0.5 s |
| Current while moving | median 0.22 A (99th percentile 0.9 A, up to 4.4 A at the start of a move) |
| Current when stuck | 1.4 to 2.5 A at zero speed; the motor stalls at only about 2.5 A |
| Holding at rest | the motor brake resists motion but not a steady push: left alone, the arm creeps more than 10 deg from where it stopped |

## Flywheel (the launcher)

| Item | Value |
|---|---|
| Actuator | one motor with an encoder, `fly_motor_enc`; model **NOT RECORDED** |
| Second motor | removed from the robot (`fly_follow` is gone) |
| Hood | fixed angle; value **NOT RECORDED** |
| Wheel size, ticks per wheel turn | **NOT RECORDED** (all speeds are in encoder ticks per second) |
| Encoder polarity | counts up when the wheel turns the launching way |
| At zero power | floats: the wheel can only coast down, it is never braked |
| Launch line | leaves the front, 2.5 in to the **right** of the robot's centerline, parallel to it |

The 2.5 in is the value in use; the Launcher Offset Finder measured 2.2 in on 2026-11-03.

**Measured behavior.**

| Quantity | Value |
|---|---|
| Top speed | about 2200 ticks/s |
| Launch speeds in use | 1325 to 1650 ticks/s, for 28.6 to 82.2 in from the hive cell |
| Power that holds 1350 ticks/s | 0.38 to 0.42 (more on a lower battery) |
| Spin-up from rest to launch speed | 1.1 s or more |
| Speed reading | moves in steps of 20 ticks/s (1.5 percent at 1350) |
| Speed during a salvo | dips as each element passes and is not recovered between elements |

### Flywheel position servo

A servo, `fly_pos`, moves the flywheel between a position for launching pollen and one for launching nectar. What it moves mechanically: **NOT RECORDED**.

| Item | Value |
|---|---|
| Pollen position | 0.45 |
| Nectar position | 0.30 |
| Travel time | 0.25 s to nectar, 0.10 s to pollen (timed by hand) |
| Feedback | none; the servo sags when no opmode drives it, so its position is unknown until first commanded |

On this branch the servo only ever holds the **pollen** position. The launch speeds above were measured there. A nectar jams at the pollen position and a pollen misses from the nectar position.

## Ramp

Two servos lower a ramp so that pollen released from the bottom opening of a FLOWER rolls straight into the intake.

| Item | Value |
|---|---|
| Actuators | two goBILDA torque servos, `depolLeft` and `depolRight`, in positional mode; part number **NOT RECORDED** |
| Arrangement | a mirrored pair moving one ramp |
| Stowed | left 1.0, right 0.0 (the full range again from 2026-11-21, the user; it was left 0.75, right 0.25) |
| Deployed | left 0.0, right 1.0 |
| Travel time | 0.8 s (measured by the team) |
| Feedback | none; the positions are commanded values |
| Reach from the robot origin when deployed | **NOT RECORDED** (the code uses a 24 in placeholder as "clear of a flower") |

- The stowed position was moved toward deployed in steps (from left 1.0 / right 0.0) at the team's request; it must still keep the robot inside the 18 in cube.
- The servos sag when no opmode drives them, so after an auto the ramp may hang partly down.
- The flower's retrieval opening is about 3.55 in tall and 3.57 in deep. Only pollen may be taken from a flower (rule G418).
- **The deployed ramp can catch on the flower stand** if the robot turns while still against it. Back straight out first.
- Whether the launcher works with the ramp deployed has not been established.
