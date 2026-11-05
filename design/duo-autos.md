# Duo autos

Two matched autos for an alliance whose partner runs the other one: **Auton - Duo Flower Side** (starts at the audience wall) and **Auton - Duo Garden Side** (starts at the rear wall). Built 2026-11-04 on branch `duo-autos`; **not yet run on the robot**. Code: `AutonDuoBase` (INIT, log, run), `AutonDuoFlowerSide`, `AutonDuoGardenSide`, `DuoKit` (the drive legs, timed steps, launch, waits; copied from `AutonSolo`) and `DuoAutos` (every Dashboard value). Blue/red is picked at INIT (D-pad right / left); red is the blue path turned 180 degrees about the field center. The rules behind this are in [game-rules.md](game-rules.md).

## Why two
A hive cell tips when it is filled: the upward cell starts with 3 nectar, so 3 more pollen tip it; the cell that then faces up is empty and needs 8 pollen. A tip takes about 4 s (assumed, conservative), and a robot must not launch into a hive that is tipping. The two robots share the tips: each launches only into the cell on its own start side, so neither needs the far-cell aim correction (`AimAtCell` is given extra 0).

| Time (s, planned) | Garden side (rear wall) | Flower side (audience wall) |
|---|---|---|
| 0 | flywheel up; launch the preload into the rear cell: 4 pollen on 3 nectar, **tip 1** (20 points, ends about 2.7 s) | flywheel up, waits |
| 7.2 | gathers the Blue Wall flower (the audience-wall flower turned 90 degrees) | launches its preload into the audience cell, gathers the audience-wall flower, launches it: the 8th pollen, **tip 2** |
| 25 | back at the rear launch spot, waits until `G_SECOND_LAUNCH_NOT_BEFORE_SEC` (the partner's tip is over), launches the flower pollen; gathers the garden; launches again: the 8th pollen, **tip 3** | parks |
| end | parks in the loading zone | |

Waits are absolute seconds from Start (a robot cannot see its partner): `F_FIRST_LAUNCH_NOT_BEFORE_SEC` = 2.7 s of G's salvo + 4 s tip + 0.5 s margin = 7.2 s; `G_SECOND_LAUNCH_NOT_BEFORE_SEC` = 25 is a first guess. Both are logged as `AUTO WAIT ... waited x s (on time | the time had already passed)`. If a wait is longer than the robot needs, the robot idles with the flywheel spinning.

## Poses (blue frame, all in `DuoAutos`)
- Starts: F back flush to the audience wall at (63.4, 7.5), heading 180; G like the solo auto, (-63.4, 16.0), heading 0.
- Launch spots: F (61.5, 7.5), G (-61.5, 15.25); the heading is the computed aim at the own-side cell with no far-cell extra, and `AimAtCell` takes out what is left.
- Flowers: F uses the solo auto's audience-wall flower numbers; G uses the same poses turned 90 degrees about the center (pre (-26.3, 38, 90), ram target (-26.3, 67.3), back-out to y 46.3) plus `WALL_FLOWER_DX_IN/DY_IN` to refine on the field.
- Garden (G): the solo auto's angled approach, slow pickup and square on the wall.
- Parks: F (40, 60, 90) through (46, 32); G (26, 60, 90) through the helper (-36, 40); both facing the Blue Wall in the loading zone (x 23.9 to 46.6, y 59.1 to 70.1; the line only has to be broken). Overshoot is 0 (the front edge is already at y 69.9).
- Every auto ends with the ramp stowed (a final `ramp stow` step).

## Checks
`tools/hive/DuoPathCheck.java` (run with `tools/hive/run-check.cmd`) sweeps the robot's footprint along every leg against the hive supports: all 15 legs clear, the least clearance 4.7 in (G launch spot to the Blue Wall flower pre-position). Copies of the default poses are in it: change both together.

## Open points (tune from the first logged runs)
- **G probably overruns 30 s.** The flower gather, the wait to 25 s, then the garden gather, the return and a second salvo do not fit if the wait is really 25 s. The first thing to look at in the `AUTO TIMING` lines: move the garden gather in front of the wait (it then happens while the partner tips), or cut it. Not done yet on purpose (nothing was optimized before it was timed).
- Aim trims `*_TRIM_DEG` are 0 for all five launches (new launch spots).
- If the partner misses pollen, the 8-pollen tip does not happen and the waits are wasted; the plan assumes every launch goes in.
- The Blue Wall flower poses are a rotation of the audience flower's tuned numbers, not measured on the field.
