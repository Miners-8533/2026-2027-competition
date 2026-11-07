# Duo autos

Two matched autos for an alliance whose partner runs the other one: **Auton - Duo Flower Side** (starts at the audience wall) and **Auton - Duo Garden Side** (starts at the rear wall). Built 2026-11-04 on branch `duo-autos`, merged into `integration` 2026-11-05 to test with everything else; **not yet run on the robot**. Code: `AutonDuoBase` (INIT, log, run), `AutonDuoFlowerSide`, `AutonDuoGardenSide`, `DuoKit` (the drive legs, timed steps, launch, waits; copied from `AutonSolo`) and `DuoAutos` (every Dashboard value). Blue/red is picked at INIT (D-pad right / left); red is the blue path turned 180 degrees about the field center. The rules behind this are in [game-rules.md](game-rules.md).

## Duo Daryl (2026-11-06, the user's design; `AutonDuoDaryl`, Dashboard values `DuoAutos.D_*` and the shared `P_*` speeds and spots; not yet run on the robot)

The rear-wall start with the garden first. Blue frame (red is turned 180 deg):

1. The pusher moves to the lock position while the flywheel spins up (`Mechanisms.preparePusherForLaunch`, see below); preload into the REAR cell.
2. The garden in ONE leg from the start straight to the wall squaring pose (-61.15, 58.5, 90 deg), the last `D_GARDEN_SLOW_ZONE_IN` 12.5 in limited to `D_GARDEN_SLOW_POWER` 0.5 (`DriveToPose.withSlowZone`; four pollen of about 2.8 in end to end are 11.2 in; the solo auto's 0.35 was relaxed by the user), then the squaring (side wall, front wall).
3. Around the outside via the east helper (36, 40) to the audience launch spot (50, 26.3); launch into the AUDIENCE cell (far-cell aim extra).
4. The Blue Wall flower (the audience flower's poses turned 90 deg), jostle by `D_WALL_JOSTLE_DEADLINE_SEC` 19 s at the latest, back out; the ramp stows.
5. The partner's window, an ABSOLUTE time (the user, 2026-11-06: our preload ends at about 3 s, the tip takes 4 s, plus 2 s of buffer, so the partner shoots at about 9 s and parks by about 18 s): the robot does not leave the flower before `D_SECOND_SHOT_NOT_BEFORE_SEC` 18 s minus `D_RETURN_TRAVEL_SEC` 2.4 s, and waits at the spot until 18 s if it still arrives early (`AUTO WAIT ...` lines). The nominal arrival is about 20 s, so it normally does not wait at all; no pause, no extra jostle.
6. Back to the audience launch spot, launch into the AUDIENCE cell again (the intake stays IN through it).
7. The audience flower (the solo auto's poses), jostle by `D_AUD_JOSTLE_DEADLINE_SEC` 26 s at the latest; skipped (`AUTO SKIP audience flower`) when it would start after `D_AUD_FLOWER_LATEST_START_SEC` 22.5 s so the park keeps its time. Its pollen stay in the robot for the TeleOp.
8. Park in the hive-side corner (26, 60), ramp stowed, flywheel off.

Estimate (2026-11-06, from the logs; about +-1.5 s): preload done 2.7, garden leg 1.3 and squaring 1.3 (5.3), around to the audience spot 3.9 and salvo 1.8 (11.0), to the Blue Wall flower 3.0 and its gather 3.7 (17.7), back at the spot at about 20.1 (the second salvo starts about 20.4) and done at 22.0, audience flower 4.9 (26.9), park 1.5 = **about 28.4 s**; the audience flower starts at about 22 s against the 22.5 s guard, so it is skipped when anything runs late. Ideas to win time if it is skipped too often: cap the audience flower's jostle (about 0.9 s), skip its pre-position settle. Path check (`tools/hive/run-check.cmd`): the Daryl legs clear the supports and the flower stands grown 2 in.

## Preload pusher pre-position (2026-11-06, all autos)

`Mechanisms.preparePusherForLaunch()` moves the arm from HOLD to the lock position (90 deg) at the very start of every auto (Solo, Flower Side, Garden Side, Primary, Daryl) while the flywheel spins up (1.1 s or more to reach the launch band; the arm needs about 0.6 s), so the preload salvo turns 310 deg instead of 400: median 1.08 s against 1.37 s in the logs (n = 135 and 77), about 0.3 s per auto. Only the preload: later salvos already start from the arm's rest about 45 deg past HOLD (median 1.22 s, so only about 0.14 s to gain), and a locked arm would block the stragglers the autos keep the intake open for. The queued salvo waits for the arm to stop (`pusher.isStopped()`), then runs from the lock; it works with `PUSHER_LOCK_ENABLED` false because it calls the pusher's lock move directly.

## Duo Primary (2026-11-06, the user's design; `AutonDuoPrimary`, Dashboard values `DuoAutos.P_*`)

A third auto for the alliance whose partner does the lighter job: the rear-wall start of the Garden Side with a longer program, **not yet run on the robot**. Blue frame (red is turned 180 deg):

1. Preload into the REAR cell from the start pose (tip 1).
2. The Blue Wall flower (the audience flower's poses turned 90 deg): ram, one-cycle jostle (deadline `P_WALL_JOSTLE_DEADLINE_SEC` 12 s), back out, ramp stows.
3. Around the outside via the east helper (36, 40) to the audience launch spot (50, 26.3), the solo auto's spot on the audience flower's line; launch into the AUDIENCE cell (the far cell: the far-cell aim extra applies).
4. The audience flower from there (the solo auto's poses), back out. Skipped if later than `P_AUD_FLOWER_LATEST_START_SEC`.
5. Back across the north side (east helper, west helper (-36, 40)) to the rear launch spot (-50, 26.3), the mirror of the audience one (31.6 in from the rear cell, 22 in from the garden pre-position); launch into the REAR cell (the user chose a mid-field spot for this salvo).
6. The garden as in the solo auto, back to the rear launch spot, launch into the REAR cell. Skipped if later than `P_GARDEN_LATEST_START_SEC`.
7. Park in the hive-side corner (26, 60) through the west helper, ramp stowed, flywheel off.

Speeds: `DuoKit.fastLegs` gives this auto the solo auto's faster legs (`P_LEG_MAX_POWER` 0.85, `P_CROSS_MAX_POWER` 1.0, `P_PASS_SETTLE_SEC` 0) and the Blue Wall flower's near-enough tolerances (`WALL_PRE_POS_TOL_IN` 2.5, `WALL_BACKOUT_TOL_IN` 2.5); the other two duo autos keep their old speeds.

**With the garden it did not fit 30 s** (the 01:57 run on 2026-11-06: preload done 3.7 s, wall flower salvo 12.2, audience flower salvo 22.7, garden salvo 28.1, park cut off at 30 s; the legs and the estimate below were right to within a second). Since then `P_GARDEN_LATEST_START_SEC` is 18 s, so the garden is skipped (logged `AUTO SKIP garden`) and the park should end at about 25 s; the user chose this to protect the park for now and will optimize the garden gather later. Earlier estimate: A rough estimate from the 2026-11-05 solo legs: preload done 3.3 s, wall flower and its salvo 16, audience flower 21, rear salvo 26.7, then the garden (gather, return, salvo) about 34 and the park about 37.5 s. The two latest-start values are 99 s (nothing cut) so the first runs time the whole program, as the user asked (they will optimize the garden gather and the paths from the `AUTO TIMING` lines); to protect the park set the garden's to about 14 s and the audience flower's to about 17 s (without the garden the park still ends at about 30 s). Waits: none, the program has no partner wait; the first tip (the rear cell, 4 s) is over long before the first launch into the audience cell.

Open points: the cell order is the user's (rear, audience, rear, rear) and is not checked against the tip logic here: the preload tips the rear cell, after which the audience cell faces up and needs 8, and the later rear-cell salvos go to a cell whose state depends on whether the hive has tipped again, so what each salvo scores is for the user to confirm; the rear and audience launch spots and the aim trims `P_*_TRIM_DEG` (0) are untuned; the Blue Wall flower poses are a rotation of the audience flower's numbers, not measured.

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
