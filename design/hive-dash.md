# Hive Dash (design, for review - nothing is built)

Driver feature in `TeleOp - Competition`: hold the driver D-pad left or right and the robot drives to the launch spot of a hive cell, target locks, and launches when aligned. Named by the user 2026-11-04.

## Behaviour

**Control.** Driver (gamepad 1) D-pad left / right, hold to run. The mapping follows the DRIVER'S view of the field (the same orientation as the field-oriented sticks):

| alliance | D-pad left | D-pad right |
|---|---|---|
| blue (driver looks toward -Y) | audience cell | rear cell |
| red (driver looks toward +Y) | rear cell | audience cell |

**Goals** (the Auto Launch zone centres, field frame): blue rear (-61.5, 15.25), blue audience (61.5, 7.5), red rear (-61.5, -7.5), red audience (61.5, -15.25). The robot's final heading is the target lock's aim heading at the goal.

**Sequence.**
1. D-pad pressed. Refused (no movement, no waypoints) when: backup mode, pose not seeded, the element counter is reading and holds nothing, or the robot's own footprint already overlaps a keep-out (pressed against a support). A refusal is a short rumble on the driver pad plus a telemetry line.
2. DRIVING: follow the planned route (below). Sticks are ignored while the D-pad is held.
3. Inside the lock radius of the goal (20 in, tunable): the target lock takes the heading, the speed eases, the goal tolerance is 2.5 in / 6 deg.
4. ARRIVED: stop, finish aligning. Launch when ALIGNED, stopped (0.15 s) and the flywheel is ready; skipped if the robot is empty on arrival (the counter reads 0).
5. The salvo always finishes, even if the D-pad was released. The dash ends after the salvo.

**Cancel.** Releasing the D-pad cancels at once and the sticks take over, also during a salvo (the salvo still completes). No stuck check and no timeout: the driver sees a stuck robot and lets go.

**Cell choice.** The D-pad's cell overrides the nearest-cell logic for the length of the dash (the launch map follows the distance to that cell); the normal logic resumes afterwards.

## Field geometry (CAD, field STEP v26-27.2, parts am-5878 and am-5876)

CAD axes: CAD X = our Y, CAD Z = our X (confirmed by the bars lying on the row borders 2|3 and 4|5). Everything is symmetric in sign, so the same four obstacles serve both alliances.

**Foot bars** (ground obstacles, 2.15 in tall, the chassis cannot cross): two of them, x -19.47 to +19.47, |y| 22.75 to 24.73.

**A-frame legs** (4, one per quadrant; 1 in square tube, 45.75 in long) lean inward both ways: foot centreline at height 0.53 in is (|x|, |y|) = (18.56, 23.81); at height 18 in it is (10.78, 19.04); at 41.08 in it is (0.51, 12.74). The robot is treated as 18 in tall (`ROBOT_HEIGHT_IN`, Dashboard), so for the chassis each leg is the thick line segment from its foot to its 18 in point, half thickness 0.57 in. At 18 in the tube outer faces are |x| 10.24 to 11.33 and |y| 18.47 to 19.61.

**The lane under the hive.** The narrowest point is at |x| = 10.78, where the inner faces are 36.9 in apart in y. A 13.70 in wide robot driving along x on y = 0 clears each leg by 11.6 in; even turned any way (the 12.0 in far-corner radius) it clears by 6.5 in. So the lane on y = 0 needs no heading constraint. (The supports lean inward, so the OUTSIDE of the hive is not narrowed by the lean.)

## Route planning (hand-placed points, no fancy logic)

**Points** (field frame, mirrored for all four signs): corner helpers (+-36, +-40), the user's tuned point; lane gates at the hive's entrance and exit (+-34, 0). The user will refine these later; they are Dashboard values.

**Footprint.** The real rectangle: 17.48 x 13.70 in, origin 7.60 in from the back and 9.88 in from the front. Each straight leg is the footprint swept along the leg with the robot ALIGNED to the leg (front or rear leading), tested against the keep-outs (2 bars + 4 leg segments) with a separating-axis check plus a margin (1.5 in, tunable). At a point where the heading changes the footprint is tested as a circle of 12.0 in (far corner) plus the margin.

**Algorithm.**
1. If the straight line from the robot to the goal is clear, go straight.
2. Otherwise the shortest chain through the points over legs that are clear (a small graph, at most 8 points plus start and goal): a leg's cost is its length plus a turn penalty (inches per degree, tunable).
3. The route is chosen once when the D-pad is pressed and kept for the dash.
4. `UNDER_HIVE_ROUTES` (Dashboard, off at first) controls whether a route between two outside points may use the lane (it removes the gate-to-gate link). A robot that STARTS under the hive always has a clear leg to a gate, so it can leave with no special case.
5. If the robot starts overlapping a keep-out, the dash is refused (see above).

Worked lengths: rear zone to audience zone about 149 in around the outside via (-36, 40) and (36, 40), about 128 in through the lane via (-34, 0) and (34, 0).

## Driving in line with the wheels

On each leg the heading is the travel direction, with the front or the rear leading, whichever needs less rotation (and, on the last leg, whichever leaves the smaller turn for the lock: the launch zones are straight out from their cells, so arriving rear-first means the front already faces the cell). The sides of a mecanum chassis strafe slower than forward, and aligned is the narrowest profile for the keep-outs. Short legs (under about 15 in) are not aligned.

**Leg follower** (new, small): field velocity along the leg at full power for the leg, a correction for sideways error, eases off near the end; the translation is converted to the robot frame with the current heading, so the robot moves along the leg while it is still turning, with the speed scaled by how well aligned it is (floor 40 percent). `DriveToPose`'s gains are kept for the final settle only (its per-axis clamp made a 45 degree first move on diagonals).

**Strafe speed.** No FTC figure found: quoted ideals are about 70 percent of forward, practical figures 87 percent and lower (anecdotal). To be measured on ROBOT_1: the first runs log speed against travel angle relative to heading.

## Code

- `drive/HiveRoute.java`: pure geometry, no hardware; `plan(start, goal, underHive)` returns the waypoints and headings. Testable on a PC.
- `drive/FieldGeometry.java`: the bar and leg constants.
- `drive/HiveDash.java`: the state machine and the leg follower; returns translation, whether to lock, and a launch request. `TeleOpCompetition` applies it.
- Pose and sensors are read once at the top of the loop and buffered; nothing in the dash reads the Pinpoint again.
- Logging: `HIVE DASH start (route, via ...)`, `waypoint reached`, `arrived`, `aligned`, `launch`, `cancelled at (x, y)`, `refused: <why>`, `finished`; plus speed against travel angle for the strafe measurement.
- Dry run: the Dashboard field overlay draws the keep-outs, the route and the goal in every mode; `HIVE_DASH_DRY_RUN` plans and draws without driving.
- Docs: the controls page, `robot-day-runbook.md`, `run-logs.md`.

## Tunables (Dashboard)

Max power (0.5 at first), margin 1.5 in, `ROBOT_HEIGHT_IN` 18, lock radius 20 in, goal tolerance 2.5 in / 6 deg, pass-through tolerance 3 in / 10 deg, turn penalty, align threshold 15 in, `UNDER_HIVE_ROUTES` off, the helper and gate points.

## Test plan

1. PC test of the router: direct line, one point, two points, both alliances, start under the hive (origin), start against a bar (refused), every goal from every corner.
2. Dry run on the robot, watching the overlay.
3. Low-power runs with a spotter: crossing between the two zones, from the garden corners, both alliances; then the lane.
4. Read the logs.

## Open items

- The robot's real maximum height (the legs are evaluated at 18 in).
- Strafe against forward speed on ROBOT_1 (measured in the first runs).
- Whether anything else hangs under the hive besides the four legs (the CAD gave only the frame and legs).
- Refusal signal (rumble + telemetry) and the 20 in lock radius are my defaults, not yet confirmed.
