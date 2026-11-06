# Park Dash (BUILT 2026-11-05, not yet run on the robot)

A TeleOp driver feature: **hold D-pad up or down** and the robot drives to one of the two park corners of its own loading zone and stays there while the D-pad is held. PARK is 5 points at the end of TELEOP and only needs the robot partly in the zone ([game-rules.md](game-rules.md)), so this is the end-of-match "get in now" button.

## Controls
- **Up** = the corner on the driver's LEFT, **down** = the corner on the driver's RIGHT (the same left / right the Hive Dash uses, from `driverForwardRad`). With the default directions, left is the audience-side corner for blue (x 40, y 60) and its 180 degree rotation for red (-40, -60); right is the hive-side corner (26, 60) / (-26, -60). These are the duo autos' park poses; `ParkDash.LEFT_X`, `RIGHT_X`, `PARK_Y` in the Dashboard.
- Hold-to-run, like the Hive Dash: releasing cancels at once and gives the sticks back. Pressing the other direction cancels it too (release first to start the other).
- It stows the ramp when it starts (the dash does not wait for it).
- Once at the corner (within `GOAL_TOL_IN`) it stays there: no drive power, and `HoldPosition` holds the pose (it resists a push) for as long as the D-pad is held.
- **Refused** (short rumble, no movement) in backup mode, with a Pinpoint fault, with a pose that is not seeded, while the Hive Dash is running, and when the robot is touching a hive support. The Hive Dash does not start while the Park Dash is running.

## How it drives
`drive/ParkDash.java` plans with `HiveRoute` (the same supports, helper corners, lane gates and speed ratio as the Hive Dash, read from the `HiveDash` Dashboard values) and follows the legs the same way: long legs aligned with the wheels, front or rear leading (whichever is nearer the heading the robot has), short legs with the heading kept. There is no cell, so no lock and no launch; the robot ends with the leg's heading (any heading counts, the corner y 60 keeps either end of the robot inside the wall). Power is `MAX_POWER` 0.5 to start (raise it after the first runs), slowing on the last leg like the Hive Dash (which drives straight at the corner, so an overshoot is driven back). `DRY_RUN` plans and logs without driving.

## Logs
`PARK DASH start (<corner>)` with the route, `PARK DASH leg <i> of <n>`, `via point ... reached`, `sample` every 0.5 s (pose, speed; power 0.85 from 2026-11-05, `TURN_MAX` stays 0.5), `parked after <s> s`, `released` / `cancelled (<why>)`, `refused (<corner>): <why>`.

## Checks
`tools/hive/HiveRouteCheck.java` (run `tools/hive/run-check.cmd`) plans from a 6 in grid of free starts to all four corners: 1852 trips, none unsafe, 16 without a route (starts jammed against the supports, which are refused; 24 before the 2026-11-05 planner change below; 32 after the lane centering of the same night, see hive-dash.md).

**Planner change 2026-11-05** (`HiveRoute.legClear`): the first Park Dash on the robot (log 20:05, from (25.0, 37.5), 22 in from the right corner) drove 126 in via the helper point (-36, 40) and took 3.85 s, because a short leg's box (a square of half-width radius + margin) reached back over the start and caught the end of a support bar 14 in away. A short leg from the start is now boxed from the start point onward, with the robot's own circle (radius, no margin) checked at the start. Effect on the 1,850-trip grid at 3 in spacing: trips that detour although the straight leg is safe fell from 88 to 4 for goals within 40 in (567 to 515 overall; the rest are long aligned legs and starts within a robot radius of a bar), and the check has a regression test for that exact start.

## To test (T11 in integration-plan.md)
`ParkDash.DRY_RUN` first (hold up / down, read the `PARK DASH start` line), then low power with a spotter: from the field center, from the launch zones and from under the hive, both corners, both alliances; cancel on release; the hold at the corner (push it); the refusal paths; D-pad left / right still starts the Hive Dash and not both together.
