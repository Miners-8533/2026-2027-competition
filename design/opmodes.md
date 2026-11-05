# Opmodes

All opmodes are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/`. The Driver Station lists only the **enabled** ones. A **disabled** opmode keeps its code with a `@Disabled` line; to use it again, delete that line and its import, then reinstall.

## Enabled

| Opmode (Driver Station name) | Group | Use |
|---|---|---|
| TeleOp - Competition | Competition | the match TeleOp (controls: the controls document) |
| Auton - Solo | Auto | the solo auto (partner just parks), BLUE or RED picked at INIT with gamepad 1 (D-pad right / left): launch the preload, garden, launch (from a spot on the flower's line), flower, launch (same spot), park; poses from the waypoint marks of 2026-11-02, **not yet run on the robot** |
| Auton - Duo Flower Side | Auto | the audience-wall half of the matched pair (blue/red at INIT): waits until the partner's preload has tipped the hive, launches the preload, gathers the audience-wall flower, launches it (8 pollen tip the audience cell), parks; design/duo-autos.md, **not yet run on the robot** |
| Auton - Duo Garden Side | Auto | the rear-wall half of the pair: launches the preload (tips the rear cell), gathers the Blue Wall flower, waits for the partner's tip, launches, takes the garden, launches again, parks; design/duo-autos.md, **not yet run on the robot** |
| Auton - Blue Students | Auto | old auto, superseded by Auton - Solo (kept for reference) |
| Auton - Hive Volley (Blue) | Auto | old auto, superseded by Auton - Solo (kept for reference) |
| Pit Check (robot on blocks) | Bring-up | the one check opmode: configuration, ports and stray devices, battery, Pinpoint, color sensors, then flywheel, pusher, intake, ramp, each wheel ([pit-check.md](pit-check.md)) |
| Tune - Axial (fwd/back 48in), Tune - Lateral (strafe 48in), Tune - Heading (turn pattern) | Tuning | the `DriveToPose` PID tuning (still to do) |
| Auton Verify - Forward 48 and back, Auton Verify - Lateral 48 and back | Auto verify | check the tuned gains |
| Launcher Offset Finder | Bring-up | no target lock: drive by hand, line up on a cell's centerline, launch, MARK good salvos; finds the launcher offset and any hive error ([launcher-offset-finder.md](launcher-offset-finder.md)); not yet run on the robot |
| Color Sensor Study | Bring-up | records CS1-CS4 raw readings against known pollen counts: baselines, an automatic slow pusher sweep, and jostle takes ([color-sensor-study.md](color-sensor-study.md)); not yet run on the robot |
| Auto Waypoint Marker | Auto build | for composing autos: pick the blue (rear wall, Y +16) or red (audience wall, Y -16) start at INIT, drive to points, **A** marks the pose (a `MARK n: x, y, heading, ramp \| new Pose2d(...)` line in the log and on screen), **B** logs a numbered `CHECK` (a spot whose real position you know, for checking the pose), **X** / **Y** deploy / stow the ramp, hold **LB + RB** reseeds to the start. Log: `waypoints.log`; no undo |

## Disabled (code kept)

| Opmode | Use when |
|---|---|
| Launch Map Tuner | extending the launch map (the far and near ends, nectar) |
| Pinpoint Offset Check | a pod or the Pinpoint was moved: re-find the offsets (last run 2026-11-03: 2.1 to 2.2 in of wobble with the current offsets, the check suggests about -158.7 / +22.8; the offsets were left as they are) |
| Pinpoint Push Test | a pod or the Pinpoint was moved or rewired: check the directions by hand |
| Color Sensors Test (CS1-CS4) | the lighting changed: re-measure the element counter's empty baseline |

## Removed (in git history)

Hardware Check and Chassis Info (the Pit Check does their work), Launch Test, Pusher Check Test and Pusher Sensor Matrix (2026-11-02), and the eight earlier bring-up opmodes (2026-10-31).
