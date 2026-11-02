# Opmodes

All opmodes are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/`. The Driver Station lists only the **enabled** ones. A **disabled** opmode keeps its code with a `@Disabled` line; to use it again, delete that line and its import, then reinstall.

## Enabled

| Opmode (Driver Station name) | Group | Use |
|---|---|---|
| TeleOp - Competition | Competition | the match TeleOp (controls: the controls document) |
| Auton - Blue Students | Auto | auto |
| Auton - Hive Volley (Blue) | Auto | auto |
| Pit Check (robot on blocks) | Bring-up | the one check opmode: configuration, ports and stray devices, battery, Pinpoint, color sensors, then flywheel, pusher, intake, ramp, each wheel ([pit-check.md](pit-check.md)) |
| Tune - Axial (fwd/back 48in), Tune - Lateral (strafe 48in), Tune - Heading (turn pattern) | Tuning | the `DriveToPose` PID tuning (still to do) |
| Auton Verify - Forward 48 and back, Auton Verify - Lateral 48 and back | Auto verify | check the tuned gains |

## Disabled (code kept)

| Opmode | Use when |
|---|---|
| Shot Map Tuner | extending the shot map (the far and near ends, nectar) |
| Pinpoint Offset Check | a pod or the Pinpoint was moved: re-find the offsets |
| Pinpoint Push Test | a pod or the Pinpoint was moved or rewired: check the directions by hand |
| Color Sensors Test (CS1-CS4) | the lighting changed: re-measure the element counter's empty baseline |

## Removed (in git history)

Hardware Check and Chassis Info (the Pit Check does their work), Shot Test, Pusher Check Test and Pusher Sensor Matrix (2026-11-02), and the eight earlier bring-up opmodes (2026-10-31).
