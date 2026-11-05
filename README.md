# Team 8533 CSPA Miners: BIOBUZZ (2026-27) robot code

The robot code for **FIRST Tech Challenge team 8533, the CSPA Miners**, for the 2026-27 game BIOBUZZ. It is built on the FTC SDK 12.0 (the original SDK readme is kept in [README-FTC-SDK.md](README-FTC-SDK.md)); our code is in `TeamCode/`.

**The robot (ROBOT_1):** a mecanum drive with a goBILDA Pinpoint odometry computer; an intake, a flywheel launcher, a rotating **pusher** arm around a curved track that holds up to 4 pollen and feeds the flywheel, a deployable **ramp** (the depollinator for gathering from a flower), and four REV color sensors (CS1 to CS4) along the track that count and identify the elements.

## What it does
- **TeleOp - Competition:** field-oriented drive with a PD target lock on a hive cell, **Auto Launch** (launch by itself in a launch zone), **Hive Dash** (hold D-pad left or right and the robot drives to a cell's launch spot around the hive supports, locks and launches), **hold position while launching**, the **pusher lock** (the arm parks across the intake entrance when the robot is full), a launch map for the flywheel speed by distance, and a backup mode for a lost pose.
- **Auton - Solo:** launch the preload, take the garden, launch, take the flower, launch, park; blue or red picked at INIT. The two **duo** autos (a flower side and a garden side that tip the hive together with the alliance partner) are being built on the `duo-autos` branch.
- Every run writes a log to the hub; tools in `tools/logs/` pull the logs and analyse them, including a competition-day report per match.

## Layout
| Folder | What is in it |
|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | `opmodes/` (the opmodes), `subsystems/` (intake, flywheel, pusher, ramp, element counter, and `Mechanisms`, the coordinator), `drive/` (mecanum drive, Pinpoint localizer, target lock, Hive Dash, hold position, hive routing), `motion/` (drive-to-pose, the auto actions), `util/` (the run log, the Dashboard config dump), `math/` |
| `design/` | the design notes, test plan and runbooks (start with [integration-plan.md](design/integration-plan.md) and [robot-day-runbook.md](design/robot-day-runbook.md)) |
| `tools/` | `logs/` (pull and analyse the run logs), `hive/` (router checks on a PC) |
| `FtcRobotController/` | the FTC SDK app (unchanged) |

## Build and run
Open the project in Android Studio (Narwhal 3 Feature Drop or later) and build and deploy to the Control Hub. The Dashboard for live tuning is at `192.168.43.1:8080/dash` over the hub's Wi-Fi. A robot-day checklist is in [robot-day-runbook.md](design/robot-day-runbook.md), and the Pit Check opmode checks the hardware.

## Branches and tags
- **`main`** is the last build that has been **run on the robot**. Each tested build is tagged `tested-YYYY-MM-DD`.
- **`integration`** is the working branch: all new work lands here first, and it can contain changes that have not run on the robot yet (the test plan rows in [integration-plan.md](design/integration-plan.md) say which).
- Risky or large changes get their own branch (for example `duo-autos`) and are merged into `integration` once they work; `main` is updated only from tested builds.

## Conventions
- **Field frame:** origin at the field center, +X toward the audience wall, +Y toward the Blue Wall, inches, heading 0 along +X and counterclockwise positive; the walls are at +-71 in. Red is blue turned 180 degrees about the center.
- **Words:** the **pusher** (the transport arm), the **ramp** (the deployable depollinator), the **feed ramp** (the end stop of the track), and **launch** / **launcher** / **launch map** instead of shoot, shot, fire.
- **Controls:** never Start or Back on either controller; the driver has no stick-button controls; risky driver actions need the left bumper. The controls are documented in the TeleOp's header comment.

## Logs and analysis
```
python tools/logs/pull_logs.py                    # pull the run logs from the hub (robot-logs/, not in git)
python tools/logs/event_day.py label  --event "<event>"   # label the runs with their matches
python tools/logs/event_day.py report --event "<event>"   # per-match and day reports, what stands out
python tools/logs/launch_stability.py             # flywheel speed during salvos
python tools/logs/baseline_health.py              # post-match health check of the color sensors' baselines
python tools/logs/loop_profile.py                 # where the loop time goes (the LOOP PROFILE lines of the newest TeleOp and auto)
python tools/logs/config_changes.py               # which Dashboard values changed between runs
```
What every log line means is in [run-logs.md](design/run-logs.md).

## Design notes
[mechanisms.md](design/mechanisms.md) (the intake, flywheel, pusher, ramp and the coordinator), [hive-dash.md](design/hive-dash.md), [game-rules.md](design/game-rules.md) (the scoring and rules that matter to the autos), [opmodes.md](design/opmodes.md), [idea-queue.md](design/idea-queue.md).

## License
The FTC SDK is under the license in [LICENSE](LICENSE) (FIRST's BSD-style license).
