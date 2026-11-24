# Team 8533 CSPA Miners: BIOBUZZ (2026-27) robot code

The robot code for **FIRST Tech Challenge team 8533, the CSPA Miners**, for the 2026-27 game BIOBUZZ. It is built on the FTC SDK 12.0 (the original SDK readme is kept in [README-FTC-SDK.md](README-FTC-SDK.md)); our code is in `TeamCode/`.

**The robot (ROBOT_1):** a mecanum drive with a goBILDA Pinpoint odometry computer; an intake, a flywheel launcher with a position servo, a rotating **pusher** arm around a curved track that holds up to 4 pollen and feeds the flywheel, and a deployable **ramp** (the depollinator for gathering from a flower). Five REV color sensors: CS1, CS3 and CS4 along the track tell what sits where and see the pusher arm; CS2 at the top of the feed ramp and CS5 at the intake entrance are fitted but switched off in the code at present.

## What it does
- **TeleOp - Competition:**
  - field-oriented drive with a PD **target lock** on a hive cell;
  - the flywheel speed and the aim from a measured **2-D launch grid** (the launch map by distance where the grid has no data), and no launch from a spot the grid measured as no shot;
  - **hold position** and a drive hold while a salvo is launched;
  - the operator's reports of where a salvo landed, which trim the aim per cell and the speed;
  - **Auto Launch**, an operator toggle (launch by itself in a launch zone);
  - **Hive Dash** and **Park Dash** (hold a D-pad direction and the robot drives to a cell's launch spot or a park corner on precomputed routes): built, not used in a match so far;
  - a backup mode for a lost pose.
- **Seven autos**, blue or red picked at INIT: **Solo** (launch the preload, take the garden, launch, take a flower, launch, take the other flower, launch, park), **Boomerang** and **Triple Tip** (Solo with the later salvos into the first cell), and four for an alliance whose partner also launches: **Duo Daryl**, **Duo Flower Side**, **Duo Garden Side** and **Blaser's Gambit**. Solo and Boomerang have run in matches; what each does and whether it has run is in [specs/autos.md](specs/autos.md).
- **Support opmodes:** Pit Check (the hardware check, robot on blocks) and the Ramp Test are shown on the Driver Station; the tuners and studies are kept hidden ([specs/support-opmodes.md](specs/support-opmodes.md)).
- Every run writes a log to the hub; tools in `tools/logs/` pull the logs and analyse them, including a competition-day report per match.

## Layout
| Folder | What is in it |
|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | `opmodes/` (the opmodes), `subsystems/` (intake, flywheel, pusher, ramp, element counter, and `Mechanisms`, the coordinator), `drive/` (mecanum drive, Pinpoint localizer, target lock, Hive Dash, hold position, hive routing), `motion/` (drive-to-pose, the auto actions), `util/` (the run log, the Dashboard config dump), `math/` |
| `specs/` | what the software does and how it behaves, as built, with the status of each feature (start with [specs/README.md](specs/README.md)) |
| `robot_design/` | the physical robot: devices, wiring, dimensions, measured constants, and the hub setup checklist ([robot_design/README.md](robot_design/README.md)) |
| `tools/` | PC tools ([specs/tools.md](specs/tools.md)): `logs/` (pull and analyse the run logs), `hive/` (route, auto path and launch grid checks), `dash/` (the dash simulator, the route table generator, the drive model), `launchgrid/` (builds the launch grid data from tuning runs), `entry/` (replays the entry counting rule), `video/` (match video analysis and the official scores) |
| `FtcRobotController/` | the FTC SDK app (unchanged) |

## Build and run
Open the project in Android Studio (Narwhal 3 Feature Drop or later) and build and deploy to the Control Hub. The Dashboard for live tuning is at `192.168.43.1:8080/dash` over the hub's Wi-Fi. The hub setup checklist is in [robot_design/hub-configuration.md](robot_design/hub-configuration.md), and the Pit Check opmode checks the hardware ([specs/support-opmodes.md](specs/support-opmodes.md)).

## Branches
- **`main`** is the last build that has been **run on the robot**. Builds are not tagged: every run log names its build in its second line, the build stamp (commit id, build time by the PC's clock and, for a build with uncommitted changes, a fingerprint of them; [specs/logging.md](specs/logging.md) section 4), and `main` is brought up to the build whose stamp is in the robot's latest logs. At present that is the build of the Novi Qualifier of 2026-11-21: commit 9599d8b with the changes of 9fcb12d (fingerprint 82e29283, built 2026-11-21 16:24:44).
- **`integration`** is the working branch: all new work lands here first, and it can contain changes that have not run on the robot yet (the feature status table in [specs/architecture.md](specs/architecture.md) says which).
- Risky or large changes get their own branch (for example `flywheel-servo-prototype`) and are merged into `integration` once they work; `main` is updated only from tested builds.

## Conventions
- **Field frame:** origin at the field center, +X toward the audience wall, +Y toward the Blue Wall, inches, heading 0 along +X and counterclockwise positive; the walls are at +-71 in. Red is blue turned 180 degrees about the center.
- **Words:** the **pusher** (the transport arm), the **ramp** (the deployable depollinator), the **feed ramp** (the end stop of the track), and **launch** / **launcher** / **launch map** instead of shoot, shot, fire.
- **Controls:** the match opmodes never use Start or Back (the Driver Station uses Start to assign the gamepads); Back appears only in support opmodes such as Pit Check, where it stops everything. The driver has no stick-button controls, and risky driver actions need the left bumper. The controls are in [specs/teleop.md](specs/teleop.md) section 2 and in the TeleOp's header comment.

## Logs and analysis
```
python tools/logs/pull_logs.py                    # pull the run logs from the hub (robot-logs/, not in git)
python tools/logs/event_day.py label  --event "<event>"   # label the runs with their matches
python tools/logs/event_day.py report --event "<event>"   # per-match and day reports, what stands out
python tools/logs/cycle_times.py --event "<event>"       # each TeleOp cycle split into salvo, collect, transit and lock
python tools/logs/launch_stability.py             # flywheel speed during salvos
python tools/logs/baseline_health.py              # post-match health check of the color sensors' baselines
python tools/logs/loop_profile.py                 # where the loop time goes (the LOOP PROFILE lines of the newest TeleOp and auto)
python tools/logs/config_changes.py               # which Dashboard values changed between runs
```
What every log line means, and how to get the logs, is in [specs/logging.md](specs/logging.md).

## Specifications
[specs/](specs/README.md) describes the software as built (the mechanisms and their coordinator, aiming, the dashes, the TeleOp, the autos, the support opmodes, logging) and [robot_design/](robot_design/README.md) the robot itself. Ideas agreed to keep but not built are in [specs/idea-queue.md](specs/idea-queue.md). The `design/` folder of earlier notes was removed on 2026-11-23; it is in the git history.

## License
The FTC SDK is under the license in [LICENSE](LICENSE) (FIRST's BSD-style license).
