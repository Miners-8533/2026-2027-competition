# PC tools

Programs in `tools/` that run on a laptop, not on the robot. This page covers them only as far as the robot code depends on them: what each produces, what it consumes, and which robot data it generates or guards. Tools that only analyze are listed briefly.

All are run from the repository root.

## 1. Tools that generate robot source

Two files in the robot code are **generated**. They must never be edited by hand; change the inputs and regenerate.

| Generated file | Generator | Inputs | Regenerate when |
|---|---|---|---|
| Grid route tables (the four route trees and the corner speed table) | the dash simulator's table command | the drive model, the field obstacles, the four goals, the lattice shape | the drive model changes, a launch spot or park corner moves, or an obstacle changes |
| Launch grid data | the launch grid table maker | the tuning runs converted to CSV, and the list of spots the tuner visits | a new tuning run is recorded |

Both files record what they were built with, and the robot code checks the route tables against its own lattice shape and goals before using them.

A third generated file, the **build stamp**, is produced by the build itself ([logging.md](logging.md)).

## 2. Tools that check robot logic on a PC

The route planner, the dashes, the leg and grid followers, the target lock math, the entry counting rule and the launch grid lookup are written with **no robot dependencies** so that they compile and run on a PC. Stand-ins replace the few SDK and Dashboard classes they touch (a clock, the log, the tunable-value annotation, the field overlay). This is a design constraint on those classes: adding a hardware dependency to them breaks every check below.

| Check | Verifies |
|---|---|
| Route check | the planner over a grid of starts to every goal: no unsafe route, how many starts have no route, the lane is kept centered; regression cases for past failures |
| Duo path check | every leg of the duo autos clears the hive supports and the flower stands, with the clearance of each. It holds copies of the default poses and of the robot's footprint, which must be changed together with the autos. Target T7 ([field.md](field.md) section 7) removes the copies |
| Launch grid check | each tuning grid is non-empty, every spot obeys the rules, and every hop of the sweep is drivable |
| Grid map check | every measured spot returns itself, the zones, the red rotation, the seed flag, continuity |
| Entry counter check | recorded proximity runs replayed through the counting rule the robot uses, against known answers and a reference implementation; then the recorded pollen run with its color, and made-up nectar, through the rule that names each entry's kind |

## 3. Dash simulator

**Purpose.** Run the real dash code against a model of the robot, to judge routes and speeds without risking the robot.

- **The loop** reproduces the TeleOp's order for a dash: Park Dash, Hive Dash, the translation, the lock's turn when the dash wants the lock, a salvo of fixed length on a launch request, hold position.
- **The plant** is a per-wheel model fitted from the Drive Characterization log: a speed curve per power, separate limits for speeding up and slowing down, braking at zero power, coasting on part power, per-axis dead zones, a strafe ratio, and battery sag. An earlier, simpler plant fitted from TeleOp logs is kept for comparison.
- **The collision model** tests the robot's real footprint against the supports, flower stands and walls every 10 ms.
- **Replay** runs every dash recorded in the TeleOp logs from its logged start, with that run's own settings, and compares the simulated track with the logged one. About 70 percent of logged dashes are reproduced within the acceptance limits; the stated target is all of them, so the simulator ranks routes and speeds but does not prove small clearances.
- **Surveys** drive the dashes from a grid of starts to every goal and report times and contacts. **Calibration** finds the corner speeds. **Table generation** writes the route tables.
- **The two gates of target T9** ([dashes.md](dashes.md) section 6.1), built 2026-11-17:
  - **Hand trips** reads an event's labeled TeleOp logs and lists every trip the drivers made to a launch: where it started (pose and velocity), which cell was launched at, where the salvo was fired from, and how long the drive took. A trip in which a Hive Dash ran is marked and left out of the baseline.
  - **The speed gate** drives the real Hive Dash from the start of each hand-driven trip of 12 in or more to the launch spot of the same cell, from rest or with the logged velocity, and sets its time beside the drivers', trip by trip. It passes when the dash is at least as fast on the median trip and touches nothing on the way. Something the logged start pose already overlaps (the robot was against a flower stand) is listed apart and does not count as a contact of the dash.
  - **The safety gate** repeats the survey nine times: once as is, and eight times with the pose the robot believes shifted from the truth by the accepted drift, one direction each. The dash plans and drives on the believed pose; contacts are judged where the robot really is. It passes when nothing is touched in any of the nine.
  - A trial option builds the four trees in memory with another drift margin and runs either gate on them, without writing the robot's tables.
  - Both gates run on today's planner or on the grid routes. The Park Dash is in the safety gate only: the logs hold no baseline for a drive to a park corner.

Other tools in the same folder fit the drive model from logs and report a Drive Characterization run segment by segment.

## 4. Log tools

| Tool | Does |
|---|---|
| Pull logs | copies every run off the hub over USB, files and indexes them, removes them from the hub |
| Event day | labels runs with match numbers; writes a report per match and a day summary |
| Loop profile | tables of the loop profiler lines |
| Config changes | which settings changed between runs; the history of one setting |
| Check config list | lists tunable classes missing from the settings dump. Not needed once target T8 ([logging.md](logging.md) section 2.1) is done |
| Baseline health | per color sensor, whether its stored empty reading still fits the logged empty-track readings |
| Sensor study | prints the Color Sensor Study's steps and the baseline lines to paste |
| Launch stability | flywheel speed at the start of every salvo; the source of the launch band |
| Launcher offset | the launcher's sideways offset from marked launches |
| CS5 counting | replays entry counting on a logged run |
| Cycle times | splits each TeleOp cycle into salvo, collect, transit and lock; pollen per minute |

The log's event prefixes and column names are these tools' input format ([logging.md](logging.md)); renaming one breaks the tools that read it.

## 5. Launch grid tools

Convert a tuning run's log to CSV; build the generated table from all CSV files (a later run replaces earlier rows for the same spot, unmeasured spots near the walls are estimated from neighbours, and a leave-one-out check reports how well each spot is predicted from the others); list the tuner's spots; plot the map.

## 6. Video tools

Match video analysis: align the robot log, the broadcast video and the official score on one match clock; read the on-screen score; detect hive tips and pollen landings; export a cycle dashboard; fetch official results from the FIRST events service. These consume logs and produce reports. Nothing on the robot depends on them.
