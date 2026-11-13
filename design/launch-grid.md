# Launch grid (2026-11-10)

The one-dimensional launch map (flywheel speed from distance only, `LaunchMap.TABLE`) is being replaced by a **two-dimensional grid** of aim and speed per launching spot. The **Launch Grid Tuner** (`opmodes/bringup/LaunchGridTuner.java`, replaced the Launch Map Tuner) collects the data; using the grid in the TeleOp is a later step. **Not yet run on the robot.**

![The 12 in grid for the near and far cells](launch-grid-draft-12in.png)

(The picture is the design drawing, made from the same CAD keep-outs; `tools/hive/LaunchGridCheck.java` prints the real spots.)

## Which spots (`drive/LaunchGrid.java`)
One grid per hive cell: the **NEAR** cell (the cell on the start side) and the **FAR** cell (the cell opposite the start wall). Red or blue does not matter for the grids (red is blue turned 180 degrees); since 2026-11-12 the tuner ASSUMES BLUE: the usual blue start is against the rear wall facing the audience wall (-63.4, 16.5, 0 deg, `START_*` in the Dashboard), the NEAR cell is the blue rear cell and the FAR cell the blue audience cell; the tuner used the red hive. A spot is kept only when

- it is on the cell's **open side** (the audience cell opens toward the audience wall, the rear cell toward the rear wall: nothing behind or beside the opening can score);
- it is at least `TargetLock.MIN_DISTANCE_IN` (12 in) from the cell;
- the shot is within `ANGLE_LIMIT_DEG` (75, a first guess) of the opening's normal;
- the robot **at its aim heading there** (the launcher pointed at the cell, `TargetLock.aimHeadingRad` with the launcher offset and the far-cell extra) does not overlap a foot bar, an A-frame leg or a flower stand (`HiveRoute`'s CAD keep-outs, the robot's rectangle grown by the planner's 1.5 in margin) and fits inside the walls less `WALL_MARGIN_IN` (3 in since 2026-11-12: at the rear-wall column the robot, turned to face the cell, ended 3 to 14 in short of four spots, its back corner about 1 in from the wall).

There is **no maximum distance**. A spot that fails only on a wall (turned to face the cell the corner would cross it) is slid toward the field center until it fits (at most 8 in). The lattice is centered on the field with `PITCH_IN` (12) spacing, so 30 spots for the near cell and 29 for the far. The spots are visited as a serpentine sweep, the column nearest the start first.

## How the tuner works
The robot drives to a spot (HOLD the right trigger; the route is `HiveRoute`'s, and when the router refuses a spot beside a flower stand it drives a short straight hop if that is clear, else it says BLOCKED and you drive by hand), already pointed at the cell with today's aim, and the flywheel at the launch map's speed for that distance. The operator loads, fires (Y), and trims the **angle** (D-pad left/right, 0.5 deg, the robot turns by itself) and the **speed** (D-pad up/down, 20 ticks/s) until the salvo scores. **A** takes the trims as good and goes to the next spot, **B** says no shot is possible from here, **X** goes back. Controls and the log lines are in the opmode's header and `robot-day-runbook.md` (Block F).

## What it logs
One `GRID POINT` line per spot (the speed and its split into the launch map's speed and the trim, the aim heading and its split, the distance, how many salvos were fired, the pose; a good spot with no salvo is marked UNVERIFIED), a `FIRE` line for every salvo, `TRIM`, `DRIVE`, `ARRIVED` lines, and at the end one `GRID TABLE` line to paste. `START_INDEX` in the Dashboard continues a sweep at a later spot.

## Checks
`tools\hive\run-check.cmd` compiles and runs `LaunchGridCheck`: each grid is non-empty, every spot obeys the rules, and every hop of the sweep (from the start pose through all spots) is routable by the router or by a clear straight hop (3 hops beside flower stands are the straight kind).
