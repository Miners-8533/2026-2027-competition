@echo off
rem The dash simulation (design/dash-planner-grid.md): compiles the REAL HiveDash, ParkDash, HoldPosition, TargetLock and HiveRoute with the PC stubs in tools\dash\stubs,
rem then runs tools\dash\DashSim.java. Needs a JDK on the PATH. Run from the repo root. Arguments go to DashSim (default: replay every logged dash and print the gate), e.g.
rem   tools\dash\run-sim.cmd                      replay with the tuned plant
rem   tools\dash\run-sim.cmd replay --plant fitted
rem   tools\dash\run-sim.cmd plant dash           the drive model alone on the logged commands (must match identify_drive.py)
rem   tools\dash\run-sim.cmd run -65 50 0 BLUE_AUDIENCE
rem   tools\dash\run-sim.cmd drivetest           the Drive Characterization routine against the plant: its time, geofence stops and clearances
rem   tools\dash\run-sim.cmd survey 12           today's planner from a start every 12 in at 4 headings to each blue goal
rem   tools\dash\run-sim.cmd gridsurvey 12       the grid planner from the same starts (calibrates the corner speeds and builds the trees first)
rem   tools\dash\run-sim.cmd gridrun 42 -18 90 BLUE_REAR   one grid dash: its route and a trace
rem   tools\dash\run-sim.cmd gentables          calibrate, build the four trees and write drive\GridTables.java, then the smooth plan tables and the clearance map for them (drive\SmoothTables.java)
rem   tools\dash\run-sim.cmd gensmooth          drive\SmoothTables.java alone, for the trees as they are
rem   tools\dash\run-sim.cmd checktables [full] are the generated tables the ones a generation would write now? exit code 1 when one is stale (specs\dashes.md, When to regenerate the dash tables)
rem   tools\dash\run-sim.cmd survey 12 --grid   the REAL HiveDash / ParkDash with GRID_ROUTES on and the generated tables (also: run ... --grid, replay --grid)
rem The two gates of specs\dashes.md section 6.1 (target T9); add --grid for the grid routes, --plant wheel for the fitted wheel model:
rem   python -I tools\dash\hand_trips.py --event "Hartland Qualifier 2026"     the drivers' trips to a launch from the labeled logs -> build\dashsim\hand_trips.csv
rem   tools\dash\run-sim.cmd trips build\dashsim\hand_trips.csv --plant wheel [--moving]   SPEED GATE: the real Hive Dash from each hand trip's start, beside the drivers' time
rem   tools\dash\run-sim.cmd driftsurvey 12 4 --plant wheel                    SAFETY GATE: the survey with the believed pose out by 4 in, eight directions and none
rem   ... --smooth                        either gate with the grid routes driven as smooth paths from the plan tables (GridDash.SMOOTH, the 2026-11-17 prototype; implies --grid)
rem   ... --smooth-search                 the same with each path found by the full search on the spot (what the tables are generated with)
rem   ... --grid --route-drift 2          either gate on trees built in memory with another drift margin (GridRoutes.DRIFT_IN), without writing the robot's tables
if not exist build\dashsim mkdir build\dashsim
set T=TeamCode\src\main\java\org\firstinspires\ftc\teamcode
set S=tools\dash\stubs
javac -d build\dashsim %S%\com\qualcomm\robotcore\util\ElapsedTime.java %S%\org\firstinspires\ftc\teamcode\util\RunLog.java %S%\com\acmerobotics\dashboard\FtcDashboard.java %S%\com\acmerobotics\dashboard\canvas\Canvas.java %S%\com\acmerobotics\dashboard\config\Config.java %S%\com\acmerobotics\dashboard\telemetry\TelemetryPacket.java %T%\math\Pose2d.java %T%\math\PoseVelocity2d.java %T%\math\Rotation2d.java %T%\math\Vector2d.java %T%\subsystems\HiveTarget.java %T%\motion\PidController.java %T%\drive\HiveRoute.java %T%\drive\LegFollower.java %T%\drive\HiveDash.java %T%\drive\ParkDash.java %T%\drive\HoldPosition.java %T%\drive\TargetLock.java %T%\drive\AutoLaunch.java %T%\drive\DriveTestRoutine.java %T%\drive\GridTables.java %T%\drive\GridRoutes.java %T%\drive\GridFollower.java %T%\drive\SmoothTables.java %T%\drive\SmoothFollower.java %T%\drive\GridDash.java tools\dash\RouteGen.java tools\dash\SmoothGen.java tools\dash\DashSim.java || exit /b 1
java -cp build\dashsim DashSim %*
