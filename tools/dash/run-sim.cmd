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
rem   tools\dash\run-sim.cmd gentables          calibrate, build the four trees and write drive\GridTables.java (the robot's tables)
rem   tools\dash\run-sim.cmd survey 12 --grid   the REAL HiveDash / ParkDash with GRID_ROUTES on and the generated tables (also: run ... --grid, replay --grid)
if not exist build\dashsim mkdir build\dashsim
set T=TeamCode\src\main\java\org\firstinspires\ftc\teamcode
set S=tools\dash\stubs
javac -d build\dashsim %S%\com\qualcomm\robotcore\util\ElapsedTime.java %S%\org\firstinspires\ftc\teamcode\util\RunLog.java %S%\com\acmerobotics\dashboard\FtcDashboard.java %S%\com\acmerobotics\dashboard\canvas\Canvas.java %S%\com\acmerobotics\dashboard\config\Config.java %S%\com\acmerobotics\dashboard\telemetry\TelemetryPacket.java %T%\math\Pose2d.java %T%\math\PoseVelocity2d.java %T%\math\Rotation2d.java %T%\math\Vector2d.java %T%\subsystems\HiveTarget.java %T%\motion\PidController.java %T%\drive\HiveRoute.java %T%\drive\HiveDash.java %T%\drive\ParkDash.java %T%\drive\HoldPosition.java %T%\drive\TargetLock.java %T%\drive\AutoLaunch.java %T%\drive\DriveTestRoutine.java %T%\drive\GridTables.java %T%\drive\GridRoutes.java %T%\drive\GridFollower.java %T%\drive\GridDash.java tools\dash\RouteGen.java tools\dash\DashSim.java || exit /b 1
java -cp build\dashsim DashSim %*
