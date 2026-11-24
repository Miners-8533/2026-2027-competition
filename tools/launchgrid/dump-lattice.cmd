@echo off
rem Writes the launching spots of the Launch Grid Tuner (the blue cells) to tools\launchgrid\data\lattice_*.csv (needs a JDK on the PATH). Run from the repo root.
if not exist build\dumplattice mkdir build\dumplattice
javac -d build\dumplattice TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\HiveRoute.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\LaunchGrid.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\HiveTarget.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\math\*.java tools\launchgrid\DumpLattice.java || exit /b 1
java -cp build\dumplattice DumpLattice tools/launchgrid/data
