@echo off
rem Regenerates nothing: compiles and runs the 2-D launch grid check (needs a JDK on the PATH). Run from the repo root.
rem To regenerate the data table first: python tools\launchgrid\make_table.py
if not exist build\gridmapcheck mkdir build\gridmapcheck
javac -d build\gridmapcheck TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\LaunchGridMap.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\LaunchGridData.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\HiveTarget.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\math\*.java tools\launchgrid\GridMapCheck.java || exit /b 1
java -cp build\gridmapcheck GridMapCheck
