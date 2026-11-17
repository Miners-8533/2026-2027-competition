@echo off
rem PC check of the from-specs rebuild (needs a JDK on the PATH). Run from the repo root.
rem Compiles the pure-logic packages of the robot code with two small stand-ins (the tunable-value marker and the build stamp)
rem and runs: the aim math, the launch spots, the route planner over a grid of starts, the CS5 entry rule replayed on the
rem recorded runs in tools\entry\data, the launch grid lookup, and both dashes on a crude model of the robot.
set T=TeamCode\src\main\java\org\firstinspires\ftc\teamcode
if exist build\fromspecs rmdir /s /q build\fromspecs
mkdir build\fromspecs
javac -nowarn -d build\fromspecs tools\fromspecs\stubs\com\acmerobotics\dashboard\config\Config.java tools\fromspecs\stubs\org\firstinspires\ftc\teamcode\util\BuildInfo.java %T%\math\*.java %T%\field\*.java %T%\robot\Footprint.java %T%\aiming\*.java %T%\navigation\*.java %T%\subsystems\EntryRule.java %T%\util\RunLog.java %T%\util\LoopProfiler.java %T%\util\Clock.java tools\fromspecs\Check.java || exit /b 1
java -cp build\fromspecs Check tools\entry\data
