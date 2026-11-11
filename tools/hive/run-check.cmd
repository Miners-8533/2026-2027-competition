@echo off
rem Compiles and runs the Hive Dash router checks (needs a JDK on the PATH). Run from the repo root.
if not exist build\hivecheck mkdir build\hivecheck
javac -d build\hivecheck TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\HiveRoute.java tools\hive\HiveRouteCheck.java || exit /b 1
java -cp build\hivecheck HiveRouteCheck
javac -d build\hivecheck TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\HiveRoute.java tools\hive\DuoPathCheck.java || exit /b 1
java -cp build\hivecheck DuoPathCheck
javac -d build\hivecheck TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\HiveRoute.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\drive\LaunchGrid.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\HiveTarget.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\math\*.java tools\hive\LaunchGridCheck.java || exit /b 1
java -cp build\hivecheck LaunchGridCheck
