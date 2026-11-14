@echo off
rem Compiles and runs the CS5 PeakCounter / EntryGrouper / EntryPipeline replay check (needs a JDK on the PATH). Run from the repo root.
if not exist build\entrycheck mkdir build\entrycheck
javac -d build\entrycheck TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\PeakCounter.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\EntryGrouper.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\EntryPipeline.java tools\entry\PeakCounterCheck.java || exit /b 1
java -cp build\entrycheck PeakCounterCheck
