@echo off
rem Compiles and runs the CS5 PeakCounter / EntryGrouper / EntryPipeline replay check (needs a JDK on the PATH). Run from the repo root.
if not exist build\entrycheck mkdir build\entrycheck
javac -d build\entrycheck TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\PeakCounter.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\EntryGrouper.java TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\EntryPipeline.java tools\entry\PeakCounterCheck.java || exit /b 1
java -cp build\entrycheck PeakCounterCheck || exit /b 1
rem The kind of each entry (CS5's color, 2026-11-17): the recorded pollen run and made-up nectar through EntryClassifier.
javac -d build\entrycheck -cp build\entrycheck TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\EntryClassifier.java tools\entry\EntryClassifierCheck.java || exit /b 1
java -cp build\entrycheck EntryClassifierCheck
