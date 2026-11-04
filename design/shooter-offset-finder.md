# Shooter Offset Finder (procedure)

Opmode **Shooter Offset Finder** (group Bring-up, `opmodes/bringup/ShooterOffsetFinder.java`). Target lock is OFF. The driver drives field-oriented, lines the shot up with the centerline of a HIVE cell by hand, launches, and marks a good salvo; the log keeps the pose at that launch. From the marks, `tools/logs/shooter_offset.py` gives the shooter's sideways offset directly, and splits it from any error in where the hive's cells are. Built 2026-11-03, **not yet run on the robot**.

## Controls (gamepad 1 only; no Start or Back)

| Control | Does |
|---|---|
| INIT: D-pad right / left | BLUE start (back against the rear wall facing +X, (-63.4, +16)) / RED start (back against the audience wall facing -X, (+63.4, -16)), the TeleOp's default start poses |
| Left stick / right stick X | drive field-oriented (stick up = away from the driver of the chosen start's alliance, as in the TeleOp) / turn |
| Left bumper (hold) | precision drive (30%), for lining up |
| Right trigger (hold) / left trigger (hold) | intake IN, even when full / intake OUT (the intake also runs IN by itself until 4 are held, like the TeleOp) |
| A | launch a salvo |
| Y | MARK a good salvo |
| D-pad left / right | MARK a salvo that landed LEFT / RIGHT of the cell |
| X (hold 1 s) | reseed the pose to the INIT start pose (put the robot on that spot first) |

The flywheel runs from Start. Its speed follows the distance to the cell nearest the robot (all four cells count, whichever alliance was picked).

## Procedure

1. Initialize with the robot on the start pose, still. Pick BLUE or RED.
2. Drive to the wall under test (the rear wall for the two rear cells, shot facing +X; the audience wall for the two audience cells, shot facing -X), back against the wall. Line the shooter up with the centerline of the cell by eye, load, launch (A). On a good salvo press Y. Repeat a few times per cell, moving the robot a little each time and finding the line again.
3. Do every cell: the two REAR cells (blue rear at y +12.75, red rear at y -12.75) from the rear wall facing +X, then the two AUDIENCE cells from the audience wall facing -X. The BLUE start seeds the pose at the rear wall and the RED start at the audience wall, so pick the one that matches the wall you are testing (restart the opmode to change it). Driving from y +16 to the red hive's line (y -12.75) is about 29 in of odometry; after any long drive, put the robot back on the start spot and reseed (hold X 1 s), or the pose will have drifted.
4. To separate the shooter's offset from an angle error, mark each cell at two distances along the same line (for example about 30 in and about 50 in out from the cell): the offset is the same at both, an angle error grows with distance.

## Reading the log

```
python tools/logs/shooter_offset.py            # the newest bringup log
```

For each cell it prints the good marks and the mean and spread of "cell center left of the front line". With the robot facing +X that number is s - t, facing -X it is s + t, where s is the shooter's sideways offset (robot frame, left positive, the same at both headings) and t is a sideways error in where the hive's cell centers are (field +Y, the same at both cells of the hive). The script gives s as the mean of a hive's two cells and t as half their difference, per hive, and the fit against distance (slope = aim angle error) when a cell has marks 8 in or more apart. The bearing and the shot-map distance use the pose, so keep the pose honest (reseed often).
