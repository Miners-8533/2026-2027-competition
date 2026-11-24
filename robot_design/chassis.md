# Chassis, drive and odometry (ROBOT_1)

## Footprint and robot frame

From CAD (2026-11-02); poses marked on the field agree to about 0.4 in.

| Dimension | Value |
|---|---|
| Depth (front to back) | 17.48 in (444 mm) |
| Width (side to side) | 13.70 in (348 mm) |
| Origin to back edge | 7.598 in (193 mm) |
| Origin to front edge | 9.88 in (251 mm) |
| Origin to each side | 6.85 in (174 mm) |
| Origin to the farthest corner | 12.02 in |
| Height | **NOT RECORDED** (fits the 18 in starting cube with the ramp stowed; the route planner treats the robot as 18 in tall) |
| Weight | **NOT RECORDED** |

**Origin.** The robot's origin is the Pinpoint's tracking point, close to the center of rotation. It is centered side to side and **not** centered front to back. A robot with its back flush against a wall has its origin 7.598 in from that wall.

**Robot frame.** +x points out of the front (the intake and launcher end), +y to the robot's left, and a positive turn is counterclockwise seen from above.

## Drive train

- Four mecanum wheels, one motor each, driven through belts. Motor model, ratio and wheel size: **NOT RECORDED**.
- The drive motors' encoders are **not wired**. The Pinpoint is the only source of position and speed.
- The left side is configured REVERSE and the right side FORWARD, so a positive power turns every wheel robot-forward.
- The motors brake at zero power and run as a plain duty cycle.

**Wheel mixing.** With x forward, y left and w counterclockwise, each from -1 to 1:

| Wheel | Power |
|---|---|
| left front | x - y - w |
| left back | x + y - w |
| right back | x - y + w |
| right front | x + y + w |

If any wheel would exceed 1, all four are divided by the largest. This fixes the roller orientation a rebuilt chassis must have.

**Belts.** A drive belt broke in a match (Hartland Q10) with nothing to show it, and before 2026-11-05 the pulleys could slip until the belts were tightened. Each drive motor's current is therefore readable and a wheel drawing far less than the others marks a broken belt. Healthy wheels draw about 1.6 A while driven, 2.4 to 2.9 A per unit of power, within about 20 percent of each other.

## Odometry

A goBILDA Pinpoint with two goBILDA 4-bar odometry pods (about 505 ticks per inch).

| Setting | Value |
|---|---|
| X pod (measures forward travel) | 133.7 mm to the **right** of the tracking point (offset -133.7), direction FORWARD |
| Y pod (measures sideways travel) | 9.5 mm **ahead** of the tracking point (offset +9.5), direction REVERSED |
| Device loop time | 640 to 670 microseconds, status READY |

The offsets are measured from the center of rotation, not the geometric center: a tape measure put the X pod 6.5 in (165 mm) to the right, and the rotation test corrected that to 134 mm.

**The Pinpoint keeps its pose while powered.** Resetting it zeroes the position and recalibrates its IMU, which needs the robot still. An auto resets it at INIT; a TeleOp that follows an auto can take over the device's pose without a reset.

**Measured accuracy (2026-11-01 to 2026-11-03).**

| Check | Result |
|---|---|
| 48 in push along a tape | read 47.6 in |
| Rear wall to audience wall, true 126.8 in | read 125.8 in (0.8 percent short) |
| Centerline to Blue Wall, true 57.3 in | read 56.2 in (2.0 percent short) |
| Heading after 3 full spins | within 5 deg |
| Position wobble while spinning in place | 0.16 in when the offsets were set; 2.1 to 2.2 in on 2026-11-03 |
| Same spot facing 180 deg against 0 deg | y reads about 0.47 in higher |

Known weakness: the pose drifts as the robot travels away from where it was seeded, about 3 percent of a cross-field traverse, to the right of the direction of travel. Launches at the hive cell on the far side from the start wall land off by about 4 in because of it. The offsets were deliberately left as they are (a later check suggested about -158.7 / +22.8 mm).

## Measured drive performance

From the Drive Characterization run of 2026-11-07 on a fresh battery (about 12.9 V at speed). These describe the robot; the software's planners and simulations are built on them.

| Quantity | Value |
|---|---|
| Forward speed at power 0.3 / 0.6 / 0.7 / 0.85 / 1.0 | 25 / 55 / 64 / 77 to 80 / 84 in/s |
| Rear-first against front-first | about 3 percent slower |
| Strafe speed | about 0.78 to 0.80 of forward (60 in/s at 0.85) |
| Turn rate at power 0.3 / 0.5 / 0.7 / 1.0 | 111 / 208 / 315 / 466 deg/s |
| Smallest power that moves it, forward | between 0.08 (does not move) and 0.12 (creeps at 5 in/s) |
| Smallest power that moves it, strafe | above 0.16 (does not move at 0.16) |
| Smallest power that moves it, turn | between 0.08 (does not turn) and 0.12 (turns) |
| Time to reach 63 percent of a speed step | 0.30 to 0.37 s |
| Braking at zero power | 175 to 210 in/s² from 0.6 to 0.85 power, 93 from 0.3 |
| Coasting on a small power (0.2 from 53 in/s) | about 35 in/s², far gentler than braking |
| Yaw while braking after a front-first run | 5 to 9 deg |

A wheel commanded to zero brakes hard; a wheel commanded to a small power below its speed coasts. Any controller that slows the robot has to account for that difference.

The battery sags only while the robot accelerates (to about 11 V) and recovers at speed.
