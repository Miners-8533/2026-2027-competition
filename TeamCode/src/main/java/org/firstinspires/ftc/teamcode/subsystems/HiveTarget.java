package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.math.Pose2d;

/**
 * Where to aim: the open (upward) CELL of each HIVE, in field coordinates (origin at field center,
 * X toward the audience, Y toward the Blue Wall). The blue HIVE is centered on Y = +12.75 and the red
 * on Y = -12.75 (25.5 in HIVE center-to-center). A CELL's outer opening face is about 21.5 in from the
 * HIVE center in X (9.42 in inner gap + 12.04 in CELL depth, manual Figs 9-9 / 9-10). As staged, blue's
 * open CELL is on the rear side and red's on the audience side; a tip flips them. The opening is
 * 53.5 to 65.6 in above the tiles.
 *
 * Distance is measured from the robot's center to this point. Any offset to the real shooter exit is
 * absorbed by measuring the shot map on the real robot.
 */
public enum HiveTarget {
    BLUE_REAR(-21.5, 12.75),
    BLUE_AUDIENCE(21.5, 12.75),
    RED_REAR(-21.5, -12.75),
    RED_AUDIENCE(21.5, -12.75);

    public final double x;
    public final double y;

    HiveTarget(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distanceFrom(Pose2d pose) {
        return Math.hypot(x - pose.position.x, y - pose.position.y);
    }

    public HiveTarget next() {
        HiveTarget[] all = values();
        return all[(ordinal() + 1) % all.length];
    }
}
