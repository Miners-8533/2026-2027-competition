package org.firstinspires.ftc.teamcode.subsystems;

import java.util.List;

/**
 * Flywheel speed from shooting distance: linear interpolation over measured points, the end values
 * held beyond the table. The hood is fixed, so there are only two columns. Like last season's launch
 * map, minus the hood angle.
 *
 * {@link #TABLE} is EMPTY until it is measured on the real robot. While it is empty the coordinator uses the
 * operator's manual setpoint (Mechanisms.manualSetpoint, starting at 1400 ticks/s, tuned with the D-pad). Use the
 * Shot Map Tuner opmode to find a working speed at each distance; it prints lines to paste in. Once the table
 * has any point, the coordinator uses it and the D-pad speed keys are ignored.
 */
public final class ShotMap {

    /** {distance in inches, flywheel velocity in ticks per second}, ascending distance. */
    public static final double[][] TABLE = {
    };

    private ShotMap() {}

    public static boolean isEmpty() {
        return TABLE.length == 0;
    }

    /** Speed for this distance from the saved table, or NaN while the table is empty. */
    public static double velocityFor(double distanceIn) {
        return interpolate(TABLE, distanceIn);
    }

    public static double interpolate(double[][] points, double distanceIn) {
        if (points.length == 0) return Double.NaN;
        if (distanceIn <= points[0][0]) return points[0][1];
        for (int i = 1; i < points.length; i++) {
            if (distanceIn <= points[i][0]) {
                double d0 = points[i - 1][0], d1 = points[i][0];
                double fraction = (d1 == d0) ? 1.0 : (distanceIn - d0) / (d1 - d0);
                return points[i - 1][1] + (points[i][1] - points[i - 1][1]) * fraction;
            }
        }
        return points[points.length - 1][1];
    }

    /** Same, over a list of {distance, velocity} points sorted by ascending distance (used by the tuner). */
    public static double interpolate(List<double[]> points, double distanceIn) {
        return interpolate(points.toArray(new double[0][]), distanceIn);
    }
}
