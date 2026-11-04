package org.firstinspires.ftc.teamcode.subsystems;

import java.util.List;

/**
 * Flywheel speed from launching distance: linear interpolation over measured points, the end values
 * held beyond the table. The hood is fixed, so there are only two columns. Like last season's launch
 * map, minus the hood angle.
 *
 * {@link #TABLE} was measured on the robot on 2026-11-01 with the Shot Map Tuner (a charged battery, real pollen, a 4-element salvo
 * at each point, launching at the red audience CELL from the audience wall and the corners; the speed depends on the distance only,
 * so it serves any cell). The first and last speeds are held beyond the table. While the table is empty the coordinator uses the
 * operator's manual setpoint (Mechanisms.manualSetpoint, 1350 ticks/s). With points in the table the D-pad speed keys add a TRIM
 * to the table's speed instead (Mechanisms.speedTrim), so the speed can still be nudged live.
 */
public final class LaunchMap {

    /** {distance in inches, flywheel velocity in ticks per second}, ascending distance. */
    public static final double[][] TABLE = {
            {28.6, 1325},
            {32.0, 1350},
            {35.8, 1350},
            {42.3, 1350},
            {46.7, 1375},
            {62.0, 1450},
            {82.2, 1650},
    };

    private LaunchMap() {}

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
