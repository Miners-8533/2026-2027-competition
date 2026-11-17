package org.firstinspires.ftc.teamcode.subsystems;

/**
 * The launch map (specs/flywheel.md section 5): flywheel speed from the distance to the hive cell. The hood is fixed,
 * so distance is the only input. Linear interpolation between measured points; the first and last speeds are held
 * outside the table. Distance is from the robot's origin to the cell point.
 */
public final class LaunchMap {
    private LaunchMap() { }

    /** {distance in, speed ticks/s}. Measured 2026-11-01 with a charged battery and real pollen, four-element salvos, at one cell. */
    private static final double[][] MEASURED = {
            {28.6, 1325},
            {32.0, 1350},
            {35.8, 1350},
            {42.3, 1350},
            {46.7, 1375},
            {62.0, 1450},
            {82.2, 1650},
    };

    private static double[][] table = MEASURED;

    public static boolean isEmpty() { return table.length == 0; }

    /** The speed for a distance, or NaN when the table is empty. */
    public static double speed(double distanceIn) {
        if (table.length == 0) return Double.NaN;
        if (distanceIn <= table[0][0]) return table[0][1];
        for (int i = 1; i < table.length; i++) {
            if (distanceIn <= table[i][0]) {
                double f = (distanceIn - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
                return table[i - 1][1] + f * (table[i][1] - table[i - 1][1]);
            }
        }
        return table[table.length - 1][1];
    }

    /** For the Launch Tuner: use the points recorded this session (sorted by distance), or null to go back to the saved map. */
    public static void use(double[][] points) {
        table = points == null ? MEASURED : points;
    }
}
