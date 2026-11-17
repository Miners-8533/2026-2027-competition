package org.firstinspires.ftc.teamcode.aiming;

import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Vec;

import java.util.ArrayList;
import java.util.List;

/**
 * Which spots a launch grid contains (specs/aiming.md section 4.4): the spot generator used by the Launch Grid Tuner.
 * Pure logic. The lattice is centered on the field with a 12 in pitch, and a lattice point is kept for a cell only when
 *   1. it is on the cell's open side;
 *   2. it is at least 12 in from the cell;
 *   3. the shot is within 75 deg of the opening's normal;
 *   4. the robot, turned to its aim heading there, overlaps no hive support or flower stand (with the route planner's
 *      margin) and fits inside the walls less 3 in.
 * A point that fails only on a wall is slid toward the field center, up to 8 in. The spots are visited as a
 * serpentine sweep.
 *
 * Rule 4 needs the robot's outline and the planner's obstacles, which this package may not reach (the layers of
 * specs/architecture.md section 2.1), so the caller supplies it as a Fit.
 *
 * SPEC GAP: the direction of the serpentine is not given. Here: columns of rising x, the first column swept toward
 * +y, the next toward -y, and so on. The slide is tried in 1 in steps.
 */
public final class LaunchGridSpots {
    private LaunchGridSpots() { }

    public static final double PITCH_IN = 12.0, MIN_FROM_CELL_IN = 12.0, MAX_OFF_NORMAL_DEG = 75.0, WALL_INSET_IN = 3.0, SLIDE_MAX_IN = 8.0;

    /** Rule 4, supplied by the caller: the robot at (x, y) turned to a heading. */
    public interface Fit {
        /** Overlaps no hive support or flower stand, with the route planner's margin. */
        boolean clearOfObstacles(double x, double y, double heading);

        /** Fits inside the walls less an inset. */
        boolean insideWalls(double x, double y, double heading, double inset);
    }

    /** The aim heading the robot is turned to at a spot: today's aim for the cell from there. */
    public static double aimAt(double x, double y, Cell cell) {
        return Aim.forCell(x, y, cell, 0, 0);
    }

    private static boolean rules123(double x, double y, Cell cell) {
        if ((x - cell.x()) * cell.openSign() <= 0) return false;                                // 1. the cell's open side
        if (Aim.distance(x, y, cell) < MIN_FROM_CELL_IN) return false;                          // 2. not too close
        double off = Math.toDegrees(Angle.diff(Aim.bearing(x, y, cell), cell.faceHeading()));   // 3. the shot against the opening's normal
        return Math.abs(off) <= MAX_OFF_NORMAL_DEG;
    }

    /** The spots of a cell's grid, in the order they are visited. */
    public static List<Vec> spots(Cell cell, Fit fit) {
        List<Vec> out = new ArrayList<>();
        int n = (int) Math.floor(Field.HALF_IN / PITCH_IN);
        boolean up = true;
        for (int i = -n; i <= n; i++) {
            List<Vec> column = new ArrayList<>();
            for (int j = -n; j <= n; j++) {
                double x = i * PITCH_IN, y = j * PITCH_IN;
                Vec kept = null;
                for (double slide = 0; slide <= SLIDE_MAX_IN + 1e-9 && kept == null; slide += 1.0) {
                    double d = Math.hypot(x, y), sx = d < 1e-9 ? x : x - x / d * slide, sy = d < 1e-9 ? y : y - y / d * slide;
                    if (!rules123(sx, sy, cell)) break;
                    double h = aimAt(sx, sy, cell);
                    if (!fit.clearOfObstacles(sx, sy, h)) break;        // only a point that fails on a wall alone is slid
                    if (fit.insideWalls(sx, sy, h, WALL_INSET_IN)) kept = new Vec(sx, sy);
                }
                if (kept != null) column.add(kept);
            }
            if (column.isEmpty()) continue;
            if (!up) java.util.Collections.reverse(column);
            up = !up;
            out.addAll(column);
        }
        return out;
    }
}
