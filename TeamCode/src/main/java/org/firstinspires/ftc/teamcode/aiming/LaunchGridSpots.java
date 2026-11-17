package org.firstinspires.ftc.teamcode.aiming;

import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Which spots a launch grid contains (specs/aiming.md section 4.4): the spot generator used by the Launch Grid Tuner.
 * Pure logic. The lattice is centered on the field with a 12 in pitch, and a lattice point is kept for a cell only when
 *   1. it is on the cell's open side;
 *   2. it is at least 12 in from the cell;
 *   3. the shot is within 75 deg of the opening's normal;
 *   4. the robot, turned to its aim heading there, overlaps no hive support or flower stand (with the route planner's
 *      margin) and fits inside the walls less 3 in.
 * A point that fails only on a wall is slid toward the field center, up to 8 in: on each axis it is moved by as much
 * as the robot's rectangle is over the limit, and that is repeated until it fits.
 *
 * The spots are visited as a serpentine over the columns (x), the column nearest the start first; inside a column
 * they are taken along y, beginning at the end nearer to where the column before finished (specs/support-opmodes.md
 * section 2.2).
 *
 * Rule 4 needs the robot's outline and the planner's obstacles, which this package may not reach (the layers of
 * specs/architecture.md section 2.1), so the caller supplies it as a Fit.
 */
public final class LaunchGridSpots {
    private LaunchGridSpots() { }

    public static final double PITCH_IN = 12.0, MIN_FROM_CELL_IN = 12.0, MAX_OFF_NORMAL_DEG = 75.0, WALL_INSET_IN = 3.0, SLIDE_MAX_IN = 8.0;

    /** Rule 4, supplied by the caller: the robot at (x, y) turned to a heading. */
    public interface Fit {
        /** Overlaps no hive support or flower stand, with the route planner's margin. */
        boolean clearOfObstacles(double x, double y, double heading);

        /** How far the robot's rectangle is over the walls less an inset, on x and on y: {0, 0} when it fits. */
        double[] overWalls(double x, double y, double heading, double inset);
    }

    /** One spot of a grid. */
    public static final class Spot {
        public final double x, y;
        /** The lattice point failed on a wall and was slid toward the field center. */
        public final boolean slid;

        Spot(double x, double y, boolean slid) {
            this.x = x;
            this.y = y;
            this.slid = slid;
        }
    }

    /** The aim heading the robot is turned to at a spot: today's aim for the cell from there. */
    public static double aimAt(double x, double y, Cell cell) {
        return Aim.forCell(x, y, cell, 0, 0);
    }

    /** Why a position is not a spot of the cell's grid, or null when it passes every rule as it stands. */
    public static String invalid(double x, double y, Cell cell, Fit fit) {
        if ((x - cell.x()) * cell.openSign() <= 0) return "not on the cell's open side";
        if (Aim.distance(x, y, cell) < MIN_FROM_CELL_IN) return "closer than " + MIN_FROM_CELL_IN + " in to the cell";
        double off = Math.toDegrees(Angle.diff(Aim.bearing(x, y, cell), cell.faceHeading()));
        if (Math.abs(off) > MAX_OFF_NORMAL_DEG) return "more than " + MAX_OFF_NORMAL_DEG + " deg off the opening's normal";
        double h = aimAt(x, y, cell);
        if (!fit.clearOfObstacles(x, y, h)) return "the robot overlaps a hive support or a flower stand";
        double[] over = fit.overWalls(x, y, h, WALL_INSET_IN);
        if (over[0] > 1e-9 || over[1] > 1e-9) return "the robot does not fit inside the walls less " + WALL_INSET_IN + " in";
        return null;
    }

    /** A lattice point as a spot: itself, or slid in from a wall, or null when it is not in the grid. */
    private static Spot place(double px, double py, Cell cell, Fit fit) {
        if (invalid(px, py, cell, fit) == null) return new Spot(px, py, false);
        // Only a point that fails on a wall alone is slid.
        boolean wallOnly = (px - cell.x()) * cell.openSign() > 0 && Aim.distance(px, py, cell) >= MIN_FROM_CELL_IN
                && Math.abs(Math.toDegrees(Angle.diff(Aim.bearing(px, py, cell), cell.faceHeading()))) <= MAX_OFF_NORMAL_DEG
                && fit.clearOfObstacles(px, py, aimAt(px, py, cell));
        if (!wallOnly) return null;
        double x = px, y = py;
        for (int k = 0; k < 40; k++) {
            double[] over = fit.overWalls(x, y, aimAt(x, y, cell), WALL_INSET_IN);
            if (over[0] <= 1e-9 && over[1] <= 1e-9) break;
            // On each axis, in by as much as the rectangle is over the limit; turning to the new aim may put it over again.
            if (over[0] > 0) x -= Math.signum(x) * (over[0] + 0.05);
            if (over[1] > 0) y -= Math.signum(y) * (over[1] + 0.05);
        }
        if (Math.hypot(x - px, y - py) > SLIDE_MAX_IN) return null;
        return invalid(x, y, cell, fit) == null ? new Spot(x, y, true) : null;
    }

    /**
     * The spots of a cell's grid, in the order they are visited.
     *
     * @param startX where the sweep starts: the column nearest to it is first
     * @param startY the first column begins at its end nearer to this
     */
    public static List<Spot> spots(Cell cell, Fit fit, double startX, double startY) {
        int n = (int) Math.floor(Field.HALF_IN / PITCH_IN);
        final List<List<Spot>> columns = new ArrayList<>();
        final List<Double> columnX = new ArrayList<>();
        for (int i = -n; i <= n; i++) {
            List<Spot> column = new ArrayList<>();
            for (int j = -n; j <= n; j++) {
                Spot s = place(i * PITCH_IN, j * PITCH_IN, cell, fit);
                if (s != null) column.add(s);
            }
            if (column.isEmpty()) continue;
            Collections.sort(column, new Comparator<Spot>() {
                @Override public int compare(Spot a, Spot b) { return Double.compare(a.y, b.y); }
            });
            columns.add(column);
            columnX.add(i * PITCH_IN);
        }
        // The column nearest the start first.
        List<Integer> order = new ArrayList<>();
        for (int k = 0; k < columns.size(); k++) order.add(k);
        final double sx = startX;
        Collections.sort(order, new Comparator<Integer>() {
            @Override public int compare(Integer a, Integer b) { return Double.compare(Math.abs(columnX.get(a) - sx), Math.abs(columnX.get(b) - sx)); }
        });
        List<Spot> out = new ArrayList<>();
        double lastY = startY;
        for (int k : order) {
            List<Spot> column = columns.get(k);
            // Begin at the end nearer to where the column before finished.
            if (Math.abs(column.get(column.size() - 1).y - lastY) < Math.abs(column.get(0).y - lastY)) Collections.reverse(column);
            out.addAll(column);
            lastY = column.get(column.size() - 1).y;
        }
        return out;
    }
}
