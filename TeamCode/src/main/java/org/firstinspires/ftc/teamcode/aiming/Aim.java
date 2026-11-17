package org.firstinspires.ftc.teamcode.aiming;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Vec;

/**
 * The aim heading for a hive cell (specs/aiming.md sections 1 and 5). The launcher's line is parallel to the robot's
 * forward axis but offset sideways, so that line, not the centerline, must pass through the cell point.
 *
 * The aim is built here and nowhere else (target T1): geometry, plus one starting allowance per cell, plus the
 * operator's remembered aim, plus the driver's nudge. The autos use the geometry and one trim per launch.
 */
@Config
public final class Aim {
    private Aim() { }

    /** The launcher's sideways position, positive to the robot's left. -2.5 = 2.5 in to the right. */
    public static double LAUNCHER_OFFSET_IN = -2.5;
    /** The starting allowance for the alliance's far cell: an extra sideways offset for odometry drift. A starting point for the operator's trims. */
    public static double FAR_CELL_EXTRA_IN = -3.0;
    /** Corrects a launcher whose line is not exactly forward. */
    public static double ANGLE_OFFSET_DEG = 0.0;
    /** The ratio offset / distance is limited to this either way. */
    public static final double MAX_RATIO = 0.9;

    /** The starting allowance for a cell, inches sideways: the far-cell extra for the far cell, 0 for the near cell. */
    public static double allowanceIn(Cell cell) {
        return cell.isFar() ? FAR_CELL_EXTRA_IN : 0.0;
    }

    public static double distance(double x, double y, Cell cell) {
        return Math.hypot(cell.x() - x, cell.y() - y);
    }

    public static double bearing(double x, double y, Cell cell) {
        return Math.atan2(cell.y() - y, cell.x() - x);
    }

    /** aim heading = bearing - asin(offset / distance) + angle offset, with an extra sideways offset for this cell. */
    public static double geometry(double x, double y, Cell cell, double extraIn) {
        double d = Math.max(1e-6, distance(x, y, cell));
        double ratio = Angle.clamp((LAUNCHER_OFFSET_IN + extraIn) / d, -MAX_RATIO, MAX_RATIO);
        return Angle.wrap(bearing(x, y, cell) - Math.asin(ratio) + Math.toRadians(ANGLE_OFFSET_DEG));
    }

    /** The aim for the TeleOp's lock, the dashes and the tuners: geometry + starting allowance + remembered aim + nudge. */
    public static double forCell(double x, double y, Cell cell, double rememberedDeg, double nudgeDeg) {
        return Angle.wrap(geometry(x, y, cell, allowanceIn(cell)) + Math.toRadians(rememberedDeg + nudgeDeg));
    }

    /** The aim for an auto launch: geometry and one trim that holds all of that launch's allowance. No far-cell extra. */
    public static double forAuto(double x, double y, Cell cell, double trimDeg) {
        return Angle.wrap(geometry(x, y, cell, 0.0) + Math.toRadians(trimDeg));
    }

    /** With a launch grid residual: aim = bearing + residual + nudge + remembered aim. */
    public static double fromResidual(double x, double y, Cell cell, double residualRad, double rememberedDeg, double nudgeDeg) {
        return Angle.wrap(bearing(x, y, cell) + residualRad + Math.toRadians(rememberedDeg + nudgeDeg));
    }

    /** The cell's launch spot, worked out from the launcher offset and the cell's allowance (target T1). */
    public static Vec launchSpot(Cell cell) {
        return Field.launchSpot(cell, LAUNCHER_OFFSET_IN + allowanceIn(cell));
    }

    /**
     * Predicted miss at a heading: how far the cell point lies to the left of the launcher's line (inches; negative = to the right).
     * Uses the launcher offset alone.
     */
    public static double missIn(double x, double y, double heading, Cell cell) {
        double lx = -Math.sin(heading), ly = Math.cos(heading);
        double ox = x + lx * LAUNCHER_OFFSET_IN, oy = y + ly * LAUNCHER_OFFSET_IN;
        return (cell.x() - ox) * lx + (cell.y() - oy) * ly;
    }
}
