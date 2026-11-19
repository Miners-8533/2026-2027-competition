package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

import java.util.Locale;

/**
 * The launch equation (2026-11-18, not run on the robot): the flywheel speed and the aim for a launch from anywhere, as two short formulas fitted to the SAME measured spots the 2-D
 * launch grid holds ({@link LaunchGridData}, the Launch Grid Tuner run of 2026-11-12 at the blue rear cell, 21 good spots with a salvo). It is the third launch method
 * (subsystems.LaunchMethod.EQUATION), beside the 1-D launch map with the formula aim and the 2-D launch grid. Pure Java (no hardware): it compiles on a PC
 * (tools/launchgrid/EquationCheck.java).
 *
 * SPEED: a quadratic in the distance to the cell, {@link #speedFor}. Over the spots the speed depends on the distance only (the angle to the cell, x or y add nothing): the fit is
 * 14 ticks/s rms (16 predicting each spot from the others; the grid's blend of the neighbors gives 39), and at 82 in it gives the 1-D launch map's own farthest point (1650). Held
 * outside {@link #DIST_MIN_IN} .. {@link #DIST_MAX_IN}, as the launch map holds its end values.
 *
 * AIM: TargetLock's formula (the launcher's line, {@link #LAUNCHER_OFFSET_LEFT_IN} to the side of the pose point, turned through the target) with two changes: the target is a point
 * {@link #AIM_DEPTH_IN} deeper into the hive than the cell's point (toward X = 0), and {@link #AIM_BIAS_DEG} is added (+ = further left). Over the spots: 2.3 deg rms, worst 4.9
 * (2.6 predicting each spot from the others; the formula alone 7.8, the grid's blend 3.5). The two numbers are a CURVE FIT, not a measurement of the robot: they reproduce what the
 * spots show (from the +Y side of the blue rear cell the aim that scored was about 11 deg further left than the formula, from the -Y side about 1 deg).
 *
 * The other cells, by the grid's own rules, so the two methods differ only in how they smooth the data: the RED cells are the blue ones turned 180 degrees about the field center
 * (the residual does not change under the turn), and the blue AUDIENCE cell is the rear cell mirrored about X = 0 KEEPING THE BALL'S PATH (the launcher's line at the mirrored rear
 * spot crosses the opening face at some y; the robot aims its launcher line at that point of the audience face: tools/launchgrid/make_table.py, seed_mirror). Only the blue rear cell
 * is measured: the audience side is a guess here exactly as it is in the grid.
 *
 * The answer has the grid's form: a speed, and the aim RESIDUAL (the heading to turn to minus the bearing from the robot to the cell's point, degrees), which
 * {@link TargetLock#turn(org.firstinspires.ftc.teamcode.math.Pose2d, double, HiveTarget, double, double, double, double, double)} takes. The equation has no no-shot zones: the
 * TeleOp still shows the grid's NO SHOT ZONE warning while this method is on.
 *
 * To refit after a new Launch Grid Tuner run: fit the good spots of tools/launchgrid/data/*.csv (speed against the distance; the heading that scored against the two aim numbers)
 * and change the constants here; EquationCheck then says how well they fit the data in LaunchGridData.
 */
public final class LaunchEquation {

    /** speed (ticks/s) = C0 + C1 d + C2 d^2, d = the distance to the cell in inches. */
    public static final double SPEED_C0 = 1333.3, SPEED_C1 = -2.188, SPEED_C2 = 0.07359;
    /** The distances the speed is fitted over (the spots reach 25.6 to 76.6 in; the 1-D launch map's point at 82.2 in agrees with the curve). Outside, the end value is held. */
    public static final double DIST_MIN_IN = 25.0, DIST_MAX_IN = 82.0;
    /** The launcher's line from the robot's centerline, inches, + = left: TargetLock.LAUNCHER_OFFSET_LEFT_IN at the time of the fit (the fit's own constant, so a change there does not move this). */
    public static final double LAUNCHER_OFFSET_LEFT_IN = -2.5;
    /** The aim point is this far past the cell's point, into the hive (toward X = 0), inches. */
    public static final double AIM_DEPTH_IN = 6.1;
    /** Added to the aim, degrees, + = counterclockwise (further left). */
    public static final double AIM_BIAS_DEG = 6.0;

    private LaunchEquation() {}

    /** The flywheel speed (ticks/s) for a launch from this distance to the cell (inches). */
    public static double speedFor(double distanceIn) {
        double d = Math.max(DIST_MIN_IN, Math.min(DIST_MAX_IN, distanceIn));
        return SPEED_C0 + SPEED_C1 * d + SPEED_C2 * d * d;
    }

    /** The aim residual, degrees: the heading to turn to minus the bearing from (x, y) to the cell's point (field frame, inches). */
    public static double residualDeg(HiveTarget cell, double x, double y) {
        boolean audience = cell == HiveTarget.BLUE_AUDIENCE || cell == HiveTarget.RED_REAR;   // the cells that are the blue AUDIENCE cell, as it is or turned
        if (!cell.isBlue()) {   // a red cell: the blue one turned 180 degrees about the field center
            x = -x;
            y = -y;
        }
        double cx = HiveTarget.BLUE_REAR.x, cy = HiveTarget.BLUE_REAR.y;
        double rx = audience ? -x : x;   // the same spot at the blue rear cell
        double h = rearHeadingRad(rx, y);
        if (!audience) return Math.toDegrees(normalize(h - Math.atan2(cy - y, cx - rx)));
        // the audience cell: where the launcher's line crosses the rear cell's opening face, aimed at from the mirrored spot
        double ox = rx - LAUNCHER_OFFSET_LEFT_IN * Math.sin(h), oy = y + LAUNCHER_OFFSET_LEFT_IN * Math.cos(h);
        double faceY = oy + (cx - ox) * Math.sin(h) / Math.max(0.1, Math.cos(h));   // (the launcher points at the face from every launching spot; the limit keeps it finite beside and behind the hive)
        double dx = -cx - x, dy = faceY - y;
        double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) return 0.0;
        double heading = Math.atan2(dy, dx) - Math.asin(clamp(LAUNCHER_OFFSET_LEFT_IN / dist, -0.9, 0.9));
        return Math.toDegrees(normalize(heading - Math.atan2(cy - y, -cx - x)));
    }

    /** The heading (radians) that scores at the blue rear cell from (x, y): the launcher's line through the deeper aim point, plus the bias. */
    private static double rearHeadingRad(double x, double y) {
        double dx = HiveTarget.BLUE_REAR.x + AIM_DEPTH_IN - x, dy = HiveTarget.BLUE_REAR.y - y;
        double dist = Math.max(1e-6, Math.hypot(dx, dy));
        return Math.atan2(dy, dx) - Math.asin(clamp(LAUNCHER_OFFSET_LEFT_IN / dist, -0.9, 0.9)) + Math.toRadians(AIM_BIAS_DEG);
    }

    /** For the log: the speed and the aim for a launch at the cell from (x, y). */
    public static String describe(HiveTarget cell, double x, double y) {
        double bearing = Math.toDegrees(Math.atan2(cell.y - y, cell.x - x)), resid = residualDeg(cell, x, y);
        return String.format(Locale.US, "speed %.0f ticks/s, aim %.1f deg (bearing %.1f %+.1f)", speedFor(Math.hypot(cell.x - x, cell.y - y)), bearing + resid, bearing, resid);
    }

    private static double normalize(double rad) {
        double a = rad % (2.0 * Math.PI);
        if (a > Math.PI) a -= 2.0 * Math.PI;
        if (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
