package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

/**
 * Target lock: while the driver holds the button, the robot turns itself to point its front (the intake and shooter end) at the open
 * CELL, and the driver keeps full control of the translation. The turn command replaces the driver's turn stick.
 *
 * The bearing comes from the Pinpoint pose to the target point ({@link HiveTarget}); heading 0 is +X and the front points along
 * the heading. The turn is a PD law on the heading error with damping from the measured angular velocity, a little static power so
 * it does not stall near the target, a dead band, and a cap. While locked the driver's turn stick nudges the aim up to
 * MAX_TRIM_DEG either way (stick right = clockwise); the nudge is forgotten when the lock is released.
 *
 * It reports {@link #isAligned()} (within ALIGNED_DEG and nearly stopped) so the driver can fire at the right moment. The gains are a
 * starting point from a simulation of the measured turn response (about 335 deg/s per unit of power): tune them on the robot.
 * AIM_OFFSET_DEG corrects a shooter whose line is not exactly the robot's forward. The shooter fires along a line parallel to the
 * front but SHOOTER_OFFSET_LEFT_IN to the side of the pose point (ROBOT_1: 2.5 in to the right, ON again since 2026-11-02 for the BLUE-bias test), so with it set
 * the robot aims that line, not its centerline, at the target: the heading is the bearing minus asin(offset / distance).
 */
@Config
public final class TargetLock {

    /** Turn power per radian of heading error. */
    public static double kP = 1.2;
    /** Damping: turn power taken off per rad/s of measured angular velocity. */
    public static double kD = 0.08;
    /** Added in the direction of the turn while the error is outside the dead band, to beat stiction. */
    public static double kStatic = 0.05;
    public static double MAX_TURN = 0.6;
    public static double DEADBAND_DEG = 1.0;
    /** Aligned = the error is within this and the robot has nearly stopped turning. */
    public static double ALIGNED_DEG = 2.0;
    public static double ALIGNED_MAX_RATE_DEG_PER_S = 25.0;
    /** The shooter's line relative to the robot's forward (positive = counterclockwise). Calibrate from where shots land. */
    public static double AIM_OFFSET_DEG = 0.0;
    /**
     * Sideways position of the shooter's line from the robot's centerline, inches, positive = to the robot's left. ROBOT_1's shooter is 2.5 in
     * to the right, so the compensation is ON at -2.5 (2026-11-02, to test the shots that landed right). Set 0 to turn it off: the lock then aims the centerline.
     */
    public static double SHOOTER_OFFSET_LEFT_IN = -2.5;
    /**
     * Extra sideways offset, inches (negative = further right, so the lock turns further left), added for the cell on the OPPOSITE side of the field from the
     * alliance's start wall (see {@link #farCellExtraLeftIn}). The marks of 2026-11-02 showed shots at that cell (blue: the audience cell, shot facing away from the
     * start heading; red by the 180 deg symmetry: the rear cell) needing about 4.5 in more, roughly the same at 23 to 41 in. The cell on the start side needed none.
     * It looks tied to the heading relative to the start, so if the start wall ever changes the cell that needs it moves too. Set 0 to turn it off.
     */
    public static double FAR_CELL_EXTRA_LEFT_IN = -4.5;
    public static double STICK_TRIM_DEG_PER_S = 40.0;
    public static double MAX_TRIM_DEG = 15.0;
    /** Closer than this to the target the bearing is meaningless (and jumps), so no turn is commanded. */
    public static double MIN_DISTANCE_IN = 12.0;

    private double trimDeg = 0.0;
    private double errorDeg = Double.NaN;
    private double bearingDeg = Double.NaN;
    private boolean aligned = false;

    /**
     * The extra sideways offset for this cell: {@link #FAR_CELL_EXTRA_LEFT_IN} when the cell is on the opposite side of the field from the alliance's start wall
     * (blue starts against the rear wall, x &lt; 0, so its audience cell, x &gt; 0; red starts against the audience wall, so its rear cell), otherwise 0.
     */
    public static double farCellExtraLeftIn(HiveTarget cell, boolean blue) {
        boolean far = blue ? cell.x > 0 : cell.x < 0;
        return far ? FAR_CELL_EXTRA_LEFT_IN : 0.0;
    }

    /**
     * One loop of the lock.
     *
     * @param pose      the robot's pose
     * @param angVel    its measured angular velocity, rad/s, counterclockwise positive
     * @param target    the point to aim at
     * @param extraOffsetLeftIn  added to {@link #SHOOTER_OFFSET_LEFT_IN} for this target (see {@link #farCellExtraLeftIn})
     * @param stickX    the driver's turn stick (right = positive), used only to nudge the aim
     * @param dtSec     seconds since the last call
     * @return the angular drive power, counterclockwise positive
     */
    public double turn(Pose2d pose, double angVel, HiveTarget target, double extraOffsetLeftIn, double stickX, double dtSec) {
        double dx = target.x - pose.position.x;
        double dy = target.y - pose.position.y;
        if (Math.hypot(dx, dy) < MIN_DISTANCE_IN) {
            errorDeg = Double.NaN;
            bearingDeg = Double.NaN;
            aligned = false;
            return 0.0;
        }
        trimDeg = clamp(trimDeg - stickX * STICK_TRIM_DEG_PER_S * dtSec, -MAX_TRIM_DEG, MAX_TRIM_DEG);
        double bearing = Math.atan2(dy, dx);
        // the shooter's line sits SHOOTER_OFFSET_LEFT_IN to the side of the pose point, so turn until THAT line passes through the target
        double parallax = Math.asin(clamp((SHOOTER_OFFSET_LEFT_IN + extraOffsetLeftIn) / Math.hypot(dx, dy), -0.9, 0.9));
        double aim = bearing - parallax + Math.toRadians(AIM_OFFSET_DEG + trimDeg);
        double error = Rotation2d.normalize(aim - pose.heading.toDouble());
        bearingDeg = Math.toDegrees(bearing);
        errorDeg = Math.toDegrees(error);
        aligned = Math.abs(errorDeg) <= ALIGNED_DEG && Math.abs(Math.toDegrees(angVel)) <= ALIGNED_MAX_RATE_DEG_PER_S;
        if (Math.abs(errorDeg) < DEADBAND_DEG) return 0.0;
        double power = kP * error - kD * angVel;
        power += Math.copySign(kStatic, power);
        return clamp(power, -MAX_TURN, MAX_TURN);
    }

    /** Ends the lock: the nudge is forgotten and the readouts clear. */
    public void release() {
        trimDeg = 0.0;
        errorDeg = Double.NaN;
        bearingDeg = Double.NaN;
        aligned = false;
    }

    /** Heading error to the (nudged) aim, degrees, positive = the target is counterclockwise of the front. NaN when not computed. */
    public double errorDeg() {
        return errorDeg;
    }

    /** Field bearing from the robot to the target, degrees. NaN when not computed. */
    public double bearingDeg() {
        return bearingDeg;
    }

    public double trimDeg() {
        return trimDeg;
    }

    public boolean isAligned() {
        return aligned;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
