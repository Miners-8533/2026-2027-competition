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
 * AIM_OFFSET_DEG corrects a shooter whose line is not exactly the robot's forward.
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
    public static double STICK_TRIM_DEG_PER_S = 40.0;
    public static double MAX_TRIM_DEG = 15.0;
    /** Closer than this to the target the bearing is meaningless (and jumps), so no turn is commanded. */
    public static double MIN_DISTANCE_IN = 12.0;

    private double trimDeg = 0.0;
    private double errorDeg = Double.NaN;
    private double bearingDeg = Double.NaN;
    private boolean aligned = false;

    /**
     * One loop of the lock.
     *
     * @param pose      the robot's pose
     * @param angVel    its measured angular velocity, rad/s, counterclockwise positive
     * @param target    the point to aim at
     * @param stickX    the driver's turn stick (right = positive), used only to nudge the aim
     * @param dtSec     seconds since the last call
     * @return the angular drive power, counterclockwise positive
     */
    public double turn(Pose2d pose, double angVel, HiveTarget target, double stickX, double dtSec) {
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
        double aim = bearing + Math.toRadians(AIM_OFFSET_DEG + trimDeg);
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
