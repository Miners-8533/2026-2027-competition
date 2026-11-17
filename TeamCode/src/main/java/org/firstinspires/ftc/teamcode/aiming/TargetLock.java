package org.firstinspires.ftc.teamcode.aiming;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * Target lock (specs/aiming.md section 2). While the lock is on, the robot turns itself onto a cell and the driver keeps
 * full control of translation. Pure logic: a snapshot in, a turn command out.
 */
@Config
public final class TargetLock {
    public static double KP = 1.2;                 // per radian
    public static double KD = 0.08;                // per rad/s
    public static double STATIC_POWER = 0.05;
    public static double MAX_TURN = 0.6;
    public static double DEAD_BAND_DEG = 1.0;
    public static double ALIGNED_DEG = 2.0;
    public static double ALIGNED_RATE_DEG_S = 25.0;
    public static double NUDGE_RATE_DEG_S = 40.0;
    public static double NUDGE_LIMIT_DEG = 15.0;
    public static double MIN_DISTANCE_IN = 12.0;

    private double nudgeDeg;
    private double errorDeg = Double.NaN;
    private double aimRad = Double.NaN;
    private boolean aligned;

    /** The turn law shared with the autos' aim: kP x error - kD x turn rate, plus a small static power, limited. 0 inside the dead band. */
    public static double turnLaw(double errorRad, double omega) {
        if (Math.abs(Math.toDegrees(errorRad)) <= DEAD_BAND_DEG) return 0;
        double turn = KP * errorRad - KD * omega + Math.signum(errorRad) * STATIC_POWER;
        return Angle.clampAbs(turn, MAX_TURN);
    }

    /**
     * @param omega          measured turn rate, rad/s
     * @param rememberedDeg  the operator's standing correction for this cell
     * @param nudgeStick     the driver's turn stick while nudging is allowed (right is positive), else 0
     * @param gridResidual   a launch grid residual in radians, or NaN for the formula
     * @return the turn command (counterclockwise positive)
     */
    public double update(Pose pose, double omega, Cell cell, double rememberedDeg, double nudgeStick, double dt, double gridResidual) {
        // Stick right is clockwise.
        nudgeDeg = Angle.clamp(nudgeDeg - nudgeStick * NUDGE_RATE_DEG_S * dt, -NUDGE_LIMIT_DEG, NUDGE_LIMIT_DEG);
        if (Aim.distance(pose.x, pose.y, cell) < MIN_DISTANCE_IN) {
            // Too close: the bearing is meaningless.
            aligned = false;
            errorDeg = Double.NaN;
            aimRad = Double.NaN;
            return 0;
        }
        aimRad = Double.isNaN(gridResidual)
                ? Aim.forCell(pose.x, pose.y, cell, rememberedDeg, nudgeDeg)
                : Aim.fromResidual(pose.x, pose.y, cell, gridResidual, rememberedDeg, nudgeDeg);
        double err = Angle.diff(aimRad, pose.h);
        errorDeg = Math.toDegrees(err);
        aligned = Math.abs(errorDeg) <= ALIGNED_DEG && Math.abs(Math.toDegrees(omega)) < ALIGNED_RATE_DEG_S;
        return turnLaw(err, omega);
    }

    /** Clears the nudge and the readouts. */
    public void release() {
        nudgeDeg = 0;
        errorDeg = Double.NaN;
        aimRad = Double.NaN;
        aligned = false;
    }

    public boolean aligned() { return aligned; }
    /** Aim minus heading, degrees; NaN when the lock is off or too close. */
    public double errorDeg() { return errorDeg; }
    public double nudgeDeg() { return nudgeDeg; }
    public double aimRad() { return aimRad; }
}
