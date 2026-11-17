package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pid;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Drive to a field pose and stop there (specs/motion.md section 2). Every auto leg is one of these.
 * Three independent PID controllers on the field-x error, the field-y error and the heading error.
 *
 * Every leg is one of two kinds (target T2): a settling leg must end inside its tolerance, and a time-out on it is
 * logged as a fault; a timed leg ends on its clock by intent (the ram).
 *
 * The gains are still the starting values of 2026-09-14: they were never tuned on ROBOT_1.
 */
@Config
public final class DriveToPose implements Action {
    public static double AXIAL_KP = 0.05, AXIAL_KI = 0.0, AXIAL_KD = 0.006;
    public static double LATERAL_KP = 0.06, LATERAL_KI = 0.0, LATERAL_KD = 0.008;
    public static double HEADING_KP = 1.5, HEADING_KI = 0.0, HEADING_KD = 0.08;      // per radian
    public static double MAX_POWER = 0.6;
    public static double INTEGRAL_CLAMP = 0.25;
    public static double POSITION_TOL_IN = 1.0;
    public static double HEADING_TOL_DEG = 2.0;
    public static double SETTLE_S = 0.20;
    public static double TIME_CAP_S = 4.0;
    public static double STICTION_RUN_FORWARD = 0.065, STICTION_RUN_STRAFE = 0.11, STICTION_RUN_TURN = 0.09;
    public static double STICTION_START_FORWARD = 0.10, STICTION_START_STRAFE = 0.15, STICTION_START_TURN = 0.14;
    public static double STICTION_MIN_ERROR_IN = 0.3;
    public static double STILL_IN_S = 1.5, STILL_DEG_S = 6.0;
    // The time cap from distance: max(minimum, base + distance / speed), plus an extra for a big turn, never above a maximum.
    public static double CAP_MIN_S = 1.5, CAP_BASE_S = 1.0, CAP_SPEED_IN_S = 22.0, CAP_SPEED_AT_POWER = 0.6;
    public static double CAP_TURN_OVER_DEG = 120.0, CAP_TURN_EXTRA_S = 0.5, CAP_MAX_S = 8.0;

    /** Supplies the target when the leg starts, so a leg can depend on the alliance or on a Dashboard value. */
    public interface Target { Pose get(); }

    private final MecanumDrive drive;
    private final Target targetSource;
    private final String name;

    private double timeout = Double.NaN;
    private boolean distanceTimeout;
    private double capSpeed = Double.NaN;
    private double tolIn = Double.NaN, tolDeg = Double.NaN, settle = Double.NaN, maxPower = Double.NaN;
    private double slowZoneIn = 0, slowZonePower = 1;
    private boolean stiction, timed;

    private final Pid px = new Pid(), py = new Pid(), ph = new Pid();
    private Pose target, start;
    private double startT = -1, lastT, insideSince = -1, cap;
    private double overshootIn, overshootDeg, turnSign, ux, uy;
    private boolean done, settled;
    private double seconds, finalErrIn, finalErrDeg;

    public DriveToPose(MecanumDrive drive, String name, Target target) {
        this.drive = drive;
        this.name = name;
        this.targetSource = target;
    }

    public DriveToPose(MecanumDrive drive, String name, final Pose target) {
        this(drive, name, new Target() {
            @Override public Pose get() { return target; }
        });
    }

    // ---- Options a leg may set ----
    /** A fixed cap for this leg. */
    public DriveToPose timeout(double s) { timeout = s; return this; }
    /** The cap worked out from the distance when the leg starts. */
    public DriveToPose distanceTimeout() { distanceTimeout = true; return this; }
    /** The same, at a given speed in in/s (for a slow leg). */
    public DriveToPose distanceTimeout(double speedInS) { distanceTimeout = true; capSpeed = speedInS; return this; }
    /** A coarser position and heading tolerance, for pass-through points. */
    public DriveToPose tolerance(double in, double deg) { tolIn = in; tolDeg = deg; return this; }
    /** 0 ends the leg the moment it is inside tolerance. */
    public DriveToPose settle(double s) { settle = s; return this; }
    /** A lower (or higher) per-axis clamp. */
    public DriveToPose maxPower(double p) { maxPower = p; return this; }
    /** Within a distance of the target the translation power magnitude is limited. */
    public DriveToPose slowZone(double in, double power) { slowZoneIn = in; slowZonePower = power; return this; }
    /** Adds the static friction term. */
    public DriveToPose stiction(boolean on) { stiction = on; return this; }
    /** A timed leg ends on its clock by intent; its time-out is not a fault. */
    public DriveToPose timed() { timed = true; return this; }

    // ---- Results, for the opmode that ran it ----
    public boolean done() { return done; }
    public boolean settled() { return settled; }
    public double seconds() { return seconds; }
    public double finalErrorIn() { return finalErrIn; }
    public double finalErrorDeg() { return finalErrDeg; }
    public double overshootIn() { return overshootIn; }
    public double overshootDeg() { return overshootDeg; }
    public Pose target() { return target; }

    @Override
    public boolean run(TelemetryPacket packet) {
        double t = Clock.now();
        Pose p = drive.pose();
        double pMax = Double.isNaN(maxPower) ? MAX_POWER : maxPower;
        if (startT < 0) {
            startT = lastT = t;
            start = p;
            target = targetSource.get();
            double dist = p.distTo(target), turnDeg = Math.abs(Math.toDegrees(Angle.diff(target.h, p.h)));
            if (!Double.isNaN(timeout)) {
                cap = timeout;
            } else if (distanceTimeout) {
                double speed = Double.isNaN(capSpeed) ? CAP_SPEED_IN_S * pMax / CAP_SPEED_AT_POWER : capSpeed;
                cap = Math.min(CAP_MAX_S, Math.max(CAP_MIN_S, CAP_BASE_S + dist / speed) + (turnDeg > CAP_TURN_OVER_DEG ? CAP_TURN_EXTRA_S : 0));
            } else {
                cap = TIME_CAP_S;
            }
            double len = Math.max(1e-6, dist);
            ux = (target.x - p.x) / len;
            uy = (target.y - p.y) / len;
            turnSign = Math.signum(Angle.diff(target.h, p.h));
            RunLog.eventf("DRIVE start %s: %s -> %s, %.1f in, %.0f deg, cap %.2f s, max power %.2f%s%s", name, p, target, dist, turnDeg, cap, pMax,
                    stiction ? ", stiction" : "", timed ? ", timed" : "");
        }
        double dt = t - lastT;
        lastT = t;
        double ex = target.x - p.x, ey = target.y - p.y, eh = Angle.diff(target.h, p.h);
        double errIn = Math.hypot(ex, ey), errDeg = Math.toDegrees(eh);
        // Overshoot: the furthest past the target along the line of travel, and past the target heading in the direction of the turn.
        overshootIn = Math.max(overshootIn, (p.x - target.x) * ux + (p.y - target.y) * uy);
        overshootDeg = Math.max(overshootDeg, -turnSign * errDeg);

        double tIn = Double.isNaN(tolIn) ? POSITION_TOL_IN : tolIn, tDeg = Double.isNaN(tolDeg) ? HEADING_TOL_DEG : tolDeg;
        boolean inside = errIn <= tIn && Math.abs(errDeg) <= tDeg;
        if (inside) {
            if (insideSince < 0) insideSince = t;
        } else {
            insideSince = -1;
        }
        boolean settledNow = inside && t - insideSince >= (Double.isNaN(settle) ? SETTLE_S : settle);
        boolean timedOut = t - startT >= cap;
        if (settledNow || timedOut) {
            drive.stop();
            done = true;
            settled = settledNow;
            seconds = t - startT;
            finalErrIn = errIn;
            finalErrDeg = errDeg;
            RunLog.eventf("DRIVE %s %s in %.2f s: error %.1f in, %.1f deg, overshoot %.1f in, %.1f deg, at %s",
                    settledNow ? "settled" : (timed ? "TIMED OUT (timed leg)" : "TIMED OUT (FAULT: a settling leg)"), name, seconds,
                    errIn, errDeg, overshootIn, overshootDeg, p);
            return false;
        }

        // The control law. The gains are read every loop, so Dashboard edits act at once.
        double cx = Angle.clampAbs(px.update(ex, dt, AXIAL_KP, AXIAL_KI, AXIAL_KD, INTEGRAL_CLAMP), pMax);
        double cy = Angle.clampAbs(py.update(ey, dt, LATERAL_KP, LATERAL_KI, LATERAL_KD, INTEGRAL_CLAMP), pMax);
        double turn = Angle.clampAbs(ph.update(eh, dt, HEADING_KP, HEADING_KI, HEADING_KD, INTEGRAL_CLAMP), pMax);
        if (slowZoneIn > 0 && errIn < slowZoneIn) {
            double mag = Math.hypot(cx, cy);
            if (mag > slowZonePower) {
                cx *= slowZonePower / mag;
                cy *= slowZonePower / mag;
            }
        }
        double c = Math.cos(p.h), s = Math.sin(p.h);
        double forward = cx * c + cy * s, left = -cx * s + cy * c;
        if (stiction) {
            // Without it a small error commands a power below the drive's dead zone and the robot does not move.
            double ef = ex * c + ey * s, el = -ex * s + ey * c;
            double vf = drive.localizer.vx() * c + drive.localizer.vy() * s, vl = -drive.localizer.vx() * s + drive.localizer.vy() * c;
            if (Math.abs(ef) >= STICTION_MIN_ERROR_IN && forward != 0) {
                forward += Math.signum(forward) * (Math.abs(vf) < STILL_IN_S ? STICTION_START_FORWARD : STICTION_RUN_FORWARD);
            }
            if (Math.abs(el) >= STICTION_MIN_ERROR_IN && left != 0) {
                left += Math.signum(left) * (Math.abs(vl) < STILL_IN_S ? STICTION_START_STRAFE : STICTION_RUN_STRAFE);
            }
            if (Math.abs(errDeg) > tDeg && turn != 0) {
                turn += Math.signum(turn) * (Math.abs(Math.toDegrees(drive.localizer.omega())) < STILL_DEG_S ? STICTION_START_TURN : STICTION_RUN_TURN);
            }
        }
        drive.drive(forward, left, turn);
        if (packet != null) {
            packet.put("drive error in", errIn);
            packet.put("drive error deg", errDeg);
        }
        return true;
    }

    /** The gains and limits in force, as one line ready to paste into the code. */
    public static String gainsLine() {
        return String.format(java.util.Locale.US,
                "AXIAL %.4f/%.4f/%.4f LATERAL %.4f/%.4f/%.4f HEADING %.3f/%.3f/%.3f MAX_POWER %.2f TOL %.1f in %.1f deg SETTLE %.2f s "
                        + "STICTION run %.3f/%.3f/%.3f start %.3f/%.3f/%.3f",
                AXIAL_KP, AXIAL_KI, AXIAL_KD, LATERAL_KP, LATERAL_KI, LATERAL_KD, HEADING_KP, HEADING_KI, HEADING_KD, MAX_POWER,
                POSITION_TOL_IN, HEADING_TOL_DEG, SETTLE_S, STICTION_RUN_FORWARD, STICTION_RUN_STRAFE, STICTION_RUN_TURN,
                STICTION_START_FORWARD, STICTION_START_STRAFE, STICTION_START_TURN);
    }
}
