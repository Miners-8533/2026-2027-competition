package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Drives the robot to a target field pose and holds it, using three independent PID
 * controllers on field-X error, field-Y error, and heading error. The field-frame
 * translation command is rotated into the robot frame and handed to
 * {@link MecanumDrive#setDrivePowers}.
 *
 * Finishes when position and heading are within tolerance for {@link #SETTLE_SEC},
 * or when {@link #TIMEOUT_SEC} elapses (whichever first). Stops the drive on finish.
 *
 * STICTION (2026-11-13, opt-in): without a static friction term a small error commands a power under the drive's dead zone and the robot does not move (the Launch Grid Tuner's 0.5 deg
 * re-aim: 1.5 power per radian is 0.013 at 0.5 deg, the drive needs about 0.1). {@link #withStiction} adds, per ROBOT axis (forward, strafe, turn: their dead zones differ) and only while
 * that axis is still outside the tolerance, a constant KS in the direction of the command (the DECODE approach), and a larger KS_*_START while the robot is nearly still (a robot at rest
 * needs more to start than to keep going: AimAtCell's logs needed 0.14 to start a turn against a 0.098 dead zone), dropping to the running value once it moves. Off unless asked for
 * (or STICTION_ALL_LEGS), so the autos' legs, tuned without it, are unchanged.
 *
 * All gains and tolerances are {@code @Config} statics so they can be tuned live in
 * FTC Dashboard while a tuning OpMode runs. Every {@link #run} copies the statics
 * into the controllers, so edits take effect immediately.
 */
@Config
public final class DriveToPose implements Action {

    // --- axial (field X) ---
    public static double AXIAL_kP = 0.05;
    public static double AXIAL_kI = 0.0;
    public static double AXIAL_kD = 0.006;
    public static double AXIAL_kS = 0.0;

    // --- lateral (field Y) --- strafing is weaker, usually needs a bit more P
    public static double LATERAL_kP = 0.06;
    public static double LATERAL_kI = 0.0;
    public static double LATERAL_kD = 0.008;
    public static double LATERAL_kS = 0.0;

    // --- heading (radians of error) ---
    public static double HEADING_kP = 1.5;
    public static double HEADING_kI = 0.0;
    public static double HEADING_kD = 0.08;
    public static double HEADING_kS = 0.0;

    public static double MAX_OUTPUT = 0.6;      // per-axis power clamp (keep modest while tuning)
    public static double MAX_INTEGRAL = 0.25;

    public static double POS_TOLERANCE_IN = 1.0;
    public static double HEADING_TOLERANCE_DEG = 2.0;
    public static double SETTLE_SEC = 0.20;     // time within tolerance before "done"
    public static double TIMEOUT_SEC = 4.0;     // hard cap

    // ---- static friction (stiction) compensation, per robot axis; see the class comment ----
    /** Every DriveToPose gets the stiction term (for a test run); false: only a leg that asks with withStiction(true). */
    public static boolean STICTION_ALL_LEGS = false;
    /** Added to the command's size while the axis moves (a little under its dead zone: 0.072 forward, about 0.12 sideways, 0.098 turning, the 2026-11-07 drive test). */
    public static double KS_FORWARD = 0.065, KS_STRAFE = 0.11, KS_TURN = 0.09;
    /** Added instead while the axis is nearly still (to get it started; the turn's 0.14 is AimAtCell.BREAKAWAY_POWER). */
    public static double KS_FORWARD_START = 0.10, KS_STRAFE_START = 0.15, KS_TURN_START = 0.14;
    /** An axis slower than this counts as still. */
    public static double STALL_IN_PER_S = 1.5, STALL_DEG_PER_S = 6.0;
    /** The term is added to an axis only when its own error is at least this (so a leg that strafes does not get a forward push from the forward PID's leftover). */
    public static double KS_MIN_ERR_IN = 0.3;

    private final MecanumDrive drive;
    private final Pose2d target;
    private double timeoutSec = TIMEOUT_SEC;
    private double posTolIn = Double.NaN, headTolDeg = Double.NaN;   // NaN: the statics
    private double settleSecOverride = Double.NaN;                   // NaN: SETTLE_SEC
    private double slowZoneIn = Double.NaN, slowZonePower = Double.NaN;   // NaN: no slow zone
    private boolean distanceTimeout = false;
    private double maxOutputOverride = Double.NaN;   // NaN: MAX_OUTPUT
    private double dtBaseSec, dtInPerSec, dtMinSec, dtTurnExtraSec, dtMaxSec;
    private boolean stiction = STICTION_ALL_LEGS;
    // the robot's own speed from successive poses, for the stalled test
    private double lastPoseX, lastPoseY, lastPoseH;
    private long lastPoseNs = 0L;
    private double vForward = 0.0, vLeft = 0.0, wDegPerS = 0.0;

    private final PidController xCtrl = new PidController(AXIAL_kP, AXIAL_kI, AXIAL_kD);
    private final PidController yCtrl = new PidController(LATERAL_kP, LATERAL_kI, LATERAL_kD);
    private final PidController hCtrl = new PidController(HEADING_kP, HEADING_kI, HEADING_kD);

    private final ElapsedTime timeout = new ElapsedTime();
    private final ElapsedTime settle = new ElapsedTime();
    private boolean initialized = false;
    // what the leg did, for the end-of-leg log line and for an opmode that judges it (opmodes/tuning/DriveAcceptance): see the getters at the end
    private double travelUx = 0.0, travelUy = 0.0;   // the unit vector of the leg's line of travel, start to target (0, 0 for a turn in place)
    private double turnSign = 0.0;                   // the direction the leg turns (0: no turn to speak of)
    private double overshootIn = 0.0, overshootDeg = 0.0;
    private boolean finished = false, settled = false;
    private double elapsedSec = 0.0, finalErrIn = 0.0, finalErrHeadingDeg = 0.0;

    public DriveToPose(MecanumDrive drive, Pose2d target) {
        this.drive = drive;
        this.target = target;
    }

    /** Use this hard cap, seconds, for this drive instead of {@link #TIMEOUT_SEC} (a long leg needs more than the default). */
    public DriveToPose withTimeout(double seconds) {
        timeoutSec = seconds;
        return this;
    }

    /** Use this position (inches) and heading (degrees) tolerance for this drive instead of the statics: a coarse one for a pass-through point. */
    /**
     * A slow zone (2026-11-06, for the garden pickup): within zoneIn inches of the target the translation power is limited to power (its magnitude, field frame), whatever the leg's own cap is; before it
     * the leg runs at its normal power. The proportional term already eases the power below about power / AXIAL_kP inches, so the zone only matters beyond that.
     */
    public DriveToPose withSlowZone(double zoneIn, double power) {
        slowZoneIn = zoneIn;
        slowZonePower = power;
        return this;
    }

    /** The time inside the tolerance before the drive is done (the static SETTLE_SEC is 0.20 s); 0 ends the drive the moment it is near, for a pass-through leg that must not wait. */
    public DriveToPose withSettle(double settleSec) {
        settleSecOverride = settleSec;
        return this;
    }

    /** Adds the static friction term (see the class comment) to this drive: true to turn it on, false off whatever STICTION_ALL_LEGS says. */
    public DriveToPose withStiction(boolean on) {
        stiction = on;
        return this;
    }

    public DriveToPose withTolerance(double posIn, double headingDeg) {
        posTolIn = posIn;
        headTolDeg = headingDeg;
        return this;
    }

    /**
     * Caps the power on every axis for this drive (the statics' MAX_OUTPUT is 0.6): a slow leg, for example while the intake picks up. Speed is roughly 60 in/s per unit
     * of power on a long straight run (0.6 reached about 36 in/s per axis on 2026-11-03), so 0.25 is about 15 in/s.
     */
    public DriveToPose withMaxOutput(double maxOutput) {
        maxOutputOverride = maxOutput;
        return this;
    }

    /**
     * The hard cap is worked out when the drive starts, from how far it has to go and how far it has to turn: max(minSec, baseSec + distance / inPerSec),
     * plus turnExtraSec for a turn of more than 120 degrees, and never more than maxSec. The measured speed of ROBOT_1 on 2026-11-03 was about 22 to 26 in/s
     * averaged over a long leg (51 in/s at the peak) and a short leg of 20 in arrived in about 1 s.
     */
    public DriveToPose withDistanceTimeout(double baseSec, double inPerSec, double minSec, double turnExtraSec, double maxSec) {
        distanceTimeout = true;
        dtBaseSec = baseSec;
        dtInPerSec = inPerSec;
        dtMinSec = minSec;
        dtTurnExtraSec = turnExtraSec;
        dtMaxSec = maxSec;
        return this;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!initialized) {
            initialized = true;
            timeout.reset();
            settle.reset();
            xCtrl.reset();
            yCtrl.reset();
            hCtrl.reset();
            Pose2d from = drive.getPose();
            double travelIn = Math.hypot(target.position.x - from.position.x, target.position.y - from.position.y);
            if (travelIn > 1.0) {
                travelUx = (target.position.x - from.position.x) / travelIn;
                travelUy = (target.position.y - from.position.y) / travelIn;
            }
            double turnRad = Rotation2d.normalize(target.heading.toDouble() - from.heading.toDouble());
            if (Math.abs(turnRad) > Math.toRadians(1.0)) turnSign = Math.signum(turnRad);
            if (distanceTimeout) {
                double dist = Math.hypot(target.position.x - from.position.x, target.position.y - from.position.y);
                double turn = Math.abs(Rotation2d.normalize(target.heading.toDouble() - from.heading.toDouble()));
                timeoutSec = Math.min(dtMaxSec, Math.max(dtMinSec, dtBaseSec + dist / dtInPerSec) + (turn > Math.toRadians(120.0) ? dtTurnExtraSec : 0.0));
            }
            RunLog.event(String.format("DRIVE start: (%.1f, %.1f, %.0f deg) -> (%.1f, %.1f, %.0f deg), timeout %.1f s%s",
                    from.position.x, from.position.y, from.heading.toDegrees(),
                    target.position.x, target.position.y, target.heading.toDegrees(), timeoutSec,
                    (Double.isNaN(posTolIn) ? "" : String.format(", tolerance %.1f in / %.0f deg", posTolIn, headTolDeg))
                            + (Double.isNaN(maxOutputOverride) ? "" : String.format(", max power %.2f", maxOutputOverride))
                            + (stiction ? String.format(", stiction kS %.3f / %.3f / %.3f (start %.3f / %.3f / %.3f)", KS_FORWARD, KS_STRAFE, KS_TURN, KS_FORWARD_START, KS_STRAFE_START, KS_TURN_START) : "")));
        }
        applyGains();

        drive.updatePoseEstimate();
        Pose2d pose = drive.getPose();
        double heading = pose.heading.toDouble();

        double errX = target.position.x - pose.position.x;                 // field frame
        double errY = target.position.y - pose.position.y;                 // field frame
        double errH = Rotation2d.normalize(target.heading.toDouble() - heading);

        // overshoot: how far the robot has gone PAST the target along the leg's line of travel, and past the target heading in the direction it was turning
        overshootIn = Math.max(overshootIn, -(errX * travelUx + errY * travelUy));
        overshootDeg = Math.max(overshootDeg, -turnSign * Math.toDegrees(errH));

        double vxField = xCtrl.calculate(errX);
        double vyField = yCtrl.calculate(errY);
        if (!Double.isNaN(slowZoneIn) && Math.hypot(errX, errY) <= slowZoneIn) {
            double mag = Math.hypot(vxField, vyField);
            if (mag > slowZonePower) {
                vxField *= slowZonePower / mag;
                vyField *= slowZonePower / mag;
            }
        }
        double omega = hCtrl.calculate(errH);

        // field-frame translation -> robot frame (rotate by -heading)
        Vector2d robotVel = new Vector2d(vxField, vyField).rotated(-heading);

        double posError = Math.hypot(errX, errY);
        double posTol = Double.isNaN(posTolIn) ? POS_TOLERANCE_IN : posTolIn;
        double headTolRad = Math.toRadians(Double.isNaN(headTolDeg) ? HEADING_TOLERANCE_DEG : headTolDeg);
        if (stiction) {
            // the robot's speed from the poses of the last two loops (0 on the first: still), a little smoothed
            long nowNs = System.nanoTime();
            if (lastPoseNs != 0L) {
                double dt = (nowNs - lastPoseNs) / 1e9;
                if (dt > 1e-3) {
                    double wx = (pose.position.x - lastPoseX) / dt, wy = (pose.position.y - lastPoseY) / dt;
                    Vector2d vr = new Vector2d(wx, wy).rotated(-heading);
                    double w = Math.toDegrees(Rotation2d.normalize(heading - lastPoseH)) / dt;
                    vForward += 0.5 * (vr.x - vForward);
                    vLeft += 0.5 * (vr.y - vLeft);
                    wDegPerS += 0.5 * (w - wDegPerS);
                }
            }
            lastPoseNs = nowNs;
            lastPoseX = pose.position.x;
            lastPoseY = pose.position.y;
            lastPoseH = heading;
            double max = Double.isNaN(maxOutputOverride) ? MAX_OUTPUT : maxOutputOverride;
            double fwd = robotVel.x, left = robotVel.y, om = omega;
            if (posError >= posTol) {
                Vector2d err = new Vector2d(errX, errY).rotated(-heading);   // the position error along the robot's own axes
                if (Math.abs(err.x) >= KS_MIN_ERR_IN) fwd = withKs(fwd, KS_FORWARD, KS_FORWARD_START, Math.abs(vForward) < STALL_IN_PER_S, max);
                if (Math.abs(err.y) >= KS_MIN_ERR_IN) left = withKs(left, KS_STRAFE, KS_STRAFE_START, Math.abs(vLeft) < STALL_IN_PER_S, max);
            }
            if (Math.abs(errH) >= headTolRad) om = withKs(om, KS_TURN, KS_TURN_START, Math.abs(wDegPerS) < STALL_DEG_PER_S, max);
            robotVel = new Vector2d(fwd, left);
            omega = om;
        }
        drive.setDrivePowers(new PoseVelocity2d(robotVel, omega));

        boolean inTolerance = posError < posTol && Math.abs(errH) < headTolRad;
        if (!inTolerance) {
            settle.reset();
        }
        boolean done = (inTolerance && settle.seconds() >= (Double.isNaN(settleSecOverride) ? SETTLE_SEC : settleSecOverride))
                || timeout.seconds() >= timeoutSec;

        packet.put("dtp/errX_in", errX);
        packet.put("dtp/errY_in", errY);
        packet.put("dtp/errH_deg", Math.toDegrees(errH));
        packet.put("dtp/posError_in", posError);
        packet.put("dtp/x_in", pose.position.x);
        packet.put("dtp/y_in", pose.position.y);
        packet.put("dtp/h_deg", pose.heading.toDegrees());
        packet.put("dtp/targetX_in", target.position.x);
        packet.put("dtp/targetY_in", target.position.y);
        packet.put("dtp/targetH_deg", target.heading.toDegrees());

        if (done) {
            drive.stop();
            finished = true;
            settled = inTolerance;
            elapsedSec = timeout.seconds();
            finalErrIn = posError;
            finalErrHeadingDeg = Math.toDegrees(errH);
            RunLog.event(String.format("DRIVE %s after %.2f s: error %.2f in (x %+.2f, y %+.2f), heading %+.1f deg, overshoot %.2f in / %.1f deg",
                    inTolerance ? "settled" : "TIMED OUT", elapsedSec, posError, errX, errY, finalErrHeadingDeg, overshootIn, overshootDeg));
        }
        return !done;
    }

    /** c with the static friction term added in its direction (ks while it moves, ksStart while it is still), limited to max; a zero command stays zero. */
    private static double withKs(double c, double ks, double ksStart, boolean still, double max) {
        if (Math.abs(c) < 1e-4) return c;
        double v = c + Math.signum(c) * (still ? ksStart : ks);
        return Math.max(-max, Math.min(max, v));
    }

    /** True once the leg has ended, settled or timed out. The getters below are final then. */
    public boolean isFinished() {
        return finished;
    }

    /** True if the leg ended inside its tolerance for the settle time; false if it ran to its time cap. */
    public boolean isSettled() {
        return settled;
    }

    /** Seconds from the leg's first loop to its end. */
    public double getElapsedSec() {
        return elapsedSec;
    }

    /** The position error, inches, when the leg ended. */
    public double getFinalErrorIn() {
        return finalErrIn;
    }

    /** The heading error, degrees (target minus heading), when the leg ended. */
    public double getFinalHeadingErrorDeg() {
        return finalErrHeadingDeg;
    }

    /** The furthest the robot went past the target along the leg's line of travel, inches (0 for a turn in place, or a leg that never passed it). */
    public double getOvershootIn() {
        return overshootIn;
    }

    /** The furthest the robot turned past the target heading, degrees (0 for a leg with no turn). */
    public double getOvershootDeg() {
        return overshootDeg;
    }

    /** The gains and limits in force, as one line ready to paste into this class (the tuners and Drive Acceptance log it: a Dashboard edit is lost when the app restarts). */
    public static String gainsText() {
        return String.format(java.util.Locale.US, "AXIAL_kP = %s, AXIAL_kI = %s, AXIAL_kD = %s; LATERAL_kP = %s, LATERAL_kI = %s, LATERAL_kD = %s; HEADING_kP = %s, HEADING_kI = %s, HEADING_kD = %s; "
                        + "MAX_OUTPUT = %s; KS_FORWARD = %s, KS_STRAFE = %s, KS_TURN = %s; KS_FORWARD_START = %s, KS_STRAFE_START = %s, KS_TURN_START = %s; "
                        + "POS_TOLERANCE_IN = %s, HEADING_TOLERANCE_DEG = %s, SETTLE_SEC = %s, TIMEOUT_SEC = %s",
                AXIAL_kP, AXIAL_kI, AXIAL_kD, LATERAL_kP, LATERAL_kI, LATERAL_kD, HEADING_kP, HEADING_kI, HEADING_kD, MAX_OUTPUT,
                KS_FORWARD, KS_STRAFE, KS_TURN, KS_FORWARD_START, KS_STRAFE_START, KS_TURN_START, POS_TOLERANCE_IN, HEADING_TOLERANCE_DEG, SETTLE_SEC, TIMEOUT_SEC);
    }

    private void applyGains() {
        xCtrl.kP = AXIAL_kP;   xCtrl.kI = AXIAL_kI;   xCtrl.kD = AXIAL_kD;   xCtrl.kStatic = AXIAL_kS;
        yCtrl.kP = LATERAL_kP; yCtrl.kI = LATERAL_kI; yCtrl.kD = LATERAL_kD; yCtrl.kStatic = LATERAL_kS;
        hCtrl.kP = HEADING_kP; hCtrl.kI = HEADING_kI; hCtrl.kD = HEADING_kD; hCtrl.kStatic = HEADING_kS;
        for (PidController c : new PidController[]{xCtrl, yCtrl, hCtrl}) {
            c.maxOutput = Double.isNaN(maxOutputOverride) ? MAX_OUTPUT : maxOutputOverride;
            c.maxIntegral = MAX_INTEGRAL;
        }
    }
}
