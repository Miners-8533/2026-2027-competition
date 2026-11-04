package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.motion.PidController;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * HOLD POSITION while launching (2026-11-04, the shelved idea of 2026-11-02): from the moment a launch is requested until the salvo is done (plus a short tail), the robot holds
 * the pose it had when it had braked and settled, with a PID on the field position (and on the heading when the target lock is not turning it), so a launch is made from
 * the pose the aim and the launch speed were worked out for and a push from another robot is resisted. The driver's sticks cancel it at once, and it latches again when the sticks
 * are let go and the launch is still going. It uses the relative error from the latched pose, so the Pinpoint's absolute errors do not matter.
 *
 * It must resist being shoved: there is no give-up cap on the error (user, 2026-11-02). The only guard is SANITY_IN, an error so large (a reseed or a pose jump, not a push) that
 * holding would drive the robot across the field. The motors run at up to MAX_OUTPUT, so watch the log's HOLD lines (peak error, peak command) and the motor heat.
 */
@Config
public final class HoldPosition {

    /** False turns the feature off. */
    public static boolean ENABLED = true;
    /** The pose is latched once the chassis has been slower than this (in/s) for LATCH_STILL_SEC. */
    public static double LATCH_SPEED_IN_PER_S = 3.0;
    public static double LATCH_STILL_SEC = 0.12;
    /** The hold goes on this long after the launch is done (so the last pollen is clear), unless the driver cancels it. */
    public static double TAIL_SEC = 0.2;
    /** Position gains on the field-frame error, power per inch (the DriveToPose values), an integral to take out a steady offset, and a static term outside the dead band. */
    public static double KP = 0.06, KI = 0.03, KD = 0.006, KS = 0.04;
    public static double MAX_INTEGRAL = 0.2;
    public static double DEADBAND_IN = 0.4;
    public static double MAX_OUTPUT = 0.7;
    /** Heading gains (only while the target lock is not turning the robot). */
    public static double HEADING_KP = 1.5, HEADING_KD = 0.08, HEADING_DEADBAND_DEG = 1.0, HEADING_MAX_TURN = 0.4;
    /** An error beyond this many inches is a pose jump, not a push: the hold lets go (and logs it). */
    public static double SANITY_IN = 30.0;

    /** What the hold wants done this loop. */
    public static final class Output {
        public boolean active;
        public double forward, left;
        /** The hold also turns the chassis (the target lock is not). */
        public boolean turnActive;
        public double turn;
    }

    private enum Phase { IDLE, WAITING, HOLDING }

    private final Output out = new Output();
    private final PidController xCtrl = new PidController(KP, KI, KD);
    private final PidController yCtrl = new PidController(KP, KI, KD);
    private final PidController hCtrl = new PidController(HEADING_KP, 0.0, HEADING_KD);
    private final ElapsedTime still = new ElapsedTime();
    private final ElapsedTime since = new ElapsedTime();
    private final ElapsedTime tail = new ElapsedTime();
    private Phase phase = Phase.IDLE;
    private double holdX, holdY, holdHeading;
    private boolean tailing = false;
    private double peakErrorIn, peakCommand;

    public boolean isHolding() {
        return phase == Phase.HOLDING;
    }

    /** "idle", "waiting to settle" or "HOLDING (error x in)", for the telemetry. */
    public String status() {
        switch (phase) {
            case WAITING: return "waiting for the robot to settle";
            case HOLDING: return String.format(Locale.US, "HOLDING (peak error %.1f in)", peakErrorIn);
            default: return ENABLED ? "idle" : "off";
        }
    }

    private void release(String why, Pose2d pose) {
        if (phase == Phase.HOLDING) {
            RunLog.event(String.format(Locale.US, "HOLD released (%s) after %.2f s at (%.1f, %.1f, %.1f deg); peak error %.1f in, peak command %.2f",
                    why, since.seconds(), pose.position.x, pose.position.y, pose.heading.toDegrees(), peakErrorIn, peakCommand));
        }
        phase = Phase.IDLE;
        tailing = false;
    }

    /**
     * Call once per loop with the loop's pose. wanted: a launch is queued or a salvo is running (and nothing else is driving); cancel: the driver is moving a stick (or the
     * dash is driving); headingFree: the target lock is not turning the robot, so the hold holds the heading too.
     */
    public Output update(Pose2d pose, PoseVelocity2d measured, boolean wanted, boolean cancel, boolean headingFree) {
        out.active = false;
        out.turnActive = false;
        out.forward = out.left = out.turn = 0.0;
        if (!ENABLED || cancel) {
            release(cancel ? "the driver moved" : "disabled", pose);
            if (phase != Phase.IDLE) phase = Phase.IDLE;
            // a held-over WAITING state is cleared too, so the next launch starts clean
            return out;
        }
        if (wanted) {
            tailing = false;
        } else if (phase == Phase.HOLDING && !tailing) {
            tailing = true;
            tail.reset();
        }
        boolean keep = wanted || (phase == Phase.HOLDING && tailing && tail.seconds() < TAIL_SEC);
        if (!keep) {
            release(wanted ? "-" : "the launch is done", pose);
            phase = Phase.IDLE;
            return out;
        }

        double speed = measured == null ? 0.0 : Math.hypot(measured.linearVel.x, measured.linearVel.y);
        if (phase == Phase.IDLE) {
            phase = Phase.WAITING;
            still.reset();
        }
        if (phase == Phase.WAITING) {
            if (speed > LATCH_SPEED_IN_PER_S) still.reset();
            if (still.seconds() < LATCH_STILL_SEC) return out;
            holdX = pose.position.x;
            holdY = pose.position.y;
            holdHeading = pose.heading.toDouble();
            xCtrl.reset();
            yCtrl.reset();
            hCtrl.reset();
            since.reset();
            peakErrorIn = 0.0;
            peakCommand = 0.0;
            phase = Phase.HOLDING;
            RunLog.event(String.format(Locale.US, "HOLD latched at (%.1f, %.1f, %.1f deg) after the robot settled (%.1f in/s)", holdX, holdY, Math.toDegrees(holdHeading), speed));
        }

        double errX = holdX - pose.position.x, errY = holdY - pose.position.y;
        double err = Math.hypot(errX, errY);
        if (err > SANITY_IN) {
            RunLog.event(String.format(Locale.US, "HOLD dropped: %.0f in from the latched pose (a pose jump, not a push)", err));
            release("pose jump", pose);
            return out;
        }
        peakErrorIn = Math.max(peakErrorIn, err);
        for (PidController c : new PidController[]{xCtrl, yCtrl}) {
            c.kP = KP;
            c.kI = KI;
            c.kD = KD;
            c.maxIntegral = MAX_INTEGRAL;
            c.maxOutput = MAX_OUTPUT;
        }
        double vx = 0.0, vy = 0.0;
        if (err > DEADBAND_IN) {
            vx = xCtrl.calculate(errX);
            vy = yCtrl.calculate(errY);
            double mag = Math.hypot(vx, vy);
            if (mag > 1e-6) {
                double boost = Math.min(MAX_OUTPUT, mag + KS) / mag;   // the static term along the push direction
                vx *= boost;
                vy *= boost;
            }
        } else {
            xCtrl.reset();
            yCtrl.reset();
        }
        Vector2d robot = new Vector2d(vx, vy).rotated(-pose.heading.toDouble());
        out.active = true;
        out.forward = robot.x;
        out.left = robot.y;
        peakCommand = Math.max(peakCommand, Math.hypot(vx, vy));
        if (headingFree) {
            double errH = Rotation2d.normalize(holdHeading - pose.heading.toDouble());
            if (Math.abs(Math.toDegrees(errH)) > HEADING_DEADBAND_DEG) {
                hCtrl.kP = HEADING_KP;
                hCtrl.kD = HEADING_KD;
                hCtrl.maxOutput = HEADING_MAX_TURN;
                out.turn = hCtrl.calculate(errH);
                out.turnActive = true;
            } else {
                hCtrl.reset();
            }
        }
        return out;
    }
}
