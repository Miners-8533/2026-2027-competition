package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Jostle (specs/motion.md section 5): shake a flower so it gives up its pollen. A sustained push works where a single
 * ram does not. Every stroke is measured from the contact point (where the robot is when the jostle starts, pressed
 * against the flower), so the robot cannot creep. All motion is along the robot's own forward axis.
 */
@Config
public final class Jostle implements Action {
    public static double START_HOLD_S = 1.0;
    public static double FORWARD_POWER = 0.5;
    public static double BACK_DISTANCE_IN = 0.5, BACK_POWER = 0.4, BACK_TIME_LIMIT_S = 0.3;
    public static double PUSH_MIN_S = 0.2, PUSH_MAX_S = 0.6;
    public static double PRESS_HOLD_S = 1.0;
    /** 0 = no limit. */
    public static int CYCLE_LIMIT = 1;
    public static double CONTACT_TOL_IN = 0.3;
    /** Early stop: enough held and nothing new for a while. 5 = never, since a robot holds 4. */
    public static int EARLY_HELD = 5;
    public static double EARLY_QUIET_S = 0.8;
    public static double HEADING_KP = 1.5, TURN_LIMIT = 0.15;

    /** How many elements are held, or -1 when no count is read. */
    public interface Held { int get(); }

    private enum Phase { HOLD, BACK, PUSH, PRESS }

    private final MecanumDrive drive;
    private final Actions.Condition done;
    private final Held held;
    private final double timeLimit;

    private Phase phase = Phase.HOLD;
    private Pose contact;
    private double startT = -1, phaseStart, lastNewAt;
    private int cycles, lastHeld = -1;

    /**
     * @param done      the caller's "done" condition (the robot is full, or its deadline has passed)
     * @param timeLimit seconds, counted from the jostle's start
     */
    public Jostle(MecanumDrive drive, Actions.Condition done, Held held, double timeLimit) {
        this.drive = drive;
        this.done = done;
        this.held = held;
        this.timeLimit = timeLimit;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        double t = Clock.now();
        Pose p = drive.pose();
        if (startT < 0) {
            startT = phaseStart = lastNewAt = t;
            contact = p;
            RunLog.eventf("AUTO JOSTLE start at %s, time limit %.1f s", p, timeLimit);
        }
        // Distance behind the contact point, along the robot's forward axis at contact.
        double behind = -((p.x - contact.x) * Math.cos(contact.h) + (p.y - contact.y) * Math.sin(contact.h));
        int h = held.get();
        if (h != lastHeld) {
            if (h > lastHeld) lastNewAt = t;
            lastHeld = h;
        }
        String why = done.get() ? "the caller's condition (full or deadline)"
                : (h >= EARLY_HELD && t - lastNewAt >= EARLY_QUIET_S) ? "enough held and nothing new"
                : t - startT >= timeLimit ? "time limit" : null;
        double el = t - phaseStart;
        double forward;
        switch (phase) {
            case HOLD:
                forward = FORWARD_POWER;
                if (el >= START_HOLD_S) next(Phase.BACK, t);
                break;
            case BACK:
                forward = -BACK_POWER;
                if (behind >= BACK_DISTANCE_IN || el >= BACK_TIME_LIMIT_S) {
                    RunLog.eventf("AUTO JOSTLE back stroke: power %.2f, %.2f s, reached %.2f in, ended by %s", BACK_POWER, el, behind,
                            behind >= BACK_DISTANCE_IN ? "distance" : "the time limit");
                    next(Phase.PUSH, t);
                }
                break;
            case PUSH:
                forward = FORWARD_POWER;
                if ((el >= PUSH_MIN_S && behind <= CONTACT_TOL_IN) || el >= PUSH_MAX_S) {
                    RunLog.eventf("AUTO JOSTLE push: %.2f s, %.2f in behind the contact point", el, behind);
                    next(Phase.PRESS, t);
                }
                break;
            default:
                forward = FORWARD_POWER;
                if (el >= PRESS_HOLD_S) {
                    cycles++;
                    if (CYCLE_LIMIT > 0 && cycles >= CYCLE_LIMIT) why = why == null ? "cycle limit (" + cycles + ")" : why;
                    else next(Phase.BACK, t);
                }
        }
        if (why != null) {
            drive.stop();
            RunLog.eventf("AUTO JOSTLE ended after %.1f s, %d cycle(s), held %s: %s", t - startT, cycles, h < 0 ? "?" : String.valueOf(h), why);
            return false;
        }
        // The heading is held at its starting value with a small turn limit.
        drive.drive(forward, 0, Angle.clampAbs(HEADING_KP * Angle.diff(contact.h, p.h), TURN_LIMIT));
        return true;
    }

    private void next(Phase ph, double t) {
        phase = ph;
        phaseStart = t;
    }
}
