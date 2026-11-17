package org.firstinspires.ftc.teamcode.navigation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pid;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Hold position (specs/motion.md section 6): keep the robot where it is while it launches, and resist a push.
 * A per-loop helper for the TeleOp. Pure logic. It works on the error from the latched pose, so absolute odometry
 * error does not matter. There is no give-up limit on the error: it must fight being shoved.
 */
@Config
public final class HoldPosition {
    public static double LATCH_SPEED_IN_S = 3.0;
    public static double LATCH_TIME_S = 0.12;
    public static double TAIL_S = 0.2;
    public static double KP = 0.06, KI = 0.03, KD = 0.006, STATIC = 0.04;
    /** Most power the integral adds, per axis. */
    public static double I_LIMIT = 0.2;
    public static double DEAD_BAND_IN = 0.4;
    public static double MAX_POWER = 0.7;
    public static double HEADING_KP = 1.5, HEADING_KD = 0.08, HEADING_DEAD_BAND_DEG = 1.0, TURN_LIMIT = 0.4;
    /** An error this large is a pose jump or a reseed, not a push. */
    public static double SANITY_IN = 30.0;
    public static double BRAKE_GAIN = 0.02, BRAKE_MAX = 0.6;

    public enum State { IDLE, WAITING, HOLDING }

    private State state = State.IDLE;
    private Pose latched;
    private final Pid px = new Pid(), py = new Pid();
    private double slowSince = -1, notWantedSince = -1, latchedAt, peakError, peakCommand;

    /** Outputs of the last update. */
    public boolean active, hasTurn;
    public double forward, left, turn;

    public State state() { return state; }

    /**
     * @param wanted      a launch is queued or a salvo is running, or a Park Dash is at its corner
     * @param cancel      the driver is moving a stick, or something else is driving
     * @param headingFree the target lock is not turning the robot
     * @param brake       brake the robot while it waits to latch
     */
    public void update(Pose pose, double vx, double vy, double omega, boolean wanted, boolean cancel, boolean headingFree, boolean brake, double t, double dt) {
        active = false;
        hasTurn = false;
        forward = left = turn = 0;
        double speed = Math.hypot(vx, vy);

        if (cancel) {
            // Released at once. If the launch is still going when the sticks are let go, it waits and latches again.
            if (state == State.HOLDING) release("cancelled", t);
            state = wanted ? State.WAITING : State.IDLE;
            slowSince = -1;
            return;
        }
        switch (state) {
            case IDLE:
                if (!wanted) return;
                state = State.WAITING;
                slowSince = -1;
                // fall through
            case WAITING:
                if (!wanted) {
                    state = State.IDLE;
                    return;
                }
                if (speed < LATCH_SPEED_IN_S) {
                    if (slowSince < 0) slowSince = t;
                    if (t - slowSince >= LATCH_TIME_S) {
                        state = State.HOLDING;
                        latched = pose;
                        latchedAt = t;
                        notWantedSince = -1;
                        peakError = 0;
                        peakCommand = 0;
                        px.reset();
                        py.reset();
                        RunLog.event("HOLD latched at " + pose);
                    }
                } else {
                    slowSince = -1;
                    if (brake) {
                        // Opposite to the measured velocity, proportional to it: removes the coast when a salvo is fired while driving.
                        double k = Math.min(BRAKE_MAX, BRAKE_GAIN * speed) / speed;
                        toRobot(-vx * k, -vy * k, pose.h);
                        active = true;
                    }
                }
                return;
            case HOLDING:
            default:
                if (wanted) {
                    notWantedSince = -1;
                } else {
                    if (notWantedSince < 0) notWantedSince = t;
                    if (t - notWantedSince > TAIL_S) {
                        release("done", t);
                        state = State.IDLE;
                        return;
                    }
                }
                double ex = latched.x - pose.x, ey = latched.y - pose.y, err = Math.hypot(ex, ey);
                if (err > SANITY_IN) {
                    RunLog.eventf("HOLD dropped: error %.1f in is a pose jump, not a push", err);
                    state = State.IDLE;
                    return;
                }
                double cx = px.update(ex, dt, KP, KI, KD, I_LIMIT), cy = py.update(ey, dt, KP, KI, KD, I_LIMIT);
                if (err > DEAD_BAND_IN) {
                    cx += STATIC * ex / err;
                    cy += STATIC * ey / err;
                }
                double mag = Math.hypot(cx, cy);
                if (mag > MAX_POWER) {
                    cx *= MAX_POWER / mag;
                    cy *= MAX_POWER / mag;
                    mag = MAX_POWER;
                }
                toRobot(cx, cy, pose.h);
                active = true;
                peakError = Math.max(peakError, err);
                peakCommand = Math.max(peakCommand, mag);
                if (headingFree) {
                    double eh = Angle.diff(latched.h, pose.h);
                    hasTurn = true;
                    turn = Math.abs(Math.toDegrees(eh)) <= HEADING_DEAD_BAND_DEG ? 0
                            : Angle.clampAbs(HEADING_KP * eh - HEADING_KD * omega, TURN_LIMIT);
                }
        }
    }

    /** Drops any hold without a tail (backup mode, a Pinpoint fault, a reseed). */
    public void reset() {
        if (state == State.HOLDING) release("reset", RunLog.seconds());
        state = State.IDLE;
        slowSince = -1;
        active = false;
    }

    private void release(String why, double t) {
        RunLog.eventf("HOLD released (%s) after %.2f s, peak error %.1f in, peak command %.2f", why, t - latchedAt, peakError, peakCommand);
    }

    private void toRobot(double fx, double fy, double h) {
        forward = fx * Math.cos(h) + fy * Math.sin(h);
        left = -fx * Math.sin(h) + fy * Math.cos(h);
    }
}
