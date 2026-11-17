package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * The pusher check (specs/pusher.md section 7): verify, and if needed correct, the pusher's zero using CS4, which sees
 * the arm once per turn. It catches a zero that was assumed wrongly at power-up and any belt slip.
 *
 * It drives the pusher only through seeks and the hold command, and switches the settle nudge off while it runs.
 */
@Config
public final class PusherCheck {
    public static double ARM_SEEN_DEG = 326.7;
    public static int PULLEY_TEETH = 20;
    public static double SETTLE_S = 0.25;
    public static int FRESH_READS = 3;
    public static double LOOK_TIMEOUT_S = 1.5;
    public static double SEEK_COAST_DEG = 4.4;
    public static double SWEEP_POWER = 0.15, SWEEP_DEG = 420.0, BACK_DEG = 90.0;
    public static int MAX_SWEEPS = 3;
    public static double SENSOR_DELAY_S = 0.15;
    public static double TOTAL_LIMIT_S = 25.0;

    public enum Result { NONE, RUNNING, OK, SWEEP, LOST, ABORTED }

    private enum Stage { IDLE, PARK, LOOK, SWEEP, CROSS_BACK, RETURN }

    private final Pusher pusher;
    private final SlotCounter slots;

    private Stage stage = Stage.IDLE;
    private Result result = Result.NONE;
    private boolean lostBefore;
    private double startedAt, stageAt, stillSince, totalCorrection;
    private int readsAtSettle, sweeps;
    // Crossing tracking: not seen, then seen, then not seen again.
    private boolean wasSeen, crossed;
    private double enterDeg, centerFwd, centerBack, fwdSpeed;
    private boolean haveFwd, haveBack;
    private boolean extended;

    public PusherCheck(Pusher pusher, SlotCounter slots) {
        this.pusher = pusher;
        this.slots = slots;
    }

    public Result result() { return result; }
    public boolean running() { return stage != Stage.IDLE; }
    /** While sweeping, the arm passes every sensor: none is counted and CS4 is read on every loop. */
    public boolean sweeping() { return stage == Stage.SWEEP || stage == Stage.CROSS_BACK; }

    /**
     * Starts a check. The caller has made sure the arm is idle and stopped and the counter reads all four sensors;
     * for a calibration sweep, also that the track is empty.
     *
     * @param calibrationSweep skip the park and the look and go straight to the sweep
     */
    public void start(boolean calibrationSweep, double t) {
        startedAt = t;
        sweeps = 0;
        totalCorrection = 0;
        result = Result.RUNNING;
        pusher.setSettleNudge(false);
        RunLog.event("PUSHER CHECK started" + (calibrationSweep ? " (calibration sweep)" : ""));
        if (calibrationSweep) beginSweep(t);
        else beginPark(t);
    }

    /** Something else needs the pusher, or the counter stopped reading: stop, send the arm forward to HOLD. A previous LOST stays LOST. */
    public void abort(String why, double t) {
        if (!running()) return;
        pusher.abortSeek();
        pusher.hold();
        end(Result.ABORTED, "aborted: " + why, t);
    }

    private void end(Result r, String text, double t) {
        result = (lostBefore && r == Result.ABORTED) ? Result.LOST : r;
        if (r == Result.LOST) lostBefore = true;
        if (r == Result.OK || r == Result.SWEEP) {
            lostBefore = false;
            pusher.setZeroVerified(true);
        }
        stage = Stage.IDLE;
        pusher.setSettleNudge(true);
        RunLog.eventf("PUSHER CHECK %s after %.1f s: %s", result, t - startedAt, text);
    }

    private void beginPark(double t) {
        // The angle where CS4 should see the arm's center, nearest to where the arm is. The target is moved back toward
        // where the arm comes from, because a seek coasts about that far past its target.
        double a = pusher.angle();
        double park = a + Angle.wrapDeg(ARM_SEEN_DEG - a);
        double target = park + (a > park ? SEEK_COAST_DEG : -SEEK_COAST_DEG);
        pusher.seek(target, Pusher.SEEK_POWER);
        stage = Stage.PARK;
        stageAt = t;
    }

    private void beginSweep(double t) {
        sweeps++;
        wasSeen = slots.cs4SeesArm();
        crossed = false;
        haveFwd = haveBack = false;
        extended = false;
        enterDeg = Double.NaN;
        pusher.seek(pusher.angle() + SWEEP_DEG, SWEEP_POWER);
        stage = Stage.SWEEP;
        stageAt = t;
    }

    public void update(double t) {
        if (stage == Stage.IDLE) return;
        if (t - startedAt > TOTAL_LIMIT_S) {
            pusher.abortSeek();
            end(Result.LOST, "time limit", t);
            return;
        }
        if (pusher.faulted() || pusher.gaveUp()) {
            end(Result.LOST, "the pusher could not finish a move", t);
            return;
        }
        switch (stage) {
            case PARK:
                if (pusher.stopped()) {
                    stage = Stage.LOOK;
                    stageAt = t;
                    stillSince = t;
                    readsAtSettle = -1;
                }
                break;
            case LOOK: {
                if (!pusher.stopped()) stillSince = t;
                boolean settled = t - stillSince >= SETTLE_S;
                if (settled && readsAtSettle < 0) readsAtSettle = slots.cs4Reads();
                boolean fresh = settled && slots.cs4Reads() - readsAtSettle >= FRESH_READS;
                if (!fresh && t - stageAt < LOOK_TIMEOUT_S + SETTLE_S) break;
                if (slots.cs4SeesArm()) {
                    // The zero is right (or has just been corrected and is now verified).
                    pusher.hold();
                    stage = Stage.RETURN;
                } else if (sweeps < MAX_SWEEPS) {
                    beginSweep(t);
                } else {
                    end(Result.LOST, "the arm is not seen where the zero says after " + sweeps + " sweeps", t);
                }
                break;
            }
            case SWEEP: {
                boolean seen = slots.cs4SeesArm();
                if (seen && !wasSeen) enterDeg = pusher.angle();
                if (!seen && wasSeen && !Double.isNaN(enterDeg)) {
                    centerFwd = (enterDeg + pusher.angle()) / 2;
                    fwdSpeed = pusher.speedDeg();
                    haveFwd = true;
                    crossed = true;
                }
                wasSeen = seen;
                if (crossed) {
                    // Cross back the other way: the sensor reports the arm late, which shifts each crossing in its direction of travel.
                    pusher.abortSeek();
                    pusher.seek(pusher.angle() - BACK_DEG, SWEEP_POWER);
                    stage = Stage.CROSS_BACK;
                    stageAt = t;
                    wasSeen = seen;
                    enterDeg = Double.NaN;
                } else if (pusher.stopped() && t - stageAt > 0.3) {
                    if (seen && !extended) {
                        // The sweep ended with the arm still in view: go once more for a whole crossing.
                        extended = true;
                        pusher.seek(pusher.angle() + 120.0, SWEEP_POWER);
                        stageAt = t;
                    } else {
                        end(Result.LOST, "the sweep never saw the arm cross CS4", t);
                    }
                }
                break;
            }
            case CROSS_BACK: {
                boolean seen = slots.cs4SeesArm();
                if (seen && !wasSeen) enterDeg = pusher.angle();
                if (!seen && wasSeen && !Double.isNaN(enterDeg)) {
                    centerBack = (enterDeg + pusher.angle()) / 2;
                    haveBack = true;
                }
                wasSeen = seen;
                if (pusher.stopped() && t - stageAt > 0.3) {
                    // The average of the two crossing centers is the true position with no delay constant.
                    double center = haveBack ? (centerFwd + centerBack) / 2 : centerFwd - fwdSpeed * SENSOR_DELAY_S;
                    double delta = Angle.wrapDeg(ARM_SEEN_DEG - center);
                    pusher.shiftZero(delta);
                    totalCorrection += delta;
                    RunLog.eventf("PUSHER CHECK crossing: forward center %.1f, back center %s, zero shifted %.1f deg (%.2f teeth)",
                            centerFwd, haveBack ? RunLog.fmt(centerBack, 1) : "not seen", delta, delta / (360.0 / PULLEY_TEETH));
                    beginPark(t);
                }
                break;
            }
            case RETURN:
                if (pusher.stopped()) {
                    if (sweeps == 0) end(Result.OK, "the arm is where the zero says", t);
                    else end(Result.SWEEP, String.format(java.util.Locale.US, "zero corrected by %.1f deg (%.2f teeth) in %d sweep(s)",
                            totalCorrection, totalCorrection / (360.0 / PULLEY_TEETH), sweeps), t);
                }
                break;
            default:
                break;
        }
    }
}
