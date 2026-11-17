package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Part of the coordinator (specs/mechanisms.md section 3): a launch is queued, started, ended or dropped. Pure logic.
 * The band is checked once: a salvo that has started is never interrupted by the flywheel speed.
 */
public final class LaunchSequencer {
    public enum State { IDLE, QUEUED, SALVO }

    /** What happened this update. */
    public enum Event { NONE, START, DROPPED, DONE, INCOMPLETE }

    private State state = State.IDLE;
    private double queuedAt, startedAt, lastBusyAt = -1e9, minSpeed;
    private boolean bandLogged;
    private int salvoCount;

    /** Why a launch was dropped, or which way the speed is off the band (for the log, once per queued launch). */
    public String detail;

    public State state() { return state; }
    public boolean queued() { return state == State.QUEUED; }
    public boolean salvo() { return state == State.SALVO; }
    public boolean busy() { return state != State.IDLE; }
    /** Finished salvos. The TeleOp watches it to release the target lock. */
    public int salvoCount() { return salvoCount; }
    public double minSpeed() { return minSpeed; }
    public double salvoSeconds(double t) { return t - startedAt; }

    /** While a launch is queued or a salvo runs, and for a moment after, the run log writes a row on every loop. */
    public boolean burst(double t) {
        return busy() || t - lastBusyAt < MechParams.BURST_AFTER_SALVO_S;
    }

    public void queue(double t) {
        state = State.QUEUED;
        queuedAt = t;
        bandLogged = false;
    }

    /** A reject cancels a queued launch or a running salvo. */
    public void cancel(double t) {
        if (busy()) lastBusyAt = t;
        state = State.IDLE;
    }

    /**
     * @param inBand        the flywheel is in the launch band (filtered)
     * @param pusherStopped the pusher is idle and not moving, wherever it rests
     * @param flyPosInPlace the flywheel position servo has finished travelling
     * @param pusherFailed  the pusher gave up on its last move, or faulted
     */
    public Event step(double t, boolean inBand, boolean pusherStopped, boolean flyPosInPlace, boolean pusherFailed, double filteredSpeed, double setpoint) {
        detail = null;
        if (state == State.QUEUED) {
            if (inBand && pusherStopped && flyPosInPlace) {
                state = State.SALVO;
                startedAt = t;
                minSpeed = filteredSpeed;
                return Event.START;
            }
            if (t - queuedAt > MechParams.LAUNCH_TIMEOUT_S) {
                state = State.IDLE;
                lastBusyAt = t;
                detail = !inBand ? "the flywheel is not in the launch band (" + Math.round(filteredSpeed) + " for " + Math.round(setpoint) + ")"
                        : !pusherStopped ? "the pusher is not stopped" : "the flywheel position servo is still travelling";
                return Event.DROPPED;
            }
            if (!inBand && !bandLogged) {
                bandLogged = true;
                detail = "waiting on the band: the speed is " + (filteredSpeed > setpoint ? "above" : "below") + " it ("
                        + Math.round(filteredSpeed) + " for " + Math.round(setpoint) + ")";
            }
            return Event.NONE;
        }
        if (state == State.SALVO) {
            minSpeed = Math.min(minSpeed, filteredSpeed);
            if (pusherStopped || pusherFailed) {
                state = State.IDLE;
                lastBusyAt = t;
                if (pusherFailed) return Event.INCOMPLETE;
                salvoCount++;
                return Event.DONE;
            }
        }
        return Event.NONE;
    }
}
