package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Part of the coordinator (specs/mechanisms.md section 3): a launch is queued, started, ended or dropped. Pure logic.
 * The band is checked once: a salvo that has started is never interrupted by the flywheel speed.
 *
 * The coordinator calls it at two places of its update (section 16.3): the end of a salvo with the pusher's events
 * (step 7), and the queued launch after the pusher check (step 9).
 */
public final class LaunchSequencer {
    public enum State { IDLE, QUEUED, SALVO }

    /** What happened to the queued launch this update. */
    public enum Event { NONE, START, DROPPED }

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
    /** Salvos that ended as a salvo. The TeleOp watches it to release the target lock. */
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

    /** Drops whatever is queued or running without counting a salvo: a reject, a pusher fault, the end of the opmode. */
    public void cancel(double t) {
        if (busy()) lastBusyAt = t;
        state = State.IDLE;
    }

    /**
     * The queued launch: dropped on its timeout, otherwise started when all its conditions hold.
     *
     * @param inBand        the flywheel is in the launch band (filtered)
     * @param pusherStopped the pusher is idle and not moving, wherever it rests
     * @param flyPosInPlace the flywheel position servo has finished travelling
     */
    public Event queuedStep(double t, boolean inBand, boolean pusherStopped, boolean flyPosInPlace, double filteredSpeed, double setpoint) {
        detail = null;
        if (state != State.QUEUED) return Event.NONE;
        if (t - queuedAt > MechParams.LAUNCH_TIMEOUT_S) {
            state = State.IDLE;
            lastBusyAt = t;
            detail = !inBand ? "the flywheel is not in the launch band (" + Math.round(filteredSpeed) + " for " + Math.round(setpoint) + ")"
                    : !pusherStopped ? "the pusher is not stopped" : "the flywheel position servo is still travelling";
            return Event.DROPPED;
        }
        if (inBand && pusherStopped && flyPosInPlace) {
            state = State.SALVO;
            startedAt = t;
            minSpeed = filteredSpeed;
            return Event.START;
        }
        if (!inBand && !bandLogged) {
            bandLogged = true;
            detail = "waiting on the band: the speed is " + (filteredSpeed > setpoint ? "above" : "below") + " it ("
                    + Math.round(filteredSpeed) + " for " + Math.round(setpoint) + ")";
        }
        return Event.NONE;
    }

    /** While a salvo runs: the lowest flywheel speed, for the log line at its end. */
    public void trackSpeed(double filteredSpeed) {
        if (state == State.SALVO) minSpeed = Math.min(minSpeed, filteredSpeed);
    }

    /**
     * The pusher has stopped again, or gave up: the salvo ends as a salvo and the salvo counter increases.
     * (A pusher fault does not come here: it drops the salvo with cancel.)
     */
    public void salvoEnded(double t) {
        if (state != State.SALVO) return;
        state = State.IDLE;
        lastBusyAt = t;
        salvoCount++;
    }
}
