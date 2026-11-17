package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Part of the coordinator (specs/mechanisms.md section 6): when the automatic pusher check is wanted, started,
 * skipped or aborted. Pure logic.
 */
public final class CheckScheduler {
    public enum Decision { NONE, START, SKIP, ABORT }

    private boolean wanted;
    private double wantedAt;
    /** Why the check was skipped or aborted. */
    public String why;

    /** After every finished salvo, and once after the first update if the pusher's zero has not been verified. */
    public void want(double t) {
        wanted = true;
        wantedAt = t;
    }

    public boolean wanted() { return wanted; }

    public void clear() { wanted = false; }

    /**
     * @param busy          a launch queued, a salvo, a reject or the manual outtake
     * @param slotsReading  the CS1 to CS4 counter is valid
     * @param slotCount     its count
     * @param pusherStopped the pusher is idle and still
     * @param pusherGaveUp  the pusher gave up on its last move
     * @param locked        the pusher lock is not "none"
     */
    public Decision step(double t, boolean running, boolean busy, boolean slotsReading, int slotCount, boolean pusherStopped, boolean pusherGaveUp, boolean locked) {
        why = null;
        if (running) {
            // Anything busy, or an element appearing in the count, aborts it.
            if (busy) why = "something else needs the pusher";
            else if (!slotsReading) why = "the counter stopped reading";
            else if (slotCount > 0) why = "an element appeared in the count";
            return why == null ? Decision.NONE : Decision.ABORT;
        }
        if (!wanted) return Decision.NONE;
        if (pusherGaveUp) {
            wanted = false;
            why = "the pusher gave up on its last move";
            return Decision.SKIP;
        }
        String blocked = busy ? "busy" : !slotsReading ? "the counter is not reading" : slotCount > 0 ? "the track does not read empty (" + slotCount + ")"
                : !pusherStopped ? "the pusher is not stopped" : locked ? "the pusher is locked" : null;
        if (blocked == null && t - wantedAt >= MechParams.CHECK_SETTLE_S) {
            wanted = false;
            return Decision.START;
        }
        if (t - wantedAt > MechParams.CHECK_WAIT_S) {
            wanted = false;
            why = blocked == null ? "not settled" : blocked;
            return Decision.SKIP;
        }
        return Decision.NONE;
    }
}
