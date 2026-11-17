package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Part of the coordinator (specs/mechanisms.md section 7): the FULL sequence and the pusher lock. Pure logic:
 * a snapshot in, decisions out. The coordinator carries the decisions to the pusher and the intake.
 */
public final class FullAndLock {
    public enum Lock { NONE, LOCKING, LOCKED, UNLOCKING }

    /** The snapshot. */
    public static final class In {
        public double t;
        /** 4 held by the CS5 entry count. */
        public boolean full;
        /** The opmode acts on FULL (the competition TeleOp in FULL AUTO). */
        public boolean fullActions;
        /** The operator's right trigger: the intake runs IN and a locked pusher unlocks. */
        public boolean forceIn;
        /** The second reason to lock (switched off): a target lock or dash is active and something is held. */
        public boolean aimWants;
        /** A lock may be tried: pusher and counter present, not busy, no fault or give-up, no check running or wanted, arm in a position to lock from. */
        public boolean canLock;
        public boolean pusherStopped, pusherLocked;
        public boolean launchPending;
        /** CS5's latest proximity and empty level, for the fifth-element watch. */
        public int proximity;
        public double emptyLevel;
    }

    // Decisions of the last update.
    public boolean cmdLock, cmdUnlock;
    /** The intake mode the sequence asks for, or null. */
    public Intake.Mode intake;
    /** Screen text of the sequence step, or null. */
    public String text;

    private Lock lock = Lock.NONE;
    private double fullSince = -1, notWantedSince = -1;
    private int failures;
    private boolean refused, justCommanded;

    // Fifth-element watch (variant, off).
    private double emptySince = -1, dwellSince = -1, clearSince = -1, outSince = -1, retryAt = -1;
    private boolean fifthArmed;

    public Lock lock() { return lock; }
    /** The CS1 to CS4 count is frozen while the arm sits in view of sensors. */
    public boolean countFrozen() { return lock != Lock.NONE; }
    public boolean sequenceActive() { return fullSince >= 0; }

    /** The pusher took the lock command, or refused it. */
    public void lockResult(boolean accepted) {
        if (accepted) {
            lock = Lock.LOCKING;
            justCommanded = true;
        } else {
            refused = true;
        }
    }

    /** A salvo sweeps from the lock position: the lock is released. */
    public void onSalvoStart() {
        lock = Lock.NONE;
        failures = 0;
    }

    public void onRejectDone() { failures = 0; }

    /** A reject unlocks first. Returns true if an unlock is needed; the coordinator then commands it. */
    public boolean unlockForReject() {
        if (lock == Lock.LOCKED || lock == Lock.LOCKING) {
            lock = Lock.UNLOCKING;
            justCommanded = true;
            return true;
        }
        return false;
    }

    public void update(In in) {
        cmdLock = cmdUnlock = false;
        intake = null;
        text = null;
        double t = in.t;

        if (in.full && in.fullActions) {
            if (fullSince < 0) {
                fullSince = t;
                failures = 0;      // a new request
                refused = false;
            }
        } else {
            fullSince = -1;
        }
        double el = fullSince < 0 ? -1 : t - fullSince;
        boolean lockPhase = el >= MechParams.FULL_RUN_ON_S + MechParams.FULL_REVERSE_S;
        boolean want = (lockPhase || in.aimWants) && !in.forceIn;
        if (want) notWantedSince = -1;
        else if (notWantedSince < 0) notWantedSince = t;

        boolean stoppedNow = in.pusherStopped && !justCommanded;
        justCommanded = false;
        switch (lock) {
            case NONE:
                if (want) {
                    if (in.canLock && failures < MechParams.LOCK_MAX_TRIES) {
                        cmdLock = true;
                        refused = false;
                    } else {
                        refused = true;
                    }
                } else {
                    refused = false;
                }
                break;
            case LOCKING:
                if (stoppedNow) {
                    if (in.pusherLocked) {
                        lock = Lock.LOCKED;
                    } else {
                        failures++;         // the attempt has failed; it may be tried again up to the limit
                        lock = Lock.NONE;
                    }
                }
                break;
            case LOCKED:
                // Unlock when the reason for the lock has ended (after a delay, and no launch is pending), or the operator forces the intake in.
                if (in.forceIn || (!want && t - notWantedSince >= MechParams.LOCK_RELEASE_DELAY_S && !in.launchPending)) {
                    cmdUnlock = true;
                    lock = Lock.UNLOCKING;
                    justCommanded = true;
                }
                break;
            case UNLOCKING:
                if (stoppedNow) lock = Lock.NONE;
                break;
        }

        if (in.forceIn && (fullSince >= 0 || lock != Lock.NONE)) {
            intake = Intake.Mode.IN;
            return;
        }
        if (fullSince < 0) return;
        if (el < MechParams.FULL_RUN_ON_S) {
            // Lets the fourth element, counted as it passed the sensor, finish entering.
            intake = Intake.Mode.IN;
            text = "FULL: INTAKE RUNNING ON";
        } else if (!lockPhase) {
            intake = Intake.Mode.OUT;
            text = "FULL: REVERSING";
        } else if (lock == Lock.LOCKED) {
            if (MechParams.INTAKE_OUT_WHILE_LOCKED) {
                intake = Intake.Mode.OUT;
                text = "FULL: LOCKED, INTAKE OUT";
            } else {
                intake = fifthWatch(in) ? Intake.Mode.OUT : Intake.Mode.OFF;
                text = "FULL: LOCKED";
            }
        } else if (lock == Lock.LOCKING || cmdLock) {
            intake = Intake.Mode.OFF;
            text = "FULL: LOCKING";
        } else if (refused) {
            // A stopped intake with the arm open lets the elements roll out.
            intake = Intake.Mode.IN;
            text = "FULL: LOCK REFUSED";
        } else {
            intake = Intake.Mode.OFF;
            text = "FULL: UNLOCKING";
        }
    }

    /** The fifth-element watch: true while the intake should run OUT to send away an element arriving at a full, locked robot. */
    private boolean fifthWatch(In in) {
        double t = in.t, above = in.proximity - in.emptyLevel;
        if (Double.isNaN(in.emptyLevel) || in.proximity < 0) return false;
        if (outSince >= 0) {
            // Running OUT: at least the minimum, until CS5 has read near empty for a while; give up after a limit.
            if (above <= MechParams.FIFTH_CLEAR_COUNTS) {
                if (clearSince < 0) clearSince = t;
            } else {
                clearSince = -1;
            }
            boolean cleared = clearSince >= 0 && t - clearSince >= MechParams.FIFTH_CLEAR_S && t - outSince >= MechParams.FIFTH_MIN_OUT_S;
            if (cleared || t - outSince >= MechParams.FIFTH_GIVE_UP_S) {
                if (!cleared) retryAt = t + MechParams.FIFTH_RETRY_S;
                outSince = -1;
                dwellSince = -1;
                return false;
            }
            return true;
        }
        // Armed once CS5 has read empty for a while, so an element resting there is one of the four.
        if (!fifthArmed) {
            if (above <= MechParams.FIFTH_CLEAR_COUNTS) {
                if (emptySince < 0) emptySince = t;
                if (t - emptySince >= MechParams.FIFTH_ARM_AFTER_S) fifthArmed = true;
            } else {
                emptySince = -1;
            }
            return false;
        }
        if (t < retryAt) return false;
        if (above >= MechParams.FIFTH_DWELL_COUNTS) {
            if (dwellSince < 0) dwellSince = t;
            if (t - dwellSince >= MechParams.FIFTH_DWELL_S) {
                outSince = t;
                clearSince = -1;
                return true;
            }
        } else {
            dwellSince = -1;
        }
        return false;
    }
}
