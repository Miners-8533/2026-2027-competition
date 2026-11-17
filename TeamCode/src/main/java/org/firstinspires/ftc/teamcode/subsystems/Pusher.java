package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * The pusher (specs/pusher.md sections 1 to 6): the arm that sweeps the stack of elements into the flywheel.
 *
 * Position is an angle that accumulates across revolutions, measured forward from HOLD (0 degrees). Deliberately
 * plain control: no PID and no speed loop. A move runs the motor at a fixed power and cuts it when the position
 * crosses a threshold; the motor brake does the rest. Faults are disabled: a move that cannot finish gives up.
 */
@Config
public final class Pusher {
    public static double TICKS_PER_REV = 751.8;
    public static int ENCODER_SIGN = -1;
    public static double SWEEP_POWER = 0.30, APPROACH_POWER = 0.30, REJECT_POWER = 0.30;
    public static double APPROACH_DEG = 30.0;
    public static double STOP_EARLY_DEG = 3.0;
    public static double COAST_LOOKAHEAD_S = 0.0;
    public static double SALVO_DEG = 400.0;
    public static double REJECT_DEG = 330.0;
    public static double HOLD_TOL_DEG = 8.0;
    public static double STOPPED_TICKS_S = 30.0;
    public static double NUDGE_MAX_DEG = 45.0, NUDGE_AFTER_S = 0.3;
    public static double HOLD_GAIN = 0.012, HOLD_MIN_POWER = 0.05, HOLD_MAX_POWER = 0.15;
    public static double HOLD_DEAD_BAND_DEG = 2.0, HOLD_LATCH_S = 0.3, HOLD_GIVE_UP_S = 2.0, HOLD_REPORT_DEG = 4.0;
    public static double SEEK_POWER = 0.2;
    public static double LOCK_DEG = 90.0, LOCK_POWER = 0.25, LOCK_TOL_DEG = 12.0;
    /** The lock goes forward from wherever the arm rests, as long as that is in this range around a HOLD. */
    public static double LOCK_FROM_MIN_DEG = -8.0, LOCK_FROM_MAX_DEG = 130.0;
    public static double MOVE_TIMEOUT_S = 3.0;
    public static double JAM_AMPS = 1.2, JAM_TICKS_S = 20.0, JAM_FOR_S = 0.3, JAM_AFTER_S = 0.4, JAM_GRACE_S = 0.3;
    public static double JAM_BACKOFF_DEG = 45.0, JAM_LOCK_BEHIND_HOLD_DEG = 45.0, JAM_MIN_BACKOFF_DEG = 10.0, JAM_POWER = 0.25;
    public static double CURRENT_REFRESH_MS = 250;
    /** Build-time: with faults off the drive team never has a fault to read or clear in a match. */
    public static final boolean FAULTS_ENABLED = false;
    public static double STALL_AMPS = 6.0, PROGRESS_DEG = 2.0, PROGRESS_S = 1.5;

    public enum State { HOLD, SALVO, RETURN, REJECT, SEEK, JOG, STALLED }

    private enum Kind { NONE, SALVO, RETURN, REJECT, SEEK, LOCK, UNLOCK }

    // The zero outlives the opmode: saved in memory for the life of the robot app.
    private static double savedZeroTicks;
    private static boolean haveSavedZero, savedVerified;

    public static boolean haveSavedZero() { return haveSavedZero; }

    private final DcMotorEx motor;
    private double zeroTicks;
    private boolean zeroVerified;

    private State state = State.HOLD;
    private Kind kind = Kind.NONE;
    private double target, basePower;
    private int dir = 1;
    private double moveStart, phaseStart, salvoHold;
    private boolean resumed, backingOff;
    private double backoffTarget, backoffStart;
    private int retries, maxRetries = 2;
    private double jamSince = -1;
    private boolean jammedThisMove, jamUnresolved, jamClearedNotice, gaveUp;
    private double progressAngle, progressAt;

    private double angle, speedTicks, amps, lastAmpsAt = -1, sentPower = Double.NaN;
    private boolean disabled, nudgeEnabled = true, nudged;
    private double restSince, stillSince = -1;
    private double holdLatch = Double.NaN, holdMaxSince = -1, holdPower;
    private boolean holdLetGo, holdReported;

    private Pusher(DcMotorEx motor, ChassisProfile profile, boolean continueZero) {
        this.motor = motor;
        motor.setDirection(profile.pusherReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // the encoder is relative and is never reset
        double ticks = motor.getCurrentPosition();
        if (continueZero && haveSavedZero) {
            zeroTicks = savedZeroTicks;
            zeroVerified = savedVerified;
            RunLog.eventf("PUSHER zero continued from the previous opmode (%s)", zeroVerified ? "verified" : "not verified");
        } else {
            // Assume at HOLD: the zero is wherever the arm is now. The arm must physically be at HOLD.
            zeroTicks = ticks;
            zeroVerified = false;
            RunLog.event("PUSHER zero assumed at HOLD where the arm is now");
        }
        saveZero();
        angle = toDeg(ticks);
        restSince = Clock.now();
    }

    /**
     * Null when the motor is not in the hub configuration.
     *
     * @param continueZero true for the TeleOp after an auto: reuse the zero the previous opmode saved, if there is one
     */
    public static Pusher create(HardwareMap hw, ChassisProfile profile, boolean continueZero) {
        DcMotorEx m = hw.tryGet(DcMotorEx.class, ChassisProfile.PUSHER);
        return m == null ? null : new Pusher(m, profile, continueZero);
    }

    private void saveZero() {
        savedZeroTicks = zeroTicks;
        savedVerified = zeroVerified;
        haveSavedZero = true;
    }

    private double toDeg(double ticks) { return calibrated() ? ENCODER_SIGN * (ticks - zeroTicks) * 360.0 / TICKS_PER_REV : 0; }

    private static double nearestHold(double deg) { return Math.round(deg / 360.0) * 360.0; }

    // ---- Queries ----
    public State state() { return state; }
    /** Degrees forward of HOLD, accumulating across revolutions. */
    public double angle() { return angle; }
    /** Degrees past the nearest HOLD, -180 to 180. */
    public double relAngle() { return angle - nearestHold(angle); }
    public double speedTicks() { return speedTicks; }
    public double speedDeg() { return calibrated() ? speedTicks * 360.0 / TICKS_PER_REV : 0; }
    public double amps() { return amps; }
    public double power() { return sentPower; }
    public double holdPower() { return holdPower; }
    /** Calibrated means the ticks per arm revolution is set. Until then every move is ignored and the coordinator warns. */
    public boolean calibrated() { return TICKS_PER_REV > 0; }
    public boolean zeroVerified() { return zeroVerified; }
    public boolean idle() { return state == State.HOLD; }
    /** Idle and not moving, wherever it rests. A salvo may start from here. */
    public boolean stopped() { return state == State.HOLD && Math.abs(speedTicks) < STOPPED_TICKS_S; }
    public boolean atHold() { return stopped() && Math.abs(relAngle()) <= HOLD_TOL_DEG; }
    public boolean locked() { return stopped() && Math.abs(relAngle() - LOCK_DEG) <= LOCK_TOL_DEG; }
    public boolean canLockFrom() {
        double r = relAngle();
        return r >= LOCK_FROM_MIN_DEG && r <= LOCK_FROM_MAX_DEG;
    }
    /** A salvo is running and has turned less than 360 deg past the HOLD it started from. */
    public boolean inLaunchSweep() { return state == State.SALVO && angle < salvoHold + 360.0; }
    /** The last move did not finish; cleared when the next move starts. */
    public boolean gaveUp() { return gaveUp; }
    public boolean faulted() { return state == State.STALLED; }
    public boolean jamUnresolved() { return jamUnresolved; }
    /** True once after a jammed move finished: a notice for the drivers. */
    public boolean takeJamCleared() {
        boolean b = jamClearedNotice;
        jamClearedNotice = false;
        return b;
    }

    // ---- Commands ----
    public void setJamRetries(int n) { maxRetries = n; }
    public void setSettleNudge(boolean on) { nudgeEnabled = on; }

    /** The zero has been seen to be right, or corrected, by a pusher check. */
    public void setZeroVerified(boolean v) {
        zeroVerified = v;
        saveZero();
    }

    /** Redefines the zero so that every angle reads deltaDeg more. The caller passes the smallest shift, so the revolution count is kept. */
    public void shiftZero(double deltaDeg) {
        zeroTicks -= ENCODER_SIGN * deltaDeg * TICKS_PER_REV / 360.0;
        angle += deltaDeg;
        holdLatch = Double.NaN;
        saveZero();
    }

    /** Forward move that launches the stack: to the nearest HOLD plus the salvo angle. */
    public void salvo() {
        salvoHold = nearestHold(angle);
        begin(Kind.SALVO, State.SALVO, salvoHold + SALVO_DEG, SWEEP_POWER);
    }

    /**
     * The hold command: a forward move to a HOLD. Short of the nearest HOLD: go to it. Past it: go forward to the next
     * one, which sweeps the track and would launch anything in it. Within tolerance: just stop.
     */
    public void hold() {
        double h = nearestHold(angle), r = angle - h;
        if (Math.abs(r) <= HOLD_TOL_DEG) {
            cancelMove();
            return;
        }
        begin(Kind.RETURN, State.RETURN, r < 0 ? h : h + 360.0, SWEEP_POWER);
    }

    /** Reverse to the HOLD nearest to "current position minus the reject angle". */
    public void reject() {
        begin(Kind.REJECT, State.REJECT, nearestHold(angle - REJECT_DEG), REJECT_POWER);
    }

    /** Go to an absolute position at a power, forward or reverse. No jam resolution: a back-off would corrupt a check's measurement. */
    public void seek(double targetDeg, double power) {
        begin(Kind.SEEK, State.SEEK, targetDeg, Math.abs(power));
    }

    /** Seek to the lock angle: the arm lies across the intake entrance. False when the arm is not in a position to lock from. */
    public boolean lock() {
        if (!canLockFrom()) return false;
        begin(Kind.LOCK, State.SEEK, nearestHold(angle) + LOCK_DEG, LOCK_POWER);
        return true;
    }

    /** Back to the HOLD behind the arm, reverse only. Does nothing if the arm is already at or behind it. */
    public void unlock() {
        double h = Math.floor((angle + HOLD_TOL_DEG) / 360.0) * 360.0;
        if (angle - h <= HOLD_TOL_DEG) return;
        begin(Kind.UNLOCK, State.SEEK, h, LOCK_POWER);
    }

    /** Stops the arm where it is. */
    public void cancelMove() {
        if (state == State.STALLED) return;
        endMove();
    }

    public void abortSeek() { cancelMove(); }

    /** Open-loop power, for bring-up only. */
    public void jog(double power) {
        kind = Kind.NONE;
        state = power == 0 ? State.HOLD : State.JOG;
        setPower(power);
        resetRest();
    }

    public void clearStall() {
        if (state == State.STALLED) endMove();
    }

    /** For the end of an opmode: stops and removes all power, including the active hold. */
    public void stopNow() {
        kind = Kind.NONE;
        state = State.HOLD;
        disabled = true;
        holdPower = 0;
        motor.setPower(0);
        sentPower = 0;
    }

    private void begin(Kind k, State s, double targetDeg, double power) {
        if (state == State.STALLED || disabled || !calibrated()) return;
        kind = k;
        state = s;
        target = targetDeg;
        dir = targetDeg >= angle ? 1 : -1;
        if (k == Kind.SALVO || k == Kind.RETURN) dir = 1;
        if (k == Kind.REJECT || k == Kind.UNLOCK) dir = -1;
        basePower = power;
        moveStart = phaseStart = Clock.now();
        resumed = false;
        backingOff = false;
        retries = 0;
        jamSince = -1;
        jammedThisMove = false;
        jamUnresolved = false;
        gaveUp = false;
        progressAngle = angle;
        progressAt = moveStart;
        holdPower = 0;
    }

    private void endMove() {
        kind = Kind.NONE;
        state = State.HOLD;
        backingOff = false;
        setPower(0);
        resetRest();
    }

    private void resetRest() {
        restSince = Clock.now();
        stillSince = -1;
        nudged = false;
        holdLatch = Double.NaN;
        holdMaxSince = -1;
        holdLetGo = false;
        holdReported = false;
        holdPower = 0;
    }

    private void giveUp(String why) {
        RunLog.eventf("PUSHER gave up on %s at %.0f deg (target %.0f): %s", kind, angle, target, why);
        endMove();
        gaveUp = true;
    }

    private void setPower(double p) {
        if (p != sentPower) {
            motor.setPower(p);
            sentPower = p;
        }
    }

    private void readAmps(double t, boolean now) {
        if (!now && (t - lastAmpsAt) * 1000 < CURRENT_REFRESH_MS) return;
        lastAmpsAt = t;
        try {
            amps = motor.getCurrent(CurrentUnit.AMPS);
        } catch (Throwable ignored) { }
    }

    // ---- Update: read once, write when the power changes ----
    public void update() {
        double t = Clock.now();
        angle = toDeg(motor.getCurrentPosition());
        speedTicks = ENCODER_SIGN * motor.getVelocity();
        if (disabled) return;
        if (state == State.JOG || state == State.STALLED) {
            readAmps(t, false);
            return;
        }
        if (kind == Kind.NONE) {
            readAmps(t, false);
            rest(t);
            return;
        }
        if (backingOff) {
            // Reverse to take a run-up, then resume the original move toward its original target.
            boolean reached = angle - backoffTarget <= STOP_EARLY_DEG;
            if (reached || t - backoffStart > 1.0) {
                backingOff = false;
                resumed = true;
                phaseStart = t;
                jamSince = -1;
            } else {
                setPower(-JAM_POWER);
            }
            if (t - moveStart > MOVE_TIMEOUT_S) giveUp("timeout during a jam back-off");
            return;
        }
        double remaining = dir * (target - angle);
        if (remaining <= STOP_EARLY_DEG + Math.abs(speedDeg()) * COAST_LOOKAHEAD_S) {
            if (jammedThisMove) {
                RunLog.eventf("PUSHER jammed %s finished after %d back-off(s)", kind, retries);
                jamClearedNotice = true;
            }
            endMove();
            return;
        }
        if (t - moveStart > MOVE_TIMEOUT_S) {
            // The move's own timer keeps running through back-offs: a jam never costs more than an unresolved move did.
            giveUp("not finished within " + MOVE_TIMEOUT_S + " s");
            return;
        }
        // Jam resolution: only for forward moves that push elements (a salvo, a return, the lock seek).
        boolean pushes = kind == Kind.SALVO || kind == Kind.RETURN || kind == Kind.LOCK;
        boolean slow = Math.abs(speedTicks) < JAM_TICKS_S;
        boolean watch = t - phaseStart >= (resumed ? JAM_GRACE_S : JAM_AFTER_S);
        readAmps(t, slow && watch);       // the current is read only while a move might be blocked
        if (pushes && watch && slow && amps > JAM_AMPS) {
            if (jamSince < 0) jamSince = t;
            if (t - jamSince >= JAM_FOR_S) {
                jammedThisMove = true;
                if (retries < maxRetries) {
                    retries++;
                    double back = kind == Kind.LOCK ? (target - LOCK_DEG) - JAM_LOCK_BEHIND_HOLD_DEG : angle - JAM_BACKOFF_DEG;
                    backoffTarget = Math.min(back, angle - JAM_MIN_BACKOFF_DEG);
                    backoffStart = t;
                    backingOff = true;
                    RunLog.eventf("PUSHER jam in %s at %.0f deg (%.1f A, %.0f ticks/s): back-off %d of %d to %.0f deg",
                            kind, angle, amps, speedTicks, retries, maxRetries, backoffTarget);
                    setPower(-JAM_POWER);
                    return;
                }
                jamUnresolved = true;
                giveUp("jam not resolved after " + retries + " back-off(s)");
                jamUnresolved = true;
                return;
            }
        } else {
            jamSince = -1;
        }
        if (FAULTS_ENABLED) {
            if (Math.abs(angle - progressAngle) >= PROGRESS_DEG) {
                progressAngle = angle;
                progressAt = t;
            }
            if ((amps > STALL_AMPS && slow) || t - progressAt > PROGRESS_S) {
                RunLog.eventf("PUSHER STALLED in %s at %.0f deg", kind, angle);
                kind = Kind.NONE;
                state = State.STALLED;
                setPower(0);
                return;
            }
        }
        setPower(dir * (remaining <= APPROACH_DEG ? (kind == Kind.REJECT ? REJECT_POWER : Math.min(basePower, APPROACH_POWER)) : basePower));
    }

    /** At rest: the settle nudge and the active hold. */
    private void rest(double t) {
        boolean still = Math.abs(speedTicks) < STOPPED_TICKS_S;
        if (!still) stillSince = -1;
        else if (stillSince < 0) stillSince = t;
        if (gaveUp) {
            setPower(0);
            return;     // nothing nudges or holds the arm until the next move is commanded
        }
        // Settle nudge: short of a HOLD by more than the tolerance but not far: nudge forward to it. Never back from an overshoot.
        double r = relAngle();
        if (nudgeEnabled && !nudged && still && t - restSince >= NUDGE_AFTER_S && r < -HOLD_TOL_DEG && r >= -NUDGE_MAX_DEG) {
            nudged = true;
            double keepRest = restSince;
            begin(Kind.RETURN, State.RETURN, nearestHold(angle), SWEEP_POWER);
            nudged = true;
            restSince = keepRest;
            return;
        }
        // Active hold: the brake resists motion but not a steady push, so the stack's pressure can walk the arm away.
        if (holdLetGo) {
            setPower(0);
            return;
        }
        if (Double.isNaN(holdLatch)) {
            if (still && stillSince >= 0 && t - stillSince >= HOLD_LATCH_S) holdLatch = angle;
            setPower(0);
            return;
        }
        double err = holdLatch - angle;
        double p = 0;
        if (Math.abs(err) > HOLD_DEAD_BAND_DEG) {
            double mag = Math.max(HOLD_MIN_POWER, Math.min(HOLD_MAX_POWER, HOLD_GAIN * Math.abs(err)));
            p = Math.signum(err) * Math.round(mag * 100.0) / 100.0;      // 0.01 steps: each change is a hub write
            if (Math.abs(err) > HOLD_REPORT_DEG && !holdReported) {
                holdReported = true;
                RunLog.eventf("PUSHER active hold correcting %.1f deg at %.0f deg", err, angle);
            }
            if (mag >= HOLD_MAX_POWER) {
                if (holdMaxSince < 0) holdMaxSince = t;
                if (t - holdMaxSince >= HOLD_GIVE_UP_S) {
                    holdLetGo = true;     // so the motor is not cooked
                    p = 0;
                    RunLog.eventf("PUSHER active hold let go: maximum power for %.1f s without holding (%.1f deg off)", HOLD_GIVE_UP_S, err);
                }
            } else {
                holdMaxSince = -1;
            }
        } else {
            holdMaxSince = -1;
        }
        holdPower = p;
        setPower(p);
    }
}
