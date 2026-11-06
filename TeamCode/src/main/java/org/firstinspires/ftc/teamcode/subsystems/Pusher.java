package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.InstantAction;

/**
 * The pusher: ONE pusher arm that rotates around the curved track the elements ride, driven by
 * one motor with a RELATIVE encoder and NO hard stops (it spins continuously, almost always forward).
 * The back of the arm limits how many elements enter (the 4 cap) while it sits at HOLD; a salvo takes
 * it SALVO_DEG (400, a full turn and 40 deg more) forward from the nearest HOLD, pushing the stack to the flywheel, so it rests
 * about 40 deg past a HOLD; the pusher check (PusherCheck) then parks it at CS4 and brings it back to HOLD. Design notes: design/mechanisms.md.
 *
 * Control is deliberately plain: every move runs the motor at a calibratable power and cuts it when the
 * position crosses a threshold, and the brake does the rest. No PID, no velocity loop.
 *   - A forward move (salvo, or a return to HOLD) runs at SWEEP_POWER and drops to the gentler
 *     APPROACH_POWER for the last APPROACH_DEG, then stops STOP_EARLY_DEG before the target.
 *   - A reject reverses at REJECT_POWER, finishing on the nearest HOLD behind.
 *   - Position is an angle on a circle: HOLD is 0 degrees, one revolution is TICKS_PER_REV ticks, and a
 *     "HOLD" is any multiple of it. Targets are absolute multiples, so landing errors do not accumulate.
 *
 * Localization: the encoder is relative, so the code assumes the pusher is at HOLD at init (Reference
 * ASSUME_AT_HOLD) and saves that zero in a static. A TeleOp after an auto uses Reference CONTINUE to keep
 * the auto's zero. The encoder is never reset in code. The encoder may count opposite to the motor
 * (ENCODER_SIGN), and the motor runs WITHOUT the hub's closed-loop encoder modes.
 *
 * Landing accuracy depends on calibrating SWEEP_POWER, APPROACH_POWER and STOP_EARLY_DEG on the real pusher
 * (the Launch Map Tuner or the TeleOp); the reading to watch is the error to the nearest HOLD. Since 2026-11-05 an ACTIVE HOLD
 * (HOLD_SERVO_ENABLED) latches the angle where the arm comes to rest and pushes it back when it drifts, besides the small nudge forward if it comes to rest a little short.
 * Call {@link #update()} once per loop.
 */
@Config
public class Pusher {

    /** Encoder ticks for one full pusher revolution. goBILDA's 751.8 for the 26.9:1 motor; ten bench turns read 751.6. */
    public static double TICKS_PER_REV = 751.8;
    /**
     * +1 if the SDK encoder count rises when the arm moves forward (the way a positive power drives it), -1 if
     * it falls. The pusher bench's encoder polarity opposes its motor's (a positive command counted DOWN).
     */
    public static double ENCODER_SIGN = -1.0;

    // placeholders: calibrate on the rig
    /** Forward power for a salvo or a return to HOLD, until APPROACH_DEG from the target. */
    public static double SWEEP_POWER = 0.3;   // was 0.6; halved 2026-10-31 so the pusher moves slower (try up to 0.5 live in the Dashboard)
    /** Forward power for the last APPROACH_DEG of a move, so it lands gently. */
    public static double APPROACH_POWER = 0.25;
    /** Degrees before the target where the power drops to APPROACH_POWER. 0 turns the slow approach off. */
    public static double APPROACH_DEG = 30.0;
    /** Cut the power this many degrees before the target; the brake and coasting cover the rest. Calibrate. */
    public static double STOP_EARLY_DEG = 3.0;
    /**
     * Also cut this many seconds' worth of the arm's current speed early. Speed times this is the extra distance
     * the arm still travels after the cut (loop delay plus coasting), so one value covers the fast sweep and the
     * slow approach where a fixed STOP_EARLY_DEG cannot. Calibrate it from "Pusher last settle". 0 turns it off.
     */
    public static double COAST_LOOKAHEAD_S = 0.0;
    public static double REJECT_POWER = 0.6;
    /**
     * How far a salvo turns the arm, in degrees forward from the nearest HOLD: a full turn (360) and a margin so the last element is
     * surely out. The arm rests this far (less 360) past a HOLD, and the next salvo targets the nearest HOLD plus this again, so
     * nothing accumulates when the pusher check does not run in between.
     */
    public static double SALVO_DEG = 400.0;
    /** A reject reverses about this far and finishes on the nearest HOLD behind (330 leaves it 30 deg short of a full turn, so it finishes a full reverse turn). */
    public static double REJECT_DEG = 330.0;
    /** Within this many degrees of a HOLD counts as "at HOLD". */
    public static double HOLD_TOLERANCE_DEG = 8.0;
    /** The arm counts as stopped below this speed. */
    public static double STOP_SPEED_TICKS_PER_SEC = 30.0;
    /** If it comes to rest short of HOLD by more than the tolerance but less than this, it nudges forward. 0 turns the nudge off. */
    public static double AUTO_CORRECT_MAX_DEG = 45.0;
    /**
     * ACTIVE HOLD (2026-11-05): in the HOLD state the motor used to be at 0 power with the brake, which resists motion but does not hold a position, so a push or the pollen pressure
     * could walk the arm away (logs of 2026-11-05: the arm sat more than 10 deg from where it stopped for 30 to 46 percent of the parked time). Now, once the arm has been still
     * for HOLD_LATCH_SEC wherever it came to rest (HOLD, the 40 deg past it after a salvo, the lock), that angle is latched and a small P power pushes it back when it
     * drifts more than HOLD_DEADBAND_DEG: HOLD_KP_POWER_PER_DEG per degree, at least HOLD_MIN_POWER (to beat the friction), at most HOLD_MAX_POWER, in 0.01 steps (each change is
     * a hub write). If it needs the maximum for HOLD_GIVE_UP_SEC (it cannot be held: a jam, or a stack pushing harder than the cap) the hold is released until the next move so the
     * motor does not cook. Not used after a move gave up, or when the pusher is not calibrated.
     */
    public static boolean HOLD_SERVO_ENABLED = true;
    public static double HOLD_KP_POWER_PER_DEG = 0.012;
    public static double HOLD_MIN_POWER = 0.05;
    public static double HOLD_MAX_POWER = 0.15;
    public static double HOLD_DEADBAND_DEG = 2.0;
    public static double HOLD_LATCH_SEC = 0.3;
    public static double HOLD_GIVE_UP_SEC = 2.0;
    /** Default power for a seek (a short move to a given position, either direction). */
    public static double SEEK_POWER = 0.2;
    /**
     * The LOCK position (2026-11-04): with the robot full the arm moves this far forward of a HOLD and rests there, across the intake entrance, so the pollen cannot roll back
     * out (and a fifth cannot get in). It moves the pollen forward a little but does not launch one. A salvo from here targets the nearest HOLD behind plus SALVO_DEG, so it
     * turns SALVO_DEG - LOCK_DEG (310). Unlocking reverses to that HOLD. The lock and unlock are seeks at LOCK_POWER.
     */
    public static double LOCK_DEG = 90.0;
    public static double LOCK_POWER = 0.25;
    /** The arm counts as locked when it is stopped within this many degrees of LOCK_DEG past a HOLD. */
    public static double LOCK_TOLERANCE_DEG = 12.0;

    /** How often (ms) the motor current is refreshed for the display when no stall check needs it. */
    public static double CURRENT_DISPLAY_REFRESH_MS = 250.0;
    public static double STALL_CURRENT_A = 6.0;
    public static double STALL_TIME_S = 0.3;
    public static double STALL_GRACE_S = 0.3;
    public static double STALL_MIN_TICKS_PER_SEC = 20.0;
    public static double STALL_MIN_JOG_POWER = 0.15;
    /**
     * TEMPORARILY OFF: with false the pusher never faults (no stall detection, no progress check), so a drive team has nothing to read or
     * clear in a match; a jam is handled by the operator pressing reject. Instead a move that has not finished within MOVE_TIMEOUT_S just
     * stops (the motor is released, the state goes back to HOLD, the log says so). Set true to get the faults back.
     */
    public static boolean FAULTS_ENABLED = false;
    public static double MOVE_TIMEOUT_S = 3.0;
    /** A move that gains less than PROGRESS_MIN_GAIN_DEG in PROGRESS_TIMEOUT_S faults (wrong sign or direction, or a jam). */
    public static double PROGRESS_TIMEOUT_S = 1.5;
    public static double PROGRESS_MIN_GAIN_DEG = 2.0;

    /** HOLD = idle and braked; SALVO and RETURN are forward moves; REJECT reverses; SEEK goes to a given position either way (the pusher check); JOG is open loop for bring-up. */
    public enum State { HOLD, SALVO, RETURN, REJECT, SEEK, JOG, STALLED }

    public enum Reference { ASSUME_AT_HOLD, CONTINUE }

    private static boolean savedReferenceValid = false;
    private static int savedHomeRaw = 0;
    /** True once a pusher check has seen the arm where the zero says (or corrected it); false whenever the zero is only an assumption. */
    private static boolean zeroVerified = false;

    private final DcMotorEx motor;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime progressTimer = new ElapsedTime();
    private final ElapsedTime restTimer = new ElapsedTime();
    private int homeRaw;
    private final boolean referenceFromSaved;
    private State state = State.HOLD;
    private double jogPower = 0.0;
    private boolean autoCorrect = true;
    private double seekDir = 1.0;
    private double seekPower = 0.2;
    private double targetTicks = 0.0;
    private boolean stalling = false;
    // active hold (HOLD_SERVO_ENABLED)
    private boolean holdLatched = false;
    private double holdLatchTicks = 0.0;
    private double holdPowerNow = 0.0;
    private boolean holdSaturated = false;
    private boolean holdGaveUp = false;
    private final ElapsedTime holdSaturatedClock = new ElapsedTime();
    private final ElapsedTime holdEpisodeClock = new ElapsedTime();
    private double holdPeakErrDeg = 0.0, holdPeakPower = 0.0, holdActiveSec = 0.0;
    private boolean holdActive = false;
    private final ElapsedTime holdActiveClock = new ElapsedTime();
    private String holdReport = null;
    private boolean gaveUp = false;   // the last move timed out (MOVE_TIMEOUT_S): the arm stays where it is until the next move is commanded
    private String faultReason = "";
    private double progressBest = Double.NEGATIVE_INFINITY;
    private double moveStartTicks = 0.0;
    private String lastCommand = "none yet";
    private String lastResult = "-";
    private String lastSettle = "-";
    private boolean pendingSettle = false;
    private double cutUnwrapped = 0.0, cutSpeedDegPerSec = 0.0, cutRemainingDeg = 0.0, cutDirection = 1.0;

    // Hub calls are blocking round trips, so this class talks to the hub in only two places: readSensors() once at
    // the top of update(), and setMotorPower() when the command actually changes. Everything else uses these.
    private final ElapsedTime currentTimer = new ElapsedTime();
    private int rawTicks = 0;
    private double velocityRaw = 0.0;
    private double currentA = 0.0;
    private double lastPower = 0.0;

    public Pusher(HardwareMap hardwareMap, Reference reference) {
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "gate");
        motor.setDirection(profile.pusherDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);

        int raw = motor.getCurrentPosition();
        rawTicks = raw;
        if (reference == Reference.CONTINUE && savedReferenceValid) {
            homeRaw = savedHomeRaw;
            referenceFromSaved = true;
        } else {
            homeRaw = raw;
            referenceFromSaved = false;
        }
        if (!referenceFromSaved) zeroVerified = false;   // a zero taken from where the arm happens to be is only assumed to be HOLD
        savedReferenceValid = true;
        savedHomeRaw = homeRaw;
    }

    /** True once a pusher check has verified the zero against CS4 since it was last assumed or set by hand. */
    public static boolean isZeroVerified() {
        return zeroVerified;
    }

    public static void setZeroVerified(boolean verified) {
        zeroVerified = verified;
    }

    public boolean isCalibrated() {
        return TICKS_PER_REV > 0.0;
    }

    /** True if the zero came from a saved auto reference rather than being assumed at this init. */
    public boolean referenceFromSaved() {
        return referenceFromSaved;
    }

    /** Declare the pusher to be at HOLD right now (bring-up). The encoder is not reset, only the reference. */
    public void markHoldHere() {
        readSensors();
        homeRaw = rawTicks;
        savedHomeRaw = homeRaw;
        savedReferenceValid = true;
        zeroVerified = false;
    }

    /** The one place the sensors are read from the hub. Current is only read when the stall check or the display needs it. */
    private void readSensors() {
        rawTicks = motor.getCurrentPosition();
        velocityRaw = motor.getVelocity();
        boolean maybeBlocked = isCommanded() && moveTimer.seconds() > STALL_GRACE_S
                && Math.abs(velocityRaw) < STALL_MIN_TICKS_PER_SEC;
        if (maybeBlocked || currentTimer.milliseconds() > CURRENT_DISPLAY_REFRESH_MS) {
            currentA = motor.getCurrent(CurrentUnit.AMPS);
            currentTimer.reset();
        }
    }

    /** Write the motor power only when it changed; each write is a blocking round trip to the hub. */
    private void setMotorPower(double power) {
        if (power != lastPower) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    private boolean isCommanded() {
        return state == State.SALVO || state == State.RETURN || state == State.REJECT || state == State.SEEK
                || (state == State.JOG && Math.abs(jogPower) >= STALL_MIN_JOG_POWER);
    }

    public State getState() {
        return state;
    }

    public boolean isStalled() {
        return state == State.STALLED;
    }

    /** True after a move gave up (took longer than MOVE_TIMEOUT_S): the arm was left where it is, with no nudge to HOLD, until a new move starts. */
    public boolean isGaveUp() {
        return gaveUp;
    }

    /** Ticks forward of HOLD since the zero, accumulating across revolutions. */
    public double getUnwrappedTicks() {
        return ENCODER_SIGN * (rawTicks - homeRaw);
    }

    /** The SDK's encoder count, before ENCODER_SIGN and the HOLD zero (bring-up). */
    public int getRawTicks() {
        return rawTicks;
    }

    /** Pusher angle in [0, 360) from HOLD, or NaN until TICKS_PER_REV is set. */
    public double getAngleDeg() {
        if (!isCalibrated()) return Double.NaN;
        double angle = getUnwrappedTicks() * 360.0 / TICKS_PER_REV;
        return ((angle % 360.0) + 360.0) % 360.0;
    }

    /** Signed so that forward is positive, whatever the encoder wiring. */
    public double getVelocityTicksPerSec() {
        return ENCODER_SIGN * velocityRaw;
    }

    /** Motor current as of the last read: fresh while a move could be blocked, otherwise refreshed a few times a second. */
    public double getCurrentAmps() {
        return currentA;
    }

    private double nearestHoldTicks() {
        return Math.round(getUnwrappedTicks() / TICKS_PER_REV) * TICKS_PER_REV;
    }

    /** The unwrapped ticks of the HOLD nearest to the arm (for planning a seek). */
    public double getNearestHoldTicks() {
        return nearestHoldTicks();
    }

    /** Degrees to the nearest HOLD: positive = still short of it (needs to go forward), negative = past it. NaN until calibrated. */
    public double getHoldErrorDeg() {
        if (!isCalibrated()) return Double.NaN;
        return (nearestHoldTicks() - getUnwrappedTicks()) * 360.0 / TICKS_PER_REV;
    }

    /** Idle, stopped, and within HOLD_TOLERANCE_DEG of a HOLD. */
    public boolean atHold() {
        return state == State.HOLD && isCalibrated() && Math.abs(getHoldErrorDeg()) <= HOLD_TOLERANCE_DEG
                && Math.abs(velocityRaw) < STOP_SPEED_TICKS_PER_SEC;
    }

    /** Idle and not moving, wherever it came to rest: a salvo can start from here (it targets the next HOLD ahead of the nearest one). */
    public boolean isStopped() {
        return state == State.HOLD && Math.abs(velocityRaw) < STOP_SPEED_TICKS_PER_SEC;
    }

    /** Go to the nearest HOLD: forward only. Already within the tolerance: just stop. */
    public void hold() {
        gaveUp = false;
        if (!isCalibrated()) {
            stopHere();
            return;
        }
        double error = getHoldErrorDeg();
        if (error > HOLD_TOLERANCE_DEG) {
            beginForwardMove(State.RETURN, nearestHoldTicks());
        } else if (error < -HOLD_TOLERANCE_DEG) {
            beginForwardMove(State.RETURN, nearestHoldTicks() + TICKS_PER_REV);
        } else {
            stopHere();
        }
    }

    /** Open-loop jog for bring-up and calibration; positive = forward. Not for match use. */
    public void jog(double power) {
        if (state != State.JOG) moveTimer.reset();
        gaveUp = false;
        state = State.JOG;
        jogPower = Math.max(-1.0, Math.min(1.0, power));
    }

    /** A salvo: forward to SALVO_DEG past the nearest HOLD (400 deg, so it rests about 40 deg past a HOLD). Ignored until calibrated, or while faulted. */
    public void startSalvo() {
        if (!isCalibrated()) {
            lastCommand = "salvo IGNORED: TICKS_PER_REV is not set";
            return;
        }
        if (state == State.STALLED) {
            lastCommand = "salvo IGNORED: the pusher is faulted (" + faultReason + "); clear it first";
            return;
        }
        double target = nearestHoldTicks() + SALVO_DEG * TICKS_PER_REV / 360.0;
        lastCommand = String.format("salvo from %s, %+.1f deg to nearest HOLD, now %.0f ticks, target %.0f ticks",
                state, getHoldErrorDeg(), getUnwrappedTicks(), target);
        beginForwardMove(State.SALVO, target);
    }

    /** Reverse about REJECT_DEG to reject an element, finishing on the nearest HOLD behind. Ignored until calibrated, or while faulted. */
    public void startReject() {
        if (!isCalibrated()) {
            lastCommand = "reject IGNORED: TICKS_PER_REV is not set";
            return;
        }
        if (state == State.STALLED) {
            lastCommand = "reject IGNORED: the pusher is faulted (" + faultReason + "); clear it first";
            return;
        }
        targetTicks = Math.round((getUnwrappedTicks() - REJECT_DEG * TICKS_PER_REV / 360.0) / TICKS_PER_REV) * TICKS_PER_REV;
        lastCommand = String.format("reject from %s, %+.1f deg to nearest HOLD, now %.0f ticks, target %.0f ticks",
                state, getHoldErrorDeg(), getUnwrappedTicks(), targetTicks);
        gaveUp = false;
        state = State.REJECT;
        pendingSettle = false;
        moveStartTicks = getUnwrappedTicks();
        moveTimer.reset();
        resetProgress();
    }

    /**
     * Go to a given position (unwrapped ticks, from {@link #getNearestHoldTicks} plus an offset), forward or in reverse, at the
     * given power, stopping STOP_EARLY_DEG short like the other moves. For the pusher check, which parks the arm in front of CS4.
     * Ignored until calibrated, or while faulted.
     */
    public void startSeek(double target, double power) {
        if (!isCalibrated() || state == State.STALLED) return;
        double diff = target - getUnwrappedTicks();
        if (Math.abs(diff) < 1.0) {
            stopHere();
            return;
        }
        seekDir = diff > 0 ? 1.0 : -1.0;
        seekPower = power;
        targetTicks = target;
        lastCommand = String.format("seek to %+.1f deg from the nearest HOLD at %.2f power, now %.0f ticks, target %.0f ticks",
                (target - nearestHoldTicks()) * 360.0 / TICKS_PER_REV, power, getUnwrappedTicks(), target);
        gaveUp = false;
        state = State.SEEK;
        pendingSettle = false;
        moveStartTicks = getUnwrappedTicks();
        moveTimer.reset();
        resetProgress();
    }

    /** Moves the arm from HOLD to the LOCK position (LOCK_DEG forward of the nearest HOLD). Ignored until calibrated, or while faulted. */
    public void startLock() {
        if (!isCalibrated() || state == State.STALLED) {
            lastCommand = "lock IGNORED: " + (isCalibrated() ? "the pusher is faulted" : "TICKS_PER_REV is not set");
            return;
        }
        startSeek(nearestHoldTicks() + LOCK_DEG * TICKS_PER_REV / 360.0, LOCK_POWER);
        lastCommand = "LOCK: " + lastCommand;
    }

    /** Moves the arm back from the LOCK position to the nearest HOLD (a reverse of LOCK_DEG; never forward, which would sweep the pollen into the flywheel). */
    public void startUnlock() {
        if (!isCalibrated() || state == State.STALLED) {
            lastCommand = "unlock IGNORED: " + (isCalibrated() ? "the pusher is faulted" : "TICKS_PER_REV is not set");
            return;
        }
        double target = nearestHoldTicks();
        if (target > getUnwrappedTicks()) {   // already behind that HOLD: nothing to reverse
            stopHere();
            lastCommand = "UNLOCK: already at or behind HOLD";
            return;
        }
        startSeek(target, LOCK_POWER);
        lastCommand = "UNLOCK: " + lastCommand;
    }

    /** The degrees past the nearest HOLD (negative = before it), for the lock check and the log. */
    public double getAnglePastHoldDeg() {
        return (getUnwrappedTicks() - nearestHoldTicks()) * 360.0 / TICKS_PER_REV;
    }

    /** Idle, stopped, and within LOCK_TOLERANCE_DEG of the LOCK position. */
    public boolean isLocked() {
        return state == State.HOLD && isCalibrated() && Math.abs(velocityRaw) < STOP_SPEED_TICKS_PER_SEC
                && Math.abs(getAnglePastHoldDeg() - LOCK_DEG) <= LOCK_TOLERANCE_DEG;
    }

    /** Ends a jog where it is: the state goes back to HOLD (so {@link #isStopped} can be true) without moving to a HOLD. */
    public void stopInPlace() {
        if (state == State.JOG) stopHere();
    }

    /** Stops a salvo, return, reject or seek where it is and releases the motor (state HOLD), for a new command that replaces it, e.g. to work a jam out the other way. */
    public void cancelMove() {
        if (state == State.SALVO || state == State.RETURN || state == State.REJECT || state == State.SEEK) {
            pendingSettle = false;
            stopHere();
        }
    }

    /** Stops a seek where it is (the arm stays put). Other moves are left alone. */
    public void abortSeek() {
        if (state == State.SEEK) stopHere();
    }

    /** False stops the small forward nudge to HOLD (the pusher check parks the arm short of HOLD on purpose). */
    public void setAutoCorrect(boolean on) {
        autoCorrect = on;
    }

    /**
     * Redefines the zero. The arm was at {@code unwrappedTicksThen} (by the old zero) when it was truly {@code trueAngleDeg}
     * ahead of HOLD; afterwards it reads that angle. The revolution count is kept (the smallest correction).
     * The new zero is saved for the next opmode, like {@link #markHoldHere}.
     */
    public void resyncAngle(double unwrappedTicksThen, double trueAngleDeg) {
        double desired = trueAngleDeg * TICKS_PER_REV / 360.0;
        double delta = desired - unwrappedTicksThen;
        delta -= TICKS_PER_REV * Math.round(delta / TICKS_PER_REV);
        homeRaw -= (int) Math.round(ENCODER_SIGN * delta);
        savedHomeRaw = homeRaw;
        savedReferenceValid = true;
    }

    public void clearStall() {
        stalling = false;
        faultReason = "";
        stopHere();
    }

    private void beginForwardMove(State moveState, double target) {
        gaveUp = false;
        targetTicks = target;
        state = moveState;
        pendingSettle = false;
        moveStartTicks = getUnwrappedTicks();
        moveTimer.reset();
        resetProgress();
    }

    /** Remember how the move that is ending ended (shown on screen so a move that does nothing can be explained). */
    private void recordFinish(double remainingDeg, double speedDegPerSec, double direction) {
        lastResult = String.format("%s cut off by threshold after %.2f s: travelled %.0f deg, %.1f deg left to target at the cut, speed %.0f deg/s",
                state, moveTimer.seconds(), Math.abs(getUnwrappedTicks() - moveStartTicks) * 360.0 / TICKS_PER_REV, remainingDeg, speedDegPerSec);
        cutUnwrapped = getUnwrappedTicks();
        cutSpeedDegPerSec = speedDegPerSec;
        cutRemainingDeg = remainingDeg;
        cutDirection = direction;
        pendingSettle = true;
    }

    /** Once the arm has stopped after a cut: how far it coasted, and where it came to rest relative to the target. */
    private void recordSettle() {
        pendingSettle = false;
        double ticksPerDeg = TICKS_PER_REV / 360.0;
        double coastDeg = cutDirection * (getUnwrappedTicks() - cutUnwrapped) / ticksPerDeg;
        double pastTargetDeg = cutDirection * (getUnwrappedTicks() - targetTicks) / ticksPerDeg;
        lastSettle = String.format("cut with %.1f deg left at %.0f deg/s; travelled %.1f deg more after the cut; rest %+.1f deg past the target (- = short)%s",
                cutRemainingDeg, cutSpeedDegPerSec, coastDeg, pastTargetDeg,
                cutSpeedDegPerSec > 50.0 ? String.format("; COAST_LOOKAHEAD_S %.3f -> about %.3f", COAST_LOOKAHEAD_S, COAST_LOOKAHEAD_S + pastTargetDeg / cutSpeedDegPerSec) : "");
    }

    private void stopHere() {
        state = State.HOLD;
        setMotorPower(0.0);
        restTimer.reset();
        resetProgress();
        endHoldEpisode();   // a new rest: the hold latches again where the arm settles
    }

    /** The active hold's push on the motor as of the last update (0 when it is not correcting), for the log. */
    public double getHoldPower() {
        return holdPowerNow;
    }

    /** One line about the last stretch the active hold was latched, when it had to correct or gave up (else null); returns it once. For the run log. */
    public String takeHoldReport() {
        String r = holdReport;
        holdReport = null;
        return r;
    }

    private void endHoldEpisode() {
        if (holdLatched) {
            if (holdActive) holdActiveSec += holdActiveClock.seconds();
            if (holdPeakErrDeg > HOLD_DEADBAND_DEG + 2.0) {
                holdReport = String.format(java.util.Locale.US, "PUSHER HOLD SERVO held the arm for %.1f s: peak drift %.1f deg, corrected for %.2f s, peak power %.2f",
                        holdEpisodeClock.seconds(), holdPeakErrDeg, holdActiveSec, holdPeakPower);
            }
        }
        holdLatched = false;
        holdGaveUp = false;
        holdActive = false;
        holdSaturated = false;
        holdPowerNow = 0.0;
        holdPeakErrDeg = 0.0;
        holdPeakPower = 0.0;
        holdActiveSec = 0.0;
    }

    /** The active hold's power for this loop (see HOLD_SERVO_ENABLED); 0 until the arm has been still long enough to latch. */
    private double holdServoPower(double unwrapped, double ticksPerDeg) {
        if (!HOLD_SERVO_ENABLED || !isCalibrated() || gaveUp || holdGaveUp) {
            holdPowerNow = 0.0;
            return 0.0;
        }
        if (!holdLatched) {
            if (restTimer.seconds() >= HOLD_LATCH_SEC && Math.abs(velocityRaw) < STOP_SPEED_TICKS_PER_SEC) {
                holdLatched = true;
                holdLatchTicks = unwrapped;
                holdEpisodeClock.reset();
                holdPeakErrDeg = 0.0;
                holdPeakPower = 0.0;
                holdActiveSec = 0.0;
                holdActive = false;
                holdSaturated = false;
            }
            holdPowerNow = 0.0;
            return 0.0;
        }
        double errDeg = (holdLatchTicks - unwrapped) / ticksPerDeg;   // positive: the arm has gone back from where it rested, so push it forward
        holdPeakErrDeg = Math.max(holdPeakErrDeg, Math.abs(errDeg));
        double power = 0.0;
        if (Math.abs(errDeg) > HOLD_DEADBAND_DEG) {
            double mag = Math.min(HOLD_MAX_POWER, Math.max(HOLD_MIN_POWER, HOLD_KP_POWER_PER_DEG * Math.abs(errDeg)));
            power = Math.signum(errDeg) * Math.round(mag * 100.0) / 100.0;
        }
        if (power != 0.0) {
            if (!holdActive) {
                holdActive = true;
                holdActiveClock.reset();
            }
            holdPeakPower = Math.max(holdPeakPower, Math.abs(power));
            if (Math.abs(power) >= HOLD_MAX_POWER - 1e-9) {
                if (!holdSaturated) {
                    holdSaturated = true;
                    holdSaturatedClock.reset();
                } else if (holdSaturatedClock.seconds() > HOLD_GIVE_UP_SEC) {
                    // it cannot be held: release it until the next move, and say so
                    holdActiveSec += holdActiveClock.seconds();
                    holdReport = String.format(java.util.Locale.US, "PUSHER HOLD SERVO GAVE UP after %.1f s: peak drift %.1f deg, the maximum power %.2f was needed for %.1f s without holding; released until the next move",
                            holdEpisodeClock.seconds(), holdPeakErrDeg, HOLD_MAX_POWER, HOLD_GIVE_UP_SEC);
                    holdGaveUp = true;
                    holdActive = false;
                    holdLatched = false;
                    holdPowerNow = 0.0;
                    return 0.0;
                }
            } else {
                holdSaturated = false;
            }
        } else if (holdActive) {
            holdActive = false;
            holdActiveSec += holdActiveClock.seconds();
            holdSaturated = false;
        }
        holdPowerNow = power;
        return power;
    }

    private void resetProgress() {
        progressBest = Double.NEGATIVE_INFINITY;
        progressTimer.reset();
    }

    /** metricDeg must INCREASE as the move succeeds. Faults if it stops gaining. */
    private void monitorProgress(double metricDeg) {
        if (!FAULTS_ENABLED) {
            if (moveTimer.seconds() > MOVE_TIMEOUT_S) {
                lastResult = String.format("%s gave up after %.2f s without finishing (no fault: faults are off): travelled %.0f deg", state, moveTimer.seconds(),
                        Math.abs(getUnwrappedTicks() - moveStartTicks) * 360.0 / TICKS_PER_REV);
                pendingSettle = false;
                gaveUp = true;
                stopHere();
            }
            return;
        }
        if (metricDeg > progressBest + PROGRESS_MIN_GAIN_DEG) {
            progressBest = metricDeg;
            progressTimer.reset();
        } else if (progressTimer.seconds() > PROGRESS_TIMEOUT_S) {
            fault("not converging: check ENCODER_SIGN and pusherDirection, or a jam");
        }
    }

    private void fault(String reason) {
        lastResult = String.format("%s FAULT after %.2f s: %s (travelled %.0f deg)", state, moveTimer.seconds(), reason,
                Math.abs(getUnwrappedTicks() - moveStartTicks) * 360.0 / TICKS_PER_REV);
        faultReason = reason;
        state = State.STALLED;
        setMotorPower(0.0);
    }

    /** Call once per loop. Reads the hub once, up front; everything below works from that snapshot. */
    public void update() {
        readSensors();
        double unwrapped = getUnwrappedTicks();
        double ticksPerDeg = TICKS_PER_REV / 360.0;
        boolean moving = state == State.SALVO || state == State.RETURN || state == State.REJECT || state == State.SEEK;

        if (moving && !isCalibrated()) {
            stopHere();
            moving = false;
        }

        switch (state) {
            case HOLD:
                setMotorPower(holdServoPower(unwrapped, ticksPerDeg));
                if (pendingSettle && isCalibrated() && restTimer.seconds() > 0.25
                        && Math.abs(velocityRaw) < STOP_SPEED_TICKS_PER_SEC) {
                    recordSettle();
                }
                if (autoCorrect && !gaveUp && isCalibrated() && AUTO_CORRECT_MAX_DEG > 0.0 && restTimer.seconds() > 0.3
                        && Math.abs(velocityRaw) < STOP_SPEED_TICKS_PER_SEC) {
                    double error = getHoldErrorDeg();
                    if (error > HOLD_TOLERANCE_DEG && error <= AUTO_CORRECT_MAX_DEG) hold();
                }
                break;
            case SALVO:
            case RETURN: {
                double remainingDeg = (targetTicks - unwrapped) / ticksPerDeg;
                double speedDegPerSec = Math.max(0.0, getVelocityTicksPerSec()) / ticksPerDeg;
                if (remainingDeg <= STOP_EARLY_DEG + speedDegPerSec * COAST_LOOKAHEAD_S) {
                    recordFinish(remainingDeg, speedDegPerSec, 1.0);
                    stopHere();
                } else {
                    setMotorPower(APPROACH_DEG > 0.0 && remainingDeg <= APPROACH_DEG ? APPROACH_POWER : SWEEP_POWER);
                    monitorProgress(unwrapped / ticksPerDeg);
                }
                break;
            }
            case REJECT: {
                double remainingDeg = (unwrapped - targetTicks) / ticksPerDeg;
                double speedDegPerSec = Math.max(0.0, -getVelocityTicksPerSec()) / ticksPerDeg;
                if (remainingDeg <= STOP_EARLY_DEG + speedDegPerSec * COAST_LOOKAHEAD_S) {
                    recordFinish(remainingDeg, speedDegPerSec, -1.0);
                    stopHere();
                } else {
                    setMotorPower(-(APPROACH_DEG > 0.0 && remainingDeg <= APPROACH_DEG ? APPROACH_POWER : REJECT_POWER));
                    monitorProgress(-unwrapped / ticksPerDeg);
                }
                break;
            }
            case SEEK: {
                double remainingDeg = seekDir * (targetTicks - unwrapped) / ticksPerDeg;
                double speedDegPerSec = Math.max(0.0, seekDir * getVelocityTicksPerSec()) / ticksPerDeg;
                if (remainingDeg <= STOP_EARLY_DEG + speedDegPerSec * COAST_LOOKAHEAD_S) {
                    recordFinish(remainingDeg, speedDegPerSec, seekDir);
                    stopHere();
                } else {
                    setMotorPower(seekDir * seekPower);
                    monitorProgress(seekDir * unwrapped / ticksPerDeg);
                }
                break;
            }
            case JOG:
                setMotorPower(jogPower);
                break;
            case STALLED:
                setMotorPower(0.0);
                break;
        }

        detectStall();
    }

    private void detectStall() {
        if (!FAULTS_ENABLED) {
            stalling = false;
            return;
        }
        boolean blocked = isCommanded() && moveTimer.seconds() > STALL_GRACE_S
                && currentA > STALL_CURRENT_A && Math.abs(velocityRaw) < STALL_MIN_TICKS_PER_SEC;
        if (!blocked) {
            stalling = false;
            return;
        }
        if (!stalling) {
            stalling = true;
            stallTimer.reset();
        } else if (stallTimer.seconds() > STALL_TIME_S) {
            fault("blocked: high current and no motion");
        }
    }

    /** The last command and how it went, as text (for logs). */
    public String getLastCommand() {
        return lastCommand;
    }

    public String getLastResult() {
        return lastResult;
    }

    public String getLastSettle() {
        return lastSettle;
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Pusher state", state + (referenceFromSaved ? "   (zero from saved auto reference)" : "   (zero assumed at init)"));
        telemetry.addData("Pusher angle (deg from HOLD)", isCalibrated() ? String.format("%.1f", getAngleDeg()) : "not calibrated (TICKS_PER_REV = 0)");
        telemetry.addData("Pusher ticks (unwrapped)", "%.0f   speed %.0f ticks/s", getUnwrappedTicks(), getVelocityTicksPerSec());
        telemetry.addData("Error to nearest HOLD (deg)", Double.isNaN(getHoldErrorDeg()) ? "-"
                : String.format("%+.1f   (+ = short of it, - = past it)", getHoldErrorDeg()));
        telemetry.addData("Pusher power / current", "%+.2f / %.2f A", lastPower, getCurrentAmps());
        telemetry.addData("Pusher last command", lastCommand);
        telemetry.addData("Pusher last move result", lastResult);
        telemetry.addData("Pusher last settle", lastSettle);
        telemetry.addData("Pusher ENCODER_SIGN", "%+.0f", ENCODER_SIGN);
        if (isStalled()) telemetry.addLine("PUSHER FAULT: " + faultReason);
    }

    /** Action: run one salvo and finish when the pusher has stopped, or has faulted, or times out. */
    public Action salvo(double timeoutSeconds) {
        return motion(true, timeoutSeconds);
    }

    /** Action: reject (reverse onto the nearest HOLD behind); finishes when stopped, faulted, or timed out. */
    public Action reject(double timeoutSeconds) {
        return motion(false, timeoutSeconds);
    }

    /** Action: return to the nearest HOLD, in the same loop. */
    public Action holdAction() {
        return new InstantAction(this::hold);
    }

    private Action motion(boolean salvo, double timeoutSeconds) {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    timer.reset();
                    if (salvo) startSalvo(); else startReject();
                    if (state != (salvo ? State.SALVO : State.REJECT)) return false;   // ignored: not calibrated or faulted
                }
                packet.put("pusher/angleDeg", getAngleDeg());
                boolean finished = state == State.HOLD || state == State.STALLED;
                return !finished && timer.seconds() < timeoutSeconds;
            }
        };
    }
}
