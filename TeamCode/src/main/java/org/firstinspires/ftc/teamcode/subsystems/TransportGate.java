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
 * The transport gate: ONE pusher arm that rotates around the curved track the elements ride, driven by
 * one motor with a RELATIVE encoder and NO hard stops (it spins continuously, almost always forward).
 * The back of the arm limits how many elements enter (the 4 cap) while it sits at HOLD; a salvo takes
 * it one full turn forward, pushing the stack to the flywheel. Design notes: design/mechanisms.md.
 *
 * Control is deliberately plain: every move runs the motor at a calibratable power and cuts it when the
 * position crosses a threshold, and the brake does the rest. No PID, no velocity loop.
 *   - A forward move (salvo, or a return to HOLD) runs at SWEEP_POWER and drops to the gentler
 *     APPROACH_POWER for the last APPROACH_DEG, then stops STOP_EARLY_DEG before the target.
 *   - A reject reverses at REJECT_POWER, finishing on the nearest HOLD behind.
 *   - Position is an angle on a circle: HOLD is 0 degrees, one revolution is TICKS_PER_REV ticks, and a
 *     "HOLD" is any multiple of it. Targets are absolute multiples, so landing errors do not accumulate.
 *
 * Localization: the encoder is relative, so the code assumes the gate is at HOLD at init (Reference
 * ASSUME_AT_HOLD) and saves that zero in a static. A TeleOp after an auto uses Reference CONTINUE to keep
 * the auto's zero. The encoder is never reset in code. The encoder may count opposite to the motor
 * (ENCODER_SIGN), and the motor runs WITHOUT the hub's closed-loop encoder modes.
 *
 * Landing accuracy depends on calibrating SWEEP_POWER, APPROACH_POWER and STOP_EARLY_DEG on the real gate
 * (Transport Gate Test); the reading to watch is the error to the nearest HOLD. Nothing holds the arm
 * against a push beyond the brake, except a small nudge forward if it comes to rest a little short.
 * Call {@link #update()} once per loop.
 */
@Config
public class TransportGate {

    /** Encoder ticks for one full gate revolution. goBILDA's 751.8 for the 26.9:1 motor; ten bench turns read 751.6. */
    public static double TICKS_PER_REV = 751.8;
    /**
     * +1 if the SDK encoder count rises when the arm moves forward (the way a positive power drives it), -1 if
     * it falls. The gate bench's encoder polarity opposes its motor's (a positive command counted DOWN).
     */
    public static double ENCODER_SIGN = -1.0;

    // placeholders: calibrate on the rig
    /** Forward power for a salvo or a return to HOLD, until APPROACH_DEG from the target. */
    public static double SWEEP_POWER = 0.6;
    /** Forward power for the last APPROACH_DEG of a move, so it lands gently. */
    public static double APPROACH_POWER = 0.25;
    /** Degrees before the target where the power drops to APPROACH_POWER. 0 turns the slow approach off. */
    public static double APPROACH_DEG = 30.0;
    /** Cut the power this many degrees before the target; the brake and coasting cover the rest. Calibrate. */
    public static double STOP_EARLY_DEG = 3.0;
    public static double REJECT_POWER = 0.6;
    /** A reject reverses about this far and finishes on the nearest HOLD behind (330 leaves it 30 deg short of a full turn, so it finishes a full reverse turn). */
    public static double REJECT_DEG = 330.0;
    /** Within this many degrees of a HOLD counts as "at HOLD". */
    public static double HOLD_TOLERANCE_DEG = 8.0;
    /** The arm counts as stopped below this speed. */
    public static double STOP_SPEED_TICKS_PER_SEC = 30.0;
    /** If it comes to rest short of HOLD by more than the tolerance but less than this, it nudges forward. 0 turns the nudge off. */
    public static double AUTO_CORRECT_MAX_DEG = 45.0;

    public static double STALL_CURRENT_A = 6.0;
    public static double STALL_TIME_S = 0.3;
    public static double STALL_GRACE_S = 0.3;
    public static double STALL_MIN_TICKS_PER_SEC = 20.0;
    public static double STALL_MIN_JOG_POWER = 0.15;
    /** A move that gains less than PROGRESS_MIN_GAIN_DEG in PROGRESS_TIMEOUT_S faults (wrong sign or direction, or a jam). */
    public static double PROGRESS_TIMEOUT_S = 1.5;
    public static double PROGRESS_MIN_GAIN_DEG = 2.0;

    /** HOLD = idle and braked; SALVO and RETURN are forward moves; REJECT reverses; JOG is open loop for bring-up. */
    public enum State { HOLD, SALVO, RETURN, REJECT, JOG, STALLED }

    public enum Reference { ASSUME_AT_HOLD, CONTINUE }

    private static boolean savedReferenceValid = false;
    private static int savedHomeRaw = 0;

    private final DcMotorEx motor;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime progressTimer = new ElapsedTime();
    private final ElapsedTime restTimer = new ElapsedTime();
    private int homeRaw;
    private final boolean referenceFromSaved;
    private State state = State.HOLD;
    private double jogPower = 0.0;
    private double targetTicks = 0.0;
    private boolean stalling = false;
    private String faultReason = "";
    private double progressBest = Double.NEGATIVE_INFINITY;

    public TransportGate(HardwareMap hardwareMap, Reference reference) {
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "gate");
        motor.setDirection(profile.gateDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);

        int raw = motor.getCurrentPosition();
        if (reference == Reference.CONTINUE && savedReferenceValid) {
            homeRaw = savedHomeRaw;
            referenceFromSaved = true;
        } else {
            homeRaw = raw;
            referenceFromSaved = false;
        }
        savedReferenceValid = true;
        savedHomeRaw = homeRaw;
    }

    public boolean isCalibrated() {
        return TICKS_PER_REV > 0.0;
    }

    /** True if the zero came from a saved auto reference rather than being assumed at this init. */
    public boolean referenceFromSaved() {
        return referenceFromSaved;
    }

    /** Declare the gate to be at HOLD right now (bring-up). The encoder is not reset, only the reference. */
    public void markHoldHere() {
        homeRaw = motor.getCurrentPosition();
        savedHomeRaw = homeRaw;
        savedReferenceValid = true;
    }

    public State getState() {
        return state;
    }

    public boolean isStalled() {
        return state == State.STALLED;
    }

    /** Ticks forward of HOLD since the zero, accumulating across revolutions. */
    public double getUnwrappedTicks() {
        return ENCODER_SIGN * (motor.getCurrentPosition() - homeRaw);
    }

    /** The SDK's encoder count, before ENCODER_SIGN and the HOLD zero (bring-up). */
    public int getRawTicks() {
        return motor.getCurrentPosition();
    }

    /** Gate angle in [0, 360) from HOLD, or NaN until TICKS_PER_REV is set. */
    public double getAngleDeg() {
        if (!isCalibrated()) return Double.NaN;
        double angle = getUnwrappedTicks() * 360.0 / TICKS_PER_REV;
        return ((angle % 360.0) + 360.0) % 360.0;
    }

    /** Signed so that forward is positive, whatever the encoder wiring. */
    public double getVelocityTicksPerSec() {
        return ENCODER_SIGN * motor.getVelocity();
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    private double nearestHoldTicks() {
        return Math.round(getUnwrappedTicks() / TICKS_PER_REV) * TICKS_PER_REV;
    }

    /** Degrees to the nearest HOLD: positive = still short of it (needs to go forward), negative = past it. NaN until calibrated. */
    public double getHoldErrorDeg() {
        if (!isCalibrated()) return Double.NaN;
        return (nearestHoldTicks() - getUnwrappedTicks()) * 360.0 / TICKS_PER_REV;
    }

    /** Idle, stopped, and within HOLD_TOLERANCE_DEG of a HOLD. */
    public boolean atHold() {
        return state == State.HOLD && isCalibrated() && Math.abs(getHoldErrorDeg()) <= HOLD_TOLERANCE_DEG
                && Math.abs(motor.getVelocity()) < STOP_SPEED_TICKS_PER_SEC;
    }

    /** Go to the nearest HOLD: forward only. Already within the tolerance: just stop. */
    public void hold() {
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
        state = State.JOG;
        jogPower = Math.max(-1.0, Math.min(1.0, power));
    }

    /** One full forward turn, from the nearest HOLD to the next. Ignored until calibrated, or while faulted. */
    public void startSalvo() {
        if (!isCalibrated() || state == State.STALLED) return;
        beginForwardMove(State.SALVO, nearestHoldTicks() + TICKS_PER_REV);
    }

    /** Reverse about REJECT_DEG to reject an element, finishing on the nearest HOLD behind. Ignored until calibrated, or while faulted. */
    public void startReject() {
        if (!isCalibrated() || state == State.STALLED) return;
        targetTicks = Math.round((getUnwrappedTicks() - REJECT_DEG * TICKS_PER_REV / 360.0) / TICKS_PER_REV) * TICKS_PER_REV;
        state = State.REJECT;
        moveTimer.reset();
        resetProgress();
    }

    public void clearStall() {
        stalling = false;
        faultReason = "";
        stopHere();
    }

    private void beginForwardMove(State moveState, double target) {
        targetTicks = target;
        state = moveState;
        moveTimer.reset();
        resetProgress();
    }

    private void stopHere() {
        state = State.HOLD;
        motor.setPower(0.0);
        restTimer.reset();
        resetProgress();
    }

    private void resetProgress() {
        progressBest = Double.NEGATIVE_INFINITY;
        progressTimer.reset();
    }

    /** metricDeg must INCREASE as the move succeeds. Faults if it stops gaining. */
    private void monitorProgress(double metricDeg) {
        if (metricDeg > progressBest + PROGRESS_MIN_GAIN_DEG) {
            progressBest = metricDeg;
            progressTimer.reset();
        } else if (progressTimer.seconds() > PROGRESS_TIMEOUT_S) {
            fault("not converging: check ENCODER_SIGN and gateDirection, or a jam");
        }
    }

    private void fault(String reason) {
        faultReason = reason;
        state = State.STALLED;
        motor.setPower(0.0);
    }

    /** Call once per loop. */
    public void update() {
        double unwrapped = getUnwrappedTicks();
        double ticksPerDeg = TICKS_PER_REV / 360.0;
        boolean moving = state == State.SALVO || state == State.RETURN || state == State.REJECT;

        if (moving && !isCalibrated()) {
            stopHere();
            moving = false;
        }

        switch (state) {
            case HOLD:
                motor.setPower(0.0);
                if (isCalibrated() && AUTO_CORRECT_MAX_DEG > 0.0 && restTimer.seconds() > 0.3
                        && Math.abs(motor.getVelocity()) < STOP_SPEED_TICKS_PER_SEC) {
                    double error = getHoldErrorDeg();
                    if (error > HOLD_TOLERANCE_DEG && error <= AUTO_CORRECT_MAX_DEG) hold();
                }
                break;
            case SALVO:
            case RETURN: {
                double remainingDeg = (targetTicks - unwrapped) / ticksPerDeg;
                if (remainingDeg <= STOP_EARLY_DEG) {
                    stopHere();
                } else {
                    motor.setPower(APPROACH_DEG > 0.0 && remainingDeg <= APPROACH_DEG ? APPROACH_POWER : SWEEP_POWER);
                    monitorProgress(unwrapped / ticksPerDeg);
                }
                break;
            }
            case REJECT: {
                double remainingDeg = (unwrapped - targetTicks) / ticksPerDeg;
                if (remainingDeg <= STOP_EARLY_DEG) {
                    stopHere();
                } else {
                    motor.setPower(-(APPROACH_DEG > 0.0 && remainingDeg <= APPROACH_DEG ? APPROACH_POWER : REJECT_POWER));
                    monitorProgress(-unwrapped / ticksPerDeg);
                }
                break;
            }
            case JOG:
                motor.setPower(jogPower);
                break;
            case STALLED:
                motor.setPower(0.0);
                break;
        }

        detectStall();
    }

    private void detectStall() {
        boolean commanded = state == State.SALVO || state == State.RETURN || state == State.REJECT
                || (state == State.JOG && Math.abs(jogPower) >= STALL_MIN_JOG_POWER);
        boolean blocked = commanded && moveTimer.seconds() > STALL_GRACE_S
                && getCurrentAmps() > STALL_CURRENT_A && Math.abs(motor.getVelocity()) < STALL_MIN_TICKS_PER_SEC;
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

    public void log(Telemetry telemetry) {
        telemetry.addData("Gate state", state + (referenceFromSaved ? "   (zero from saved auto reference)" : "   (zero assumed at init)"));
        telemetry.addData("Gate angle (deg from HOLD)", isCalibrated() ? String.format("%.1f", getAngleDeg()) : "not calibrated (TICKS_PER_REV = 0)");
        telemetry.addData("Gate ticks (unwrapped)", "%.0f   speed %.0f ticks/s", getUnwrappedTicks(), getVelocityTicksPerSec());
        telemetry.addData("Error to nearest HOLD (deg)", Double.isNaN(getHoldErrorDeg()) ? "-"
                : String.format("%+.1f   (+ = short of it, - = past it)", getHoldErrorDeg()));
        telemetry.addData("Gate power / current", "%+.2f / %.2f A", motor.getPower(), getCurrentAmps());
        telemetry.addData("Gate ENCODER_SIGN", "%+.0f", ENCODER_SIGN);
        if (isStalled()) telemetry.addLine("GATE FAULT (Options clears): " + faultReason);
    }

    /** Action: run one salvo and finish when the gate has stopped, or has faulted, or times out. */
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
                packet.put("gate/angleDeg", getAngleDeg());
                boolean finished = state == State.HOLD || state == State.STALLED;
                return !finished && timer.seconds() < timeoutSeconds;
            }
        };
    }
}
