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
import org.firstinspires.ftc.teamcode.motion.PidController;

/**
 * The transport gate: ONE pusher arm that rotates around the curved track the elements ride, driven by
 * one motor with a RELATIVE encoder and NO hard stops (it spins continuously, almost always forward).
 * The back of the arm limits how many elements enter (the 4 cap) while it holds; a salvo sweeps it
 * through the ramp, pushing the stack to the flywheel. Design notes: design/mechanisms.md.
 *
 * Position is an angle on a circle: HOLD is 0 degrees, one revolution is TICKS_PER_REV ticks. Moves go
 * FORWARD only. Holding targets the nearest HOLD ahead of the gate, but a small overshoot (up to
 * OVERSHOOT_TOL_DEG) is pulled back instead of commanding almost a full extra turn. A salvo is about one
 * forward revolution: sweep at SWEEP_DEG_PER_SEC until SWEEP_END_DEG, return at RETURN_DEG_PER_SEC, then
 * position-hold the next HOLD. Reverse exists only for rejecting an opponent's nectar.
 *
 * Localization: the encoder is relative, so the code assumes the gate is at HOLD at init (Reference
 * ASSUME_AT_HOLD) and saves that zero in a static. A TeleOp after an auto uses Reference CONTINUE to keep
 * the auto's zero. The encoder is never reset in code. A fresh session with no saved zero assumes HOLD.
 *
 * TICKS_PER_REV and SWEEP_END_DEG start at 0 = not calibrated; until they are set, only jogging works
 * (use the Transport Gate Test). Call {@link #update()} once per loop.
 */
@Config
public class TransportGate {

    /**
     * Encoder ticks for one full gate revolution. 0 = not calibrated. goBILDA publishes 751.8 counts per output
     * revolution for the 26.9:1 motor and the bench agrees: ten mark-to-mark turns read 9.825 revolutions at 765,
     * which is 751.6 per turn. (An earlier five-turn reading that suggested 765 was a mark-alignment error.)
     */
    public static double TICKS_PER_REV = 751.8;
    /** Gate angle (degrees from HOLD) where the push sweep ends and the faster return starts. 0 = not set. */
    public static double SWEEP_END_DEG = 360.0;

    /**
     * +1 if the SDK encoder count rises when the arm moves forward (the way a positive power drives it), -1 if
     * it falls. The gate bench's encoder polarity opposes its motor's (a positive command counted DOWN), and
     * flipping the motor direction cannot fix that because the SDK reverses power and encoder together. The
     * gate runs WITHOUT the hub's closed-loop encoder modes (those would run away with the wrong sign), so
     * this only affects the ticks, angle and velocity this class reads. Transport Gate Test shows a hint.
     */
    public static double ENCODER_SIGN = -1.0;
    /** Software velocity control for the sweep: feedforward 1 / MAX_TICKS_PER_SEC plus VELOCITY_KP on the error. Placeholders. */
    public static double MAX_TICKS_PER_SEC = 2600.0;
    /**
     * Power added per tick/s of slowdown. 0.0008 adds 0.08 power per 100 ticks/s the load pulls the sweep below
     * target. 0.002 was jerky on the bench: the hub's speed reading is noisy, so a stiff loop chased the noise.
     */
    public static double VELOCITY_KP = 0.0008;
    /** Time constant of the low-pass filter on the speed reading used by the sweep loop. Larger = smoother but slower to respond. */
    public static double VELOCITY_FILTER_TAU_S = 0.1;
    /** A move that gains less than PROGRESS_MIN_GAIN_DEG in PROGRESS_TIMEOUT_S faults (wrong sign or direction, or a jam). */
    public static double PROGRESS_TIMEOUT_S = 1.5;
    public static double PROGRESS_MIN_GAIN_DEG = 2.0;
    /** Hold is only checked for progress while farther than this from its target. */
    public static double PROGRESS_MIN_ERROR_DEG = 15.0;

    // placeholders: tune on the rig
    public static double SWEEP_DEG_PER_SEC = 360.0;
    public static double RETURN_DEG_PER_SEC = 540.0;
    public static double CAPTURE_DEG = 30.0;
    /** Reject reverses this far, then finishes onto the NEAREST HOLD (330 deg back leaves it 30 deg short of a full reverse turn). */
    public static double REJECT_DEG = 330.0;
    public static double REJECT_POWER = 0.6;

    public static double HOLD_KP = 0.012;
    public static double HOLD_KD = 0.0008;
    /** Breakaway power added in the direction of the hold output. Without it the hold stalls short of HOLD. Placeholder: tune. */
    public static double HOLD_KS = 0.08;
    /** Inside this many degrees of the hold target the motor is left alone (avoids hunting). */
    public static double HOLD_DEADBAND_DEG = 1.0;
    public static double HOLD_MAX_POWER = 0.7;
    public static double HOLD_TOLERANCE_DEG = 2.0;
    public static double OVERSHOOT_TOL_DEG = 10.0;

    public static double STALL_CURRENT_A = 6.0;
    public static double STALL_TIME_S = 0.3;
    public static double STALL_GRACE_S = 0.3;
    public static double STALL_MIN_TICKS_PER_SEC = 20.0;
    public static double STALL_MIN_JOG_POWER = 0.15;

    public enum State { HOLD, SALVO, REJECT, JOG, STALLED }

    public enum Reference { ASSUME_AT_HOLD, CONTINUE }

    private static boolean savedReferenceValid = false;
    private static int savedHomeRaw = 0;

    private final DcMotorEx motor;
    private final PidController holdPid = new PidController(HOLD_KP, 0.0, HOLD_KD);
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();
    private int homeRaw;
    private final boolean referenceFromSaved;
    private State state = State.HOLD;
    private double jogPower = 0.0;
    private double salvoStartTicks = 0.0;
    private double rejectTargetTicks = 0.0;
    private double holdErrorDeg = Double.NaN;
    /** When not NaN, the hold target in unwrapped ticks (set by a reject to finish on the nearest HOLD, even if that is behind). */
    private double holdTargetOverrideTicks = Double.NaN;
    private boolean stalling = false;
    private double velocityFiltered = 0.0;
    private final ElapsedTime velocityTimer = new ElapsedTime();
    private String faultReason = "";
    private double progressBest = Double.NEGATIVE_INFINITY;
    private final ElapsedTime progressTimer = new ElapsedTime();

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

    public boolean isSweepConfigured() {
        return isCalibrated() && SWEEP_END_DEG > 0.0;
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
        holdPid.reset();
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

    /** Low-pass filtered speed, signed so that forward is positive, whatever the encoder wiring. Updated in update(). */
    public double getVelocityTicksPerSec() {
        return velocityFiltered;
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /** At HOLD, holding, and not moving. */
    public boolean atHold() {
        return state == State.HOLD && isCalibrated() && Math.abs(holdErrorDeg) <= HOLD_TOLERANCE_DEG
                && Math.abs(motor.getVelocity()) < 30.0;
    }

    /** Return to (and hold) the nearest HOLD ahead. */
    public void hold() {
        state = State.HOLD;
        holdTargetOverrideTicks = Double.NaN;
        holdPid.reset();
        resetProgress();
    }

    /** Open-loop jog for bring-up and calibration; positive = forward. Not for match use. */
    public void jog(double power) {
        if (state != State.JOG) moveTimer.reset();
        state = State.JOG;
        jogPower = Math.max(-1.0, Math.min(1.0, power));
    }

    /** Start one full salvo from the HOLD at or behind the gate. Ignored until calibrated. */
    public void startSalvo() {
        if (!isSweepConfigured() || state == State.STALLED) return;
        double tol = OVERSHOOT_TOL_DEG * TICKS_PER_REV / 360.0;
        salvoStartTicks = Math.floor((getUnwrappedTicks() + tol) / TICKS_PER_REV) * TICKS_PER_REV;
        state = State.SALVO;
        moveTimer.reset();
        resetProgress();
    }

    /** Reverse by REJECT_DEG to reject an element, then finish on the nearest HOLD (the one behind, for 330 deg). Ignored until calibrated. */
    public void startReject() {
        if (!isCalibrated() || state == State.STALLED) return;
        rejectTargetTicks = getUnwrappedTicks() - REJECT_DEG * TICKS_PER_REV / 360.0;
        state = State.REJECT;
        moveTimer.reset();
        resetProgress();
    }

    public void clearStall() {
        stalling = false;
        faultReason = "";
        hold();
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
        double dt = Math.max(1e-3, Math.min(0.1, velocityTimer.seconds()));
        velocityTimer.reset();
        velocityFiltered += dt / (VELOCITY_FILTER_TAU_S + dt) * (ENCODER_SIGN * motor.getVelocity() - velocityFiltered);

        double unwrapped = getUnwrappedTicks();
        double ticksPerDeg = TICKS_PER_REV / 360.0;

        switch (state) {
            case HOLD:
                updateHold(unwrapped, ticksPerDeg);
                break;
            case SALVO: {
                if (!isSweepConfigured()) {
                    hold();
                    break;
                }
                double endTicks = salvoStartTicks + TICKS_PER_REV;
                double sweepEndTicks = salvoStartTicks + SWEEP_END_DEG * ticksPerDeg;
                if (unwrapped >= endTicks - CAPTURE_DEG * ticksPerDeg) {
                    hold();
                    updateHold(unwrapped, ticksPerDeg);
                } else {
                    double degPerSec = unwrapped < sweepEndTicks ? SWEEP_DEG_PER_SEC : RETURN_DEG_PER_SEC;
                    double targetTps = degPerSec * ticksPerDeg;
                    double power = targetTps / MAX_TICKS_PER_SEC + VELOCITY_KP * (targetTps - getVelocityTicksPerSec());
                    motor.setPower(Math.max(0.0, Math.min(1.0, power)));
                    monitorProgress(unwrapped / ticksPerDeg);
                }
                break;
            }
            case REJECT:
                if (unwrapped <= rejectTargetTicks) {
                    hold();
                    holdTargetOverrideTicks = Math.round(unwrapped / TICKS_PER_REV) * TICKS_PER_REV;
                    updateHold(unwrapped, ticksPerDeg);
                } else {
                    motor.setPower(-REJECT_POWER);
                    monitorProgress(-unwrapped / ticksPerDeg);
                }
                break;
            case JOG:
                motor.setPower(jogPower);
                break;
            case STALLED:
                motor.setPower(0.0);
                break;
        }

        detectStall();
    }

    private void updateHold(double unwrapped, double ticksPerDeg) {
        if (!isCalibrated()) {
            holdErrorDeg = Double.NaN;
            motor.setPower(0.0);
            return;
        }
        double overshootTicks = OVERSHOOT_TOL_DEG * ticksPerDeg;
        double targetTicks = Double.isNaN(holdTargetOverrideTicks)
                ? Math.ceil((unwrapped - overshootTicks) / TICKS_PER_REV) * TICKS_PER_REV
                : holdTargetOverrideTicks;
        holdErrorDeg = (targetTicks - unwrapped) / ticksPerDeg;

        holdPid.kP = HOLD_KP;
        holdPid.kD = HOLD_KD;
        holdPid.kStatic = HOLD_KS;
        holdPid.maxOutput = HOLD_MAX_POWER;
        double power = Math.abs(holdErrorDeg) < HOLD_DEADBAND_DEG ? 0.0 : holdPid.calculate(holdErrorDeg);
        motor.setPower(power);
        if (Math.abs(holdErrorDeg) > PROGRESS_MIN_ERROR_DEG && Math.abs(power) > 0.1) {
            monitorProgress(-Math.abs(holdErrorDeg));
        } else {
            resetProgress();
        }
    }

    private void detectStall() {
        boolean commanded = state == State.SALVO || state == State.REJECT
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
        telemetry.addData("Gate ticks (unwrapped)", "%.0f   velocity %.0f ticks/s", getUnwrappedTicks(), getVelocityTicksPerSec());
        telemetry.addData("Gate hold error (deg)", Double.isNaN(holdErrorDeg) ? "-" : String.format("%.1f", holdErrorDeg));
        telemetry.addData("Gate power / current", "%+.2f / %.2f A", motor.getPower(), getCurrentAmps());
        telemetry.addData("Gate ENCODER_SIGN", "%+.0f", ENCODER_SIGN);
        if (isStalled()) telemetry.addLine("GATE FAULT (Options clears): " + faultReason);
    }

    /** Action: run one salvo and finish when the gate is back at HOLD, stalls, or times out. */
    public Action salvo(double timeoutSeconds) {
        return motion(true, timeoutSeconds);
    }

    /** Action: reject (reverse, then return to HOLD); finishes when back at HOLD, stalls, or times out. */
    public Action reject(double timeoutSeconds) {
        return motion(false, timeoutSeconds);
    }

    /** Action: return to and hold HOLD, in the same loop. */
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
                    if (state != (salvo ? State.SALVO : State.REJECT)) return false;   // ignored: not calibrated or stalled
                }
                packet.put("gate/angleDeg", getAngleDeg());
                boolean finished = (state == State.HOLD && atHold()) || state == State.STALLED;
                return !finished && timer.seconds() < timeoutSeconds;
            }
        };
    }
}
