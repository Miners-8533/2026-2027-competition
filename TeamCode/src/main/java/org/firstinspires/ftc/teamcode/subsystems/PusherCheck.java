package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The pusher check: with the track EMPTY and the arm at rest after a salvo, park the arm in front of CS4 and see whether the
 * sensor sees it. If the belt has skipped, the arm is not where the encoder says and the sensor sees nothing; the check
 * then finds the arm and corrects the pusher's zero, and finally returns the arm to HOLD.
 *
 * Measured on ROBOT_1 (2026-11-01, Pusher Sensor Matrix): the arm's center is {@link #ARM_CENTER_DEG} ahead of HOLD
 * (about 33 deg BEHIND it), and the arc it covers is about 28 deg wide. The pusher pulley has {@link #PULLEY_TEETH} teeth, so
 * one skipped tooth is 18 deg: more than the arc's +/-14 deg half-width, so a parked look sees it as "arm not seen", and slips come
 * in whole teeth.
 *
 * Stages: (1) reverse from the rest position to the expected arc center and LOOK (wait for fresh CS4 reads, then ask the
 * element counter); seen = the zero is right. (2) If not seen, try the positions a whole number of teeth either side
 * (+1, -1, +2, -2, +3, -3): where the arm is seen, the zero was off by that many teeth, so it is corrected there. (3) If it is
 * still not seen, one slow forward sweep of about SWEEP_DEG reading CS4 directly finds the arc's edges, the zero is corrected from
 * their center, and the parked look is repeated (steps +/-1, +/-2 allowed). (4) Not found: LOST (the zero cannot be trusted).
 * Finally the arm is sent forward to HOLD.
 *
 * It needs the element counter (CS4) and the track empty, which the coordinator checks. Drive it with {@link #start()} and
 * {@link #update()} once per loop (before the pusher's own update); {@link #abort} sends the arm home if something else needs it.
 */
@Config
public class PusherCheck {

    /** Where CS4 sees the arm's center, in degrees ahead of HOLD, at zero speed. Measured 2026-11-01: about 326.7 (33 deg behind HOLD). */
    public static double ARM_CENTER_DEG = 326.7;
    public static int PULLEY_TEETH = 20;
    /** How many whole teeth either side of the expected center to try before sweeping. */
    public static int MAX_TEETH = 3;
    /** The arm must have been still this long before the first look at a position. */
    public static double SETTLE_SEC = 0.25;
    /** Fresh CS4 reads (about 160 ms apart) needed before a look is judged. */
    public static int LOOK_FRESH_READS = 3;
    public static double LOOK_TIMEOUT_SEC = 1.5;
    public static double SWEEP_POWER = 0.15;
    public static double SWEEP_DEG = 400.0;
    /** The sensor reports the arm this long after it is there; the sweep subtracts speed times this from the edges. Calibrate. */
    public static double SWEEP_LATENCY_SEC = 0.1;
    /** The whole check gives up (LOST) after this long. */
    public static double MAX_TOTAL_SEC = 25.0;

    public enum Stage { IDLE, SEEK, LOOK, SWEEP, RETURN }

    public enum Result { NONE, OK, TEETH, SWEEP, LOST, ABORTED }

    private final Pusher pusher;
    private final ElementCounter counter;

    private Stage stage = Stage.IDLE;
    private Result result = Result.NONE;
    private String detail = "no check yet";
    private int teethCorrected = 0;
    private boolean finishedFlag = false;
    private boolean lost = false;

    private int[] candidates = new int[0];
    private int candidateIndex = 0;
    private boolean viaSweep = false;
    private double baseTicks = 0.0;
    private Result pendingResult = Result.OK;
    private int readStart = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime total = new ElapsedTime();
    private boolean returnStarted = false;

    private boolean sweepSeen = false;
    private double sweepRiseTicks = 0.0;
    private double sweepRiseSpeed = 0.0;

    public PusherCheck(Pusher pusher, ElementCounter counter) {
        this.pusher = pusher;
        this.counter = counter;
    }

    public boolean isActive() {
        return stage != Stage.IDLE;
    }

    public Result lastResult() {
        return result;
    }

    public String lastDetail() {
        return detail;
    }

    /** True once after a check ends (any result), so the caller can report it. */
    public boolean takeFinished() {
        boolean f = finishedFlag;
        finishedFlag = false;
        return f;
    }

    /** The zero cannot be trusted: the last check did not find the arm. */
    public boolean isLost() {
        return lost;
    }

    /** A short code for the log: the stage while running, else the last result. */
    public String statusCode() {
        if (stage != Stage.IDLE) return stage.name().toLowerCase(Locale.US);
        switch (result) {
            case OK: return "ok";
            case TEETH: return "teeth" + (teethCorrected >= 0 ? "+" : "") + teethCorrected;
            case SWEEP: return "sweep";
            case LOST: return "LOST";
            case ABORTED: return "aborted";
            default: return "-";
        }
    }

    private double toothDeg() {
        return 360.0 / PULLEY_TEETH;
    }

    private double ticksPerDeg() {
        return Pusher.TICKS_PER_REV / 360.0;
    }

    /** Starts a check. False if the arm is not at rest, the pusher is not calibrated or the counter is not reading. */
    public boolean start() {
        if (isActive() || !pusher.isCalibrated() || !pusher.isStopped() || !counter.isValid()) return false;
        pusher.setAutoCorrect(false);
        total.reset();
        viaSweep = false;
        teethCorrected = 0;
        int n = Math.max(0, MAX_TEETH);
        candidates = new int[1 + 2 * n];
        for (int k = 0; k < n; k++) {
            candidates[1 + 2 * k] = k + 1;
            candidates[2 + 2 * k] = -(k + 1);
        }
        candidateIndex = 0;
        baseTicks = pusher.getNearestHoldTicks();
        RunLog.event(String.format(Locale.US, "PUSHER CHECK started: parking at %.1f deg ahead of HOLD (%.1f deg from the nearest HOLD)",
                ARM_CENTER_DEG, ARM_CENTER_DEG - 360.0));
        seekCandidate();
        return true;
    }

    /** Ends the check early and sends the arm forward to HOLD. */
    public void abort(String why) {
        if (!isActive()) return;
        pusher.abortSeek();
        pusher.setAutoCorrect(true);
        pusher.hold();
        stage = Stage.IDLE;
        result = Result.ABORTED;
        detail = why;
        finishedFlag = true;
        RunLog.event("PUSHER CHECK aborted: " + why);
    }

    private void finish(Result r, String text) {
        stage = Stage.IDLE;
        result = r;
        detail = text;
        finishedFlag = true;
        lost = r == Result.LOST || (lost && r == Result.ABORTED);
        pusher.setAutoCorrect(true);
        RunLog.event("PUSHER CHECK " + r + ": " + text);
    }

    private void seekCandidate() {
        int j = candidates[candidateIndex];
        double target = baseTicks + (ARM_CENTER_DEG - 360.0 + j * toothDeg()) * ticksPerDeg();
        pusher.startSeek(target, Pusher.SEEK_POWER);
        stage = Stage.SEEK;
    }

    /** Call once per loop, before the pusher's own update. */
    public void update() {
        if (!isActive()) return;
        if (total.seconds() > MAX_TOTAL_SEC) {
            pusher.abortSeek();
            finish(Result.LOST, "timed out after " + MAX_TOTAL_SEC + " s");
            return;
        }
        if (pusher.isStalled()) {
            finish(Result.LOST, "the pusher faulted during the check");
            return;
        }
        if (!counter.isValid() && stage != Stage.RETURN) {
            abort("the element counter stopped reading");
            return;
        }
        switch (stage) {
            case SEEK:
                if (pusher.getState() == Pusher.State.SEEK) return;
                if (pusher.isStopped()) {
                    stage = Stage.LOOK;
                    timer.reset();
                    readStart = counter.readCount(ElementCounter.ARM_SENSOR);
                }
                return;
            case LOOK: {
                boolean enough = counter.readCount(ElementCounter.ARM_SENSOR) - readStart >= LOOK_FRESH_READS && timer.seconds() >= SETTLE_SEC;
                if (!enough && timer.seconds() < LOOK_TIMEOUT_SEC) return;
                judgeLook();
                return;
            }
            case SWEEP:
                sweep();
                return;
            case RETURN:
                if (!returnStarted) {
                    returnStarted = true;
                    pusher.hold();
                    return;
                }
                if (pusher.isStopped()) {
                    String text = pendingResult == Result.OK ? "the arm is where the zero says"
                            : pendingResult == Result.TEETH ? String.format(Locale.US, "slip of %+d tooth(s) (%+.0f deg) found and the zero corrected", teethCorrected, teethCorrected * toothDeg())
                            : "found by a sweep and the zero corrected (a slip of more than " + MAX_TEETH + " teeth, or lost); check the arm";
                    finish(pendingResult, text);
                }
                return;
            default:
        }
    }

    private void judgeLook() {
        boolean seen = counter.pusherArmSeen();
        int j = candidates[candidateIndex];
        if (seen) {
            if (j != 0 || viaSweep) {
                // the arm is in front of CS4 here, so this is the expected center: redefine the zero to say so
                if (j != 0) teethCorrected = viaSweep ? teethCorrected + j : j;
                pusher.resyncAngle(pusher.getUnwrappedTicks(), ARM_CENTER_DEG);
            }
            pendingResult = viaSweep ? Result.SWEEP : (j == 0 ? Result.OK : Result.TEETH);
            RunLog.event(String.format(Locale.US, "PUSHER CHECK: arm seen at %+d tooth/teeth from the expected center", j));
            returnStarted = false;
            stage = Stage.RETURN;
            return;
        }
        candidateIndex++;
        if (candidateIndex < candidates.length) {
            seekCandidate();
        } else if (!viaSweep) {
            RunLog.event("PUSHER CHECK: not seen within " + MAX_TEETH + " teeth either way; sweeping");
            startSweep();
        } else {
            finish(Result.LOST, "the arm was not seen after the sweep either: the sensor, the arm or the zero is wrong");
        }
    }

    private void startSweep() {
        sweepSeen = false;
        pusher.startSeek(pusher.getUnwrappedTicks() + SWEEP_DEG * ticksPerDeg(), SWEEP_POWER);
        stage = Stage.SWEEP;
    }

    private void sweep() {
        boolean seen = counter.armSeenNow();
        double unwrapped = pusher.getUnwrappedTicks();
        double speed = pusher.getVelocityTicksPerSec();
        if (seen && !sweepSeen) {
            sweepSeen = true;
            sweepRiseTicks = unwrapped;
            sweepRiseSpeed = speed;
        } else if (!seen && sweepSeen) {
            // the arc has been crossed: its center, less the sensor's delay, is where the arm truly passed CS4
            double riseTrue = sweepRiseTicks - sweepRiseSpeed * SWEEP_LATENCY_SEC;
            double fallTrue = unwrapped - speed * SWEEP_LATENCY_SEC;
            double centerTicks = (riseTrue + fallTrue) / 2.0;
            pusher.abortSeek();
            pusher.resyncAngle(centerTicks, ARM_CENTER_DEG);
            RunLog.event(String.format(Locale.US, "PUSHER CHECK: arc found by the sweep, %.1f deg wide; zero corrected from its center",
                    (fallTrue - riseTrue) / ticksPerDeg()));
            viaSweep = true;
            candidates = new int[]{0, 1, -1, 2, -2};
            candidateIndex = 0;
            baseTicks = pusher.getNearestHoldTicks();
            seekCandidate();
        } else if (pusher.getState() != Pusher.State.SEEK) {
            finish(Result.LOST, "the sweep went around without seeing the arm");
        }
    }
}
