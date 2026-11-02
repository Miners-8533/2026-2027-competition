package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The pusher check: with the track EMPTY and the arm at rest after a salvo, park the arm in front of CS4 and see whether the
 * sensor sees it. If the belt has skipped, the arm is not where the encoder says and the sensor sees nothing; the check
 * then finds the arm with one continuous sweep, corrects the pusher's zero, and finally returns the arm to HOLD.
 *
 * Measured on ROBOT_1 (2026-11-01): the arm's center is {@link #ARM_CENTER_DEG} ahead of HOLD (about 33 deg BEHIND it). The
 * pusher pulley has {@link #PULLEY_TEETH} teeth, so one skipped tooth is 18 deg. CS4 classifies the arm over a narrow part of
 * its arc (about 10 deg wide), so a parked look sees any slip of a tooth as "not seen".
 *
 * Stages: (1) reverse from the rest position to the expected center and LOOK (wait for fresh CS4 reads, then ask the element
 * counter); seen = the zero is right (OK). (2) Not seen: sweep forward continuously, reading CS4 on every loop, until the arm has
 * crossed the sensor; the sweep does not give up before a full turn ({@link #SWEEP_DEG}, more than 360), because wherever the
 * belt slipped to the arm must pass the sensor within one turn. The sensor reports the arm late (its refresh and integration time),
 * which shifts the crossing in the direction of travel, so the arm then goes back across the sensor the other way, which shifts it
 * the opposite way: the average of the two crossing centers is the arm's true position with no delay constant needed. The zero is
 * corrected from it. (3) VERIFY: park at the expected center again and look; seen = done (SWEEP). Not seen = another sweep (up to {@link #MAX_SWEEPS} in all),
 * then LOST (the zero cannot be trusted). Finally the arm is sent forward to HOLD.
 *
 * It needs the element counter (CS4) and the track empty, which the coordinator checks. Drive it with {@link #start()} and
 * {@link #update()} once per loop (before the pusher's own update); {@link #abort} sends the arm home if something else needs it.
 */
@Config
public class PusherCheck {

    /** Where CS4 sees the arm's center, in degrees ahead of HOLD, at zero speed. Measured 2026-11-01: about 326.7 (33 deg behind HOLD). */
    public static double ARM_CENTER_DEG = 326.7;
    public static int PULLEY_TEETH = 20;
    /** The arm must have been still this long before a look. */
    public static double SETTLE_SEC = 0.25;
    /** Fresh CS4 reads (about 160 ms apart) needed before a look is judged. */
    public static int LOOK_FRESH_READS = 3;
    public static double LOOK_TIMEOUT_SEC = 1.5;
    /**
     * A seek cuts the power STOP_EARLY_DEG before its target and the arm coasts on: on the robot it came to rest about 4.4 deg PAST the target in the
     * direction it was travelling, both ways (2026-11-01 logs), which put a parked look near the edge of the sensor's view (the arm is
     * classified over about +/-6 deg). The target is moved this far back toward where the arm comes from, so it rests on the center.
     */
    public static double SEEK_COAST_DEG = 4.4;
    public static double SWEEP_POWER = 0.15;
    /** How far a sweep goes before it gives up: a full turn and a margin, so an arc that was in view at the start is crossed whole. */
    public static double SWEEP_DEG = 420.0;
    /** How far the arm reverses after crossing the sensor, to cross it again the other way (more than the arc, the coast and a margin). */
    public static double BACK_DEG = 90.0;
    /** How many sweeps (the first, and up to two more if the verification look fails) before LOST. */
    public static int MAX_SWEEPS = 3;
    /** Only if the return crossing fails: the sensor reports the arm this long after it is there; the forward crossing alone subtracts speed times this. */
    public static double SWEEP_LATENCY_SEC = 0.15;
    /** The whole check gives up (LOST) after this long. */
    public static double MAX_TOTAL_SEC = 25.0;

    public enum Stage { IDLE, SEEK, LOOK, SWEEP, SWEEP_BACK, RETURN }

    public enum Result { NONE, OK, SWEEP, LOST, ABORTED }

    private final Pusher pusher;
    private final ElementCounter counter;

    private Stage stage = Stage.IDLE;
    private Result result = Result.NONE;
    private String detail = "no check yet";
    private boolean finishedFlag = false;
    private boolean lost = false;

    private double baseTicks = 0.0;
    private int sweeps = 0;
    private boolean verifying = false;
    private double correctionDeg = 0.0;
    private int readStart = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime total = new ElapsedTime();
    private boolean returnStarted = false;

    // one crossing of the sensor: armed once CS4 has been seen NOT seeing the arm (so a rise after that is a whole edge)
    private boolean crossArmed = false;
    private boolean crossSeen = false;
    private double crossRise = 0.0;
    private double crossFall = 0.0;
    private double forwardCenter = 0.0;
    private double forwardSpeed = 0.0;

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

    /** The zero cannot be trusted: a check did not find the arm and none has succeeded since. */
    public boolean isLost() {
        return lost;
    }

    /** A short code for the log: the stage while running, else the last result. */
    public String statusCode() {
        if (stage != Stage.IDLE) return stage.name().toLowerCase(Locale.US);
        switch (result) {
            case OK: return "ok";
            case SWEEP: return String.format(Locale.US, "swept%+.0f", correctionDeg);
            case LOST: return "LOST";
            case ABORTED: return "aborted";
            default: return "-";
        }
    }

    private double ticksPerDeg() {
        return Pusher.TICKS_PER_REV / 360.0;
    }

    /** The zero's angle for an unwrapped tick count, in [0, 360), for the log. */
    private double angleOf(double ticks) {
        double a = ticks / ticksPerDeg();
        return ((a % 360.0) + 360.0) % 360.0;
    }

    /** Why a check cannot start now, or null if it can. */
    public String startRefusal() {
        if (isActive()) return "a check is already running";
        if (!pusher.isCalibrated()) return "the pusher is not calibrated (TICKS_PER_REV)";
        if (!pusher.isStopped()) return "the arm is not at rest (pusher state " + pusher.getState() + ")";
        if (!counter.isValid()) return "the element counter is not reading all four sensors";
        return null;
    }

    /** Starts a check. False (see {@link #startRefusal}) if the arm is not at rest, the pusher is not calibrated or the counter is not reading. */
    public boolean start() {
        if (startRefusal() != null) return false;
        pusher.setAutoCorrect(false);
        total.reset();
        sweeps = 0;
        verifying = false;
        correctionDeg = 0.0;
        baseTicks = pusher.getNearestHoldTicks();
        RunLog.event(String.format(Locale.US, "PUSHER CHECK started: parking at %.1f deg ahead of HOLD (%.1f deg from the nearest HOLD)",
                ARM_CENTER_DEG, ARM_CENTER_DEG - 360.0));
        seekCenter();
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

    private void seekCenter() {
        double center = baseTicks + (ARM_CENTER_DEG - 360.0) * ticksPerDeg();
        double direction = center > pusher.getUnwrappedTicks() ? 1.0 : -1.0;
        pusher.startSeek(center - direction * SEEK_COAST_DEG * ticksPerDeg(), Pusher.SEEK_POWER);
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
            case SWEEP_BACK:
                sweepBack();
                return;
            case RETURN:
                if (!returnStarted) {
                    returnStarted = true;
                    pusher.hold();
                    return;
                }
                if (pusher.isStopped()) {
                    if (sweeps == 0) {
                        finish(Result.OK, "the arm is where the zero says");
                    } else {
                        finish(Result.SWEEP, String.format(Locale.US,
                                "the arm was not at the expected position: found by a sweep and the zero corrected by %+.0f deg (about %+.1f teeth)",
                                correctionDeg, correctionDeg / (360.0 / PULLEY_TEETH)));
                    }
                }
                return;
            default:
        }
    }

    private void judgeLook() {
        RunLog.event(String.format(Locale.US, "PUSHER CHECK look: arm at %.1f deg by the zero (expected %.1f), CS4 %s",
                angleOf(pusher.getUnwrappedTicks()), ARM_CENTER_DEG, counter.pusherArmSeen() ? "SEES the arm" : "does not see the arm"));
        if (counter.pusherArmSeen()) {
            RunLog.event(verifying ? "PUSHER CHECK: verified, the arm is at the expected position after the correction"
                    : "PUSHER CHECK: the arm is at the expected position");
            returnStarted = false;
            stage = Stage.RETURN;
            return;
        }
        if (sweeps < MAX_SWEEPS) {
            RunLog.event(verifying ? "PUSHER CHECK: not seen after the correction; sweeping again"
                    : "PUSHER CHECK: the arm is not at the expected position; sweeping to find it");
            startSweep();
        } else {
            finish(Result.LOST, "the arm was not seen at the expected position after " + MAX_SWEEPS + " sweeps: the sensor, the arm or the zero is wrong");
        }
    }

    private void startSweep() {
        sweeps++;
        resetCrossing();
        pusher.startSeek(pusher.getUnwrappedTicks() + SWEEP_DEG * ticksPerDeg(), SWEEP_POWER);
        stage = Stage.SWEEP;
    }

    private void resetCrossing() {
        crossArmed = false;
        crossSeen = false;
    }

    /** Feeds one CS4 reading. True when the arc has been crossed whole (the arm was seen and now is not), with crossRise and crossFall set. */
    private boolean trackCrossing() {
        boolean seen = counter.armSeenNow();
        double unwrapped = pusher.getUnwrappedTicks();
        if (!seen && !crossSeen) {
            crossArmed = true;
        } else if (seen && !crossSeen && crossArmed) {
            crossSeen = true;
            crossRise = unwrapped;
        } else if (!seen && crossSeen) {
            crossFall = unwrapped;
            return true;
        }
        return false;
    }

    /** The forward sweep: until the arm has crossed the sensor, or a full turn and more has gone by. */
    private void sweep() {
        if (trackCrossing()) {
            forwardCenter = (crossRise + crossFall) / 2.0;
            forwardSpeed = Math.abs(pusher.getVelocityTicksPerSec());
            RunLog.event(String.format(Locale.US, "PUSHER CHECK forward crossing: seen from %.1f to %.1f deg by the zero (center %.1f, width %.1f), %.0f deg/s",
                    angleOf(crossRise), angleOf(crossFall), angleOf(forwardCenter), (crossFall - crossRise) / ticksPerDeg(), forwardSpeed / ticksPerDeg()));
            pusher.abortSeek();
            resetCrossing();
            pusher.startSeek(pusher.getUnwrappedTicks() - BACK_DEG * ticksPerDeg(), SWEEP_POWER);
            stage = Stage.SWEEP_BACK;
            return;
        }
        if (pusher.getState() != Pusher.State.SEEK) {
            if (crossSeen && sweeps < MAX_SWEEPS) {
                startSweep();   // the sweep ended with the arm still in view: go once more for the whole crossing
                return;
            }
            finish(Result.LOST, String.format(Locale.US, "a sweep of %.0f deg did not see the arm cross CS4: the sensor or the arm is wrong", SWEEP_DEG));
        }
    }

    /** The return crossing, the other way: the sensor's delay shifts it the opposite way, so the average of the two centers has none. */
    private void sweepBack() {
        if (trackCrossing()) {
            double backCenter = (crossRise + crossFall) / 2.0;
            RunLog.event(String.format(Locale.US, "PUSHER CHECK back crossing: seen from %.1f to %.1f deg by the zero (center %.1f, width %.1f)",
                    angleOf(crossRise), angleOf(crossFall), angleOf(backCenter), (crossRise - crossFall) / ticksPerDeg()));
            pusher.abortSeek();
            applyCenter((forwardCenter + backCenter) / 2.0,
                    String.format(Locale.US, "the forward and back crossings (their centers differ by %.1f deg: the sensor's delay)", (backCenter - forwardCenter) / ticksPerDeg()));
            return;
        }
        if (pusher.getState() != Pusher.State.SEEK) {
            applyCenter(forwardCenter - forwardSpeed * SWEEP_LATENCY_SEC, "the forward crossing only (the return pass did not see the arm)");
        }
    }

    /** The arm was truly at the expected center when the zero said centerTicks: correct the zero, then verify at the expected center. */
    private void applyCenter(double centerTicks, String how) {
        double before = pusher.getUnwrappedTicks();
        pusher.resyncAngle(centerTicks, ARM_CENTER_DEG);
        double shiftDeg = (pusher.getUnwrappedTicks() - before) / ticksPerDeg();
        correctionDeg += shiftDeg;
        RunLog.event(String.format(Locale.US, "PUSHER CHECK: zero corrected by %+.1f deg from %s", shiftDeg, how));
        verifying = true;
        baseTicks = pusher.getNearestHoldTicks();
        seekCenter();
    }
}
