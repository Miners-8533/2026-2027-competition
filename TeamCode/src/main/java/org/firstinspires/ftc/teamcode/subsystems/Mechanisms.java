package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.InstantAction;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Coordinates the four mechanisms and enforces the cross-actuator rules confirmed in design/mechanisms.md:
 *
 *   1. REMOVED 2026-10-31: the intake is no longer gated on the pusher's position. (The pusher can rest a few degrees
 *      past HOLD after a salvo, and gating the intake and the next salvo on "within 8 deg of HOLD" locked both out.)
 *   2. Deploying the ramp turns the intake on; stowing turns it off unless the driver holds intake.
 *   3. The flywheel must be ready (first ball only) before a salvo. Launch is queued: it spins up if needed and
 *      launches the moment the flywheel is ready and the pusher has stopped (wherever it rests: a salvo is a full
 *      turn from the nearest HOLD), with a timeout.
 *   4. A reject is a pusher reverse plus an intake outtake for the whole reject move.
 *   5. A stow requested under a flower is remembered and happens once the pose shows the robot is clear.
 *   6. With the element counter (CS1 to CS4), the intake stops by itself when the robot holds CAPACITY (4) elements and
 *      the operator's intake toggle is cleared (they press it again after a salvo). Opponent nectar in the robot only
 *      raises a warning (reject is still the operator's call).
 *   Also: the flywheel stops by itself shortly after a salvo ends (unless setAutoStopAfterSalvo(false), which the
 *   competition TeleOp uses to keep the flywheel running the whole time).
 *
 * Any mechanism the robot does not have yet (no matching hardware-config entry) is null and skipped, so this
 * works on a partially built chassis. Call {@link #update(Pose2d)} once per loop; autos pass it to
 * Actions.runBlocking as the per-loop callback.
 *
 * Naming: the pusher (hardware name "gate") and the ramp (hardware names "depolLeft" and "depolRight"); the hub
 * configuration names were kept.
 *
 * Hardware config names: motors "intake", "fly_motor_enc", "gate"; servos "depolLeft", "depolRight".
 */
@Config
public class Mechanisms {

    public static double LAUNCH_TIMEOUT_SEC = 3.0;
    /** True: while a launch is queued or a salvo runs, and for BURST_LOG_AFTER_SEC after it, the log writes a sample row on every loop (see RunLog.setBurst). */
    public static boolean BURST_LOG_SALVOS = true;
    public static double BURST_LOG_AFTER_SEC = 0.6;
    /** While the track reads empty, a COUNTER EMPTY line is logged this often: the four colour shifts from the stored empty baselines (tools/logs/baseline_health.py reads them). 0 turns it off. */
    public static double HEALTH_LOG_SEC = 1.0;
    /**
     * PUSHER LOCK (2026-11-04, changed 2026-11-05): while the opmode REQUESTS it ({@link #setPusherLockWanted}: the TeleOp asks during a target lock or a Hive Dash, never because the robot is
     * full, since the count of 4 cannot be trusted) and the arm is at HOLD, move the arm LOCK_DEG (90) forward across the intake entrance so the pollen cannot roll back out; the element count
     * is frozen and the intake is OFF while locked (it used to run OUT). A launch goes from the lock (310 deg), a reject unlocks first, and the operator forcing the intake IN (right trigger) unlocks it.
     * When the request has ended for LOCK_RELEASE_DELAY_SEC the arm goes back to HOLD. The autos never request it. False turns the whole lock off.
     */
    /** FALSE since 2026-11-06 (user): the intake is always IN now, so the arm must not lock across the entrance; the lock stays off until the element sensors are more reliable. True brings it back. */
    public static boolean PUSHER_LOCK_ENABLED = false;
    /**
     * AUTOS (2026-11-05, user): the flower gather keeps the intake running IN, whatever the count says, from the ram until the flower salvo is done ({@link #forceIntakeIn}), so pollen still
     * coming off the ramp keep coming in while the robot backs off, turns and aims. False: the old behavior (the intake is requested at the ram and the automatic stop at 4 can end it).
     * A robot that takes in more than it can launch holds the rest for the TeleOp (the user decided against an automatic second salvo).
     */
    public static boolean FLOWER_INTAKE_THROUGH_SALVO = true;
    /** How long the lock request must have been over before the arm is sent back to HOLD (so a quick release and re-press does not move the arm twice). */
    public static double LOCK_RELEASE_DELAY_SEC = 0.5;
    public static double FLYWHEEL_STOP_DELAY_SEC = 0.5;
    public static double NOTICE_SEC = 4.0;
    /** Rule 6: stop the intake when the element counter reports a full robot. */
    public static boolean AUTO_STOP_INTAKE_AT_FULL = true;
    /** After a salvo, when the robot is empty and idle, check the pusher's zero against CS4 and correct a belt slip (see PusherCheck). */
    public static boolean AUTO_PUSHER_CHECK = true;
    /** Also check right after Start when the zero has not been verified since the app started (a power cycle, an install, or a zero set by hand). */
    public static boolean AUTO_STARTUP_CHECK = true;
    /** The check waits this long after the salvo for the arm to rest and the count to read 0, then gives up. */
    public static double CHECK_WAIT_SEC = 2.0;
    public static double CHECK_SETTLE_SEC = 0.4;
    /** After a pusher check, the sensors are kept out of the element count this long (the arm leaving their view reads as an element). */
    public static double CHECK_COUNT_HOLDOFF_SEC = 0.8;
    /** With auto target on, the target cell switches only when the other cell is this much closer (so it does not flip-flop midway). */
    public static double AUTO_TARGET_HYSTERESIS_IN = 6.0;
    /** The arm is "near CS4" (an unclassified reading there is the arm, not an element) from this far behind HOLD to this far past it, degrees. Measured loosely: see design/integration-plan.md. */
    public static double ARM_NEAR_BEHIND_DEG = 70.0;
    /**
     * While the arm is within this many degrees ahead of HOLD, CS4's "other" reading is taken as the edge of the arm, not an element. A salvo
     * leaves the arm resting 40 to 60 deg past HOLD, where CS4 reads "other": with 25 here that phantom counted as 1 element in an empty robot (so
     * the post-salvo pusher check was skipped and the manual sweep refused), or as a false 4th element. Pollen and nectar still count.
     */
    public static double ARM_NEAR_AHEAD_DEG = 70.0;

    public final Intake intake;
    public final Flywheel flywheel;
    public final Pusher pusher;
    public final Ramp ramp;
    /** Counts and identifies the elements with CS1 to CS4; null if those sensors are not in the configuration. */
    public final ElementCounter counter;
    /** Checks the pusher's zero against CS4 after a salvo; null without the pusher or the counter. */
    private final PusherCheck pusherCheck;

    private boolean intakeRequested = false;
    private boolean manualOuttake = false;
    private boolean fullNow = false;
    private int salvosDone = 0;
    private boolean flywheelLostSeen = false;
    private boolean manualCalibration = false;   // the operator's pusher calibration sweep is running: the intake is held off
    private double dashDistanceIn = 0.0;       // > 0 during a Hive Dash: the launch speed is worked out for this distance (the launch spot) from the start of the dash, not for where the robot is now
    private double fixedDistanceIn = 0.0;      // > 0 in the backup mode: the launch distance is this, whatever the pose says
    private boolean manualIntake = false;
    private boolean forcedIntakeIn = false;   // an auto holds the intake IN whatever the count says (forceIntakeIn)
    private boolean intakeAlwaysOn = false;
    private boolean launchQueued = false;
    private boolean bandWaitLogged = false;
    private enum LockState { NONE, LOCKING, LOCKED, UNLOCKING }

    private LockState lockState = LockState.NONE;
    private boolean pendingRejectAfterUnlock = false;
    private boolean unlockForIntake = false;     // the unlock in progress was the operator forcing the intake IN
    private boolean intakeStoppedByFull = false;     // the automatic stop at full cleared the intake request (the toggle is off until it is pressed again)
    private boolean lockWanted = false;              // the opmode asks for the pusher lock (a target lock or a Hive Dash is on)
    private final ElapsedTime lockWantedOffClock = new ElapsedTime();   // since when the request has been off
    private final ElapsedTime lockClock = new ElapsedTime();
    private final ElapsedTime burstTimer = new ElapsedTime();
    private final ElapsedTime healthTimer = new ElapsedTime();
    private boolean baselinesLogged = false;
    private boolean salvoActive = false;
    private boolean rejectActive = false;
    private boolean spinRequested = false;
    private boolean autoStopAfterSalvo = true;   // the competition TeleOp turns this off and keeps the flywheel running
    private boolean stopScheduled = false;
    private boolean stowPending = false;
    private boolean useLaunchMap = true;
    private double manualSetpoint = 1350.0;   // ticks/s; used while the launch map is empty, tuned with the operator's D-pad
    private double speedTrim = 0.0;            // ticks/s added to the launch map's speed, nudged with the operator's D-pad
    private HiveTarget openCell = HiveTarget.BLUE_REAR;
    private boolean blueAlliance = true;
    private boolean autoTarget = false;

    private final ElapsedTime launchTimer = new ElapsedTime();
    private final ElapsedTime stopTimer = new ElapsedTime();
    private final ElapsedTime noticeTimer = new ElapsedTime();
    private String notice = "";
    private final List<String> warnings = new ArrayList<>();
    private double lastDistance = Double.NaN;
    private double lastSetpoint = 0.0;

    // for the run log
    private final List<String> prevWarnings = new ArrayList<>();
    private int prevCount = -1;
    private Pusher.State prevPusherState = null;
    private Intake.Mode prevIntakeMode = null;
    private boolean prevManualOuttake = false;
    private boolean checkPending = false;
    private boolean startupCheckConsidered = false;
    private final ElapsedTime checkTimer = new ElapsedTime();

    // overrides: the operator or driver switches off an automatic behavior that depends on a sensor that has failed
    private boolean counterOverride = false;   // ignore the element counter: no 4-element stop, no element warnings
    private boolean poseOverride = false;      // ignore the pose: the ramp stows without waiting to be clear of a flower
    private double salvoMinSpeed = Double.NaN;

    public Mechanisms(Intake intake, Flywheel flywheel, Pusher pusher, Ramp ramp) {
        this(intake, flywheel, pusher, ramp, null);
    }

    public Mechanisms(Intake intake, Flywheel flywheel, Pusher pusher, Ramp ramp, ElementCounter counter) {
        this.intake = intake;
        this.flywheel = flywheel;
        this.pusher = pusher;
        this.ramp = ramp;
        this.counter = counter;
        this.pusherCheck = pusher != null && counter != null ? new PusherCheck(pusher, counter) : null;
    }

    /** Builds whichever mechanisms the hardware configuration contains. */
    public static Mechanisms fromHardware(HardwareMap hardwareMap, Pusher.Reference pusherReference) {
        return fromHardware(hardwareMap, pusherReference, true);
    }

    /**
     * As above; with commandRampAtInit false the ramp is not commanded when built (the competition TeleOp: G403 allows no powered movement in the
     * transition, and an auto may leave the ramp stuck or deployed). Call {@link #stowRampAtStart()} at Start.
     */
    public static Mechanisms fromHardware(HardwareMap hardwareMap, Pusher.Reference pusherReference, boolean commandRampAtInit) {
        Intake intake = hardwareMap.tryGet(DcMotorEx.class, "intake") != null ? new Intake(hardwareMap) : null;
        Flywheel flywheel = hardwareMap.tryGet(DcMotorEx.class, "fly_motor_enc") != null ? new Flywheel(hardwareMap) : null;
        Pusher pusher = hardwareMap.tryGet(DcMotorEx.class, "gate") != null
                ? new Pusher(hardwareMap, pusherReference) : null;
        Ramp ramp = hardwareMap.tryGet(Servo.class, "depolLeft") != null
                && hardwareMap.tryGet(Servo.class, "depolRight") != null ? new Ramp(hardwareMap, commandRampAtInit) : null;
        ElementCounter counter = ElementCounter.tryCreate(hardwareMap);
        return new Mechanisms(intake, flywheel, pusher, ramp, counter);
    }

    public String describe() {
        return "intake " + (intake != null) + ", flywheel " + (flywheel != null)
                + ", pusher " + (pusher != null) + ", ramp " + (ramp != null) + ", element counter " + (counter != null);
    }

    // ---- commands ----

    public void setIntakeRequested(boolean requested) {
        intakeRequested = requested;
        intakeStoppedByFull = false;
    }

    /** Flip the intake on or off (the operator's toggle). A stow leaves it on if it was toggled on. */
    public void toggleIntake() {
        intakeRequested = !intakeRequested;
        RunLog.event("COMMAND intake toggle -> " + (intakeRequested ? "ON" : "off") + " (" + heldText() + ")");
    }

    /**
     * True (TeleOp - Competition): the intake is ALWAYS IN (user, 2026-11-06), whatever the element count and the pusher lock say; only a reject and the operator's
     * {@link #setManualOuttake} (OUT) run it OUT (a pusher calibration sweep still holds it off); the toggle and AUTO_STOP_INTAKE_AT_FULL are not used. {@link #setManualIntake} (the right
     * trigger) no longer changes the intake: it unlocks a locked pusher. False (the default, as in the autos):
     * the toggle and the ramp decide, and a full robot stops the intake.
     */
    public void setIntakeAlwaysOn(boolean on) {
        intakeAlwaysOn = on;
    }

    /** Hold: the intake runs IN whatever the count says (only with {@link #setIntakeAlwaysOn}). The outtake wins if both are held. */
    public void setManualIntake(boolean requested) {
        // forcing the intake IN while the pusher is locked unlocks it, so the pollen can actually go into the rotary transport (the automatic lock stays off while it is held)
        if (requested && !manualIntake && (lockState == LockState.LOCKED || lockState == LockState.LOCKING) && pusher != null) {
            unlockForIntake = true;
            beginUnlock("the operator forced the intake IN");
        }
        manualIntake = requested;
    }

    public void setManualOuttake(boolean requested) {
        manualOuttake = requested;
        if (requested != prevManualOuttake) {
            prevManualOuttake = requested;
            RunLog.event("COMMAND manual outtake " + (requested ? "held" : "released"));
        }
    }

    /** Queue a salvo (rule 3). */
    public void requestLaunch() {
        if (flywheel == null || pusher == null) {
            notice("Launch ignored: needs the flywheel and the pusher.");
            return;
        }
        if (pusher.isStalled()) {
            notice("Launch ignored: the pusher is faulted.");
            return;
        }
        if (pusherCheck != null && pusherCheck.isActive()) pusherCheck.abort("a launch was requested");
        if (launchQueued || salvoActive) {
            RunLog.event("COMMAND launch ignored: " + (launchQueued ? "already queued" : "salvo running"));
            return;
        }
        if (rejectActive) {
            // the operator's way out of a jam in the other direction (there are no pusher faults): a launch cancels a running reject
            RunLog.event("COMMAND launch cancels the reject");
            rejectActive = false;
            pusher.cancelMove();
        }
        if (pendingRejectAfterUnlock) {
            // a reject waiting for the unlock is cancelled the same way (left set, it would go off at the next unlock, long after)
            RunLog.event("COMMAND launch cancels the reject waiting for the unlock");
            pendingRejectAfterUnlock = false;
        }
        launchQueued = true;
        bandWaitLogged = false;
        launchTimer.reset();
        RunLog.event(String.format(Locale.US, "COMMAND launch queued: flywheel %.0f of %.0f ticks/s, %s",
                flywheel.getFilteredVelocity(), lastSetpoint, heldText()));
    }

    // ---- the pusher lock ----

    public boolean isPusherLocked() {
        return lockState == LockState.LOCKED;
    }

    /**
     * Ask for the pusher lock (the TeleOp: a target lock or a Hive Dash is on) or let it go. The lock only happens when the arm is at HOLD, something is held and nothing else is
     * moving the pusher; when the request has been off for LOCK_RELEASE_DELAY_SEC the arm goes back to HOLD (a salvo releases it by itself). Call every loop.
     */
    public void setPusherLockWanted(boolean wanted) {
        if (wanted != lockWanted) {
            lockWanted = wanted;
            lockWantedOffClock.reset();
            RunLog.event("PUSHER LOCK request " + (wanted ? "ON (target lock or Hive Dash)" : "off") + " (" + heldText() + ", lock " + lockCode() + ")");
        }
    }

    /** For the sample rows: - / locking / LOCKED / unlocking. */
    private String lockCode() {
        return lockState == LockState.NONE ? "-" : lockState == LockState.LOCKED ? "LOCKED" : lockState.name().toLowerCase(Locale.US);
    }

    private String lockText() {
        if (pusher == null) return "no pusher";
        switch (lockState) {
            case LOCKING: return "locking (the arm is moving across the entrance)";
            case LOCKED: return "LOCKED: the arm blocks the entrance, the count is frozen, the intake is off; a launch goes from here, a reject unlocks first";
            case UNLOCKING: return "unlocking";
            default: return PUSHER_LOCK_ENABLED ? "off (locks during a target lock or a Hive Dash)" : "off (the lock is disabled)";
        }
    }

    /** Why the arm cannot be moved to the lock now, or null. */
    private String lockRefusal() {
        if (pusher == null || !pusher.isCalibrated()) return "the pusher is missing or not calibrated";
        if (counter == null) return "there is no element counter";
        if (launchQueued || salvoActive || rejectActive || manualOuttake || manualCalibration) return "the robot is busy";
        if (pusher.isStalled()) return "the pusher is faulted";
        if (pusherCheck != null && (pusherCheck.isActive() || checkPending)) return "a pusher check is running or wanted";
        if (!pusher.atHold()) return "the arm is not at HOLD (it is " + String.format(Locale.US, "%.0f", pusher.getAnglePastHoldDeg()) + " deg past it)";
        return null;
    }

    private void beginLock(String why) {
        counter.setFrozen(true);
        pusher.startLock();
        lockState = LockState.LOCKING;
        lockClock.reset();
        RunLog.event("PUSHER LOCK (" + why + "): " + heldText() + " | " + pusher.getLastCommand());
    }

    private void beginUnlock(String why) {
        pusher.startUnlock();
        lockState = LockState.UNLOCKING;
        lockClock.reset();
        RunLog.event("PUSHER UNLOCK (" + why + "): " + pusher.getLastCommand());
    }

    /** A salvo starts from the lock (310 deg): the count follows the sensors again as the arm sweeps. */
    private void releaseLockForSalvo() {
        if (lockState == LockState.NONE) return;
        RunLog.event("PUSHER LOCK released by the salvo (the arm is " + String.format(Locale.US, "%.1f", pusher.getAnglePastHoldDeg()) + " deg past HOLD)");
        lockState = LockState.NONE;
        // the salvo can start before an unlock in progress has been seen to finish, so what that unlock was for is dropped here (or it would act at the next unlock)
        pendingRejectAfterUnlock = false;
        unlockForIntake = false;
        if (counter != null) counter.setFrozen(false);
        lockClock.reset();
    }

    private void updateLock(boolean full) {
        if (pusher == null || counter == null) return;
        if (!full && lockState == LockState.NONE) intakeStoppedByFull = false;
        switch (lockState) {
            case NONE:
                if (PUSHER_LOCK_ENABLED && lockWanted && counter.count() > 0 && !manualIntake && counter.isValid() && !counterOverride && lockRefusal() == null) {
                    beginLock("a target lock or a Hive Dash is on");
                }
                break;
            case LOCKING:
                if (pusher.isStalled() || (pusher.getState() == Pusher.State.HOLD && pusher.isStopped())) {
                    if (pusher.isLocked()) {
                        RunLog.event(String.format(Locale.US, "PUSHER LOCKED at %.1f deg past HOLD after %.2f s (%s)", pusher.getAnglePastHoldDeg(), lockClock.seconds(), pusher.getLastResult()));
                        lockState = LockState.LOCKED;
                        lockClock.reset();
                    } else {
                        RunLog.event(String.format(Locale.US, "PUSHER LOCK FAILED: the arm rests at %.1f deg past HOLD (%s); the lock is off (the automatic lock tries again when the conditions hold)",
                                pusher.getAnglePastHoldDeg(), pusher.getLastResult()));
                        lockState = LockState.NONE;
                        counter.setFrozen(false);
                        lockClock.reset();
                    }
                }
                break;
            case LOCKED:
                // the request is over (the target lock was released or the dash ended without a launch): back to HOLD so the intake can take more. A queued launch or a salvo releases the lock itself.
                if (!lockWanted && lockWantedOffClock.seconds() >= LOCK_RELEASE_DELAY_SEC && !launchQueued && !salvoActive && !rejectActive) {
                    beginUnlock("the target lock or Hive Dash ended without a launch");
                }
                break;
            case UNLOCKING:
                if (pusher.isStalled() || (pusher.getState() == Pusher.State.HOLD && pusher.isStopped())) {
                    RunLog.event(String.format(Locale.US, "PUSHER UNLOCKED: the arm is %.1f deg past HOLD (%s)", pusher.getAnglePastHoldDeg(), pusher.getLastResult()));
                    lockState = LockState.NONE;
                    counter.setFrozen(false, unlockForIntake);
                    lockClock.reset();
                    unlockForIntake = false;
                    if (pendingRejectAfterUnlock) {
                        pendingRejectAfterUnlock = false;
                        requestReject();
                    }
                }
                break;
            default:
                break;
        }
    }

    /** Start a reject (rule 4). */
    public void requestReject() {
        if (pusher == null || !pusher.isCalibrated()) {
            notice("Reject ignored: the pusher is missing or not calibrated.");
            return;
        }
        if (pusherCheck != null && pusherCheck.isActive()) pusherCheck.abort("a reject was requested");
        if (lockState != LockState.NONE) {
            // the arm is across the entrance: unlock first (a reject is a full reverse turn from a HOLD), then the reject starts by itself
            if (launchQueued) {
                RunLog.event("COMMAND reject cancels the queued launch");
                launchQueued = false;
            }
            pendingRejectAfterUnlock = true;
            if (lockState != LockState.UNLOCKING) beginUnlock("a reject was requested");
            RunLog.event("COMMAND reject waits for the pusher to unlock");
            return;
        }
        if (rejectActive || pusher.isStalled()) {
            RunLog.event("COMMAND reject ignored: " + (pusher.isStalled() ? "pusher stalled" : "reject already running"));
            return;
        }
        if (salvoActive || launchQueued) {
            // the operator's way out of a jam (there are no pusher faults): a reject cancels a queued launch or a running salvo
            RunLog.event("COMMAND reject cancels the " + (salvoActive ? "salvo" : "queued launch"));
            salvoActive = false;
            launchQueued = false;
        }
        pusher.startReject();
        rejectActive = true;
        RunLog.event("COMMAND reject started (" + heldText() + "): " + pusher.getLastCommand());
    }

    public void requestDeploy() {
        if (ramp == null) return;
        stowPending = false;
        ramp.deploy();
        RunLog.event("COMMAND ramp deploy");
    }

    /** Request a stow (rule 5): done at once if clear of a flower, otherwise remembered until clear. */
    public void requestStow() {
        if (ramp == null) return;
        stowPending = true;
        RunLog.event("COMMAND ramp stow requested");
    }

    /**
     * Stow at once, without waiting for the pose to show the robot is clear of a flower. For an auto whose path guarantees
     * the ramp is clear when it calls this (the operator's stow, {@link #requestStow}, checks the pose instead).
     */
    public void requestStowNow() {
        if (ramp == null) return;
        stowPending = false;
        ramp.stow();
        RunLog.event("COMMAND ramp stow now (no flower check)");
    }

    /** True while a ramp exists that has not been commanded yet (built with commandRampAtInit false): its position is unknown. For the INIT screen. */
    public boolean rampPositionUnknown() {
        return ramp != null && ramp.isConfigured() && !ramp.isPositionKnown();
    }

    /** The first ramp command of a TeleOp built with commandRampAtInit false: stow, with no flower check (the position is unknown, the servos may have sagged). */
    public void stowRampAtStart() {
        if (ramp == null) return;
        stowPending = false;
        ramp.stow();
        RunLog.event("COMMAND ramp stow at Start (not commanded in INIT: the position was unknown)");
    }

    public void toggleSpin() {
        spinRequested = !spinRequested;
        RunLog.event("COMMAND flywheel spin -> " + (spinRequested ? "ON" : "off"));
        if (!spinRequested && !launchQueued && !salvoActive) stopScheduled = false;
    }

    /** Ask for the flywheel to run (or not), the same as the operator's spin toggle. */
    public void setSpinRequested(boolean requested) {
        spinRequested = requested;
        if (requested) stopScheduled = false;
    }

    /** True (the default): the flywheel stops by itself shortly after a salvo. False: it keeps running until told to stop. */
    public void setAutoStopAfterSalvo(boolean autoStop) {
        autoStopAfterSalvo = autoStop;
    }

    public void stopFlywheel() {
        if (spinRequested) RunLog.event("COMMAND flywheel stop");
        spinRequested = false;
        stopScheduled = false;
    }

    /** Clears the speed trim (the operator's guarded reset). */
    public void resetSpeedTrim() {
        speedTrim = 0.0;
        RunLog.event("COMMAND speed trim reset -> +0 ticks/s");
    }

    /** The operator's speed keys: the manual setpoint while the launch map is empty or unused, else a trim added to the map's speed. */
    public void adjustManualSetpoint(double delta) {
        if (useLaunchMap && !LaunchMap.isEmpty()) {
            speedTrim += delta;
            RunLog.event(String.format(Locale.US, "COMMAND speed trim -> %+.0f ticks/s on the launch map", speedTrim));
        } else {
            manualSetpoint = Math.max(0.0, manualSetpoint + delta);
            RunLog.event(String.format(Locale.US, "COMMAND manual setpoint -> %.0f ticks/s", manualSetpoint));
        }
    }

    /** Sets the manual setpoint outright (the launch map tuner follows its recorded points this way). Not logged: it may be called every loop. */
    public void setManualSetpoint(double ticksPerSecond) {
        manualSetpoint = Math.max(0.0, ticksPerSecond);
    }

    /** Which alliance this is, so the element counter knows which nectar is the opponent's. */
    public void setBlueAlliance(boolean blue) {
        blueAlliance = blue;
    }

    /**
     * True: the target cell is always the closer of the alliance's own two cells (set by {@link #setBlueAlliance}), with a hysteresis, so the
     * operator never picks a cell. The launch distance and the target lock both use it. False (the default, as in the autos): the cell set
     * with {@link #setOpenCell} stays.
     */
    public void setAutoTarget(boolean auto) {
        autoTarget = auto;
    }

    public void setOpenCell(HiveTarget cell) {
        if (cell != openCell) RunLog.event("OPEN CELL -> " + cell);
        openCell = cell;
    }

    public HiveTarget getOpenCell() {
        return openCell;
    }

    public void setUseLaunchMap(boolean use) {
        useLaunchMap = use;
    }

    /** Override: ignore the element counter (a sensor failed). The intake then runs until the operator turns it off. */
    public void toggleCounterOverride() {
        counterOverride = !counterOverride;
        RunLog.event("OVERRIDE element counter " + (counterOverride ? "ON (ignored)" : "off (used again)"));
    }

    /** Override: ignore the pose (the Pinpoint is unreliable). A stow request is then carried out at once. */
    public void togglePoseOverride() {
        poseOverride = !poseOverride;
        RunLog.event("OVERRIDE pose " + (poseOverride ? "ON (ramp stows without the flower check)" : "off (flower check used again)"));
    }

    /** Override: run the flywheel open loop (its speed sensor failed). */
    public void toggleFlywheelOpenLoop() {
        if (flywheel == null) return;
        flywheel.setOpenLoop(!flywheel.isOpenLoop());
        RunLog.event("OVERRIDE flywheel open loop " + (flywheel.isOpenLoop() ? "ON (speed sensor ignored)" : "off (speed sensor used again)"));
    }

    /** True while the pose override is on (the pose cannot be trusted, so nothing should aim by it). */
    public boolean isPoseOverridden() {
        return poseOverride;
    }

    public boolean isCounterOverridden() {
        return counterOverride;
    }

    /** The active overrides as letters for the log (C counter, P pose, F flywheel open loop), or "-". */
    private String overridesCode() {
        String s = (counterOverride ? "C" : "") + (poseOverride ? "P" : "") + (flywheel != null && flywheel.isOpenLoop() ? "F" : "");
        return s.isEmpty() ? "-" : s;
    }

    /**
     * The operator's command: run a calibration sweep of the pusher now (find the arm with CS4, correct the zero, return to HOLD).
     * Pressing it again while it runs stops it. It turns the arm forward through a whole turn, so it is refused unless the track
     * reads empty and the robot is otherwise idle.
     */
    public void requestPusherCalibration() {
        if (pusherCheck == null || pusher == null) {
            notice("Pusher calibration ignored: it needs the pusher and the element counter.");
            return;
        }
        if (pusherCheck.isActive()) {
            pusherCheck.abort("the operator stopped it");
            return;
        }
        String why = null;
        if (counterOverride) why = "the element counter is overridden";
        else if (counter == null || !counter.isValid()) why = "the element counter is not reading";
        else if (counter.count() > 0) why = counter.count() + " element(s) held: the sweep would launch them (launch or reject them first)";
        else if (launchQueued || salvoActive || rejectActive || manualOuttake) why = "the robot is busy";
        else if (intakeRequested || (ramp != null && ramp.isDeployedTarget())) why = "the intake is running (a sweep could push a new element out); turn it off first";
        if (why == null) why = pusherCheck.startRefusal();
        if (why != null) {
            notice("Pusher calibration refused: " + why);
            return;
        }
        checkPending = false;
        pusherCheck.startCalibration();
        manualCalibration = pusherCheck.isActive();
        RunLog.event("COMMAND pusher calibration sweep (the intake is held off until it ends)");
    }

    public void clearPusherStall() {
        if (pusher != null && pusher.isStalled()) RunLog.event("COMMAND cleared the pusher fault");
        if (pusher != null) pusher.clearStall();
    }

    /**
     * BACKUP MODE (one way, until the opmode ends): the pose cannot be trusted, so the launch speed comes from this fixed distance (the
     * driver is expected to drive to that spot), the target cell stops following the pose, and the warning shows. The speed trim still works.
     */
    public void enterBackupMode(double distanceIn) {
        fixedDistanceIn = distanceIn;
        autoTarget = false;
        poseOverride = true;
        RunLog.event(String.format(Locale.US, "BACKUP MODE: the launch speed is fixed for %.0f in, no pose", distanceIn));
    }

    /**
     * The Hive Dash knows where it will launch from, so the flywheel goes to that speed at once instead of following the distance to the cell all the way along the route (a flywheel that was
     * sent up for a far point arrived above the launch band and the launch waited for it to coast down: 9 times, up to 0.7 s, in the 2026-11-06 log). 0 or less clears it.
     */
    public void setDashLaunchDistance(double distanceIn) {
        double next = Math.max(0.0, distanceIn);
        if ((next > 0.0) != (dashDistanceIn > 0.0)) {
            RunLog.event(next > 0.0 ? String.format(Locale.US, "DASH launch speed fixed for the launch spot, %.1f in from the cell", next) : "DASH launch speed back to the pose distance");
        }
        dashDistanceIn = next;
    }

    public boolean isBackupMode() {
        return fixedDistanceIn > 0.0;
    }

    /** How many salvos have finished this run (a cancelled or dropped one does not count). TeleOp watches it to release the target lock. */
    public int getSalvosDone() {
        return salvosDone;
    }

    /** The flywheel speed (ticks/s) the coordinator last set from the launch map, the trim and the distance (for the log). */
    public double getLastSetpoint() {
        return lastSetpoint;
    }

    public double getSpeedTrim() {
        return speedTrim;
    }

    /** The distance the launch speed was last worked out for (the open cell, the dash's spot or the backup mode's fixed distance), inches. For the competition screen. */
    public double getLastDistance() {
        return lastDistance;
    }

    /** A copy of the current warnings (for the competition screen). */
    public java.util.List<String> getWarnings() {
        return new ArrayList<>(warnings);
    }

    /** The notice while it is still shown (NOTICE_SEC), else an empty string. */
    public String getNotice() {
        return !notice.isEmpty() && noticeTimer.seconds() < NOTICE_SEC ? notice : "";
    }

    /**
     * For the autos (the user, 2026-11-06): moves the arm from HOLD to the lock position (LOCK_DEG, 90) while the flywheel spins up for the PRELOAD, so that salvo turns about 310 deg instead of 400 (median 1.08 s
     * against 1.37 s in the logs) with no time lost: the flywheel takes 1.1 s or more to reach the launch band and the arm needs about 0.6 s. Only for the preload: later salvos already start from the arm's rest
     * about 45 deg past HOLD (1.22 s), and a locked arm would block the stragglers the autos keep the intake open for. The queued salvo waits for the arm to stop, then runs from the lock. Does nothing
     * unless the arm is idle and calibrated.
     */
    public void preparePusherForLaunch() {
        if (pusher == null || !pusher.isCalibrated() || pusher.getState() != Pusher.State.HOLD) return;
        pusher.startLock();
        RunLog.event("COMMAND pusher to the lock position for the preload salvo (310 deg instead of 400): " + pusher.getLastCommand());
    }

    /** The most jam back-offs per pusher move (the autos use Pusher.JAM_RETRIES_AUTO). */
    public void setJamRetries(int n) {
        if (pusher != null) pusher.setMaxJamRetries(n);
    }

    public boolean isRejectActive() {
        return rejectActive;
    }

    public boolean isLaunchBusy() {
        return launchQueued || salvoActive;
    }

    /** True while the element counter reports a full robot (CAPACITY elements, confirmed). False when the counter is overridden or not reading. */
    public boolean isFull() {
        return fullNow;
    }

    // ---- per-loop update ----

    public void update(Pose2d pose) {
        long profiled = LoopProfiler.start();
        try {
            updateAll(pose);
        } finally {
            LoopProfiler.stop("mechanisms", profiled);   // the sections below (mech.*) are inside this one
        }
    }

    private void updateAll(Pose2d pose) {
        warnings.clear();

        // the element counter reads one sensor every READ_PERIOD_MS (never more than one per loop)
        boolean full = false;
        if (counter != null) {
            if (pusher != null && pusher.isCalibrated()) {
                double angle = pusher.getAngleDeg();
                counter.setArmNear(angle <= ARM_NEAR_AHEAD_DEG || angle >= 360.0 - ARM_NEAR_BEHIND_DEG);
            }
            long profiledCounter = LoopProfiler.start();
            counter.update();
            LoopProfiler.stop("mech.counter", profiledCounter);   // one color sensor read every READ_PERIOD_MS, so the maximum is the read
            if (counterOverride) {
                warnings.add("COUNTER OVERRIDE: no 4-element stop and no element warnings; the intake runs until you turn it off");
            } else if (!counter.isValid()) {
                warnings.add("ELEMENT COUNTER NOT READING: there is no 4-element stop (count by eye; hold the left trigger to push extras out)");
            } else {
                full = counter.isFull();
                if (full) warnings.add("FULL: " + counter.count() + " elements held");
                int opponentAt = counter.opponentNectarAt(blueAlliance);
                if (opponentAt > 0) warnings.add("OPPONENT NECTAR at sensor " + opponentAt + " (CS" + opponentAt + "): reject (LB + B)");
            }
        }

        if (counter != null && !baselinesLogged) {
            baselinesLogged = true;
            RunLog.event("COUNTER baselines: " + ElementCounter.describeBaselines() + " | presence ON at shift " + ElementCounter.COLOR_SHIFT_ON + ", OFF below " + ElementCounter.COLOR_SHIFT_OFF);
        }
        if (counter != null && HEALTH_LOG_SEC > 0.0 && counter.isValid() && !counterOverride && !counter.isFrozen() && counter.count() == 0 && healthTimer.seconds() >= HEALTH_LOG_SEC) {
            healthTimer.reset();
            boolean armNear = pusher != null && pusher.isCalibrated() && (pusher.getAngleDeg() <= ARM_NEAR_AHEAD_DEG || pusher.getAngleDeg() >= 360.0 - ARM_NEAR_BEHIND_DEG);
            RunLog.event(String.format(Locale.US, "COUNTER EMPTY shift %.3f %.3f %.3f %.3f | alpha %.3f %.3f %.3f %.3f | dist %.1f %.1f %.1f %.1f | arm %s",
                    counter.colorShift(0), counter.colorShift(1), counter.colorShift(2), counter.colorShift(3),
                    counter.lastAlpha(0), counter.lastAlpha(1), counter.lastAlpha(2), counter.lastAlpha(3),
                    counter.lastDistCm(0), counter.lastDistCm(1), counter.lastDistCm(2), counter.lastDistCm(3), armNear ? "near CS4" : "clear"));
        }

        if (counter != null && counter.isValid() && counter.count() != prevCount) {
            if (prevCount >= 0) RunLog.event("ELEMENTS " + prevCount + " -> " + counter.count() + "  stack " + counter.slotsCompact() + " (CS1..CS4)");
            prevCount = counter.count();
        }

        if (autoTarget && fixedDistanceIn <= 0.0) {
            HiveTarget near = HiveTarget.nearestOf(blueAlliance, pose);
            if (near != openCell && (openCell.isBlue() != blueAlliance
                    || near.distanceFrom(pose) + AUTO_TARGET_HYSTERESIS_IN < openCell.distanceFrom(pose))) {
                setOpenCell(near);
            }
        }
        lastDistance = fixedDistanceIn > 0.0 ? fixedDistanceIn : dashDistanceIn > 0.0 ? dashDistanceIn : openCell.distanceFrom(pose);
        boolean mapAvailable = useLaunchMap && !LaunchMap.isEmpty();
        lastSetpoint = mapAvailable ? Math.max(0.0, LaunchMap.velocityFor(lastDistance) + speedTrim) : manualSetpoint;
        // a dead flywheel encoder: no closed loop, so every launch is the same base power (Flywheel.LOST_BASE_SETPOINT) moved only by the speed trim
        if (flywheel != null && flywheel.isEncoderLost()) lastSetpoint = Math.max(0.0, Flywheel.LOST_BASE_SETPOINT + speedTrim);
        if (flywheel != null && !mapAvailable) warnings.add("Launch map empty: using the manual setpoint");
        if (flywheel != null && flywheel.isEncoderLost()) {
            warnings.add(String.format(Locale.US, "FLYWHEEL ENCODER LOST: open loop at a fixed power for %.0f ticks/s %+.0f (speed trim); every launch is the same, drive to the same spot", Flywheel.LOST_BASE_SETPOINT, speedTrim));
        } else if (flywheel != null && flywheel.isOpenLoop()) warnings.add("FLYWHEEL OPEN LOOP: the speed sensor is ignored, ready is a timer");

        fullNow = full;

        // rule 5: remembered stow
        if (ramp != null) {
            if (!ramp.isConfigured()) warnings.add("Ramp positions not measured: it will not move");
            if (poseOverride) warnings.add(String.format(Locale.US, "BACKUP MODE (no pose): robot-oriented drive, launch speed fixed for %.0f in, no target lock; service the robot", fixedDistanceIn));
            if (stowPending) {
                if (poseOverride || Ramp.isClearOfFlower(pose)) {
                    ramp.stow();
                    stowPending = false;
                    RunLog.event(poseOverride ? "RAMP stowing (pose override)" : "RAMP stowing (clear of the flowers)");
                } else {
                    warnings.add("Stow waiting: robot not clear of a flower yet");
                }
            }
        }

        // pusher events and warnings
        if (pusher != null) {
            if (!pusher.isCalibrated()) warnings.add("Pusher not calibrated: salvos and rejects disabled");
            if (pusher.isStalled()) warnings.add("Pusher STALLED: clear it before the next move");
            if (pusher.isJamUnresolved()) warnings.add("Pusher JAMMED: the arm could not be freed; reject (LB + B) or launch again");
            // finished means stopped, wherever it came to rest: not "within the HOLD tolerance", which a salvo that
            // lands a few degrees past HOLD never satisfies
            boolean pusherIdle = pusher.isStopped();
            if (salvoActive && flywheel != null) salvoMinSpeed = Math.min(salvoMinSpeed, flywheel.getVelocity());
            if (rejectActive && pusherIdle) {
                rejectActive = false;
                RunLog.event("REJECT done: " + pusher.getLastResult() + " | " + pusher.getLastSettle());
            }
            if (salvoActive && pusherIdle) {
                salvoActive = false;
                salvosDone++;
                RunLog.event(String.format(Locale.US, "SALVO done: lowest flywheel speed %.0f ticks/s | %s | %s",
                        salvoMinSpeed, pusher.getLastResult(), pusher.getLastSettle()));
                if (AUTO_PUSHER_CHECK && pusherCheck != null) {
                    checkPending = true;
                    checkTimer.reset();
                }
                if (autoStopAfterSalvo) {
                    stopScheduled = true;
                    stopTimer.reset();
                }
            }
            if (pusher.isStalled()) {
                if (salvoActive || rejectActive) {
                    RunLog.event("PUSHER STALLED during a " + (salvoActive ? "salvo" : "reject") + ": " + pusher.getLastResult());
                }
                salvoActive = false;
                rejectActive = false;
            }
            if (pusher.getState() != prevPusherState) {
                RunLog.event("PUSHER state " + prevPusherState + " -> " + pusher.getState());
                prevPusherState = pusher.getState();
            }
        }

        // the pusher check: after a salvo, once the robot is empty and idle, look for the arm at CS4 and correct a slipped zero
        if (pusherCheck != null && pusher != null) {
            if (!startupCheckConsidered) {
                startupCheckConsidered = true;
                if (AUTO_PUSHER_CHECK && AUTO_STARTUP_CHECK && !Pusher.isZeroVerified()) {
                    checkPending = true;   // the same wait as after a salvo: empty robot, nothing else going on, within CHECK_WAIT_SEC
                    checkTimer.reset();
                    RunLog.event("PUSHER CHECK wanted at start: the zero has not been verified against CS4 since it was set");
                }
            }
            // the intake may be running (the operator's toggle, or the ramp deployed): the check still runs, and aborts if an element shows up
            boolean busy = launchQueued || salvoActive || rejectActive || manualOuttake || counterOverride || pusher.isStalled();
            boolean counterOk = counter != null && counter.isValid();
            counter.ignoreArmSensorCount(pusherCheck.isActive());
            counter.ignoreAllCount(pusherCheck.isSweeping());   // the arm passes CS1 to CS3 in a sweep and reads as an element there
            if (pusherCheck.isActive()) {
                if (busy) pusherCheck.abort("the robot is busy again");
                else if (counterOk && counter.count() > 0) pusherCheck.abort("an element is in the robot");
                else pusherCheck.update();
            } else if (checkPending) {
                if (pusher.isGaveUp()) {
                    checkPending = false;
                    RunLog.event("PUSHER CHECK skipped: the pusher gave up on its last move and stays where it is until the next salvo or reject");
                } else if (busy) {
                    checkPending = false;
                    RunLog.event("PUSHER CHECK skipped: the robot is busy");
                } else if (counterOk && counter.count() == 0 && pusher.isStopped() && checkTimer.seconds() >= CHECK_SETTLE_SEC) {
                    checkPending = false;
                    if (!pusherCheck.start()) RunLog.event("PUSHER CHECK could not start: " + pusherCheck.startRefusal());
                } else if (checkTimer.seconds() > CHECK_WAIT_SEC) {
                    checkPending = false;
                    RunLog.event("PUSHER CHECK skipped: the robot did not read empty within " + CHECK_WAIT_SEC + " s");
                }
            }
            if (pusherCheck.takeFinished()) counter.ignoreCountFor(CHECK_COUNT_HOLDOFF_SEC);
            if (!Pusher.isZeroVerified() && !pusherCheck.isLost() && !pusherCheck.isActive()) {
                warnings.add("Pusher zero not verified against CS4 yet (the first pusher check will do it)");
            }
            if (pusherCheck.isLost()) {
                warnings.add("PUSHER ZERO UNCERTAIN: the pusher check could not find the arm (" + pusherCheck.lastDetail() + ")");
            }
        }

        // rule 3: queued launch
        if (launchQueued && flywheel != null && pusher != null) {
            if (!pusher.isCalibrated()) {
                launchQueued = false;
                notice("Launch dropped: set the pusher's TICKS_PER_REV first.");
            } else if (launchTimer.seconds() > LAUNCH_TIMEOUT_SEC) {
                launchQueued = false;
                notice(pusher.isStalled() ? "Launch dropped: the pusher is faulted."
                        : !pusher.isStopped() ? "Launch dropped: the pusher was still moving."
                        : flywheel.isReady() ? "Launch dropped: the flywheel speed stayed outside the launch band (" + flywheel.launchBandState() + ", " + String.format(Locale.US, "%.0f of %.0f ticks/s", flywheel.getFilteredVelocity(), lastSetpoint) + ")."
                        : "Launch dropped: the flywheel was not ready in time.");
            } else {
                flywheel.setTargetVelocity(lastSetpoint);
                boolean inBand = flywheel.isInLaunchBand();
                if (!inBand && flywheel.isReady() && !bandWaitLogged) {
                    bandWaitLogged = true;
                    RunLog.event(String.format(Locale.US, "LAUNCH held: the flywheel (filtered %.0f ticks/s) is %s %.0f to %.0f; the salvo starts when it is inside (at most %.1f s)",
                            flywheel.getFilteredVelocity(), flywheel.launchBandState(), lastSetpoint * Flywheel.LAUNCH_BAND_LOW_FRACTION, lastSetpoint * Flywheel.LAUNCH_BAND_HIGH_FRACTION, LAUNCH_TIMEOUT_SEC));
                }
                if (inBand && pusher.isStopped()) {
                    if (bandWaitLogged) RunLog.event(String.format(Locale.US, "LAUNCH released: the flywheel is inside the band after %.2f s of waiting", launchTimer.seconds()));
                    releaseLockForSalvo();
                    pusher.startSalvo();
                    salvoActive = true;
                    salvoMinSpeed = flywheel.getVelocity();
                    RunLog.event(String.format(Locale.US, "SALVO started: flywheel %.0f of %.0f ticks/s, %s | %s",
                            flywheel.getFilteredVelocity(), lastSetpoint, heldText(), pusher.getLastCommand()));
                    launchQueued = false;
                    stopScheduled = false;
                }
            }
        }

        updateLock(full);

        if (launchQueued || salvoActive) burstTimer.reset();
        RunLog.setBurst(BURST_LOG_SALVOS && burstTimer.seconds() < BURST_LOG_AFTER_SEC);

        // flywheel target, and its automatic stop after a salvo
        if (flywheel != null) {
            if (stopScheduled && !launchQueued && !salvoActive && stopTimer.seconds() >= FLYWHEEL_STOP_DELAY_SEC) {
                spinRequested = false;
                stopScheduled = false;
            }
            boolean wanted = spinRequested || launchQueued || salvoActive || stopScheduled;
            flywheel.setTargetVelocity(wanted ? lastSetpoint : 0.0);
        }

        // rules 2 and 4: what the intake does (not gated on the pusher's position)
        if (intake != null) {
            // rule 6: a full robot stops the intake and clears the operator's toggle (they press it again after a salvo)
            boolean stopForFull = full && AUTO_STOP_INTAKE_AT_FULL;
            if (stopForFull && intakeRequested && !intakeAlwaysOn) {
                intakeRequested = false;
                intakeStoppedByFull = true;
                notice("Intake stopped: " + counter.count() + " elements held.");
            }
            boolean wantIn = intakeRequested || (ramp != null && ramp.isDeployedTarget());
            Intake.Mode mode = Intake.Mode.OFF;
            if (rejectActive || manualOuttake) mode = Intake.Mode.OUT;
            else if (wantIn && !stopForFull) mode = Intake.Mode.IN;
            if (intakeAlwaysOn) {
                // the TeleOp (user, 2026-11-06): the intake is ALWAYS IN, whatever the element count says (a phantom or a real full no longer stops it) and whatever the pusher's lock is doing;
                // only the operator's forced OUT (left trigger) and a reject run it OUT. A fifth pollen is the operator's to push out with the left trigger.
                mode = (rejectActive || manualOuttake) ? Intake.Mode.OUT : Intake.Mode.IN;
            }
            // an auto's flower gather holds the intake IN through the flower salvo, whatever the count says (a reject or the operator's outtake still wins)
            if (forcedIntakeIn && !rejectActive && !manualOuttake) mode = Intake.Mode.IN;
            // locked or locking: the arm is across the entrance, so the intake is OFF (a pollen must not be pushed against it), whatever it would have been doing; it used to run OUT when locked.
            // (Only outside the always-IN TeleOp, which since 2026-11-06 keeps the intake IN against the locked arm.)
            // The operator's forced outtake and a reject still run it OUT, and forcing it IN unlocks the arm first.
            if (!intakeAlwaysOn && (lockState == LockState.LOCKED || lockState == LockState.LOCKING) && !manualIntake && !manualOuttake && !rejectActive) mode = Intake.Mode.OFF;   // not in the always-IN TeleOp: the user wants it IN even against the locked arm
            if (manualCalibration && (pusherCheck == null || !pusherCheck.isActive())) {
                manualCalibration = false;
                RunLog.event("COMMAND pusher calibration sweep over: the intake runs again");
            }
            if (manualCalibration) mode = Intake.Mode.OFF;   // a manual calibration sweep needs a still track (a launch or reject aborts it first)
            intake.setMode(mode);
            if (mode != prevIntakeMode) {
                RunLog.event("INTAKE mode " + prevIntakeMode + " -> " + mode);
                prevIntakeMode = mode;
            }
        }

        long profiledIntake = LoopProfiler.start();
        if (intake != null) {
            intake.update();
            String intakeNote = intake.takeNote();
            if (intakeNote != null) RunLog.event(intakeNote + " (" + heldText() + ")");
        }
        LoopProfiler.stop("mech.intake", profiledIntake);
        long profiledFlywheel = LoopProfiler.start();
        if (flywheel != null) flywheel.update();
        LoopProfiler.stop("mech.flywheel", profiledFlywheel);
        if (flywheel != null && flywheel.isEncoderLost() && !flywheelLostSeen) {
            flywheelLostSeen = true;
            RunLog.event("FLYWHEEL ENCODER LOST: " + flywheel.getLostReason() + "; open loop for the rest of the run (base power for "
                    + Flywheel.LOST_BASE_SETPOINT + " ticks/s, speed trim " + speedTrim + ")");
            notice("Flywheel encoder lost: open loop at a fixed power. Use the speed trim.");
        }
        long profiledPusher = LoopProfiler.start();
        if (pusher != null) {
            pusher.update();
            String holdReport = pusher.takeHoldReport();
            if (holdReport != null) RunLog.event(holdReport);
            String jamReport = pusher.takeJamReport();
            if (jamReport != null) RunLog.event(jamReport + " (" + heldText() + ")");
            if (pusher.takeJamResolvedNotice()) notice("Pusher jam cleared");
        }
        LoopProfiler.stop("mech.pusher", profiledPusher);

        for (String w : warnings) if (!prevWarnings.contains(w)) RunLog.event("WARNING " + w);
        for (String w : prevWarnings) if (!warnings.contains(w)) RunLog.event("WARNING cleared: " + w);
        prevWarnings.clear();
        prevWarnings.addAll(warnings);
    }

    private void notice(String message) {
        RunLog.event("NOTICE " + message);
        notice = message;
        noticeTimer.reset();
    }

    /** For example "3 held (.PRB)". */
    private String heldText() {
        return counter == null ? "no element counter" : counter.count() + " held (" + counter.slotsCompact() + ")";
    }

    /** The column names of {@link #sampleRow()}, for {@link RunLog#setSampler}. */
    public static final String SAMPLE_COLUMNS = "setpoint,fly_target,fly_vel,fly_ready,pusher_state,pusher_deg,pusher_vel,pusher_amps,intake_mode,ramp,count,slots,launch,warnings,overrides,check,fly_power,fly_amps,pusher_lock,pusher_hold,intake_amps";

    /** One log row of the mechanisms' state, from values already read this loop (no hub reads). */
    public String sampleRow() {
        String launch = launchQueued ? "queued" : salvoActive ? "salvo" : rejectActive ? "reject" : "idle";
        return String.format(Locale.US, "%.0f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%s,%s,%s,%s,%s,%s,%s",
                lastSetpoint,
                flywheel == null ? "" : String.format(Locale.US, "%.0f", flywheel.getTargetVelocity()),
                flywheel == null ? "" : String.format(Locale.US, "%.0f", flywheel.getVelocity()),
                flywheel == null ? "" : flywheel.isReady(),
                pusher == null ? "" : pusher.getState(),
                pusher == null ? "" : String.format(Locale.US, "%.1f", pusher.getAngleDeg()),
                pusher == null ? "" : String.format(Locale.US, "%.0f", pusher.getVelocityTicksPerSec()),
                pusher == null ? "" : String.format(Locale.US, "%.2f", pusher.getCurrentAmps()),
                intake == null ? "" : intake.getMode(),
                ramp == null ? "" : (ramp.isDeployedTarget() ? "deployed" : "stowed"),
                counter == null ? "" : counter.count(),
                counter == null ? "" : counter.slotsCompact(),
                launch, warnings.size(), overridesCode(), pusherCheck == null ? "" : pusherCheck.statusCode(),
                flywheel == null ? "" : String.format(Locale.US, "%.2f", flywheel.getLastPower()),
                flywheel == null ? "" : String.format(Locale.US, "%.2f", flywheel.getCurrentAmps()),
                lockCode(),
                pusher == null ? "" : String.format(Locale.US, "%.2f", pusher.getHoldPower()),
                intake == null ? "" : String.format(Locale.US, "%.2f", intake.getCachedCurrentAmps()));
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Mechanisms present", describe());
        telemetry.addData("Overrides active", overridesCode().equals("-") ? "none" : overridesCode() + "  (C counter, P pose, F flywheel open loop)");
        telemetry.addData("Open cell / distance (in)", "%s / %.1f", openCell, lastDistance);
        telemetry.addData("Flywheel setpoint (ticks/s)", "%.0f   (%s)", lastSetpoint,
                useLaunchMap && !LaunchMap.isEmpty() ? String.format(Locale.US, "launch map %+.0f trim", speedTrim) : "manual");
        telemetry.addData("Pusher lock", lockText());
        telemetry.addData("Launch", launchQueued ? (flywheel != null && !flywheel.isInLaunchBand() ? "QUEUED, waiting: the flywheel is " + flywheel.launchBandState() : "QUEUED, starting") : salvoActive ? "SALVO running" : "idle");
        telemetry.addData("Reject", rejectActive ? "running" : "idle");
        if (intake != null) intake.log(telemetry);
        if (flywheel != null) flywheel.log(telemetry);
        if (pusher != null) pusher.log(telemetry);
        if (pusherCheck != null) {
            telemetry.addData("Pusher check", "%s   last: %s (%s)", pusherCheck.statusCode(), pusherCheck.lastResult(), pusherCheck.lastDetail());
        }
        if (ramp != null) ramp.log(telemetry);
        if (counter != null) counter.log(telemetry);
        for (String w : warnings) telemetry.addLine("WARNING: " + w);
        if (!notice.isEmpty() && noticeTimer.seconds() < NOTICE_SEC) telemetry.addLine("NOTICE: " + notice);
    }

    // ---- actions for autos ----

    /** Action: queue a salvo and finish when it has completed (or been dropped). */
    public Action launch() {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    requestLaunch();
                }
                return launchQueued || salvoActive;
            }
        };
    }

    /** Action: run a reject and finish when it has completed. */
    public Action reject() {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    requestReject();
                }
                return rejectActive;
            }
        };
    }

    /** Action: ask for the intake on or off. Finishes in the same loop. */
    public Action setIntake(boolean on) {
        return new InstantAction(() -> setIntakeRequested(on));
    }

    /**
     * Action (autos): hold the intake IN whatever the element count says (the automatic stop at 4 does not end it), or let go. A reject and the outtake still win. It does nothing unless
     * FLOWER_INTAKE_THROUGH_SALVO is true. Finishes in the same loop.
     */
    public Action forceIntakeIn(boolean on) {
        return new InstantAction(() -> {
            boolean next = on && FLOWER_INTAKE_THROUGH_SALVO;
            if (next != forcedIntakeIn) {
                forcedIntakeIn = next;
                RunLog.event("COMMAND intake forced IN (auto, through the flower salvo) -> " + (next ? "ON" : "off") + " (" + heldText() + ")");
            }
        });
    }

    /** Action: deploy the ramp and finish once it has settled. */
    public Action deploy() {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    requestDeploy();
                }
                return ramp != null && ramp.isConfigured() && !ramp.isSettled();
            }
        };
    }

    /** Action: stow the ramp at once, with no flower check (the path must already be clear), and finish once it has settled. */
    public Action stowNow() {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    requestStowNow();
                }
                return ramp != null && ramp.isConfigured() && !ramp.isSettled();
            }
        };
    }

    /** Action: stow the ramp (after the robot is clear of the flower) and finish once it has settled. */
    public Action stow() {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    requestStow();
                }
                return stowPending || (ramp != null && ramp.isConfigured() && !ramp.isSettled());
            }
        };
    }
}
