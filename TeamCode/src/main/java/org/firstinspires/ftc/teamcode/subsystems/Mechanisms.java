package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2d;
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
 *   3. The flywheel must be ready (first ball only) before a salvo. Fire is queued: it spins up if needed and
 *      fires the moment the flywheel is ready and the pusher has stopped (wherever it rests: a salvo is a full
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

    public static double FIRE_TIMEOUT_SEC = 3.0;
    public static double FLYWHEEL_STOP_DELAY_SEC = 0.5;
    public static double NOTICE_SEC = 4.0;
    /** Rule 6: stop the intake when the element counter reports a full robot. */
    public static boolean AUTO_STOP_INTAKE_AT_FULL = true;

    public final Intake intake;
    public final Flywheel flywheel;
    public final Pusher pusher;
    public final Ramp ramp;
    /** Counts and identifies the elements with CS1 to CS4; null if those sensors are not in the configuration. */
    public final ElementCounter counter;

    private boolean intakeRequested = false;
    private boolean manualOuttake = false;
    private boolean fireQueued = false;
    private boolean salvoActive = false;
    private boolean rejectActive = false;
    private boolean spinRequested = false;
    private boolean autoStopAfterSalvo = true;   // the competition TeleOp turns this off and keeps the flywheel running
    private boolean stopScheduled = false;
    private boolean stowPending = false;
    private boolean useShotMap = true;
    private double manualSetpoint = 1350.0;   // ticks/s; used while the shot map is empty, tuned with the operator's D-pad
    private HiveTarget openCell = HiveTarget.BLUE_REAR;
    private boolean blueAlliance = true;

    private final ElapsedTime fireTimer = new ElapsedTime();
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
    }

    /** Builds whichever mechanisms the hardware configuration contains. */
    public static Mechanisms fromHardware(HardwareMap hardwareMap, Pusher.Reference pusherReference) {
        Intake intake = hardwareMap.tryGet(DcMotorEx.class, "intake") != null ? new Intake(hardwareMap) : null;
        Flywheel flywheel = hardwareMap.tryGet(DcMotorEx.class, "fly_motor_enc") != null ? new Flywheel(hardwareMap) : null;
        Pusher pusher = hardwareMap.tryGet(DcMotorEx.class, "gate") != null
                ? new Pusher(hardwareMap, pusherReference) : null;
        Ramp ramp = hardwareMap.tryGet(Servo.class, "depolLeft") != null
                && hardwareMap.tryGet(Servo.class, "depolRight") != null ? new Ramp(hardwareMap) : null;
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
    }

    /** Flip the intake on or off (the operator's toggle). A stow leaves it on if it was toggled on. */
    public void toggleIntake() {
        intakeRequested = !intakeRequested;
        RunLog.event("OPERATOR intake toggle -> " + (intakeRequested ? "ON" : "off") + " (" + heldText() + ")");
    }

    public void setManualOuttake(boolean requested) {
        manualOuttake = requested;
        if (requested != prevManualOuttake) {
            prevManualOuttake = requested;
            RunLog.event("OPERATOR manual outtake " + (requested ? "held" : "released"));
        }
    }

    /** Queue a salvo (rule 3). */
    public void requestFire() {
        if (flywheel == null || pusher == null) {
            notice("Fire ignored: needs the flywheel and the pusher.");
            return;
        }
        if (fireQueued || salvoActive || rejectActive) {
            RunLog.event("OPERATOR fire ignored: " + (fireQueued ? "already queued" : salvoActive ? "salvo running" : "reject running"));
            return;
        }
        fireQueued = true;
        fireTimer.reset();
        RunLog.event(String.format(Locale.US, "OPERATOR fire queued: flywheel %.0f of %.0f ticks/s, %s",
                flywheel.getFilteredVelocity(), lastSetpoint, heldText()));
    }

    /** Start a reject (rule 4). */
    public void requestReject() {
        if (pusher == null || !pusher.isCalibrated()) {
            notice("Reject ignored: the pusher is missing or not calibrated.");
            return;
        }
        if (salvoActive || rejectActive || fireQueued || pusher.isStalled()) {
            RunLog.event("OPERATOR reject ignored: " + (pusher.isStalled() ? "pusher stalled" : fireQueued ? "fire queued" : "pusher busy"));
            return;
        }
        pusher.startReject();
        rejectActive = true;
        RunLog.event("OPERATOR reject started (" + heldText() + "): " + pusher.getLastCommand());
    }

    public void requestDeploy() {
        if (ramp == null) return;
        stowPending = false;
        ramp.deploy();
        RunLog.event("OPERATOR ramp deploy");
    }

    /** Request a stow (rule 5): done at once if clear of a flower, otherwise remembered until clear. */
    public void requestStow() {
        if (ramp == null) return;
        stowPending = true;
        RunLog.event("OPERATOR ramp stow requested");
    }

    public void toggleSpin() {
        spinRequested = !spinRequested;
        RunLog.event("OPERATOR flywheel spin -> " + (spinRequested ? "ON" : "off"));
        if (!spinRequested && !fireQueued && !salvoActive) stopScheduled = false;
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
        if (spinRequested) RunLog.event("OPERATOR flywheel stop");
        spinRequested = false;
        stopScheduled = false;
    }

    public void adjustManualSetpoint(double delta) {
        manualSetpoint = Math.max(0.0, manualSetpoint + delta);
        RunLog.event(String.format(Locale.US, "OPERATOR manual setpoint -> %.0f ticks/s", manualSetpoint));
    }

    /** Which alliance this is, so the element counter knows which nectar is the opponent's. */
    public void setBlueAlliance(boolean blue) {
        blueAlliance = blue;
    }

    public void setOpenCell(HiveTarget cell) {
        if (cell != openCell) RunLog.event("OPEN CELL -> " + cell);
        openCell = cell;
    }

    public HiveTarget getOpenCell() {
        return openCell;
    }

    public void setUseShotMap(boolean use) {
        useShotMap = use;
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

    public boolean isCounterOverridden() {
        return counterOverride;
    }

    /** The active overrides as letters for the log (C counter, P pose, F flywheel open loop), or "-". */
    private String overridesCode() {
        String s = (counterOverride ? "C" : "") + (poseOverride ? "P" : "") + (flywheel != null && flywheel.isOpenLoop() ? "F" : "");
        return s.isEmpty() ? "-" : s;
    }

    public void clearPusherStall() {
        if (pusher != null && pusher.isStalled()) RunLog.event("OPERATOR cleared the pusher fault");
        if (pusher != null) pusher.clearStall();
    }

    public boolean isFireBusy() {
        return fireQueued || salvoActive;
    }

    // ---- per-loop update ----

    public void update(Pose2d pose) {
        warnings.clear();

        // the element counter reads one sensor every READ_PERIOD_MS (never more than one per loop)
        boolean full = false;
        if (counter != null) {
            counter.update();
            if (counterOverride) {
                warnings.add("COUNTER OVERRIDE (operator L3): no 4-element stop and no element warnings; the intake runs until you turn it off");
            } else if (!counter.isValid()) {
                warnings.add("ELEMENT COUNTER NOT READING: there is no 4-element stop (operator L3 = override and count by eye)");
            } else {
                full = counter.isFull();
                if (full) warnings.add("FULL: " + counter.count() + " elements held");
                int opponentAt = counter.opponentNectarAt(blueAlliance);
                if (opponentAt > 0) warnings.add("OPPONENT NECTAR at sensor " + opponentAt + " (CS" + opponentAt + "): reject (B)");
            }
        }

        if (counter != null && counter.isValid() && counter.count() != prevCount) {
            if (prevCount >= 0) RunLog.event("ELEMENTS " + prevCount + " -> " + counter.count() + "  stack " + counter.slotsCompact() + " (CS1..CS4)");
            prevCount = counter.count();
        }

        lastDistance = openCell.distanceFrom(pose);
        boolean mapAvailable = useShotMap && !ShotMap.isEmpty();
        lastSetpoint = mapAvailable ? ShotMap.velocityFor(lastDistance) : manualSetpoint;
        if (flywheel != null && !mapAvailable) warnings.add("Shot map empty: using the manual setpoint");
        if (flywheel != null && flywheel.isOpenLoop()) warnings.add("FLYWHEEL OPEN LOOP (operator R3): the speed sensor is ignored, ready is a timer");

        // rule 5: remembered stow
        if (ramp != null) {
            if (!ramp.isConfigured()) warnings.add("Ramp positions not measured: it will not move");
            if (poseOverride) warnings.add("POSE OVERRIDE (driver LB + B): the ramp stows without the flower check");
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
                RunLog.event(String.format(Locale.US, "SALVO done: lowest flywheel speed %.0f ticks/s | %s | %s",
                        salvoMinSpeed, pusher.getLastResult(), pusher.getLastSettle()));
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

        // rule 3: queued fire
        if (fireQueued && flywheel != null && pusher != null) {
            if (!pusher.isCalibrated()) {
                fireQueued = false;
                notice("Fire dropped: set the pusher's TICKS_PER_REV first.");
            } else if (fireTimer.seconds() > FIRE_TIMEOUT_SEC) {
                fireQueued = false;
                notice("Fire dropped: the flywheel was not ready in time.");
            } else {
                flywheel.setTargetVelocity(lastSetpoint);
                if (flywheel.isReady() && pusher.isStopped()) {
                    pusher.startSalvo();
                    salvoActive = true;
                    salvoMinSpeed = flywheel.getVelocity();
                    RunLog.event(String.format(Locale.US, "SALVO started: flywheel %.0f of %.0f ticks/s, %s | %s",
                            flywheel.getFilteredVelocity(), lastSetpoint, heldText(), pusher.getLastCommand()));
                    fireQueued = false;
                    stopScheduled = false;
                }
            }
        }

        // flywheel target, and its automatic stop after a salvo
        if (flywheel != null) {
            if (stopScheduled && !fireQueued && !salvoActive && stopTimer.seconds() >= FLYWHEEL_STOP_DELAY_SEC) {
                spinRequested = false;
                stopScheduled = false;
            }
            boolean wanted = spinRequested || fireQueued || salvoActive || stopScheduled;
            flywheel.setTargetVelocity(wanted ? lastSetpoint : 0.0);
        }

        // rules 2 and 4: what the intake does (not gated on the pusher's position)
        if (intake != null) {
            // rule 6: a full robot stops the intake and clears the operator's toggle (they press it again after a salvo)
            boolean stopForFull = full && AUTO_STOP_INTAKE_AT_FULL;
            if (stopForFull && intakeRequested) {
                intakeRequested = false;
                notice("Intake stopped: " + counter.count() + " elements held.");
            }
            boolean wantIn = intakeRequested || (ramp != null && ramp.isDeployedTarget());
            Intake.Mode mode = Intake.Mode.OFF;
            if (rejectActive || manualOuttake) mode = Intake.Mode.OUT;
            else if (wantIn && !stopForFull) mode = Intake.Mode.IN;
            intake.setMode(mode);
            if (mode != prevIntakeMode) {
                RunLog.event("INTAKE mode " + prevIntakeMode + " -> " + mode);
                prevIntakeMode = mode;
            }
        }

        if (intake != null) intake.update();
        if (flywheel != null) flywheel.update();
        if (pusher != null) pusher.update();

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
    public static final String SAMPLE_COLUMNS = "setpoint,fly_target,fly_vel,fly_ready,pusher_state,pusher_deg,pusher_vel,pusher_amps,intake_mode,ramp,count,slots,fire,warnings,overrides";

    /** One log row of the mechanisms' state, from values already read this loop (no hub reads). */
    public String sampleRow() {
        String fire = fireQueued ? "queued" : salvoActive ? "salvo" : rejectActive ? "reject" : "idle";
        return String.format(Locale.US, "%.0f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%s",
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
                fire, warnings.size(), overridesCode());
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Mechanisms present", describe());
        telemetry.addData("Overrides active", overridesCode().equals("-") ? "none" : overridesCode() + "  (C counter, P pose, F flywheel open loop)");
        telemetry.addData("Open cell / distance (in)", "%s / %.1f", openCell, lastDistance);
        telemetry.addData("Flywheel setpoint (ticks/s)", "%.0f   (%s)", lastSetpoint,
                useShotMap && !ShotMap.isEmpty() ? "shot map" : "manual");
        telemetry.addData("Fire", fireQueued ? "QUEUED, waiting for flywheel" : salvoActive ? "SALVO running" : "idle");
        telemetry.addData("Reject", rejectActive ? "running" : "idle");
        if (intake != null) intake.log(telemetry);
        if (flywheel != null) flywheel.log(telemetry);
        if (pusher != null) pusher.log(telemetry);
        if (ramp != null) ramp.log(telemetry);
        if (counter != null) counter.log(telemetry);
        for (String w : warnings) telemetry.addLine("WARNING: " + w);
        if (!notice.isEmpty() && noticeTimer.seconds() < NOTICE_SEC) telemetry.addLine("NOTICE: " + notice);
    }

    // ---- actions for autos ----

    /** Action: queue a salvo and finish when it has completed (or been dropped). */
    public Action fire() {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    requestFire();
                }
                return fireQueued || salvoActive;
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
