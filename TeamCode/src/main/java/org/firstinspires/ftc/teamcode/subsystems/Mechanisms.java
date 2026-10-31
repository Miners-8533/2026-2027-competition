package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.InstantAction;

import java.util.ArrayList;
import java.util.List;

/**
 * Coordinates the four mechanisms and enforces the cross-actuator rules confirmed in design/mechanisms.md:
 *
 *   1. REMOVED 2026-10-31: the intake is no longer gated on the gate's position. (The gate can rest a few degrees
 *      past HOLD after a salvo, and gating the intake and the next salvo on "within 8 deg of HOLD" locked both out.)
 *   2. Deploying the depollinator turns the intake on; stowing turns it off unless the driver holds intake.
 *   3. The flywheel must be ready (first ball only) before a salvo. Fire is queued: it spins up if needed and
 *      fires the moment the flywheel is ready and the gate has stopped (wherever it rests: a salvo is a full
 *      turn from the nearest HOLD), with a timeout.
 *   4. A reject is a gate reverse plus an intake outtake for the whole reject move.
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
    public final TransportGate gate;
    public final Depollinator depollinator;
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

    public Mechanisms(Intake intake, Flywheel flywheel, TransportGate gate, Depollinator depollinator) {
        this(intake, flywheel, gate, depollinator, null);
    }

    public Mechanisms(Intake intake, Flywheel flywheel, TransportGate gate, Depollinator depollinator, ElementCounter counter) {
        this.intake = intake;
        this.flywheel = flywheel;
        this.gate = gate;
        this.depollinator = depollinator;
        this.counter = counter;
    }

    /** Builds whichever mechanisms the hardware configuration contains. */
    public static Mechanisms fromHardware(HardwareMap hardwareMap, TransportGate.Reference gateReference) {
        Intake intake = hardwareMap.tryGet(DcMotorEx.class, "intake") != null ? new Intake(hardwareMap) : null;
        Flywheel flywheel = hardwareMap.tryGet(DcMotorEx.class, "fly_motor_enc") != null ? new Flywheel(hardwareMap) : null;
        TransportGate gate = hardwareMap.tryGet(DcMotorEx.class, "gate") != null
                ? new TransportGate(hardwareMap, gateReference) : null;
        Depollinator depollinator = hardwareMap.tryGet(Servo.class, "depolLeft") != null
                && hardwareMap.tryGet(Servo.class, "depolRight") != null ? new Depollinator(hardwareMap) : null;
        ElementCounter counter = ElementCounter.tryCreate(hardwareMap);
        return new Mechanisms(intake, flywheel, gate, depollinator, counter);
    }

    public String describe() {
        return "intake " + (intake != null) + ", flywheel " + (flywheel != null)
                + ", gate " + (gate != null) + ", depollinator " + (depollinator != null) + ", element counter " + (counter != null);
    }

    // ---- commands ----

    public void setIntakeRequested(boolean requested) {
        intakeRequested = requested;
    }

    /** Flip the intake on or off (the operator's toggle). A stow leaves it on if it was toggled on. */
    public void toggleIntake() {
        intakeRequested = !intakeRequested;
    }

    public void setManualOuttake(boolean requested) {
        manualOuttake = requested;
    }

    /** Queue a salvo (rule 3). */
    public void requestFire() {
        if (flywheel == null || gate == null) {
            notice("Fire ignored: needs the flywheel and the gate.");
            return;
        }
        if (fireQueued || salvoActive || rejectActive) return;
        fireQueued = true;
        fireTimer.reset();
    }

    /** Start a reject (rule 4). */
    public void requestReject() {
        if (gate == null || !gate.isCalibrated()) {
            notice("Reject ignored: the gate is missing or not calibrated.");
            return;
        }
        if (salvoActive || rejectActive || fireQueued || gate.isStalled()) return;
        gate.startReject();
        rejectActive = true;
    }

    public void requestDeploy() {
        if (depollinator == null) return;
        stowPending = false;
        depollinator.deploy();
    }

    /** Request a stow (rule 5): done at once if clear of a flower, otherwise remembered until clear. */
    public void requestStow() {
        if (depollinator == null) return;
        stowPending = true;
    }

    public void toggleSpin() {
        spinRequested = !spinRequested;
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
        spinRequested = false;
        stopScheduled = false;
    }

    public void adjustManualSetpoint(double delta) {
        manualSetpoint = Math.max(0.0, manualSetpoint + delta);
    }

    /** Which alliance this is, so the element counter knows which nectar is the opponent's. */
    public void setBlueAlliance(boolean blue) {
        blueAlliance = blue;
    }

    public void setOpenCell(HiveTarget cell) {
        openCell = cell;
    }

    public HiveTarget getOpenCell() {
        return openCell;
    }

    public void setUseShotMap(boolean use) {
        useShotMap = use;
    }

    public void clearGateStall() {
        if (gate != null) gate.clearStall();
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
            full = counter.isValid() && counter.isFull();
            if (full) warnings.add("FULL: " + counter.count() + " elements held");
            int opponentAt = counter.opponentNectarAt(blueAlliance);
            if (opponentAt > 0) warnings.add("OPPONENT NECTAR at sensor " + opponentAt + " (CS" + opponentAt + "): reject (B)");
        }

        lastDistance = openCell.distanceFrom(pose);
        boolean mapAvailable = useShotMap && !ShotMap.isEmpty();
        lastSetpoint = mapAvailable ? ShotMap.velocityFor(lastDistance) : manualSetpoint;
        if (flywheel != null && !mapAvailable) warnings.add("Shot map empty: using the manual setpoint");

        // rule 5: remembered stow
        if (depollinator != null) {
            if (!depollinator.isConfigured()) warnings.add("Depollinator positions not measured: it will not move");
            if (stowPending) {
                if (Depollinator.isClearOfFlower(pose)) {
                    depollinator.stow();
                    stowPending = false;
                } else {
                    warnings.add("Stow waiting: robot not clear of a flower yet");
                }
            }
        }

        // gate events and warnings
        if (gate != null) {
            if (!gate.isCalibrated()) warnings.add("Gate not calibrated: salvos and rejects disabled");
            if (gate.isStalled()) warnings.add("Gate STALLED: clear it before the next move");
            // finished means stopped, wherever it came to rest: not "within the HOLD tolerance", which a salvo that
            // lands a few degrees past HOLD never satisfies
            boolean gateIdle = gate.isStopped();
            if (rejectActive && gateIdle) rejectActive = false;
            if (salvoActive && gateIdle) {
                salvoActive = false;
                if (autoStopAfterSalvo) {
                    stopScheduled = true;
                    stopTimer.reset();
                }
            }
            if (gate.isStalled()) {
                salvoActive = false;
                rejectActive = false;
            }
        }

        // rule 3: queued fire
        if (fireQueued && flywheel != null && gate != null) {
            if (!gate.isCalibrated()) {
                fireQueued = false;
                notice("Fire dropped: set the gate's TICKS_PER_REV first.");
            } else if (fireTimer.seconds() > FIRE_TIMEOUT_SEC) {
                fireQueued = false;
                notice("Fire dropped: the flywheel was not ready in time.");
            } else {
                flywheel.setTargetVelocity(lastSetpoint);
                if (flywheel.isReady() && gate.isStopped()) {
                    gate.startSalvo();
                    salvoActive = true;
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

        // rules 2 and 4: what the intake does (not gated on the gate's position)
        if (intake != null) {
            // rule 6: a full robot stops the intake and clears the operator's toggle (they press it again after a salvo)
            boolean stopForFull = full && AUTO_STOP_INTAKE_AT_FULL;
            if (stopForFull && intakeRequested) {
                intakeRequested = false;
                notice("Intake stopped: " + counter.count() + " elements held.");
            }
            boolean wantIn = intakeRequested || (depollinator != null && depollinator.isDeployedTarget());
            Intake.Mode mode = Intake.Mode.OFF;
            if (rejectActive || manualOuttake) mode = Intake.Mode.OUT;
            else if (wantIn && !stopForFull) mode = Intake.Mode.IN;
            intake.setMode(mode);
        }

        if (intake != null) intake.update();
        if (flywheel != null) flywheel.update();
        if (gate != null) gate.update();
    }

    private void notice(String message) {
        notice = message;
        noticeTimer.reset();
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Mechanisms present", describe());
        telemetry.addData("Open cell / distance (in)", "%s / %.1f", openCell, lastDistance);
        telemetry.addData("Flywheel setpoint (ticks/s)", "%.0f   (%s)", lastSetpoint,
                useShotMap && !ShotMap.isEmpty() ? "shot map" : "manual");
        telemetry.addData("Fire", fireQueued ? "QUEUED, waiting for flywheel" : salvoActive ? "SALVO running" : "idle");
        telemetry.addData("Reject", rejectActive ? "running" : "idle");
        if (intake != null) intake.log(telemetry);
        if (flywheel != null) flywheel.log(telemetry);
        if (gate != null) gate.log(telemetry);
        if (depollinator != null) depollinator.log(telemetry);
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
                return depollinator != null && depollinator.isConfigured() && !depollinator.isSettled();
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
                return stowPending || (depollinator != null && depollinator.isConfigured() && !depollinator.isSettled());
            }
        };
    }
}
