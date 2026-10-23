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
 *   1. The intake runs only while the gate is at HOLD (HOLD limits capacity to 4). Held while the gate is away,
 *      it waits and starts when the gate returns to HOLD.
 *   2. Deploying the depollinator turns the intake on; stowing turns it off unless the driver holds intake.
 *      If the gate is away from HOLD, the ramp deploys and the intake waits (rule 1 wins).
 *   3. The flywheel must be ready (first ball only) before a salvo. Fire is queued: it spins up if needed and
 *      fires the moment the flywheel is ready, with a timeout.
 *   4. A reject is a gate reverse plus an intake outtake for the whole reject move.
 *   5. A stow requested under a flower is remembered and happens once the pose shows the robot is clear.
 *   Also: the flywheel stops by itself shortly after a salvo ends.
 *
 * Any mechanism the robot does not have yet (no matching hardware-config entry) is null and skipped, so this
 * works on a partially built chassis. If the gate is not calibrated yet, rule 1 cannot be enforced and is
 * disabled with a warning. Call {@link #update(Pose2d)} once per loop; autos pass it to
 * Actions.runBlocking as the per-loop callback.
 *
 * Hardware config names: motors "intake", "fly_motor_enc", "gate"; servos "depolLeft", "depolRight".
 */
@Config
public class Mechanisms {

    public static double FIRE_TIMEOUT_SEC = 3.0;
    public static double FLYWHEEL_STOP_DELAY_SEC = 0.5;
    public static double NOTICE_SEC = 4.0;

    public final Intake intake;
    public final Flywheel flywheel;
    public final TransportGate gate;
    public final Depollinator depollinator;

    private boolean intakeRequested = false;
    private boolean manualOuttake = false;
    private boolean fireQueued = false;
    private boolean salvoActive = false;
    private boolean rejectActive = false;
    private boolean spinRequested = false;
    private boolean stopScheduled = false;
    private boolean stowPending = false;
    private boolean useShotMap = true;
    private double manualSetpoint = 400.0;
    private HiveTarget openCell = HiveTarget.BLUE_REAR;

    private final ElapsedTime fireTimer = new ElapsedTime();
    private final ElapsedTime stopTimer = new ElapsedTime();
    private final ElapsedTime noticeTimer = new ElapsedTime();
    private String notice = "";
    private final List<String> warnings = new ArrayList<>();
    private double lastDistance = Double.NaN;
    private double lastSetpoint = 0.0;

    public Mechanisms(Intake intake, Flywheel flywheel, TransportGate gate, Depollinator depollinator) {
        this.intake = intake;
        this.flywheel = flywheel;
        this.gate = gate;
        this.depollinator = depollinator;
    }

    /** Builds whichever mechanisms the hardware configuration contains. */
    public static Mechanisms fromHardware(HardwareMap hardwareMap, TransportGate.Reference gateReference) {
        Intake intake = hardwareMap.tryGet(DcMotorEx.class, "intake") != null ? new Intake(hardwareMap) : null;
        Flywheel flywheel = hardwareMap.tryGet(DcMotorEx.class, "fly_motor_enc") != null ? new Flywheel(hardwareMap) : null;
        TransportGate gate = hardwareMap.tryGet(DcMotorEx.class, "gate") != null
                ? new TransportGate(hardwareMap, gateReference) : null;
        Depollinator depollinator = hardwareMap.tryGet(Servo.class, "depolLeft") != null
                && hardwareMap.tryGet(Servo.class, "depolRight") != null ? new Depollinator(hardwareMap) : null;
        return new Mechanisms(intake, flywheel, gate, depollinator);
    }

    public String describe() {
        return "intake " + (intake != null) + ", flywheel " + (flywheel != null)
                + ", gate " + (gate != null) + ", depollinator " + (depollinator != null);
    }

    // ---- commands ----

    public void setIntakeRequested(boolean requested) {
        intakeRequested = requested;
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

    public void stopFlywheel() {
        spinRequested = false;
        stopScheduled = false;
    }

    public void adjustManualSetpoint(double delta) {
        manualSetpoint = Math.max(0.0, manualSetpoint + delta);
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
            if (!gate.isCalibrated()) warnings.add("Gate not calibrated: intake interlock and salvos disabled");
            if (gate.isStalled()) warnings.add("Gate STALLED: clear it before the next move");
            boolean gateIdle = gate.getState() == TransportGate.State.HOLD && gate.atHold();
            if (rejectActive && gateIdle) rejectActive = false;
            if (salvoActive && gateIdle) {
                salvoActive = false;
                stopScheduled = true;
                stopTimer.reset();
            }
            if (gate.isStalled()) {
                salvoActive = false;
                rejectActive = false;
            }
        }

        // rule 3: queued fire
        if (fireQueued && flywheel != null && gate != null) {
            if (!gate.isSweepConfigured()) {
                fireQueued = false;
                notice("Fire dropped: set the gate's TICKS_PER_REV and SWEEP_END_DEG first.");
            } else if (fireTimer.seconds() > FIRE_TIMEOUT_SEC) {
                fireQueued = false;
                notice("Fire dropped: the flywheel was not ready in time.");
            } else {
                flywheel.setTargetVelocity(lastSetpoint);
                if (flywheel.isReady() && gate.atHold()) {
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

        // rules 1, 2 and 4: what the intake does
        if (intake != null) {
            boolean gateAllows = gate == null || !gate.isCalibrated() || gate.atHold();
            boolean wantIn = intakeRequested || (depollinator != null && depollinator.isDeployedTarget());
            Intake.Mode mode = Intake.Mode.OFF;
            if (rejectActive || manualOuttake) mode = Intake.Mode.OUT;
            else if (wantIn && gateAllows) mode = Intake.Mode.IN;
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

    /** Action: ask for the intake on or off (rule 1 still applies). Finishes in the same loop. */
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
