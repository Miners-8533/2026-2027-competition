package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.aiming.LaunchGrid;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The mechanisms coordinator (specs/mechanisms.md): the one object every opmode talks to for the intake, flywheel,
 * pusher, ramp and both element counts. Built from whatever the hub configuration contains; every rule skips a
 * missing mechanism.
 *
 * It is made of parts (target T4). The coordinator itself keeps five jobs: build the actuators that exist, read them
 * once, run the parts in a fixed order, apply their decisions to the actuators, and log. The parts are pure logic:
 * HeldCount, SpeedSelector, LaunchSequencer, the reject sequencer, CheckScheduler, FullAndLock, the intake arbiter,
 * the ramp rules and Warnings.
 *
 * SPEC GAP: the order the parts run in is behavior, and the specs say it is to be "written down from the code".
 * The order here (counts, speed, launch, reject, check, FULL and lock, intake, ramp, flywheel, actuators, warnings)
 * is this rebuild's choice. Commands act at once, in the order the opmode calls them.
 */
public final class Mechanisms {
    /** The per-opmode settings (specs/mechanisms.md section 1). The opmode may change them at Start. */
    public static final class Settings {
        public boolean intakeAlwaysOn = false;
        public boolean flywheelStopsAfterSalvo = true;
        public boolean targetFollowsPose = false;
        public boolean fullActions = false;
        public boolean launchGridAllowed = false;
        public int jamRetries = 2;
        public boolean rampAtInit = true;
        public boolean flyPosAtInit = false;
        /** Reuse the pusher zero the previous opmode saved (the TeleOp after an auto). */
        public boolean continuePusherZero = false;
        /** The CS5 held count at the first loop. */
        public int startHeld = 0;

        /** The competition TeleOp: nothing is commanded at INIT (rule G403). The rest is switched on at Start. */
        public static Settings teleop(int startHeld) {
            Settings s = new Settings();
            s.rampAtInit = false;
            s.continuePusherZero = true;
            s.startHeld = startHeld;
            return s;
        }

        /** The autos. */
        public static Settings auto() {
            Settings s = new Settings();
            s.intakeAlwaysOn = true;
            s.flywheelStopsAfterSalvo = false;
            s.jamRetries = 1;
            s.flyPosAtInit = true;
            s.startHeld = MechParams.AUTO_START_HELD;
            return s;
        }
    }

    private enum Reject { IDLE, WAIT_UNLOCK, RUNNING }

    public final Settings settings;
    public final Intake intake;
    public final Flywheel flywheel;
    public final FlywheelPosition flyPos;
    public final Pusher pusher;
    public final PusherCheck check;
    public final Ramp ramp;
    public final SlotCounter slots;
    public final EntryCounter entry;

    private final HeldCount held = new HeldCount();
    private final SpeedSelector speed = new SpeedSelector();
    private final SpeedSelector.In speedIn = new SpeedSelector.In();
    private final LaunchSequencer launch = new LaunchSequencer();
    private final CheckScheduler sched = new CheckScheduler();
    private final FullAndLock fullLock = new FullAndLock();
    private final FullAndLock.In flIn = new FullAndLock.In();
    private final Warnings warnings = new Warnings();

    private Alliance alliance;
    private Reject reject = Reject.IDLE;
    private boolean rejectJustStarted;
    private boolean outtakeHeld, forceInOperator, forceInAuto, intakeRequested, spinRequested, aimLockWanted;
    private boolean calibrationSweep, stowPending, backup, stopped, first = true, wasFull, flyReported;
    private double speedTrim, backupDistance, flyStopAt = -1, countHoldoffUntil = -1, lastHealthLine = -1e9, slotsEmptySince = -1;
    private double manualSetpoint = Double.NaN;
    private Cell setCell, dashCell;
    private Vec dashSpot;
    private double dashDistance;
    private Intake.Mode lastIntakeMode = Intake.Mode.OFF;
    private Pose lastPose;

    public Mechanisms(HardwareMap hw, ChassisProfile profile, Alliance alliance, Settings settings) {
        this.settings = settings;
        this.alliance = alliance;
        intake = Intake.create(hw, profile);
        flywheel = Flywheel.create(hw, profile);
        flyPos = FlywheelPosition.create(hw);
        pusher = Pusher.create(hw, profile, settings.continuePusherZero);
        ramp = Ramp.create(hw, profile, settings.rampAtInit);
        slots = SlotCounter.create(hw);
        entry = EntryCounter.create(hw);
        check = (pusher != null && slots != null) ? new PusherCheck(pusher, slots) : null;
        if (pusher != null) pusher.setJamRetries(settings.jamRetries);
        if (settings.flyPosAtInit && flyPos != null) flyPos.commandPollen();
        if (entry != null) held.start(settings.startHeld);
        RunLog.event("SETUP mechanisms: " + found());
        if (slots != null) RunLog.event("COUNTER stored baselines: " + SlotCounter.baselinesText());
        if (entry != null) {
            RunLog.event("ENTRY COUNT starts at " + held.held());
            if (!entry.fastMode()) RunLog.event("CS5 ENTRY the sensor could not be put into its fast proximity mode: the count is useless");
        }
    }

    /** Which mechanisms the hub configuration contains, for INIT screens and the log. */
    public String found() {
        return (intake != null ? "intake " : "(no intake) ") + (flywheel != null ? "flywheel " : "(no flywheel) ")
                + (pusher != null ? "pusher " : "(no pusher) ") + (ramp != null ? "ramp " : "(no ramp) ")
                + (slots != null ? "CS1-4 " : "(no CS1-4) ") + (entry != null ? "CS5 " : "(no CS5) ") + (flyPos != null ? "fly_pos" : "(no fly_pos)");
    }

    public void setAlliance(Alliance a) { alliance = a; }

    /** Before the first update: the CS5 held count to start with (the TeleOp's INIT can discard a saved count). */
    public void setStartHeld(int n, String why) {
        if (entry == null) return;
        held.reset();
        held.start(n);
        RunLog.event("ENTRY COUNT starts at " + n + " (" + why + ")");
    }

    /** For INIT: reads the slot count and nothing else. Nothing moves. */
    public void initLoop() {
        if (slots == null) return;
        slots.setContext(false, false, false, false, false);
        slots.update(Clock.now());
    }

    // ================= Commands =================

    /** Queues a launch. Returns false when it is refused or one is already queued or running. */
    public boolean requestLaunch(String who) {
        double t = Clock.now();
        if (flywheel == null || pusher == null || pusher.faulted()) {
            String why = flywheel == null ? "no flywheel" : pusher == null ? "no pusher" : "the pusher is faulted";
            RunLog.event("LAUNCH refused (" + who + "): " + why);
            warnings.notice("Launch refused: " + why, t);
            return false;
        }
        if (launch.busy()) return false;
        if (check != null && check.running()) check.abort("a launch was requested", t);
        if (reject != Reject.IDLE) {
            // The operator's way out of a jam in that direction.
            if (reject == Reject.RUNNING) pusher.cancelMove();
            reject = Reject.IDLE;
            RunLog.event("REJECT cancelled by a launch");
        }
        launch.queue(t);
        RunLog.eventf("LAUNCH queued (%s): setpoint %.0f, flywheel %.0f, held %s", who, speed.setpoint, flywheel.filtered(), heldText());
        return true;
    }

    /** A reject: a pusher reverse together with an intake outtake. Cancels a queued launch or a running salvo. */
    public void requestReject(String who) {
        double t = Clock.now();
        if (pusher == null || reject != Reject.IDLE) return;
        RunLog.event("COMMAND reject (" + who + ")");
        if (check != null && check.running()) check.abort("a reject was requested", t);
        boolean wasBusy = launch.busy(), wasSalvo = launch.salvo();
        if (wasBusy) {
            launch.cancel(t);
            RunLog.event(wasSalvo ? "SALVO cancelled by a reject" : "LAUNCH dropped: cancelled by a reject");
        }
        if (fullLock.unlockForReject()) {
            // Unlock first; the reject starts by itself when the unlock finishes.
            pusher.unlock();
            reject = Reject.WAIT_UNLOCK;
            return;
        }
        if (wasSalvo) {
            pusher.cancelMove();
            if (entry != null) held.salvoIncomplete();
        }
        startReject();
    }

    private void startReject() {
        pusher.reject();
        reject = Reject.RUNNING;
        rejectJustStarted = true;
        RunLog.eventf("REJECT started at %.0f deg", pusher.angle());
    }

    /** The operator's left trigger: intake OUT while held. */
    public void setOuttake(boolean heldDown) {
        if (heldDown != outtakeHeld) RunLog.event("COMMAND outtake " + (heldDown ? "on" : "off"));
        outtakeHeld = heldDown;
    }

    /** The operator's right trigger: unlock a locked pusher and keep the intake IN through a FULL sequence. */
    public void setOperatorForceIn(boolean heldDown) {
        if (heldDown != forceInOperator) RunLog.event("COMMAND force intake in " + (heldDown ? "on" : "off"));
        forceInOperator = heldDown;
    }

    /** An auto forces the intake in whatever any count says; a reject or outtake still wins. */
    public void setAutoForceIn(boolean on) {
        if (on != forceInAuto) RunLog.event("COMMAND auto force intake in " + (on ? "on" : "off"));
        forceInAuto = on;
    }

    /** The intake request of the non-always-on mode. */
    public void setIntakeRequested(boolean on) { intakeRequested = on; }

    /** Keep the flywheel at the setpoint without a launch (the TeleOp and the autos: runs from Start). */
    public void setSpin(boolean on) { spinRequested = on; }

    public void setSpeedTrim(double ticks) { speedTrim = ticks; }
    public double speedTrim() { return speedTrim; }

    /** The cell an opmode aims at when the target does not follow the pose. */
    public void setTargetCell(Cell cell) { setCell = cell; }

    /** While a Hive Dash runs: its cell overrides the nearest-cell choice and the speed is worked out for the launch spot. Null to clear. */
    public void setDash(Cell cell, Vec spot, double distance) {
        dashCell = cell;
        dashSpot = spot;
        dashDistance = distance;
    }

    /** A target lock or a Hive Dash is active (the second reason to lock; switched off by a master switch). */
    public void setAimLockWanted(boolean on) { aimLockWanted = on; }

    /** One way until the opmode ends: the launch distance becomes a fixed value and the target stops following the pose. */
    public void enterBackup(double fixedDistance) {
        if (!backup) RunLog.eventf("BACKUP MODE mechanisms: fixed launch distance %.0f in", fixedDistance);
        backup = true;
        backupDistance = fixedDistance;
    }

    /** For tuner and study opmodes: a fixed flywheel setpoint in place of the launch map. NaN to go back. */
    public void setManualSetpoint(double ticks) { manualSetpoint = ticks; }

    public void clearPusherStall() {
        if (pusher != null) pusher.clearStall();
    }

    public void deployRamp() {
        if (ramp == null) return;
        RunLog.event("COMMAND ramp deploy");
        stowPending = false;
        ramp.deploy();
    }

    /** At once, with no check. The TeleOp and the autos use this. */
    public void stowRampNow() {
        if (ramp == null) return;
        RunLog.event("COMMAND ramp stow now");
        stowPending = false;
        ramp.stow();
    }

    /** Carried out when the robot's origin is clear of every flower, or at once if the pose is not trusted. */
    public void stowRampChecked() {
        if (ramp == null) return;
        RunLog.event("COMMAND ramp stow (checked)");
        stowPending = true;
    }

    /** The TeleOp's first servo commands, at Start. */
    public void commandFlyPosPollen() {
        if (flyPos != null) flyPos.commandPollen();
    }

    /** Autos: move the arm to the lock angle while the flywheel spins up, so the preload salvo is shorter. Independent of the lock switches. */
    public void prePositionPreload() {
        if (pusher != null && pusher.lock()) RunLog.event("PUSHER preload pre-position: to the lock angle");
    }

    /** The operator's calibration sweep of the pusher; again to stop it. */
    public void toggleCalibrationSweep() {
        double t = Clock.now();
        if (check == null) return;
        if (calibrationSweep && check.running()) {
            check.abort("stopped by the operator", t);
            calibrationSweep = false;
            return;
        }
        String why = !slots.valid() ? "the counter is not reading" : slots.count() > 0 ? "the track is not empty"
                : busy() ? "the robot is busy" : (intakeRequested || (ramp != null && ramp.deployed())) ? "the intake is requested or the ramp is deployed"
                : !pusher.stopped() ? "the pusher is not stopped" : null;
        if (why != null) {
            warnings.notice("Calibration sweep refused: " + why, t);
            return;
        }
        RunLog.event("COMMAND pusher calibration sweep");
        sched.clear();
        calibrationSweep = true;
        check.start(true, t);
    }

    /** For the end of an opmode: drops anything queued or running and switches the intake, flywheel and pusher off at once. It replaces the last update. */
    public void stopAll() {
        stopped = true;
        launch.cancel(Clock.now());
        reject = Reject.IDLE;
        if (intake != null) intake.stopNow();
        if (flywheel != null) flywheel.stopNow();
        if (pusher != null) pusher.stopNow();
        RunLog.burst(false);
        RunLog.event("COMMAND stop all mechanisms");
    }

    // ================= Update =================

    private boolean busy() {
        return launch.busy() || reject != Reject.IDLE || outtakeHeld;
    }

    /**
     * Once per loop, after the opmode's commands.
     *
     * @param pose        the pose read this loop, or null when there is none
     * @param poseTrusted false in backup mode
     */
    public void update(Pose pose, boolean poseTrusted) {
        if (stopped) return;
        double t = Clock.now();
        lastPose = pose;
        LoopProfiler.begin("mechanisms");
        if (first) {
            first = false;
            // A safety net: if nothing has commanded the flywheel position yet, do it now.
            if (flyPos != null && !flyPos.commanded()) flyPos.commandPollen();
            // Once after the first update, if the pusher's zero has not been verified since it was assumed.
            if (check != null && !pusher.zeroVerified()) sched.want(t);
            if (entry != null) RunLog.event("CS5 ENTRY COUNTER started" + (entry.fastMode() ? "" : " (NOT in fast mode)"));
        }

        // ---- 1. Counts ----
        LoopProfiler.begin("mech.counter");
        if (slots != null) {
            double rel = pusher == null ? 0 : pusher.relAngle();
            boolean armNear = pusher != null && rel >= -MechParams.ARM_NEAR_CS4_BEHIND_DEG && rel <= MechParams.ARM_NEAR_CS4_AHEAD_DEG;
            boolean checking = check != null && check.running(), sweeping = check != null && check.sweeping();
            slots.setContext(armNear, checking, sweeping || t < countHoldoffUntil, fullLock.countFrozen(), sweeping);
            slots.update(t);
        }
        if (entry != null) {
            for (EntryRule.Entry e : entry.collect()) {
                boolean counted = held.onEntry(e.t);
                RunLog.eventf("CS5 ENTRY height %d, proximity %d, %.3f s ago, %s, held %d", e.height, e.proximity, t - e.t,
                        counted ? "counted" : "not counted (the intake was running OUT)", held.held());
            }
            String note = entry.takeNote();
            if (note != null) RunLog.event("CS5 ENTRY note: " + note);
            String stats = entry.stats(t);
            if (stats != null) RunLog.event("CS5 ENTRY thread: " + stats);
            if (MechParams.RESET_WHEN_SLOTS_EMPTY_S > 0 && slots != null && slots.valid() && slots.count() == 0) {
                if (slotsEmptySince < 0) slotsEmptySince = t;
                if (held.held() > 0 && t - slotsEmptySince >= MechParams.RESET_WHEN_SLOTS_EMPTY_S) {
                    held.reset();
                    RunLog.event("ENTRY COUNT reset: the slots have read empty");
                }
            } else {
                slotsEmptySince = -1;
            }
        }
        LoopProfiler.end("mech.counter");
        boolean full = full();
        if (full != wasFull) RunLog.event(full ? "ENTRY FULL: " + heldText() + " held" : "ENTRY no longer full");
        wasFull = full;

        // ---- 2. Launch speed ----
        speedIn.pose = pose;
        speedIn.alliance = alliance;
        speedIn.followsPose = settings.targetFollowsPose && poseTrusted;
        speedIn.setCell = setCell;
        speedIn.backup = backup;
        speedIn.backupDistance = backupDistance;
        speedIn.dashCell = backup ? null : dashCell;
        speedIn.dashSpot = dashSpot;
        speedIn.dashDistance = dashDistance;
        speedIn.speedTrim = speedTrim;
        speedIn.gridAllowed = settings.launchGridAllowed;
        speedIn.flywheelLost = flywheel != null && flywheel.lost();
        speed.update(speedIn);
        double setpoint = Double.isNaN(manualSetpoint) ? speed.setpoint : manualSetpoint + speedTrim;

        // ---- 3. Launch ----
        if (launch.busy() && flywheel != null && pusher != null) {
            boolean failed = launch.salvo() && (pusher.gaveUp() || pusher.faulted());
            LaunchSequencer.Event ev = launch.step(t, flywheel.inBand(), pusher.stopped(), flyPos == null || flyPos.inPlace(), failed,
                    flywheel.filtered(), setpoint);
            switch (ev) {
                case START: {
                    fullLock.onSalvoStart();        // the salvo sweeps from the lock position
                    pusher.salvo();
                    int taken = held.salvoStarted(slots != null && slots.valid() ? slots.count() : 0);
                    RunLog.eventf("SALVO started: flywheel %.0f for %.0f, arm at %.0f deg, %d taken to be aboard, tally %d",
                            flywheel.filtered(), setpoint, pusher.angle(), taken, held.tally());
                    break;
                }
                case DROPPED:
                    RunLog.event("LAUNCH dropped: " + launch.detail);
                    warnings.notice("Launch dropped: " + launch.detail, t);
                    break;
                case DONE:
                    RunLog.eventf("SALVO done #%d in %.2f s: lowest flywheel speed %.0f, arm settled at %.0f deg",
                            launch.salvoCount(), launch.salvoSeconds(t), launch.minSpeed(), pusher.angle());
                    sched.want(t);
                    if (settings.flywheelStopsAfterSalvo) flyStopAt = t + MechParams.FLYWHEEL_STOP_DELAY_S;
                    break;
                case INCOMPLETE:
                    // A robot that shows too many is launched or rejected again; safer than one that shows 0 while jammed.
                    held.salvoIncomplete();
                    RunLog.eventf("SALVO did not complete (the pusher %s): count put back to %d, tally %d",
                            pusher.faulted() ? "faulted" : "gave up", held.held(), held.tally());
                    break;
                default:
                    if (launch.detail != null) RunLog.event("LAUNCH " + launch.detail);
            }
        }

        // ---- 4. Reject ----
        if (pusher != null) {
            if (reject == Reject.WAIT_UNLOCK && fullLock.lock() == FullAndLock.Lock.NONE && pusher.stopped()) {
                startReject();
            } else if (reject == Reject.RUNNING && !rejectJustStarted && pusher.idle() && (pusher.stopped() || pusher.gaveUp())) {
                reject = Reject.IDLE;
                held.reset();
                fullLock.onRejectDone();
                RunLog.eventf("REJECT done at %.0f deg%s; the entry count is reset", pusher.angle(), pusher.gaveUp() ? " (the pusher gave up)" : "");
            }
            rejectJustStarted = false;
        }

        // ---- 5. Pusher check ----
        if (check != null) {
            boolean slotsOk = slots.valid();
            CheckScheduler.Decision d = sched.step(t, check.running(), busy(), slotsOk, slots.count(), pusher.stopped(), pusher.gaveUp(),
                    fullLock.lock() != FullAndLock.Lock.NONE);
            if (d == CheckScheduler.Decision.START) check.start(false, t);
            else if (d == CheckScheduler.Decision.SKIP) RunLog.event("PUSHER CHECK skipped: " + sched.why);
            else if (d == CheckScheduler.Decision.ABORT) check.abort(sched.why, t);
            boolean was = check.running();
            check.update(t);
            if (was && !check.running()) {
                countHoldoffUntil = t + MechParams.CHECK_COUNT_HOLDOFF_S;
                calibrationSweep = false;
            }
        }

        // ---- 6. FULL and the pusher lock ----
        boolean checkBusy = check != null && (check.running() || sched.wanted());
        flIn.t = t;
        flIn.full = entry != null && held.full() && (!MechParams.FULL_NEEDS_SLOT_AGREEMENT || (slots != null && slots.valid() && slots.full()));
        flIn.fullActions = settings.fullActions;
        flIn.forceIn = forceInOperator;
        flIn.aimWants = MechParams.LOCK_FOR_AIM && aimLockWanted && anyHeld();
        flIn.canLock = pusher != null && slots != null && !busy() && !pusher.faulted() && !pusher.gaveUp() && !checkBusy && pusher.canLockFrom();
        flIn.pusherStopped = pusher != null && pusher.stopped();
        flIn.pusherLocked = pusher != null && pusher.locked();
        flIn.launchPending = launch.queued();
        flIn.proximity = entry == null ? -1 : entry.latest();
        flIn.emptyLevel = entry == null ? Double.NaN : entry.level();
        FullAndLock.Lock lockBefore = fullLock.lock();
        fullLock.update(flIn);
        if (pusher != null) {
            if (fullLock.cmdLock) fullLock.lockResult(pusher.lock());
            if (fullLock.cmdUnlock) pusher.unlock();
        }
        if (fullLock.lock() != lockBefore) RunLog.eventf("PUSHER lock %s -> %s at %.0f deg", lockBefore, fullLock.lock(), pusher == null ? 0 : pusher.angle());

        // ---- 7. Intake: decided fresh each update; later rules override earlier ones ----
        Intake.Mode mode;
        boolean outNow = reject == Reject.RUNNING || outtakeHeld;
        if (settings.intakeAlwaysOn) {
            mode = Intake.Mode.IN;
            if (fullLock.intake != null) mode = fullLock.intake;
            if (outNow) mode = Intake.Mode.OUT;
            else if (forceInAuto) mode = Intake.Mode.IN;
            if (calibrationSweep) mode = Intake.Mode.OFF;
        } else {
            mode = (intakeRequested || (ramp != null && ramp.deployed())) ? Intake.Mode.IN : Intake.Mode.OFF;
            if (mode == Intake.Mode.IN && full) {
                mode = Intake.Mode.OFF;
                if (intakeRequested) warnings.notice("Full: the intake is stopped", t);
                intakeRequested = false;
            }
            if (outNow) mode = Intake.Mode.OUT;
            if (fullLock.lock() == FullAndLock.Lock.LOCKED || fullLock.lock() == FullAndLock.Lock.LOCKING) mode = Intake.Mode.OFF;
            if (calibrationSweep) mode = Intake.Mode.OFF;
        }
        if (mode != lastIntakeMode) RunLog.event("INTAKE " + lastIntakeMode + " -> " + mode);
        lastIntakeMode = mode;
        held.intakeOut(mode == Intake.Mode.OUT, t);

        // ---- 8. Ramp ----
        if (ramp != null && stowPending && (!poseTrusted || pose == null || Ramp.clearOfFlowers(pose.x, pose.y))) {
            stowPending = false;
            ramp.stow();
            RunLog.event("RAMP checked stow carried out");
        }

        // ---- 9. Flywheel target: runs while spin is requested, a launch is queued, a salvo runs, or a stop is pending ----
        boolean runs = spinRequested || launch.busy() || (flyStopAt >= 0 && t < flyStopAt);

        // ---- 10. The actuators: each reads once and writes when needed ----
        if (intake != null) {
            LoopProfiler.begin("mech.intake");
            intake.set(mode);
            intake.update(t);
            LoopProfiler.end("mech.intake");
        }
        if (flywheel != null) {
            LoopProfiler.begin("mech.flywheel");
            flywheel.setTarget(runs ? setpoint : 0);
            flywheel.update(t);
            LoopProfiler.end("mech.flywheel");
            String report = flywheel.takeReport();
            if (report != null) {
                RunLog.event(report);
                warnings.notice(flywheel.lost() ? "Flywheel encoder lost: open loop" : "Flywheel encoder back", t);
            }
        }
        if (pusher != null) {
            LoopProfiler.begin("mech.pusher");
            pusher.update();
            LoopProfiler.end("mech.pusher");
            if (pusher.takeJamCleared()) warnings.notice("Pusher jam cleared", t);
        }

        // ---- 11. Warnings, burst logging, the empty-track health line ----
        raiseWarnings();
        warnings.commit();
        RunLog.burst(launch.burst(t));
        if (slots != null && slots.valid() && slots.count() == 0 && t - lastHealthLine >= MechParams.HEALTH_LINE_S) {
            lastHealthLine = t;
            RunLog.event("COUNTER empty track, color shifts " + slots.shiftsText());
        }
        LoopProfiler.end("mechanisms");
    }

    private void raiseWarnings() {
        if (slots != null && !slots.valid()) warnings.raise(Warnings.Code.COUNTER_NOT_READING, "CS" + slots.staleSensor() + " has stopped answering");
        if (entry != null && entry.started() && !entry.healthy()) warnings.raise(Warnings.Code.ENTRY_NOT_ANSWERING, "CS5 is not answering");
        if (full()) warnings.raise(Warnings.Code.FULL, "4 held");
        if (slots != null && slots.valid()) {
            int at = slots.opponentNectarAt(alliance);
            if (at > 0) warnings.raise(Warnings.Code.OPPONENT_NECTAR, "the other alliance's nectar is at CS" + at + "; reject is the operator's decision");
        }
        if (speed.mapEmpty) warnings.raise(Warnings.Code.LAUNCH_MAP_EMPTY, "no speed source; the manual setpoint is in use");
        if (flywheel != null && flywheel.lost()) warnings.raise(Warnings.Code.FLYWHEEL_LOST, "flywheel encoder lost: open loop at a fixed power");
        if (ramp != null && !ramp.configured()) warnings.raise(Warnings.Code.RAMP_NOT_CONFIGURED, "ramp servo positions missing from the chassis profile");
        if (backup) warnings.raise(Warnings.Code.BACKUP_MODE, "no pose: fixed launch distance");
        if (stowPending) warnings.raise(Warnings.Code.STOW_WAITING, "a checked stow is waiting for the robot to clear a flower");
        if (pusher != null) {
            if (!pusher.calibrated()) warnings.raise(Warnings.Code.PUSHER_NOT_CALIBRATED, "pusher not calibrated");
            if (pusher.faulted()) warnings.raise(Warnings.Code.PUSHER_STALLED, "pusher stalled");
            if (pusher.jamUnresolved()) warnings.raise(Warnings.Code.PUSHER_JAMMED, "pusher jam not resolved: launch or reject again");
            if (check != null && check.result() == PusherCheck.Result.LOST) warnings.raise(Warnings.Code.ZERO_UNCERTAIN, "pusher zero uncertain: the last check ended LOST");
            else if (!pusher.zeroVerified()) warnings.raise(Warnings.Code.ZERO_NOT_VERIFIED, "pusher zero not verified yet");
        }
    }

    // ================= Queries =================

    public boolean launchQueued() { return launch.queued(); }
    public boolean salvoRunning() { return launch.salvo(); }
    public boolean launchBusy() { return launch.busy(); }
    public int salvoCount() { return launch.salvoCount(); }
    public boolean rejecting() { return reject != Reject.IDLE; }
    public boolean inLaunchSweep() { return pusher != null && pusher.inLaunchSweep(); }
    public Cell targetCell() { return speed.cell; }
    public double distance() { return speed.distance; }
    public double setpoint() { return speed.setpoint; }
    public double mapSpeed() { return speed.mapSpeed; }
    public LaunchGrid.Result grid() { return speed.grid; }
    public boolean gridUsed() { return speed.gridUsed; }
    public boolean backup() { return backup; }
    public Warnings warnings() { return warnings; }
    public String notice() { return warnings.notice(Clock.now()); }
    public FullAndLock.Lock lockState() { return fullLock.lock(); }
    /** The FULL sequence step for the screen, or null. */
    public String fullStepText() { return fullLock.text; }
    public int launchedTally() { return held.tally(); }
    public boolean entryAnswering() { return entry != null && (!entry.started() || entry.healthy()); }

    /** The CS5 held count, or -1 when CS5 is missing or not answering. */
    public int heldCount() {
        return entry != null && entryAnswering() ? held.held() : -1;
    }

    /** The held count whether or not CS5 answers now, for saving between opmodes; -1 with no CS5. */
    public int heldCountForSave() { return entry == null ? -1 : held.held(); }

    public String heldText() {
        int h = heldCount();
        return h < 0 ? "?" : String.valueOf(h);
    }

    /** Full, for every consumer: the entry count reaching 4 (the slot count with the "count by entries" switch off). */
    public boolean full() {
        if (MechParams.COUNT_BY_ENTRIES) return entry != null && held.full();
        return slots != null && slots.valid() && slots.full();
    }

    private boolean anyHeld() {
        return (entry != null && held.held() > 0) || (slots != null && slots.valid() && slots.count() > 0);
    }

    /** For a dash: a count is being read and neither count holds anything. If no count is reading, the robot is taken to hold something. */
    public boolean readsEmpty() {
        boolean slotsRead = slots != null && slots.valid(), entryRead = entry != null && entry.healthy();
        if (!slotsRead && !entryRead) return false;
        return (!slotsRead || slots.count() == 0) && (!entryRead || held.held() == 0);
    }

    /** For auto-launch: both counts read and both hold nothing. */
    public boolean bothReadEmpty() {
        return slots != null && slots.valid() && slots.count() == 0 && entry != null && entry.healthy() && held.held() == 0;
    }

    public String flywheelText() {
        if (flywheel == null || flywheel.target() <= 0) return "NO FLY";
        if (flywheel.inBand()) return "READY";
        return flywheel.filtered() > flywheel.target() ? "SLOWING" : "SPIN UP";
    }

    // ================= Actions for autos (specs/mechanisms.md section 9) =================

    /** Finishes when the queued launch has completed or been dropped. */
    public Action launchAction(final String who) {
        return new Action() {
            boolean asked;

            @Override public boolean run(TelemetryPacket p) {
                if (!asked) {
                    asked = true;
                    if (!requestLaunch(who)) return launch.busy();
                    return true;
                }
                return launch.busy();
            }
        };
    }

    /** Finishes when the reject has completed. */
    public Action rejectAction() {
        return new Action() {
            boolean asked;

            @Override public boolean run(TelemetryPacket p) {
                if (!asked) {
                    asked = true;
                    requestReject("auto");
                    return pusher != null;
                }
                return reject != Reject.IDLE;
            }
        };
    }

    /** Finishes when the ramp has settled. */
    public Action deployAction() {
        return new Action() {
            boolean asked;

            @Override public boolean run(TelemetryPacket p) {
                if (!asked) {
                    asked = true;
                    deployRamp();
                }
                return ramp != null && !ramp.settled();
            }
        };
    }

    /** Finishes when the ramp has settled. */
    public Action stowNowAction() {
        return new Action() {
            boolean asked;

            @Override public boolean run(TelemetryPacket p) {
                if (!asked) {
                    asked = true;
                    stowRampNow();
                }
                return ramp != null && !ramp.settled();
            }
        };
    }

    /** Finishes once the ramp has settled wherever it was last commanded. */
    public Action waitRampSettled() {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                return ramp != null && !ramp.settled();
            }
        };
    }

    // ================= Log columns (specs/mechanisms.md section 14) =================

    public static final String LOG_COLUMNS = "setpoint,fly_target,fly_vel,fly_ready,pusher_state,pusher_deg,pusher_vel,pusher_amps,"
            + "intake_mode,ramp,count,slots,launch,warnings,overrides,check,fly_power,fly_amps,pusher_lock,pusher_hold,intake_amps,"
            + "cs5_entries,cs5_held,cs5_full,cs5_prox,cs5_raw,tally";

    public String logRow() {
        String overrides = (backup ? "B" : "") + (forceInOperator ? "F" : "") + (forceInAuto ? "A" : "") + (outtakeHeld ? "O" : "");
        return String.format(Locale.US, "%.0f,%.0f,%.0f,%d,%s,%.1f,%.0f,%.2f,%s,%s,%s,%s,%s,%d,%s,%s,%.2f,%.1f,%s,%.2f,%.1f,%d,%s,%d,%d,%s,%d",
                speed.setpoint,
                flywheel == null ? 0 : flywheel.target(), flywheel == null ? 0 : flywheel.speed(), flywheel != null && flywheel.inBand() ? 1 : 0,
                pusher == null ? "-" : pusher.state(), pusher == null ? 0 : pusher.angle(), pusher == null ? 0 : pusher.speedTicks(), pusher == null ? 0 : pusher.amps(),
                lastIntakeMode, ramp == null ? "-" : ramp.target(),
                slots == null ? "-" : (slots.valid() ? String.valueOf(slots.count()) : "?"), slots == null ? "-" : slots.slots(),
                reject != Reject.IDLE ? "REJECT" : launch.state(), warnings.count(), overrides.isEmpty() ? "-" : overrides,
                check == null ? "-" : check.result(),
                flywheel == null ? 0 : flywheel.command(), flywheel == null ? 0 : flywheel.amps(),
                fullLock.lock(), pusher == null ? 0 : pusher.holdPower(), intake == null ? 0 : intake.amps(),
                held.totalEntries(), heldText(), full() ? 1 : 0, entry == null ? -1 : entry.latest(), entry == null ? "" : entry.takeRaw(), held.tally());
    }
}
