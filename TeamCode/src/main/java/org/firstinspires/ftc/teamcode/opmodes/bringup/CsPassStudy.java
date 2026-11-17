package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.EntryRule;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPosition;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.SlotCounter;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * CS Pass Study (specs/support-opmodes.md sections 3 and 3.2): can CS5 count entering elements without slowing the
 * loop? The real mechanisms with the intake in; CS5 read on its own thread; the operator presses a button per element
 * fed as the ground truth.
 *
 * Intake IN throughout, flywheel at 500 ticks/s, loop padded to 28 ms. CS5 on its own thread every 20 ms, one 15-byte
 * burst read per cycle (status, proximity and the four light channels); a CS5 THREAD line every 5 s.
 *
 * Controls (gamepad 1): sticks drive robot-oriented, right bumper precision. B = one element fed (a "FED n" line).
 * Y salvo; X automatic clear on / off; A mark; D-pad up / down setpoint by 20; left trigger intake OUT; Back clears a
 * pusher fault (one of the six support opmodes that use Back).
 *
 * Automatic clear: while it is on, a salvo fires by itself when the slot count reads 4 or more, and not within 2 s
 * of the last salvo.
 */
@Config
@TeleOp(name = "CS Pass Study", group = "2 Bring-up")
public final class CsPassStudy extends TeamOpMode {
    public static double LOOP_MS = 28.0, THREAD_PERIOD_MS = 20.0, FLYWHEEL_SPEED = 500.0, SPEED_STEP = 20.0, THREAD_LINE_S = 5.0;
    public static int BURST_BYTES = 15, CLEAR_AT = 4;
    public static double CLEAR_PAUSE_S = 2.0;
    private static final double PRECISION = 0.30, TRIGGER = 0.3;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "CS Pass Study"; }

    // The thread's side. It never writes the log; the loop collects from it.
    private final ConcurrentLinkedQueue<Integer> raw = new ConcurrentLinkedQueue<>();
    private final EntryRule rule = new EntryRule();
    private volatile boolean threadRun;
    private volatile int entries, latest = -1;
    private volatile long samples, errors;
    private volatile double maxGapS, readMsSum, level = Double.NaN;

    private void sampleLoop(RevColorSensorV3 sensor) {
        double last = Clock.now();
        try {
            while (threadRun) {
                double t = Clock.now();
                try {
                    // One burst from the status register: status, proximity (2 bytes), then infrared, green, blue and red (3 bytes each).
                    byte[] b = sensor.read(BroadcomColorSensor.Register.MAIN_STATUS, BURST_BYTES);
                    int prox = (b[1] & 0xFF) | ((b[2] & 0x07) << 8);
                    readMsSum += (Clock.now() - t) * 1000;
                    samples++;
                    latest = prox;
                    maxGapS = Math.max(maxGapS, t - last);
                    last = t;
                    raw.add(prox);
                    if (rule.sample(t, prox) != null) entries++;
                    level = rule.level();
                } catch (Throwable e) {
                    errors++;
                }
                long sleep = (long) THREAD_PERIOD_MS - (long) ((Clock.now() - t) * 1000);
                if (sleep > 0) Thread.sleep(sleep);
            }
        } catch (InterruptedException ignored) { }
    }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
        final Intake intake = Intake.create(hardwareMap, profile);
        final Flywheel fly = Flywheel.create(hardwareMap, profile);
        FlywheelPosition flyPos = FlywheelPosition.create(hardwareMap);
        final Pusher pusher = Pusher.create(hardwareMap, profile, false);
        final SlotCounter slots = SlotCounter.create(hardwareMap);
        final RevColorSensorV3 cs5 = hardwareMap.tryGet(RevColorSensorV3.class, ChassisProfile.ENTRY_SENSOR);
        boolean fast = false;
        if (cs5 != null) {
            try {
                // Fast proximity mode: 8-bit, a new value every 6.25 ms.
                cs5.write8(BroadcomColorSensor.Register.PS_MEAS_RATE,
                        BroadcomColorSensor.PSResolution.R8BIT.bitOr(BroadcomColorSensor.PSMeasurementRate.R6_25ms.bVal));
                fast = true;
            } catch (Throwable ignored) { }
        }
        telemetry.addLine("CS Pass Study: intake IN from Start, flywheel at " + (int) FLYWHEEL_SPEED + ". The pusher must be at HOLD.");
        telemetry.addLine((cs5 == null ? "NO CS5 in the configuration" : fast ? "CS5 in fast mode" : "CS5 NOT in fast mode") + ". B = one element fed.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        markStarted();
        RunLog.eventf("SETUP CS Pass Study: CS5 %s, thread period %.0f ms, %d-byte burst, loop padded to %.0f ms, flywheel %.0f ticks/s",
                cs5 == null ? "missing" : fast ? "fast mode" : "NOT in fast mode", THREAD_PERIOD_MS, BURST_BYTES, LOOP_MS, FLYWHEEL_SPEED);
        if (flyPos != null) flyPos.commandPollen();
        final int[] counts = new int[2];       // fed, marks
        RunLog.sampler("cmd_fwd,cmd_left,cmd_turn,fly_target,fly_vel,pusher_state,pusher_deg,intake_mode,slots,cs5_entries,cs5_prox,cs5f,fed,marks", 0, new RunLog.Sampler() {
            @Override public String row() {
                StringBuilder f = new StringBuilder();
                Integer v;
                while ((v = raw.poll()) != null) {
                    if (f.length() > 0) f.append(' ');
                    f.append(v);
                }
                return String.format(Locale.US, "%.2f,%.2f,%.2f,%.0f,%.0f,%s,%.1f,%s,%s,%d,%d,%s,%d,%d", drive.cmdForward(), drive.cmdLeft(), drive.cmdTurn(),
                        fly == null ? 0 : fly.target(), fly == null ? 0 : fly.speed(), pusher == null ? "-" : pusher.state(), pusher == null ? 0 : pusher.angle(),
                        intake == null ? "-" : intake.mode(), slots == null ? "-" : slots.slots(), entries, latest, f, counts[0], counts[1]);
            }
        });
        RunLog.burst(true);
        Thread thread = null;
        if (cs5 != null) {
            threadRun = true;
            thread = new Thread(new Runnable() {
                @Override public void run() { sampleLoop(cs5); }
            }, "CS Pass Study CS5");
            thread.setDaemon(true);
            thread.start();
        }
        Edge fed = new Edge(), salvo = new Edge(), auto = new Edge(), mark = new Edge(), up = new Edge(), down = new Edge(), back = new Edge();
        LoopPad pad = new LoopPad();
        double setpoint = FLYWHEEL_SPEED, lastThreadLine = Clock.now();
        boolean autoClear = false, wasSalvo = false;
        double lastSalvoAt = -1e9;
        int entriesAtSalvo = 0, lastEntries = 0, salvos = 0;
        long lastSamples = 0;
        try {
            while (opModeIsActive()) {
                double t = Clock.now();
                drive.updatePose();
                driveRobotOriented(gamepad1.right_bumper ? PRECISION : 1.0);
                if (fed.pressed(gamepad1.b)) {
                    counts[0]++;
                    RunLog.eventf("FED %d (the thread has counted %d)", counts[0], entries);
                }
                if (mark.pressed(gamepad1.a)) {
                    counts[1]++;
                    RunLog.eventf("STUDY MARK %d: fed %d, counted %d", counts[1], counts[0], entries);
                }
                if (up.pressed(gamepad1.dpad_up)) setpoint += SPEED_STEP;
                if (down.pressed(gamepad1.dpad_down)) setpoint = Math.max(0, setpoint - SPEED_STEP);
                if (auto.pressed(gamepad1.x)) {
                    autoClear = !autoClear;
                    RunLog.event("STUDY automatic clear " + (autoClear ? "on" : "off"));
                }
                if (back.pressed(gamepad1.back) && pusher != null) {
                    pusher.clearStall();
                    RunLog.event("COMMAND clear pusher fault");
                }
                int n = entries;
                if (n != lastEntries) {
                    RunLog.eventf("CS5 ENTRY counted %d (fed %d), proximity %d", n, counts[0], latest);
                    lastEntries = n;
                }
                boolean want = salvo.pressed(gamepad1.y)
                        || (autoClear && slots != null && slots.valid() && slots.count() >= CLEAR_AT && t - lastSalvoAt >= CLEAR_PAUSE_S);
                if (want && pusher != null && pusher.stopped()) {
                    pusher.salvo();
                    entriesAtSalvo = n;
                    lastSalvoAt = t;
                    salvos++;
                    RunLog.eventf("SALVO started (study) %d: fed %d, counted %d, flywheel %.0f", salvos, counts[0], n, fly == null ? 0 : fly.filtered());
                }
                boolean salvoNow = pusher != null && pusher.state() == Pusher.State.SALVO;
                if (wasSalvo && !salvoNow) RunLog.eventf("SALVO done (study) %d%s", salvos, pusher.gaveUp() ? ": the pusher gave up" : "");
                wasSalvo = salvoNow;
                if (slots != null) {
                    slots.setContext(false, false, false, false, false);
                    slots.update(t);
                }
                if (intake != null) {
                    intake.set(gamepad1.left_trigger > TRIGGER ? Intake.Mode.OUT : Intake.Mode.IN);     // IN throughout
                    intake.update(t);
                }
                if (fly != null) {
                    fly.setTarget(setpoint);
                    fly.update(t);
                }
                if (pusher != null) pusher.update();
                if (t - lastThreadLine >= THREAD_LINE_S) {
                    long s = samples;
                    RunLog.eventf("CS5 THREAD %d samples in %.1f s, %d read errors in all, mean read %.1f ms, longest gap %.0f ms, empty level %.0f, latest %d, counted %d",
                            s - lastSamples, t - lastThreadLine, errors, s == 0 ? 0 : readMsSum / s, maxGapS * 1000, level, latest, n);
                    lastSamples = s;
                    lastThreadLine = t;
                    maxGapS = 0;
                }
                RunLog.tick();
                telemetry.addLine(String.format(Locale.US, "fed %d, counted %d, difference %+d", counts[0], n, n - counts[0]));
                telemetry.addLine(String.format(Locale.US, "CS5 %d (empty %.0f), flywheel %.0f for %.0f", latest, level, fly == null ? 0 : fly.filtered(), setpoint));
                telemetry.addLine("automatic clear " + (autoClear ? "ON" : "off") + ", salvos " + salvos + ", marks " + counts[1]);
                telemetry.addLine("B fed, Y salvo, X auto clear, A mark, D-pad up/down speed, LT out, Back clears a fault");
                telemetry.update();
                pad.pad(LOOP_MS);
            }
        } finally {
            threadRun = false;
            if (thread != null) thread.interrupt();
            if (intake != null) intake.stopNow();
            if (fly != null) fly.stopNow();
            if (pusher != null) pusher.stopNow();
            RunLog.eventf("STUDY RESULT fed %d, counted %d, %d samples, %d read errors", counts[0], entries, samples, errors);
        }
    }
}
