package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.SlotCounter;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Color Sensor Study (specs/support-opmodes.md sections 3 and 3.2): what do CS1 to CS4 read after the track or the
 * lighting changes? Four guided steps: the empty baseline, an empty pusher sweep, a full baseline with 4 pollen, and a
 * slow reject. It prints the new empty readings to paste into the element counter.
 *
 * Controls (gamepad 1): A starts the step shown, and stops a running one. D-pad left / right goes to the previous /
 * next step, between steps only. Right bumper logs a note. Holding X runs the intake IN to load pollen, between steps
 * only. The baselines hold still about 8 s; the sweep and the slow reject turn the pusher backward one full turn at
 * 0.15 power, open loop, and are given up after 25 s, or after 2 s if the arm has not moved 5 deg. One sensor is read
 * every 25 ms. The pusher must be at HOLD at INIT. The intake runs OUT during the slow reject, so the elements can leave.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Config
@Disabled
@TeleOp(name = "Color Sensor Study", group = "2 Bring-up")
public final class ColorSensorStudy extends TeamOpMode {
    public static double BASELINE_S = 8.0, READ_PERIOD_MS = 25.0, SLOW_POWER = 0.15, TURN_LIMIT_S = 25.0, NO_MOVE_S = 2.0, NO_MOVE_DEG = 5.0;
    private static final String[] STEPS = {
            "1. EMPTY BASELINE: track empty, hands off (about 8 s)",
            "2. EMPTY SWEEP: track empty; the pusher turns backward one full turn, slowly",
            "3. FULL BASELINE: load 4 pollen (hold X), then hands off (about 8 s)",
            "4. SLOW REJECT: 4 pollen in the track; the pusher turns backward one full turn, slowly, intake OUT"};

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Color Sensor Study"; }

    private int step, sensor, notes;
    private double[] latest;
    private Pusher pusher;

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        ColorReader cs = ColorReader.create(hardwareMap);
        pusher = Pusher.create(hardwareMap, profile, false);        // the zero is assumed: the arm must be at HOLD
        Intake intake = Intake.create(hardwareMap, profile);
        telemetry.addLine(cs == null ? "CS1 to CS4 are not all in the configuration" : "Color Sensor Study: the PUSHER MUST BE AT HOLD now. Press Start.");
        telemetry.addLine("A starts or stops a step; D-pad left/right changes the step; hold X to load; RB logs a note.");
        telemetry.update();
        waitForStart();
        if (isStopRequested() || cs == null) return;
        markStarted();
        RunLog.event("COUNTER stored baselines: " + SlotCounter.baselinesText());
        RunLog.sampler("step,running,sensor,alpha,cm,red,green,blue,kind,pusher_deg", 0, new RunLog.Sampler() {
            @Override public String row() {
                double[] r = latest;
                return String.format(Locale.US, "%d,%d,%d,%s,%s", step + 1, running ? 1 : 0, sensor + 1,
                        r == null ? ",,,," : String.format(Locale.US, "%.4f,%.2f,%.4f,%.4f,%.4f", r[0], r[1], r[2], r[3], r[4]),
                        (r == null ? "" : kindOf(sensor, r)) + "," + (pusher == null ? "" : String.format(Locale.US, "%.1f", pusher.angle())));
            }
        });
        Edge a = new Edge(), left = new Edge(), right = new Edge(), note = new Edge();
        LoopPad pad = new LoopPad();
        ColorReader.Mean[] mean = new ColorReader.Mean[4];
        int[] present = new int[4], arm = new int[4], reads = new int[4];
        double startedAt = 0, turnFrom = 0;
        String result = "";
        try {
            while (opModeIsActive()) {
                double t = Clock.now();
                boolean turnStep = step == 1 || step == 3;
                if (a.pressed(gamepad1.a)) {
                    if (running) {
                        running = false;
                        if (pusher != null) pusher.cancelMove();
                        RunLog.eventf("STUDY step %d stopped by A", step + 1);
                        result = "stopped";
                    } else if (turnStep && pusher == null) {
                        result = "no pusher in the configuration";
                    } else {
                        running = true;
                        startedAt = t;
                        for (int i = 0; i < 4; i++) {
                            mean[i] = new ColorReader.Mean();
                            present[i] = arm[i] = reads[i] = 0;
                        }
                        if (turnStep) {
                            turnFrom = pusher.angle();
                            pusher.jog(-SLOW_POWER);        // backward, open loop
                        }
                        RunLog.eventf("STUDY step %s started%s", STEPS[step], pusher == null ? "" : String.format(Locale.US, ", pusher at %.0f deg", pusher.angle()));
                        result = "";
                    }
                }
                if (!running) {
                    if (left.pressed(gamepad1.dpad_left)) step = Math.max(0, step - 1);
                    if (right.pressed(gamepad1.dpad_right)) step = Math.min(STEPS.length - 1, step + 1);
                }
                if (note.pressed(gamepad1.right_bumper)) {
                    notes++;
                    RunLog.eventf("STUDY NOTE %d at step %d%s", notes, step + 1, running ? " (running)" : "");
                }
                // One sensor is read every loop, and the loop is padded to the read period.
                sensor = (sensor + 1) % 4;
                latest = cs.read(sensor);
                if (running && latest != null) {
                    mean[sensor].add(latest);
                    reads[sensor]++;
                    SlotCounter.Kind k = kindOf(sensor, latest);
                    if (k != SlotCounter.Kind.EMPTY) present[sensor]++;
                    if (k == SlotCounter.Kind.PUSHER_ARM) arm[sensor]++;
                }
                if (running) {
                    double turnedDeg = turnStep ? turnFrom - pusher.angle() : 0;
                    String givenUp = !turnStep ? null : t - startedAt > TURN_LIMIT_S ? "not done after " + TURN_LIMIT_S + " s"
                            : (t - startedAt >= NO_MOVE_S && turnedDeg < NO_MOVE_DEG) ? "the arm has not moved " + NO_MOVE_DEG + " deg in " + NO_MOVE_S + " s" : null;
                    boolean done = turnStep ? turnedDeg >= 360.0 || givenUp != null : t - startedAt >= BASELINE_S;
                    if (done) {
                        if (turnStep) pusher.jog(0);
                        if (givenUp != null) RunLog.eventf("STUDY step %s GIVEN UP after %.0f deg: %s", STEPS[step], turnedDeg, givenUp);
                        running = false;
                        result = finish(mean, present, arm, reads);
                    }
                }
                if (intake != null) {
                    boolean load = !running && gamepad1.x;
                    intake.set(load ? Intake.Mode.IN : (running && step == 3) ? Intake.Mode.OUT : Intake.Mode.OFF);
                    intake.update(t);
                }
                if (pusher != null) pusher.update();
                RunLog.tick();
                telemetry.addLine(STEPS[step]);
                telemetry.addLine(running ? String.format(Locale.US, "RUNNING %.1f s (A stops)", t - startedAt) : "A starts it; D-pad left/right changes the step");
                telemetry.addLine("CS" + (sensor + 1) + ": " + ColorReader.text(latest));
                if (pusher != null) telemetry.addLine(String.format(Locale.US, "pusher %s at %.0f deg", pusher.state(), pusher.angle()));
                telemetry.addLine(result);
                telemetry.update();
                pad.pad(READ_PERIOD_MS);
            }
        } finally {
            if (pusher != null) pusher.stopNow();
            if (intake != null) intake.stopNow();
        }
    }

    private boolean running;

    private static SlotCounter.Kind kindOf(int i, double[] r) {
        return SlotCounter.presentAgainst(r[0], r[1], r[2], r[3], r[4], SlotCounter.storedEmpty(i)) ? SlotCounter.classify(i, r[2], r[3], r[4], r[1]) : SlotCounter.Kind.EMPTY;
    }

    /** Logs a finished step: the means of every sensor and, for the empty baseline, the lines to paste into the element counter. */
    private String finish(ColorReader.Mean[] mean, int[] present, int[] arm, int[] reads) {
        StringBuilder sb = new StringBuilder();
        double[][] m = new double[4][];
        for (int i = 0; i < 4; i++) {
            m[i] = mean[i].get();
            sb.append(String.format(Locale.US, "%sCS%d %s, %d readings, %d present, %d as the arm", i == 0 ? "" : "; ", i + 1, ColorReader.text(m[i]), reads[i], present[i], arm[i]));
        }
        RunLog.eventf("STUDY step %d done: %s", step + 1, sb);
        if (step == 0 && m[0] != null && m[1] != null && m[2] != null && m[3] != null) {
            String[] names = {"EMPTY_ALPHA", "EMPTY_CM", "EMPTY_RED", "EMPTY_GREEN", "EMPTY_BLUE"};
            String[] format = {"%.3f", "%.2f", "%.3f", "%.3f", "%.3f"};
            for (int k = 0; k < 5; k++) {
                RunLog.eventf("STUDY PASTE public static double[] %s = {" + format[k] + ", " + format[k] + ", " + format[k] + ", " + format[k] + "};",
                        names[k], m[0][k], m[1][k], m[2][k], m[3][k]);
            }
            return "done: the new empty readings are in the log (STUDY PASTE lines)";
        }
        return "done: " + sb;
    }
}
