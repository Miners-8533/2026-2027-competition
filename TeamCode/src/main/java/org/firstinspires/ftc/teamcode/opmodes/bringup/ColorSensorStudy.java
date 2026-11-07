package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * A guided check of the four element sensors (CS1 to CS4), for after the rotary transport or the lighting changes. Four steps, each started with A:
 *
 *   1. EMPTY baseline   the track empty and the pusher at HOLD, the robot still for about 8 s. The median reading of each sensor is the new empty
 *                       baseline; the opmode prints it as the three lines to paste into ElementCounter (BASE_ALPHA, BASE_DIST, BASE_SHARES).
 *   2. EMPTY sweep      the track still empty: the pusher turns backward one full turn slowly, so each sensor sees the arm pass. Every stretch where a
 *                       sensor sees something (against the baseline just measured) is listed with the pusher angle and what the counter would call it.
 *   3. FULL baseline    4 pollen loaded (check by eye), pusher at HOLD, still for about 8 s: what each sensor reads with pollen in front of it.
 *   4. FULL slow reject the same 4 pollen: the pusher turns backward one full turn slowly with the intake OUT, so the pollen slide past each sensor
 *                       in turn; the lit stretches are listed as in step 2. The pollen end up on the floor in front of the intake.
 *
 * Nothing else moves: no drive, no flywheel. The log (BRINGUP kind, about 20 rows a second, every sensor's raw reading next to what the counter reported
 * and the pusher angle) is read afterwards with tools/logs/sensor_study.py; the RESULT lines in it repeat what the screen shows. The sensors are read one
 * every 25 ms (each about every 100 ms), at the SDK's default rate and gain.
 *
 * Controls (gamepad 1; no Start or Back): A starts the step shown (and stops a running one), D-pad left / right goes to the previous / next step (to
 * repeat or skip one; it cannot move while a step is running), right bumper logs a NOTE, and holding X runs the intake IN so pollen can be loaded
 * (between steps only; it is off while a step runs and stops when X is released).
 *
 * The pusher must be at HOLD when the opmode is initialized (its zero is assumed there). A sweep that is aborted leaves the arm where it stopped: stop
 * the opmode and initialize again with the arm at HOLD before the next sweep. Steps 2 and 4 are the two steps in which the pusher moves.
 */
@Config
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@TeleOp(name = "Color Sensor Study", group = "Bring-up")
public class ColorSensorStudy extends LinearOpMode {

    /** One sensor is read this often (ms), cycling CS1 to CS4. */
    public static double SENSOR_READ_PERIOD_MS = 25.0;
    /** Rows in the log. */
    public static long SAMPLE_PERIOD_MS = 50;

    /** A still step: how long, and how much of the start is thrown away (the sensors settle, hands move away). */
    public static double STILL_SEC = 8.0;
    public static double STILL_SETTLE_SEC = 1.0;

    public static double SWEEP_POWER = 0.15;       // backward, open loop (about 120 deg/s empty)
    public static double SWEEP_DEG = 360.0;
    public static double SWEEP_MAX_SEC = 25.0;
    public static double SWEEP_NO_MOVE_SEC = 2.0;  // abort if the arm has not moved 5 deg by then (raise SWEEP_POWER)
    public static double LEAD_SEC = 2.0;
    public static double TAIL_SEC = 2.0;

    private enum Step {
        EMPTY_STILL("1. Empty baseline", 0, false, false,
                "Track EMPTY, pusher at HOLD, robot still. A: record the empty baseline (about 8 s)."),
        EMPTY_SWEEP("2. Pusher sweep, empty", 0, true, false,
                "Track still EMPTY. A: the pusher turns back one full turn slowly (about 5 s); keep hands clear."),
        FULL_STILL("3. Full baseline", 4, false, false,
                "Load 4 pollen (check by eye), pusher at HOLD, robot still. A: record (about 8 s)."),
        FULL_SWEEP("4. Slow reject, full", 4, true, true,
                "The same 4 pollen. A: the pusher turns back slowly with the intake OUT; the pollen slide out onto the floor.");

        final String title;
        final int known;
        final boolean sweep;
        final boolean intakeOut;
        final String prompt;

        Step(String title, int known, boolean sweep, boolean intakeOut, String prompt) {
            this.title = title;
            this.known = known;
            this.sweep = sweep;
            this.intakeOut = intakeOut;
            this.prompt = prompt;
        }
    }

    private enum Phase { WAIT, LEAD_IN, RUN, SWEEP, TAIL }

    private static final int NS = ElementCounter.N;

    // read by the log sampler
    private Step step = Step.EMPTY_STILL;
    private Phase phase = Phase.WAIT;
    private int takeId = 0;
    private boolean recording = false;
    private final ElapsedTime takeClock = new ElapsedTime();
    private final ElapsedTime phaseClock = new ElapsedTime();

    private ElementCounter counter;
    private Pusher pusher;
    private Intake intake;
    private double sweepStartTicks;

    private final int[] lastReadSeen = new int[NS];
    private final List<List<double[]>> samples = new ArrayList<>();   // per sensor: {alpha, dist, r, g, b} of the fresh reads of a still step
    private final List<List<double[]>> stretch = new ArrayList<>();   // per sensor: the reads of the stretch in progress during a sweep
    private final boolean[] present = new boolean[NS];
    private final double[] stretchFromDeg = new double[NS];
    private final double[] stretchToDeg = new double[NS];
    private final int[] stretchesFound = new int[NS];

    private boolean haveBaseline = false;
    private final double[] baseAlpha = new double[NS];
    private final double[] baseDist = new double[NS];
    private final double[][] baseShares = new double[NS][3];

    private final List<String> results = new ArrayList<>();       // the RESULT lines of the step last run
    private final List<String> baselineLines = new ArrayList<>(); // the three paste lines, kept on screen
    private boolean allDone = false;
    private String warning = "";

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Color Sensor Study", hardwareMap);
        double savedReadPeriod = ElementCounter.READ_PERIOD_MS;
        try {
            study();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            ElementCounter.READ_PERIOD_MS = savedReadPeriod;
            RunLog.close();
        }
    }

    private static String columns() {
        StringBuilder sb = new StringBuilder("take,type,phase,known,take_s,intake");
        for (int i = 1; i <= ElementCounter.N; i++) {
            String p = ",cs" + i + "_";
            sb.append(p).append("dist").append(p).append("alpha").append(p).append("r").append(p).append("g").append(p).append("b")
                    .append(p).append("shift").append(p).append("on").append(p).append("n");
        }
        sb.append(",slots,raw_count,count,pusher_deg,pusher_unwrapped_deg,pusher_state,pusher_dps,pusher_amps,intake_amps");
        return sb.toString();
    }

    private String row() {
        StringBuilder sb = new StringBuilder();
        sb.append(recording ? takeId : 0).append(',').append(recording ? step.name() : "NONE").append(',').append(phase).append(',')
                .append(recording ? step.known : 0).append(',');
        sb.append(String.format(Locale.US, "%.2f", recording ? takeClock.seconds() : 0.0)).append(',').append(intake.getMode());
        for (int i = 0; i < ElementCounter.N; i++) {
            sb.append(String.format(Locale.US, ",%.2f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,%d", counter.lastDistCm(i), counter.lastAlpha(i),
                    counter.lastShare(i, 0), counter.lastShare(i, 1), counter.lastShare(i, 2), counter.colorShift(i),
                    counter.rawPresent(i) ? 1 : 0, counter.readCount(i)));
        }
        double dps = pusher.getVelocityTicksPerSec() * 360.0 / Pusher.TICKS_PER_REV;
        sb.append(String.format(Locale.US, ",%s,%d,%d,%.1f,%.1f,%s,%.0f,%.2f,%.2f",
                counter.slotsCompact(), counter.elementsNow(), counter.count(), pusher.getAngleDeg(),
                pusher.getUnwrappedTicks() * 360.0 / Pusher.TICKS_PER_REV, pusher.getState(), dps, pusher.getCurrentAmps(), intake.getCurrentAmps()));
        return sb.toString();
    }

    // ---- the steps ----

    private void startStep() {
        takeId++;
        recording = true;
        results.clear();
        warning = "";
        for (int i = 0; i < NS; i++) {
            samples.get(i).clear();
            stretch.get(i).clear();
            present[i] = false;
            stretchesFound[i] = 0;
        }
        takeClock.reset();
        phaseClock.reset();
        intake.setMode(Intake.Mode.OFF);
        phase = step.sweep ? Phase.LEAD_IN : Phase.RUN;
        RunLog.event(String.format(Locale.US, "TAKE %d START: %s (known pollen %d), pusher %.0f deg, sensor read period %.0f ms",
                takeId, step.title, step.known, pusher.getAngleDeg(), ElementCounter.READ_PERIOD_MS));
    }

    private void endStep(String how, boolean ok) {
        if (step.sweep) {
            pusher.stopInPlace();
            intake.setMode(Intake.Mode.OFF);
        }
        RunLog.event(String.format(Locale.US, "TAKE %d END (%s): %.1f s, counter said %d, %d sensors lit, pusher %.0f deg", takeId, how, takeClock.seconds(),
                counter.count(), counter.elementsNow(), pusher.getAngleDeg()));
        recording = false;
        phase = Phase.WAIT;
        if (ok) {
            if (step.ordinal() + 1 < Step.values().length) step = Step.values()[step.ordinal() + 1];
            else allDone = true;
        } else if (step.sweep) {
            warning = "The pusher may not be at HOLD: stop the opmode and initialize again before the next sweep.";
        }
    }

    private void result(String line) {
        results.add(line);
        RunLog.event("RESULT " + line);
    }

    // ---- reads ----

    /** One fresh reading of sensor i (the read counter moved). */
    private void freshRead(int i) {
        double[] r = {counter.lastAlpha(i), counter.lastDistCm(i), counter.lastShare(i, 0), counter.lastShare(i, 1), counter.lastShare(i, 2)};
        if (!step.sweep && phase == Phase.RUN && takeClock.seconds() >= STILL_SETTLE_SEC) {
            samples.get(i).add(r);
        } else if (step.sweep && phase == Phase.SWEEP && haveBaseline) {
            boolean now = ElementCounter.presenceNext(present[i], r[2], r[3], r[4], r[0], r[1],
                    baseShares[i][0], baseShares[i][1], baseShares[i][2], baseAlpha[i], baseDist[i]);
            double deg = (pusher.getUnwrappedTicks() - sweepStartTicks) * 360.0 / Pusher.TICKS_PER_REV;   // negative: backward
            if (now) {
                if (!present[i]) {
                    stretch.get(i).clear();
                    stretchFromDeg[i] = deg;
                }
                stretch.get(i).add(r);
                stretchToDeg[i] = deg;
            } else if (present[i]) {
                closeStretch(i);
            }
            present[i] = now;
        }
    }

    private void closeStretch(int i) {
        List<double[]> s = stretch.get(i);
        if (s.isEmpty()) return;
        double a = median(s, 0), d = median(s, 1), red = median(s, 2), g = median(s, 3), b = median(s, 4);
        stretchesFound[i]++;
        result(String.format(Locale.US, "%s lit at pusher %.0f to %.0f deg (%d reads): looks like %s, rgb %.2f/%.2f/%.2f dist %.1f cm alpha %.3f",
                ElementCounter.NAMES[i], stretchFromDeg[i], stretchToDeg[i], s.size(), ElementCounter.classify(i, red, g, b, d), red, g, b, d, a));
        s.clear();
    }

    private static double median(List<double[]> list, int col) {
        if (list.isEmpty()) return Double.NaN;
        double[] v = new double[list.size()];
        for (int k = 0; k < v.length; k++) v[k] = list.get(k)[col];
        Arrays.sort(v);
        return v.length % 2 == 1 ? v[v.length / 2] : 0.5 * (v[v.length / 2 - 1] + v[v.length / 2]);
    }

    private void finishStill() {
        for (int i = 0; i < NS; i++) {
            List<double[]> s = samples.get(i);
            if (s.size() < 5) {
                result(String.format(Locale.US, "%s: only %d fresh reads, not enough (is the sensor reading?)", ElementCounter.NAMES[i], s.size()));
                continue;
            }
            double a = median(s, 0), d = median(s, 1), red = median(s, 2), g = median(s, 3), b = median(s, 4);
            double aMin = Double.MAX_VALUE, aMax = -Double.MAX_VALUE, dMin = Double.MAX_VALUE, dMax = -Double.MAX_VALUE;
            for (double[] r : s) {
                aMin = Math.min(aMin, r[0]);
                aMax = Math.max(aMax, r[0]);
                dMin = Math.min(dMin, r[1]);
                dMax = Math.max(dMax, r[1]);
            }
            if (step == Step.EMPTY_STILL) {
                baseAlpha[i] = a;
                baseDist[i] = d;
                baseShares[i][0] = red;
                baseShares[i][1] = g;
                baseShares[i][2] = b;
                result(String.format(Locale.US, "%s empty: alpha %.3f (%.3f to %.3f), dist %.2f cm (%.2f to %.2f), rgb %.3f/%.3f/%.3f, %d reads",
                        ElementCounter.NAMES[i], a, aMin, aMax, d, dMin, dMax, red, g, b, s.size()));
            } else {
                String vs = "";
                if (haveBaseline) {
                    double shift = Math.sqrt(Math.pow(red - baseShares[i][0], 2) + Math.pow(g - baseShares[i][1], 2) + Math.pow(b - baseShares[i][2], 2));
                    boolean on = ElementCounter.presenceNext(false, red, g, b, a, d, baseShares[i][0], baseShares[i][1], baseShares[i][2], baseAlpha[i], baseDist[i]);
                    vs = String.format(Locale.US, ", shift %.3f from the empty baseline -> %s", shift, on ? "LIT" : "not lit");
                }
                result(String.format(Locale.US, "%s with pollen nearby: looks like %s, alpha %.3f, dist %.2f cm (%.2f to %.2f), rgb %.3f/%.3f/%.3f%s",
                        ElementCounter.NAMES[i], ElementCounter.classify(i, red, g, b, d), a, d, dMin, dMax, red, g, b, vs));
            }
        }
        if (step == Step.EMPTY_STILL) {
            boolean ok = true;
            for (int i = 0; i < NS; i++) ok &= samples.get(i).size() >= 5;
            haveBaseline = ok;
            if (ok) {
                baselineLines.clear();
                baselineLines.add(String.format(Locale.US, "BASE_ALPHA = {%.3f, %.3f, %.3f, %.3f};", baseAlpha[0], baseAlpha[1], baseAlpha[2], baseAlpha[3]));
                baselineLines.add(String.format(Locale.US, "BASE_DIST = {%.2f, %.2f, %.2f, %.2f};", baseDist[0], baseDist[1], baseDist[2], baseDist[3]));
                StringBuilder sh = new StringBuilder("BASE_SHARES = {");
                for (int i = 0; i < NS; i++) {
                    sh.append(String.format(Locale.US, "{%.3f, %.3f, %.3f}%s", baseShares[i][0], baseShares[i][1], baseShares[i][2], i < NS - 1 ? ", " : "};"));
                }
                baselineLines.add(sh.toString());
                for (String l : baselineLines) RunLog.event("BASELINE (paste into ElementCounter) " + l);
            } else {
                warning = "Not every sensor gave enough reads: the baseline was not set. Repeat step 1.";
            }
        }
    }

    private void finishSweep() {
        for (int i = 0; i < NS; i++) {
            if (present[i]) closeStretch(i);
            present[i] = false;
        }
        if (!haveBaseline) {
            result("No baseline measured in this run, so the lit stretches are not listed (do step 1 first); the log has every reading.");
            return;
        }
        for (int i = 0; i < NS; i++) {
            if (stretchesFound[i] == 0) result(ElementCounter.NAMES[i] + " never lit during the sweep");
        }
    }

    private void study() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElementCounter.READ_PERIOD_MS = SENSOR_READ_PERIOD_MS;
        for (int i = 0; i < NS; i++) {
            samples.add(new ArrayList<>());
            stretch.add(new ArrayList<>());
        }

        counter = ElementCounter.tryCreate(hardwareMap);
        if (counter == null) {
            while (!isStopRequested()) {
                telemetry.addLine("The four color sensors CS1 to CS4 are not all configured. Fix the hub configuration.");
                telemetry.update();
            }
            return;
        }
        pusher = new Pusher(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);
        pusher.setAutoCorrect(false);   // nothing moves the arm except the sweeps
        intake = new Intake(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            counter.update();
            pusher.update();
            telemetry.addLine("Color Sensor Study. The pusher is at HOLD now (its zero is assumed here) and the track should be EMPTY. Keep the robot still until Start.");
            addSensorLines();
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.setSampler(columns(), SAMPLE_PERIOD_MS, this::row);
        RunLog.event(String.format(Locale.US, "SETUP sensor read period %.0f ms (the robot's counter uses 40), log rows every %d ms, still %.0f s, sweep power %.2f for %.0f deg",
                ElementCounter.READ_PERIOD_MS, SAMPLE_PERIOD_MS, STILL_SEC, SWEEP_POWER, SWEEP_DEG));
        RunLog.event("SETUP stored baselines: " + ElementCounter.describeBaselines());

        while (opModeIsActive()) {
            counter.update();
            pusher.update();
            intake.update();
            for (int i = 0; i < NS; i++) {
                int n = counter.readCount(i);
                if (n != lastReadSeen[i]) {
                    lastReadSeen[i] = n;
                    if (recording) freshRead(i);
                }
            }

            if (!recording) {   // a loading aid, not part of a step: hold X to run the intake IN while loading pollen
                Intake.Mode wanted = gamepad1.x ? Intake.Mode.IN : Intake.Mode.OFF;
                if (intake.getMode() != wanted) intake.setMode(wanted);
            }
            if (phase == Phase.WAIT) {
                if (gamepad1.dpadRightWasPressed() && step.ordinal() + 1 < Step.values().length) {
                    step = Step.values()[step.ordinal() + 1];
                    allDone = false;
                    results.clear();
                } else if (gamepad1.dpadLeftWasPressed() && step.ordinal() > 0) {
                    step = Step.values()[step.ordinal() - 1];
                    allDone = false;
                    results.clear();
                }
            }
            if (gamepad1.rightBumperWasPressed()) {
                RunLog.event(String.format(Locale.US, "NOTE (take %d, %.1f s, counter %d, %d lit)", recording ? takeId : 0,
                        recording ? takeClock.seconds() : 0.0, counter.count(), counter.elementsNow()));
            }
            if (gamepad1.aWasPressed()) {
                if (!recording) {
                    allDone = false;
                    startStep();
                } else if (step.sweep) {
                    endStep("ABORTED by the driver", false);
                } else {
                    endStep("stopped by the driver", false);
                }
            }

            if (recording && !step.sweep) {
                if (phase == Phase.RUN && takeClock.seconds() >= STILL_SEC) {
                    finishStill();
                    endStep("complete", true);
                }
            } else if (recording) {
                if (phase == Phase.LEAD_IN && phaseClock.seconds() >= LEAD_SEC) {
                    phase = Phase.SWEEP;
                    phaseClock.reset();
                    sweepStartTicks = pusher.getUnwrappedTicks();
                    if (step.intakeOut) intake.setMode(Intake.Mode.OUT);
                    pusher.jog(-SWEEP_POWER);
                    RunLog.event(String.format(Locale.US, "TAKE %d SWEEP start: backward at %.2f power, intake %s", takeId, SWEEP_POWER, intake.getMode()));
                } else if (phase == Phase.SWEEP) {
                    double travelled = (sweepStartTicks - pusher.getUnwrappedTicks()) * 360.0 / Pusher.TICKS_PER_REV;
                    if (travelled >= SWEEP_DEG) {
                        pusher.stopInPlace();
                        intake.setMode(Intake.Mode.OFF);
                        RunLog.event(String.format(Locale.US, "TAKE %d SWEEP done: %.0f deg in %.1f s", takeId, travelled, phaseClock.seconds()));
                        finishSweep();
                        phase = Phase.TAIL;
                        phaseClock.reset();
                    } else if (phaseClock.seconds() > SWEEP_NO_MOVE_SEC && travelled < 5.0) {
                        endStep("ABORTED: the arm did not move; raise SWEEP_POWER", false);
                    } else if (phaseClock.seconds() > SWEEP_MAX_SEC) {
                        endStep("ABORTED: the sweep took too long", false);
                    }
                } else if (phase == Phase.TAIL && phaseClock.seconds() >= TAIL_SEC) {
                    endStep("complete", true);
                }
            }

            showScreen();
            RunLog.tick();
        }
        pusher.stopInPlace();
        intake.setMode(Intake.Mode.OFF);
    }

    private void showScreen() {
        telemetry.addData("Step", step.title + (allDone ? "   (all four steps done; D-pad left to repeat one)" : ""));
        if (recording) {
            String what = phase == Phase.RUN ? String.format(Locale.US, "recording, %.0f s of %.0f s", takeClock.seconds(), STILL_SEC)
                    : phase == Phase.LEAD_IN ? "still, the sweep starts soon" : phase == Phase.SWEEP ? "SWEEPING (A aborts)" : "still, finishing";
            telemetry.addData("Now", what);
        } else {
            telemetry.addData("Next", step.prompt);
        }
        if (!warning.isEmpty()) telemetry.addData("WARNING", warning);
        if (!baselineLines.isEmpty()) {
            telemetry.addLine("New empty baseline, to paste into ElementCounter:");
            for (String l : baselineLines) telemetry.addLine("  " + l);
        } else if (!haveBaseline) {
            telemetry.addLine("No empty baseline measured yet: do step 1 first.");
        }
        if (!results.isEmpty()) {
            telemetry.addLine("Results of the step just run (also in the log):");
            for (String l : results) telemetry.addLine("  " + l);
        }
        telemetry.addData("Pusher", "%.0f deg, %s", pusher.getAngleDeg(), pusher.getState());
        addSensorLines();
        telemetry.addLine("A = start / stop, D-pad left / right = previous / next step, right bumper = NOTE, hold X = intake IN to load pollen (between steps only).");
        telemetry.update();
    }

    private void addSensorLines() {
        for (int i = 0; i < NS; i++) {
            telemetry.addData(ElementCounter.NAMES[i], "%s  dist %.1f cm  alpha %.3f  rgb %.2f / %.2f / %.2f  shift %.3f  reads %d",
                    counter.rawPresent(i) ? "LIT" : "---", counter.lastDistCm(i), counter.lastAlpha(i), counter.lastShare(i, 0), counter.lastShare(i, 1),
                    counter.lastShare(i, 2), counter.colorShift(i), counter.readCount(i));
        }
        telemetry.addData("Counter", "%s   %d lit now, reported %d%s", counter.slotsCompact(), counter.elementsNow(), counter.count(),
                counter.isValid() ? "" : "   (sensors not read recently)");
    }
}
