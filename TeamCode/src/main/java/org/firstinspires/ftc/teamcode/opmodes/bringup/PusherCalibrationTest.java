package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;
import org.firstinspires.ftc.teamcode.subsystems.EntryCounter;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSensor;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.PusherCheck;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * PUSHER CALIBRATION TEST (2026-11-19, the user's request, after the pusher check stopped finding the arm at CS4; built, not yet run on the robot): turns the pusher slowly through its travel with NO ELEMENTS
 * in the robot and records what the arm looks like to EACH color sensor, CS1 to CS5, at every pusher angle.
 *
 *   1. BASELINE   the track empty, the pusher at HOLD, the robot still for STILL_SEC: the empty reading of every sensor (CS1, CS3, CS4: distance, alpha and color shares through the counter; CS2 and CS5: the
 *                 proximity of their own threads).
 *   2. FORWARD    the pusher turns FORWARD (the way a salvo turns) at SWEEP_POWER, open loop (about 120 deg/s), SWEEP_DEG past HOLD (420: the pusher check's sweep, so the arm passes CS4 once at about 327 deg and
 *                 again a turn later) and stops.
 *   3. BACK       the pusher turns back to where step 2 started (HOLD).
 *
 * For every sensor and every pusher angle the log has the raw readings, and every stretch where a sensor sees something (the counter's own presence rule for CS1, CS3, CS4; a proximity rise of PROX_RISE for
 * CS2 and CS5) is listed with the angle range, the peak angle and what the counter would call it (RESULT lines, also on the screen). The PROFILE lines give the strongest signal of each BIN_DEG of pusher
 * angle (the shift of the color shares for CS1, CS3, CS4; the proximity rise for CS2 and CS5), so what the arm looks like to a sensor is there even where the presence rule said no. CS4 is compared with where the
 * pusher check parks the arm (PusherCheck.ARM_CENTER_DEG). Nothing else moves: no drive, no flywheel, no intake.
 *
 * CS5 and CS2: their switches (EntryCounter.CS5_PRESENT, LaunchSensor.CS2_PRESENT) are off in the competition code, so this test turns both ON while it runs and restores them when it ends; a sensor that is not in the
 * hub configuration is still shown as not configured.
 *
 * Gamepad 1 (no Start or Back): A starts the step shown (and stops a running one), D-pad left / right goes to the previous / next step (between steps), right bumper logs a NOTE.
 *
 * The pusher must be at HOLD when the opmode is initialized (its zero is assumed there), and the robot must be still and EMPTY (no pollen or nectar anywhere in the track or the ramp). A sweep that is aborted
 * leaves the arm where it stopped: stop the opmode and initialize again with the arm at HOLD before the next run. The log is a BRINGUP log (about 25 rows a second).
 */
@Config
@TeleOp(name = "Pusher Calibration Test", group = "Bring-up")
public class PusherCalibrationTest extends LinearOpMode {

    public static double SWEEP_POWER = 0.15;       // open loop, about 120 deg/s empty (the pusher check's sweep power)
    public static double SWEEP_DEG = 420.0;
    public static double SWEEP_MAX_SEC = 30.0;
    public static double SWEEP_NO_MOVE_SEC = 2.0;  // abort if the arm has not moved 5 deg by then (raise SWEEP_POWER)
    public static double STILL_SEC = 4.0;
    public static double STILL_SETTLE_SEC = 1.0;
    public static double LEAD_SEC = 1.0, TAIL_SEC = 1.0;
    /** CS2 and CS5 see the arm when their proximity is at least this many counts above the empty median. */
    public static double PROX_RISE = 4.0;
    /** The width of a PROFILE bin (degrees of pusher angle). */
    public static double BIN_DEG = 10.0;
    /** One of CS1, CS3, CS4 is read this often (ms), cycling; as in the Color Sensor Study. */
    public static double SENSOR_READ_PERIOD_MS = 25.0;
    public static long SAMPLE_PERIOD_MS = 40;

    private static final String[] NAMES = {"CS1", "CS2", "CS3", "CS4", "CS5"};
    private static final int NS = NAMES.length;
    /** Sensors read through the ElementCounter (the color ones along the track); CS2 and CS5 are proximity threads. */
    private static final boolean[] COLOR = {true, false, true, true, false};
    private static final int CS2 = 1, CS4 = 3, CS5 = 4;

    private enum Step {
        BASELINE("1. Empty baseline", false, 0, "Track EMPTY, pusher at HOLD, robot still. A: record the empty readings."),
        FORWARD("2. Sweep forward", true, +1, "Track still EMPTY. A: the pusher turns forward slowly (about 4 s); keep hands clear."),
        BACK("3. Sweep back", true, -1, "A: the pusher turns back to HOLD slowly; keep hands clear.");

        final String title;
        final boolean sweep;
        final int dir;
        final String prompt;

        Step(String title, boolean sweep, int dir, String prompt) {
            this.title = title;
            this.sweep = sweep;
            this.dir = dir;
            this.prompt = prompt;
        }
    }

    private enum Phase { WAIT, LEAD_IN, RUN, SWEEP, TAIL }

    private ElementCounter counter;
    private EntryCounter cs5;
    private LaunchSensor cs2;
    private Pusher pusher;

    private Step step = Step.BASELINE;
    private Phase phase = Phase.WAIT;
    private boolean recording = false;
    private boolean allDone = false;
    private String warning = "";
    private int takeId = 0;
    private final ElapsedTime takeClock = new ElapsedTime();
    private final ElapsedTime phaseClock = new ElapsedTime();
    private double holdTicks = Double.NaN;   // the unwrapped ticks of the HOLD where the baseline was taken: angles are measured from it

    private final int[] lastReadSeen = new int[NS];
    private final List<List<double[]>> samples = new ArrayList<>();   // per sensor, a still step: {alpha, dist, r, g, b} or {prox}
    private final boolean[] haveBase = new boolean[NS];
    private final double[] baseAlpha = new double[NS], baseDist = new double[NS], baseProx = new double[NS];
    private final double[][] baseShares = new double[NS][3];

    // the sweep in progress
    private final boolean[] lit = new boolean[NS];
    private final List<List<double[]>> stretch = new ArrayList<>();   // per sensor: {alpha, dist, r, g, b, signal, deg} of the reads of the stretch in progress
    private final int[] stretchesFound = new int[NS];
    private double[][] profile;       // [sensor][bin]: the strongest signal seen in the bin during this sweep
    private final List<String> results = new ArrayList<>();
    private final List<String> verdicts = new ArrayList<>();
    private final double[] cs4Centers = new double[]{Double.NaN, Double.NaN};   // CS4's stretch centers (forward, back) for the verdict
    private double sweepStartDeg;

    private double degNow() {
        return Double.isNaN(holdTicks) ? 0.0 : (pusher.getUnwrappedTicks() - holdTicks) * 360.0 / Pusher.TICKS_PER_REV;
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Pusher Calibration Test", hardwareMap);
        double savedReadPeriod = ElementCounter.READ_PERIOD_MS;
        // the CS5 and CS2 switches are OFF in the competition code (2026-11-20); this test needs to see the arm on both, so it turns them on for its own run and puts them back at the end
        boolean savedCs5 = EntryCounter.CS5_PRESENT, savedCs2 = LaunchSensor.CS2_PRESENT;
        EntryCounter.CS5_PRESENT = true;
        LaunchSensor.CS2_PRESENT = true;
        RunLog.event(String.format(Locale.US, "SETUP the CS5 and CS2 switches are forced ON for this test (they were CS5 %s, CS2 %s) and restored at the end", savedCs5 ? "on" : "off", savedCs2 ? "on" : "off"));
        try {
            test();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            ElementCounter.READ_PERIOD_MS = savedReadPeriod;
            EntryCounter.CS5_PRESENT = savedCs5;
            LaunchSensor.CS2_PRESENT = savedCs2;
            if (cs5 != null) cs5.stop();
            if (cs2 != null) cs2.stop();
            RunLog.close();
        }
    }

    private static String columns() {
        StringBuilder sb = new StringBuilder("take,step,phase,take_s,deg");
        for (int i = 0; i < NS; i++) {
            if (COLOR[i]) {
                String p = ",cs" + (i + 1) + "_";
                sb.append(p).append("dist").append(p).append("alpha").append(p).append("r").append(p).append("g").append(p).append("b").append(p).append("shift");
            } else {
                sb.append(",cs").append(i + 1).append("_prox");
            }
        }
        sb.append(",pusher_state,pusher_dps,pusher_amps");
        return sb.toString();
    }

    private String row() {
        StringBuilder sb = new StringBuilder();
        sb.append(recording ? takeId : 0).append(',').append(recording ? step.name() : "NONE").append(',').append(phase).append(',')
                .append(String.format(Locale.US, "%.2f,%.1f", recording ? takeClock.seconds() : 0.0, degNow()));
        for (int i = 0; i < NS; i++) {
            if (COLOR[i]) {
                sb.append(String.format(Locale.US, ",%.2f,%.4f,%.3f,%.3f,%.3f,%.3f", counter.lastDistCm(i), counter.lastAlpha(i), counter.lastShare(i, 0), counter.lastShare(i, 1),
                        counter.lastShare(i, 2), counter.colorShift(i)));
            } else {
                sb.append(',').append(prox(i));
            }
        }
        double dps = pusher.getVelocityTicksPerSec() * 360.0 / Pusher.TICKS_PER_REV;
        sb.append(String.format(Locale.US, ",%s,%.0f,%.2f", pusher.getState(), dps, pusher.getCurrentAmps()));
        return sb.toString();
    }

    /** The latest proximity of CS2 or CS5, or -1 when that sensor is not configured. */
    private int prox(int i) {
        if (i == CS2) return cs2 == null ? -1 : cs2.latestProx();
        if (i == CS5) return cs5 == null ? -1 : cs5.latestProx();
        return -1;
    }

    private boolean present(int i) {
        if (COLOR[i]) return true;
        return i == CS2 ? cs2 != null && cs2.isStarted() : cs5 != null && cs5.isStarted();
    }

    // ---- the steps ----

    private void startStep() {
        takeId++;
        recording = true;
        warning = "";
        results.clear();
        for (int i = 0; i < NS; i++) {
            samples.get(i).clear();
            stretch.get(i).clear();
            lit[i] = false;
            stretchesFound[i] = 0;
        }
        takeClock.reset();
        phaseClock.reset();
        if (step == Step.BASELINE) holdTicks = pusher.getUnwrappedTicks();
        else if (Double.isNaN(holdTicks)) {
            warning = "No baseline yet: do step 1 first (the angles are measured from the HOLD where it was taken).";
            recording = false;
            return;
        }
        if (step.sweep) {
            int bins = (int) Math.ceil((SWEEP_DEG + 40.0) / BIN_DEG) + 1;
            profile = new double[NS][bins];
            for (double[] p : profile) Arrays.fill(p, Double.NaN);
        }
        phase = step.sweep ? Phase.LEAD_IN : Phase.RUN;
        RunLog.event(String.format(Locale.US, "TAKE %d START: %s, pusher %.0f deg from the baseline HOLD, sweep power %.2f for %.0f deg", takeId, step.title, degNow(), SWEEP_POWER, SWEEP_DEG));
    }

    private void endStep(String how, boolean ok) {
        if (step.sweep) pusher.stopInPlace();
        RunLog.event(String.format(Locale.US, "TAKE %d END (%s): %.1f s, pusher %.0f deg", takeId, how, takeClock.seconds(), degNow()));
        recording = false;
        phase = Phase.WAIT;
        if (ok) {
            if (step.ordinal() + 1 < Step.values().length) step = Step.values()[step.ordinal() + 1];
            else allDone = true;
        } else if (step.sweep) {
            warning = "The pusher may not be at HOLD: stop the opmode and initialize again before the next run.";
        }
    }

    private void result(String line) {
        results.add(line);
        RunLog.event("RESULT " + line);
    }

    // ---- reads ----

    /** A fresh reading of a color sensor (CS1, CS3, CS4): the read counter moved. */
    private void freshColor(int i) {
        double[] r = {counter.lastAlpha(i), counter.lastDistCm(i), counter.lastShare(i, 0), counter.lastShare(i, 1), counter.lastShare(i, 2)};
        if (step == Step.BASELINE) {
            if (phase == Phase.RUN && takeClock.seconds() >= STILL_SETTLE_SEC) samples.get(i).add(r);
        } else if (step.sweep && phase == Phase.SWEEP && haveBase[i]) {
            double shift = Math.sqrt(Math.pow(r[2] - baseShares[i][0], 2) + Math.pow(r[3] - baseShares[i][1], 2) + Math.pow(r[4] - baseShares[i][2], 2));
            boolean now = ElementCounter.presenceNext(lit[i], r[2], r[3], r[4], r[0], r[1], baseShares[i][0], baseShares[i][1], baseShares[i][2], baseAlpha[i], baseDist[i]);
            sweepSample(i, now, new double[]{r[0], r[1], r[2], r[3], r[4], shift, degNow()});
        }
    }

    /** A proximity reading of CS2 or CS5, taken every loop. */
    private void freshProx(int i) {
        int p = prox(i);
        if (p < 0 || !present(i)) return;
        if (step == Step.BASELINE) {
            if (phase == Phase.RUN && takeClock.seconds() >= STILL_SETTLE_SEC) samples.get(i).add(new double[]{p});
        } else if (step.sweep && phase == Phase.SWEEP && haveBase[i]) {
            double rise = p - baseProx[i];
            boolean now = lit[i] ? rise >= PROX_RISE * 0.5 : rise >= PROX_RISE;
            sweepSample(i, now, new double[]{Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, rise, degNow()});
        }
    }

    private void sweepSample(int i, boolean now, double[] s) {
        int bin = (int) Math.max(0, Math.min(profile[i].length - 1, Math.floor(s[6] / BIN_DEG)));
        if (Double.isNaN(profile[i][bin]) || s[5] > profile[i][bin]) profile[i][bin] = s[5];
        if (now) stretch.get(i).add(s);
        else if (lit[i]) closeStretch(i);
        lit[i] = now;
    }

    private void closeStretch(int i) {
        List<double[]> s = stretch.get(i);
        if (s.isEmpty()) return;
        double from = Double.MAX_VALUE, to = -Double.MAX_VALUE, peak = -Double.MAX_VALUE, peakDeg = Double.NaN;
        for (double[] r : s) {
            from = Math.min(from, r[6]);
            to = Math.max(to, r[6]);
            if (r[5] > peak) {
                peak = r[5];
                peakDeg = r[6];
            }
        }
        stretchesFound[i]++;
        if (COLOR[i]) {
            double a = median(s, 0), d = median(s, 1), red = median(s, 2), g = median(s, 3), b = median(s, 4);
            result(String.format(Locale.US, "%s %s sees something at pusher %.0f to %.0f deg (%d reads), peak shift %.3f at %.0f deg: counter would say %s; rgb %.2f/%.2f/%.2f dist %.1f cm alpha %.3f",
                    NAMES[i], step.dir > 0 ? "FWD" : "BACK", from, to, s.size(), peak, peakDeg, ElementCounter.classify(i, red, g, b, d), red, g, b, d, a));
        } else {
            result(String.format(Locale.US, "%s %s sees something at pusher %.0f to %.0f deg (%d reads), peak proximity rise +%.0f at %.0f deg",
                    NAMES[i], step.dir > 0 ? "FWD" : "BACK", from, to, s.size(), peak, peakDeg));
        }
        if (i == CS4) cs4Centers[step.dir > 0 ? 0 : 1] = 0.5 * (from + to);
        s.clear();
    }

    private static double median(List<double[]> list, int col) {
        if (list.isEmpty()) return Double.NaN;
        double[] v = new double[list.size()];
        for (int k = 0; k < v.length; k++) v[k] = list.get(k)[col];
        Arrays.sort(v);
        return v.length % 2 == 1 ? v[v.length / 2] : 0.5 * (v[v.length / 2 - 1] + v[v.length / 2]);
    }

    private void finishBaseline() {
        boolean all = true;
        for (int i = 0; i < NS; i++) {
            List<double[]> s = samples.get(i);
            haveBase[i] = false;
            if (!present(i)) {
                result(NAMES[i] + ": not configured or not answering, so not measured");
                continue;
            }
            if (s.size() < 5) {
                all &= !COLOR[i];
                result(String.format(Locale.US, "%s: only %d fresh reads, not enough (is the sensor reading?)", NAMES[i], s.size()));
                continue;
            }
            if (COLOR[i]) {
                baseAlpha[i] = median(s, 0);
                baseDist[i] = median(s, 1);
                baseShares[i][0] = median(s, 2);
                baseShares[i][1] = median(s, 3);
                baseShares[i][2] = median(s, 4);
                result(String.format(Locale.US, "%s empty: alpha %.3f, dist %.2f cm, rgb %.3f/%.3f/%.3f, %d reads", NAMES[i], baseAlpha[i], baseDist[i], baseShares[i][0], baseShares[i][1], baseShares[i][2], s.size()));
            } else {
                baseProx[i] = median(s, 0);
                double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
                for (double[] r : s) {
                    lo = Math.min(lo, r[0]);
                    hi = Math.max(hi, r[0]);
                }
                result(String.format(Locale.US, "%s empty: proximity median %.0f (range %.0f to %.0f), %d reads", NAMES[i], baseProx[i], lo, hi, s.size()));
            }
            haveBase[i] = true;
        }
        if (!all) warning = "A color sensor did not give enough reads. Repeat step 1.";
    }

    private void finishSweep() {
        for (int i = 0; i < NS; i++) {
            if (lit[i]) closeStretch(i);
            lit[i] = false;
        }
        for (int i = 0; i < NS; i++) {
            if (!haveBase[i]) continue;
            if (stretchesFound[i] == 0) result(NAMES[i] + " never saw anything during the " + (step.dir > 0 ? "forward" : "back") + " sweep");
            StringBuilder sb = new StringBuilder();
            for (int b = 0; b < profile[i].length; b++) {
                if (!Double.isNaN(profile[i][b])) sb.append(String.format(Locale.US, "%.0f:%.3f ", b * BIN_DEG, profile[i][b]));
            }
            RunLog.event(String.format(Locale.US, "PROFILE %s %s (pusher deg of the bin start : strongest signal, %s): %s", NAMES[i], step.dir > 0 ? "FWD" : "BACK",
                    COLOR[i] ? "shift of the color shares" : "proximity rise", sb.toString().trim()));
        }
        if (step == Step.BACK) {
            verdicts.clear();
            String fwd = Double.isNaN(cs4Centers[0]) ? "never" : String.format(Locale.US, "%.0f deg", cs4Centers[0]);
            String back = Double.isNaN(cs4Centers[1]) ? "never" : String.format(Locale.US, "%.0f deg", cs4Centers[1]);
            String line = String.format(Locale.US, "CS4 sees the arm: forward at %s, back at %s; the pusher check parks the arm at %.1f deg ahead of HOLD (a second pass of CS4 is expected a turn later).",
                    fwd, back, PusherCheck.ARM_CENTER_DEG);
            verdicts.add(line);
            RunLog.event("VERDICT " + line);
            if (Double.isNaN(cs4Centers[0]) && Double.isNaN(cs4Centers[1])) {
                String v = "CS4 never saw the arm in either direction: the sensor, its wiring, or its view of the arm. The PROFILE lines show whether it moved at all.";
                verdicts.add(v);
                RunLog.event("VERDICT " + v);
            }
        }
    }

    private void test() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElementCounter.READ_PERIOD_MS = SENSOR_READ_PERIOD_MS;
        for (int i = 0; i < NS; i++) {
            samples.add(new ArrayList<>());
            stretch.add(new ArrayList<>());
        }
        counter = ElementCounter.tryCreate(hardwareMap);
        if (counter == null) {
            while (!isStopRequested()) {
                telemetry.addLine("The color sensors CS1, CS3 and CS4 are not all configured. Fix the hub configuration.");
                telemetry.update();
            }
            return;
        }
        cs5 = EntryCounter.tryCreate(hardwareMap);
        cs2 = LaunchSensor.tryCreate(hardwareMap);
        pusher = new Pusher(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);
        pusher.setAutoCorrect(false);   // nothing moves the arm except the sweeps

        while (!isStarted() && !isStopRequested()) {
            counter.update();
            pusher.update();
            if (cs5 != null) cs5.touch();
            if (cs2 != null) cs2.touch(false);
            telemetry.addLine("Pusher Calibration Test. The pusher is at HOLD now (its zero is assumed here), the track and the ramp are EMPTY. Keep the robot still until Start.");
            addSensorLines();
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.setSampler(columns(), SAMPLE_PERIOD_MS, this::row);
        RunLog.event(String.format(Locale.US, "SETUP sensor read period %.0f ms, log rows every %d ms, still %.0f s, sweep power %.2f for %.0f deg, CS2 %s, CS5 %s, pusher check parks at %.1f deg",
                ElementCounter.READ_PERIOD_MS, SAMPLE_PERIOD_MS, STILL_SEC, SWEEP_POWER, SWEEP_DEG, cs2 == null ? "not configured" : "configured", cs5 == null ? "not configured" : "configured",
                PusherCheck.ARM_CENTER_DEG));

        while (opModeIsActive()) {
            counter.update();
            pusher.update();
            if (cs5 != null) cs5.touch();
            if (cs2 != null) cs2.touch(recording && step.sweep);   // the CS2 thread reads fast while a sweep runs
            for (int i = 0; i < NS; i++) {
                if (!COLOR[i]) continue;
                int n = counter.readCount(i);
                if (n != lastReadSeen[i]) {
                    lastReadSeen[i] = n;
                    if (recording) freshColor(i);
                }
            }
            if (recording) {
                freshProx(CS2);
                freshProx(CS5);
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
            if (gamepad1.rightBumperWasPressed()) RunLog.event(String.format(Locale.US, "NOTE (take %d, %.1f s, pusher %.0f deg)", recording ? takeId : 0, recording ? takeClock.seconds() : 0.0, degNow()));
            if (gamepad1.aWasPressed()) {
                if (!recording) {
                    allDone = false;
                    startStep();
                } else {
                    endStep("stopped by the driver", false);
                }
            }

            if (recording && !step.sweep) {
                if (phase == Phase.RUN && takeClock.seconds() >= STILL_SEC) {
                    finishBaseline();
                    endStep("complete", true);
                }
            } else if (recording) {
                if (phase == Phase.LEAD_IN && phaseClock.seconds() >= LEAD_SEC) {
                    phase = Phase.SWEEP;
                    phaseClock.reset();
                    sweepStartDeg = degNow();
                    pusher.jog(step.dir * SWEEP_POWER);
                    RunLog.event(String.format(Locale.US, "TAKE %d SWEEP start: %s at %.2f power from %.0f deg", takeId, step.dir > 0 ? "forward" : "back", SWEEP_POWER, sweepStartDeg));
                } else if (phase == Phase.SWEEP) {
                    double deg = degNow();
                    boolean done = step.dir > 0 ? deg >= SWEEP_DEG : deg <= 0.0;
                    double moved = Math.abs(deg - sweepStartDeg);
                    if (done) {
                        pusher.stopInPlace();
                        RunLog.event(String.format(Locale.US, "TAKE %d SWEEP done: %.0f deg in %.1f s, now at %.0f deg", takeId, moved, phaseClock.seconds(), deg));
                        finishSweep();
                        phase = Phase.TAIL;
                        phaseClock.reset();
                    } else if (phaseClock.seconds() > SWEEP_NO_MOVE_SEC && moved < 5.0) {
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
    }

    private void showScreen() {
        telemetry.addData("Step", step.title + (allDone ? "   (all three steps done; D-pad left to repeat one)" : ""));
        if (recording) {
            String what = phase == Phase.RUN ? String.format(Locale.US, "recording, %.0f s of %.0f s", takeClock.seconds(), STILL_SEC)
                    : phase == Phase.LEAD_IN ? "still, the sweep starts soon" : phase == Phase.SWEEP ? "SWEEPING (A stops it)" : "still, finishing";
            telemetry.addData("Now", what);
        } else {
            telemetry.addData("Next", step.prompt);
        }
        if (!warning.isEmpty()) telemetry.addData("WARNING", warning);
        for (String v : verdicts) telemetry.addLine("VERDICT: " + v);
        if (!results.isEmpty()) {
            telemetry.addLine("Results of the step just run (also in the log):");
            for (String l : results) telemetry.addLine("  " + l);
        }
        telemetry.addData("Pusher", "%.0f deg from the baseline HOLD, %s", degNow(), pusher.getState());
        addSensorLines();
        telemetry.addLine("A = start / stop, D-pad left / right = previous / next step, right bumper = NOTE.");
        telemetry.update();
    }

    private void addSensorLines() {
        for (int i = 0; i < NS; i++) {
            if (COLOR[i]) {
                telemetry.addData(NAMES[i], "dist %.1f cm  alpha %.3f  rgb %.2f / %.2f / %.2f  shift %.3f  reads %d", counter.lastDistCm(i), counter.lastAlpha(i),
                        counter.lastShare(i, 0), counter.lastShare(i, 1), counter.lastShare(i, 2), counter.colorShift(i), counter.readCount(i));
            } else {
                telemetry.addData(NAMES[i], present(i) ? "proximity %d%s" : "%d (not answering)", prox(i), haveBase[i] ? String.format(Locale.US, "  (empty %.0f)", baseProx[i]) : "");
            }
        }
    }
}
