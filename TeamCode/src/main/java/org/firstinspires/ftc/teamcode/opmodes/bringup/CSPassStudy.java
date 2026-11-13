package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * CS pass study (2026-11-10 as the CS4 Pass Study; CS5 added 2026-11-12): can the color sensors at the transport classify and count the elements that
 * come in, at match loop times? CS4 is the one nearest the flywheel; CS5 (a second REV Color Sensor V3 named "CS5", added 2026-11-12) is nearer the INTAKE, so it is
 * the first sensor an element reaches. Both are read as PRIMARY sensors (the same fast preset, the same decision rules, each with its own empty baseline, its own passes
 * and tallies) and everything is logged for both. Without CS5 in the configuration the opmode runs on CS4 alone.
 *
 * The robot runs like a match: the real mechanisms (intake IN the whole time, the flywheel spinning at a slow SETPOINT, the pusher, the element counter on
 * CS1 to CS4 for fullness and the pusher check), the Pinpoint, the drive, the Dashboard and DS telemetry, and the run log, padded to LOOP_TARGET_MS per loop
 * (the TeleOp's 28 to 30 ms; 0 runs flat out). On top of that the opmode reads CS5 and CS4 ITSELF, once per loop each (CSn_EVERY_N_LOOPS to read less often; color, and
 * the distance when READ_DISTANCE), times each read, and decides from the readings alone whether something is in front of a sensor, what it is, and when it has gone.
 * Each "pass" (present, then gone) at a sensor is one element, logged as a CS4 PASS or CS5 PASS line with how many reads it got, its duration, the peak alpha, the colors
 * it voted, the pusher state, its start and end on the study clock, and the other sensor's latest pass (open or last finished) with its times, so the two can be paired
 * offline (the delay from a CS5 pass to the CS4 pass of the same element is how fast it travels).
 * There is NO ground truth: the passes are compared with each other, with the element counter's count and with the salvos (the "N held" of each SALVO started line),
 * and with what you fed in (A drops a MARK line in the log whenever you want to note something).
 *
 * Feed pollen (by hand or from the floor with the intake IN). The track is cleared by salvos at the slow SETPOINT (500 ticks/s: the elements just fall out
 * of the front): AUTO_CLEAR fires one when the element counter reads FULL (CLEAR_AT_COUNT), or press Y. The counter's own CS4 reads share CS4 and its fast rate, so its
 * CS4 slot may misbehave; fire by hand (Y) when it does, and read the log. CS5 is not in the element counter.
 *
 * Every loop's row has both readings next to the usual mechanism columns, so any classifier idea can be replayed over the log offline: study_t (the study clock, seconds
 * since Start), and per sensor (cs4_ and cs5_): age_ms (how long before the row was written the read finished: the row's t minus this is the read time), raw_r/g/b (the
 * SDK's channel values, 0 to 1, six decimals, red x1.07 and blue x1.55 already applied by the SDK), alpha, r/g/b (the color shares), prox (the raw proximity count) and dist
 * (cm, from the SDK's fit), read_ms, fresh (1 when the color value is new since that sensor's previous read; 0 on a loop the sensor was not read), the live decision
 * (present, class, pass) and then the mechanism columns (pusher state and angle, count and slots, flywheel, intake amps).
 *
 * INIT: keep the track EMPTY and the pusher arm at HOLD while the Pinpoint calibrates. The opmode measures each sensor's empty view with the rate preset it uses (the
 * readings scale with the sensor's integration time, so the production baselines do not apply) from the last BASELINE_READS INIT reads when Start is pressed
 * (A in INIT measures them right away and again). The baseline lines are logged.
 *
 * Gamepad 1:  left stick / right stick X  drive / turn (robot oriented, RB precision)     Left trigger (hold): outtake (clear a jam)
 *             Y  fire a salvo now      X  AUTO_CLEAR on / off      A  MARK (a note in the log)      B  clear the pass tallies
 *             D-pad up / down  flywheel setpoint +/- 20 ticks/s     Back  clear a pusher fault
 */
@Config
@TeleOp(name = "CS Pass Study", group = "Bring-up")
public class CSPassStudy extends LinearOpMode {

    /** The flywheel's slow speed while the elements come in (ticks/s). */
    public static double SETPOINT = 500.0;
    /**
     * The flywheel's hold power (Flywheel.FEED_FORWARD, 0.35) is used whenever the wheel is ABOVE its setpoint, and it never brakes: at 0.35 the wheel settles near 800 ticks/s, so
     * the slow SETPOINT (500) was never inside the launch band (480 to 520) and no salvo could start (the 2026-11-12 runs: "LAUNCH held ... ABOVE the band", dropped after 3 s). The study
     * therefore runs the hold power at this fraction of the power that holds the setpoint (setpoint / OPEN_LOOP_TOP_SPEED, 500 / 2200 = 0.23), never above the Flywheel's own value, so
     * the wheel decays below the setpoint and the full-power kick brings it back: it hovers around the setpoint. The Flywheel's own value is put back when the opmode ends.
     */
    public static double STUDY_FEED_FORWARD_FRACTION = 0.90;
    /** Loop time to emulate, ms: the loop is padded up to this (the TeleOp runs 27 to 30 ms). 0 = no padding, as fast as it goes. */
    public static double LOOP_TARGET_MS = 28.0;
    /** The sensors' fast preset: color 25 ms 16-bit, proximity 6.25 ms (set by reflection; the SDK default is 100 ms for both). Applied at INIT. */
    public static boolean FAST_RATE = true;
    /** Read the distance too (a second blocking I2C transaction). The arm and the elements differ most in distance, but note the fast rate leaves it uncalibrated. */
    public static boolean READ_DISTANCE = true;
    /** Read CS4 / CS5 on every Nth loop (1 = every loop). Reading both every loop is the study; 2 shows what alternating does to the loop time. */
    public static int CS4_EVERY_N_LOOPS = 1;
    public static int CS5_EVERY_N_LOOPS = 1;
    /** Fire a salvo by itself when the element counter reports CLEAR_AT_COUNT or more. */
    public static boolean AUTO_CLEAR = true;
    public static int CLEAR_AT_COUNT = 4;
    /** Seconds to wait after a salvo before AUTO_CLEAR may fire again (the pusher check runs then). */
    public static double CLEAR_PAUSE_SEC = 2.0;
    /** Reads in a row that must agree before presence turns on / off. */
    public static int ON_READS = 2;
    public static int OFF_READS = 2;
    /** INIT reads averaged into the empty baseline. */
    public static int BASELINE_READS = 40;
    public static double STEP = 20.0;
    public static double TRIGGER_THRESHOLD = 0.3;
    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;

    private String rateNote = "";
    private final ElapsedTime sinceStart = new ElapsedTime();
    private double origFeedForward = Flywheel.FEED_FORWARD;

    /** The hold power that lets the wheel come down to the (slow) setpoint: a fraction of the power that holds it, never more than the Flywheel's own. */
    private void applyStudyFeedForward() {
        double hold = STUDY_FEED_FORWARD_FRACTION * SETPOINT / Flywheel.OPEN_LOOP_TOP_SPEED;
        Flywheel.FEED_FORWARD = Math.max(0.0, Math.min(origFeedForward, hold));
    }

    private static String applyFastRate(RevColorSensorV3 sensor) {
        try {
            Method ps = BroadcomColorSensorImpl.class.getDeclaredMethod("setPSRateAndRes",
                    BroadcomColorSensor.PSResolution.class, BroadcomColorSensor.PSMeasurementRate.class);
            ps.setAccessible(true);
            ps.invoke(sensor, BroadcomColorSensor.PSResolution.R8BIT, BroadcomColorSensor.PSMeasurementRate.R6_25ms);
            Method ls = BroadcomColorSensorImpl.class.getDeclaredMethod("setLSRateAndRes",
                    BroadcomColorSensor.LSResolution.class, BroadcomColorSensor.LSMeasurementRate.class);
            ls.setAccessible(true);
            ls.invoke(sensor, BroadcomColorSensor.LSResolution.R16BIT, BroadcomColorSensor.LSMeasurementRate.R25ms);
            return "fast rate applied (color 25 ms 16-bit, proximity 6.25 ms)";
        } catch (Exception e) {
            return "FAST RATE FAILED (" + e + ")";
        }
    }

    private static double median(double[] v, int n) {
        double[] s = Arrays.copyOf(v, n);
        Arrays.sort(s);
        return n == 0 ? Double.NaN : (n % 2 == 1 ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]));
    }

    /** One color sensor read as a primary sensor: its reading, empty baseline, presence decision, pass bookkeeping and tallies. */
    private final class Probe {
        final String name;            // CS4 or CS5
        final String prefix;          // cs4_ or cs5_
        final RevColorSensorV3 sensor;
        /** The sensor number the classifier's rules are given: CS4 is the arm's sensor (it can read the arm), the others cannot. */
        final int ruleSensor;

        /** The SDK's distance fit (a protected method), reached by reflection so the raw count is read ONCE and the distance still comes from it. */
        Method inFromOptical;
        boolean readThisLoop;
        long lastReadNs;

        // the last reading, as logged
        double alpha, dist = Double.NaN, shareR, shareG, shareB, readMs, rawR, rawG, rawB;
        int proxRaw = -1;
        boolean fresh, present;
        ElementCounter.Slot slotNow = ElementCounter.Slot.EMPTY;
        int passId;
        double prevAlpha = -1, prevR = -1;
        int reads, freshReads;
        double readMsSum, readMsMax;

        // empty baseline
        double baseAlpha = 0.05, baseDist = 7.7;
        double[] baseShares = {0.24, 0.41, 0.35};
        boolean baselineSet = false;
        final double[][] initReads = new double[512][5];
        int initCount = 0;

        // detection state
        boolean candidate;
        int candidateRuns;

        // the pass being collected
        boolean inPass;
        double passStartT, passPeakAlpha, passMinDist;
        int passReads;
        final int[] votes = new int[ElementCounter.Slot.values().length];
        String passPusherState = "";

        // the last pass that finished (for the other sensor's lines)
        int lastDoneId;
        double lastDoneStartT, lastDoneEndT;
        ElementCounter.Slot lastDoneClass = ElementCounter.Slot.EMPTY;

        // tallies
        int passesTotal, passesDuringIntake, passesDuringSalvo;
        final int[] classTally = new int[ElementCounter.Slot.values().length];
        final List<Double> passDurations = new ArrayList<>();
        final List<Integer> passReadCounts = new ArrayList<>();
        String lastPass = "none yet";

        Probe(String name, RevColorSensorV3 sensor, int ruleSensor) {
            this.name = name;
            this.prefix = name.toLowerCase(Locale.US) + "_";
            this.sensor = sensor;
            this.ruleSensor = ruleSensor;
        }

        /**
         * One read: the color (a status read, plus the 9 data bytes only when the sensor has a NEW value; otherwise the SDK hands back the old one), the raw proximity
         * count when wanted (one more transaction) and its distance from the SDK's own fit, the read time, and whether the value is new.
         */
        void read() {
            long t0 = System.nanoTime();
            NormalizedRGBA c = sensor.getNormalizedColors();
            double d = Double.NaN;
            int raw = -1;
            if (READ_DISTANCE) {
                if (inFromOptical == null) {
                    try {
                        inFromOptical = RevColorSensorV3.class.getDeclaredMethod("inFromOptical", int.class);
                        inFromOptical.setAccessible(true);
                    } catch (Exception e) {
                        inFromOptical = null;
                    }
                }
                if (inFromOptical != null) {
                    raw = sensor.rawOptical();
                    try {
                        d = DistanceUnit.CM.fromUnit(DistanceUnit.INCH, (Double) inFromOptical.invoke(sensor, raw));
                    } catch (Exception e) {
                        d = Double.NaN;
                    }
                } else {
                    d = sensor.getDistance(DistanceUnit.CM);
                }
            }
            long t1 = System.nanoTime();
            rawR = c.red;
            rawG = c.green;
            rawB = c.blue;
            proxRaw = raw;
            double sum = c.red + c.green + c.blue + 1e-9;
            alpha = c.alpha;
            shareR = c.red / sum;
            shareG = c.green / sum;
            shareB = c.blue / sum;
            dist = d;
            readMs = (t1 - t0) / 1e6;
            fresh = alpha != prevAlpha || shareR != prevR;   // the color value is new (the proximity register is read every time)
            prevAlpha = alpha;
            prevR = shareR;
            reads++;
            if (fresh) freshReads++;
            readMsSum += readMs;
            readMsMax = Math.max(readMsMax, readMs);
            lastReadNs = t1;
            readThisLoop = true;
        }

        /** A loop this sensor was not read in: the row repeats its last values, flagged not fresh. */
        void skipped() {
            readThisLoop = false;
            fresh = false;
            readMs = 0.0;
        }

        void rememberInitRead() {
            double[] r = initReads[initCount % initReads.length];
            r[0] = alpha;
            r[1] = Double.isNaN(dist) ? 0.0 : dist;
            r[2] = shareR;
            r[3] = shareG;
            r[4] = shareB;
            initCount++;
        }

        /** The empty baseline from the last BASELINE_READS INIT reads. */
        void captureBaseline() {
            int n = Math.min(Math.min(initCount, BASELINE_READS), initReads.length);
            if (n < 5) return;
            double[] a = new double[n], d = new double[n], r = new double[n], g = new double[n], b = new double[n];
            for (int i = 0; i < n; i++) {
                double[] row = initReads[((initCount - 1 - i) % initReads.length + initReads.length) % initReads.length];
                a[i] = row[0];
                d[i] = row[1];
                r[i] = row[2];
                g[i] = row[3];
                b[i] = row[4];
            }
            baseAlpha = median(a, n);
            baseDist = READ_DISTANCE ? median(d, n) : Double.NaN;
            baseShares = new double[]{median(r, n), median(g, n), median(b, n)};
            baselineSet = true;
            RunLog.event(String.format(Locale.US, "%s BASELINE (empty, %d reads, %s): alpha %.4f, dist %.2f cm, shares %.3f / %.3f / %.3f", name, n, rateNote, baseAlpha,
                    baseDist, baseShares[0], baseShares[1], baseShares[2]));
        }

        /** Presence, class and the pass bookkeeping from the reading just taken. */
        void classifyAndTrack(double nowT, String pusherState, Probe other) {
            boolean raw = ElementCounter.presenceNext(present, shareR, shareG, shareB, alpha, Double.isNaN(dist) ? Double.NaN : dist,
                    baseShares[0], baseShares[1], baseShares[2], baseAlpha, Double.isNaN(baseDist) ? 99.0 : baseDist);
            if (raw == candidate) candidateRuns++;
            else {
                candidate = raw;
                candidateRuns = 1;
            }
            boolean wasPresent = present;
            if (candidate != present && candidateRuns >= (candidate ? ON_READS : OFF_READS)) present = candidate;
            slotNow = present ? ElementCounter.classify(ruleSensor, shareR, shareG, shareB, Double.isNaN(dist) ? 99.0 : dist) : ElementCounter.Slot.EMPTY;

            if (present && !wasPresent) {
                inPass = true;
                passStartT = nowT;
                passPeakAlpha = alpha;
                passMinDist = Double.isNaN(dist) ? Double.NaN : dist;
                passReads = 0;
                Arrays.fill(votes, 0);
                passPusherState = pusherState;
                passId++;
            }
            if (inPass && present) {
                passReads++;
                votes[slotNow.ordinal()]++;
                passPeakAlpha = Math.max(passPeakAlpha, alpha);
                if (!Double.isNaN(dist)) passMinDist = Double.isNaN(passMinDist) ? dist : Math.min(passMinDist, dist);
            }
            if (inPass && !present) finishPass(nowT, pusherState, other);
        }

        private void finishPass(double nowT, String pusherState, Probe other) {
            inPass = false;
            int best = 0;
            for (int i = 1; i < votes.length; i++) if (votes[i] > votes[best]) best = i;
            ElementCounter.Slot cls = ElementCounter.Slot.values()[best];
            double dur = nowT - passStartT;
            passesTotal++;
            classTally[best]++;
            boolean arrival = "HOLD".equals(passPusherState) && "HOLD".equals(pusherState);
            if (arrival) passesDuringIntake++;
            else passesDuringSalvo++;
            passDurations.add(dur);
            passReadCounts.add(passReads);
            StringBuilder v = new StringBuilder();
            for (int i = 0; i < votes.length; i++) {
                if (votes[i] > 0) v.append(ElementCounter.Slot.values()[i]).append(' ').append(votes[i]).append(' ');
            }
            lastPass = String.format(Locale.US, "#%d %s, %d reads, %.0f ms", passId, cls, passReads, dur * 1000.0);
            lastDoneId = passId;
            lastDoneStartT = passStartT;
            lastDoneEndT = nowT;
            lastDoneClass = cls;
            RunLog.event(String.format(Locale.US, "%s PASS #%d: %s, %d reads over %.0f ms (%.1f ms per read), peak alpha %.3f, nearest %.1f cm, votes: %spusher %s -> %s (%s); study clock %.3f to %.3f s; %s",
                    name, passId, cls, passReads, dur * 1000.0, passReads > 0 ? dur * 1000.0 / passReads : 0.0, passPeakAlpha, passMinDist, v, passPusherState, pusherState,
                    arrival ? "arrival, pusher at HOLD" : "pusher moving", passStartT, nowT, other == null ? "no other sensor" : other.latestPassText(nowT)));
        }

        /** This sensor's pass that is open now, else its last finished one, for the other sensor's PASS lines. */
        String latestPassText(double nowT) {
            if (inPass) return String.format(Locale.US, "%s pass #%d OPEN since %.3f s (%s so far)", name, passId, passStartT, bestVote());
            if (lastDoneId > 0) return String.format(Locale.US, "%s last pass #%d %s, %.3f to %.3f s", name, lastDoneId, lastDoneClass, lastDoneStartT, lastDoneEndT);
            return name + " no pass yet";
        }

        private ElementCounter.Slot bestVote() {
            int best = 0;
            for (int i = 1; i < votes.length; i++) if (votes[i] > votes[best]) best = i;
            return ElementCounter.Slot.values()[best];
        }

        void clearTallies() {
            passesTotal = passesDuringIntake = passesDuringSalvo = 0;
            Arrays.fill(classTally, 0);
            passDurations.clear();
            passReadCounts.clear();
        }

        String columns() {
            return prefix + "age_ms," + prefix + "raw_r," + prefix + "raw_g," + prefix + "raw_b," + prefix + "alpha," + prefix + "r," + prefix + "g," + prefix + "b," + prefix + "prox,"
                    + prefix + "dist," + prefix + "read_ms," + prefix + "fresh," + prefix + "present," + prefix + "class," + prefix + "pass";
        }

        String row() {
            return String.format(Locale.US, "%.1f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%d,%s,%.2f,%d,%d,%s,%d",
                    (System.nanoTime() - lastReadNs) / 1e6, rawR, rawG, rawB, alpha, shareR, shareG, shareB, proxRaw,
                    Double.isNaN(dist) ? "" : String.format(Locale.US, "%.2f", dist), readMs, fresh ? 1 : 0, present ? 1 : 0, slotNow, inPass ? passId : 0);
        }

        double meanReads() {
            if (passReadCounts.isEmpty()) return 0.0;
            double s = 0;
            for (int n : passReadCounts) s += n;
            return s / passReadCounts.size();
        }

        double meanPassMs() {
            if (passDurations.isEmpty()) return 0.0;
            double s = 0;
            for (double d : passDurations) s += d * 1000.0;
            return s / passDurations.size();
        }

        String classTallyText() {
            StringBuilder cls = new StringBuilder();
            for (ElementCounter.Slot s : ElementCounter.Slot.values()) if (classTally[s.ordinal()] > 0) cls.append(s).append(' ').append(classTally[s.ordinal()]).append("  ");
            return cls.length() == 0 ? "-" : cls.toString();
        }
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "CS Pass Study", hardwareMap);
        origFeedForward = Flywheel.FEED_FORWARD;
        try {
            runStudy();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            Flywheel.FEED_FORWARD = origFeedForward;   // a static: left changed it would stay changed for the match TeleOp until the app restarts
            RunLog.close();
        }
    }

    private void runStudy() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);
        mechanisms.setUseLaunchMap(false);
        mechanisms.setAutoStopAfterSalvo(false);
        mechanisms.setBlueAlliance(true);
        mechanisms.setOpenCell(HiveTarget.BLUE_REAR);
        mechanisms.setIntakeAlwaysOn(true);

        // the probes in reading order: CS5 first (nearest the intake: the first sensor an element reaches), then CS4
        List<Probe> probes = new ArrayList<>();
        RevColorSensorV3 s5 = hardwareMap.tryGet(RevColorSensorV3.class, "CS5");
        RevColorSensorV3 s4 = hardwareMap.tryGet(RevColorSensorV3.class, "CS4");
        Probe cs5 = null, cs4 = null;
        if (s5 != null) {
            cs5 = new Probe("CS5", s5, 0);
            probes.add(cs5);
        }
        if (s4 != null) {
            cs4 = new Probe("CS4", s4, ElementCounter.ARM_SENSOR);
            probes.add(cs4);
        }
        if (probes.isEmpty()) {
            telemetry.addLine("Neither CS4 nor CS5 is a REV Color Sensor V3 in the configuration: nothing to study");
            telemetry.update();
            waitForStart();
            return;
        }
        rateNote = FAST_RATE ? "fast rate" : "default rate (100 ms)";
        StringBuilder rates = new StringBuilder();
        for (Probe p : probes) {
            String note = FAST_RATE ? applyFastRate(p.sensor) : "default rate (100 ms)";
            rates.append(p.name).append(": ").append(note).append("; ");
        }
        String rateText = rates.toString();
        telemetry.addData("Sensor rates", rateText);
        if (cs5 == null) telemetry.addLine("CS5 is not in the configuration: running on CS4 alone");
        telemetry.addData("Mechanisms present", mechanisms.describe());

        // INIT: read the sensors every loop and keep the reads for the empty baselines
        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();
            for (Probe p : probes) {
                p.read();
                p.rememberInitRead();
            }
            if (gamepad1.aWasPressed()) for (Probe p : probes) p.captureBaseline();
            telemetry.addData("Sensor rates", rateText);
            for (Probe p : probes) {
                telemetry.addData(p.name + " now", "alpha %.4f  dist %s  shares %.3f / %.3f / %.3f  (%.1f ms per read)", p.alpha,
                        Double.isNaN(p.dist) ? "-" : String.format(Locale.US, "%.2f cm", p.dist), p.shareR, p.shareG, p.shareB, p.readMs);
                telemetry.addData(p.name + " empty baseline", p.baselineSet ? String.format(Locale.US, "alpha %.4f dist %.2f", p.baseAlpha, p.baseDist) : "not taken yet (taken automatically at Start)");
            }
            telemetry.addLine("Keep the track EMPTY and the arm at HOLD; keep still while the Pinpoint calibrates. A = take the baselines now. Then Start.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        for (Probe p : probes) p.captureBaseline();
        applyStudyFeedForward();
        mechanisms.setManualSetpoint(SETPOINT);
        mechanisms.setSpinRequested(true);
        RunLog.started();
        RunLog.event("SETUP CS pass study: sensors " + (cs5 != null ? "CS5 (nearest the intake) + " : "") + "CS4, " + rateText + "READ_DISTANCE " + READ_DISTANCE + ", read every "
                + CS5_EVERY_N_LOOPS + " (CS5) / " + CS4_EVERY_N_LOOPS + " (CS4) loops, loop target " + LOOP_TARGET_MS + " ms, setpoint " + SETPOINT
                + ", flywheel hold power " + String.format(Locale.US, "%.3f", Math.max(0.0, Math.min(origFeedForward, STUDY_FEED_FORWARD_FRACTION * SETPOINT / Flywheel.OPEN_LOOP_TOP_SPEED))) + " (was " + origFeedForward + ")"
                + ", auto clear " + AUTO_CLEAR + " at " + CLEAR_AT_COUNT + ", presence " + ON_READS + "/" + OFF_READS + " reads, " + mechanisms.describe());
        StringBuilder cols = new StringBuilder("study_t,");
        for (Probe p : probes) cols.append(p.columns()).append(',');
        cols.append("auto_clear,").append(Mechanisms.SAMPLE_COLUMNS);
        RunLog.setSampler(cols.toString(), 1, () -> {
            StringBuilder sb = new StringBuilder(String.format(Locale.US, "%.3f,", sinceStart.seconds()));
            for (Probe p : probes) sb.append(p.row()).append(',');
            sb.append(AUTO_CLEAR ? 1 : 0).append(',').append(mechanisms.sampleRow());
            return sb.toString();
        });

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime sinceSalvo = new ElapsedTime();
        int marks = 0;
        double loopSum = 0, loopMax = 0;
        long loops = 0;
        int lastSalvos = 0;
        sinceStart.reset();

        while (opModeIsActive()) {
            loopTimer.reset();
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();

            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale),
                    -gamepad1.right_stick_x * TURN_SCALE * scale));

            if (gamepad1.backWasPressed()) mechanisms.clearPusherStall();
            if (gamepad1.xWasPressed()) {
                AUTO_CLEAR = !AUTO_CLEAR;
                RunLog.event("AUTO CLEAR " + (AUTO_CLEAR ? "on" : "off"));
            }
            if (gamepad1.bWasPressed()) {
                for (Probe p : probes) p.clearTallies();
                RunLog.event("TALLIES cleared");
            }
            if (gamepad1.aWasPressed()) {
                marks++;
                StringBuilder counts = new StringBuilder();
                for (Probe p : probes) counts.append(p.name).append(" passes so far ").append(p.passesTotal).append(", ");
                RunLog.event(String.format(Locale.US, "MARK %d: %scounter %d, salvos %d", marks, counts,
                        mechanisms.counter != null ? mechanisms.counter.count() : -1, mechanisms.getSalvosDone()));
            }
            if (gamepad1.dpadUpWasPressed()) SETPOINT += STEP;
            if (gamepad1.dpadDownWasPressed()) SETPOINT = Math.max(0.0, SETPOINT - STEP);
            mechanisms.setManualOuttake(gamepad1.left_trigger > TRIGGER_THRESHOLD);
            if (gamepad1.yWasPressed()) {
                RunLog.event("SALVO requested by hand");
                mechanisms.requestLaunch();
                sinceSalvo.reset();
            }
            if (AUTO_CLEAR && mechanisms.counter != null && mechanisms.counter.count() >= CLEAR_AT_COUNT && sinceSalvo.seconds() > CLEAR_PAUSE_SEC
                    && !mechanisms.isLaunchBusy()) {
                RunLog.event("AUTO CLEAR: the counter reads " + mechanisms.counter.count() + ", firing a salvo");
                mechanisms.requestLaunch();
                sinceSalvo.reset();
            }
            if (mechanisms.getSalvosDone() != lastSalvos) {
                lastSalvos = mechanisms.getSalvosDone();
                sinceSalvo.reset();
            }

            // the reads of this loop, after the drive and the controls, before the mechanisms (as a counter read would be): CS5 first, then CS4
            String pusherState = mechanisms.pusher == null ? "" : String.valueOf(mechanisms.pusher.getState());
            for (Probe p : probes) {
                int every = Math.max(1, p == cs5 ? CS5_EVERY_N_LOOPS : CS4_EVERY_N_LOOPS);
                if (loops % every == 0) {
                    p.read();
                    p.classifyAndTrack(sinceStart.seconds(), pusherState, p == cs5 ? cs4 : cs5);
                } else {
                    p.skipped();
                }
            }

            applyStudyFeedForward();   // follows the D-pad speed changes
            mechanisms.setManualSetpoint(SETPOINT);
            mechanisms.setSpinRequested(true);
            mechanisms.update(pose);

            for (Probe p : probes) {
                telemetry.addData(p.name + " now", "%s  alpha %.3f  dist %s  shares %.2f/%.2f/%.2f", p.present ? p.slotNow : "empty", p.alpha,
                        Double.isNaN(p.dist) ? "-" : String.format(Locale.US, "%.1f", p.dist), p.shareR, p.shareG, p.shareB);
                telemetry.addData(p.name + " passes", "%d total (%d at HOLD, %d pusher moving)   %s", p.passesTotal, p.passesDuringIntake, p.passesDuringSalvo, p.classTallyText());
                telemetry.addData(p.name + " last pass", "%s   mean %.1f reads, %.0f ms", p.lastPass, p.meanReads(), p.meanPassMs());
                telemetry.addData(p.name + " reads", "%.1f ms each (max %.0f), %.0f%% fresh", p.reads > 0 ? p.readMsSum / p.reads : 0.0, p.readMsMax,
                        p.reads > 0 ? 100.0 * p.freshReads / p.reads : 0.0);
            }
            telemetry.addData("Loop", "%.1f ms mean, %.0f ms max (target %.0f)", loops > 0 ? loopSum / loops : 0.0, loopMax, LOOP_TARGET_MS);
            telemetry.addData("Counter", "%d held, salvos %d, setpoint %.0f", mechanisms.counter != null ? mechanisms.counter.count() : -1,
                    mechanisms.getSalvosDone(), SETPOINT);
            telemetry.addData("Auto clear", "%s at %d", AUTO_CLEAR ? "ON" : "off", CLEAR_AT_COUNT);
            mechanisms.log(telemetry);
            telemetry.addLine("Y salvo, X auto clear, A mark, B clear tallies, D-pad up/down speed, LT outtake, Back clear fault");
            telemetry.update();

            // pad the loop to the emulated match loop time
            while (LOOP_TARGET_MS > 0 && loopTimer.milliseconds() < LOOP_TARGET_MS && opModeIsActive()) sleep(1);
            double lastLoopTime = loopTimer.milliseconds();
            loopSum += lastLoopTime;
            loopMax = Math.max(loopMax, lastLoopTime);
            loops++;
            RunLog.tick();
        }
        for (Probe p : probes) {
            RunLog.event(String.format(Locale.US, "%s PASS SUMMARY: %d passes (%d at HOLD, %d with the pusher moving), classes %s, mean %.1f reads and %.0f ms per pass; "
                            + "%d reads, %.1f ms each, %.0f%% fresh",
                    p.name, p.passesTotal, p.passesDuringIntake, p.passesDuringSalvo, Arrays.toString(p.classTally), p.meanReads(), p.meanPassMs(),
                    p.reads, p.reads > 0 ? p.readMsSum / p.reads : 0.0, p.reads > 0 ? 100.0 * p.freshReads / p.reads : 0.0));
        }
        RunLog.event(String.format(Locale.US, "CS PASS SUMMARY: loop %.1f ms mean, %.0f ms max over %d loops; counter %d, salvos %d", loops > 0 ? loopSum / loops : 0.0, loopMax, loops,
                mechanisms.counter != null ? mechanisms.counter.count() : -1, mechanisms.getSalvosDone()));
        drive.stop();
        mechanisms.setIntakeAlwaysOn(false);
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
