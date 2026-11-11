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
 * CS4 pass study (2026-11-10): can CS4 alone classify and count the elements that come into the rotary transport, at match loop times?
 *
 * The robot runs like a match: the real mechanisms (intake IN the whole time, the flywheel spinning at a slow SETPOINT, the pusher, the element counter on
 * CS1 to CS4 for fullness and the pusher check), the Pinpoint, the drive, the Dashboard and DS telemetry, and the run log, padded to LOOP_TARGET_MS per loop
 * (the TeleOp's 28 to 30 ms; 0 runs flat out). On top of that the opmode reads CS4 ITSELF, once per loop (color, and the distance when READ_DISTANCE), times the
 * read, and decides from the reading alone whether something is in front of the sensor, what it is, and when it has gone. Each "pass" (present, then gone) is
 * one element, logged as a CS4 PASS line with how many reads it got, its duration, the peak alpha, the colors it voted and the pusher state when it happened.
 * There is NO ground truth: the passes are compared with the element counter's count and with the salvos (the "N held" of each SALVO started line), and with
 * what you fed in (A drops a MARK line in the log whenever you want to note something).
 *
 * Feed pollen (by hand or from the floor with the intake IN). The track is cleared by salvos at the slow SETPOINT (500 ticks/s: the elements just fall out
 * of the front): AUTO_CLEAR fires one when the element counter reads FULL (CLEAR_AT_COUNT), or press Y. The counter's own CS4 reads share the sensor and the
 * fast rate, so its CS4 slot may misbehave; fire by hand (Y) when it does, and read the log.
 *
 * Every loop's row has the CS4 reading next to the usual mechanism columns, so any classifier idea can be replayed over the log offline:
 * cs4_age_ms (how long before the row was written the read finished: the row's t minus this is the read time), cs4_raw_r/g/b (the SDK's channel values, 0 to 1, six
 * decimals, red x1.07 and blue x1.55 already applied by the SDK), cs4_alpha, cs4_r/g/b (the color shares), cs4_prox (the raw proximity count) and cs4_dist (cm,
 * from the SDK's fit), cs4_read_ms, cs4_fresh (1 when the color value is new since the previous read), the live decision (cs4_present, cs4_class, cs4_pass) and the
 * mechanism columns (pusher state and angle, count and slots, flywheel, intake amps).
 *
 * INIT: keep the track EMPTY and the pusher arm at HOLD while the Pinpoint calibrates. The opmode measures CS4's empty view with the rate preset it uses (the
 * readings scale with the sensor's integration time, so the production baselines do not apply) from the last BASELINE_READS INIT reads when Start is pressed
 * (A in INIT measures it right away and again). The baseline line is logged.
 *
 * Gamepad 1:  left stick / right stick X  drive / turn (robot oriented, RB precision)     Left trigger (hold): outtake (clear a jam)
 *             Y  fire a salvo now      X  AUTO_CLEAR on / off      A  MARK (a note in the log)      B  clear the pass tallies
 *             D-pad up / down  flywheel setpoint +/- 20 ticks/s     Back  clear a pusher fault
 */
@Config
@TeleOp(name = "CS4 Pass Study", group = "Bring-up")
public class CS4PassStudy extends LinearOpMode {

    /** The flywheel's slow speed while the elements come in (ticks/s). */
    public static double SETPOINT = 500.0;
    /** Loop time to emulate, ms: the loop is padded up to this (the TeleOp runs 27 to 30 ms). 0 = no padding, as fast as it goes. */
    public static double LOOP_TARGET_MS = 28.0;
    /** The sensor's fast preset: color 25 ms 16-bit, proximity 6.25 ms (set by reflection; the SDK default is 100 ms for both). Applied at INIT. */
    public static boolean FAST_RATE = true;
    /** Read the distance too (a second blocking I2C transaction). The arm and the elements differ most in distance, but note the fast rate leaves it uncalibrated. */
    public static boolean READ_DISTANCE = true;
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

    private static final int SENSOR = ElementCounter.ARM_SENSOR;   // CS4

    private String rateNote = "";
    private long lastReadNs = 0L;

    // the last CS4 reading, as logged
    private double alpha, dist = Double.NaN, shareR, shareG, shareB, readMs;
    /** The raw channel values the SDK gives (red x1.07 and blue x1.55 already applied; 0 to 1 = 0 to 65535) and the raw proximity count, for offline work. */
    private double rawR, rawG, rawB;
    private int proxRaw = -1;
    /** The SDK's distance fit (a protected method), reached by reflection so the raw count is read ONCE and the distance still comes from it. */
    private Method inFromOptical;
    private boolean fresh, present;
    private ElementCounter.Slot slotNow = ElementCounter.Slot.EMPTY;
    private int passId;
    private double prevAlpha = -1, prevR = -1, prevDist = -1;

    // empty baseline
    private double baseAlpha = 0.05, baseDist = 7.7;
    private double[] baseShares = {0.24, 0.41, 0.35};
    private boolean baselineSet = false;

    // detection state
    private boolean candidate;
    private int candidateRuns;
    private int reads, freshReads;
    private double readMsSum, readMsMax;
    private final ElapsedTime sinceStart = new ElapsedTime();

    // the pass being collected
    private boolean inPass;
    private double passStartT, passPeakAlpha, passMinDist;
    private int passReads;
    private final int[] votes = new int[ElementCounter.Slot.values().length];
    private String passPusherState = "";

    // tallies
    private int passesTotal, passesDuringIntake, passesDuringSalvo;
    private final int[] classTally = new int[ElementCounter.Slot.values().length];
    private final List<Double> passDurations = new ArrayList<>();
    private final List<Integer> passReadCounts = new ArrayList<>();
    private String lastPass = "none yet";

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

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "CS4 Pass Study", hardwareMap);
        try {
            runStudy();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    /**
     * One CS4 read: the color (a status read, plus the 9 data bytes only when the sensor has a NEW value; otherwise the SDK hands back the old one),
     * the raw proximity count when wanted (one more transaction) and its distance from the SDK's own fit, the read time, and whether the value is new.
     */
    private void readCs4(RevColorSensorV3 sensor) {
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
        prevDist = d;
        reads++;
        if (fresh) freshReads++;
        readMsSum += readMs;
        readMsMax = Math.max(readMsMax, readMs);
        lastReadNs = t1;
    }

    private final double[][] initReads = new double[512][6];
    private int initCount = 0;

    private void rememberInitRead() {
        double[] r = initReads[initCount % initReads.length];
        r[0] = alpha;
        r[1] = Double.isNaN(dist) ? 0.0 : dist;
        r[2] = shareR;
        r[3] = shareG;
        r[4] = shareB;
        initCount++;
    }

    private static double median(double[] v, int n) {
        double[] s = Arrays.copyOf(v, n);
        Arrays.sort(s);
        return n == 0 ? Double.NaN : (n % 2 == 1 ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]));
    }

    /** The empty baseline from the last BASELINE_READS INIT reads. */
    private void captureBaseline() {
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
        RunLog.event(String.format(Locale.US, "CS4 BASELINE (empty, %d reads, %s): alpha %.4f, dist %.2f cm, shares %.3f / %.3f / %.3f", n, rateNote, baseAlpha,
                baseDist, baseShares[0], baseShares[1], baseShares[2]));
    }

    /** Presence, class and the pass bookkeeping from the reading just taken. */
    private void classifyAndTrack(double nowT, String pusherState) {
        boolean raw = ElementCounter.presenceNext(present, shareR, shareG, shareB, alpha, Double.isNaN(dist) ? Double.NaN : dist,
                baseShares[0], baseShares[1], baseShares[2], baseAlpha, Double.isNaN(baseDist) ? 99.0 : baseDist);
        if (raw == candidate) candidateRuns++;
        else {
            candidate = raw;
            candidateRuns = 1;
        }
        boolean wasPresent = present;
        if (candidate != present && candidateRuns >= (candidate ? ON_READS : OFF_READS)) present = candidate;
        slotNow = present ? ElementCounter.classify(SENSOR, shareR, shareG, shareB, Double.isNaN(dist) ? 99.0 : dist) : ElementCounter.Slot.EMPTY;

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
        if (inPass && !present) {
            finishPass(nowT, pusherState);
        }
    }

    private void finishPass(double nowT, String pusherState) {
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
        RunLog.event(String.format(Locale.US, "CS4 PASS #%d: %s, %d reads over %.0f ms (%.1f ms per read), peak alpha %.3f, nearest %.1f cm, votes: %spusher %s -> %s (%s)",
                passId, cls, passReads, dur * 1000.0, passReads > 0 ? dur * 1000.0 / passReads : 0.0, passPeakAlpha, passMinDist, v, passPusherState, pusherState,
                arrival ? "arrival, pusher at HOLD" : "pusher moving"));
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

        RevColorSensorV3 cs4 = hardwareMap.tryGet(RevColorSensorV3.class, "CS4");
        if (cs4 == null) {
            telemetry.addLine("CS4 is not a REV Color Sensor V3 in the configuration: nothing to study");
            telemetry.update();
            waitForStart();
            return;
        }
        rateNote = FAST_RATE ? applyFastRate(cs4) : "default rate (100 ms)";
        telemetry.addData("CS4 rate", rateNote);
        telemetry.addData("Mechanisms present", mechanisms.describe());

        // INIT: read CS4 every loop and keep the reads for the empty baseline
        ElapsedTime initLoop = new ElapsedTime();
        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();
            readCs4(cs4);
            rememberInitRead();
            if (gamepad1.aWasPressed()) captureBaseline();
            telemetry.addData("CS4 rate", rateNote);
            telemetry.addData("CS4 now", "alpha %.4f  dist %s  shares %.3f / %.3f / %.3f  (%.1f ms per read)", alpha,
                    Double.isNaN(dist) ? "-" : String.format(Locale.US, "%.2f cm", dist), shareR, shareG, shareB, readMs);
            telemetry.addData("Empty baseline", baselineSet ? String.format(Locale.US, "alpha %.4f dist %.2f", baseAlpha, baseDist) : "not taken yet (taken automatically at Start)");
            telemetry.addLine("Keep the track EMPTY and the arm at HOLD; keep still while the Pinpoint calibrates. A = take the baseline now. Then Start.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        captureBaseline();
        mechanisms.setManualSetpoint(SETPOINT);
        mechanisms.setSpinRequested(true);
        RunLog.started();
        RunLog.event("SETUP CS4 pass study: " + rateNote + ", READ_DISTANCE " + READ_DISTANCE + ", loop target " + LOOP_TARGET_MS + " ms, setpoint " + SETPOINT
                + ", auto clear " + AUTO_CLEAR + " at " + CLEAR_AT_COUNT + ", presence " + ON_READS + "/" + OFF_READS + " reads, " + mechanisms.describe());
        RunLog.setSampler("cs4_age_ms,cs4_raw_r,cs4_raw_g,cs4_raw_b,cs4_alpha,cs4_r,cs4_g,cs4_b,cs4_prox,cs4_dist,cs4_read_ms,cs4_fresh,cs4_present,cs4_class,cs4_pass,auto_clear,"
                        + Mechanisms.SAMPLE_COLUMNS, 1,
                () -> String.format(Locale.US, "%.1f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%d,%s,%.2f,%d,%d,%s,%d,%d,%s",
                        (System.nanoTime() - lastReadNs) / 1e6, rawR, rawG, rawB, alpha, shareR, shareG, shareB, proxRaw,
                        Double.isNaN(dist) ? "" : String.format(Locale.US, "%.2f", dist), readMs, fresh ? 1 : 0, present ? 1 : 0, slotNow, inPass ? passId : 0,
                        AUTO_CLEAR ? 1 : 0, mechanisms.sampleRow()));

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime sinceSalvo = new ElapsedTime();
        int marks = 0;
        double loopSum = 0, loopMax = 0;
        long loops = 0;
        int lastSalvos = 0;
        sinceStart.reset();
        double lastLoopTime = 0;

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
                passesTotal = passesDuringIntake = passesDuringSalvo = 0;
                Arrays.fill(classTally, 0);
                passDurations.clear();
                passReadCounts.clear();
                RunLog.event("TALLIES cleared");
            }
            if (gamepad1.aWasPressed()) {
                marks++;
                RunLog.event(String.format(Locale.US, "MARK %d: CS4 passes so far %d, counter %d, salvos %d", marks, passesTotal,
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

            // the CS4 read of this loop, after the drive and the controls, before the mechanisms (as a counter read would be)
            readCs4(cs4);
            String pusherState = mechanisms.pusher == null ? "" : String.valueOf(mechanisms.pusher.getState());
            classifyAndTrack(sinceStart.seconds(), pusherState);

            mechanisms.setManualSetpoint(SETPOINT);
            mechanisms.setSpinRequested(true);
            mechanisms.update(pose);

            telemetry.addData("CS4 now", "%s  alpha %.3f  dist %s  shares %.2f/%.2f/%.2f", present ? slotNow : "empty", alpha,
                    Double.isNaN(dist) ? "-" : String.format(Locale.US, "%.1f", dist), shareR, shareG, shareB);
            telemetry.addData("CS4 passes", "%d total: %d at HOLD (arrivals), %d with the pusher moving", passesTotal, passesDuringIntake, passesDuringSalvo);
            StringBuilder cls = new StringBuilder();
            for (ElementCounter.Slot s : ElementCounter.Slot.values()) if (classTally[s.ordinal()] > 0) cls.append(s).append(' ').append(classTally[s.ordinal()]).append("  ");
            telemetry.addData("By class", cls.length() == 0 ? "-" : cls.toString());
            telemetry.addData("Last pass", lastPass);
            double meanReads = 0, meanMs = 0;
            for (int n : passReadCounts) meanReads += n;
            for (double d : passDurations) meanMs += d * 1000.0;
            if (!passReadCounts.isEmpty()) {
                meanReads /= passReadCounts.size();
                meanMs /= passDurations.size();
            }
            telemetry.addData("Per pass", "%.1f reads, %.0f ms (mean)", meanReads, meanMs);
            telemetry.addData("CS4 reads", "%.1f ms each (max %.0f), %.0f%% fresh, %s", reads > 0 ? readMsSum / reads : 0.0, readMsMax,
                    reads > 0 ? 100.0 * freshReads / reads : 0.0, rateNote);
            telemetry.addData("Loop", "%.1f ms mean, %.0f ms max (target %.0f)", loops > 0 ? loopSum / loops : 0.0, loopMax, LOOP_TARGET_MS);
            telemetry.addData("Counter", "%d held, salvos %d, setpoint %.0f", mechanisms.counter != null ? mechanisms.counter.count() : -1,
                    mechanisms.getSalvosDone(), SETPOINT);
            telemetry.addData("Auto clear", "%s at %d", AUTO_CLEAR ? "ON" : "off", CLEAR_AT_COUNT);
            mechanisms.log(telemetry);
            telemetry.addLine("Y salvo, X auto clear, A mark, B clear tallies, D-pad up/down speed, LT outtake, Back clear fault");
            telemetry.update();

            // pad the loop to the emulated match loop time
            while (LOOP_TARGET_MS > 0 && loopTimer.milliseconds() < LOOP_TARGET_MS && opModeIsActive()) sleep(1);
            lastLoopTime = loopTimer.milliseconds();
            loopSum += lastLoopTime;
            loopMax = Math.max(loopMax, lastLoopTime);
            loops++;
            RunLog.tick();
        }
        RunLog.event(String.format(Locale.US, "CS4 PASS SUMMARY: %d passes (%d at HOLD, %d with the pusher moving), classes %s, mean %.1f reads and %.0f ms per pass; "
                        + "%d CS4 reads, %.1f ms each, %.0f%% fresh; loop %.1f ms mean, %.0f ms max over %d loops; counter %d, salvos %d",
                passesTotal, passesDuringIntake, passesDuringSalvo, Arrays.toString(classTally),
                passReadCounts.isEmpty() ? 0.0 : passReadCounts.stream().mapToInt(Integer::intValue).average().orElse(0),
                passDurations.isEmpty() ? 0.0 : passDurations.stream().mapToDouble(Double::doubleValue).average().orElse(0) * 1000.0,
                reads, reads > 0 ? readMsSum / reads : 0.0, reads > 0 ? 100.0 * freshReads / reads : 0.0, loops > 0 ? loopSum / loops : 0.0, loopMax, loops,
                mechanisms.counter != null ? mechanisms.counter.count() : -1, mechanisms.getSalvosDone()));
        drive.stop();
        mechanisms.setIntakeAlwaysOn(false);
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
