package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import java.util.concurrent.locks.LockSupport;

/**
 * CS pass study (2026-11-10 as the CS4 Pass Study; CS5 added 2026-11-12; reworked 2026-11-14): can CS5, a second REV Color Sensor V3 named "CS5" nearer the INTAKE than the
 * counter's sensors (the first sensor an element reaches), see and count the elements that come in, at match loop times, WITHOUT slowing the loop?
 *
 * The robot runs like a match: the real mechanisms (intake IN the whole time, the flywheel spinning at a slow SETPOINT, the pusher, the element counter on CS1 to CS4 for fullness and
 * the pusher check, read EXACTLY as in the TeleOp: one sensor every ElementCounter.READ_PERIOD_MS, cycling, standard SDK reads, no fast rate), the Pinpoint, the drive, the Dashboard and DS
 * telemetry and the run log, padded to LOOP_TARGET_MS per loop (the TeleOp's 28 to 30 ms; 0 runs flat out). The main loop does NOT read CS5 at all, and does not read CS4 on its own either:
 * CS4's data is the counter's (the cs4_ columns repeat the counter's last read of each of CS1 to CS4).
 *
 * CS5 has its own THREAD (2026-11-14), paced at CS5_PERIOD_MS (10 ms, a Dashboard value read live; 0 or less runs flat out): each cycle is ONE I2C transaction, a 15-byte burst read from
 * MAIN_STATUS that returns the status flags, the proximity count, and the IR, green, blue and red channels together (the SDK's own color + distance read is three transactions, 9.8 ms
 * with nothing else on the hub in the 2026-11-12 runs; the 2026-11-13 flat-out proximity thread took the hub for the whole run and the main loop went from 58 ms to 129 ms). Reading the status
 * register also clears the "new data" flags, so every sample says whether the proximity and the color are NEW since the previous sample (the color updates every 25 ms, the proximity every 6.25 ms).
 * The sensor is set to the same fast preset as before (color 25 ms 16-bit, proximity 6.25 ms 8-bit, set by reflection at INIT). Each sample is stamped on the study clock (nanoTime from the
 * same origin as study_t) in a ring that the main loop drains into the log row, so the main loop only formats text.
 *
 * Log: study_t, cs5_n (samples in this row), cs5f ("prox@ms ..." for every sample since the previous row, ms on the study clock; the same format as before, tools/logs/cs5_counting.py --fast
 * replays it), cs5c ("ms:status:green:blue:red:ir ..." the same samples with the raw 16-bit channel values (blue and red WITHOUT the SDK's x1.55 / x1.07 factors) and the status byte: bit 0 =
 * new proximity, bit 3 = new color), cs1..cs4 alpha, color shift and distance from the counter, then the usual mechanism columns. A CS5 THREAD line every FAST_STATS_SEC says how it really ran: the
 * mean and maximum interval, the intervals over 20 ms, the read time, the thread's share of the time (read time over elapsed: the hub time it takes), the fresh shares and the errors.
 * The empty baseline is the first 1 s of the run (keep the track EMPTY and the arm at HOLD until the first line). CS PASS SUMMARY gives the main loop's mean and maximum, to put next to the thread's.
 * There is NO ground truth: compare the samples with the counter's count and the salvos' "N held". A drops a MARK line in the log.
 *
 * Feed pollen (by hand or from the floor with the intake IN). The track is cleared by salvos at the slow SETPOINT (500 ticks/s): AUTO_CLEAR fires one when the element counter reads FULL
 * (CLEAR_AT_COUNT), or press Y.
 *
 * Gamepad 1:  left stick / right stick X  drive / turn (robot oriented, RB precision)     Left trigger (hold): outtake (clear a jam)
 *             Y  fire a salvo now      X  AUTO_CLEAR on / off      A  MARK (a note in the log)
 *             B  FED: press once for EACH element as it goes into the intake (a FED line with the study clock; the ground truth for the CS5 counts: your reaction is 0.2 to 0.4 s late, the number is exact)
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
    /** CS5's fast preset: color 25 ms 16-bit, proximity 6.25 ms (set by reflection; the SDK default is 100 ms for both). Applied at INIT. */
    public static boolean FAST_RATE = true;
    /**
     * The CS5 thread's period, ms (read live): one burst read starts every this many ms (a read that runs late is not caught up). 20 = 50 Hz (10 until 2026-11-14: the 13:16 run took 7 ms per read, 69 percent of
     * the hub, and the main loop ran 47 ms against the TeleOp's 26). 0 or less: back to back, as fast as it goes (which keeps the hub busy the whole time: the 2026-11-13 run).
     * Set it in the Dashboard before Start to try another value without a rebuild.
     */
    public static double CS5_PERIOD_MS = 20.0;
    /** Seconds between the CS5 THREAD lines in the log. */
    public static double FAST_STATS_SEC = 5.0;
    /** How long at the start of the run the empty baseline is taken from, seconds. */
    public static double BASELINE_SEC = 1.0;
    /** Fire a salvo by itself when the element counter reports CLEAR_AT_COUNT or more. */
    public static boolean AUTO_CLEAR = true;
    public static int CLEAR_AT_COUNT = 4;
    /** Seconds to wait after a salvo before AUTO_CLEAR may fire again (the pusher check runs then). */
    public static double CLEAR_PAUSE_SEC = 2.0;
    public static double STEP = 20.0;
    public static double TRIGGER_THRESHOLD = 0.3;
    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;

    /** Burst read: MAIN_STATUS (0x07) through LS_DATA_RED's third byte (0x15), 15 bytes. */
    private static final int BURST_BYTES = 15;

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

    private static int u16(byte[] d, int i) {
        return (d[i] & 0xFF) | ((d[i + 1] & 0xFF) << 8);
    }

    /**
     * The CS5 thread: one burst read per cycle (status, proximity, IR, green, blue, red) into a ring, paced at CS5_PERIOD_MS; the main loop drains the ring into the log rows. The time of each
     * sample is nanoTime at the START of its read, from the same origin as the study clock.
     */
    private final class Cs5Thread implements Runnable {
        static final int RING = 4096;
        final RevColorSensorV3 sensor;
        volatile long originNs;
        final long[] ts = new long[RING];
        final int[] status = new int[RING], prox = new int[RING], ir = new int[RING], g = new int[RING], b = new int[RING], r = new int[RING], readUs = new int[RING];
        volatile long head = 0L;            // samples written (the thread)
        long tail = 0L;                     // samples drained (the OpMode thread)
        volatile boolean running = true;
        volatile long errors = 0L;
        long dropped = 0L;
        Thread thread;
        // statistics of the samples drained: the window since the last stats line, and the whole run
        long winN = 0L, runN = 0L, winOver20 = 0L, runOver20 = 0L, winPsFresh = 0L, runPsFresh = 0L, winLsFresh = 0L, runLsFresh = 0L, winSamples = 0L, runSamples = 0L;
        double winSumMs = 0.0, winMaxMs = 0.0, runSumMs = 0.0, runMaxMs = 0.0, winReadMs = 0.0, runReadMs = 0.0, winReadMax = 0.0, runReadMax = 0.0;
        long lastTs = 0L;
        int lastDrainCount = 0;
        String colorText = "";
        // the empty baseline: the first BASELINE_SEC of the run
        final List<double[]> baseline = new ArrayList<>();
        boolean baselineDone = false;

        Cs5Thread(RevColorSensorV3 sensor, long originNs) {
            this.sensor = sensor;
            this.originNs = originNs;
        }

        void start() {
            originNs = System.nanoTime();
            thread = new Thread(this, "CS5 burst reader");
            thread.setDaemon(true);
            thread.start();
        }

        @Override
        public void run() {
            long next = System.nanoTime();
            while (running) {
                long t0 = System.nanoTime();
                try {
                    byte[] d = sensor.read(BroadcomColorSensor.Register.MAIN_STATUS, BURST_BYTES);
                    long t1 = System.nanoTime();
                    int i = (int) (head % RING);
                    ts[i] = t0;
                    status[i] = d[0] & 0xFF;
                    prox[i] = u16(d, 1) & 0x7FF;
                    ir[i] = u16(d, 3);
                    g[i] = u16(d, 6);
                    b[i] = u16(d, 9);
                    r[i] = u16(d, 12);
                    readUs[i] = (int) ((t1 - t0) / 1000L);
                    head = head + 1;
                } catch (Throwable e) {
                    errors++;
                    try {
                        Thread.sleep(2);
                    } catch (InterruptedException ie) {
                        return;
                    }
                }
                double period = CS5_PERIOD_MS;
                if (period > 0.0) {
                    next += (long) (period * 1e6);
                    long now = System.nanoTime();
                    if (next <= now) next = now;   // late: no catch-up burst
                    else LockSupport.parkNanos(next - now);
                } else {
                    next = System.nanoTime();
                }
            }
        }

        /** Drains the samples since the last call into the cs5f and cs5c texts; updates the statistics and the baseline. */
        String[] drain() {
            long h = head;
            if (h - tail > RING) {
                dropped += h - tail - RING;
                tail = h - RING;
            }
            StringBuilder f = new StringBuilder(), c = new StringBuilder();
            int n = 0;
            for (long k = tail; k < h; k++) {
                int i = (int) (k % RING);
                long t = ts[i];
                long tenths = (t - originNs) / 100_000L;
                if (n > 0) {
                    f.append(' ');
                    c.append(' ');
                }
                String stamp = (tenths / 10) + "." + (tenths % 10);
                f.append(prox[i]).append('@').append(stamp);
                c.append(stamp).append(':').append(status[i]).append(':').append(g[i]).append(':').append(b[i]).append(':').append(r[i]).append(':').append(ir[i]);
                boolean psFresh = (status[i] & 0x01) != 0, lsFresh = (status[i] & 0x08) != 0;
                winSamples++;
                runSamples++;
                if (psFresh) {
                    winPsFresh++;
                    runPsFresh++;
                }
                if (lsFresh) {
                    winLsFresh++;
                    runLsFresh++;
                }
                double rms = readUs[i] / 1000.0;
                winReadMs += rms;
                runReadMs += rms;
                winReadMax = Math.max(winReadMax, rms);
                runReadMax = Math.max(runReadMax, rms);
                if (lastTs != 0L) {
                    double dtMs = (t - lastTs) / 1e6;
                    winN++;
                    runN++;
                    winSumMs += dtMs;
                    runSumMs += dtMs;
                    winMaxMs = Math.max(winMaxMs, dtMs);
                    runMaxMs = Math.max(runMaxMs, dtMs);
                    if (dtMs > 20.0) {
                        winOver20++;
                        runOver20++;
                    }
                }
                lastTs = t;
                double tSec = (t - originNs) / 1e9;
                if (!baselineDone && tSec <= BASELINE_SEC) baseline.add(new double[]{prox[i], g[i], b[i], r[i], ir[i]});
                n++;
            }
            tail = h;
            lastDrainCount = n;
            if (!baselineDone && (lastTs - originNs) / 1e9 > BASELINE_SEC) {
                baselineDone = true;
                logBaseline();
            }
            if (n > 0) {
                int j = (int) ((h - 1) % RING);
                colorText = String.format(Locale.US, "prox %d  green %d  blue %d  red %d  ir %d", prox[j], g[j], b[j], r[j], ir[j]);
            }
            return new String[]{f.toString(), c.toString()};
        }

        private void logBaseline() {
            int m = baseline.size();
            if (m < 5) {
                RunLog.event("CS5 BASELINE: only " + m + " samples in the first " + BASELINE_SEC + " s");
                return;
            }
            double[] med = new double[5];
            for (int k = 0; k < 5; k++) {
                double[] v = new double[m];
                for (int q = 0; q < m; q++) v[q] = baseline.get(q)[k];
                Arrays.sort(v);
                med[k] = m % 2 == 1 ? v[m / 2] : 0.5 * (v[m / 2 - 1] + v[m / 2]);
            }
            RunLog.event(String.format(Locale.US, "CS5 BASELINE (empty, the first %.1f s, %d samples): proximity %.0f, green %.0f, blue %.0f, red %.0f, ir %.0f", BASELINE_SEC, m, med[0], med[1], med[2], med[3], med[4]));
        }

        private String stats(long n, long over20, double sumMs, double maxMs, long samples, double readMs, double readMax, long ps, long ls) {
            if (n == 0) return "no samples";
            double meanMs = sumMs / n;
            return String.format(Locale.US, "%d samples, period %.1f ms asked: interval mean %.1f ms (%.0f Hz), max %.0f ms, %d over 20 ms; read %.1f ms mean (max %.0f), the thread holds the hub %.0f%% of the time; "
                            + "new proximity %.0f%%, new color %.0f%%; errors %d, dropped %d",
                    samples, CS5_PERIOD_MS, meanMs, 1000.0 / Math.max(meanMs, 1e-9), maxMs, over20, readMs / Math.max(1, samples), readMax, 100.0 * readMs / Math.max(sumMs, 1e-9),
                    100.0 * ps / Math.max(1, samples), 100.0 * ls / Math.max(1, samples), errors, dropped);
        }

        String windowStats() {
            String text = stats(winN, winOver20, winSumMs, winMaxMs, winSamples, winReadMs, winReadMax, winPsFresh, winLsFresh);
            winN = winOver20 = winPsFresh = winLsFresh = winSamples = 0L;
            winSumMs = winMaxMs = winReadMs = winReadMax = 0.0;
            return text;
        }

        String runStats() {
            return stats(runN, runOver20, runSumMs, runMaxMs, runSamples, runReadMs, runReadMax, runPsFresh, runLsFresh);
        }

        void stop() {
            running = false;
            try {
                if (thread != null) thread.join(500);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private Cs5Thread cs5Thread;

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
            if (cs5Thread != null) cs5Thread.stop();
            Flywheel.FEED_FORWARD = origFeedForward;   // a static: left changed it would stay changed for the match TeleOp until the app restarts
            RunLog.close();
        }
    }

    private void runStudy() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);
        mechanisms.setUseLaunchMap(false);
        mechanisms.setSpeedSetByOpMode(true);
        mechanisms.setAutoStopAfterSalvo(false);
        mechanisms.setBlueAlliance(true);
        mechanisms.setOpenCell(HiveTarget.BLUE_REAR);
        mechanisms.setIntakeAlwaysOn(true);
        if (mechanisms.entries != null) mechanisms.entries.disable();   // this opmode reads CS5 on its own thread

        RevColorSensorV3 s5 = hardwareMap.tryGet(RevColorSensorV3.class, "CS5");
        if (s5 == null) {
            telemetry.addLine("CS5 is not a REV Color Sensor V3 in the configuration: nothing to study");
            telemetry.update();
            waitForStart();
            return;
        }
        String rateText = FAST_RATE ? applyFastRate(s5) : "default rate (100 ms)";

        // INIT: the counter reads as in the TeleOp; CS5 is read once per loop (a burst) to show it works
        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();
            String cs5Now;
            try {
                byte[] d = s5.read(BroadcomColorSensor.Register.MAIN_STATUS, BURST_BYTES);
                cs5Now = String.format(Locale.US, "proximity %d  green %d  blue %d  red %d  ir %d  status 0x%02X", u16(d, 1) & 0x7FF, u16(d, 6), u16(d, 9), u16(d, 12), u16(d, 3), d[0] & 0xFF);
            } catch (Throwable e) {
                cs5Now = "READ FAILED: " + e;
            }
            telemetry.addData("CS5 rate", rateText);
            telemetry.addData("CS5 now", cs5Now);
            telemetry.addData("Counter", mechanisms.counter == null ? "not configured" : mechanisms.counter.isValid() ? "reading   " + mechanisms.counter.stackString() : "NOT READING");
            telemetry.addData("CS5 thread period", "%.1f ms (CS5_PERIOD_MS in the Dashboard)", CS5_PERIOD_MS);
            telemetry.addLine("Keep the track EMPTY and the arm at HOLD; keep still while the Pinpoint calibrates (the first " + BASELINE_SEC + " s after Start is the CS5 empty baseline). Then Start.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        applyStudyFeedForward();
        mechanisms.setManualSetpoint(SETPOINT);
        mechanisms.setSpinRequested(true);
        RunLog.started();
        RunLog.event("SETUP CS pass study: CS5 (nearest the intake) on its own thread, one 15-byte burst read per cycle, period " + CS5_PERIOD_MS + " ms, " + rateText + "; CS1 to CS4 read by the element counter exactly as in the TeleOp (one every "
                + ElementCounter.READ_PERIOD_MS + " ms), the main loop reads no color sensor; loop target " + LOOP_TARGET_MS + " ms, setpoint " + SETPOINT
                + ", flywheel hold power " + String.format(Locale.US, "%.3f", Math.max(0.0, Math.min(origFeedForward, STUDY_FEED_FORWARD_FRACTION * SETPOINT / Flywheel.OPEN_LOOP_TOP_SPEED))) + " (was " + origFeedForward + ")"
                + ", auto clear " + AUTO_CLEAR + " at " + CLEAR_AT_COUNT + ", " + mechanisms.describe());
        long originNs = System.nanoTime();
        cs5Thread = new Cs5Thread(s5, originNs);
        final Cs5Thread cs5 = cs5Thread;
        final ElementCounter counter = mechanisms.counter;
        StringBuilder cols = new StringBuilder("study_t,cs5_n,cs5f,cs5c,");
        for (int i = 1; i <= ElementCounter.N; i++) cols.append("cs").append(i).append("_alpha,cs").append(i).append("_shift,cs").append(i).append("_dist,");
        cols.append("auto_clear,").append(Mechanisms.SAMPLE_COLUMNS);
        RunLog.setSampler(cols.toString(), 1, () -> {
            StringBuilder sb = new StringBuilder(String.format(Locale.US, "%.3f,", sinceStart.seconds()));
            String[] samples = cs5.drain();
            sb.append(cs5.lastDrainCount).append(',').append(samples[0]).append(',').append(samples[1]).append(',');
            for (int i = 0; i < ElementCounter.N; i++) {
                if (counter != null && counter.isValid()) {
                    sb.append(String.format(Locale.US, "%.3f,%.3f,%.1f,", counter.lastAlpha(i), counter.colorShift(i), counter.lastDistCm(i)));
                } else {
                    sb.append(",,,");
                }
            }
            sb.append(AUTO_CLEAR ? 1 : 0).append(',').append(mechanisms.sampleRow());
            return sb.toString();
        });

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime sinceSalvo = new ElapsedTime();
        int marks = 0, fed = 0;
        double loopSum = 0, loopMax = 0;
        long loops = 0;
        long slowLoops = 0;
        int lastSalvos = 0;
        sinceStart.reset();
        cs5.start();
        double lastStatsT = 0.0;
        double windowLoopSum = 0.0, windowLoopMax = 0.0;
        long windowLoops = 0;

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
            if (gamepad1.aWasPressed()) {
                marks++;
                RunLog.event(String.format(Locale.US, "MARK %d: study clock %.3f s, counter %d, salvos %d", marks, sinceStart.seconds(),
                        mechanisms.counter != null ? mechanisms.counter.count() : -1, mechanisms.getSalvosDone()));
            }
            if (gamepad1.bWasPressed()) {
                fed++;
                RunLog.event(String.format(Locale.US, "FED %d at study clock %.3f s (counter %d)", fed, sinceStart.seconds(), mechanisms.counter != null ? mechanisms.counter.count() : -1));
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

            applyStudyFeedForward();   // follows the D-pad speed changes
            mechanisms.setManualSetpoint(SETPOINT);
            mechanisms.setSpinRequested(true);
            mechanisms.update(pose);

            telemetry.addData("Elements fed (B, once per element)", fed);
            telemetry.addData("CS5 (latest sample)", cs5.colorText.isEmpty() ? "-" : cs5.colorText);
            telemetry.addData("CS5 thread", "%d samples; %s", cs5.head, cs5.runStats());
            telemetry.addData("Loop", "%.1f ms mean, %.0f ms max, %d over 80 ms (target %.0f)", loops > 0 ? loopSum / loops : 0.0, loopMax, slowLoops, LOOP_TARGET_MS);
            telemetry.addData("Counter", "%d held, salvos %d, setpoint %.0f", mechanisms.counter != null ? mechanisms.counter.count() : -1,
                    mechanisms.getSalvosDone(), SETPOINT);
            telemetry.addData("Auto clear", "%s at %d", AUTO_CLEAR ? "ON" : "off", CLEAR_AT_COUNT);
            mechanisms.log(telemetry);
            telemetry.addLine("Y salvo, X auto clear, A mark, B = one element fed, D-pad up/down speed, LT outtake, Back clear fault");
            telemetry.update();

            if (sinceStart.seconds() - lastStatsT >= FAST_STATS_SEC) {
                lastStatsT = sinceStart.seconds();
                RunLog.event(String.format(Locale.US, "CS5 THREAD: %s | main loop in this window: %.1f ms mean, %.0f ms max over %d loops", cs5.windowStats(),
                        windowLoops > 0 ? windowLoopSum / windowLoops : 0.0, windowLoopMax, windowLoops));
                windowLoopSum = 0.0;
                windowLoopMax = 0.0;
                windowLoops = 0;
            }
            // pad the loop to the emulated match loop time
            while (LOOP_TARGET_MS > 0 && loopTimer.milliseconds() < LOOP_TARGET_MS && opModeIsActive()) sleep(1);
            double lastLoopTime = loopTimer.milliseconds();
            loopSum += lastLoopTime;
            loopMax = Math.max(loopMax, lastLoopTime);
            windowLoopSum += lastLoopTime;
            windowLoopMax = Math.max(windowLoopMax, lastLoopTime);
            windowLoops++;
            if (lastLoopTime > 80.0) slowLoops++;
            loops++;
            RunLog.tick();
        }
        cs5.stop();
        RunLog.event("CS5 THREAD SUMMARY: " + cs5.runStats());
        RunLog.event(String.format(Locale.US, "CS PASS SUMMARY: loop %.1f ms mean, %.0f ms max over %d loops (%d over 80 ms), CS5 period %.1f ms; counter %d, salvos %d, elements fed (B) %d", loops > 0 ? loopSum / loops : 0.0, loopMax, loops,
                slowLoops, CS5_PERIOD_MS, mechanisms.counter != null ? mechanisms.counter.count() : -1, mechanisms.getSalvosDone(), fed));
        drive.stop();
        mechanisms.setIntakeAlwaysOn(false);
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
