package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.locks.LockSupport;

/**
 * Counts the elements as they ENTER the intake, from CS5 (a REV Color Sensor V3 named "CS5", nearer the intake than the counter's CS1 to CS4) (2026-11-14, SHADOW MODE: it counts and logs, and
 * nothing in the robot's behavior depends on it yet).
 *
 * A thread reads CS5's raw proximity (rawOptical: one 2-byte I2C read, about 3 ms) every PERIOD_MS (30) and feeds a {@link PeakCounter}; every confirmed peak is one element. The 2026-11-14 study
 * (design/opmodes.md, CS Pass Study) showed that sampling every 60 to 80 ms still finds 94 to 97 percent of the elements, and the thread at 30 ms keeps about 10 percent of the hub's time
 * (a 10 ms burst thread took 69 percent and ran the main loop at 47 ms against the TeleOp's 26; a 20 ms one 34 percent and 40 ms). The thread starts with the first {@link #touch()} (Mechanisms.update
 * calls it every loop, so nothing is read in INIT) and ends when touch stops (the opmode is over) or on {@link #stop()}.
 *
 * The sensor is put on its fast proximity preset (8-bit, a new value every 6.25 ms; the SDK default is 100 ms) when this is built (INIT). A failed preset is logged and the count would be useless
 * (10 Hz), so {@link #presetOk} says whether it worked.
 *
 * The main loop reads {@link #entries()} and drains {@link #pollEntry()} for the log (the log is not written from the thread).
 */
@Config
public final class EntryCounter {

    /**
     * Time between the starts of two reads, ms. 25 since 2026-11-14 (user: a middle ground; 30 first: the 13:57 TeleOp ran 28.9 ms mean against the TeleOp's 25.9, 17 percent of the loops over 40 ms and 1 Pinpoint blip in 84 s; 20 next: the 14:16 TeleOp
     * ran 31.8 ms mean, 26 percent over 40 ms and 7 Pinpoint blips in 72 s; each read takes 6 ms of wall-clock time, queued behind the main loop's commands). 0 or less reads back to back (keeps the hub busy: not for a match).
     */
    public static double PERIOD_MS = 25.0;
    /** A bump counts when it stands this many counts above its surroundings (the empty level is about 65 with noise of about 0.5; elements are +8 to +44). Read when the thread starts. */
    public static double PROMINENCE = 6.0;
    /** 1: no smoothing (default), 3: a median of three samples. Read when the thread starts. */
    public static int SMOOTH = 1;
    /** Two peaks closer than this, seconds, are one. */
    public static double MIN_SEPARATION_SEC = 0.06;
    /**
     * (EntryGrouper, 2026-11-14) A group of peaks that follow each other within MERGE_GAP_SEC and contain one of at least MERGE_HEIGHT counts is ONE entry (a wall, a flower stand or an element at close
     * range rings with several sub-peaks; the study's elements were +8 to +44). Groups of smaller peaks are counted peak by peak. MERGE_HEIGHT 100000 turns the merging off. Read when the thread starts.
     */
    public static double MERGE_HEIGHT = 45.0;
    public static double MERGE_GAP_SEC = 0.4;
    /**
     * (EntryPipeline, 2026-11-14) A peak whose VALLEY (the lowest point before it) is more than this many counts above the empty level is rejected. OFF (100000) since 2026-11-14 (user): it rejected the wiggles of elements
     * parked under the sensor after a pile-up (valleys +4 to +8; the valley rule at 3 gave 18 entries on the 14:16 TeleOp, 15 of them right), but the robot now goes OUT when full, which removes that case, and tight groupings
     * whose dips do not come back to the empty level are expected. 3 would turn it on again. Read when the thread starts.
     */
    public static double VALLEY_MAX = 100000.0;
    /** The empty-level statistics (for the log) are taken over the first this many seconds after the thread starts. */
    public static double BASELINE_SEC = 1.0;
    /** The thread ends when touch() has not been called for this long (the opmode ended). */
    public static double IDLE_EXIT_SEC = 1.5;
    /** Seconds between the CS5 ENTRY THREAD stats lines. */
    public static double STATS_SEC = 15.0;
    /** False: never start the thread (an opmode that reads CS5 itself, the CS Pass Study). */
    public static boolean ENABLED = true;

    /** One counted element. */
    public static final class Entry {
        public final long number;
        public final double timeSec;     // seconds since the thread started
        public final double height;      // counts above the lowest point before it
        public final int proxAtPeak;      // the proximity at the latest sample when it was counted (a merged group: the last)
        public final int peaks;           // raw peaks merged into it (1 = a plain entry)
        public final String detail;       // "+26@15.51 +94@15.13" for a merged group
        public final long peakNanos;      // System.nanoTime() of its (tallest) peak, to put it on the log's clock

        Entry(long number, double timeSec, double height, int proxAtPeak, int peaks, String detail, long peakNanos) {
            this.number = number;
            this.timeSec = timeSec;
            this.height = height;
            this.proxAtPeak = proxAtPeak;
            this.peaks = peaks;
            this.detail = detail;
            this.peakNanos = peakNanos;
        }

        public boolean merged() {
            return peaks > 1;
        }
    }

    private final RevColorSensorV3 sensor;
    public final String presetNote;
    public final boolean presetOk;
    private final ConcurrentLinkedQueue<Entry> queue = new ConcurrentLinkedQueue<>();
    // the raw samples for the log (drained by Mechanisms.sampleRow): the nanoTime and the proximity of each
    private static final int RING = 2048;
    private final long[] ringT = new long[RING];
    private final int[] ringV = new int[RING];
    private volatile long ringHead = 0L;
    private long ringTail = 0L;
    private long ringDropped = 0L;
    private volatile long entries = 0L;
    private volatile int lastProx = -1;
    private volatile long lastSampleNs = 0L;
    private volatile long samples = 0L;
    private volatile long errors = 0L;
    private volatile long lastTouchNs = 0L;
    private volatile boolean running = false;
    private volatile boolean started = false;
    private volatile boolean disabled = false;
    private volatile String baselineNote = "";
    private volatile double baseline = Double.NaN, sigma = Double.NaN;
    private volatile long rejected = 0L;
    private Thread thread;
    private long originNs;
    // thread-side interval statistics, copied out by statsLine()
    private final Object statLock = new Object();
    private long winN = 0L, winOver = 0L;
    private double winSumMs = 0.0, winMaxMs = 0.0, winReadMs = 0.0, winReadMax = 0.0;
    private long runN = 0L;
    private double runSumMs = 0.0, runMaxMs = 0.0, runReadMs = 0.0, runReadMax = 0.0;
    private long runOver = 0L;

    private EntryCounter(RevColorSensorV3 sensor) {
        this.sensor = sensor;
        String note;
        boolean ok;
        try {
            Method ps = BroadcomColorSensorImpl.class.getDeclaredMethod("setPSRateAndRes", BroadcomColorSensor.PSResolution.class, BroadcomColorSensor.PSMeasurementRate.class);
            ps.setAccessible(true);
            ps.invoke(sensor, BroadcomColorSensor.PSResolution.R8BIT, BroadcomColorSensor.PSMeasurementRate.R6_25ms);
            note = "proximity preset applied (8-bit, a new value every 6.25 ms)";
            ok = true;
        } catch (Exception e) {
            note = "PROXIMITY PRESET FAILED (" + e + "): the sensor refreshes every 100 ms, the count will be useless";
            ok = false;
        }
        this.presetNote = note;
        this.presetOk = ok;
    }

    /** The counter on the configuration's "CS5", or null when there is none (the robot runs without it). */
    public static EntryCounter tryCreate(HardwareMap hardwareMap) {
        RevColorSensorV3 s = hardwareMap.tryGet(RevColorSensorV3.class, "CS5");
        return s == null ? null : new EntryCounter(s);
    }

    /** Called every loop by Mechanisms.update: starts the thread on the first call and keeps it alive. */
    public void touch() {
        lastTouchNs = System.nanoTime();
        if (!started && ENABLED && !disabled) start();
    }

    /** This opmode reads CS5 itself: the thread will not start (and stops if it is running). */
    public void disable() {
        disabled = true;
        stop();
    }

    private synchronized void start() {
        if (started) return;
        started = true;
        running = true;
        originNs = System.nanoTime();
        thread = new Thread(this::run, "CS5 entry counter");
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        running = false;
        Thread t = thread;
        if (t != null && t != Thread.currentThread()) {
            try {
                t.join(300);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private void run() {
        EntryPipeline pipe = new EntryPipeline(PROMINENCE, SMOOTH, MIN_SEPARATION_SEC, MERGE_HEIGHT, MERGE_GAP_SEC, VALLEY_MAX, BASELINE_SEC);
        long next = System.nanoTime();
        long lastStart = 0L;
        int consecutiveErrors = 0;
        while (running) {
            long t0 = System.nanoTime();
            if ((t0 - lastTouchNs) / 1e9 > IDLE_EXIT_SEC) break;   // the opmode is over
            try {
                int v = sensor.rawOptical();
                long t1 = System.nanoTime();
                consecutiveErrors = 0;
                lastProx = v;
                lastSampleNs = t1;
                samples++;
                int ri = (int) (ringHead % RING);
                ringT[ri] = t0;
                ringV[ri] = v;
                ringHead = ringHead + 1;
                double tSec = (t0 - originNs) / 1e9;
                synchronized (statLock) {
                    double readMs = (t1 - t0) / 1e6;
                    winReadMs += readMs;
                    runReadMs += readMs;
                    winReadMax = Math.max(winReadMax, readMs);
                    runReadMax = Math.max(runReadMax, readMs);
                    if (lastStart != 0L) {
                        double dtMs = (t0 - lastStart) / 1e6;
                        winN++;
                        runN++;
                        winSumMs += dtMs;
                        runSumMs += dtMs;
                        winMaxMs = Math.max(winMaxMs, dtMs);
                        runMaxMs = Math.max(runMaxMs, dtMs);
                        if (dtMs > 2.0 * Math.max(PERIOD_MS, 5.0)) {
                            winOver++;
                            runOver++;
                        }
                    }
                }
                lastStart = t0;
                for (EntryGrouper.Entry g : pipe.push(tSec, v)) {
                    long n = entries + 1;
                    queue.add(new Entry(n, g.time, g.height, v, g.peaks, g.detail, originNs + (long) (g.time * 1e9)));
                    entries = n;
                }
                rejected = pipe.rejected();
                baseline = pipe.emptyLevel();
                sigma = pipe.noise();
                String note = pipe.takeNote();
                if (note != null) baselineNote = note;
            } catch (Throwable e) {
                errors++;
                consecutiveErrors++;
                try {
                    Thread.sleep(consecutiveErrors > 20 ? 200 : 30);
                } catch (InterruptedException ie) {
                    break;
                }
            }
            double period = PERIOD_MS;
            if (period > 0.0) {
                next += (long) (period * 1e6);
                long now = System.nanoTime();
                if (next <= now) next = now;   // late: no catch-up burst
                else LockSupport.parkNanos(next - now);
            } else {
                next = System.nanoTime();
            }
        }
        running = false;
    }

    /** Elements counted since the thread started (the count of entries, not of what is held). */
    public long entries() {
        return entries;
    }

    /** The next counted element for the log, or null (drain this from the main loop). */
    public Entry pollEntry() {
        return queue.poll();
    }

    public int latestProx() {
        return lastProx;
    }

    public boolean isStarted() {
        return started;
    }

    /** True when the thread has produced a sample in the last 0.5 s. */
    public boolean isHealthy() {
        return started && running && (System.nanoTime() - lastSampleNs) / 1e9 < 0.5;
    }

    public long errors() {
        return errors;
    }

    /** Peaks rejected by the valley rule (the signal had not come back down to the empty level). */
    public long rejected() {
        return rejected;
    }

    /** The empty-level line once it has been measured (the first BASELINE_SEC), else "". */
    public String baselineNote() {
        return baselineNote;
    }

    public double baseline() {
        return baseline;
    }

    public double noise() {
        return sigma;
    }

    /**
     * The raw samples since the last call as "prox@-ms" separated by spaces, ms BEFORE this call (so they line up with the log row written now: the row's own time minus the number), for the log's cs5f
     * column. "" when there are none. Called from the main loop only (Mechanisms.sampleRow).
     */
    public String drainSamples() {
        long h = ringHead;
        if (h - ringTail > RING) {
            ringDropped += h - ringTail - RING;
            ringTail = h - RING;
        }
        if (h == ringTail) return "";
        long now = System.nanoTime();
        StringBuilder sb = new StringBuilder();
        for (long k = ringTail; k < h; k++) {
            int i = (int) (k % RING);
            if (sb.length() > 0) sb.append(' ');
            long tenths = Math.max(0L, (now - ringT[i]) / 100_000L);
            sb.append(ringV[i]).append("@-").append(tenths / 10).append('.').append(tenths % 10);
        }
        ringTail = h;
        return sb.toString();
    }

    /** The thread's rate for the window since the last call (and resets the window), for the log. */
    public String windowStats() {
        synchronized (statLock) {
            String text = format(winN, winOver, winSumMs, winMaxMs, winReadMs, winReadMax);
            winN = winOver = 0L;
            winSumMs = winMaxMs = winReadMs = winReadMax = 0.0;
            return text;
        }
    }

    public String runStats() {
        synchronized (statLock) {
            return format(runN, runOver, runSumMs, runMaxMs, runReadMs, runReadMax);
        }
    }

    private String format(long n, long over, double sumMs, double maxMs, double readMs, double readMax) {
        if (n == 0) return "no samples";
        double mean = sumMs / n;
        return String.format(Locale.US, "%d samples, interval mean %.1f ms (asked %.0f), max %.0f ms, %d late; read %.1f ms mean (max %.0f), %.0f%% of the hub; %d entries, %d rejected (valley), empty level %.0f, %d errors",
                n, mean, PERIOD_MS, maxMs, over, readMs / n, readMax, 100.0 * readMs / Math.max(sumMs, 1e-9), entries, rejected, baseline, errors);
    }
}
