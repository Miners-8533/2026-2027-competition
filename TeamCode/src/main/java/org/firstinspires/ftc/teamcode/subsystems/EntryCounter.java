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
 * Counts the elements as they ENTER the intake, from CS5 (a REV Color Sensor V3 named "CS5", nearer the intake than the counter's CS1 to CS4). From 2026-11-17 this is the count the robot relies on
 * (Mechanisms.COUNT_BY_ENTRIES): FULL, the driver screen, the autos' jostle end and the launched total come from it. (It began on 2026-11-14 in shadow mode, counting and logging only.)
 *
 * A thread reads CS5's raw proximity (rawOptical: one 2-byte I2C read, about 3 ms) every PERIOD_MS (25) and runs the counting rule on each sample: EDGE counting (USE_EDGE, from 2026-11-15: an entry
 * is the signal's rise to EDGE_HI above the empty level for EDGE_CONFIRM samples, re-armed when it is back at EDGE_LO), or, with that switch off, the earlier PEAK counting ({@link PeakCounter}).
 * The CS Pass Study of 2026-11-14 (design/opmodes.md) showed that a 20 ms thread ran the main loop at 40 ms against the TeleOp's 26 and a 10 ms one at 47 ms; at 25 ms the thread costs nothing
 * measurable. The thread starts with the first {@link #touch()} (Mechanisms.update calls it every loop, so nothing is read in INIT) and ends when touch stops (the opmode is over) or on {@link #stop()}.
 *
 * The sensor is put on its fast proximity preset (8-bit, a new value every 6.25 ms; the SDK default is 100 ms) when this is built (INIT). A failed preset is logged and the count would be useless
 * (10 Hz), so {@link #presetOk} says whether it worked.
 *
 * The main loop reads {@link #entries()} and drains {@link #pollEntry()} for the log (the log is not written from the thread).
 *
 * THE KIND OF EACH ENTRY (2026-11-17, not run on the robot; {@link EntryClassifier}, CLASSIFY): while an element is in front of the sensor the thread's read is the 15-byte BURST from MAIN_STATUS
 * (status, proximity, IR, green, blue, red in one transaction, the CS Pass Study's read: 6.8 ms against the 2-byte read's 5.6 to 6.4 ms in the 2026-11-14 logs) in place of the 2-byte one, so the
 * count gets the same proximity value and no transaction is added. With nothing in front, one read in IDLE_COLOR_EVERY is a burst (the empty color level the rise is measured from) and every read
 * of the first BASELINE_SEC is. The sensor's color is put on 25 ms / 16-bit when this is built; if that fails, or three bursts in a row fail, the color is off for the run, the thread reads the
 * 2 bytes as before and every entry's kind is UNKNOWN. Edge counting only. The main loop drains {@link #pollKind()}.
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
     * EDGE counting (2026-11-15, the user's idea; replaces the peaks): an entry is the RISE of the proximity to EDGE_HI counts above the empty level (held for EDGE_CONFIRM samples), counted at the rise (about 50 ms after, not 0.5 s as the peaks),
     * and the next one needs the signal back at EDGE_LO or lower. A persistent element, its wiggles and its ringing are one entry. USE_EDGE false goes back to the peaks (PROMINENCE, MERGE_*, VALLEY_MAX). Read when the thread starts.
     *
     * EDGE_LO 4 since 2026-11-18 (2 before; not run on the robot with 4). The first robot runs (ten TeleOps of 2026-11-17) showed that a driven load comes in as one burst of about a second whose dips
     * between the elements stay at +3 to +5, so with a re-arm at +2 the count read under 4 at 19 of the 37 salvos where all four slots read pollen (COUNT ONLY, loads from an emptied track). Replayed
     * with a re-arm at +4: under 4 at 5 of them, 4 at 22, 5 at 9, 6 at 1; the hand-fed study gives 32 for 31 fed. A re-arm at +5 overcounts (the 15:22 TeleOp of 2026-11-14 goes from 25 to 50; 29 at +4).
     * EDGE_HI stays 6: a rise of +7 reads the same on the driven loads but, at the robot's 25 ms, loses 1 to 3 of the study's 31 hand-fed elements (the weak ones are at +7 for one sample). The slots are
     * only a lower limit, so whether the fives are fifth elements or overcounts is not known. tools/entry/run-check.cmd replays all of it.
     */
    public static boolean USE_EDGE = true;
    public static double EDGE_HI = 6.0;
    public static double EDGE_LO = 4.0;
    public static int EDGE_CONFIRM = 2;
    /**
     * (EntryPipeline, 2026-11-14) A peak whose VALLEY (the lowest point before it) is more than this many counts above the empty level is rejected. OFF (100000) since 2026-11-14 (user): it rejected the wiggles of elements
     * parked under the sensor after a pile-up (valleys +4 to +8; the valley rule at 3 gave 18 entries on the 14:16 TeleOp, 15 of them right), but the robot now goes OUT when full, which removes that case, and tight groupings
     * whose dips do not come back to the empty level are expected. 3 would turn it on again. Read when the thread starts.
     */
    public static double VALLEY_MAX = 100000.0;
    /** The empty-level statistics (for the log) are taken over the first this many seconds after the thread starts. */
    public static double BASELINE_SEC = 1.0;
    /** The thread ends when touch() has not been called for this long (the opmode ended); a touch() after that (the loop only stood still) starts it again. */
    public static double IDLE_EXIT_SEC = 1.5;
    /** Seconds between the CS5 ENTRY THREAD stats lines. */
    public static double STATS_SEC = 15.0;
    /** False: never start the thread (an opmode that reads CS5 itself, the CS Pass Study). */
    public static boolean ENABLED = true;
    /**
     * (2026-11-17, not run on the robot) True: the thread reads CS5's color while an element passes and says what each entry is (EntryClassifier). False: the 2-byte proximity read only, exactly as before,
     * and every kind is UNKNOWN. Read live.
     */
    public static boolean CLASSIFY = true;
    /** With nothing in front of the sensor, one read in this many carries the color (the empty color level; 8 = every 200 ms). 1 = every read (the most hub time: 6.8 ms a read against about 6). Read live. */
    public static int IDLE_COLOR_EVERY = 8;
    /** A pass begins at this many proximity counts above the empty level (under EDGE_HI, so the color read is on when the entry is counted). Read when the thread starts, as the next eight. */
    public static double CLASS_NEAR = 2.0;
    /** A pass ends when the proximity has been under CLASS_NEAR for more than this many samples (the color is a 25 ms average and lags the proximity). */
    public static int CLASS_TAIL_SAMPLES = 2;
    /** An element still in front this long after it was counted is classified with what there is, and the color reads stop until it has gone. */
    public static double CLASS_MAX_SEC = 0.5;
    /** The empty color level is the median of this many of the latest color samples taken with nothing in front. */
    public static int CLASS_REF_SAMPLES = 3;
    /** Under this summed rise (green + blue + red, counts) the kind is UNKNOWN. The weakest of the 31 pollen of 2026-11-14 gave 121 at 20 ms. */
    public static double CLASS_MIN_SIGNAL = 60.0;
    /** Red's share of the rise at this or more: RED NECTAR. A FIRST GUESS (no nectar has been recorded at CS5; pollen's red share was 0.31 to 0.46). */
    public static double CLASS_RED_MIN = 0.55;
    /** Blue's share of the rise at this or more: BLUE NECTAR. A FIRST GUESS (pollen's blue share was 0.00 to 0.13). */
    public static double CLASS_BLUE_MIN = 0.40;
    /** POLLEN: green's share at CLASS_POLLEN_GREEN_MIN or more (measured 0.52 to 0.65) and blue's at CLASS_POLLEN_BLUE_MAX or less. */
    public static double CLASS_POLLEN_GREEN_MIN = 0.45;
    public static double CLASS_POLLEN_BLUE_MAX = 0.20;
    /** Burst read: MAIN_STATUS (0x07) through LS_DATA_RED's third byte (0x15), 15 bytes. */
    private static final int BURST_BYTES = 15;

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
    /** What the color preset did (for the log). */
    public final String colorNote;
    private final boolean colorPresetOk;
    private final ConcurrentLinkedQueue<Entry> queue = new ConcurrentLinkedQueue<>();
    private final ConcurrentLinkedQueue<EntryClassifier.Result> kinds = new ConcurrentLinkedQueue<>();
    private EntryClassifier classifier;   // used only by the counter thread, kept across a restart like the pipeline
    private volatile boolean colorOff = false;
    private volatile String colorFault = "";
    private volatile long colorReads = 0L, colorErrors = 0L;
    private volatile String colorReference = "";
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
    private EntryPipeline pipe;           // used only by the counter thread (one at a time: restart() waits for the old one to end)
    private volatile int restarts = 0;
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
        String cNote;
        boolean cOk;
        try {
            Method ls = BroadcomColorSensorImpl.class.getDeclaredMethod("setLSRateAndRes", BroadcomColorSensor.LSResolution.class, BroadcomColorSensor.LSMeasurementRate.class);
            ls.setAccessible(true);
            ls.invoke(sensor, BroadcomColorSensor.LSResolution.R16BIT, BroadcomColorSensor.LSMeasurementRate.R25ms);
            cNote = "color preset applied (16-bit, a new value every 25 ms)";
            cOk = true;
        } catch (Exception e) {
            cNote = "COLOR PRESET FAILED (" + e + "): the kind of an entry will be UNKNOWN";
            cOk = false;
        }
        this.colorNote = cNote;
        this.colorPresetOk = cOk;
    }

    /** The counting rule and its levels as they are set now, for the log's start line (2026-11-18: the line named the peak rule's settings while the edge rule was counting). */
    public static String ruleText() {
        return USE_EDGE
                ? String.format(Locale.US, "edge counting: an entry at +%.0f counts over the empty level for %d samples, the next one after the signal is back at +%.0f", EDGE_HI, EDGE_CONFIRM, EDGE_LO)
                : String.format(Locale.US, "peaks of at least %.0f counts, smoothing %d, a group with a peak of +%.0f or more within %.1f s counts once", PROMINENCE, SMOOTH, MERGE_HEIGHT, MERGE_GAP_SEC);
    }

    private static int u16(byte[] d, int i) {
        return (d[i] & 0xFF) | ((d[i + 1] & 0xFF) << 8);
    }

    /** The counter on the configuration's "CS5", or null when there is none (the robot runs without it). */
    public static EntryCounter tryCreate(HardwareMap hardwareMap) {
        RevColorSensorV3 s = hardwareMap.tryGet(RevColorSensorV3.class, "CS5");
        return s == null ? null : new EntryCounter(s);
    }

    /**
     * Called every loop by Mechanisms.update: starts the thread on the first call and keeps it alive. A thread that stopped because the loop itself stood still for IDLE_EXIT_SEC (a hub that
     * stopped answering for a moment) is started again here (2026-11-16: it used to stay stopped until the opmode ended, so one long loop ended the count and FULL for the rest of a match).
     */
    public void touch() {
        lastTouchNs = System.nanoTime();
        if (!ENABLED || disabled) return;
        if (!started) start();
        else if (!running) restart();
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
        pipe = USE_EDGE ? EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, BASELINE_SEC)
                : new EntryPipeline(PROMINENCE, SMOOTH, MIN_SEPARATION_SEC, MERGE_HEIGHT, MERGE_GAP_SEC, VALLEY_MAX, BASELINE_SEC);
        // the kinds need the edge rule: its entry is counted while the element is still in front of the sensor (a peak is counted after it has gone)
        classifier = USE_EDGE && colorPresetOk ? new EntryClassifier(CLASS_NEAR, CLASS_TAIL_SAMPLES, CLASS_MAX_SEC, CLASS_REF_SAMPLES,
                new EntryClassifier.Limits(CLASS_MIN_SIGNAL, CLASS_RED_MIN, CLASS_BLUE_MIN, CLASS_POLLEN_GREEN_MIN, CLASS_POLLEN_BLUE_MAX)) : null;
        thread = new Thread(this::run, "CS5 entry counter");
        thread.setDaemon(true);
        thread.start();
    }

    /**
     * Starts a new thread after the last one stopped by itself (the idle exit, or an interrupt). The pipeline and the time origin are kept, so the empty level measured at the start still holds
     * (a new baseline taken in the middle of a match could have an element under the sensor) and the entry numbers and times go on. Not after {@link #disable}.
     */
    private synchronized void restart() {
        if (!started || running || disabled) return;
        Thread old = thread;
        if (old != null && old.isAlive()) return;   // still on its way out: the next loop's touch starts the new one
        running = true;
        restarts++;
        thread = new Thread(this::run, "CS5 entry counter");
        thread.setDaemon(true);
        thread.start();
    }

    /** How many times the thread has been started again after stopping by itself (0 in a normal run). */
    public int restarts() {
        return restarts;
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
        final EntryPipeline pipe = this.pipe;   // made in start(), kept across a restart
        long next = System.nanoTime();
        final EntryClassifier classifier = this.classifier;
        long lastStart = 0L;
        int consecutiveErrors = 0;
        int colorFailsInARow = 0, idleReads = 0;
        while (running) {
            long t0 = System.nanoTime();
            if ((t0 - lastTouchNs) / 1e9 > IDLE_EXIT_SEC) break;   // the opmode is over
            try {
                // the color rides on the same transaction as the proximity (a burst in place of the 2-byte read): while an element is in front, through the first second (the empty color level), and
                // one read in IDLE_COLOR_EVERY with nothing in front
                boolean wantColor = classifier != null && CLASSIFY && !colorOff
                        && (classifier.wantsColor() || !pipe.baselineDone() || idleReads + 1 >= Math.max(1, IDLE_COLOR_EVERY));
                int v = -1, green = 0, blue = 0, red = 0;
                boolean color = false;
                if (wantColor) {
                    try {
                        byte[] d = sensor.read(BroadcomColorSensor.Register.MAIN_STATUS, BURST_BYTES);
                        if (d == null || d.length < BURST_BYTES) throw new IllegalStateException("a burst of " + (d == null ? 0 : d.length) + " bytes");
                        v = u16(d, 1) & 0x7FF;
                        green = u16(d, 6);
                        blue = u16(d, 9);
                        red = u16(d, 12);
                        color = (d[0] & 0x08) != 0;   // MAIN_STATUS bit 3: a new color value since the status was last read
                        colorFailsInARow = 0;
                        colorReads++;
                        idleReads = 0;
                    } catch (Throwable ce) {
                        colorErrors++;
                        if (++colorFailsInARow >= 3) {
                            colorOff = true;
                            colorFault = "three color reads in a row failed (" + ce + "): the color is off for this run, every kind is UNKNOWN";
                        }
                    }
                } else {
                    idleReads++;
                }
                if (v < 0) v = sensor.rawOptical();
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
                long counted = 0L;
                for (EntryGrouper.Entry g : pipe.push(tSec, v)) {
                    long n = entries + 1;
                    queue.add(new Entry(n, g.time, g.height, v, g.peaks, g.detail, originNs + (long) (g.time * 1e9)));
                    entries = n;
                    counted = n;
                }
                if (classifier != null && CLASSIFY && !colorOff) {
                    double above = pipe.baselineDone() && !Double.isInfinite(pipe.emptyLevel()) ? v - pipe.emptyLevel() : Double.NaN;
                    EntryClassifier.Result kind = classifier.push(tSec, above, counted, color, green, blue, red);
                    if (kind != null) kinds.add(kind);
                    if (color) colorReference = classifier.referenceText();
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

    /** The next classified entry (its number and kind), or null (drain this from the main loop). It comes after its entry: when the element has passed, at most CLASS_MAX_SEC later. */
    public EntryClassifier.Result pollKind() {
        return kinds.poll();
    }

    /** True while the thread reads the color and names the kinds: the switch is on, the edge rule is in use, the color preset worked and the color reads have not failed. */
    public boolean isClassifying() {
        return CLASSIFY && classifier != null && !colorOff;
    }

    /** Why the color was switched off in this run, or "". */
    public String colorFault() {
        return colorFault;
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
        return String.format(Locale.US, "%d samples, interval mean %.1f ms (asked %.0f), max %.0f ms, %d late; read %.1f ms mean (max %.0f), %.0f%% of the hub; %d entries, %d rejected (valley), empty level %.0f, %d errors; color: %s",
                n, mean, PERIOD_MS, maxMs, over, readMs / n, readMax, 100.0 * readMs / Math.max(sumMs, 1e-9), entries, rejected, baseline, errors,
                classifier == null ? "not read" : String.format(Locale.US, "%d reads in all, %d failed, empty level %s%s", colorReads, colorErrors, colorReference.isEmpty() ? "-" : colorReference,
                        colorOff ? ", OFF" : CLASSIFY ? "" : ", switched off"));
    }
}
