package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.Locale;
import java.util.concurrent.locks.LockSupport;

/**
 * RECORDS CS2, the color sensor at the top of the feed ramp (2026-11-18, not run on the robot). It counts nothing yet: the samples go to the log (the cs2f column and one line per salvo), so
 * that a rule for counting the elements a salvo launches can be made from what the sensor really reads there (specs/element-counting.md, target T11).
 *
 * CS2 was the second place along the track until the team moved it to the top of the feed ramp, past the HOLD position, to count launches. It keeps its hub, bus and name. The slot count
 * ({@link ElementCounter}) no longer reads it: the fast proximity setting used here (8-bit, a new value every 6.25 ms, as CS5 has) leaves no usable distance.
 *
 * A thread reads CS2's raw proximity (one 2-byte I2C read):
 * - every PERIOD_MS while a launch is queued or a salvo runs, and for TAIL_SEC after it (the FAST stretch);
 * - every IDLE_PERIOD_MS otherwise, which gives the empty level and shows anything that stays in front of the sensor between salvos.
 * The fast reads take hub time from the main loop DURING A SALVO, when the loop time sets how exactly the pusher stops: the first run has to show what PERIOD_MS costs (the log's loop_ms
 * through a salvo with ENABLED true and false). Nothing is known yet of how long a launched element is in view, so PERIOD_MS is a first value.
 *
 * The thread starts with the first {@link #touch} (Mechanisms.update calls it every loop, so nothing is read in INIT) and ends when touch stops or on {@link #stop()}. The main loop drains
 * {@link #drainSamples()} for the log and {@link #takeSalvoSummary()} after each fast stretch; the thread never writes the log.
 */
@Config
public final class LaunchSensor {

    /** False: the thread reads nothing (CS2 is then not read by anything). Read live. */
    public static boolean ENABLED = true;
    /**
     * THE CS2 SWITCH (2026-11-20, user), OFF by default: false = the robot runs as if CS2 were not there, WITHOUT changing the hub configuration (the sensor may stay configured, plugged in or not): tryCreate
     * returns null, so nothing talks to the sensor (no preset write, no thread, no reads) and the CS2 columns and the per-salvo launch summary are empty. CS2 is only RECORDED (nothing counts or acts on it), so
     * nothing else changes. Set it true to record the launches again (the pusher check is off and the entry count does not use it). Read once, at the start of an opmode.
     */
    public static boolean CS2_PRESENT = false;
    /**
     * Time between the starts of two reads in a fast stretch, ms. 25 since 2026-11-19 (was 12.5, which held the hub 73 to 80 % of every salvo: ten ms per read, the main loop waited behind it). The 324 salvos
     * recorded at 12.5 ms, thinned to 25 ms, give the same launch count as at 12.5 in 323 with the peak rule rise 5 / prominence 3 (a launch peak is 25 to 50 ms wide); 37.5 ms fails in a third. Read live.
     */
    public static double PERIOD_MS = 25.0;
    /** Time between two reads outside a fast stretch, ms. Read live. */
    public static double IDLE_PERIOD_MS = 250.0;
    /**
     * The fast stretch goes on this long after the salvo is over. 0.1 since 2026-11-19 (was 0.6): in the 324 recorded salvos the last launch peak came 0.27 to 0.50 s BEFORE "SALVO done" (the arm
     * is still turning, the elements have left). Read live.
     */
    public static double TAIL_SEC = 0.1;
    /** The thread ends when touch() has not been called for this long (the opmode ended); a touch() after that starts it again. */
    public static double IDLE_EXIT_SEC = 1.5;
    /** The empty level is the median of this many of the latest idle samples. */
    private static final int IDLE_KEEP = 9;

    private final RevColorSensorV3 sensor;
    public final String presetNote;
    public final boolean presetOk;

    // the raw samples for the log (drained by Mechanisms.sampleRow): the nanoTime and the proximity of each
    private static final int RING = 2048;
    private final long[] ringT = new long[RING];
    private final int[] ringV = new int[RING];
    private volatile long ringHead = 0L;
    private long ringTail = 0L;

    private volatile long lastTouchNs = 0L;
    private volatile long fastUntilNs = 0L;
    private volatile boolean wanted = false;
    private volatile boolean running = false;
    private volatile boolean started = false;
    private volatile int lastProx = -1;
    private volatile long lastSampleNs = 0L;
    private volatile long errors = 0L;
    private volatile double emptyLevel = Double.NaN;
    private volatile String summary = null;
    private Thread thread;

    private LaunchSensor(RevColorSensorV3 sensor) {
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
            note = "PROXIMITY PRESET FAILED (" + e + "): the sensor refreshes every 100 ms, too slow to follow a salvo";
            ok = false;
        }
        this.presetNote = note;
        this.presetOk = ok;
    }

    /** The reader on the configuration's "CS2", or null when there is none (the robot runs without it). */
    public static LaunchSensor tryCreate(HardwareMap hardwareMap) {
        if (!CS2_PRESENT) {
            org.firstinspires.ftc.teamcode.util.RunLog.event("CS2 SWITCHED OFF (LaunchSensor.CS2_PRESENT is false): no launch sensor, nothing talks to the sensor; the CS2 columns and the launch summary are empty");
            return null;
        }
        RevColorSensorV3 s = hardwareMap.tryGet(RevColorSensorV3.class, "CS2");
        return s == null ? null : new LaunchSensor(s);
    }

    /**
     * Called every loop by Mechanisms.update: starts the thread on the first call and keeps it alive.
     *
     * @param launching a launch is queued or a salvo is running: the thread reads fast from now until TAIL_SEC after this was last true
     */
    public void touch(boolean launching) {
        long now = System.nanoTime();
        lastTouchNs = now;
        if (launching) fastUntilNs = now + (long) (TAIL_SEC * 1e9);
        if (!ENABLED) return;
        if (!started || !running) start();
        if (launching && !wanted) {
            Thread t = thread;
            if (t != null) LockSupport.unpark(t);   // out of the idle wait at once: the first element is on its way
        }
        wanted = launching;
    }

    private synchronized void start() {
        if (running) return;
        Thread old = thread;
        if (old != null && old.isAlive()) return;   // still on its way out: the next loop's touch starts the new one
        started = true;
        running = true;
        thread = new Thread(this::run, "CS2 launch sensor");
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        running = false;
        Thread t = thread;
        if (t != null && t != Thread.currentThread()) {
            LockSupport.unpark(t);
            try {
                t.join(300);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private void run() {
        final int[] idle = new int[IDLE_KEEP];
        int idleN = 0;
        boolean wasFast = false;
        long fastStartNs = 0L, lastStart = 0L;
        int n = 0, min = Integer.MAX_VALUE, max = Integer.MIN_VALUE, late = 0;
        double sumDtMs = 0.0, maxDtMs = 0.0, sumReadMs = 0.0, maxReadMs = 0.0;
        long errorsAtStart = 0L;
        int consecutiveErrors = 0;
        long next = System.nanoTime();
        while (running) {
            long t0 = System.nanoTime();
            if ((t0 - lastTouchNs) / 1e9 > IDLE_EXIT_SEC || !ENABLED) break;   // the opmode is over, or the switch went off
            boolean fast = t0 < fastUntilNs;
            if (fast && !wasFast) {
                fastStartNs = t0;
                lastStart = 0L;
                n = 0;
                late = 0;
                min = Integer.MAX_VALUE;
                max = Integer.MIN_VALUE;
                sumDtMs = maxDtMs = sumReadMs = maxReadMs = 0.0;
                errorsAtStart = errors;
            }
            if (!fast && wasFast) {
                double level = emptyLevel;
                summary = n == 0 ? String.format(Locale.US, "no samples in %.2f s (%d read errors)", (t0 - fastStartNs) / 1e9, errors - errorsAtStart)
                        : String.format(Locale.US, "%d samples in %.2f s, interval mean %.1f ms (asked %.1f), max %.0f ms, %d late; read %.1f ms mean (max %.0f), %.0f%% of the hub; proximity %d to %d, empty level %s, %d read errors",
                        n, (t0 - fastStartNs) / 1e9, n > 1 ? sumDtMs / (n - 1) : 0.0, PERIOD_MS, maxDtMs, late, sumReadMs / n, maxReadMs,
                        100.0 * sumReadMs / Math.max((t0 - fastStartNs) / 1e6, 1e-9), min, max, Double.isNaN(level) ? "not known yet" : String.format(Locale.US, "%.0f", level), errors - errorsAtStart);
            }
            wasFast = fast;
            try {
                int v = sensor.rawOptical();
                long t1 = System.nanoTime();
                consecutiveErrors = 0;
                lastProx = v;
                lastSampleNs = t1;
                int ri = (int) (ringHead % RING);
                ringT[ri] = t0;
                ringV[ri] = v;
                ringHead = ringHead + 1;
                if (fast) {
                    double readMs = (t1 - t0) / 1e6;
                    n++;
                    min = Math.min(min, v);
                    max = Math.max(max, v);
                    sumReadMs += readMs;
                    maxReadMs = Math.max(maxReadMs, readMs);
                    if (lastStart != 0L) {
                        double dtMs = (t0 - lastStart) / 1e6;
                        sumDtMs += dtMs;
                        maxDtMs = Math.max(maxDtMs, dtMs);
                        if (dtMs > 2.0 * Math.max(PERIOD_MS, 5.0)) late++;
                    }
                    lastStart = t0;
                } else {
                    idle[idleN % IDLE_KEEP] = v;
                    idleN++;
                    emptyLevel = median(idle, Math.min(idleN, IDLE_KEEP));
                }
            } catch (Throwable e) {
                errors++;
                consecutiveErrors++;
                try {
                    Thread.sleep(consecutiveErrors > 20 ? 200 : 30);
                } catch (InterruptedException ie) {
                    break;
                }
            }
            double period = fast ? PERIOD_MS : IDLE_PERIOD_MS;
            next += (long) (Math.max(period, 0.0) * 1e6);
            long now = System.nanoTime();
            if (next <= now) {
                next = now;   // late, or reading back to back: no catch-up burst
            } else if (fast) {
                LockSupport.parkNanos(next - now);
            } else {
                // idle: wait in short steps so that a launch (touch unparks, and fastUntilNs moves) is followed at once
                while (running && System.nanoTime() < next && System.nanoTime() >= fastUntilNs) LockSupport.parkNanos(Math.min(next - System.nanoTime(), 20_000_000L));
                if (System.nanoTime() < fastUntilNs) next = System.nanoTime();
            }
        }
        running = false;
    }

    private static double median(int[] values, int count) {
        int[] c = java.util.Arrays.copyOf(values, count);
        java.util.Arrays.sort(c);
        return count == 0 ? Double.NaN : count % 2 == 1 ? c[count / 2] : 0.5 * (c[count / 2 - 1] + c[count / 2]);
    }

    public boolean isStarted() {
        return started;
    }

    /** True when the thread has produced a sample in the last second (the idle reads are IDLE_PERIOD_MS apart). */
    public boolean isHealthy() {
        return started && running && (System.nanoTime() - lastSampleNs) / 1e9 < 1.0;
    }

    public int latestProx() {
        return lastProx;
    }

    /** The median of the latest idle samples: the reading with nothing in front of the sensor, as long as nothing stays there between salvos. NaN before the first idle sample. */
    public double emptyLevel() {
        return emptyLevel;
    }

    public long errors() {
        return errors;
    }

    /** The statistics of the fast stretch that has just ended, once, or null. Called from the main loop only. */
    public String takeSalvoSummary() {
        String text = summary;
        if (text != null) summary = null;
        return text;
    }

    /**
     * The raw samples since the last call as "prox@-ms" separated by spaces, ms BEFORE this call (the log row's own time minus the number is the sample's time), for the log's cs2f column:
     * the form of CS5's cs5f. "" when there are none. Called from the main loop only (Mechanisms.sampleRow).
     */
    public String drainSamples() {
        long h = ringHead;
        if (h - ringTail > RING) ringTail = h - RING;
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
}
