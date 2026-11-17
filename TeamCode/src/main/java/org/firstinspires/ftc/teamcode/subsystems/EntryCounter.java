package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.Clock;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * The entry counter's sampling (specs/element-counting.md section 2.1): a separate thread reads CS5's raw proximity
 * (one small read) every 25 ms and runs the counting rule on each sample. It starts with the first collect(), so
 * nothing is read during INIT, and stops by itself when collects stop for 1.5 s.
 *
 * The thread never writes the log. The main loop collects finished entries and the raw samples from it.
 */
public final class EntryCounter {
    public static final double STOPS_AFTER_S = 1.5, NOT_ANSWERING_S = 0.5, STATS_EVERY_S = 15.0;

    private final RevColorSensorV3 sensor;
    private final boolean fastMode;
    private final EntryRule rule = new EntryRule();
    private final ConcurrentLinkedQueue<EntryRule.Entry> entries = new ConcurrentLinkedQueue<>();
    private final ConcurrentLinkedQueue<Integer> raw = new ConcurrentLinkedQueue<>();

    private volatile Thread thread;
    private volatile double lastCollect, lastSampleAt = -1e9, level = Double.NaN, maxPeriod;
    private volatile int latest = -1;
    private volatile long samples, errors;
    private volatile String note;
    private double lastStats;
    private boolean started;

    private EntryCounter(RevColorSensorV3 sensor) {
        this.sensor = sensor;
        boolean ok;
        try {
            // Fast proximity mode: 8-bit, a new value every 6.25 ms (the standard setting gives one every 100 ms, too slow).
            sensor.write8(BroadcomColorSensor.Register.PS_MEAS_RATE,
                    BroadcomColorSensor.PSResolution.R8BIT.bitOr(BroadcomColorSensor.PSMeasurementRate.R6_25ms.bVal));
            ok = true;
        } catch (Throwable e) {
            ok = false;
        }
        fastMode = ok;
    }

    /** Null when CS5 is not configured: no entry count, and the robot never declares itself full. */
    public static EntryCounter create(HardwareMap hw) {
        RevColorSensorV3 s = hw.tryGet(RevColorSensorV3.class, ChassisProfile.ENTRY_SENSOR);
        return s == null ? null : new EntryCounter(s);
    }

    /** False if the sensor could not be put into its fast proximity mode: the count is then useless. */
    public boolean fastMode() { return fastMode; }

    /** Healthy while samples keep arriving. */
    public boolean healthy() { return started && Clock.now() - lastSampleAt <= NOT_ANSWERING_S; }

    public boolean started() { return started; }
    public int latest() { return latest; }
    public double level() { return level; }

    /** Called from the main loop every update: starts the thread if needed and returns the entries finished since the last call. */
    public List<EntryRule.Entry> collect() {
        lastCollect = Clock.now();
        if (thread == null) {
            started = true;
            lastSampleAt = lastCollect;     // give the first sample time to arrive
            Thread th = new Thread(new Runnable() {
                @Override public void run() { sampleLoop(); }
            }, "CS5 entry counter");
            th.setDaemon(true);
            thread = th;
            th.start();
        }
        List<EntryRule.Entry> out = new ArrayList<>();
        EntryRule.Entry e;
        while ((e = entries.poll()) != null) out.add(e);
        return out;
    }

    /** The raw samples since the last call, for the log's sample row. */
    public String takeRaw() {
        StringBuilder sb = new StringBuilder();
        Integer v;
        while ((v = raw.poll()) != null) {
            if (sb.length() > 0) sb.append(' ');
            sb.append(v);
        }
        return sb.toString();
    }

    public String takeNote() {
        String n = note;
        note = null;
        return n;
    }

    /** A statistics line every 15 s, or null. */
    public String stats(double t) {
        if (t - lastStats < STATS_EVERY_S) return null;
        lastStats = t;
        String s = String.format(Locale.US, "%d samples, %d read errors, longest gap %.0f ms, empty level %.0f, latest %d, %s",
                samples, errors, maxPeriod * 1000, level, latest, healthy() ? "healthy" : "NOT ANSWERING");
        maxPeriod = 0;
        return s;
    }

    private void sampleLoop() {
        double last = Clock.now();
        try {
            while (Clock.now() - lastCollect < STOPS_AFTER_S) {
                long period = Math.max(5, (long) EntryRule.SAMPLE_PERIOD_MS);
                double t = Clock.now();
                try {
                    byte[] b = sensor.read(BroadcomColorSensor.Register.PS_DATA, 2);
                    int prox = (b[0] & 0xFF) | ((b[1] & 0x07) << 8);
                    samples++;
                    latest = prox;
                    lastSampleAt = t;
                    maxPeriod = Math.max(maxPeriod, t - last);
                    last = t;
                    raw.add(prox);
                    EntryRule.Entry e = rule.sample(t, prox);
                    level = rule.level();
                    if (e != null) entries.add(e);
                    String n = rule.takeNote();
                    if (n != null) note = n;
                } catch (Throwable e) {
                    errors++;
                }
                long sleep = period - (long) ((Clock.now() - t) * 1000);
                if (sleep > 0) Thread.sleep(sleep);
            }
        } catch (InterruptedException ignored) {
        } finally {
            thread = null;
        }
    }
}
