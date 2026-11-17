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
 * Some of those reads are a longer read that also returns the color (section 2.8): status, proximity and the infrared,
 * green, blue and red channels in one transaction. It replaces the small read, it is not added to it, and the count
 * gets the same proximity value from either. {@link EntryKinds} says which read is next and names each entry's kind.
 *
 * The thread never writes the log. The main loop collects finished entries, their kinds and the raw samples from it.
 */
public final class EntryCounter {
    public static final double STOPS_AFTER_S = 1.5, NOT_ANSWERING_S = 0.5, STATS_EVERY_S = 15.0;

    private final RevColorSensorV3 sensor;
    private final boolean fastMode;
    private final EntryRule rule = new EntryRule();
    private final EntryKinds kinds = new EntryKinds();
    private final ConcurrentLinkedQueue<EntryRule.Entry> entries = new ConcurrentLinkedQueue<>();
    private final ConcurrentLinkedQueue<EntryKinds.Result> kindResults = new ConcurrentLinkedQueue<>();
    /** The longer read: MAIN_STATUS, PS_DATA (2 bytes), then infrared, green, blue and red (3 bytes each). */
    private static final int COLOR_READ_BYTES = 15;
    /** MAIN_STATUS bit 3: a new color value since the last read. */
    private static final int STATUS_NEW_COLOR = 0x08;
    private static final int COLOR_FAILS_GIVE_UP = 3;
    private volatile boolean colorOff;
    private volatile String kindNote;
    private volatile long colorReads;
    private int colorFails;
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
        try {
            // The color: a new 16-bit value every 25 ms (resolution in bits 6 to 4, rate in bits 2 to 0).
            sensor.write8(BroadcomColorSensor.Register.LS_MEAS_RATE,
                    (BroadcomColorSensor.LSResolution.R16BIT.bVal << 4) | BroadcomColorSensor.LSMeasurementRate.R25ms.bVal);
        } catch (Throwable e) {
            colorOff = true;
            kindNote = "the color setting failed: the proximity is read alone and every kind is unknown for this run";
        }
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

    /** The kinds of the entries whose passes have closed since the last call. */
    public List<EntryKinds.Result> collectKinds() {
        List<EntryKinds.Result> out = new ArrayList<>();
        EntryKinds.Result r;
        while ((r = kindResults.poll()) != null) out.add(r);
        return out;
    }

    /** Once, when the color reads were given up for the run; else null. */
    public String takeKindNote() {
        String n = kindNote;
        kindNote = null;
        return n;
    }

    /** True once the color reads were given up for the run. */
    public boolean colorOff() { return colorOff; }

    /** A statistics line every 15 s, or null. */
    public String stats(double t) {
        if (t - lastStats < STATS_EVERY_S) return null;
        lastStats = t;
        String s = String.format(Locale.US, "%d samples (%d with the color), %d read errors, longest gap %.0f ms, empty level %.0f, latest %d, %s",
                samples, colorReads, errors, maxPeriod * 1000, level, latest, healthy() ? "healthy" : "NOT ANSWERING");
        maxPeriod = 0;
        return s;
    }

    /** Three bytes, low byte first. */
    private static int u24(byte[] b, int at) {
        return (b[at] & 0xFF) | ((b[at + 1] & 0xFF) << 8) | ((b[at + 2] & 0xFF) << 16);
    }

    private void sampleLoop() {
        double last = Clock.now();
        try {
            while (Clock.now() - lastCollect < STOPS_AFTER_S) {
                long period = Math.max(5, (long) EntryRule.SAMPLE_PERIOD_MS);
                double t = Clock.now();
                boolean withColor = !colorOff && kinds.wantColor(t);
                try {
                    int prox;
                    int[] color = null;
                    if (withColor) {
                        byte[] b = sensor.read(BroadcomColorSensor.Register.MAIN_STATUS, COLOR_READ_BYTES);
                        prox = (b[1] & 0xFF) | ((b[2] & 0x07) << 8);
                        // A color value is used only when the sensor reports it as new since the last color read.
                        if ((b[0] & STATUS_NEW_COLOR) != 0) color = new int[] {u24(b, 6), u24(b, 9), u24(b, 12)};
                        colorFails = 0;
                        colorReads++;
                    } else {
                        byte[] b = sensor.read(BroadcomColorSensor.Register.PS_DATA, 2);
                        prox = (b[0] & 0xFF) | ((b[1] & 0x07) << 8);
                    }
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
                    if (!colorOff && EntryKinds.ENABLED && EntryRule.EDGE_COUNTING) {
                        EntryKinds.Result k = kinds.sample(t, prox, rule.level(), e != null, color);
                        if (k != null) kindResults.add(k);
                    }
                } catch (Throwable e) {
                    errors++;
                    if (withColor && ++colorFails >= COLOR_FAILS_GIVE_UP) {
                        // The count is not affected: from here the proximity is read alone.
                        colorOff = true;
                        kindNote = COLOR_FAILS_GIVE_UP + " color reads in a row failed: the proximity is read alone and every kind is unknown for the rest of this run";
                    }
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
