package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * The whole CS5 counting pipeline in one pure class (no FTC classes; EntryCounter's thread feeds it and tools/entry/PeakCounterCheck.java replays recorded runs through THE SAME code):
 * the empty-level baseline, {@link PeakCounter} (a peak of at least `prominence` counts), the VALLEY rule, and {@link EntryGrouper} (a tall burst counts once).
 *
 * THE VALLEY RULE (2026-11-14, from the 14:16 TeleOp): an element passing the sensor is a bump that comes back DOWN TO THE EMPTY LEVEL before the next one: in the 14:16 run's 16 real entries the
 * valley before the peak was between 2 counts below and 2 above the empty level, and in the hand-fed study run (31 elements) at most 3 above. Once the driver had picked up too many elements and stopped with
 * them parked in front of the sensor, the proximity sat 6 to 7 counts above empty with wiggles of +6 to +15 on top (the intake jostling them): the 16 peaks there had valleys 4 to 8 above empty and 14 of them
 * were counted as elements. A peak whose valley is more than `valleyMax` counts above the empty level is REJECTED (counted in {@link #rejected()}, not offered to the grouper). The empty level is the median of
 * the first `baselineSec` (keep the intake clear then) and can only be LOWERED afterwards: when the last 2 s were quiet (a range of at most 4 counts) and their median is more than 1 count under the level (so a
 * high start is corrected, and an object parked in front can never raise it).
 */
public final class EntryPipeline {

    private final PeakCounter peaks;
    private final EntryGrouper grouper;
    private final double valleyMax, baselineSec;
    private final ArrayList<double[]> baseline = new ArrayList<>();
    private boolean baselineDone = false;
    private double empty = Double.NaN, noise = Double.NaN, t0 = Double.NaN;
    private long rejected = 0L;
    private final ArrayDeque<double[]> window = new ArrayDeque<>();   // {t, v} for the last 2 s
    private double nextCheckT = Double.NaN;
    private String note = null;

    public EntryPipeline(double prominence, int smooth, double minSepSec, double mergeHeight, double mergeGapSec, double valleyMax, double baselineSec) {
        this.peaks = new PeakCounter(prominence, smooth, minSepSec);
        this.grouper = new EntryGrouper(mergeHeight, mergeGapSec);
        this.valleyMax = valleyMax;
        this.baselineSec = baselineSec;
    }

    /** One sample; returns the entries that have just closed (usually none). */
    public List<EntryGrouper.Entry> push(double t, double v) {
        if (Double.isNaN(t0)) {
            t0 = t;
            nextCheckT = t + 1.0;
        }
        if (!baselineDone) {
            if (t - t0 <= baselineSec) baseline.add(new double[]{v});
            else finishBaseline();
        } else {
            adapt(t, v);
        }
        List<EntryGrouper.Entry> out = new ArrayList<>();
        double h = peaks.push(t, v);
        if (!Double.isNaN(h)) {
            double valleyAboveEmpty = peaks.lastValley() - empty;
            if (baselineDone && valleyAboveEmpty > valleyMax) {
                rejected++;
            } else {
                out.addAll(grouper.offer(peaks.lastPeakTime(), h));
            }
        }
        out.addAll(grouper.poll(t));
        return out;
    }

    /** Closes whatever is pending (the end of a run). */
    public List<EntryGrouper.Entry> flush() {
        return grouper.flush();
    }

    private void finishBaseline() {
        int n = baseline.size();
        baselineDone = true;
        if (n < 5) {
            empty = Double.POSITIVE_INFINITY;   // nothing known: the valley rule cannot reject
            note = "baseline: only " + n + " samples, the valley rule is off until the empty level is found";
            return;
        }
        double[] s = new double[n];
        for (int i = 0; i < n; i++) s[i] = baseline.get(i)[0];
        Arrays.sort(s);
        double med = n % 2 == 1 ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
        double[] dev = new double[n];
        for (int i = 0; i < n; i++) dev[i] = Math.abs(s[i] - med);
        Arrays.sort(dev);
        double mad = n % 2 == 1 ? dev[n / 2] : 0.5 * (dev[n / 2 - 1] + dev[n / 2]);
        empty = med;
        noise = Math.max(1.4826 * mad, 0.5);
        note = String.format(Locale.US, "empty proximity %.0f, noise %.2f counts (%d samples in the first %.1f s; an element in front of the sensor then would read as the empty level; a peak whose valley is more than +%.0f above it is rejected)", med, noise, n, baselineSec, valleyMax);
    }

    /** Lowers the empty level when the last 2 s were quiet and clearly lower (never raises it). */
    private void adapt(double t, double v) {
        window.addLast(new double[]{t, v});
        while (!window.isEmpty() && t - window.peekFirst()[0] > 2.0) window.removeFirst();
        if (t < nextCheckT) return;
        nextCheckT = t + 1.0;
        if (window.size() < 20 || t - window.peekFirst()[0] < 1.8) return;
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        double[] vals = new double[window.size()];
        int i = 0;
        for (double[] w : window) {
            lo = Math.min(lo, w[1]);
            hi = Math.max(hi, w[1]);
            vals[i++] = w[1];
        }
        if (hi - lo > 4.0) return;
        Arrays.sort(vals);
        double med = vals.length % 2 == 1 ? vals[vals.length / 2] : 0.5 * (vals[vals.length / 2 - 1] + vals[vals.length / 2]);
        if (med < empty - 1.0) {
            note = String.format(Locale.US, "empty level lowered from %.0f to %.0f (a quiet 2 s)", empty, med);
            empty = med;
        }
    }

    /** A note for the log (the baseline once measured, or a change of the empty level); null when there is none. Cleared by reading it. */
    public String takeNote() {
        String n = note;
        note = null;
        return n;
    }

    public long rejected() {
        return rejected;
    }

    public double emptyLevel() {
        return empty;
    }

    public double noise() {
        return noise;
    }

    public boolean baselineDone() {
        return baselineDone;
    }
}
