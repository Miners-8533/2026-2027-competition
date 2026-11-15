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

    /** EDGE MODE (2026-11-15, the robot's mode): a seconds figure for the stuck-high note. */
    public static final double STUCK_SEC = 10.0;
    private final boolean edgeMode;
    private final double edgeHi, edgeLo;
    private final int edgeConfirm;
    private boolean edgeArmed = true;
    private int edgeAbove = 0;
    private double edgeFirstAboveT = 0.0, edgeStuckSince = Double.NaN;
    private boolean stuckNoted = false;

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

    /**
     * EDGE MODE (2026-11-15, the user's idea, replaces the peaks on the robot): count the RISE of the proximity above the empty level + `hi` (held for `confirm` samples), at the time of the rise, and count
     * again only after the signal has come back down to the empty level + `lo`. There is no fall to detect for the count, so an element that stays under the sensor, wiggling or ringing, is ONE entry (all its readings
     * are above the threshold), at the price of counting a clump that never dips back as one. On the recorded runs (hi 6, lo 2, confirm 2): the 31-element study 30 of 31 matched with 1 extra, the 14:16 TeleOp exactly 4, 4 and 4,
     * the 15:22 TeleOp 25 entries where the peaks gave 37 (the counter held 25 in all). A signal that stays above lo for STUCK_SEC puts a note out (a parked element, or the empty level moved) and no more entries come until it drops.
     */
    public static EntryPipeline edge(double hi, double lo, int confirm, double baselineSec) {
        return new EntryPipeline(true, hi, lo, confirm, baselineSec);
    }

    private EntryPipeline(boolean edge, double hi, double lo, int confirm, double baselineSec) {
        this.edgeMode = edge;
        this.edgeHi = hi;
        this.edgeLo = lo;
        this.edgeConfirm = Math.max(1, confirm);
        this.peaks = new PeakCounter(6.0, 1, 0.06);
        this.grouper = new EntryGrouper(45.0, 0.4);
        this.valleyMax = 100000.0;
        this.baselineSec = baselineSec;
    }

    public EntryPipeline(double prominence, int smooth, double minSepSec, double mergeHeight, double mergeGapSec, double valleyMax, double baselineSec) {
        this.edgeMode = false;
        this.edgeHi = 0.0;
        this.edgeLo = 0.0;
        this.edgeConfirm = 1;
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
        if (edgeMode) return pushEdge(t, v);
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

    private List<EntryGrouper.Entry> pushEdge(double t, double v) {
        List<EntryGrouper.Entry> out = new ArrayList<>();
        if (!baselineDone || Double.isInfinite(empty)) return out;
        double d = v - empty;
        if (edgeArmed) {
            if (d >= edgeHi) {
                if (edgeAbove == 0) edgeFirstAboveT = t;
                edgeAbove++;
                if (edgeAbove >= edgeConfirm) {
                    out.add(new EntryGrouper.Entry(edgeFirstAboveT, d, 1, ""));
                    edgeArmed = false;
                    edgeAbove = 0;
                    edgeStuckSince = Double.NaN;
                }
            } else {
                edgeAbove = 0;
            }
        } else if (d <= edgeLo) {
            edgeArmed = true;
            edgeAbove = 0;
            edgeStuckSince = Double.NaN;
            if (stuckNoted) {
                stuckNoted = false;
                note = "the proximity is back near the empty level, counting again";
            }
        } else if (Double.isNaN(edgeStuckSince)) {
            edgeStuckSince = t;
        } else if (!stuckNoted && t - edgeStuckSince >= STUCK_SEC) {
            stuckNoted = true;
            note = String.format(Locale.US, "the proximity has read above the empty level (+%.0f now) for %.0f s: an element parked under the sensor, or the empty level moved; no more entries until it is back at +%.0f", d, STUCK_SEC, edgeLo);
        }
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
        // the empty level is the UPPER end of the empty readings (the 90th percentile of the first second), not the median: the empty proximity flickers between two neighbouring counts (65 and 66 in the study, 66 and 67 on
        // other days), the thresholds are counts above it, and with the median a 1-count difference in which of the two it landed on moved the study from 31 entries to 28 (edge mode, hi 6, lo 2)
        empty = s[Math.min(n - 1, (int) Math.ceil(0.9 * n) - 1)];
        noise = Math.max(1.4826 * mad, 0.5);
        note = String.format(Locale.US, "empty proximity %.0f (the median of the first second %.0f), noise %.2f counts (%d samples in the first %.1f s; an element in front of the sensor then would read as the empty level)", empty, med, noise, n, baselineSec);
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
