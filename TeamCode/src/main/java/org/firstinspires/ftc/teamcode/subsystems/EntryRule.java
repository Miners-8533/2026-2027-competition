package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;

/**
 * The entry counting rule (specs/element-counting.md sections 2.2 to 2.4). One pure piece of logic with no robot
 * dependencies, so recorded proximity traces can be replayed through exactly the code the robot runs.
 *
 * Edge counting is the rule in use: an entry is the rise of the signal. Peak counting, the earlier rule, is kept
 * behind a switch until edge counting has run in a match.
 */
@Config
public final class EntryRule {
    public static double SAMPLE_PERIOD_MS = 25;
    public static double EMPTY_WINDOW_S = 1.0;
    public static int RISE_COUNTS = 6, REARM_COUNTS = 2, CONFIRM_SAMPLES = 2;
    public static boolean EDGE_COUNTING = true;
    public static double LOWER_WINDOW_S = 2.0;
    public static int LOWER_QUIET_RANGE = 4;
    public static double PARKED_NOTE_S = 10.0;
    public static int PEAK_PROMINENCE = 6;
    public static double PEAK_MIN_SEPARATION_S = 0.06;
    public static int PEAK_MERGE_HEIGHT = 45;
    public static double PEAK_MERGE_GAP_S = 0.4;
    /** A peak whose preceding valley is more than this above the empty level is rejected. 0 = off. */
    public static int PEAK_VALLEY_LIMIT = 0;

    /** One counted entry. */
    public static final class Entry {
        public final double t;
        /** Counts above the empty level. */
        public final int height;
        public final int proximity;

        Entry(double t, int height, int proximity) {
            this.t = t;
            this.height = height;
            this.proximity = proximity;
        }
    }

    private double startT = Double.NaN;
    private int[] first = new int[128];
    private int firstN;
    private double level = Double.NaN;

    // Recent samples for lowering the level.
    private final int[] recent = new int[256];
    private final double[] recentT = new double[256];
    private int recentN, recentAt;

    // Edge rule.
    private boolean armed = true;
    private int above;
    private double disarmedAt = -1;
    private boolean parkedNoted;
    private String note;

    // Peak rule.
    private int valley = Integer.MAX_VALUE, peak = Integer.MIN_VALUE;
    private double peakT, lastPeakT = -1e9;
    private boolean rising = true, groupBig;
    private int groupCounted;

    /** The empty level, or NaN during the first second. */
    public double level() { return level; }

    /** A note for the log, once: an element is parked under the sensor, or the empty level has moved. */
    public String takeNote() {
        String n = note;
        note = null;
        return n;
    }

    /** Feeds one sample. Returns the entry it completes, or null. */
    public Entry sample(double t, int prox) {
        if (Double.isNaN(startT)) startT = t;
        if (Double.isNaN(level)) {
            // The empty level is the 90th percentile of the first second: the empty reading flickers between two
            // neighbouring counts and the percentile lands on the upper one. The intake must be clear for that second.
            if (firstN < first.length) first[firstN++] = prox;
            if (t - startT < EMPTY_WINDOW_S) return null;
            int[] s = Arrays.copyOf(first, firstN);
            Arrays.sort(s);
            level = s[Math.min(firstN - 1, (int) Math.floor(0.9 * (firstN - 1) + 0.5))];
            first = null;
            return null;
        }
        lowerLevel(t, prox);
        return EDGE_COUNTING ? edge(t, prox) : peak(t, prox);
    }

    /** Afterwards the level can only be lowered: an object parked in front of the sensor can never raise it. */
    private void lowerLevel(double t, int prox) {
        recent[recentAt] = prox;
        recentT[recentAt] = t;
        recentAt = (recentAt + 1) % recent.length;
        recentN = Math.min(recent.length, recentN + 1);
        int n = 0, min = Integer.MAX_VALUE, max = Integer.MIN_VALUE;
        int[] w = new int[recentN];
        double oldest = t;
        for (int i = 0; i < recentN; i++) {
            if (t - recentT[i] > LOWER_WINDOW_S) continue;
            w[n++] = recent[i];
            min = Math.min(min, recent[i]);
            max = Math.max(max, recent[i]);
            oldest = Math.min(oldest, recentT[i]);
        }
        if (n < 4 || t - oldest < LOWER_WINDOW_S * 0.9 || max - min > LOWER_QUIET_RANGE) return;
        Arrays.sort(w, 0, n);
        double median = w[n / 2];
        if (median < level - 1) level = median;
    }

    private Entry edge(double t, int prox) {
        if (armed) {
            // The signal reaches the rise threshold and stays there for some samples in a row: one entry, counted at that moment.
            if (prox >= level + RISE_COUNTS) {
                if (++above >= CONFIRM_SAMPLES) {
                    armed = false;
                    above = 0;
                    disarmedAt = t;
                    parkedNoted = false;
                    return new Entry(t, (int) Math.round(prox - level), prox);
                }
            } else {
                above = 0;
            }
        } else if (prox <= level + REARM_COUNTS) {
            // No further entry is possible until the signal has come back down.
            armed = true;
        } else if (!parkedNoted && t - disarmedAt >= PARKED_NOTE_S) {
            parkedNoted = true;
            note = "the signal has stayed above the re-arm level for " + PARKED_NOTE_S + " s: an element is parked at CS5, or the empty level has moved";
        }
        return null;
    }

    /** The earlier rule: an entry is a local peak that stands out from its surroundings. */
    private Entry peak(double t, int prox) {
        Entry out = null;
        if (rising) {
            if (prox > peak) {
                peak = prox;
                peakT = t;
            }
            if (peak - prox >= PEAK_PROMINENCE && peak - Math.min(valley, peak) >= PEAK_PROMINENCE) {
                // A peak: prominent on both sides.
                boolean valleyOk = PEAK_VALLEY_LIMIT <= 0 || valley - level <= PEAK_VALLEY_LIMIT;
                int height = (int) Math.round(peak - level);
                if (valleyOk && peakT - lastPeakT >= PEAK_MIN_SEPARATION_S) {
                    boolean sameGroup = peakT - lastPeakT < PEAK_MERGE_GAP_S;
                    if (!sameGroup) {
                        groupBig = false;
                        groupCounted = 0;
                    }
                    groupBig |= height >= PEAK_MERGE_HEIGHT;
                    // A group containing a big peak counts once (one hard arrival that rings); small peaks count one by one.
                    if (!(groupBig && groupCounted >= 1)) {
                        groupCounted++;
                        out = new Entry(peakT, height, peak);
                    }
                    lastPeakT = peakT;
                }
                rising = false;
                valley = prox;
            }
        } else {
            if (prox < valley) valley = prox;
            if (prox - valley >= PEAK_PROMINENCE) {
                rising = true;
                peak = prox;
                peakT = t;
            }
        }
        if (valley == Integer.MAX_VALUE) valley = (int) Math.round(level);
        return out;
    }
}
