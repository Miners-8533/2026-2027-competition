package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;
import java.util.List;

/**
 * Turns the raw peaks of {@link PeakCounter} into counted ENTRIES (2026-11-14, pure Java, replayed by tools/entry/PeakCounterCheck.java).
 *
 * The 2026-11-14 13:57 TeleOp (the shadow counter in a real drive) showed that a big excursion of the proximity (+45 to +163 counts; the study's hand-fed elements never went over +44) comes with a
 * burst of sub-peaks that are the SAME object (a wall or flower stand in front of the sensor, or one element rolling past at close range): 8 entries in 1.8 s while one element was gained. So peaks that
 * follow each other within `mergeGapSec` are one GROUP, and a group that contains a peak of at least `mergeHeight` counts is counted ONCE (at its tallest peak); a group of small peaks only (the
 * study's clumps of 3 and 4 elements: +8 to +34 each) is counted peak by peak as before.
 *
 * A group is closed once no peak has followed it for `mergeGapSec` plus a margin (a peak is only confirmed a few samples after it happened), so an entry is reported up to about mergeGapSec + 0.2 s after
 * its peak: fine for logging, to be looked at again before anything controls the intake with it. On the 13:57 log it takes 39 entries to 23 (the six salvos held 23).
 */
public final class EntryGrouper {

    /** One counted entry. */
    public static final class Entry {
        public final double time;      // the time of its (tallest) peak, as given to offer()
        public final double height;    // counts
        public final int peaks;        // how many raw peaks it merged (1 for a plain entry)
        public final String detail;    // "+26@15.51 +94@15.13 ..." the raw peaks of a merged group, "" for a plain entry

        Entry(double time, double height, int peaks, String detail) {
            this.time = time;
            this.height = height;
            this.peaks = peaks;
            this.detail = detail;
        }

        public boolean merged() {
            return peaks > 1;
        }
    }

    /** The margin added to the gap before a group is closed (the confirmation delay of a peak). */
    public static final double CLOSE_MARGIN_SEC = 0.1;

    private final double mergeHeight, mergeGapSec;
    private final List<double[]> group = new ArrayList<>();   // {time, height}

    public EntryGrouper(double mergeHeight, double mergeGapSec) {
        this.mergeHeight = mergeHeight;
        this.mergeGapSec = mergeGapSec;
    }

    /**
     * A confirmed raw peak (times in seconds, non-decreasing). A peak at least mergeGapSec after the previous one belongs to a new group, so the group before it is closed first and its entries are returned
     * (the rule is on the times of the PEAKS, not on when they were confirmed).
     */
    public List<Entry> offer(double time, double height) {
        List<Entry> out = new ArrayList<>();
        if (!group.isEmpty() && time - group.get(group.size() - 1)[0] >= mergeGapSec) close(out);
        group.add(new double[]{time, height});
        return out;
    }

    /**
     * Call with the current time after every sample: returns the entries of the group that has just closed (none while a group is still open or there is nothing pending).
     */
    public List<Entry> poll(double now) {
        List<Entry> out = new ArrayList<>();
        if (group.isEmpty()) return out;
        double last = group.get(group.size() - 1)[0];
        // peaks more than the gap apart inside the group would have closed it already (poll is called every sample); a new peak arriving later than gap + margin starts a new group
        if (now - last < mergeGapSec + CLOSE_MARGIN_SEC) return out;
        close(out);
        return out;
    }

    /** Closes whatever is pending (the end of a run). */
    public List<Entry> flush() {
        List<Entry> out = new ArrayList<>();
        if (!group.isEmpty()) close(out);
        return out;
    }

    private void close(List<Entry> out) {
        double tallest = -1.0, tallestT = 0.0;
        StringBuilder detail = new StringBuilder();
        for (double[] p : group) {
            if (p[1] > tallest) {
                tallest = p[1];
                tallestT = p[0];
            }
            if (detail.length() > 0) detail.append(' ');
            detail.append(String.format(java.util.Locale.US, "+%.0f@%.2f", p[1], p[0]));
        }
        if (group.size() > 1 && tallest >= mergeHeight) {
            out.add(new Entry(tallestT, tallest, group.size(), detail.toString()));
        } else {
            for (double[] p : group) out.add(new Entry(p[0], p[1], 1, ""));
        }
        group.clear();
    }
}
