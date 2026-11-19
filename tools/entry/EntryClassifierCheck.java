import org.firstinspires.ftc.teamcode.subsystems.EntryClassifier;
import org.firstinspires.ftc.teamcode.subsystems.EntryGrouper;
import org.firstinspires.ftc.teamcode.subsystems.EntryPipeline;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Replays CS5's proximity AND color recorded on 2026-11-14 (the 13:27 CS Pass Study: 31 pollen fed, no nectar) through the edge counter and {@link EntryClassifier}, the code the robot runs
 * (subsystems/EntryCounter), reading the color only when the robot's thread would (while an element is in front, through the first second, and one read in 8 otherwise), at the recording's 20 ms
 * and thinned to the robot's 25 ms. Then the kind rule and the pass logic on made-up signals. THE NECTAR CASES ARE MADE UP: no nectar has been recorded passing CS5 with its color, so they check
 * the rule's logic, not that real nectar lands inside the limits. Run from the repo root: tools\entry\run-check.cmd.
 */
public class EntryClassifierCheck {
    static int failures = 0;

    static void check(boolean ok, String what) {
        System.out.println((ok ? "  ok    " : "  FAIL  ") + what);
        if (!ok) failures++;
    }

    static final EntryClassifier.Limits LIMITS = new EntryClassifier.Limits(60.0, 0.55, 0.40, 0.45, 0.20);   // EntryCounter's CLASS_* values

    static EntryClassifier classifier() {
        return new EntryClassifier(2.0, 2, 0.5, 3, LIMITS);
    }

    /** One sample of the recording: {t, prox, status, green, blue, red}. */
    static List<double[]> readCsv(String path) throws Exception {
        List<double[]> rows = new ArrayList<>();
        try (BufferedReader r = new BufferedReader(new FileReader(path))) {
            String line;
            boolean header = true;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("#") || line.trim().isEmpty()) continue;
                if (header) {
                    header = false;
                    continue;
                }
                String[] p = line.split(",");
                double[] v = new double[p.length];
                for (int i = 0; i < p.length; i++) v[i] = Double.parseDouble(p[i]);
                rows.add(v);
            }
        }
        return rows;
    }

    /**
     * The robot's thread on a recording: every `step`-th sample is a read; a read carries the color when the thread would ask for it (or always, with `always`), and that color is NEW when the
     * sensor produced a value since the last color read. Returns the results, one per classified entry; counts[0] = entries, counts[1] = color reads, counts[2] = all reads.
     */
    static List<EntryClassifier.Result> replay(List<double[]> rec, int offset, double periodSec, boolean always, int idleEvery, int[] counts) {
        EntryPipeline pipe = EntryPipeline.edge(6.0, 4.0, 2, 1.0);   // EntryCounter.EDGE_HI and EDGE_LO (6 and 4 since 2026-11-18)
        EntryClassifier cls = classifier();
        List<EntryClassifier.Result> out = new ArrayList<>();
        long entries = 0;
        int idle = 0;
        boolean fresh = false;
        double nextT = rec.get(offset)[0];
        for (int i = offset; i < rec.size(); i++) {
            double[] s = rec.get(i);
            fresh |= (((int) s[2]) & 0x08) != 0;   // the sensor made a new color value at some sample since the last color read
            if (s[0] + 1e-9 < nextT) continue;      // not a read of the thinned thread
            nextT += periodSec;
            boolean want = always || cls.wantsColor() || !pipe.baselineDone() || idle + 1 >= idleEvery;
            boolean color = false;
            if (want) {
                color = fresh;
                fresh = false;
                idle = 0;
                counts[1]++;
            } else {
                idle++;
            }
            counts[2]++;
            long counted = 0;
            for (EntryGrouper.Entry e : pipe.push(s[0], s[1])) counted = ++entries;
            double above = pipe.baselineDone() ? s[1] - pipe.emptyLevel() : Double.NaN;
            EntryClassifier.Result r = cls.push(s[0], above, counted, color, (int) s[3], (int) s[4], (int) s[5]);
            if (r != null) out.add(r);
        }
        counts[0] = (int) entries;
        return out;
    }

    static int count(List<EntryClassifier.Result> rs, EntryClassifier.Kind k) {
        int n = 0;
        for (EntryClassifier.Result r : rs) if (r.kind == k) n++;
        return n;
    }

    /** Feeds one made-up pass: `idleN` empty samples (color every one), then `n` samples at `prox` above empty with the given color rise per sample, then empty again. Returns every result. */
    static List<EntryClassifier.Result> pass(EntryClassifier cls, double[] clock, long entryNumber, int n, double prox, int dG, int dB, int dR, boolean withColor) {
        List<EntryClassifier.Result> out = new ArrayList<>();
        final int g0 = 2382, b0 = 1450, r0 = 1198;
        for (int i = 0; i < 6; i++) {
            EntryClassifier.Result r = cls.push(clock[0], 0.0, 0, true, g0, b0, r0);
            if (r != null) out.add(r);
            clock[0] += 0.025;
        }
        for (int i = 0; i < n; i++) {
            EntryClassifier.Result r = cls.push(clock[0], prox, i == 1 ? entryNumber : 0, withColor && cls.wantsColor(), g0 + dG, b0 + dB, r0 + dR);
            if (r != null) out.add(r);
            clock[0] += 0.025;
        }
        for (int i = 0; i < 6; i++) {
            EntryClassifier.Result r = cls.push(clock[0], 0.0, 0, withColor && cls.wantsColor(), g0, b0, r0);
            if (r != null) out.add(r);
            clock[0] += 0.025;
        }
        return out;
    }

    public static void main(String[] args) throws Exception {
        List<double[]> rec = readCsv("tools/entry/data/2026-11-14_132704_cs5_color.csv");
        System.out.println("study run: " + rec.size() + " samples with color");

        // ---- the recorded pollen, the color read only when the robot's thread asks for it ----
        int[] c20 = new int[3];
        List<EntryClassifier.Result> r20 = replay(rec, 0, 0.020, false, 8, c20);
        System.out.println(String.format(Locale.US, "20 ms, color on demand: %d entries, %d classified: %d POLLEN, %d RED, %d BLUE, %d UNKNOWN; %d of %d reads carried the color (%.0f%%)", c20[0], r20.size(),
                count(r20, EntryClassifier.Kind.POLLEN), count(r20, EntryClassifier.Kind.RED_NECTAR), count(r20, EntryClassifier.Kind.BLUE_NECTAR), count(r20, EntryClassifier.Kind.UNKNOWN), c20[1], c20[2], 100.0 * c20[1] / c20[2]));
        for (EntryClassifier.Result r : r20) if (r.kind != EntryClassifier.Kind.POLLEN) System.out.println("    entry " + r.entry + ": " + r.text());
        check(r20.size() == c20[0], "20 ms: every counted entry gets a kind (" + r20.size() + " of " + c20[0] + ")");
        check(count(r20, EntryClassifier.Kind.POLLEN) >= c20[0] - 2, "20 ms: all but at most 2 of the pollen entries are POLLEN (got " + count(r20, EntryClassifier.Kind.POLLEN) + " of " + c20[0] + ")");
        check(count(r20, EntryClassifier.Kind.RED_NECTAR) + count(r20, EntryClassifier.Kind.BLUE_NECTAR) == 0, "20 ms: no pollen is called nectar");
        check(c20[1] < 0.35 * c20[2], "20 ms: under 35 percent of the reads carry the color (got " + c20[1] + " of " + c20[2] + ")");

        double gMin = 1, gMax = 0, bMax = 0, rMin = 1, rMax = 0;
        for (EntryClassifier.Result r : r20) {
            if (r.kind != EntryClassifier.Kind.POLLEN) continue;
            gMin = Math.min(gMin, r.green);
            gMax = Math.max(gMax, r.green);
            bMax = Math.max(bMax, r.blue);
            rMin = Math.min(rMin, r.red);
            rMax = Math.max(rMax, r.red);
        }
        System.out.println(String.format(Locale.US, "    pollen shares at 20 ms: green %.2f to %.2f, blue up to %.2f, red %.2f to %.2f", gMin, gMax, bMax, rMin, rMax));
        check(rMax < LIMITS.redMin - 0.05 && bMax < LIMITS.blueMin - 0.15, "the pollen's red and blue shares stay clear of the nectar limits (red under " + (LIMITS.redMin - 0.05) + ", blue under " + (LIMITS.blueMin - 0.15) + ")");

        // ---- thinned to the robot's 25 ms (the recording is at 20 ms: every read is the first sample at or after the 25 ms tick), at four offsets ----
        for (int off = 0; off < 4; off++) {
            int[] c = new int[3];
            List<EntryClassifier.Result> r = replay(rec, off, 0.025, false, 8, c);
            int pol = count(r, EntryClassifier.Kind.POLLEN), nec = count(r, EntryClassifier.Kind.RED_NECTAR) + count(r, EntryClassifier.Kind.BLUE_NECTAR);
            System.out.println(String.format(Locale.US, "25 ms offset %d: %d entries, %d POLLEN, %d nectar, %d UNKNOWN; %.0f%% of the reads carried the color", off, c[0], pol, nec, count(r, EntryClassifier.Kind.UNKNOWN), 100.0 * c[1] / c[2]));
            check(r.size() == c[0], "25 ms offset " + off + ": every counted entry gets a kind");
            check(pol >= c[0] - 3, "25 ms offset " + off + ": all but at most 3 entries are POLLEN (got " + pol + " of " + c[0] + ")");
            check(nec == 0, "25 ms offset " + off + ": no pollen is called nectar");
        }

        // ---- the same with the color on every read (IDLE_COLOR_EVERY 1): what reading it on demand gives up ----
        int[] ca = new int[3];
        List<EntryClassifier.Result> ra = replay(rec, 0, 0.025, true, 1, ca);
        System.out.println(String.format(Locale.US, "25 ms, color on every read: %d entries, %d POLLEN, %d UNKNOWN", ca[0], count(ra, EntryClassifier.Kind.POLLEN), count(ra, EntryClassifier.Kind.UNKNOWN)));
        check(count(ra, EntryClassifier.Kind.RED_NECTAR) + count(ra, EntryClassifier.Kind.BLUE_NECTAR) == 0, "color on every read: no pollen is called nectar");

        // ---- the kind rule on made-up rises ----
        check(EntryClassifier.classify(1, 300, 20, 200, 4, 0.1, LIMITS, "").kind == EntryClassifier.Kind.POLLEN, "rule: green 0.58, blue 0.04, red 0.38 is POLLEN");
        check(EntryClassifier.classify(1, 80, 20, 300, 4, 0.1, LIMITS, "").kind == EntryClassifier.Kind.RED_NECTAR, "rule: red 0.75 is RED NECTAR (made up)");
        check(EntryClassifier.classify(1, 150, 250, 40, 4, 0.1, LIMITS, "").kind == EntryClassifier.Kind.BLUE_NECTAR, "rule: blue 0.57 is BLUE NECTAR (made up)");
        check(EntryClassifier.classify(1, 150, 110, 120, 4, 0.1, LIMITS, "").kind == EntryClassifier.Kind.UNKNOWN, "rule: a white rise (0.39, 0.29, 0.32) is UNKNOWN");
        check(EntryClassifier.classify(1, 20, 5, 15, 4, 0.1, LIMITS, "").kind == EntryClassifier.Kind.UNKNOWN, "rule: a rise of 40 counts is UNKNOWN (under the minimum signal)");
        check(EntryClassifier.classify(1, 300, 20, 200, 0, 0.1, LIMITS, "").kind == EntryClassifier.Kind.UNKNOWN, "rule: no color sample is UNKNOWN");
        check(EntryClassifier.classify(1, 582, -91, 411, 11, 0.3, LIMITS, "").kind == EntryClassifier.Kind.POLLEN, "rule: a channel that fell counts as 0 (the study's 68.28 s pass is POLLEN)");

        // ---- the pass logic on made-up signals ----
        double[] clock = {0.0};
        EntryClassifier cls = classifier();
        List<EntryClassifier.Result> rs = pass(cls, clock, 1, 5, 10.0, 100, 10, 65, true);
        check(rs.size() == 1 && rs.get(0).kind == EntryClassifier.Kind.POLLEN && rs.get(0).entry == 1, "pass: a pollen-colored pass of 5 samples is one POLLEN result for entry 1");
        rs = pass(cls, clock, 2, 5, 12.0, 25, 10, 110, true);
        check(rs.size() == 1 && rs.get(0).kind == EntryClassifier.Kind.RED_NECTAR && rs.get(0).entry == 2, "pass: a red pass right after it is RED NECTAR for entry 2 (made up)");
        rs = pass(cls, clock, 3, 5, 12.0, 60, 110, 15, true);
        check(rs.size() == 1 && rs.get(0).kind == EntryClassifier.Kind.BLUE_NECTAR, "pass: a blue pass is BLUE NECTAR (made up)");
        rs = pass(cls, clock, 0, 5, 4.0, 100, 10, 65, true);
        check(rs.isEmpty(), "pass: a pass in which nothing was counted gives no result");
        rs = pass(cls, clock, 4, 5, 10.0, 100, 10, 65, false);
        check(rs.size() == 1 && rs.get(0).kind == EntryClassifier.Kind.UNKNOWN, "pass: an entry with no color read is UNKNOWN");

        // a parked element: one result CLASS_MAX_SEC after the entry, then no more color is asked for until it has gone
        cls = classifier();
        clock[0] = 0.0;
        for (int i = 0; i < 6; i++, clock[0] += 0.025) cls.push(clock[0], 0.0, 0, true, 2382, 1450, 1198);
        int parkedResults = 0, wantedAfter = 0;
        double resultAt = -1;
        for (int i = 0; i < 200; i++, clock[0] += 0.025) {
            EntryClassifier.Result r = cls.push(clock[0], 9.0, i == 1 ? 7 : 0, cls.wantsColor(), 2482, 1460, 1263);
            if (r != null) {
                parkedResults++;
                resultAt = i * 0.025;
            } else if (parkedResults > 0 && cls.wantsColor()) {
                wantedAfter++;
            }
        }
        check(parkedResults == 1 && resultAt >= 0.5 && resultAt <= 0.6, "parked: one result, 0.5 s after the entry (got " + parkedResults + " at " + resultAt + " s)");
        check(wantedAfter == 0, "parked: no color is asked for while it stays (got " + wantedAfter + " reads)");
        for (int i = 0; i < 6; i++, clock[0] += 0.025) cls.push(clock[0], 0.0, 0, cls.wantsColor(), 2382, 1450, 1198);
        rs = pass(cls, clock, 8, 5, 10.0, 100, 10, 65, true);
        check(rs.size() == 1 && rs.get(0).entry == 8 && rs.get(0).kind == EntryClassifier.Kind.POLLEN, "parked: after it has gone the next pass is classified as usual");

        // two entries in one stretch (the signal dipped to the re-arm level, not under the pass level): the first closes when the second is counted
        cls = classifier();
        clock[0] = 0.0;
        for (int i = 0; i < 6; i++, clock[0] += 0.025) cls.push(clock[0], 0.0, 0, true, 2382, 1450, 1198);
        List<EntryClassifier.Result> two = new ArrayList<>();
        double[] prox = {8, 10, 9, 2, 2, 9, 11, 8, 0, 0, 0, 0};
        long[] ent = {0, 11, 0, 0, 0, 0, 12, 0, 0, 0, 0, 0};
        for (int i = 0; i < prox.length; i++, clock[0] += 0.025) {
            boolean red = i >= 5;
            EntryClassifier.Result r = cls.push(clock[0], prox[i], ent[i], cls.wantsColor(), 2382 + (prox[i] > 0 ? (red ? 25 : 100) : 0), 1450 + (prox[i] > 0 ? 10 : 0), 1198 + (prox[i] > 0 ? (red ? 110 : 65) : 0));
            if (r != null) two.add(r);
        }
        check(two.size() == 2 && two.get(0).entry == 11 && two.get(1).entry == 12, "touching: two entries in one stretch give two results, in order");
        check(two.size() == 2 && two.get(1).kind == EntryClassifier.Kind.RED_NECTAR, "touching: the second is named from its own samples (RED NECTAR, made up; the first holds part of both: " + (two.isEmpty() ? "-" : two.get(0).kind) + ")");

        System.out.println(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
        if (failures != 0) System.exit(1);
    }
}
