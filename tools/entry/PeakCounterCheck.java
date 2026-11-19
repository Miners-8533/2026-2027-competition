import org.firstinspires.ftc.teamcode.subsystems.EntryGrouper;
import org.firstinspires.ftc.teamcode.subsystems.EntryPipeline;
import org.firstinspires.ftc.teamcode.subsystems.PeakCounter;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Replays the CS5 proximity recorded on 2026-11-14 (the 13:27 CS Pass Study, 31 elements fed, the operator's B presses are the ground truth) through {@link PeakCounter}, the detector the robot
 * runs (subsystems/EntryCounter), at the native 20 ms and resampled to a 30 ms grid (the robot's PERIOD_MS) at six offsets, and checks it against the Python reference of the same algorithm
 * (31 at 20 ms; 30 or 31 at 30 ms) and against the 31 presses. Run from the repo root: tools\entry\run-check.cmd.
 */
public class PeakCounterCheck {
    static int failures = 0;
    /** The edge rule's levels: keep them the same as EntryCounter.EDGE_HI, EDGE_LO and EDGE_CONFIRM (7, 5 and 1 since 2026-11-19; 6, 4 and 2 on 2026-11-18; 6, 2 and 2 before). */
    static final double EDGE_HI = 7.0, EDGE_LO = 5.0;
    static final int EDGE_CONFIRM = 1;

    /**
     * The labelled loads of 2026-11-18 (tools/entry/data/2026-11-18_labelled_loads.csv: what each load held is known from CS2, which sees every element a salvo launches) through the edge rule.
     * The count is capped at 4, as the robot uses it. Adds to tally: [0] loads, [1] right, [2] low, [3] high.
     */
    static void labelledLoads(String path, double hi, double lo, int confirm, int[] tally) throws Exception {
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
                int held = Integer.parseInt(p[2].trim());
                String[] tok = p[3].trim().split(" ");
                EntryPipeline pipe = EntryPipeline.edge(hi, lo, confirm, 1.0);
                final int lead = 44;   // 1.1 s at the empty level first: the pipeline measures the empty level over its first second
                for (int i = 0; i < lead; i++) pipe.push(i * 0.025, 65.0);
                int n = 0;
                for (int i = 0; i < tok.length; i++) {
                    boolean out = tok[i].endsWith("o");
                    double v = Double.parseDouble(out ? tok[i].substring(0, tok[i].length() - 1) : tok[i]);
                    for (EntryGrouper.Entry e : pipe.push((lead + i) * 0.025, 65.0 + v)) {
                        int at = (int) Math.round(e.time / 0.025) - lead;   // the sample the entry began on
                        boolean leaving = at >= 0 && at < tok.length && tok[at].endsWith("o");
                        if (!leaving) n++;
                    }
                }
                int got = Math.min(n, 4), want = Math.min(held, 4);
                tally[0]++;
                tally[got == want ? 1 : got < want ? 2 : 3]++;
            }
        }
    }

    /**
     * One 2026-11-17 COUNT ONLY TeleOp through the edge rule: the entries of every load that began on an emptied track and ended with all four slots reading pollen (so at least 4 were aboard; the
     * slots are a lower limit, not the true number). Adds to tally: [0] loads, [1] under 4, [2] exactly 4, [3] exactly 5, [4] over 5.
     */
    static void drivenLoads(String dir, String run, double hi, double lo, int confirm, int[] tally) throws Exception {
        List<double[]> prox = readCsv(dir + "2026-11-17_" + run + "_teleop_cs5_proximity.csv", 2);
        EntryPipeline pipe = EntryPipeline.edge(hi, lo, confirm, 1.0);
        List<Double> times = new ArrayList<>();
        for (double[] sm : prox) for (EntryGrouper.Entry e : pipe.push(sm[0], sm[1])) times.add(e.time);
        double prev = Double.NaN;
        try (BufferedReader r = new BufferedReader(new FileReader(dir + "2026-11-17_" + run + "_salvos.csv"))) {
            String line;
            boolean header = true;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("#") || line.trim().isEmpty()) continue;
                if (header) {
                    header = false;
                    continue;
                }
                String[] p = line.split(",");
                double t = Double.parseDouble(p[0]);
                if (p[3].trim().equals("1") && p[1].equals("PPPP")) {
                    int n = 0;
                    for (double e : times) if (e >= prev + 1.3 && e < t) n++;
                    tally[0]++;
                    tally[n < 4 ? 1 : n == 4 ? 2 : n == 5 ? 3 : 4]++;
                }
                prev = t;
            }
        }
    }

    static void check(boolean ok, String what) {
        System.out.println((ok ? "  ok    " : "  FAIL  ") + what);
        if (!ok) failures++;
    }

    static List<double[]> readCsv(String path, int cols) throws Exception {
        List<double[]> rows = new ArrayList<>();
        try (BufferedReader r = new BufferedReader(new FileReader(path))) {
            String line;
            boolean header = true;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("#")) continue;
                if (header) {
                    header = false;
                    continue;
                }
                String[] p = line.trim().split(",");
                double[] v = new double[cols];
                for (int i = 0; i < cols; i++) v[i] = Double.parseDouble(p[i]);
                rows.add(v);
            }
        }
        return rows;
    }

    static List<Double> detect(double[] t, double[] x, double prominence, int smooth) {
        PeakCounter pc = new PeakCounter(prominence, smooth, 0.06);
        List<Double> peaks = new ArrayList<>();
        for (int i = 0; i < t.length; i++) {
            double h = pc.push(t[i], x[i]);
            if (!Double.isNaN(h)) peaks.add(pc.lastPeakTime());
        }
        return peaks;
    }

    /** How many presses get a peak: one to one, greedy in time order, the peak between 3 s before and 1 s after the press. */
    static int matched(List<Double> peaks, List<double[]> presses) {
        boolean[] used = new boolean[peaks.size()];
        int m = 0;
        for (double[] pr : presses) {
            int best = -1;
            for (int j = 0; j < peaks.size(); j++) {
                double lag = peaks.get(j) - pr[0];
                if (used[j] || lag < -3.0 || lag > 1.0) continue;
                if (best < 0 || Math.abs(lag + 0.3) < Math.abs(peaks.get(best) - pr[0] + 0.3)) best = j;
            }
            if (best >= 0) {
                used[best] = true;
                m++;
            }
        }
        return m;
    }

    static int groupCount(List<double[]> raw, double mergeHeight, double gap) {
        EntryGrouper g = new EntryGrouper(mergeHeight, gap);
        int n = 0;
        for (double[] p : raw) {
            n += g.poll(p[0]).size();   // time moves on to this peak: a group that has been quiet long enough closes first
            n += g.offer(p[0], p[1]).size();
        }
        n += g.flush().size();
        return n;
    }

    public static void main(String[] args) throws Exception {
        String dir = "tools/entry/data/";
        List<double[]> samples = readCsv(dir + "2026-11-14_132704_cs5_proximity.csv", 2);
        List<double[]> presses = readCsv(dir + "2026-11-14_132704_presses.csv", 1);
        double[] t = new double[samples.size()], x = new double[samples.size()];
        for (int i = 0; i < t.length; i++) {
            t[i] = samples.get(i)[0];
            x[i] = samples.get(i)[1];
        }
        System.out.println(String.format(Locale.US, "%d samples over %.1f s, %d presses", t.length, t[t.length - 1], presses.size()));

        List<Double> native20 = detect(t, x, 6.0, 1);
        int m = matched(native20, presses);
        System.out.println(String.format(Locale.US, "native 20 ms, prominence 6, no smoothing: %d peaks, %d of %d presses matched", native20.size(), m, presses.size()));
        check(native20.size() == 31, "20 ms: 31 peaks (the Python reference)");
        check(m >= 29, "20 ms: at least 29 of the 31 presses matched (recall 94 percent or better)");
        check(native20.size() - m <= 2, "20 ms: at most 2 peaks without a press");

        double[] offsets = {0.0, 0.005, 0.010, 0.015, 0.020, 0.025};
        for (double off : offsets) {
            List<Double> tg = new ArrayList<>(), xg = new ArrayList<>();
            for (double g = t[0] + off; g < t[t.length - 1]; g += 0.030) {
                int idx = java.util.Arrays.binarySearch(t, g);
                if (idx < 0) idx = -idx - 1;
                idx = Math.min(idx, t.length - 1);
                tg.add(t[idx]);
                xg.add(x[idx]);
            }
            double[] tt = new double[tg.size()], xx = new double[xg.size()];
            for (int i = 0; i < tt.length; i++) {
                tt[i] = tg.get(i);
                xx[i] = xg.get(i);
            }
            List<Double> p30 = detect(tt, xx, 6.0, 1);
            int m30 = matched(p30, presses);
            System.out.println(String.format(Locale.US, "30 ms grid, offset %.0f ms: %d peaks, %d matched", off * 1000, p30.size(), m30));
            check(p30.size() == 30 || p30.size() == 31, "30 ms offset " + (int) (off * 1000) + ": 30 or 31 peaks (the Python reference)");
            check(m30 >= 28, "30 ms offset " + (int) (off * 1000) + ": at least 28 of 31 presses matched");
        }

        // an empty stretch gives no peaks; a lone bump gives one; two bumps with a dip to 3 counts give two
        double[] flat = new double[200];
        double[] ft = new double[200];
        for (int i = 0; i < 200; i++) {
            ft[i] = i * 0.03;
            flat[i] = 65 + (i % 3 == 0 ? 1 : 0);
        }
        check(detect(ft, flat, 6.0, 1).isEmpty(), "an empty level with 1-count noise: no peaks");
        double[] two = flat.clone();
        for (int i = 50; i < 60; i++) two[i] = 65 + 12 - Math.abs(i - 55) * 2;      // peak +12 at 55
        for (int i = 62; i < 72; i++) two[i] = 65 + 12 - Math.abs(i - 67) * 2;      // peak +12 at 67, the dip between is at the empty level
        check(detect(ft, two, 6.0, 1).size() == 2, "two bumps with a dip between them: 2 peaks");
        double[] touching = flat.clone();
        for (int i = 50; i < 70; i++) touching[i] = 65 + 8 + (i == 56 || i == 57 ? -3 : 0) + (i > 60 ? 3 : 0);   // a 3-count dip inside a plateau: below the prominence of 6, so one
        check(detect(ft, touching, 6.0, 1).size() <= 2, "a shallow dip inside a plateau is not a separate element (at most 2)");

        // ---- EntryGrouper: the merge rule ----
        // the 39 raw peaks of the 13:57 TeleOp (the shadow counter in a real drive): tall bursts merge, 39 become 23 (the six salvos held 23)
        List<double[]> raw = readCsv(dir + "2026-11-14_135758_teleop_raw_peaks.csv", 2);
        check(raw.size() == 39, "the TeleOp raw peak list has 39 peaks");
        check(groupCount(raw, 45.0, 0.4) == 23, "TeleOp raw peaks, merge height 45, gap 0.4 s: 23 entries (was " + groupCount(raw, 45.0, 0.4) + ")");
        check(groupCount(raw, 100000.0, 0.4) == 39, "merging off (height 100000): all 39 stay");
        // the study run through PeakCounter and EntryGrouper: no peak reaches +45, so nothing is merged and it is still 31
        PeakCounter pc = new PeakCounter(6.0, 1, 0.06);
        EntryGrouper g = new EntryGrouper(45.0, 0.4);
        int studyEntries = 0, maxPeak = 0;
        for (int i = 0; i < t.length; i++) {
            double h = pc.push(t[i], x[i]);
            if (!Double.isNaN(h)) {
                studyEntries += g.offer(pc.lastPeakTime(), h).size();
                maxPeak = Math.max(maxPeak, (int) h);
            }
            studyEntries += g.poll(t[i]).size();
        }
        studyEntries += g.flush().size();
        System.out.println(String.format(Locale.US, "study run through the grouper: %d entries, tallest raw peak +%d", studyEntries, maxPeak));
        check(studyEntries == 31, "study run: 31 entries after the merge rule (nothing in it is over +45)");
        // synthetic: a tall ringing burst is one entry; a clump of four small ones stays four
        EntryGrouper ring = new EntryGrouper(45.0, 0.4);
        int n = 0;
        double[][] burst = {{10.00, 25}, {10.20, 42}, {10.40, 94}, {10.55, 7}, {10.70, 88}, {10.85, 26}};
        for (double[] b : burst) {
            n += ring.offer(b[0], b[1]).size();
            n += ring.poll(b[0] + 0.05).size();
        }
        n += ring.poll(12.0).size();
        check(n == 1, "a burst of six peaks with a +94 in it: 1 entry (got " + n + ")");
        EntryGrouper clump = new EntryGrouper(45.0, 0.4);
        n = 0;
        double[][] four = {{20.00, 12}, {20.30, 15}, {20.60, 11}, {20.90, 14}};
        for (double[] b : four) {
            n += clump.offer(b[0], b[1]).size();
            n += clump.poll(b[0] + 0.05).size();
        }
        n += clump.poll(23.0).size();
        check(n == 4, "four small peaks 0.3 s apart (+11 to +15): 4 entries (got " + n + ")");
        EntryGrouper twoSeparate = new EntryGrouper(45.0, 0.4);
        n = 0;
        n += twoSeparate.offer(30.0, 90).size();
        n += twoSeparate.poll(30.1).size();
        n += twoSeparate.poll(31.0).size();      // closes: 1
        n += twoSeparate.offer(31.5, 80).size();
        n += twoSeparate.poll(32.5).size();
        check(n == 2, "two tall peaks 1.5 s apart are two entries (got " + n + ")");

        // ---- EntryPipeline: the valley rule, replayed on the 14:16 TeleOp ----
        List<double[]> tele = readCsv(dir + "2026-11-14_141611_teleop_cs5_proximity.csv", 2);
        EntryPipeline pipe = new EntryPipeline(6.0, 1, 0.06, 45.0, 0.4, 3.0, 1.0);
        List<double[]> entries = new ArrayList<>();
        for (double[] sm : tele) {
            for (EntryGrouper.Entry e : pipe.push(sm[0], sm[1])) entries.add(new double[]{e.time, e.height});
        }
        for (EntryGrouper.Entry e : pipe.flush()) entries.add(new double[]{e.time, e.height});
        int empty = 0, loadFirst = 0, loadEnd = 0, salvoLoads = 0;
        for (double[] e : entries) {
            if (e[0] < 29.0) empty++;
            else if (e[0] < 65.0) loadFirst++;
            else loadEnd++;
            if (e[0] < 57.9) salvoLoads++;   // before the third salvo started (57.98 s)
        }
        System.out.println(String.format(Locale.US, "14:16 TeleOp through the pipeline: %d entries (%d while empty, %d before the loaded end, %d at the loaded end), %d peaks rejected by the valley rule, empty level %.0f",
                entries.size(), empty, loadFirst, loadEnd, pipe.rejected(), pipe.emptyLevel()));
        check(empty == 0, "the empty drive (0 to 29 s, 1080 in of path): 0 entries");
        check(salvoLoads == 12, "the three loads of 4 pollen before the third salvo (the operator: 4 in the bot at every salvo): exactly 12 entries (got " + salvoLoads + ")");
        check(loadEnd <= 4, "the loaded end (a nectar and too many pollen, parked): at most 4 entries (it was 17 without the valley rule), got " + loadEnd);
        check(pipe.rejected() >= 12, "the valley rule rejected at least 12 of the loaded end's wiggles");
        // the study run through the whole pipeline is unchanged
        EntryPipeline sp = new EntryPipeline(6.0, 1, 0.06, 45.0, 0.4, 3.0, 1.0);
        int studyAll = 0;
        for (int i = 0; i < t.length; i++) studyAll += sp.push(t[i], x[i]).size();
        studyAll += sp.flush().size();
        check(studyAll == 31, "the study run through the whole pipeline: still 31 entries (got " + studyAll + ", rejected " + sp.rejected() + ")");
        // synthetic: a parked object 7 counts above empty with +10 wiggles: no entries; a real bump from the empty level after it comes back down: one
        EntryPipeline park = new EntryPipeline(6.0, 1, 0.06, 45.0, 0.4, 3.0, 1.0);
        int pn = 0;
        for (int i = 0; i < 600; i++) {
            double tt = i * 0.02;
            double v = 66 + (i % 2 == 0 ? 0 : 1);
            if (i >= 200 && i < 400) v = 73 + (i % 40 < 6 ? 10 : 0);   // parked at +7 with wiggles to +17
            if (i >= 450 && i < 462) v = 66 + 12 - Math.abs(i - 456) * 2;   // a real element after it has gone
            pn += park.push(tt, v).size();
        }
        pn += park.flush().size();
        check(pn == 2, "an object arriving (1 entry) and parking with wiggles on top (rejected), then a real bump after it has gone (1 entry): 2 entries (got " + pn + ")");
        check(park.rejected() >= 4, "the wiggles on the parked object are rejected (got " + park.rejected() + ")");

        // ---- EDGE mode (2026-11-15): the robot's counter ----
        EntryPipeline eStudy = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0);
        List<Double> edgeStudy = new ArrayList<>();
        for (int i = 0; i < t.length; i++) for (EntryGrouper.Entry e : eStudy.push(t[i], x[i])) edgeStudy.add(e.time);
        int edgeMatched = matched(edgeStudy, presses);
        System.out.println(String.format(Locale.US, "EDGE on the study run: %d entries, %d of %d presses matched", edgeStudy.size(), edgeMatched, presses.size()));
        check(edgeStudy.size() >= 31 && edgeStudy.size() <= 35, "edge, study run: 31 to 35 entries for the 31 elements fed, never under (32 with the re-arm at +4; the re-arm at +5 counts a few of its hand-fed elements twice) (got " + edgeStudy.size() + ")");
        check(edgeMatched >= 29, "edge, study run: at least 29 of the 31 presses matched (got " + edgeMatched + ")");
        EntryPipeline eTele = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0);
        int eEmpty = 0, eLoads = 0;
        for (double[] sm : tele) {
            for (EntryGrouper.Entry e : eTele.push(sm[0], sm[1])) {
                if (e.time < 29.0) eEmpty++;
                if (e.time < 57.9) eLoads++;
            }
        }
        System.out.println(String.format(Locale.US, "EDGE on the 14:16 TeleOp: %d entries while driving empty (0 to 29 s), %d before the third salvo (3 loads of 4)", eEmpty, eLoads));
        check(eEmpty == 0, "edge, 14:16 empty drive: 0 entries");
        check(eLoads >= 12 && eLoads <= 14, "edge, 14:16: 12 to 14 entries in the three loads of 4 (12 with the re-arm at +2; 13 at +4; 14 at +5, both extra ones in the second load, which the cap at 4 hides) (got " + eLoads + ")");
        // the robot samples every 25 ms, the study was recorded at 20: thinned to 25 ms a rise of +7 that must hold for TWO samples loses some of the 31 hand-fed elements (the weak ones are at +7 for one sample); on ONE sample it loses none
        int thinMin = Integer.MAX_VALUE, thinMax = 0, thin7Min = Integer.MAX_VALUE;
        for (double off : new double[]{0.0, 0.005, 0.010, 0.015, 0.020}) {
            EntryPipeline e25 = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0), e25hi7 = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM + 1, 1.0);
            int n25 = 0, n7 = 0;
            double next = t[0] + off;
            for (int i = 0; i < t.length; i++) {
                if (t[i] + 1e-9 < next) continue;
                next += 0.025;
                n25 += e25.push(t[i], x[i]).size();
                n7 += e25hi7.push(t[i], x[i]).size();
            }
            thinMin = Math.min(thinMin, n25);
            thinMax = Math.max(thinMax, n25);
            thin7Min = Math.min(thin7Min, n7);
        }
        System.out.println("EDGE on the study run thinned to 25 ms (five offsets): " + thinMin + " to " + thinMax + " entries for the 31 fed (held for two samples it would give as few as " + thin7Min + ")");
        check(thinMin >= 31 && thinMax <= 35, "edge, study at 25 ms: 31 to 35 entries at every offset, never under the 31 fed (got " + thinMin + " to " + thinMax + ")");
        List<double[]> comp = readCsv(dir + "2026-11-14_152200_teleop_cs5_proximity.csv", 2);
        EntryPipeline eComp = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0);
        int compTotal = 0;
        for (double[] sm : comp) compTotal += eComp.push(sm[0], sm[1]).size();
        System.out.println("EDGE on the 15:22 TeleOp: " + compTotal + " entries in all (the counter held 25 at the nine salvos; the peaks gave 37)");
        check(compTotal >= 38 && compTotal <= 42, "edge, 15:22: 38 to 42 entries (25 with the re-arm at +2, 29 at +4; this pile-up run is where the re-arm at +5 costs most, and +6 / +5 on two samples gives 50) (got " + compTotal + ")");
        // synthetic: an element parked at +8 with wiggles to +20: ONE entry; four elements with dips to +2: four; a baseline that moves up and stays: a note, no entries
        EntryPipeline eParked = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0);
        int pe = 0;
        for (int i = 0; i < 600; i++) {
            double v = 66 + (i % 2);
            if (i >= 100 && i < 400) v = 74 + (i % 7 == 0 ? 12 : 0) + (i % 11 == 0 ? 6 : 0);
            pe += eParked.push(i * 0.025, v).size();
        }
        check(pe == 1, "edge: an element parked at +8 with wiggles to +20 is ONE entry (got " + pe + ")");
        EntryPipeline eClump = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0);
        int ce = 0;
        for (int i = 0; i < 400; i++) {
            double v = 66 + (i % 2);
            for (int k = 0; k < 4; k++) if (i >= 60 + k * 12 && i < 66 + k * 12) v = 66 + 14;     // a bump of 6 samples, 6 samples apart, dipping to the empty level between
            ce += eClump.push(i * 0.025, v).size();
        }
        check(ce == 4, "edge: four bumps 0.3 s apart that dip back to empty: 4 entries (got " + ce + ")");
        EntryPipeline eStuck = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0);
        int se = 0;
        String stuckNote = null;
        for (int i = 0; i < 1200; i++) {
            double v = i < 100 ? 66 : 76;   // the level jumps to +9 at 2.5 s and stays (+9: above the rise, and above the re-arm level whichever of 66 and 67 the empty level lands on)
            se += eStuck.push(i * 0.025, v).size();
            String nt = eStuck.takeNote();
            if (nt != null && nt.contains("above the empty level")) stuckNote = nt;
        }
        check(se == 1 && stuckNote != null, "edge: a level that jumps up and stays: one entry for the arrival, then a stuck note and no more (entries " + se + ", note " + (stuckNote != null) + ")");

        // synthetic: a driven burst, four bumps of +14 whose dips stay ABOVE the empty level: dips at +5 re-arm the count (4 entries), dips at +6 do not (1 entry)
        for (int dip = 5; dip <= 6; dip++) {
            EntryPipeline eBurst = EntryPipeline.edge(EDGE_HI, EDGE_LO, EDGE_CONFIRM, 1.0);
            int be = 0;
            for (int i = 0; i < 400; i++) {
                double v = 66 + (i % 2);
                if (i >= 60 && i < 102) v = 67 + dip;                                                  // the burst never comes back to the empty level
                for (int k = 0; k < 4; k++) if (i >= 60 + k * 12 && i < 66 + k * 12) v = 67 + 14;
                be += eBurst.push(i * 0.025, v).size();
            }
            check(be == (dip == 5 ? 4 : 1), "edge: a burst of four bumps with dips to +" + dip + ": " + (dip == 5 ? 4 : 1) + " entries (got " + be + ")");
        }

        // ---- the first robot runs of the edge count (2026-11-17, COUNT ONLY): driven loads against the slots ----
        String[] runs = {"191900", "193034", "194209"};
        int[] now = new int[5], before = new int[5];
        for (String run : runs) {
            drivenLoads(dir, run, EDGE_HI, EDGE_LO, EDGE_CONFIRM, now);
            drivenLoads(dir, run, 6.0, 2.0, 2, before);
        }
        System.out.println(String.format(Locale.US, "driven loads with all four slots reading pollen (%d, three COUNT ONLY TeleOps of 2026-11-17): the edge rule at +%.0f / +%.0f gives under 4 at %d, 4 at %d, 5 at %d, over 5 at %d"
                + " (at +6 / +2: under 4 at %d, 4 at %d, 5 at %d, over 5 at %d)", now[0], EDGE_HI, EDGE_LO, now[1], now[2], now[3], now[4], before[1], before[2], before[3], before[4]));
        check(now[0] == 35, "driven loads: 35 loads read from the three runs (got " + now[0] + ")");
        check(before[1] >= 15, "driven loads: the re-arm at +2 undercounted at least 15 of them, as the robot did (got " + before[1] + ")");
        check(now[1] <= 3, "driven loads: at most 3 under 4 (got " + now[1] + ")");
        check(now[4] <= 1, "driven loads: at most 1 over 5 (got " + now[4] + ")");
        check(now[2] >= 20, "driven loads: at least 20 read exactly 4 (got " + now[2] + ")");

        // ---- the labelled loads of 2026-11-18: what each load held is known from CS2 ----
        int[] lab = new int[4], labBefore = new int[4];
        labelledLoads(dir + "2026-11-18_labelled_loads.csv", EDGE_HI, EDGE_LO, EDGE_CONFIRM, lab);
        labelledLoads(dir + "2026-11-18_labelled_loads.csv", 6.0, 4.0, 2, labBefore);
        System.out.println(String.format(Locale.US, "labelled loads of 2026-11-18 (%d, the truth from CS2, the count capped at 4): the edge rule at +%.0f / +%.0f on %d sample(s) is right at %d, low at %d, high at %d"
                + " (at +6 / +4 on 2 samples, as the robot ran them: right at %d, low at %d, high at %d)", lab[0], EDGE_HI, EDGE_LO, EDGE_CONFIRM, lab[1], lab[2], lab[3], labBefore[1], labBefore[2], labBefore[3]));
        check(lab[0] == 208, "labelled loads: 208 loads read (got " + lab[0] + ")");
        check(labBefore[1] == 198 && labBefore[2] == 8, "labelled loads: +6 / +4 on 2 samples is right at 198 and low at 8, as scored from the logs (got " + labBefore[1] + " and " + labBefore[2] + ")");
        check(lab[1] >= 203, "labelled loads: right at 203 or more of the 208 (got " + lab[1] + ")");
        check(lab[2] <= 2, "labelled loads: low at 2 at most (got " + lab[2] + ")");
        check(lab[3] <= 3, "labelled loads: high at 3 at most (got " + lab[3] + ")");

        System.out.println(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
        if (failures != 0) System.exit(1);
    }
}
