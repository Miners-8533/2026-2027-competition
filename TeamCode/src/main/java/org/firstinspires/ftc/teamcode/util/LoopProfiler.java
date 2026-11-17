package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Where the loop time goes (specs/logging.md section 3). Named sections are timed, named counters count events,
 * and the run log's tick ends each loop. One "LOOP PROFILE window" line every 5 s, one "LOOP PROFILE RUN" line at the end.
 */
public final class LoopProfiler {
    private LoopProfiler() { }

    public static final double WINDOW_S = 5.0;

    private static final class Sec {
        long start;
        boolean top;
        double winSum, winMax, runSum, runMax;
        long winCalls, runCalls;
    }

    private static final class Stats {
        long loops, over40, over60, over80;
        double sum, max, outside;
        long startNs = System.nanoTime();

        void add(double ms, double topMs) {
            loops++;
            sum += ms;
            if (ms > max) max = ms;
            if (ms > 40) over40++;
            if (ms > 60) over60++;
            if (ms > 80) over80++;
            outside += Math.max(0, ms - topMs);
        }
    }

    private static final Map<String, Sec> sections = new LinkedHashMap<>();
    private static final Map<String, long[]> counters = new LinkedHashMap<>();   // {window, run}
    private static int depth;
    private static double loopTopMs;
    private static Stats win = new Stats(), run = new Stats();

    public static void reset() {
        sections.clear();
        counters.clear();
        depth = 0;
        loopTopMs = 0;
        win = new Stats();
        run = new Stats();
    }

    public static void begin(String name) {
        Sec s = sections.get(name);
        if (s == null) {
            s = new Sec();
            sections.put(name, s);
        }
        s.top = depth == 0;
        depth++;
        s.start = System.nanoTime();
    }

    public static void end(String name) {
        Sec s = sections.get(name);
        if (s == null) return;
        double ms = (System.nanoTime() - s.start) / 1e6;
        if (depth > 0) depth--;
        s.winSum += ms;
        s.runSum += ms;
        s.winCalls++;
        s.runCalls++;
        if (ms > s.winMax) s.winMax = ms;
        if (ms > s.runMax) s.runMax = ms;
        if (s.top) loopTopMs += ms;   // a nested section is reported but not counted twice
    }

    public static void count(String name, int n) {
        long[] c = counters.get(name);
        if (c == null) {
            c = new long[2];
            counters.put(name, c);
        }
        c[0] += n;
        c[1] += n;
    }

    /** Called by the run log's tick with the length of the loop that just ended. */
    static void endLoop(double loopMs) {
        win.add(loopMs, loopTopMs);
        run.add(loopMs, loopTopMs);
        loopTopMs = 0;
        depth = 0;
        if ((System.nanoTime() - win.startNs) / 1e9 >= WINDOW_S) {
            RunLog.event(line("LOOP PROFILE window", win, true));
            win = new Stats();
            for (Sec s : sections.values()) {
                s.winSum = 0;
                s.winMax = 0;
                s.winCalls = 0;
            }
            for (long[] c : counters.values()) c[0] = 0;
        }
    }

    static void runLine() {
        if (run.loops > 0) RunLog.event(line("LOOP PROFILE RUN", run, false));
    }

    private static String line(String head, Stats st, boolean window) {
        double n = Math.max(1, st.loops);
        StringBuilder sb = new StringBuilder(head);
        sb.append(String.format(Locale.US, " %.1f s, %d loops, over 40/60/80 ms: %d/%d/%d, avg %.1f ms, max %.1f ms, outside sections %.1f ms",
                (System.nanoTime() - st.startNs) / 1e9, st.loops, st.over40, st.over60, st.over80, st.sum / n, st.max, st.outside / n));
        for (Map.Entry<String, Sec> e : sections.entrySet()) {
            Sec s = e.getValue();
            double sum = window ? s.winSum : s.runSum, max = window ? s.winMax : s.runMax;
            long calls = window ? s.winCalls : s.runCalls;
            if (calls == 0) continue;
            sb.append(String.format(Locale.US, " | %s avg %.2f max %.1f calls/loop %.2f", e.getKey(), sum / n, max, calls / n));
        }
        for (Map.Entry<String, long[]> e : counters.entrySet()) {
            long v = e.getValue()[window ? 0 : 1];
            sb.append(String.format(Locale.US, " | #%s %.2f/loop", e.getKey(), v / n));
        }
        return sb.toString();
    }
}
