package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Where the loop time goes. Named sections of the loop are timed with {@link #start()} / {@link #stop(String, long)}; {@link #endLoop()} (called once per
 * loop by {@link RunLog#tick()}) measures the loop itself. Every LOG_PERIOD_SEC the totals of the window are written to the run log as one
 * {@code LOOP PROFILE window ...} line, and once more for the whole run as {@code LOOP PROFILE RUN ...} when the log closes. Read them with
 * tools/logs/loop_profile.py.
 *
 * <pre>
 * LOOP PROFILE window 5.0 s 175 loops | slow 40 60 80: 31 4 0 | loop 28.6 74.2 | other 2.1 | pose 8.1 21.0 1.00 | mechanisms 9.7 40.1 1.00 | mech.counter 4.2 16.0 1.00 | ...
 * </pre>
 * After "loop" and "other" each section is: name, its average per LOOP (ms, total of the section over the loops, so a section that runs every 4th loop
 * is already spread), the longest single call (ms), and how many calls per loop. "other" is the loop time not inside any top-level section (the
 * opmode's own logic: inputs, the drive and lock math, and so on). A section started while another is running is NESTED (for example the sections inside
 * "mechanisms", or "pose" and "drive_write" inside an auto's "action"); nested sections are listed but not counted again in "other".
 *
 * It costs two System.nanoTime calls and a map lookup per section per loop (microseconds) and is meant to stay on. Opmode thread only.
 */
@Config
public final class LoopProfiler {

    /** Off switch (the timing calls then return at once). */
    public static boolean ENABLED = true;
    /** Seconds between the window lines. */
    public static double LOG_PERIOD_SEC = 5.0;

    private static final class Stat {
        long winNs, winCalls, winMaxNs;
        long runNs, runCalls, runMaxNs;

        void add(long ns) {
            winNs += ns;
            winCalls++;
            if (ns > winMaxNs) winMaxNs = ns;
            runNs += ns;
            runCalls++;
            if (ns > runMaxNs) runMaxNs = ns;
        }
    }

    private static final Map<String, Stat> stats = new LinkedHashMap<>();
    private static final Map<String, long[]> counters = new LinkedHashMap<>();   // name -> {this window, the whole run}
    private static int depth = 0;
    private static long topWinNs, topRunNs;
    private static long loopWinNs, loopRunNs, loopWinMaxNs, loopRunMaxNs;
    private static long loopsWin, loopsRun;
    private static final long[] slowWin = new long[3], slowRun = new long[3];   // loops over 40, 60 and 80 ms
    private static final long[] SLOW_NS = {40_000_000L, 60_000_000L, 80_000_000L};
    private static long lastLoopNs, windowStartNs, runStartNs;

    private LoopProfiler() {}

    /** Forgets everything: call at the start of a run. */
    public static void reset() {
        stats.clear();
        counters.clear();
        depth = 0;
        topWinNs = topRunNs = 0L;
        loopWinNs = loopRunNs = loopWinMaxNs = loopRunMaxNs = 0L;
        loopsWin = loopsRun = 0L;
        for (int i = 0; i < 3; i++) slowWin[i] = slowRun[i] = 0L;
        lastLoopNs = 0L;
        windowStartNs = runStartNs = System.nanoTime();
    }

    /** Starts timing a section. Pass the result to {@link #stop(String, long)}. */
    public static long start() {
        if (!ENABLED) return 0L;
        depth++;
        return System.nanoTime();
    }

    /** Ends a section started with {@link #start()}. */
    public static void stop(String name, long startedAt) {
        if (startedAt == 0L) return;
        long dt = System.nanoTime() - startedAt;
        if (depth > 0) depth--;
        Stat s = stats.get(name);
        if (s == null) {
            s = new Stat();
            stats.put(name, s);
        }
        s.add(dt);
        if (depth == 0) {
            topWinNs += dt;
            topRunNs += dt;
        }
    }

    /** Counts an event (for example a drive write sent or skipped). Shown per loop after the sections as {@code count <name> <per loop>}. */
    public static void count(String name) {
        if (!ENABLED) return;
        long[] c = counters.get(name);
        if (c == null) {
            c = new long[2];
            counters.put(name, c);
        }
        c[0]++;   // this window
        c[1]++;   // the whole run
    }

    /** Marks the end of a loop. Called by {@link RunLog#tick()}, once per loop. */
    public static void endLoop() {
        if (!ENABLED) return;
        long now = System.nanoTime();
        depth = 0;
        if (windowStartNs == 0L) windowStartNs = runStartNs = now;
        if (lastLoopNs != 0L) {
            long dt = now - lastLoopNs;
            loopWinNs += dt;
            loopRunNs += dt;
            if (dt > loopWinMaxNs) loopWinMaxNs = dt;
            if (dt > loopRunMaxNs) loopRunMaxNs = dt;
            loopsWin++;
            loopsRun++;
            for (int i = 0; i < 3; i++) {
                if (dt > SLOW_NS[i]) {
                    slowWin[i]++;
                    slowRun[i]++;
                }
            }
        }
        lastLoopNs = now;
        if ((now - windowStartNs) / 1e9 >= LOG_PERIOD_SEC && loopsWin > 0) {
            RunLog.event(format("window", (now - windowStartNs) / 1e9, false));
            for (Stat s : stats.values()) {
                s.winNs = s.winCalls = s.winMaxNs = 0L;
            }
            for (long[] c : counters.values()) c[0] = 0L;
            topWinNs = loopWinNs = loopWinMaxNs = loopsWin = 0L;
            for (int i = 0; i < 3; i++) slowWin[i] = 0L;
            windowStartNs = now;
        }
    }

    /** Writes the whole-run line. Called when the log closes. */
    public static void finish() {
        if (!ENABLED || loopsRun == 0L) return;
        RunLog.event(format("RUN", (System.nanoTime() - runStartNs) / 1e9, true));
    }

    private static String format(String kind, double seconds, boolean wholeRun) {
        long loops = wholeRun ? loopsRun : loopsWin;
        long loopNs = wholeRun ? loopRunNs : loopWinNs;
        long loopMax = wholeRun ? loopRunMaxNs : loopWinMaxNs;
        long topNs = wholeRun ? topRunNs : topWinNs;
        long[] slow = wholeRun ? slowRun : slowWin;
        StringBuilder sb = new StringBuilder(String.format(Locale.US, "LOOP PROFILE %s %.1f s %d loops | slow 40 60 80: %d %d %d | loop %.1f %.1f | other %.1f",
                kind, seconds, loops, slow[0], slow[1], slow[2], loopNs / 1e6 / loops, loopMax / 1e6, (loopNs - topNs) / 1e6 / loops));
        for (Map.Entry<String, Stat> e : stats.entrySet()) {
            Stat s = e.getValue();
            long ns = wholeRun ? s.runNs : s.winNs;
            long calls = wholeRun ? s.runCalls : s.winCalls;
            long max = wholeRun ? s.runMaxNs : s.winMaxNs;
            if (calls == 0L) continue;
            sb.append(String.format(Locale.US, " | %s %.1f %.1f %.2f", e.getKey(), ns / 1e6 / loops, max / 1e6, (double) calls / loops));
        }
        for (Map.Entry<String, long[]> e : counters.entrySet()) {
            long n = wholeRun ? e.getValue()[1] : e.getValue()[0];
            if (n > 0L) sb.append(String.format(Locale.US, " | count %s %.2f", e.getKey(), (double) n / loops));
        }
        return sb.toString();
    }
}
