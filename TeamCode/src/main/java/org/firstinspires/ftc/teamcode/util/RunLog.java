package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * The run log (specs/logging.md section 1). One text file per run, on the Control Hub, written by a background thread.
 *
 * Lines: "#" comment, "H" column names, "S" sample row, "E" event. Fields are separated by commas, with no quoting.
 * The layout is part of the interface: the PC tools read it (specs/logging.md section 1.2).
 *
 * Every call does nothing while no log is open, so any code may log freely. Only the opmode thread calls this class.
 */
public final class RunLog {
    private RunLog() { }

    public enum Kind {
        AUTO("auto"), TELEOP("teleop"), PITCHECK("pitcheck"), BRINGUP("bringup"), WAYPOINTS("waypoints"), DRIVETEST("drivetest");

        public final String file;

        Kind(String file) { this.file = file; }
    }

    /** Builds a sample row from values already read this loop. It must not read the hub. */
    public interface Sampler { String row(); }

    /** Reads the battery voltage (the only hub read the log adds, once a second). */
    public interface Volts { double read(); }

    /** The folder on the hub. A PC check may point this elsewhere. */
    public static String DIR = "/sdcard/FIRST/runlogs";
    public static final int RUNS_KEPT = 10;
    public static final double NORMAL_PERIOD_S = 0.100;
    public static final double SLOW_LOOP_MS = 80.0;
    public static final double SLOW_LOOP_GAP_S = 0.5;
    public static final double VOLTS_PERIOD_S = 1.0;

    public static final int CRASH_TRACE_LINES = 60;

    private static final String STOP = new String("<stop>");
    private static final String INIT_ONLY = "init only, stopped before start";
    private static final String NO_END = "NO END MARKER: the app stopped hard or the robot lost power";

    private static BlockingQueue<String> queue;
    private static Thread writer;
    private static boolean crashed;
    private static long t0;

    private static Sampler sampler;
    private static double period = NORMAL_PERIOD_S;
    private static Volts volts;
    private static double battery = Double.NaN;
    private static double nextVolts, nextSample, lastSlow = -10;
    private static boolean burst;
    private static long lastTick;
    private static double loopSum, loopMax;
    private static int loopN;

    public static boolean isOpen() { return queue != null; }

    /** Seconds since the log opened (0 with no log). */
    public static double seconds() {
        return queue == null ? 0 : (System.nanoTime() - t0) / 1e9;
    }

    /** Opens a new log of a kind. Call first, before any hardware is touched. */
    public static void open(Kind k, String opmode) {
        close();
        try {
            File d = new File(DIR);
            d.mkdirs();
            File latest = new File(d, name(k, 0));
            String prev = previousEnd(latest, k);
            // The one run that is discarded: an INIT that was stopped without starting or crashing. It is deleted
            // here, when the next log of its kind is opened, and the other runs are not shifted (section 1.2).
            if (prev.equals(INIT_ONLY)) latest.delete();
            else if (latest.exists()) shift(d, k);
            crashed = false;
            sampler = null;
            volts = null;
            period = NORMAL_PERIOD_S;
            burst = false;
            battery = Double.NaN;
            nextVolts = 0;
            nextSample = 0;
            lastSlow = -10;
            lastTick = 0;
            loopSum = 0;
            loopMax = 0;
            loopN = 0;
            t0 = System.nanoTime();
            final BlockingQueue<String> q = new LinkedBlockingQueue<>();
            final File f = latest;
            writer = new Thread(new Runnable() {
                @Override public void run() { writeLoop(f, q); }
            }, "RunLog writer");
            writer.setDaemon(true);
            writer.start();
            queue = q;
            LoopProfiler.reset();
            String now = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
            line("# " + opmode + ", opened " + now + " (hub clock). Previous " + k.file + " run: " + prev);
            line("# Build: " + BuildInfo.summary());
        } catch (Throwable e) {
            queue = null;   // the opmode runs without a log
        }
    }

    /** Start was pressed. Times keep counting from the open. */
    public static void started() {
        if (queue == null) return;
        event("RUN STARTED");
        LoopProfiler.reset();   // the loop profiler counts from Start
    }

    public static void sampler(String columns, double periodS, Sampler s) {
        if (queue == null) return;
        sampler = s;
        period = periodS;
        line("H,t,loop_ms,loop_max_ms,volt," + columns);
    }

    public static void volts(Volts v) { volts = v; }

    /** While on, a sample row is written on every loop. */
    public static void burst(boolean on) { burst = on; }

    public static void event(String text) {
        if (queue == null) return;
        line("E," + fmt(seconds(), 3) + "," + text.replace('\n', ' '));
    }

    public static void eventf(String format, Object... args) {
        if (queue == null) return;
        event(String.format(Locale.US, format, args));
    }

    /** Once per loop: writes a sample when one is due, notes a slow loop, and ends the profiler's loop. */
    public static void tick() {
        long now = System.nanoTime();
        if (lastTick != 0) {
            double ms = (now - lastTick) / 1e6;
            loopSum += ms;
            loopN++;
            if (ms > loopMax) loopMax = ms;
            LoopProfiler.endLoop(ms);
            if (queue != null && ms > SLOW_LOOP_MS && seconds() - lastSlow >= SLOW_LOOP_GAP_S) {
                lastSlow = seconds();
                eventf("SLOW LOOP %.0f ms", ms);
            }
        }
        lastTick = now;
        if (queue == null) return;
        double t = seconds();
        if (volts != null && t >= nextVolts) {
            nextVolts = t + VOLTS_PERIOD_S;
            try { battery = volts.read(); } catch (Throwable e) { battery = Double.NaN; }
        }
        if (sampler != null && (burst || t >= nextSample)) {
            nextSample = t + period;
            String row;
            try { row = sampler.row(); } catch (Throwable e) { row = "sampler failed: " + e; }
            line("S," + fmt(t, 3) + "," + fmt(loopN == 0 ? 0 : loopSum / loopN, 1) + "," + fmt(loopMax, 1) + "," + fmt(battery, 2) + "," + row);
            loopSum = 0;
            loopMax = 0;
            loopN = 0;
        }
    }

    /** Records a crash: the exception, then up to 60 lines of its stack trace, each its own event. The caller rethrows. */
    public static void crash(Throwable e) {
        if (queue == null) return;
        crashed = true;
        event("CRASH: " + e);
        StringWriter sw = new StringWriter();
        e.printStackTrace(new PrintWriter(sw));
        String[] trace = sw.toString().replace("\r", "").split("\n");
        int n = 0;
        for (int i = 1; i < trace.length && n < CRASH_TRACE_LINES; i++) {   // line 0 repeats the exception
            String s = trace[i].trim();
            if (s.isEmpty()) continue;
            event(s);
            n++;
        }
    }

    /** Call on every way out of the opmode. Writes the whole-run loop profile, then the end marker as the last line. */
    public static void close() {
        if (queue == null) return;
        LoopProfiler.runLine();
        event(crashed ? "RUN ENDED (after a crash)" : "RUN ENDED (normal)");
        BlockingQueue<String> q = queue;
        queue = null;
        q.offer(STOP);
        try { writer.join(1500); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
    }

    public static String fmt(double v, int decimals) {
        if (Double.isNaN(v)) return "";
        return String.format(Locale.US, "%." + decimals + "f", v);
    }

    // ---- internals ----

    private static void line(String s) {
        BlockingQueue<String> q = queue;
        if (q != null) q.offer(s);
    }

    private static String name(Kind k, int age) {
        return k.file + (age == 0 ? "" : age == 1 ? "_prev" : "_prev" + age) + ".log";
    }

    /** Shifts the kept runs of this kind back one place and drops the oldest, which frees the latest name. */
    private static void shift(File d, Kind k) {
        new File(d, name(k, RUNS_KEPT - 1)).delete();
        for (int age = RUNS_KEPT - 2; age >= 0; age--) {
            File f = new File(d, name(k, age));
            if (f.exists()) f.renameTo(new File(d, name(k, age + 1)));
        }
    }

    /**
     * How the latest run of a kind ended, read from its events: "RUN ENDED" is the end marker, and a run with the
     * marker and neither "RUN STARTED" nor "CRASH:" is an INIT that was stopped before Start.
     * SPEC GAP: the file named after CRASHED is not stated; it is the name the run has once this open has shifted it.
     */
    private static String previousEnd(File f, Kind k) {
        if (!f.exists()) return "none";
        boolean ended = false, wasStarted = false, wasCrashed = false;
        try (BufferedReader r = new BufferedReader(new FileReader(f), 1 << 15)) {
            String s;
            while ((s = r.readLine()) != null) {
                if (!s.startsWith("E,")) continue;
                int c = s.indexOf(',', 2);
                if (c < 0) continue;
                String text = s.substring(c + 1);
                if (text.startsWith("RUN ENDED")) ended = true;
                else if (text.startsWith("RUN STARTED")) wasStarted = true;
                else if (text.startsWith("CRASH:")) wasCrashed = true;
            }
        } catch (Throwable e) {
            return NO_END;
        }
        if (wasCrashed) return "CRASHED (see the exception in " + name(k, 1) + ")";
        if (!ended) return NO_END;
        return wasStarted ? "ended normally" : INIT_ONLY;
    }

    private static void writeLoop(File f, BlockingQueue<String> q) {
        try (BufferedWriter w = new BufferedWriter(new FileWriter(f), 1 << 15)) {
            long lastFlush = System.nanoTime();
            while (true) {
                String s = q.poll(250, TimeUnit.MILLISECONDS);
                if (s == STOP) break;
                if (s != null) {
                    w.write(s);
                    w.write('\n');
                }
                if (System.nanoTime() - lastFlush > 1_000_000_000L) {
                    w.flush();
                    lastFlush = System.nanoTime();
                }
            }
            w.flush();
        } catch (Throwable ignored) { }
    }
}
