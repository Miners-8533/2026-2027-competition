package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
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

    private static final String STOP = new String("<stop>");

    private static BlockingQueue<String> queue;
    private static Thread writer;
    private static File dir, tmpFile;
    private static Kind kind;
    private static boolean promoted, started, crashed;
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
            dir = new File(DIR);
            dir.mkdirs();
            File tmp = new File(dir, k.file + ".log.new");
            if (tmp.exists()) rotateIn(dir, k, tmp);      // a run that was killed before it was filed
            String prev = previousEnd(new File(dir, k.file + ".log"));
            kind = k;
            tmpFile = tmp;
            promoted = false;
            started = false;
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
            final File f = tmp;
            writer = new Thread(new Runnable() {
                @Override public void run() { writeLoop(f, q); }
            }, "RunLog writer");
            writer.setDaemon(true);
            writer.start();
            queue = q;
            LoopProfiler.reset();
            String now = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
            line("# " + opmode + ", opened " + now + " (hub clock); the previous " + k.file + " run: " + prev);
            line("# Build: " + BuildInfo.summary());
        } catch (Throwable e) {
            queue = null;   // the opmode runs without a log
        }
    }

    /** Start was pressed. Times keep counting from the open. */
    public static void started() {
        if (queue == null) return;
        started = true;
        promote();
        event("STARTED");
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

    /** Records a crash with its stack trace. The caller rethrows. */
    public static void crash(Throwable e) {
        if (queue == null) return;
        crashed = true;
        promote();
        StringWriter sw = new StringWriter();
        e.printStackTrace(new PrintWriter(sw));
        event("CRASH: " + sw.toString().replace("\r", "").replace("\n", " | "));
    }

    /** Call on every way out of the opmode. */
    public static void close() {
        if (queue == null) return;
        LoopProfiler.runLine();
        // A run that ends normally says so in its last event; a crashed run has its CRASH: event and no RUN ENDED.
        if (crashed) line("# crashed");
        else event("RUN ENDED normally");
        BlockingQueue<String> q = queue;
        queue = null;
        q.offer(STOP);
        try { writer.join(1500); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
        try {
            if (!started && !crashed) tmpFile.delete();    // an INIT that never started: not a run
            else promote();
        } catch (Throwable ignored) { }
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

    private static void promote() {
        if (promoted) return;
        promoted = true;
        try { rotateIn(dir, kind, tmpFile); } catch (Throwable ignored) { }
    }

    private static String name(Kind k, int age) {
        return k.file + (age == 0 ? "" : age == 1 ? "_prev" : "_prev" + age) + ".log";
    }

    /** Shifts the kept runs of this kind back one place, drops the oldest, and files the new one as the latest. */
    private static void rotateIn(File d, Kind k, File fresh) {
        new File(d, name(k, RUNS_KEPT - 1)).delete();
        for (int age = RUNS_KEPT - 2; age >= 0; age--) {
            File f = new File(d, name(k, age));
            if (f.exists()) f.renameTo(new File(d, name(k, age + 1)));
        }
        fresh.renameTo(new File(d, name(k, 0)));
    }

    private static String previousEnd(File f) {
        if (!f.exists()) return "none on the hub";
        try (RandomAccessFile r = new RandomAccessFile(f, "r")) {
            long len = r.length();
            int n = (int) Math.min(len, 600);
            byte[] b = new byte[n];
            r.seek(len - n);
            r.readFully(b);
            String[] lines = new String(b, "UTF-8").split("\n");
            for (int i = lines.length - 1; i >= 0; i--) {
                String s = lines[i].trim();
                if (s.isEmpty()) continue;
                if (s.startsWith("# crashed")) return "ended with a crash";
                if (s.startsWith("E,") && s.contains(",RUN ENDED")) return "ended normally";
                break;
            }
        } catch (Throwable ignored) { }
        return "no end marker (the app stopped hard or power was lost)";
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
