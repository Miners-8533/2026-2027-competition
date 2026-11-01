package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Iterator;
import java.util.Locale;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * A log file of the last auto and the last TeleOp, for looking at after the match (copy it off the hub with
 * {@code adb pull /sdcard/FIRST/runlogs}). One run of each kind (auto, teleop, pitcheck) is kept as {@code auto.log} / {@code teleop.log}; the
 * run before it is kept as {@code auto_prev.log} / {@code teleop_prev.log}. Starting an opmode rotates the files, so
 * the auto and the TeleOp never touch each other's, and an opmode that crashes and is started again leaves the crash
 * in {@code *_prev.log} and the rest of the run in {@code *.log}. The only run that is discarded is one that was
 * initialized and stopped without ever starting (and did not crash).
 *
 * Lines are text, one per line, the first field says what it is:
 *   #   a comment (opmode, date, how the previous run ended)
 *   H   the column names of the S rows
 *   S   a sample: seconds since the log opened, average and worst loop time since the last sample (ms), battery
 *       volts (refreshed once a second), then the columns the opmode registered
 *   E   an event: seconds, then free text (button commands, state changes, warnings, a crash and its stack trace)
 *
 * Writing happens on a background thread, so logging never waits on the file system and costs the loop only the
 * work of building a line. The thread flushes about once a second, so a hard crash loses at most that.
 * All methods do nothing while no log is open, so mechanisms and actions can log freely in any opmode.
 */
public final class RunLog {

    public enum Kind {
        AUTO("auto"), TELEOP("teleop"), PITCHECK("pitcheck"), BRINGUP("bringup");

        final String baseName;

        Kind(String baseName) {
            this.baseName = baseName;
        }
    }

    /** Builds the opmode's part of a sample row (comma separated, no newline). Called on the opmode thread. */
    public interface RowSource {
        String row();
    }

    private static final String TAG = "RunLog";
    private static final String POISON = "\u0000end";
    private static final long FLUSH_NS = 1_000_000_000L;
    private static final long VOLTAGE_PERIOD_NS = 1_000_000_000L;
    private static final long SLOW_LOOP_EVENT_GAP_NS = 500_000_000L;
    private static final double SLOW_LOOP_MS = 80.0;

    private static final String MARK_STARTED = "RUN STARTED";
    private static final String MARK_ENDED = "RUN ENDED";
    private static final String MARK_CRASH = "CRASH:";

    private static volatile RunLog active;

    private final BlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private final Thread writer;
    private final long openNs = System.nanoTime();
    private final VoltageSensor battery;

    // touched only by the opmode thread (tick) or under the class lock
    private RowSource source;
    private long samplePeriodNs;
    private long nextSampleNs;
    private long lastTickNs;
    private long lastSlowEventNs;
    private double loopSumMs;
    private double loopMaxMs;
    private int loopCount;
    private long voltageAtNs;
    private double voltage = Double.NaN;
    private volatile boolean crashed;

    private RunLog(File file, VoltageSensor battery) throws IOException {
        this.battery = battery;
        final BufferedWriter out = new BufferedWriter(new FileWriter(file, false));
        writer = new Thread(() -> writeLoop(out), "RunLog writer");
        writer.setDaemon(true);
        writer.start();
    }

    // ---- opening and closing ----

    /**
     * Starts the log for this run, keeping the previous run of the same kind. Call it first in runOpMode (before the
     * hardware is touched, so a crash while initializing is captured too). A log that cannot be opened is skipped
     * (the opmode runs without one).
     */
    public static synchronized void open(Kind kind, String opModeName, HardwareMap hardwareMap) {
        close("superseded by a new run");
        try {
            File dir = new File(AppUtil.FIRST_FOLDER, "runlogs");
            dir.mkdirs();
            File current = new File(dir, kind.baseName + ".log");
            File previous = new File(dir, kind.baseName + "_prev.log");
            String previousRun = rotate(current, previous);

            VoltageSensor battery = null;
            Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
            if (sensors.hasNext()) battery = sensors.next();

            RunLog log = new RunLog(current, battery);
            log.write("# " + opModeName + ", opened " + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date())
                    + " (hub clock). Previous " + kind.baseName + " run: " + previousRun);
            active = log;
        } catch (IOException | RuntimeException e) {
            RobotLog.ee(TAG, e, "could not open the run log; continuing without it");
            active = null;
        }
    }

    /**
     * Moves the last run of this kind to the "previous" file unless it was only an init that stopped cleanly.
     * Returns how that run ended, for the new log's first line.
     */
    private static String rotate(File current, File previous) {
        if (!current.exists()) return "none";
        boolean started = false, ended = false, crash = false;
        try (BufferedReader in = new BufferedReader(new FileReader(current))) {
            String line;
            while ((line = in.readLine()) != null) {
                if (!line.startsWith("E,")) continue;
                if (line.contains(MARK_STARTED)) started = true;
                if (line.contains(MARK_ENDED)) ended = true;
                if (line.contains(MARK_CRASH)) crash = true;
            }
        } catch (IOException e) {
            started = true;   // cannot tell: keep it
        }
        String how = crash ? "CRASHED (see the exception in " + previous.getName() + ")"
                : !ended ? "NO END MARKER: the app stopped hard or the robot lost power"
                : started ? "ended normally" : "init only, stopped before start";
        if (crash || !ended || started) {
            previous.delete();
            if (!current.renameTo(previous)) current.delete();
        } else {
            current.delete();
        }
        return how;
    }

    /** Marks the moment the opmode was started (Start pressed). Times in the file keep counting from the open. */
    public static void started() {
        event(MARK_STARTED);
    }

    /** Ends the log. Call it in a finally block, so every way out of the opmode is recorded. */
    public static synchronized void close() {
        RunLog log = active;
        close(log != null && log.crashed ? "after a crash" : "normal");
    }

    public static synchronized void close(String reason) {
        RunLog log = active;
        if (log == null) return;
        active = null;
        log.write("E," + log.time() + "," + MARK_ENDED + " (" + reason + ")");
        log.queue.offer(POISON);
        try {
            log.writer.join(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // ---- writing ----

    /** A one-off line: a command, a state change, a warning. */
    public static void event(String text) {
        RunLog log = active;
        if (log != null) log.write("E," + log.time() + "," + clean(text));
    }

    /** Records an exception and its stack trace. The opmode should rethrow it afterwards. */
    public static void crash(Throwable t) {
        RunLog log = active;
        if (log == null) return;
        log.crashed = true;
        StringWriter trace = new StringWriter();
        t.printStackTrace(new PrintWriter(trace));
        String[] lines = trace.toString().split("\\r?\\n");
        log.write("E," + log.time() + "," + MARK_CRASH + " " + clean(String.valueOf(t)));
        for (int i = 1; i < lines.length && i < 60; i++) log.write("E," + log.time() + "," + clean(lines[i]));
    }

    /**
     * Registers the sample row. {@code columns} names the fields {@code source} produces; samples are written every
     * {@code periodMs}, whenever {@link #tick} is called after that much time.
     */
    public static void setSampler(String columns, long periodMs, RowSource source) {
        RunLog log = active;
        if (log == null) return;
        log.source = source;
        log.samplePeriodNs = periodMs * 1_000_000L;
        log.nextSampleNs = 0L;
        log.write("H,t,loop_ms,loop_max_ms,volt," + columns);
    }

    /** Call once per loop. Writes a sample when one is due, and notes a loop that took too long. */
    public static void tick() {
        RunLog log = active;
        if (log != null) log.onTick();
    }

    public static boolean isOpen() {
        return active != null;
    }

    private void onTick() {
        long now = System.nanoTime();
        if (lastTickNs != 0L) {
            double ms = (now - lastTickNs) / 1e6;
            loopSumMs += ms;
            loopCount++;
            if (ms > loopMaxMs) loopMaxMs = ms;
            if (ms > SLOW_LOOP_MS && now - lastSlowEventNs > SLOW_LOOP_EVENT_GAP_NS) {
                lastSlowEventNs = now;
                write("E," + time() + ",SLOW LOOP " + String.format(Locale.US, "%.0f ms", ms));
            }
        }
        lastTickNs = now;

        if (source == null || now < nextSampleNs) return;
        nextSampleNs = now + samplePeriodNs;
        if (battery != null && now - voltageAtNs > VOLTAGE_PERIOD_NS) {
            voltageAtNs = now;
            try {
                voltage = battery.getVoltage();
            } catch (RuntimeException e) {
                voltage = Double.NaN;
            }
        }
        String row;
        try {
            row = source.row();
        } catch (RuntimeException e) {
            row = "row failed: " + e;
        }
        double avg = loopCount == 0 ? Double.NaN : loopSumMs / loopCount;
        write(String.format(Locale.US, "S,%s,%.1f,%.1f,%.2f,%s", time(), avg, loopMaxMs, voltage, row));
        loopSumMs = 0.0;
        loopMaxMs = 0.0;
        loopCount = 0;
    }

    private String time() {
        return String.format(Locale.US, "%.3f", (System.nanoTime() - openNs) / 1e9);
    }

    private static String clean(String text) {
        return text == null ? "" : text.replace('\r', ' ').replace('\n', ' ');
    }

    private void write(String line) {
        queue.offer(line);
    }

    private void writeLoop(BufferedWriter out) {
        try {
            long lastFlush = System.nanoTime();
            while (true) {
                String line = queue.poll(1, TimeUnit.SECONDS);
                if (POISON.equals(line)) break;
                if (line != null) {
                    out.write(line);
                    out.write('\n');
                }
                long now = System.nanoTime();
                if (line == null || now - lastFlush >= FLUSH_NS) {
                    out.flush();
                    lastFlush = now;
                }
            }
        } catch (IOException | InterruptedException e) {
            RobotLog.ee(TAG, e, "run log writer stopped");
        } finally {
            try {
                out.close();
            } catch (IOException ignored) {
                // nothing to do
            }
        }
    }
}
