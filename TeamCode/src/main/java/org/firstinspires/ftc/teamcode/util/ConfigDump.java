package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.drive.AutoLaunch;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
import org.firstinspires.ftc.teamcode.drive.HiveDash;
import org.firstinspires.ftc.teamcode.drive.HoldPosition;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ParkDash;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.motion.AimAtCell;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.motion.Jostle;
import org.firstinspires.ftc.teamcode.motion.SquareOnWall;
import org.firstinspires.ftc.teamcode.opmodes.AutonSolo;
import org.firstinspires.ftc.teamcode.opmodes.DuoAutos;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpCompetition;
import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.PusherCheck;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Writes the live Dashboard values into the run log when a run starts (design/run-logs.md). Dashboard edits live in memory and reset when the app restarts, so without this
 * a log does not say which trim, speed or tolerance was in force. One {@code CONFIG <class> name=value, ...} event per class, then {@code CONFIG CHANGED} lines for every value
 * that differs from the previous run in this app process (the edits made between the two runs). tools/logs/config_changes.py reads them across runs.
 *
 * The class list is written by hand: add a new {@code @Config} class here (tools/logs/check_config_list.py lists the ones that are missing). Bring-up, verify and auto-build opmodes are left out.
 */
public final class ConfigDump {

    private static final Class<?>[] CLASSES = {
            AutoLaunch.class, FieldGeometry.class, HiveDash.class, HoldPosition.class, ParkDash.class, MecanumDrive.class, PinpointLocalizer.class, TargetLock.class,
            AimAtCell.class, DriveToPose.class, Jostle.class, SquareOnWall.class,
            AutonSolo.class, DuoAutos.class, TeleOpCompetition.class,
            ElementCounter.class, Flywheel.class, Intake.class, Mechanisms.class, Pusher.class, PusherCheck.class, Ramp.class,
            LoopProfiler.class};

    private static Map<String, String> previous = null;

    private ConfigDump() {}

    /** Logs every value, then what changed since the last call in this app process. Never throws. */
    public static void logAll() {
        try {
            Map<String, String> now = new LinkedHashMap<>();
            for (Class<?> c : CLASSES) {
                StringBuilder line = new StringBuilder();
                for (Field f : c.getDeclaredFields()) {
                    int m = f.getModifiers();
                    if (!Modifier.isStatic(m) || !Modifier.isPublic(m)) continue;
                    String key = c.getSimpleName() + "." + f.getName();
                    String value = valueOf(f);
                    if (value == null) {
                        // a final holder such as MecanumDrive.PARAMS: its public instance fields
                        if (Modifier.isFinal(m) && !f.getType().isPrimitive() && f.getType() != String.class) {
                            Object holder = f.get(null);
                            if (holder == null) continue;
                            for (Field g : holder.getClass().getFields()) {
                                if (Modifier.isStatic(g.getModifiers())) continue;
                                String v = valueOf(g, holder);
                                if (v == null) continue;
                                now.put(key + "." + g.getName(), v);
                                if (line.length() > 0) line.append(", ");
                                line.append(f.getName()).append('.').append(g.getName()).append('=').append(v);
                            }
                        }
                        continue;
                    }
                    if (Modifier.isFinal(m)) continue;   // constants are not tunable
                    now.put(key, value);
                    if (line.length() > 0) line.append(", ");
                    line.append(f.getName()).append('=').append(value);
                }
                if (line.length() > 0) RunLog.event("CONFIG " + c.getSimpleName() + " " + line);
            }
            if (previous != null) {
                int changed = 0;
                for (Map.Entry<String, String> e : now.entrySet()) {
                    String was = previous.get(e.getKey());
                    if (was != null && !was.equals(e.getValue())) {
                        RunLog.event("CONFIG CHANGED " + e.getKey() + ": " + was + " -> " + e.getValue());
                        changed++;
                    }
                }
                RunLog.event("CONFIG " + changed + " value(s) changed since the previous run in this app session");
            } else {
                RunLog.event("CONFIG first run in this app session: these are the compiled defaults unless they were edited in the Dashboard before the run");
            }
            previous = now;
        } catch (Throwable t) {
            RunLog.event("CONFIG dump failed: " + t);
        }
    }

    private static String valueOf(Field f) {
        try {
            return valueOf(f.get(null), f.getType());
        } catch (IllegalAccessException | RuntimeException e) {
            return null;
        }
    }

    private static String valueOf(Field f, Object holder) {
        try {
            return valueOf(f.get(holder), f.getType());
        } catch (IllegalAccessException | RuntimeException e) {
            return null;
        }
    }

    private static String valueOf(Object v, Class<?> type) {
        if (v == null) return null;
        if (type == double.class || type == float.class) return String.format(Locale.US, "%s", trim(((Number) v).doubleValue()));
        if (type.isPrimitive() || type == String.class || type.isEnum() || v instanceof Number || v instanceof Boolean) return String.valueOf(v).replace(',', ';');
        return null;
    }

    private static String trim(double d) {
        if (d == Math.rint(d) && Math.abs(d) < 1e9) return String.format(Locale.US, "%.1f", d);
        return Double.toString(d);
    }
}
