package org.firstinspires.ftc.teamcode.util;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;

/**
 * The settings dump (specs/logging.md section 2, target T8). At the start of a run: one "CONFIG class name=value, ..." event
 * per tunable class, then one "CONFIG CHANGED class.name: old -> new" event for every value that differs from the previous
 * run in the same app session.
 *
 * The classes are not listed by hand: the build generates ConfigClasses from every team source file that carries the
 * tunable-value marker (TeamCode/build.gradle, task generateBuildInfo).
 */
public final class ConfigDump {
    private ConfigDump() { }

    private static final Map<String, String> previous = new HashMap<>();
    private static boolean havePrevious;

    public static void dump() {
        if (!RunLog.isOpen()) return;
        Map<String, String> now = new HashMap<>();
        for (String name : ConfigClasses.NAMES) {
            try {
                Class<?> c = Class.forName(name);
                StringBuilder sb = new StringBuilder();
                for (Field f : c.getDeclaredFields()) {
                    int m = f.getModifiers();
                    if (!Modifier.isStatic(m) || !Modifier.isPublic(m) || Modifier.isFinal(m)) continue;
                    String v = text(f.get(null));
                    if (sb.length() > 0) sb.append(", ");
                    sb.append(f.getName()).append('=').append(v);
                    now.put(c.getSimpleName() + "." + f.getName(), v);
                }
                RunLog.event("CONFIG " + c.getSimpleName() + " " + sb);
            } catch (Throwable e) {
                RunLog.event("CONFIG " + name + " could not be read: " + e);
            }
        }
        if (havePrevious) {
            for (Map.Entry<String, String> e : now.entrySet()) {
                String old = previous.get(e.getKey());
                if (old != null && !old.equals(e.getValue())) {
                    RunLog.event("CONFIG CHANGED " + e.getKey() + ": " + old + " -> " + e.getValue());
                }
            }
        }
        previous.clear();
        previous.putAll(now);
        havePrevious = true;
    }

    private static String text(Object v) {
        if (v == null) return "null";
        if (v.getClass().isArray()) {
            StringBuilder sb = new StringBuilder("[");
            for (int i = 0; i < Array.getLength(v); i++) {
                if (i > 0) sb.append(' ');
                sb.append(text(Array.get(v, i)));
            }
            return sb.append(']').toString();
        }
        return String.valueOf(v);
    }
}
