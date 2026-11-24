import org.firstinspires.ftc.teamcode.drive.GridFollower;
import org.firstinspires.ftc.teamcode.drive.GridRoutes;
import org.firstinspires.ftc.teamcode.drive.SmoothFollower;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.stream.IntStream;

/**
 * The offline generator of what the smooth paths (SmoothFollower, the 2026-11-17 prototype) look up on the robot instead of working out:
 *
 * THE PLAN TABLES: per goal (the route trees' goals) and per start (every SmoothFollower.START_PITCH_IN inches at START_HEADINGS headings) the code of the quickest clear
 * smooth plan, found with SmoothFollower.search, which tries a few hundred plans per start; 0 where there is none (no route, a route that moves clear first, no
 * clear plan). The robot checks only the plan the table names, at its exact pose.
 *
 * THE CLEARANCE MAP: per inch of the field the distance to the nearest support or flower stand, in quarter inches rounded down.
 *
 * Both go into drive/SmoothTables.java with the INPUTS they were made from (SmoothFollower.planInputs and fieldInputs). The robot compares those with the values
 * in force and does not use a table whose inputs differ; tools/dash/run-sim.cmd checktables says so on the PC.
 */
public final class SmoothGen {

    static final class Result {
        final List<String> names = new ArrayList<>();
        final List<short[]> plans = new ArrayList<>();
        byte[] field;
        String planInputs, fieldInputs;
        double seconds;
    }

    /** One goal's plan codes: (node, heading) -> code. The starts are in the tables' blue frame. */
    static short[] plans(GridRoutes.Table t) {
        final int n = SmoothFollower.START_N, nodes = n * n, heads = SmoothFollower.START_HEADINGS;
        final short[] out = new short[nodes * heads];
        IntStream.range(0, nodes * heads).parallel().forEach(e -> {
            int node = e / heads, k = e % heads;
            double x = -GridRoutes.HALF_SPAN_IN + SmoothFollower.START_PITCH_IN * (node / n), y = -GridRoutes.HALF_SPAN_IN + SmoothFollower.START_PITCH_IN * (node % n), h = k * 2.0 * Math.PI / heads;
            GridRoutes.Route r = GridRoutes.pickOrMoveClear(t, x, y, h);
            if (r == null || r.moveClearLegs > 0) return;
            double[][] f = GridFollower.toField(r, true);
            SmoothFollower s = SmoothFollower.search(f[0], f[1], t.aimHeading, 2.5, h);
            if (s != null) out[e] = (short) s.code();
        });
        return out;
    }

    static byte[] field() {
        int n = SmoothFollower.MAP_N, span = SmoothFollower.MAP_SPAN_IN;
        byte[] out = new byte[n * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) out[i * n + j] = (byte) Math.min(255, (int) Math.floor(4.0 * SmoothFollower.obstacleDistance(i - span, j - span)));
        }
        return out;
    }

    /** Generates everything for the route trees in force (GridRoutes.tables(), or the ones put in their place with useTables). */
    static Result generate() {
        long t0 = System.nanoTime();
        Result r = new Result();
        GridRoutes.Table[] ts = GridRoutes.tables();
        if (ts == null) throw new IllegalStateException("no route trees: " + GridRoutes.tablesProblem());
        GridRoutes.pickOrMoveClear(ts[0], -30.0, -30.0, 0.0);   // loads the pick's obstacles before the threads start
        r.planInputs = SmoothFollower.planInputs();
        r.fieldInputs = SmoothFollower.fieldInputs();
        r.field = field();
        for (GridRoutes.Table t : ts) {
            r.names.add(t.name);
            r.plans.add(plans(t));
        }
        r.seconds = (System.nanoTime() - t0) / 1e9;
        return r;
    }

    static String describe(Result r) {
        StringBuilder b = new StringBuilder();
        for (int t = 0; t < r.names.size(); t++) {
            int with = 0;
            for (short c : r.plans.get(t)) if (c != 0) with++;
            b.append(String.format(Locale.US, "%-14s a smooth plan from %d of %d starts%n", r.names.get(t), with, r.plans.get(t).length));
        }
        b.append(String.format(Locale.US, "clearance map %d values; %.0f s in all", r.field.length, r.seconds));
        return b.toString();
    }

    static String pack(short[] v) {
        byte[] b = new byte[2 * v.length];
        for (int i = 0; i < v.length; i++) {
            b[2 * i] = (byte) (v[i] >> 8);
            b[2 * i + 1] = (byte) v[i];
        }
        return java.util.Base64.getEncoder().encodeToString(b);
    }

    static void write(Result r, Path out, String generatedBy) throws java.io.IOException {
        StringBuilder b = new StringBuilder();
        b.append("package org.firstinspires.ftc.teamcode.drive;\n\n");
        b.append("/**\n * GENERATED: do not edit. Written by ").append(generatedBy).append(" (tools/dash/SmoothGen.java).\n");
        b.append(" * What SmoothFollower looks up instead of working out: per goal and start the code of the smooth plan to check (PLANS: 2 bytes per start position and\n");
        b.append(" * heading, high byte first, base64, in several texts), and the clearance map (FIELD: a byte per inch of the field, quarter inches to the nearest support or flower stand).\n");
        b.append(" * PLAN_INPUTS and FIELD_INPUTS are what they were made from: SmoothFollower compares them with the values in force and does not use a table whose inputs\n");
        b.append(" * differ (specs/dashes.md, \"When to regenerate the dash tables\"). Regenerate with tools/dash/run-sim.cmd gentables; check with ... checktables.\n */\n");
        b.append("final class SmoothTables {\n\n    private SmoothTables() {}\n\n");
        b.append("    static final String PLAN_INPUTS = \"").append(r.planInputs).append("\";\n");
        b.append("    static final String FIELD_INPUTS = \"").append(r.fieldInputs).append("\";\n");
        b.append("    static final String[] NAMES = {");
        for (int i = 0; i < r.names.size(); i++) b.append(i == 0 ? "" : ", ").append('"').append(r.names.get(i)).append('"');
        b.append("};\n\n    static final String FIELD =\n");
        RouteGen.literal(b, java.util.Base64.getEncoder().encodeToString(r.field), "            ");
        // a Java text constant holds at most 65535 bytes: each table is cut into texts of 48000 characters (a whole number of entries each)
        b.append("            ;\n\n    static final String[][] PLANS = {\n");
        for (int i = 0; i < r.plans.size(); i++) {
            b.append("            // ").append(r.names.get(i)).append("\n            {\n");
            String all = pack(r.plans.get(i));
            for (int at = 0; at < all.length(); at += 48000) {
                RouteGen.literal(b, all.substring(at, Math.min(all.length(), at + 48000)), "            ");
                if (at + 48000 < all.length()) b.append("            ,\n");
            }
            b.append(i + 1 < r.plans.size() ? "            },\n" : "            }\n");
        }
        b.append("    };\n}\n");
        Files.write(out, b.toString().getBytes(StandardCharsets.UTF_8));
    }
}
