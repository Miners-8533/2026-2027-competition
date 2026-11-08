import org.firstinspires.ftc.teamcode.drive.AutoLaunch;
import org.firstinspires.ftc.teamcode.drive.GridRoutes;
import org.firstinspires.ftc.teamcode.drive.HiveRoute;
import org.firstinspires.ftc.teamcode.drive.ParkDash;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.PriorityQueue;

/**
 * The route generator (design/dash-planner-grid.md, "The grid planner"): for each goal (the two blue launch spots, the two blue park corners; red is the same
 * turned 180 deg) a shortest-time tree over the states of {@link GridRoutes} (a node every 6 in, 16 travel directions, and FINAL = straight to the goal).
 *
 * CLEARANCE (no route may come nearer than this to a hive support, a flower stand or a wall): the robot is a box 2 x L2 by 2 x W2 (L2 the larger of the origin's
 * distances to the front and the back, so the box does not care which end leads), grown by MARGIN_IN plus a TRACKING ALLOWANCE E that is E_MAX_IN away from the
 * goal and falls to GridRoutes.E_GOAL at it (over E_TAPER_IN; near the goal the robot is slow and tracks closely). An edge is valid when the box swept along it is clear;
 * a corner when the box is clear at every body heading between the two legs' (the robot turns there to stay aligned); the final leg when its sweep, the real
 * footprint at the goal and (for a cell) the turn onto the aim are clear.
 *
 * TIME: an edge costs its length at GridRoutes.V_MAX; a corner of travel angle theta costs the time lost slowing to the corner speed v_c and speeding up again, with
 * v_c = sqrt(2 A_GRIP E / g(theta)), g = sin^2(theta) up to 90 deg and 1 beyond (the overshoot of a robot that carries v into the corner and can push sideways at
 * A_GRIP must stay within the allowance E), plus TURN_SEC_PER_RAD of body turn; the final leg its length, the stop, and the turn onto the aim.
 * END SPEEDS: along the tree, the most speed allowed where each edge ends: the corner speed there and what still allows the braking (GridRoutes.A_DEC) for what follows.
 */
public final class RouteGen {

    // the robot (ROBOT_1) and the allowances
    static double DEPTH = 17.48, WIDTH = 13.70, FROM_BACK = 7.598;
    static double MARGIN_IN = 1.5;
    /** The final leg (straight to the goal) is at most this long. */
    static double FINAL_MAX_IN = 30.0;
    // the motion model the costs and speeds come from (the tuned simulation plant of 2026-11-06; regenerate after a refit)
    // (GridRoutes.V_MAX, GridRoutes.A_DEC, A_GRIP and the allowance are in GridRoutes, shared with the robot); a direction change also costs CORNER_SEC, so routes keep to few, long legs
    static double A_ACC = 160.0, TURN_SEC_PER_RAD = 0.15, AIM_SEC_PER_RAD = 0.25, CORNER_SEC = 0.06;
    /** A cell's final leg is driven turned to the aim, partly sideways: its time is costed at this share of the aligned speed (strafe 0.74 of forward, 2026-11-07). */
    static double UNALIGNED_RATIO = 0.8;
    static final double WALL = 71.0;

    static final class Box {
        final double cx, cy, hx, hy, ang;
        final double[] k;

        Box(double cx, double cy, double hx, double hy, double ang) {
            this.cx = cx;
            this.cy = cy;
            this.hx = hx;
            this.hy = hy;
            this.ang = ang;
            double ux = Math.cos(ang), uy = Math.sin(ang), vx = -uy, vy = ux;
            k = new double[]{cx + ux * hx + vx * hy, cy + uy * hx + vy * hy, cx - ux * hx + vx * hy, cy - uy * hx + vy * hy,
                    cx - ux * hx - vx * hy, cy - uy * hx - vy * hy, cx + ux * hx - vx * hy, cy + uy * hx - vy * hy};
        }

        static Box of(HiveRoute.Rect r) {
            return new Box(r.cx, r.cy, r.hx, r.hy, r.ang);
        }
    }

    static boolean overlap(Box a, Box b) {
        double[][] axes = {{Math.cos(a.ang), Math.sin(a.ang)}, {-Math.sin(a.ang), Math.cos(a.ang)}, {Math.cos(b.ang), Math.sin(b.ang)}, {-Math.sin(b.ang), Math.cos(b.ang)}};
        for (double[] ax : axes) {
            double minA = Double.MAX_VALUE, maxA = -Double.MAX_VALUE, minB = Double.MAX_VALUE, maxB = -Double.MAX_VALUE;
            for (int i = 0; i < 8; i += 2) {
                double pa = a.k[i] * ax[0] + a.k[i + 1] * ax[1], pb = b.k[i] * ax[0] + b.k[i + 1] * ax[1];
                minA = Math.min(minA, pa);
                maxA = Math.max(maxA, pa);
                minB = Math.min(minB, pb);
                maxB = Math.max(maxB, pb);
            }
            if (maxA < minB || maxB < minA) return false;
        }
        return true;
    }

    static final List<Box> OBST = new ArrayList<>();

    static {
        HiveRoute.Params p = new HiveRoute.Params();
        p.robotHeightIn = 18.0;
        for (HiveRoute.Rect r : HiveRoute.keepOuts(p, false)) OBST.add(Box.of(r));
        for (HiveRoute.Rect r : HiveRoute.flowerKeepOuts(0.0)) OBST.add(Box.of(r));
    }

    static boolean clear(Box b) {
        for (int i = 0; i < 8; i++) if (Math.abs(b.k[i]) > WALL) return false;
        for (Box o : OBST) if (overlap(b, o)) return false;
        return true;
    }

    static double L2() {
        return Math.max(DEPTH - FROM_BACK, FROM_BACK);
    }

    /** The robot's real footprint (origin FROM_BACK from the back) at a pose, grown by g. */
    static Box footprint(double x, double y, double h, double g) {
        double mid = ((DEPTH - FROM_BACK) - FROM_BACK) / 2.0;
        return new Box(x + Math.cos(h) * mid, y + Math.sin(h) * mid, DEPTH / 2.0 + g, WIDTH / 2.0 + g, h);
    }

    /** The heading-symmetric box swept from (ax, ay) to (bx, by), grown by g, extended extA behind the start and extB past the end. */
    static Box sweep(double ax, double ay, double bx, double by, double g, double extA, double extB) {
        double len = Math.hypot(bx - ax, by - ay), dir = Math.atan2(by - ay, bx - ax);
        double mid = (len + extB - extA) / 2.0;
        return new Box(ax + Math.cos(dir) * mid, ay + Math.sin(dir) * mid, (len + extA + extB) / 2.0, WIDTH / 2.0 + g, dir);
    }

    static double cornerSpeed(double theta, double e) {
        return GridRoutes.cornerSpeed(theta, e);
    }

    static double cornerCost(double theta, double e) {
        if (Math.abs(theta) < 1e-3) return 0.0;
        double vc = cornerSpeed(theta, e), v = GridRoutes.V_MAX;
        return CORNER_SEC + (v - vc) * (v - vc) * (1.0 / (2.0 * GridRoutes.A_DEC) + 1.0 / (2.0 * A_ACC)) / v;
    }

    /** One goal's table plus what the generation found (for the report). */
    static final class Result {
        GridRoutes.Table table;
        int validStates, reachableStates, nodesReachable, freeNodes;
        double ms;
    }

    static double allowance(double x, double y, double gx, double gy) {
        return GridRoutes.allowance(x, y, gx, gy);
    }

    /** Builds the tree for a goal (blue frame). aim: the heading at the goal for a cell, NaN for a park corner. */
    static Result build(String name, double gx, double gy, double aim) {
        long t0 = System.nanoTime();
        final int N = GridRoutes.N, S = GridRoutes.STATES_PER_NODE, NN = N * N, ND = GridRoutes.NDIR;
        double l2 = L2();
        float[] cost = new float[NN * S];
        float[] endSpeed = new float[NN * S];
        int[] next = new int[NN * S];
        Arrays.fill(cost, Float.POSITIVE_INFINITY);
        Arrays.fill(next, GridRoutes.NONE);
        double[] e = new double[NN];
        for (int n = 0; n < NN; n++) e[n] = allowance(GridRoutes.nodeX(n), GridRoutes.nodeY(n), gx, gy);
        // edges
        boolean[] edge = new boolean[NN * S];
        double[] edgeLen = new double[NN * S];
        int valid = 0;
        for (int n = 0; n < NN; n++) {
            double x = GridRoutes.nodeX(n), y = GridRoutes.nodeY(n);
            for (int d = 0; d < ND; d++) {
                int m = GridRoutes.stepNode(n, d);
                if (m == GridRoutes.NONE) continue;
                double g = MARGIN_IN + Math.max(e[n], e[m]);
                double mx = GridRoutes.nodeX(m), my = GridRoutes.nodeY(m);
                if (clear(sweep(x, y, mx, my, g, l2 + g, l2 + g))) {
                    edge[n * S + d] = true;
                    edgeLen[n * S + d] = Math.hypot(mx - x, my - y);
                    valid++;
                }
            }
        }
        // final legs: the sweep to the goal (no box past it) and the real footprint at the goal; for a cell the robot drives the leg ALREADY TURNED TO THE AIM
        // (GridFollower: it turns at the leg's start, so the lock does not spin it near a flower at speed), so its real footprint at the aim is checked all along
        // the leg (every 2 in, the allowance there; found 2026-11-07: the early lock turned a robot 180 deg over the last 35 in into the audience flower)
        double finalCostBase = GridRoutes.V_MAX / (2.0 * GridRoutes.A_DEC);
        PriorityQueue<double[]> pq = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
        for (int n = 0; n < NN; n++) {
            double x = GridRoutes.nodeX(n), y = GridRoutes.nodeY(n);
            double len = Math.hypot(gx - x, gy - y);
            if (len > FINAL_MAX_IN || len < 1.0) continue;
            double g = MARGIN_IN + e[n];
            if (!clear(sweep(x, y, gx, gy, g, l2 + g, 0.0))) continue;
            double dir = Math.atan2(gy - y, gx - x);
            double gg = MARGIN_IN * 0.5 + GridRoutes.E_GOAL;
            double turnCost = 0.0;
            boolean ok;
            if (Double.isNaN(aim)) {
                ok = clear(footprint(gx, gy, dir, gg)) || clear(footprint(gx, gy, dir + Math.PI, gg));
            } else {
                ok = true;
                int steps = Math.max(1, (int) Math.ceil(len / 2.0));
                for (int k = 0; k <= steps && ok; k++) {
                    double px = x + (gx - x) * k / steps, py = y + (gy - y) * k / steps;
                    ok = clear(footprint(px, py, aim, k == steps ? gg : MARGIN_IN + allowance(px, py, gx, gy)));
                }
                // the leg is driven sideways to its direction as much as the aim says: slower than aligned (UNALIGNED_RATIO)
                turnCost = len / GridRoutes.V_MAX * (1.0 / UNALIGNED_RATIO - 1.0);
            }
            if (!ok) continue;
            int s = n * S + GridRoutes.FINAL;
            cost[s] = (float) (len / GridRoutes.V_MAX + finalCostBase + turnCost);
            endSpeed[s] = 0f;
            pq.add(new double[]{cost[s], s});
        }
        // Dijkstra backward from the FINAL states; a predecessor of state (m, d2) is (n, d) with n + step(d) = m and a clear turn at m
        boolean[] done = new boolean[NN * S];
        byte[] turnOk = new byte[NN * S * S];   // 0 unknown, 1 clear, 2 blocked: the turn at node m from direction d to d2
        while (!pq.isEmpty()) {
            double[] top = pq.poll();
            int s2 = (int) top[1];
            if (done[s2]) continue;
            done[s2] = true;
            int m = s2 / S, d2 = s2 % S;
            double ang2 = d2 == GridRoutes.FINAL ? Math.atan2(gy - GridRoutes.nodeY(m), gx - GridRoutes.nodeX(m)) : GridRoutes.dirAngle(d2);
            // the body's heading on the next leg: its axis, or the aim on a cell's final leg
            boolean toAim = d2 == GridRoutes.FINAL && !Double.isNaN(aim);
            for (int d = 0; d < ND; d++) {
                int[] st = GridRoutes.DIRS[d];
                int i = m / N - st[0], j = m % N - st[1];
                if (i < 0 || j < 0 || i >= N || j >= N) continue;
                int n = GridRoutes.node(i, j), s = n * S + d;
                if (done[s] || !edge[s]) continue;
                double ang1 = GridRoutes.dirAngle(d);
                int key = (m * S + d) * S + d2;
                if (turnOk[key] == 0) turnOk[key] = (byte) ((toAim ? aimTurnClear(m, ang1, aim, MARGIN_IN + e[m]) : turnClear(m, ang1, ang2, MARGIN_IN + e[m])) ? 1 : 2);
                if (turnOk[key] != 1) continue;
                double theta = Rotation2d.normalize(ang2 - ang1);
                double bodyTurn = toAim ? GridRoutes.aimTurn(ang1, aim) : GridRoutes.axisTurn(ang1, ang2);
                double c = edgeLen[s] / GridRoutes.V_MAX + cornerCost(theta, e[m]) + bodyTurn * TURN_SEC_PER_RAD + cost[s2];
                if (c < cost[s]) {
                    cost[s] = (float) c;
                    next[s] = s2;
                    pq.add(new double[]{c, s});
                }
            }
        }
        // end speeds in increasing cost (each state's successor is cheaper, so it is done first)
        Integer[] order = new Integer[NN * S];
        int reach = 0;
        for (int s = 0; s < NN * S; s++) {
            order[s] = s;
            if (!Float.isInfinite(cost[s])) reach++;
        }
        Arrays.sort(order, (a, b) -> Float.compare(cost[a], cost[b]));
        for (int s : order) {
            if (Float.isInfinite(cost[s]) || s % S == GridRoutes.FINAL) continue;
            int s2 = next[s];
            int m = s2 / S, d2 = s2 % S;
            double ang1 = GridRoutes.dirAngle(s % S);
            double ang2;
            double entry2;   // the most speed at the start of the successor's edge
            if (d2 == GridRoutes.FINAL) {
                double len = Math.hypot(gx - GridRoutes.nodeX(m), gy - GridRoutes.nodeY(m));
                ang2 = Math.atan2(gy - GridRoutes.nodeY(m), gx - GridRoutes.nodeX(m));
                entry2 = Math.sqrt(2.0 * GridRoutes.A_DEC * len);
            } else {
                ang2 = GridRoutes.dirAngle(d2);
                entry2 = Math.sqrt(endSpeed[s2] * endSpeed[s2] + 2.0 * GridRoutes.A_DEC * edgeLen[s2]);
            }
            endSpeed[s] = (float) Math.min(GridRoutes.V_MAX, Math.min(cornerSpeed(Rotation2d.normalize(ang2 - ang1), e[m]), entry2));
        }
        Result r = new Result();
        r.table = new GridRoutes.Table(name, gx, gy, aim, next, cost, endSpeed);
        r.validStates = valid;
        r.reachableStates = reach;
        for (int n = 0; n < NN; n++) {
            boolean any = false, free = false;
            for (int d = 0; d < S; d++) {
                if (!Float.isInfinite(cost[n * S + d])) any = true;
                if (d < ND && edge[n * S + d]) free = true;
            }
            if (any) r.nodesReachable++;
            if (free) r.freeNodes++;
        }
        r.ms = (System.nanoTime() - t0) / 1e6;
        return r;
    }

    /** The leg's heading is either end of its axis: the heading-symmetric box at node m turning from the axis end nearer the aim to the aim (steps of at most 7.5 deg). */
    static boolean aimTurnClear(int m, double ang1, double aim, double g) {
        double x = GridRoutes.nodeX(m), y = GridRoutes.nodeY(m), l2 = L2();
        double h0 = Math.abs(Rotation2d.normalize(ang1 - aim)) <= Math.abs(Rotation2d.normalize(ang1 + Math.PI - aim)) ? ang1 : ang1 + Math.PI;
        double d = Rotation2d.normalize(aim - h0);
        int steps = (int) Math.ceil(Math.abs(d) / Math.toRadians(7.5));
        for (int k = 1; k <= steps; k++) if (!clear(new Box(x, y, l2 + g, WIDTH / 2.0 + g, h0 + d * k / steps))) return false;
        return true;
    }

    /** The heading-symmetric box at node m is clear at every body heading on the short way between the two legs' axes (steps of at most 7.5 deg). */
    static boolean turnClear(int m, double ang1, double ang2, double g) {
        double x = GridRoutes.nodeX(m), y = GridRoutes.nodeY(m), l2 = L2();
        double d = Rotation2d.normalize(ang2 - ang1);
        if (d > Math.PI / 2) d -= Math.PI;   // the short way between the axes (a leg can be driven either end first)
        if (d < -Math.PI / 2) d += Math.PI;
        int steps = (int) Math.ceil(Math.abs(d) / Math.toRadians(7.5));
        for (int k = 1; k <= steps; k++) {
            double h = ang1 + d * k / steps;
            if (!clear(new Box(x, y, l2 + g, WIDTH / 2.0 + g, h))) return false;
        }
        return true;
    }

    /** The four blue goals: the two launch spots (aim from TargetLock) and the two park corners. */
    static List<Result> buildAll() {
        List<Result> out = new ArrayList<>();
        for (HiveTarget cell : new HiveTarget[]{HiveTarget.BLUE_REAR, HiveTarget.BLUE_AUDIENCE}) {
            double gx = AutoLaunch.centerX(cell), gy = AutoLaunch.centerY(cell);
            out.add(build(cell.name(), gx, gy, TargetLock.aimHeadingRad(gx, gy, cell, TargetLock.farCellExtraLeftIn(cell, true))));
        }
        for (ParkDash.Corner c : new ParkDash.Corner[]{ParkDash.Corner.LEFT, ParkDash.Corner.RIGHT}) {
            double[] g = ParkDash.cornerXY(c, true, Math.toRadians(-90.0));
            out.add(build(c.name(), g[0], g[1], Double.NaN));
        }
        return out;
    }

    // ---- the generated source (GridTables.java) ----

    /** A table packed as GridRoutes.decode reads it: next state (2 bytes, 0xFFFF none), time in ms (2 bytes, 0xFFFF unreachable), end speed in half in/s (1 byte, rounded down). */
    static String[] pack(GridRoutes.Table t) {
        int states = t.next.length;
        byte[] nb = new byte[2 * states], cb = new byte[2 * states], sb = new byte[states];
        for (int s = 0; s < states; s++) {
            int n = t.next[s] == GridRoutes.NONE ? 0xFFFF : t.next[s];
            if (n > 0xFFFF) throw new IllegalStateException("too many states for 2 bytes");
            int c = Float.isInfinite(t.cost[s]) ? 0xFFFF : (int) Math.round(t.cost[s] * 1000.0);
            if (!Float.isInfinite(t.cost[s]) && c >= 0xFFFF) throw new IllegalStateException(t.name + ": state " + s + " has a time of " + t.cost[s] + " s");
            int v = (int) Math.floor(t.endSpeed[s] * 2.0 + 1e-6);
            nb[2 * s] = (byte) (n >> 8);
            nb[2 * s + 1] = (byte) n;
            cb[2 * s] = (byte) (c >> 8);
            cb[2 * s + 1] = (byte) c;
            sb[s] = (byte) Math.max(0, Math.min(255, v));
        }
        java.util.Base64.Encoder e = java.util.Base64.getEncoder();
        return new String[]{e.encodeToString(nb), e.encodeToString(cb), e.encodeToString(sb)};
    }

    /** The table as the robot will have it (packed and unpacked: times to 1 ms, speeds down to 0.5 in/s). */
    static GridRoutes.Table quantize(GridRoutes.Table t) {
        String[] k = pack(t);
        return GridRoutes.decode(t.name, t.goalX, t.goalY, t.aimHeading, k[0], k[1], k[2]);
    }

    static String javaDouble(double v) {
        return Double.isNaN(v) ? "Double.NaN" : String.format(Locale.US, "%.6f", v);
    }

    /** A long string as concatenated literals of at most 100 characters (javac folds them into one constant). */
    static void literal(StringBuilder b, String s, String indent) {
        for (int i = 0; i < s.length(); i += 100) {
            b.append(indent).append(i == 0 ? "  " : "+ ").append('"').append(s, i, Math.min(s.length(), i + 100)).append('"').append('\n');
        }
    }

    /** Writes TeamCode/.../drive/GridTables.java: the four tables, the corner speeds they were built with, and the grid and model they belong to. */
    static void writeSource(List<Result> rs, java.nio.file.Path out, String generatedBy) throws java.io.IOException {
        StringBuilder b = new StringBuilder();
        b.append("package org.firstinspires.ftc.teamcode.drive;\n\n");
        b.append("/**\n * GENERATED: do not edit. Written by ").append(generatedBy).append(" (tools/dash/RouteGen.java; design/dash-planner-grid.md).\n");
        b.append(" * The four blue route trees of the grid planner, packed for GridRoutes.decode (red is the same turned 180 deg about the field center), and the corner\n");
        b.append(" * speed table they were built with. Regenerate after a drive refit, a change to the supports, flowers, launch spots or park corners, or to the grid.\n */\n");
        b.append("final class GridTables {\n\n    private GridTables() {}\n\n");
        b.append("    static final String GEOMETRY = \"").append(GridRoutes.geometry()).append("\";\n");
        b.append("    static final String MODEL = \"").append(GridRoutes.model()).append("\";\n");
        b.append("    static final double[] CORNER_SPEED = {");
        for (int i = 0; i < GridRoutes.CORNER_SPEED.length; i++) b.append(i == 0 ? "" : ", ").append(String.format(Locale.US, "%.1f", GridRoutes.CORNER_SPEED[i]));
        b.append("};\n");
        StringBuilder names = new StringBuilder(), gx = new StringBuilder(), gy = new StringBuilder(), aim = new StringBuilder();
        for (int i = 0; i < rs.size(); i++) {
            GridRoutes.Table t = rs.get(i).table;
            String sep = i == 0 ? "" : ", ";
            names.append(sep).append('"').append(t.name).append('"');
            gx.append(sep).append(javaDouble(t.goalX));
            gy.append(sep).append(javaDouble(t.goalY));
            aim.append(sep).append(javaDouble(t.aimHeading));
        }
        b.append("    static final String[] NAMES = {").append(names).append("};\n");
        b.append("    static final double[] GOAL_X = {").append(gx).append("};\n");
        b.append("    static final double[] GOAL_Y = {").append(gy).append("};\n");
        b.append("    /** The heading at the goal (radians, a hive cell), or NaN (a park corner). */\n");
        b.append("    static final double[] AIM = {").append(aim).append("};\n");
        String[][] packed = new String[rs.size()][];
        for (int i = 0; i < rs.size(); i++) packed[i] = pack(rs.get(i).table);
        String[] arrays = {"NEXT", "COST", "SPEED"};
        for (int a = 0; a < 3; a++) {
            b.append("\n    static final String[] ").append(arrays[a]).append(" = {\n");
            for (int i = 0; i < rs.size(); i++) {
                b.append("            // ").append(rs.get(i).table.name).append('\n');
                literal(b, packed[i][a], "            ");
                if (i + 1 < rs.size()) b.append("            ,\n");
            }
            b.append("    };\n");
        }
        b.append("}\n");
        java.nio.file.Files.write(out, b.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8));
    }

    public static void main(String[] args) {
        for (Result r : buildAll()) {
            System.out.printf(Locale.US, "%-14s goal (%.1f, %.1f)%s: %d valid edges, %d states reach the goal, %d of %d nodes with a clear edge reach it; %.0f ms%n",
                    r.table.name, r.table.goalX, r.table.goalY, Double.isNaN(r.table.aimHeading) ? "" : String.format(Locale.US, " aim %.1f deg", Math.toDegrees(r.table.aimHeading)),
                    r.validStates, r.reachableStates, r.nodesReachable, r.freeNodes, r.ms);
        }
    }
}
