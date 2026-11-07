import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AutoLaunch;
import org.firstinspires.ftc.teamcode.drive.DriveTestRoutine;
import org.firstinspires.ftc.teamcode.drive.HiveDash;
import org.firstinspires.ftc.teamcode.drive.HiveRoute;
import org.firstinspires.ftc.teamcode.drive.HoldPosition;
import org.firstinspires.ftc.teamcode.drive.ParkDash;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The dash simulation (design/dash-planner-grid.md, Phases 1b to 1d). It runs the REAL HiveDash, ParkDash, TargetLock and HoldPosition classes (compiled with the small SDK stubs in
 * tools/dash/stubs) in the TeleOp's loop against a model of the drive fitted from the logs (tools/dash/identify_drive.py), with a Pinpoint that is a noisy, optionally delayed copy
 * of the true pose, and checks the robot's real footprint against the hive supports, the flower stands and the walls.
 *
 * Run from the repo root with tools/dash/run-sim.cmd, or the javac / java lines in it. Modes:
 *   replay [options]     every Hive Dash and Park Dash in the TeleOp logs of 2026-11-05 and 11-06: each is started from its logged pose, speed, cell and battery with the
 *                        Dashboard values of that run (its CONFIG lines) and compared with the logged track; prints a row per dash and the acceptance gate
 *   plant [dash]         the drive model alone, fed the logged commands in 8-row windows exactly as identify_drive.py does, to check the Java port against the Python fit
 *   run x y deg TARGET   one dash with today's values from a pose at rest (TARGET: a cell such as BLUE_AUDIENCE, or LEFT / RIGHT with --red for a park corner)
 * The plant: --plant tuned (the default: the dash fit with its velocity lag in the field frame, a grip limit of 150 in/s^2, a turn boost of 1.5 and the battery at 12.5 V; chosen on the
 *   11-05 logs and checked on the 11-06 logs) or --plant fitted (identify_drive.py's model alone: body frame, logged volts); then any of --model dash|all|file.npy,
 *   --frame body|field, --accel IN_PER_S2 (grip limit, implies the field frame), --turn-boost B, --volts V, --gains F,S,T (multipliers) to change one part.
 * Other options: --loop-ms N (default: the logged loop time), --delay-ms N (Pinpoint delay, default 0), --noise IN (default 0.1), --no-force (do not put the logged route in when
 *   today's planner plans another), --days PREFIX (only the logs whose name starts with it, e.g. 2026-11-05), --case DAY/HHMMSS:T0 (only that dash, with a trace),
 *   --tracks DIR (write a CSV per dash), --seed N.
 */
public final class DashSim {

    // ROBOT_1's footprint (RobotProfile): depth, width, origin from the back edge
    static final double DEPTH = 17.48, WIDTH = 13.70, FROM_BACK = 7.598;
    static final double WALL = 71.0;

    // ================================================================ the drive model

    /** The per-axis model of identify_drive.py: gain, dead zone, tau up, tau down for forward, strafe, turn. */
    static final class Model {
        final String name;
        final double[] k = new double[3], p0 = new double[3], tu = new double[3], td = new double[3];

        /** The fits of 2026-11-06 (identify_drive.py, full precision): dash rows only, and all TeleOp driving rows. */
        static final double[] DASH = {99.09112149473049, 52.32274777761792, 6.646016054636474, 0.11350099123039413, 0.022569029578922054, 0.06683631950982398,
                0.29624991310327675, 0.2501358421085961, 0.20178995973904792, 0.21469726830227168, 0.2176978050149036, 0.10626490959126848};
        static final double[] ALL = {85.91350564892359, 63.362645977697774, 6.317871255764879, 2.1250818183763447e-10, 0.04628499628770735, 0.03488289508003439,
                0.3766606450624685, 0.310107086858538, 0.07247276780262008, 0.1660961371257952, 0.11517184672897446, 0.09598679821767207};

        Model(String name, double[] th) {
            this.name = name;
            for (int a = 0; a < 3; a++) {
                k[a] = th[a];
                p0[a] = th[3 + a];
                tu[a] = th[6 + a];
                td[a] = th[9 + a];
            }
        }

        static Model of(String arg) throws IOException {
            if (arg.equals("dash")) return new Model("dash", DASH);
            if (arg.equals("all")) return new Model("all", ALL);
            return new Model(Paths.get(arg).getFileName().toString(), readNpy(Paths.get(arg)));
        }

        String describe() {
            return String.format(Locale.US, "model %s: forward %.1f in/s per unit (dead %.3f, tau %.3f / %.3f s), strafe %.1f (%.3f, %.3f / %.3f), turn %.2f rad/s (%.3f, %.3f / %.3f)",
                    name, k[0], p0[0], tu[0], td[0], k[1], p0[1], tu[1], td[1], k[2], p0[2], tu[2], td[2]);
        }
    }

    /** A float64 vector from a .npy file (the format numpy.save writes). */
    static double[] readNpy(Path p) throws IOException {
        byte[] b = Files.readAllBytes(p);
        if (b.length < 10 || (b[0] & 0xff) != 0x93 || b[1] != 'N') throw new IOException("not a .npy file: " + p);
        int major = b[6];
        int headerLen, off;
        if (major == 1) {
            headerLen = (b[8] & 0xff) | ((b[9] & 0xff) << 8);
            off = 10;
        } else {
            headerLen = ByteBuffer.wrap(b, 8, 4).order(ByteOrder.LITTLE_ENDIAN).getInt();
            off = 12;
        }
        String header = new String(b, off, headerLen, StandardCharsets.ISO_8859_1);
        if (!header.contains("<f8")) throw new IOException("expected little-endian float64 in " + p + ": " + header);
        int start = off + headerLen, n = (b.length - start) / 8;
        ByteBuffer bb = ByteBuffer.wrap(b, start, n * 8).order(ByteOrder.LITTLE_ENDIAN);
        double[] out = new double[n];
        for (int i = 0; i < n; i++) out[i] = bb.getDouble();
        return out;
    }

    /**
     * The plant: the command is scaled the way MecanumDrive.setDrivePowers scales it (wheel powers x - y - w, x + y - w, x - y + w, x + y + w over the largest when it passes 1),
     * then each axis is a first-order lag towards gain * volts / 12 * sign(c) * max(0, |c| - dead zone), tau up when speeding up and tau down when slowing; the body velocity
     * integrates into the pose with the midpoint heading. Identical to identify_drive.py's simulate() in the body frame.
     */
    static final class Plant {
        final Model m;
        final boolean fieldFrame;
        /** The most the translation velocity may change per second (in/s^2, the tires' grip), field frame only; 0 or less: no limit. */
        final double accelMax;
        /** The turn gain is multiplied by 1 + turnBoost * (translation speed / 60 in/s): a robot that is already rolling turns more easily (0: off). */
        double turnBoost = 0.0;
        /** Multipliers on the three gains (forward, strafe, turn). */
        double[] gainScale = {1.0, 1.0, 1.0};
        double x, y, h;
        /** The walls stop the robot: no corner goes further out than this on either axis (set from the start pose, which may already read a little past 71 in). 0: no walls. */
        double wallX = 0.0, wallY = 0.0;
        /** Body-frame velocity: forward, left (in/s), turn (rad/s). */
        final double[] v = new double[3];

        Plant(Model m, boolean fieldFrame, double accelMax, double x, double y, double h, double vf, double vl, double w) {
            this.m = m;
            this.fieldFrame = fieldFrame;
            this.accelMax = accelMax;
            this.x = x;
            this.y = y;
            this.h = h;
            v[0] = vf;
            v[1] = vl;
            v[2] = w;
        }

        void step(double cf, double cl, double ct, double volts, double d) {
            stepFree(cf, cl, ct, volts, d);
            walls();
        }

        /** One step without the walls (identify_drive.py's model). */
        void stepFree(double cf, double cl, double ct, double volts, double d) {
            double lf = cf - cl - ct, lb = cf + cl - ct, rb = cf - cl + ct, rf = cf + cl + ct;
            double mx = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rb), Math.abs(rf))));
            double[] c = {cf / mx, cl / mx, ct / mx};
            double vs = clamp(volts, 6.0, 13.5) / 12.0;
            double[] tgt = new double[3];
            for (int a = 0; a < 3; a++) tgt[a] = m.k[a] * gainScale[a] * vs * Math.signum(c[a]) * Math.max(0.0, Math.abs(c[a]) - m.p0[a]);
            if (turnBoost != 0.0) tgt[2] *= 1.0 + turnBoost * Math.hypot(v[0], v[1]) / 60.0;
            if (fieldFrame) {
                // the lag acts on the FIELD velocity (momentum keeps its direction while the robot turns): rotate the body target and the state into the field frame
                double co = Math.cos(h), si = Math.sin(h);
                double vx = v[0] * co - v[1] * si, vy = v[0] * si + v[1] * co;
                double tx = tgt[0] * co - tgt[1] * si, ty = tgt[0] * si + tgt[1] * co;
                // speeding up or slowing down is judged on the body axes, as in the body model
                double ax = lag(vx, tx, Math.abs(tgt[0]) > Math.abs(v[0]) ? m.tu[0] : m.td[0], d);
                double ay = lag(vy, ty, Math.abs(tgt[1]) > Math.abs(v[1]) ? m.tu[1] : m.td[1], d);
                if (accelMax > 0.0) {
                    // the tires can only push so hard: the change of the field velocity is limited to accelMax (a robot coming into a corner fast keeps going)
                    double dvx = ax - vx, dvy = ay - vy, dv = Math.hypot(dvx, dvy), lim = accelMax * d;
                    if (dv > lim) {
                        ax = vx + dvx * lim / dv;
                        ay = vy + dvy * lim / dv;
                    }
                }
                double wn = lag(v[2], tgt[2], Math.abs(tgt[2]) > Math.abs(v[2]) ? m.tu[2] : m.td[2], d);
                x += 0.5 * (vx + ax) * d;
                y += 0.5 * (vy + ay) * d;
                double hn = h + 0.5 * (v[2] + wn) * d;
                double co2 = Math.cos(hn), si2 = Math.sin(hn);
                v[0] = ax * co2 + ay * si2;
                v[1] = -ax * si2 + ay * co2;
                v[2] = wn;
                h = hn;
                return;
            }
            double[] vn = new double[3];
            for (int a = 0; a < 3; a++) vn[a] = lag(v[a], tgt[a], Math.abs(tgt[a]) > Math.abs(v[a]) ? m.tu[a] : m.td[a], d);
            double vm0 = 0.5 * (v[0] + vn[0]), vm1 = 0.5 * (v[1] + vn[1]), vm2 = 0.5 * (v[2] + vn[2]);
            double hmid = h + 0.5 * vm2 * d;
            x += (vm0 * Math.cos(hmid) - vm1 * Math.sin(hmid)) * d;
            y += (vm0 * Math.sin(hmid) + vm1 * Math.cos(hmid)) * d;
            h += vm2 * d;
            System.arraycopy(vn, 0, v, 0, 3);
        }

        /** ROBOT_1's corners at the current pose, x0, y0, x1, y1, ... */
        double[] corners() {
            return HiveRoute.poseRect(x, y, h, ROBOT, 0.0).corners();
        }

        /** Sets the walls at 71 in, or further out where the start pose already reads past them. */
        void setWalls() {
            double[] k = corners();
            double mx = 0.0, my = 0.0;
            for (int i = 0; i < 8; i += 2) {
                mx = Math.max(mx, Math.abs(k[i]));
                my = Math.max(my, Math.abs(k[i + 1]));
            }
            wallX = Math.max(WALL, mx);
            wallY = Math.max(WALL, my);
        }

        /** A corner past a wall is pushed back to it and the velocity into that wall is removed (the walls are rigid; the robot does not bounce). */
        void walls() {
            if (wallX <= 0.0) return;
            double[] k = corners();
            double px = 0.0, py = 0.0;
            for (int i = 0; i < 8; i += 2) {
                if (k[i] > wallX) px = Math.min(px, wallX - k[i]);
                if (k[i] < -wallX) px = Math.max(px, -wallX - k[i]);
                if (k[i + 1] > wallY) py = Math.min(py, wallY - k[i + 1]);
                if (k[i + 1] < -wallY) py = Math.max(py, -wallY - k[i + 1]);
            }
            if (px == 0.0 && py == 0.0) return;
            x += px;
            y += py;
            double vx = vx(), vy = vy();
            if (px != 0.0 && vx * px < 0) vx = 0.0;
            if (py != 0.0 && vy * py < 0) vy = 0.0;
            double co = Math.cos(h), si = Math.sin(h);
            v[0] = vx * co + vy * si;
            v[1] = -vx * si + vy * co;
        }

        static double lag(double v, double tgt, double tau, double d) {
            return v + (tgt - v) * (1.0 - Math.exp(-d / tau));
        }

        double vx() {
            return v[0] * Math.cos(h) - v[1] * Math.sin(h);
        }

        double vy() {
            return v[0] * Math.sin(h) + v[1] * Math.cos(h);
        }
    }

    static Plant newPlant(Options o, double x, double y, double h, double vf, double vl, double w) {
        Plant p = new Plant(o.model, o.fieldFrame, o.accelMax, x, y, h, vf, vl, w);
        p.turnBoost = o.turnBoost;
        p.gainScale = o.gainScale.clone();
        return p;
    }

    /** The Pinpoint: the true pose (and field velocity) delayed by delaySec, with white noise. */
    static final class Pinpoint {
        final double delaySec, noiseIn, noiseRad, noiseVel;
        final Random rnd;
        final List<double[]> hist = new ArrayList<>();   // t, x, y, h, vx, vy, w

        Pinpoint(double delaySec, double noiseIn, long seed) {
            this.delaySec = delaySec;
            this.noiseIn = noiseIn;
            this.noiseRad = Math.toRadians(noiseIn);   // 0.1 in goes with 0.1 deg
            this.noiseVel = 10.0 * noiseIn;
            this.rnd = new Random(seed);
        }

        void record(double t, Plant p) {
            hist.add(new double[]{t, p.x, p.y, p.h, p.vx(), p.vy(), p.v[2]});
        }

        double[] read(double t) {
            double want = t - delaySec;
            double[] s = hist.get(0);
            for (int i = hist.size() - 1; i >= 0; i--) {
                if (hist.get(i)[0] <= want + 1e-9) {
                    s = hist.get(i);
                    break;
                }
            }
            return new double[]{s[1] + noiseIn * rnd.nextGaussian(), s[2] + noiseIn * rnd.nextGaussian(), s[3] + noiseRad * rnd.nextGaussian(),
                    s[4] + noiseVel * rnd.nextGaussian(), s[5] + noiseVel * rnd.nextGaussian(), s[6] + noiseRad * 10.0 * rnd.nextGaussian()};
        }
    }

    // ================================================================ the collision model (Phase 1c)

    static final class Obstacle {
        final String name;
        final HiveRoute.Rect r;

        Obstacle(String name, HiveRoute.Rect r) {
            this.name = name;
            this.r = r;
        }
    }

    static HiveRoute.Params robotParams() {
        HiveRoute.Params p = new HiveRoute.Params();
        p.depthIn = DEPTH;
        p.widthIn = WIDTH;
        p.fromBackIn = FROM_BACK;
        p.robotHeightIn = 18.0;
        return p;
    }

    /** The hive's foot bars and A-frame legs (at the robot's height, 18 in) and the four flower stands, from HiveRoute (the field CAD). The walls are handled on their own. */
    static List<Obstacle> obstacles() {
        List<Obstacle> out = new ArrayList<>();
        HiveRoute.Params p = robotParams();
        for (HiveRoute.Rect r : HiveRoute.keepOuts(p, false)) {
            boolean bar = Math.abs(r.ang) < 1e-9 && r.hx > 10.0;
            String side = r.cy > 0 ? "blue" : "red";
            out.add(new Obstacle(bar ? side + " foot bar" : String.format(Locale.US, "%s %s leg", side, r.cx > 0 ? "audience" : "rear"), r));
        }
        String[] names = {"audience flower", "Blue Wall flower", "rear flower", "Red Wall flower"};
        List<HiveRoute.Rect> fl = HiveRoute.flowerKeepOuts(0.0);
        for (int i = 0; i < fl.size(); i++) out.add(new Obstacle(names[i], fl.get(i)));
        return out;
    }

    static final List<Obstacle> OBSTACLES = obstacles();
    static final HiveRoute.Params ROBOT = robotParams();

    /** Signed distance between two oriented rectangles: the gap when apart, minus the smallest overlap (the separating-axis depth) when they overlap. */
    static double rectDistance(HiveRoute.Rect a, HiveRoute.Rect b) {
        double[] ca = a.corners(), cb = b.corners();
        double[][] axes = {{Math.cos(a.ang), Math.sin(a.ang)}, {-Math.sin(a.ang), Math.cos(a.ang)}, {Math.cos(b.ang), Math.sin(b.ang)}, {-Math.sin(b.ang), Math.cos(b.ang)}};
        double minOverlap = Double.MAX_VALUE;
        boolean separated = false;
        for (double[] ax : axes) {
            double minA = Double.MAX_VALUE, maxA = -Double.MAX_VALUE, minB = Double.MAX_VALUE, maxB = -Double.MAX_VALUE;
            for (int i = 0; i < 8; i += 2) {
                double pa = ca[i] * ax[0] + ca[i + 1] * ax[1], pb = cb[i] * ax[0] + cb[i + 1] * ax[1];
                minA = Math.min(minA, pa);
                maxA = Math.max(maxA, pa);
                minB = Math.min(minB, pb);
                maxB = Math.max(maxB, pb);
            }
            double ov = Math.min(maxA - minB, maxB - minA);
            if (ov < 0) separated = true;
            minOverlap = Math.min(minOverlap, ov);
        }
        if (!separated) return -minOverlap;
        double best = Double.MAX_VALUE;
        for (int i = 0; i < 8; i += 2) {
            for (int j = 0; j < 8; j += 2) {
                int jn = (j + 2) % 8;
                best = Math.min(best, pointSegment(ca[i], ca[i + 1], cb[j], cb[j + 1], cb[jn], cb[jn + 1]));
                best = Math.min(best, pointSegment(cb[i], cb[i + 1], ca[j], ca[j + 1], ca[jn], ca[jn + 1]));
            }
        }
        return best;
    }

    static double pointSegment(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
        double s = l2 < 1e-12 ? 0.0 : clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0.0, 1.0);
        return Math.hypot(px - (ax + s * dx), py - (ay + s * dy));
    }

    /** What a track touched: per obstacle the least clearance and the first time it was below zero; the walls separately. */
    static final class Contacts {
        final Map<String, Double> minClear = new LinkedHashMap<>();
        final Map<String, Double> firstHit = new LinkedHashMap<>();
        double wallClear = Double.MAX_VALUE;

        TreeSet<String> touched() {
            return new TreeSet<>(firstHit.keySet());
        }

        double flowerClear() {
            double m = Double.MAX_VALUE;
            for (Map.Entry<String, Double> e : minClear.entrySet()) if (e.getKey().contains("flower")) m = Math.min(m, e.getValue());
            return m;
        }

        double supportClear() {
            double m = Double.MAX_VALUE;
            for (Map.Entry<String, Double> e : minClear.entrySet()) if (!e.getKey().contains("flower")) m = Math.min(m, e.getValue());
            return m;
        }
    }

    /** Checks a track (rows t, x, y, h) every 10 ms (linear in between) over [t0, t1]. */
    static Contacts contacts(List<double[]> track, double t0, double t1) {
        Contacts c = new Contacts();
        HiveRoute.Params p = robotParams();
        for (double t = t0; t <= t1 + 1e-9; t += 0.01) {
            double[] s = interp(track, t);
            if (s == null) continue;
            HiveRoute.Rect me = HiveRoute.poseRect(s[1], s[2], s[3], p, 0.0);
            for (Obstacle o : OBSTACLES) {
                double d = rectDistance(me, o.r);
                c.minClear.merge(o.name, d, Math::min);
                if (d < 0 && !c.firstHit.containsKey(o.name)) c.firstHit.put(o.name, t);
            }
            double[] k = me.corners();
            for (int i = 0; i < 8; i++) c.wallClear = Math.min(c.wallClear, WALL - Math.abs(k[i]));
        }
        return c;
    }

    /** The track's row at time t (linear between rows, heading unwrapped), or null outside it. */
    static double[] interp(List<double[]> tr, double t) {
        if (tr.isEmpty() || t < tr.get(0)[0] - 1e-9 || t > tr.get(tr.size() - 1)[0] + 1e-9) return null;
        int lo = 0, hi = tr.size() - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (tr.get(mid)[0] <= t) lo = mid;
            else hi = mid;
        }
        double[] a = tr.get(lo), b = tr.get(hi);
        double s = b[0] - a[0] < 1e-9 ? 0.0 : clamp((t - a[0]) / (b[0] - a[0]), 0.0, 1.0);
        double dh = normalize(b[3] - a[3]);
        return new double[]{t, a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2]), a[3] + s * dh};
    }

    // ================================================================ the logs

    static final class LogData {
        final String name;
        final Map<String, Map<String, String>> config = new LinkedHashMap<>();
        /** S rows that parse: t, loop ms, volts, x, y, heading (rad), cmd fwd, cmd left, cmd turn. */
        final List<double[]> rows = new ArrayList<>();
        final List<String> modes = new ArrayList<>();
        final List<Double> evT = new ArrayList<>();
        final List<String> evText = new ArrayList<>();

        LogData(Path path) throws IOException {
            name = path.getParent().getFileName() + "/" + path.getFileName().toString().replace("_teleop.log", "");
            for (String l : Files.readAllLines(path, StandardCharsets.ISO_8859_1)) {
                if (l.startsWith("E,")) {
                    String[] p = l.split(",", 3);
                    if (p.length < 3) continue;
                    try {
                        double t = Double.parseDouble(p[1]);
                        evT.add(t);
                        evText.add(p[2]);
                        if (p[2].startsWith("CONFIG ")) {
                            String rest = p[2].substring(7);
                            int sp = rest.indexOf(' ');
                            if (sp > 0) {
                                Map<String, String> m = config.computeIfAbsent(rest.substring(0, sp), k -> new LinkedHashMap<>());
                                for (String kv : rest.substring(sp + 1).split(", ")) {
                                    int eq = kv.indexOf('=');
                                    if (eq > 0) m.put(kv.substring(0, eq).trim(), kv.substring(eq + 1).trim());
                                }
                            }
                        }
                    } catch (NumberFormatException e) {
                        // skip
                    }
                } else if (l.startsWith("S,")) {
                    String[] p = l.split(",", -1);
                    if (p.length <= 14) continue;
                    try {
                        rows.add(new double[]{num(p[1]), num(p[2]), num(p[4]), num(p[5]), num(p[6]), Math.toRadians(num(p[7])), num(p[10]), num(p[11]), num(p[12])});
                        modes.add(p[13]);
                    } catch (NumberFormatException e) {
                        // skip (as identify_drive.py does)
                    }
                }
            }
        }

        double cfgDouble(String cls, String key, double dflt) {
            Map<String, String> m = config.get(cls);
            if (m == null || !m.containsKey(key)) return dflt;
            try {
                return Double.parseDouble(m.get(key));
            } catch (NumberFormatException e) {
                return dflt;
            }
        }

        /** The index of the last row at or before t (or 0). */
        int rowAt(double t) {
            int lo = 0, hi = rows.size() - 1;
            if (hi < 0 || rows.get(0)[0] > t) return 0;
            while (hi - lo > 1) {
                int mid = (lo + hi) >>> 1;
                if (rows.get(mid)[0] <= t) lo = mid;
                else hi = mid;
            }
            return rows.get(hi)[0] <= t ? hi : lo;
        }
    }

    static double num(String s) {
        String t = s.trim();
        if (t.equalsIgnoreCase("nan")) return Double.NaN;
        if (t.equalsIgnoreCase("inf")) return Double.POSITIVE_INFINITY;
        if (t.equalsIgnoreCase("-inf")) return Double.NEGATIVE_INFINITY;
        return Double.parseDouble(t);
    }

    static List<LogData> loadLogs() throws IOException {
        List<Path> files = new ArrayList<>();
        for (String day : new String[]{"2026-11-05", "2026-11-06"}) {
            Path dir = Paths.get("robot-logs", day);
            if (!Files.isDirectory(dir)) continue;
            try (DirectoryStream<Path> ds = Files.newDirectoryStream(dir, "*teleop.log")) {
                for (Path f : ds) files.add(f);
            }
        }
        files.sort((a, b) -> a.toString().compareTo(b.toString()));
        List<LogData> out = new ArrayList<>();
        for (Path f : files) out.add(new LogData(f));
        return out;
    }

    // ================================================================ the logged dashes

    static final Pattern START = Pattern.compile("^(HIVE|PARK) DASH start \\((\\w+)(?: corner)?\\): from \\((-?[\\d.]+), (-?[\\d.]+), (-?[\\d.]+) deg\\) to (?:the launch spot )?\\((-?[\\d.]+), (-?[\\d.]+)\\), ([^;]*);");
    static final Pattern ARRIVED = Pattern.compile("^(?:HIVE DASH arrived|PARK DASH parked) after ([\\d.]+) s at \\((-?[\\d.]+), (-?[\\d.]+), (-?[\\d.]+) deg\\)");
    static final Pattern POINT = Pattern.compile("\\((-?[\\d.]+), (-?[\\d.]+)\\)");

    static final class Case {
        LogData log;
        boolean hive;
        String target;
        double t0, sx, sy, sh, gx, gy;
        final List<double[]> via = new ArrayList<>();
        String desc;
        double tArrive = Double.NaN, ax, ay;
        double tCancel = Double.NaN, tFinish = Double.NaN, tLaunch = Double.NaN;
        String cancelWhy = "";
        double windowEnd;
        double v0f, v0l, v0w;
        /** The log row the pre-roll starts from (-1: none). */
        int preRow = -1;
        double minVolt = Double.MAX_VALUE, loopMs;
        final List<double[]> track = new ArrayList<>();   // relative t, x, y, h

        boolean blue() {
            return hive ? HiveTarget.valueOf(target).isBlue() : gy > 0;
        }

        boolean arrived() {
            return !Double.isNaN(tArrive);
        }

        double[][] route() {
            double[][] w = new double[via.size() + 2][];
            w[0] = new double[]{sx, sy};
            for (int i = 0; i < via.size(); i++) w[i + 1] = via.get(i);
            w[w.length - 1] = new double[]{gx, gy};
            return w;
        }

        String id() {
            return log.name + ":" + String.format(Locale.US, "%.1f", t0);
        }

        double voltAt(double tAbs) {
            int i = log.rowAt(tAbs);
            return log.rows.isEmpty() ? 12.5 : log.rows.get(i)[2];
        }
    }

    static final double COAST_WINDOW_SEC = 0.8;
    static final double PRE_ROLL_SEC = 0.3;

    /**
     * The body velocity at the press: from the central difference at the pre-roll row, the plant runs on the logged commands (each row's command until the next row) up to the
     * press. Returns {vf, vl, w}, or null when the rows do not allow it.
     */
    static double[] preRoll(Case c, Options o) {
        if (c.preRow < 1) return null;
        List<double[]> rows = c.log.rows;
        int i = c.preRow;
        double[] a = rows.get(i - 1), b = rows.get(i + 1), m = rows.get(i);
        double dt = b[0] - a[0];
        if (dt < 0.03 || dt > 0.4) return null;
        double dh = normalize(b[5] - a[5]), hm = m[5];
        double dx = b[3] - a[3], dy = b[4] - a[4];
        Plant p = newPlant(o, m[3], m[4], m[5], (dx * Math.cos(hm) + dy * Math.sin(hm)) / dt, (-dx * Math.sin(hm) + dy * Math.cos(hm)) / dt, dh / dt);
        double t = m[0];
        for (int k = i; k < rows.size() && t < c.t0 - 1e-9; k++) {
            double[] r = rows.get(k);
            double tNext = Math.min(c.t0, k + 1 < rows.size() ? rows.get(k + 1)[0] : c.t0);
            double volts = Double.isNaN(o.fixedVolts) ? r[2] : o.fixedVolts;
            while (t < tNext - 1e-9) {
                double d = Math.min(o.subStepSec, tNext - t);
                p.stepFree(r[6], r[7], r[8], volts, d);
                t += d;
            }
        }
        return new double[]{p.v[0], p.v[1], p.v[2]};
    }

    static List<Case> findCases(LogData L) {
        List<Case> out = new ArrayList<>();
        Case cur = null;
        for (int i = 0; i < L.evT.size(); i++) {
            String e = L.evText.get(i);
            double t = L.evT.get(i);
            Matcher m = START.matcher(e);
            if (m.find()) {
                cur = new Case();
                cur.log = L;
                cur.hive = m.group(1).equals("HIVE");
                cur.target = m.group(2);
                cur.t0 = t;
                cur.sx = Double.parseDouble(m.group(3));
                cur.sy = Double.parseDouble(m.group(4));
                cur.sh = Math.toRadians(Double.parseDouble(m.group(5)));
                cur.gx = Double.parseDouble(m.group(6));
                cur.gy = Double.parseDouble(m.group(7));
                cur.desc = m.group(8);
                if (cur.desc.startsWith("via")) {
                    Matcher pm = POINT.matcher(cur.desc);
                    while (pm.find()) cur.via.add(new double[]{Double.parseDouble(pm.group(1)), Double.parseDouble(pm.group(2))});
                }
                out.add(cur);
                continue;
            }
            if (cur == null) continue;
            String pre = cur.hive ? "HIVE DASH " : "PARK DASH ";
            if (!e.startsWith(pre)) continue;
            Matcher am = ARRIVED.matcher(e);
            if (am.find() && Double.isNaN(cur.tArrive)) {
                cur.tArrive = t - cur.t0;
                cur.ax = Double.parseDouble(am.group(2));
                cur.ay = Double.parseDouble(am.group(3));
            } else if (e.startsWith(pre + "launching") && Double.isNaN(cur.tLaunch)) {
                cur.tLaunch = t - cur.t0;
            } else if ((e.startsWith(pre + "cancelled") || e.startsWith(pre + "released")) && Double.isNaN(cur.tCancel)) {
                cur.tCancel = t - cur.t0;
                int a = e.indexOf('('), b = e.indexOf(')');
                cur.cancelWhy = a > 0 && b > a ? e.substring(a + 1, b) : "";
                cur = null;
            } else if (e.startsWith(pre + "finished")) {
                cur.tFinish = t - cur.t0;
                cur = null;
            }
        }
        for (Case c : out) prepare(c);
        return out;
    }

    static void prepare(Case c) {
        LogData L = c.log;
        double end = Double.MAX_VALUE;
        if (c.arrived()) end = c.tArrive + COAST_WINDOW_SEC;
        if (!Double.isNaN(c.tCancel)) end = Math.min(end, c.tCancel);
        if (!Double.isNaN(c.tFinish)) end = Math.min(end, c.tFinish);
        if (end == Double.MAX_VALUE) end = 10.0;   // the log ended during the dash
        c.windowEnd = end;
        // the logged track (relative time) from the start row to the end of the window
        int i0 = L.rowAt(c.t0);
        List<Double> loops = new ArrayList<>();
        for (int i = i0; i < L.rows.size(); i++) {
            double[] r = L.rows.get(i);
            double tr = r[0] - c.t0;
            if (tr < -1e-6) continue;
            if (tr > end + 1e-6) break;
            c.track.add(new double[]{tr, r[3], r[4], r[5]});
            c.minVolt = Math.min(c.minVolt, r[2]);
            if (r[1] > 5) loops.add(r[1]);
        }
        c.track.add(0, new double[]{0.0, c.sx, c.sy, c.sh});
        TrackUtil.sortTracks(c.track);
        c.loopMs = loops.isEmpty() ? 31.0 : median(loops);
        // the speed at the start: the plant runs on the logged commands from PRE_ROLL_SEC before the press (see preRoll), from the central difference there. The backward
        // difference over the two rows before the press is the fallback (it can be 0.1 s stale: a robot braking hard started the simulation 35 in/s too fast)
        int pr = L.rowAt(c.t0 - PRE_ROLL_SEC);
        if (pr >= 1 && pr + 1 < L.rows.size() && L.rows.get(pr)[0] < c.t0 && c.t0 - L.rows.get(pr)[0] < PRE_ROLL_SEC + 0.15) c.preRow = pr;
        if (i0 >= 1 && i0 < L.rows.size()) {
            double[] a = L.rows.get(i0 - 1), b = L.rows.get(i0);
            double dt = b[0] - a[0];
            if (dt > 0.02 && dt < 0.3) {
                double dx = b[3] - a[3], dy = b[4] - a[4], dh = normalize(b[5] - a[5]);
                double hm = a[5] + 0.5 * dh;
                c.v0f = (dx * Math.cos(hm) + dy * Math.sin(hm)) / dt;
                c.v0l = (-dx * Math.sin(hm) + dy * Math.cos(hm)) / dt;
                c.v0w = dh / dt;
            }
        }
    }

    /** Sorting and de-duplicating the time column of a track. */
    static final class TrackUtil {
        static void sortTracks(List<double[]> tr) {
            tr.sort((a, b) -> Double.compare(a[0], b[0]));
            for (int i = tr.size() - 1; i > 0; i--) if (tr.get(i)[0] - tr.get(i - 1)[0] < 1e-6) tr.remove(i);
        }
    }

    // ================================================================ the Dashboard values of a run

    static final Class<?>[] CONFIG_CLASSES = {HiveDash.class, ParkDash.class, AutoLaunch.class, TargetLock.class, HoldPosition.class};
    static final Map<Field, Object> DEFAULTS = new HashMap<>();

    static {
        for (Class<?> k : CONFIG_CLASSES) {
            for (Field f : k.getFields()) {
                int mod = f.getModifiers();
                if (!Modifier.isStatic(mod) || Modifier.isFinal(mod)) continue;
                try {
                    DEFAULTS.put(f, f.get(null));
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    static void restoreDefaults() {
        try {
            for (Map.Entry<Field, Object> e : DEFAULTS.entrySet()) e.getKey().set(null, e.getValue());
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
        HiveDash.OVERLAY_ENABLED = false;
    }

    /**
     * Today's defaults, then the run's CONFIG values. A HiveDash feature that the run's code did not have yet (its value is missing from the CONFIG line) is set to its "off" value,
     * so the follower behaves as it did then: APPROACH_POWER and CORNER_POWER 0 (before 2026-11-06 00:12), EARLY_LOCK_RADIUS_IN 0 and LAUNCH_HOLD_SEC = AutoLaunch.ALIGNED_HOLD_SEC
     * (before 11-05 23:43), LANE_HALF_WIDTH_IN 22.75 (before 11-05 21:38), FLOWER_CLEARANCE_IN 0. Returns the keys set that way.
     */
    static List<String> applyConfig(LogData L) {
        restoreDefaults();
        List<String> off = new ArrayList<>();
        for (Class<?> k : CONFIG_CLASSES) {
            Map<String, String> m = L.config.get(k.getSimpleName());
            if (m == null) continue;
            for (Map.Entry<String, String> e : m.entrySet()) {
                try {
                    Field f = k.getField(e.getKey());
                    if (!Modifier.isStatic(f.getModifiers()) || Modifier.isFinal(f.getModifiers())) continue;
                    Class<?> t = f.getType();
                    if (t == double.class) f.setDouble(null, Double.parseDouble(e.getValue()));
                    else if (t == boolean.class) f.setBoolean(null, Boolean.parseBoolean(e.getValue()));
                    else if (t == int.class) f.setInt(null, Integer.parseInt(e.getValue()));
                    else if (t == long.class) f.setLong(null, Long.parseLong(e.getValue()));
                } catch (NoSuchFieldException | IllegalAccessException | NumberFormatException ex) {
                    // a value the current code no longer has
                }
            }
        }
        Map<String, String> hd = L.config.get("HiveDash");
        if (hd != null) {
            Map<String, Double> offValues = new LinkedHashMap<>();
            offValues.put("APPROACH_POWER", 0.0);
            offValues.put("CORNER_POWER", 0.0);
            offValues.put("EARLY_LOCK_RADIUS_IN", 0.0);
            offValues.put("LAUNCH_HOLD_SEC", AutoLaunch.ALIGNED_HOLD_SEC);
            offValues.put("LANE_HALF_WIDTH_IN", 22.75);
            offValues.put("FLOWER_CLEARANCE_IN", 0.0);
            for (Map.Entry<String, Double> e : offValues.entrySet()) {
                if (hd.containsKey(e.getKey())) continue;
                try {
                    HiveDash.class.getField(e.getKey()).setDouble(null, e.getValue());
                    off.add(e.getKey());
                } catch (NoSuchFieldException | IllegalAccessException ex) {
                    throw new RuntimeException(ex);
                }
            }
        }
        HiveDash.OVERLAY_ENABLED = false;
        HiveDash.DRY_RUN = false;
        ParkDash.DRY_RUN = false;
        return off;
    }

    // ================================================================ one simulated dash

    static final class Options {
        Model model;
        boolean fieldFrame = false;
        double accelMax = 0.0;
        double turnBoost = 0.0;
        double[] gainScale = {1.0, 1.0, 1.0};
        /** NaN: the logged battery volts row by row; otherwise this fixed value. */
        double fixedVolts = Double.NaN;
        double loopMs = Double.NaN;   // NaN: the logged loop time
        double delayMs = 0.0;
        double noiseIn = 0.1;
        boolean force = true;
        long seed = 1;
        double salvoSec = 1.3;
        double subStepSec = 0.005;
        String onlyCase = null;
        /** Only the logs whose name starts with this (for example 2026-11-05), or null for all. */
        String days = null;
        Path tracksDir = null;
    }

    static final class Result {
        final List<double[]> track = new ArrayList<>();   // t, x, y, h (true pose), fwd, left, turn
        double tArrive = Double.NaN, ax, ay;
        boolean forced, routeMatched;
        String plannedDesc = "";
        String refused = null;
        final List<RunLog.Event> events = new ArrayList<>();
    }

    static Field field(Class<?> k, String name) throws NoSuchFieldException {
        Field f = k.getDeclaredField(name);
        f.setAccessible(true);
        return f;
    }

    static Method method(Class<?> k, String name, Class<?>... args) throws NoSuchMethodException {
        Method m = k.getDeclaredMethod(name, args);
        m.setAccessible(true);
        return m;
    }

    static boolean sameRoute(double[][] a, double[][] b) {
        if (a.length != b.length) return false;
        for (int i = 1; i < a.length; i++) if (Math.hypot(a[i][0] - b[i][0], a[i][1] - b[i][1]) > 1.0) return false;
        return true;
    }

    /**
     * Puts the logged route into a dash that has just planned another one (today's planner is not the one the run had): the plan, leg 0 again with the start heading, and the
     * first loop's output driven again on the logged route. Only HiveDash / ParkDash private state is touched, by reflection; the follower code is the real one.
     */
    static void forceRoute(Object dash, Object input, Object output, double[][] wp, double startHeading) throws ReflectiveOperationException {
        Class<?> k = dash.getClass();
        Constructor<HiveRoute.Plan> ctor = HiveRoute.Plan.class.getDeclaredConstructor(boolean.class, String.class, double[][].class, boolean[].class, double.class, String.class);
        ctor.setAccessible(true);
        boolean[] al = new boolean[wp.length - 1];
        double len = 0.0;
        for (int i = 0; i + 1 < wp.length; i++) {
            double l = Math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1]);
            al[i] = l >= HiveDash.ALIGN_MIN_LEG_IN;
            len += l;
        }
        HiveRoute.Plan plan = ctor.newInstance(true, null, wp, al, len, "forced (the logged route)");
        field(k, "plan").set(dash, plan);
        field(k, "legIndex").setInt(dash, 0);
        field(k, "prevLegHeading").setDouble(dash, startHeading);
        if (k == HiveDash.class) method(k, "startLeg", HiveDash.Input.class).invoke(dash, input);
        else method(k, "startLeg").invoke(dash);
        method(output.getClass(), "reset").invoke(output);
        method(k, "drive", input.getClass()).invoke(dash, input);
    }

    static double[][] planOf(Object dash) throws ReflectiveOperationException {
        HiveRoute.Plan p = (HiveRoute.Plan) field(dash.getClass(), "plan").get(dash);
        return p == null ? null : p.waypoints;
    }

    /** Runs one dash from the case's start (or a free run when c.log is null) to the end of its window; the dash classes use the static values already set. */
    static Result simulate(Case c, Options o, boolean trace) throws ReflectiveOperationException {
        Result res = new Result();
        RunLog.clear();
        ElapsedTime.now = -1.0;
        HiveDash hd = new HiveDash(DEPTH, WIDTH, FROM_BACK);
        ParkDash pd = new ParkDash(DEPTH, WIDTH, FROM_BACK);
        HoldPosition hold = new HoldPosition();
        TargetLock lock = new TargetLock();
        RunLog.clear();
        ElapsedTime.now = 0.0;
        double dt = (Double.isNaN(o.loopMs) ? c.loopMs : o.loopMs) / 1000.0;
        double[] v0 = c.log == null ? null : preRoll(c, o);
        Plant plant = v0 == null ? newPlant(o, c.sx, c.sy, c.sh, c.v0f, c.v0l, c.v0w) : newPlant(o, c.sx, c.sy, c.sh, v0[0], v0[1], v0[2]);
        plant.setWalls();
        Pinpoint pin = new Pinpoint(o.delayMs / 1000.0, o.noiseIn, o.seed);
        pin.record(-1.0, plant);
        pin.record(0.0, plant);
        boolean blue = c.blue();
        double blueFwd = c.log == null ? -90.0 : c.log.cfgDouble("TeleOpCompetition", "BLUE_DRIVER_FORWARD_DEG", -90.0);
        double redFwd = c.log == null ? 90.0 : c.log.cfgDouble("TeleOpCompetition", "RED_DRIVER_FORWARD_DEG", 90.0);
        double driverForwardRad = Math.toRadians(blue ? blueFwd : redFwd);
        HiveTarget cell = c.hive ? HiveTarget.valueOf(c.target) : null;
        ParkDash.Corner corner = c.hive ? ParkDash.Corner.NONE : ParkDash.Corner.valueOf(c.target);
        boolean arrivedInLog = c.log == null || c.arrived();
        double releaseAt = arrivedInLog ? Double.MAX_VALUE : c.tCancel;
        HiveDash.Input dashIn = new HiveDash.Input();
        ParkDash.Input parkIn = new ParkDash.Input();
        HiveTarget openCell = cell != null ? cell : HiveTarget.BLUE_REAR;
        boolean lockOn = false;
        double launchBusyUntil = -1.0;
        double t = 0.0;
        boolean first = true;
        double hardStop = c.log == null ? 10.0 : c.windowEnd + 3.0;
        while (true) {
            boolean simDone = !Double.isNaN(res.tArrive) && t >= res.tArrive + COAST_WINDOW_SEC;
            if (t > hardStop || (t >= c.windowEnd - 1e-9 && (simDone || !arrivedInLog))) break;
            boolean held = t < releaseAt;
            double[] m = pin.read(t);
            Pose2d pose = new Pose2d(m[0], m[1], m[2]);
            PoseVelocity2d meas = new PoseVelocity2d(new Vector2d(m[3], m[4]), m[5]);
            boolean launchBusy = t < launchBusyUntil;

            parkIn.pose = pose;
            parkIn.measured = meas;
            parkIn.requested = hd.state() != HiveDash.State.IDLE || !held ? ParkDash.Corner.NONE : corner;
            parkIn.blue = blue;
            parkIn.driverForwardRad = driverForwardRad;
            parkIn.blockedWhy = null;
            parkIn.otherDashBusy = hd.state() != HiveDash.State.IDLE;
            ParkDash.Output park = pd.update(parkIn);

            dashIn.pose = pose;
            dashIn.measured = meas;
            dashIn.requestedCell = pd.state() != ParkDash.State.IDLE || !held ? null : cell;
            dashIn.blue = blue;
            dashIn.blockedWhy = null;
            dashIn.counterReadsEmpty = false;
            dashIn.hasElements = true;
            dashIn.lockAligned = lock.isAligned();
            dashIn.launchBusy = launchBusy;
            HiveDash.Output dash = hd.update(dashIn);

            if (first) {
                first = false;
                Object d = c.hive ? hd : pd;
                double[][] planned = planOf(d);
                boolean running = c.hive ? hd.state() != HiveDash.State.IDLE : pd.state() != ParkDash.State.IDLE;
                if (!running) {
                    res.refused = RunLog.EVENTS.isEmpty() ? "did not start" : RunLog.EVENTS.get(RunLog.EVENTS.size() - 1).text;
                    break;
                }
                double[][] logged = c.route();
                res.routeMatched = planned != null && sameRoute(planned, logged);
                res.plannedDesc = describeRoute(planned);
                if (!res.routeMatched && o.force && c.log != null) {
                    forceRoute(d, c.hive ? dashIn : parkIn, c.hive ? dash : park, logged, pose.heading.toDouble());
                    res.forced = true;
                }
            }

            double fwd = 0.0, left = 0.0, turn;
            if (dash.translation) {
                fwd = dash.forward;
                left = dash.left;
            }
            if (park.translation) {
                fwd = park.forward;
                left = park.left;
            }
            if (dash.overrideCell != null) openCell = dash.overrideCell;
            if (dash.lockWanted) {
                lockOn = true;
                turn = lock.turn(pose, meas.angVel, openCell, TargetLock.farCellExtraLeftIn(openCell, blue), 0.0, 0.0, dt);
            } else {
                if (lockOn) {
                    lockOn = false;
                    lock.release();
                }
                turn = dash.turnActive ? dash.turn : park.turnActive ? park.turn : 0.0;
            }
            if (dash.launchNow) launchBusyUntil = t + o.salvoSec;
            boolean dashDriving = (dash.translation && Math.abs(dash.forward) + Math.abs(dash.left) > 1e-3) || (park.translation && Math.abs(park.forward) + Math.abs(park.left) > 1e-3);
            HoldPosition.Output h = hold.update(pose, meas, launchBusy || pd.holdWanted(), dashDriving, !lockOn);
            if (h.active) {
                fwd = h.forward;
                left = h.left;
                if (h.turnActive) turn = h.turn;
            }
            res.track.add(new double[]{t, plant.x, plant.y, plant.h, fwd, left, turn});
            if (trace) {
                double[] lg = c.log == null ? null : interp(c.track, t);
                System.out.printf(Locale.US, "  t %5.2f  sim (%6.1f, %6.1f, %6.1f deg) %5.1f in/s  cmd (%+.2f, %+.2f, %+.2f)  %-8s%s%n", t, plant.x, plant.y, Math.toDegrees(plant.h),
                        Math.hypot(plant.v[0], plant.v[1]), fwd, left, turn, c.hive ? hd.state() : pd.state(),
                        lg == null ? "" : String.format(Locale.US, "  log (%6.1f, %6.1f, %6.1f deg)  diff %.1f in", lg[1], lg[2], Math.toDegrees(lg[3]), Math.hypot(lg[1] - plant.x, lg[2] - plant.y)));
            }
            // the plant runs to the next loop with this loop's command
            double volts = !Double.isNaN(o.fixedVolts) ? o.fixedVolts : c.log == null ? 12.5 : c.voltAt(c.t0 + t);
            int n = Math.max(1, (int) Math.ceil(dt / o.subStepSec));
            for (int s = 0; s < n; s++) {
                plant.step(fwd, left, turn, volts, dt / n);
                pin.record(t + dt * (s + 1) / n, plant);
            }
            t += dt;
            ElapsedTime.now = t;
            if (Double.isNaN(res.tArrive)) {
                for (int e = RunLog.EVENTS.size() - 1; e >= 0 && e >= RunLog.EVENTS.size() - 4; e--) {
                    Matcher am = ARRIVED.matcher(RunLog.EVENTS.get(e).text);
                    if (am.find()) {
                        res.tArrive = RunLog.EVENTS.get(e).t;
                        double[] at = interp(res.track, res.tArrive);
                        res.ax = at == null ? plant.x : at[1];
                        res.ay = at == null ? plant.y : at[2];
                        break;
                    }
                }
            }
        }
        res.events.addAll(RunLog.EVENTS);
        return res;
    }

    static String describeRoute(double[][] w) {
        if (w == null) return "none";
        if (w.length == 2) return "direct";
        StringBuilder b = new StringBuilder("via");
        for (int i = 1; i + 1 < w.length; i++) b.append(String.format(Locale.US, " (%.0f, %.0f)", w[i][0], w[i][1]));
        return b.toString();
    }

    // ================================================================ comparing a simulated dash with the log (Phase 1d)

    static double offRoute(double x, double y, double[][] w) {
        double best = Double.MAX_VALUE;
        for (int i = 0; i + 1 < w.length; i++) best = Math.min(best, pointSegment(x, y, w[i][0], w[i][1], w[i + 1][0], w[i + 1][1]));
        return best;
    }

    static double maxOffRoute(List<double[]> tr, double t1, double[][] w) {
        double m = 0.0;
        for (double[] r : tr) if (r[0] <= t1 + 1e-9) m = Math.max(m, offRoute(r[1], r[2], w));
        return m;
    }

    /** The most the track went past the goal: its largest distance from the goal over the coast window after the arrival. */
    static double overshoot(List<double[]> tr, double tA, double gx, double gy) {
        double m = 0.0;
        for (double t = tA; t <= tA + COAST_WINDOW_SEC + 1e-9; t += 0.01) {
            double[] s = interp(tr, t);
            if (s != null) m = Math.max(m, Math.hypot(s[1] - gx, s[2] - gy));
        }
        return m;
    }

    /** The highest speed (in/s) over [t0, t1], from the positions 0.1 s apart. */
    static double peakSpeed(List<double[]> tr, double t0, double t1) {
        double m = 0.0;
        for (double t = t0; t + 0.1 <= t1 + 1e-9; t += 0.05) {
            double[] a = interp(tr, t), b = interp(tr, t + 0.1);
            if (a != null && b != null) m = Math.max(m, Math.hypot(b[1] - a[1], b[2] - a[2]) / 0.1);
        }
        return m;
    }

    // the acceptance gate (the handoff's suggestions)
    static final double GATE_MEDIAN_IN = 4.0, GATE_ARRIVAL_IN = 3.0, GATE_SWING_IN = 5.0, GATE_OVERSHOOT_IN = 3.0;
    static final double MIN_DASH_SEC = 0.3, LOW_BATTERY_V = 9.0;

    static final class Verdict {
        Case c;
        Result r;
        boolean excluded;
        String excludedWhy = "";
        double medDiff, p90Diff, maxDiff, arrivalDiff = Double.NaN, arrivalDt = Double.NaN, osLog = Double.NaN, osSim = Double.NaN, offLog, offSim, peakLog, peakSim;
        Contacts conLog, conSim;
        double comparedUntil;
        final List<String> fails = new ArrayList<>();
        boolean medOk, arrivalOk = true, swingOk = true, overshootOk = true, contactsOk;
    }

    static Verdict judge(Case c, Result r) {
        Verdict v = new Verdict();
        v.c = c;
        v.r = r;
        double tEnd = Math.min(c.windowEnd, r.track.isEmpty() ? 0.0 : r.track.get(r.track.size() - 1)[0]);
        v.conLog = contacts(c.track, 0.0, c.windowEnd);
        v.conSim = contacts(r.track, 0.0, r.track.isEmpty() ? 0.0 : r.track.get(r.track.size() - 1)[0]);
        // the plant has no contact forces: once the real robot met a support or a flower the tracks part for that reason alone, so the track difference stops there
        for (double th : v.conLog.firstHit.values()) tEnd = Math.min(tEnd, th);
        v.comparedUntil = tEnd;
        List<Double> diffs = new ArrayList<>();
        for (double[] lr : c.track) {
            if (lr[0] > tEnd + 1e-9) break;
            double[] s = interp(r.track, lr[0]);
            if (s != null) diffs.add(Math.hypot(s[1] - lr[1], s[2] - lr[2]));
        }
        v.medDiff = diffs.isEmpty() ? Double.NaN : median(diffs);
        v.p90Diff = diffs.isEmpty() ? Double.NaN : percentile(diffs, 90);
        v.maxDiff = diffs.isEmpty() ? Double.NaN : java.util.Collections.max(diffs);
        double[][] w = c.route();
        double tRouteLog = c.arrived() ? c.tArrive : c.windowEnd, tRouteSim = !Double.isNaN(r.tArrive) ? r.tArrive : tEnd;
        v.offLog = maxOffRoute(c.track, tRouteLog, w);
        v.offSim = maxOffRoute(r.track, tRouteSim, w);
        v.peakLog = peakSpeed(c.track, 0.0, c.windowEnd);
        v.peakSim = peakSpeed(r.track, 0.0, r.track.isEmpty() ? 0.0 : r.track.get(r.track.size() - 1)[0]);
        if (c.arrived() && !Double.isNaN(r.tArrive)) {
            v.arrivalDiff = Math.hypot(c.ax - r.ax, c.ay - r.ay);
            v.arrivalDt = r.tArrive - c.tArrive;
            v.osLog = overshoot(c.track, c.tArrive, c.gx, c.gy);
            v.osSim = overshoot(r.track, r.tArrive, c.gx, c.gy);
        }
        if (r.refused != null) {
            v.excluded = true;
            v.excludedWhy = "the simulated dash did not start: " + r.refused;
        } else if (c.minVolt < LOW_BATTERY_V) {
            v.excluded = true;
            v.excludedWhy = String.format(Locale.US, "battery down to %.1f V", c.minVolt);
        } else if (c.windowEnd < MIN_DASH_SEC) {
            v.excluded = true;
            v.excludedWhy = String.format(Locale.US, "released after %.2f s", c.windowEnd);
        }
        v.medOk = Double.isNaN(v.medDiff) || v.medDiff <= GATE_MEDIAN_IN;
        if (!v.medOk) v.fails.add(String.format(Locale.US, "median %.1f in", v.medDiff));
        if (c.arrived()) {
            if (Double.isNaN(r.tArrive)) {
                v.arrivalOk = false;
                v.fails.add("no simulated arrival");
            } else {
                v.arrivalOk = v.arrivalDiff <= GATE_ARRIVAL_IN;
                if (!v.arrivalOk) v.fails.add(String.format(Locale.US, "arrival %.1f in off", v.arrivalDiff));
                v.overshootOk = Math.abs(v.osSim - v.osLog) <= GATE_OVERSHOOT_IN;
                if (!v.overshootOk) v.fails.add(String.format(Locale.US, "overshoot %.1f vs %.1f in", v.osSim, v.osLog));
            }
        }
        if (!c.via.isEmpty()) {
            v.swingOk = Math.abs(v.offSim - v.offLog) <= GATE_SWING_IN;
            if (!v.swingOk) v.fails.add(String.format(Locale.US, "off-route %.1f vs %.1f in", v.offSim, v.offLog));
        }
        v.contactsOk = v.conLog.touched().equals(v.conSim.touched());
        if (!v.contactsOk) v.fails.add("contacts " + v.conSim.touched() + " vs " + v.conLog.touched());
        return v;
    }

    // ================================================================ modes

    static void replay(Options o) throws Exception {
        List<LogData> logs = loadLogs();
        System.out.println(o.model.describe());
        System.out.printf(Locale.US, "loop %s, Pinpoint delay %.0f ms, noise %.2f in, velocity lag in the %s frame, grip limit %s, logged route forced in when today's planner differs: %s%n",
                Double.isNaN(o.loopMs) ? "as logged" : o.loopMs + " ms", o.delayMs, o.noiseIn, o.fieldFrame ? "field" : "body", o.accelMax > 0 ? o.accelMax + " in/s^2" : "none", o.force ? "yes" : "no");
        System.out.printf(Locale.US, "plant extras: turn boost %.2f, gain scales %s, battery %s%n", o.turnBoost, Arrays.toString(o.gainScale), Double.isNaN(o.fixedVolts) ? "as logged" : o.fixedVolts + " V");
        System.out.printf(Locale.US, "gate: median track difference <= %.0f in, arrival point within %.0f in, overshoot within %.0f in, max off-route within %.0f in (dashes with via points), the same contacts%n%n",
                GATE_MEDIAN_IN, GATE_ARRIVAL_IN, GATE_OVERSHOOT_IN, GATE_SWING_IN);
        List<Verdict> all = new ArrayList<>();
        String lastLog = null;
        for (LogData L : logs) {
            if (o.days != null && !L.name.startsWith(o.days)) continue;
            List<Case> cases = findCases(L);
            if (cases.isEmpty()) continue;
            List<String> off = applyConfig(L);
            for (Case c : cases) {
                if (o.onlyCase != null && !c.id().equals(o.onlyCase)) continue;
                if (!L.name.equals(lastLog)) {
                    lastLog = L.name;
                    System.out.printf(Locale.US, "== %s: %d dashes; power %.2f; features the run did not have yet, set off: %s%n", L.name, cases.size(), HiveDash.MAX_POWER, off.isEmpty() ? "none" : String.join(", ", off));
                    System.out.println("   t0      dash          route                  end        dur log/sim  diff med/p90/max  arrival dist,dt  overshoot log/sim  off-route log/sim  peak in/s  contacts log | sim   verdict");
                }
                applyConfig(L);
                boolean trace = o.onlyCase != null;
                Result r = simulate(c, o, trace);
                Verdict v = judge(c, r);
                all.add(v);
                printRow(v);
                if (o.tracksDir != null) writeTrack(o.tracksDir, c, r);
            }
        }
        summary(all);
    }

    static String fmt(double v, String f) {
        return Double.isNaN(v) ? "-" : String.format(Locale.US, f, v);
    }

    static String contactsText(Contacts con) {
        if (con.touched().isEmpty()) return String.format(Locale.US, "none (flower %.1f)", con.flowerClear());
        StringBuilder b = new StringBuilder();
        for (String n : con.touched()) b.append(b.length() == 0 ? "" : ", ").append(n).append(String.format(Locale.US, " %.1f", con.minClear.get(n)));
        return b.toString();
    }

    static void printRow(Verdict v) {
        Case c = v.c;
        Result r = v.r;
        String end = c.arrived() ? "arrived" : "cancel";
        String route = describeRoute(c.route()) + (r.forced ? "*" : "");
        double simDur = !Double.isNaN(r.tArrive) ? r.tArrive : (r.track.isEmpty() ? 0.0 : r.track.get(r.track.size() - 1)[0]);
        String verdict = v.excluded ? "excluded: " + v.excludedWhy : v.fails.isEmpty() ? "PASS" : "FAIL: " + String.join("; ", v.fails);
        System.out.printf(Locale.US, "   %-7.1f %-13s %-22s %-10s %4.2f/%4.2f    %4.1f/%4.1f/%4.1f    %5s,%6s     %5s/%-5s       %5.1f/%-5.1f        %3.0f/%-3.0f   %s | %s   %s%n",
                c.t0, (c.hive ? "" : "park ") + c.target, route, end, c.arrived() ? c.tArrive : c.windowEnd, simDur, v.medDiff, v.p90Diff, v.maxDiff,
                fmt(v.arrivalDiff, "%.1f"), fmt(v.arrivalDt, "%+.2f"), fmt(v.osLog, "%.1f"), fmt(v.osSim, "%.1f"), v.offLog, v.offSim, v.peakLog, v.peakSim,
                contactsText(v.conLog), contactsText(v.conSim), verdict);
    }

    static void summary(List<Verdict> all) {
        int n = 0, pass = 0, med = 0, arr = 0, arrN = 0, os = 0, sw = 0, swN = 0, con = 0, excluded = 0, forced = 0;
        List<Double> meds = new ArrayList<>(), arrs = new ArrayList<>(), osErr = new ArrayList<>(), arrDt = new ArrayList<>();
        for (Verdict v : all) {
            if (v.r.forced) forced++;
            if (v.excluded) {
                excluded++;
                continue;
            }
            n++;
            if (v.fails.isEmpty()) pass++;
            if (v.medOk) med++;
            if (!Double.isNaN(v.medDiff)) meds.add(v.medDiff);
            if (v.c.arrived()) {
                arrN++;
                if (v.arrivalOk) arr++;
                if (v.overshootOk && !Double.isNaN(v.osSim)) os++;
                if (!Double.isNaN(v.arrivalDiff)) {
                    arrs.add(v.arrivalDiff);
                    osErr.add(v.osSim - v.osLog);
                    arrDt.add(v.arrivalDt);
                }
            }
            if (!v.c.via.isEmpty()) {
                swN++;
                if (v.swingOk) sw++;
            }
            if (v.contactsOk) con++;
        }
        System.out.printf(Locale.US, "%n== GATE: %d of %d dashes pass (%d excluded: short, low battery or not started; %d had the logged route put in)%n", pass, n, excluded, forced);
        System.out.printf(Locale.US, "   median track difference <= %.0f in: %d of %d (the medians: median %.1f in, 90th %.1f in)%n", GATE_MEDIAN_IN, med, n, median(meds), percentile(meds, 90));
        if (!arrs.isEmpty()) {
            System.out.printf(Locale.US, "   arrival point within %.0f in: %d of %d (median %.1f in); arrival time sim - log: median %+.2f s, 10th to 90th %+.2f to %+.2f s%n", GATE_ARRIVAL_IN, arr, arrN,
                    median(arrs), median(arrDt), percentile(arrDt, 10), percentile(arrDt, 90));
            System.out.printf(Locale.US, "   overshoot within %.0f in: %d of %d (sim - log: median %+.1f in, 10th to 90th %+.1f to %+.1f in)%n", GATE_OVERSHOOT_IN, os, arrN, median(osErr),
                    percentile(osErr, 10), percentile(osErr, 90));
        }
        System.out.printf(Locale.US, "   off-route within %.0f in (dashes with via points): %d of %d%n", GATE_SWING_IN, sw, swN);
        System.out.printf(Locale.US, "   the same contacts: %d of %d%n", con, n);
        List<Double> offErr = new ArrayList<>();
        for (Verdict v : all) if (!v.excluded && !v.c.via.isEmpty()) offErr.add(Math.abs(v.offSim - v.offLog));
        List<Double> osAbs = new ArrayList<>();
        for (double e : osErr) osAbs.add(Math.abs(e));
        double meanMed = 0.0;
        for (double m : meds) meanMed += m / Math.max(1, meds.size());
        System.out.printf(Locale.US, "   SCORE mean median difference %.2f in, mean |off-route error| %.2f in, mean |overshoot error| %.2f in, mean |arrival error| %.2f in, passes %d of %d%n",
                meanMed, mean(offErr), mean(osAbs), mean(arrs), pass, n);
        System.out.println("   named cases (the handoff's evidence):");
        for (Verdict v : all) {
            boolean corner = !v.c.via.isEmpty() && v.c.hive && v.c.target.equals("BLUE_AUDIENCE");
            boolean flower = v.conLog.touched().stream().anyMatch(s -> s.contains("flower"));
            if (!corner && !flower) continue;
            System.out.printf(Locale.US, "     %s %s %s: off-route log %.1f / sim %.1f in; contacts log %s | sim %s%s%n", v.c.id(), v.c.target, describeRoute(v.c.route()), v.offLog, v.offSim,
                    contactsText(v.conLog), contactsText(v.conSim), v.excluded ? " (excluded: " + v.excludedWhy + ")" : "");
        }
    }

    static void writeTrack(Path dir, Case c, Result r) throws IOException {
        Files.createDirectories(dir);
        Path f = dir.resolve(c.id().replace('/', '_').replace(':', '_') + ".csv");
        try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(f))) {
            w.println("source,t,x,y,heading_deg,cmd_fwd,cmd_left,cmd_turn");
            for (double[] s : c.track) w.printf(Locale.US, "log,%.3f,%.2f,%.2f,%.1f,,,%n", s[0], s[1], s[2], Math.toDegrees(s[3]));
            for (double[] s : r.track) w.printf(Locale.US, "sim,%.3f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f%n", s[0], s[1], s[2], Math.toDegrees(s[3]), s[4], s[5], s[6]);
        }
    }

    /** The drive model alone on the logged commands, the windows and figures of identify_drive.py (to check this port against the Python fit). */
    static void plantCheck(Options o, boolean dashOnly) throws IOException {
        final int W = 8;
        List<double[][]> wins = new ArrayList<>();
        for (LogData L : loadLogs()) {
            int n = L.rows.size();
            if (n == 0) continue;
            boolean[] keep = new boolean[n];
            if (!dashOnly) Arrays.fill(keep, true);
            else {
                Double cur = null;
                for (int e = 0; e < L.evT.size(); e++) {
                    String m = L.evText.get(e);
                    double tt = L.evT.get(e);
                    if (m.startsWith("HIVE DASH start") || m.startsWith("PARK DASH start")) cur = tt;
                    else if (cur != null && (m.startsWith("HIVE DASH arrived") || m.startsWith("PARK DASH arrived") || (m.contains("cancelled") && m.contains("DASH")))) {
                        for (int i = 0; i < n; i++) if (L.rows.get(i)[0] >= cur - 0.05 && L.rows.get(i)[0] <= tt + 0.3) keep[i] = true;
                        cur = null;
                    }
                }
            }
            for (int i = 1; i < n - W; i++) {
                if (!keep[i]) continue;
                double dmin = Double.MAX_VALUE, dmax = -Double.MAX_VALUE, cmax = 0.0;
                boolean backup = false;
                for (int k = 0; k < W; k++) {
                    double[] r = L.rows.get(i + k);
                    if (k > 0) {
                        double d = r[0] - L.rows.get(i + k - 1)[0];
                        dmin = Math.min(dmin, d);
                        dmax = Math.max(dmax, d);
                    }
                    if (L.modes.get(i + k).equals("backup")) backup = true;
                    cmax = Math.max(cmax, Math.max(Math.abs(r[6]), Math.max(Math.abs(r[7]), Math.abs(r[8]))));
                }
                if (dmin < 0.03 || dmax > 0.25 || backup || cmax < 0.12) continue;
                double[][] w = new double[W][];
                for (int k = 0; k < W; k++) w[k] = L.rows.get(i + k).clone();
                for (int k = 1; k < W; k++) {   // numpy.unwrap
                    double dd = w[k][5] - w[k - 1][5];
                    double ddmod = ((dd + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
                    if (ddmod == -Math.PI && dd > 0) ddmod = Math.PI;
                    if (Math.abs(dd) >= Math.PI) w[k][5] = w[k - 1][5] + ddmod;
                    else w[k][5] = w[k - 1][5] + dd;
                }
                wins.add(w);
            }
        }
        System.out.println(o.model.describe() + (o.fieldFrame ? "; lag in the field frame" : "") + (o.accelMax > 0 ? "; grip limit " + o.accelMax + " in/s^2" : ""));
        System.out.printf(Locale.US, "windows: %d (%s)%n", wins.size(), dashOnly ? "dash rows only" : "all TeleOp driving rows");
        double se = 0.0, base = 0.0;
        long cnt = 0;
        int[] steps = {2, 4, 7};
        List<List<Double>> pos = new ArrayList<>(), hdg = new ArrayList<>(), tim = new ArrayList<>();
        for (int s = 0; s < 3; s++) {
            pos.add(new ArrayList<>());
            hdg.add(new ArrayList<>());
            tim.add(new ArrayList<>());
        }
        for (double[][] w : wins) {
            double d0 = w[1][0] - w[0][0];
            double dx = w[1][3] - w[0][3], dy = w[1][4] - w[0][4], dh = w[1][5] - w[0][5], hm = 0.5 * (w[1][5] + w[0][5]);
            Plant p = newPlant(o, w[0][3], w[0][4], w[0][5], (dx * Math.cos(hm) + dy * Math.sin(hm)) / d0, (-dx * Math.sin(hm) + dy * Math.cos(hm)) / d0, dh / d0);
            for (int k = 1; k < W; k++) {
                double[] c = w[k - 1];
                p.step(c[6], c[7], c[8], Double.isNaN(o.fixedVolts) ? c[2] : o.fixedVolts, w[k][0] - w[k - 1][0]);
                double ex = p.x - w[k][3], ey = p.y - w[k][4], eh = (p.h - w[k][5]) * 12.0;
                se += ex * ex + ey * ey + eh * eh;
                double bx = w[k][3] - w[0][3], by = w[k][4] - w[0][4], bh = (w[k][5] - w[0][5]) * 12.0;
                base += bx * bx + by * by + bh * bh;
                cnt += 3;
                for (int s = 0; s < 3; s++) {
                    if (k != steps[s]) continue;
                    pos.get(s).add(Math.hypot(ex, ey));
                    hdg.get(s).add(Math.toDegrees(Math.abs(p.h - w[k][5])));
                    tim.get(s).add(w[k][0] - w[0][0]);
                }
            }
        }
        System.out.printf(Locale.US, "do-nothing rms (pose error within a window): %.2f in%n", Math.sqrt(base / cnt));
        System.out.printf(Locale.US, "model rms (pose error within a window): %.2f in%n", Math.sqrt(se / cnt));
        for (int s = 0; s < 3; s++) {
            System.out.printf(Locale.US, "after %d steps (%.2f s): position median %.2f in, 90th %.2f; heading median %.1f deg, 90th %.1f%n", steps[s], median(tim.get(s)), median(pos.get(s)),
                    percentile(pos.get(s), 90), median(hdg.get(s)), percentile(hdg.get(s), 90));
        }
    }

    /** One dash from a pose at rest with today's values (Phase 2 and later). */
    static void freeRun(Options o, double x, double y, double deg, String target, boolean red) throws Exception {
        restoreDefaults();
        Case c = new Case();
        c.hive = !target.equals("LEFT") && !target.equals("RIGHT");
        c.target = target;
        c.sx = x;
        c.sy = y;
        c.sh = Math.toRadians(deg);
        if (c.hive) {
            HiveTarget cell = HiveTarget.valueOf(target);
            c.gx = AutoLaunch.centerX(cell);
            c.gy = AutoLaunch.centerY(cell);
        } else {
            double[] g = ParkDash.cornerXY(ParkDash.Corner.valueOf(target), !red, Math.toRadians(red ? 90.0 : -90.0));
            c.gx = g[0];
            c.gy = g[1];
        }
        c.windowEnd = 0.0;
        c.loopMs = 31.0;
        o.force = false;
        Result r = simulate(c, o, true);
        for (RunLog.Event e : r.events) System.out.printf(Locale.US, "E,%.3f,%s%n", e.t, e.text);
        Contacts con = contacts(r.track, 0.0, r.track.isEmpty() ? 0.0 : r.track.get(r.track.size() - 1)[0]);
        System.out.printf(Locale.US, "route %s; arrival %s; contacts %s; least clearance to a flower %.1f in, to a support %.1f in, to a wall %.1f in%n", r.plannedDesc,
                Double.isNaN(r.tArrive) ? "none" : String.format(Locale.US, "after %.2f s at (%.1f, %.1f)", r.tArrive, r.ax, r.ay), con.touched(), con.flowerClear(), con.supportClear(), con.wallClear);
    }

    /**
     * The drive characterization routine (DriveTestRoutine, the same class the robot runs) from the blue start against the plant, with the deadman held throughout:
     * the time it takes, what the geofence stopped, and what the robot came near (the supports, the flowers, the walls and the center line).
     */
    static void driveTest(Options o) {
        restoreDefaults();
        RunLog.clear();
        List<DriveTestRoutine.Test> tests = DriveTestRoutine.standardTests();
        DriveTestRoutine routine = new DriveTestRoutine(tests, 1, DEPTH, WIDTH, FROM_BACK);
        double sx = -(WALL - FROM_BACK), sy = 16.0;
        Plant plant = newPlant(o, sx, sy, 0.0, 0.0, 0.0, 0.0);
        plant.setWalls();
        Pinpoint pin = new Pinpoint(o.delayMs / 1000.0, o.noiseIn, o.seed);
        double dt = (Double.isNaN(o.loopMs) ? 15.0 : o.loopMs) / 1000.0;
        pin.record(-1.0, plant);
        pin.record(0.0, plant);
        List<double[]> track = new ArrayList<>();
        double t = 0.0, minY = Double.MAX_VALUE;
        DriveTestRoutine.Output out = null;
        double[] testStart = new double[tests.size() + 1];
        double[] runTime = new double[tests.size() + 1];
        HiveRoute.Params p = robotParams();
        while (t < 900.0) {
            ElapsedTime.now = t;
            double[] m = pin.read(t);
            out = routine.update(t, m[0], m[1], m[2], m[3], m[4], m[5], true, false);
            if (out.phase == DriveTestRoutine.Phase.DONE) break;
            if (testStart[out.test] == 0.0) testStart[out.test] = t;
            if (out.phase == DriveTestRoutine.Phase.RUN) runTime[out.test] += dt;
            track.add(new double[]{t, plant.x, plant.y, plant.h});
            double[] k = HiveRoute.poseRect(plant.x, plant.y, plant.h, p, 0.0).corners();
            for (int i = 1; i < 8; i += 2) minY = Math.min(minY, k[i]);
            int n = Math.max(1, (int) Math.ceil(dt / o.subStepSec));
            for (int s = 0; s < n; s++) {
                plant.step(out.forward, out.left, out.turn, Double.isNaN(o.fixedVolts) ? 12.5 : o.fixedVolts, dt / n);
                pin.record(t + dt * (s + 1) / n, plant);
            }
            t += dt;
        }
        System.out.println(o.model.describe() + (o.fieldFrame ? "; field frame" : "") + (o.accelMax > 0 ? "; grip " + o.accelMax : "") + "; loop " + Math.round(dt * 1000) + " ms");
        System.out.printf(Locale.US, "%-4s %-62s %7s %7s%n", "test", "name", "starts", "driving");
        for (int i = 1; i <= tests.size(); i++) System.out.printf(Locale.US, "%-4d %-62s %6.1fs %6.2fs%n", i, tests.get(i - 1).name, testStart[i], runTime[i]);
        for (RunLog.Event e : RunLog.EVENTS) if (e.text.contains("GEOFENCE") || e.text.contains("not reached") || e.text.contains("all done")) System.out.printf(Locale.US, "  %.1f s: %s%n", e.t, e.text);
        Contacts con = contacts(track, 0.0, t);
        System.out.printf(Locale.US, "total %.0f s (%.1f min) from the start pose to the last test; %d finished, %d stopped by the geofence%n", t, t / 60.0, routine.finished(), routine.aborted());
        System.out.printf(Locale.US, "contacts %s; least clearance to a support %.1f in, to a flower %.1f in, to a wall %.1f in; lowest footprint y %.1f in (the center line is 0)%n",
                con.touched(), con.supportClear(), con.flowerClear(), con.wallClear, minY);
    }

    // ================================================================ helpers and main

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static double normalize(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    static double mean(List<Double> v) {
        double s = 0.0;
        for (double x : v) s += x;
        return v.isEmpty() ? Double.NaN : s / v.size();
    }

    static double median(List<Double> v) {
        return percentile(v, 50);
    }

    /** numpy.percentile (linear). */
    static double percentile(List<Double> v, double q) {
        if (v.isEmpty()) return Double.NaN;
        double[] a = new double[v.size()];
        for (int i = 0; i < a.length; i++) a[i] = v.get(i);
        Arrays.sort(a);
        double pos = q / 100.0 * (a.length - 1);
        int lo = (int) Math.floor(pos), hi = Math.min(a.length - 1, lo + 1);
        return a[lo] + (pos - lo) * (a[hi] - a[lo]);
    }

    /** The "tuned" plant (2026-11-06): grip limit in/s^2, turn boost, fixed battery volts; with the dash fit and the lag in the field frame. */
    static final double TUNED_ACCEL = 150.0, TUNED_TURN_BOOST = 1.5, TUNED_VOLTS = 12.5;

    public static void main(String[] args) throws Exception {
        Options o = new Options();
        String modelArg = "dash";
        // the default plant is the tuned one; --plant fitted gives identify_drive.py's model alone, and the single options after it override either
        o.accelMax = TUNED_ACCEL;
        o.fieldFrame = true;
        o.turnBoost = TUNED_TURN_BOOST;
        o.fixedVolts = TUNED_VOLTS;
        List<String> rest = new ArrayList<>();
        boolean red = false;
        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--model": modelArg = args[++i]; break;
                case "--loop-ms": o.loopMs = Double.parseDouble(args[++i]); break;
                case "--delay-ms": o.delayMs = Double.parseDouble(args[++i]); break;
                case "--noise": o.noiseIn = Double.parseDouble(args[++i]); break;
                case "--frame": o.fieldFrame = args[++i].equals("field"); break;
                case "--accel": o.accelMax = Double.parseDouble(args[++i]); o.fieldFrame = true; break;
                case "--plant":
                    // fitted: identify_drive.py's model as it is; tuned: the dash fit plus the three extras chosen on the 11-05 logs and checked on the 11-06 logs (design/dash-planner-grid.md)
                    if (args[++i].equals("tuned")) {
                        modelArg = "dash";
                        o.accelMax = TUNED_ACCEL;
                        o.fieldFrame = true;
                        o.turnBoost = TUNED_TURN_BOOST;
                        o.fixedVolts = TUNED_VOLTS;
                    } else {
                        o.accelMax = 0.0;
                        o.fieldFrame = false;
                        o.turnBoost = 0.0;
                        o.fixedVolts = Double.NaN;
                    }
                    break;
                case "--turn-boost": o.turnBoost = Double.parseDouble(args[++i]); break;
                case "--volts": o.fixedVolts = Double.parseDouble(args[++i]); break;
                case "--gains": {
                    String[] g = args[++i].split(",");
                    for (int k = 0; k < 3; k++) o.gainScale[k] = Double.parseDouble(g[k]);
                    break;
                }
                case "--no-force": o.force = false; break;
                case "--case": o.onlyCase = args[++i]; break;
                case "--days": o.days = args[++i]; break;
                case "--tracks": o.tracksDir = Paths.get(args[++i]); break;
                case "--seed": o.seed = Long.parseLong(args[++i]); break;
                case "--red": red = true; break;
                default: rest.add(args[i]);
            }
        }
        o.model = Model.of(modelArg);
        String mode = rest.isEmpty() ? "replay" : rest.get(0);
        switch (mode) {
            case "replay":
                replay(o);
                break;
            case "plant":
                plantCheck(o, rest.size() > 1 && rest.get(1).equals("dash"));
                break;
            case "drivetest":
                driveTest(o);
                break;
            case "run":
                freeRun(o, Double.parseDouble(rest.get(1)), Double.parseDouble(rest.get(2)), Double.parseDouble(rest.get(3)), rest.get(4), red);
                break;
            default:
                System.out.println("usage: DashSim [replay | plant [dash] | run x y deg TARGET] [--model dash|all|file.npy] [--loop-ms N] [--delay-ms N] [--noise IN] [--frame body|field] [--no-force] [--case FILE:T0] [--tracks DIR] [--red]");
        }
    }
}
