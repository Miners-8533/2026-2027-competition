import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AutoLaunch;
import org.firstinspires.ftc.teamcode.drive.DriveTestRoutine;
import org.firstinspires.ftc.teamcode.drive.GridDash;
import org.firstinspires.ftc.teamcode.drive.GridFollower;
import org.firstinspires.ftc.teamcode.drive.GridRoutes;
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

    /**
     * The WHEEL model (tools/dash/fit_drivetest.py, fitted 2026-11-07 on the Drive Characterization run 100246). The wheel powers p are split into the body command
     * (forward, strafe, turn); each axis loses its DEAD ZONE (d_f, d_t; the strafe's d_l fading out by vFade in/s of body speed, by default never); the wheel powers
     * q of the translation set each wheel's target speed g_i (V / 12) (sign(q) F(|q|) +- kw K_t turn), F a speed curve through KNOTS (the forward steps' steady speeds,
     * measured), the turn linear (K_t rad/s per unit, the spins' steady rates); a wheel
     * at zero power BRAKES (du/dt = -u / tau_b - a_b sign u, a_b per side), one driven slower than it runs COASTS (du/dt = -a_c sign u - (u - target) / tau_c, held at
     * test 10's measurement), otherwise it DRIVES (du/dt = (target - u) / tau_u); the wheel speeds and accelerations map to the body through the mecanum kinematics
     * (strafe ratio s, kw in of wheel travel per radian; the sideways response is sa times slower); the translation accel is at most A_up while it speeds the robot up
     * and A_down while it slows it, less YAW_GRIP |aw| kw; the momentum keeps its direction in the field. The terminal voltage SAGS: V = V_rest - SAG_K sum |p V - 12 u / 95 g|.
     */
    static final class Wheel {
        /**
         * fit_drivetest.py (2026-11-07, second fit) on 100246_drivetest.log: the forward dead zone and speed curve, the turn line and the coast held at their
         * measurements, the strafe dead zone always on, YAW_GRIP 0.3. Order: g_lf g_lb g_rb F15 F3 F6 F85 F10 d_f d_l d_t tau_u tau_b a_bL a_bR a_c tau_c s kw
         * A_up A_down sa K_t.
         */
        static final double[] FIT_1107 = {1.008013374428489, 0.9845810994269959, 1.017590772636981, 15.300000999999998, 30.599999000000004, 59.79999900000001, 78.09999900000001, 89.49999900000002, 0.07200099999999536, 0.12537721125266632, 0.09799900000000002, 0.16925386279773574, 0.1043207507885011, 70.66926985210422, 43.14545003195509, 37.00000099999999, 4.999999000000001, 0.8027209798466017, 8.672628920339566, 146.52146254035307, 185.4975331216584, 0.9037763564867982, 9.000000999999997};
        static final double[] KNOTS = {0.0, 0.15, 0.3, 0.6, 0.85, 1.0};
        static final double SAG_K = 0.0551;
        /** The strafe dead zone fades out by this body speed (in/s); 1e9: always on (the fit's choice: the strafe steps are 0.15 short while rolling). */
        static double vFade = 1e9;
        final String name;
        final double[] g = new double[4], f = new double[6];
        final double dF, dL, dT, tauU, tauB, aBL, aBR, aC, tauC, s, kw, aUp, aDown, sa, kT;

        Wheel(String name, double[] th) {
            if (th.length != 23) throw new IllegalArgumentException("a wheel model has 23 values, this one " + th.length + " (an old fit?)");
            this.name = name;
            g[0] = th[0];
            g[1] = th[1];
            g[2] = th[2];
            g[3] = 4.0 - th[0] - th[1] - th[2];
            for (int k = 0; k < 5; k++) f[k + 1] = th[3 + k];
            dF = th[8];
            dL = th[9];
            dT = th[10];
            tauU = th[11];
            tauB = th[12];
            aBL = th[13];
            aBR = th[14];
            aC = th[15];
            tauC = th[16];
            s = th[17];
            kw = th[18];
            aUp = th[19];
            aDown = th[20];
            sa = th[21];
            kT = th[22];
        }

        /** The speed curve F at |q| (in/s at 12 V for the average wheel). */
        double curve(double q) {
            q = Math.abs(q);
            for (int k = 0; k + 1 < KNOTS.length; k++) {
                if (q <= KNOTS[k + 1]) return f[k] + (f[k + 1] - f[k]) * (q - KNOTS[k]) / (KNOTS[k + 1] - KNOTS[k]);
            }
            return f[f.length - 1];
        }

        static double dead(double c, double dz) {
            return Math.signum(c) * Math.max(0.0, Math.abs(c) - dz);
        }

        String describe() {
            return String.format(Locale.US, "wheel model %s: speed curve %.1f %.1f %.1f %.1f %.1f at %s, wheel shares %.3f %.3f %.3f %.3f, dead zones %.3f / %.3f / %.3f, rise %.3f s, brake %.3f s + %.0f / %.0f, coast %.0f + 1/%.2f s, strafe %.3f (response %.2f), kw %.2f, turn %.2f rad/s per unit, accel %.0f up / %.0f down, yaw grip %.2f",
                    name, f[1], f[2], f[3], f[4], f[5], Arrays.toString(KNOTS), g[0], g[1], g[2], g[3], dF, dL, dT, tauU, tauB, aBL, aBR, aC, tauC, s, sa, kw, kT, aUp, aDown, YAW_GRIP);
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
        /** Not null: the wheel model instead of the per-axis one; the volts given to step() are then the REST volts when sag is on (the terminal volts otherwise). */
        Wheel wheel;
        boolean sag = true;
        /** The wheel model's terminal voltage in the last step (for a trace). */
        double lastVolts = Double.NaN;

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

        /** One step without the walls (identify_drive.py's model, or the wheel model). */
        void stepFree(double cf, double cl, double ct, double volts, double d) {
            if (wheel != null) {
                stepWheel(cf, cl, ct, volts, d);
                return;
            }
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

        /** One step of the wheel model (fit_drivetest.py's sim_wheel, explicit Euler, the caller's step of at most 5 ms). */
        void stepWheel(double cf, double cl, double ct, double volts, double d) {
            Wheel m = wheel;
            double[] p = {cf - cl - ct, cf + cl - ct, cf - cl + ct, cf + cl + ct};
            double mx = Math.max(1.0, Math.max(Math.max(Math.abs(p[0]), Math.abs(p[1])), Math.max(Math.abs(p[2]), Math.abs(p[3]))));
            for (int i = 0; i < 4; i++) p[i] /= mx;
            double vf = v[0], vl = v[1], w = v[2];
            // the body command from the wheel powers, each axis less its dead zone
            double bf = Wheel.dead((p[0] + p[1] + p[2] + p[3]) / 4.0, m.dF);
            double bt = Wheel.dead((-p[0] - p[1] + p[2] + p[3]) / 4.0, m.dT);
            double bl = Wheel.dead((-p[0] + p[1] - p[2] + p[3]) / 4.0, m.dL * Math.max(0.0, 1.0 - Math.hypot(vf, vl) / Wheel.vFade));
            double[] q = {bf - bl, bf + bl, bf - bl, bf + bl};
            double[] rot = {-bt, -bt, bt, bt};
            double[] u = {vf - vl / m.s - m.kw * w, vf + vl / m.s - m.kw * w, vf - vl / m.s + m.kw * w, vf + vl / m.s + m.kw * w};
            double vt = volts;
            if (sag) {
                for (int it = 0; it < 3; it++) {
                    double sum = 0.0;
                    for (int i = 0; i < 4; i++) sum += Math.abs(p[i] * vt - 12.0 * u[i] / (95.0 * m.g[i]));
                    vt = Math.max(6.0, volts - Wheel.SAG_K * sum);
                }
            }
            lastVolts = vt;
            double[] du = new double[4];
            for (int i = 0; i < 4; i++) {
                double tgt = m.g[i] * (vt / 12.0) * (Math.signum(q[i]) * m.curve(q[i]) + rot[i] * m.kT * m.kw);
                if (Math.abs(p[i]) < 0.005) {
                    double ab = i < 2 ? m.aBL : m.aBR;
                    du[i] = -u[i] / m.tauB - ab * Math.signum(u[i]);
                    if (Math.abs(du[i] * d) > Math.abs(u[i])) du[i] = -u[i] / d;   // stops at zero
                } else if (p[i] * u[i] > 0 && Math.abs(tgt) < Math.abs(u[i])) {
                    du[i] = -m.aC * Math.signum(u[i]) - (u[i] - tgt) / m.tauC;
                    if (Math.abs(du[i] * d) > Math.abs(u[i] - tgt)) du[i] = -(u[i] - tgt) / d;   // does not drop below the target
                } else {
                    du[i] = (tgt - u[i]) / m.tauU;
                }
            }
            double af = (du[0] + du[1] + du[2] + du[3]) / 4.0;
            double al = m.sa * m.s * (-du[0] + du[1] - du[2] + du[3]) / 4.0;
            double aw = (-du[0] - du[1] + du[2] + du[3]) / (4.0 * m.kw);
            double mag = Math.hypot(af, al), lim = af * vf + al * vl >= 0.0 ? m.aUp : m.aDown;
            // the turn uses the same tires: the translation gets what is left of the limit after |aw| kw YAW_GRIP
            if (YAW_GRIP > 0.0) lim = Math.max(0.3 * lim, lim - YAW_GRIP * Math.abs(aw) * m.kw);
            if (mag > lim) {
                af *= lim / mag;
                al *= lim / mag;
            }
            double nf = vf + (af + w * vl) * d, nl = vl + (al - w * vf) * d, nw = w + aw * d;
            double hm = h + 0.5 * (w + nw) * d * 0.5;
            double mf = 0.5 * (vf + nf), ml = 0.5 * (vl + nl);
            x += (mf * Math.cos(hm) - ml * Math.sin(hm)) * d;
            y += (mf * Math.sin(hm) + ml * Math.cos(hm)) * d;
            h += 0.5 * (w + nw) * d;
            v[0] = nf;
            v[1] = nl;
            v[2] = nw;
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
        if (o.wheel != null) {
            p.wheel = o.wheel;
            p.sag = o.sag;
        }
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

    static final Class<?>[] CONFIG_CLASSES = {HiveDash.class, ParkDash.class, AutoLaunch.class, TargetLock.class, HoldPosition.class, GridDash.class, GridFollower.class};
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

    /** --set Class.FIELD=value: Dashboard values put in after every restore of the defaults (a trial without editing the source). */
    static final List<String[]> OVERRIDES = new ArrayList<>();

    static void restoreDefaults() {
        try {
            for (Map.Entry<Field, Object> e : DEFAULTS.entrySet()) e.getKey().set(null, e.getValue());
            for (String[] ov : OVERRIDES) {
                for (Class<?> k : CONFIG_CLASSES) {
                    if (!k.getSimpleName().equals(ov[0])) continue;
                    Field f = k.getField(ov[1]);
                    if (f.getType() == double.class) f.setDouble(null, Double.parseDouble(ov[2]));
                    else if (f.getType() == int.class) f.setInt(null, Integer.parseInt(ov[2]));
                    else if (f.getType() == boolean.class) f.setBoolean(null, Boolean.parseBoolean(ov[2]));
                }
            }
        } catch (IllegalAccessException | NoSuchFieldException e) {
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
        /** True: the real HiveDash and ParkDash run with GRID_ROUTES on (after the defaults or a log's CONFIG values). */
        boolean grid = false;
        /** Not null: the wheel model (--plant wheel); the volts are then rest volts when sag is on. */
        Wheel wheel = null;
        boolean sag = true;

        String describe() {
            return wheel != null ? wheel.describe() + (sag ? "; battery sag on, rest " : "; no sag, terminal ") + (Double.isNaN(fixedVolts) ? "volts from the log" : fixedVolts + " V")
                    : model.describe() + (fieldFrame ? "; field frame" : "") + (accelMax > 0 ? "; grip " + accelMax : "") + (turnBoost != 0 ? "; turn boost " + turnBoost : "")
                    + (Double.isNaN(fixedVolts) ? "" : "; " + fixedVolts + " V");
        }
        /** survey: the red alliance (the same starts turned 180 deg, the red cells and corners). */
        boolean red = false;
    }

    static final class Result {
        final List<double[]> track = new ArrayList<>();   // t, x, y, h (true pose), fwd, left, turn
        double tArrive = Double.NaN, ax, ay;
        boolean forced, routeMatched;
        String plannedDesc = "";
        GridRoutes.Route route;
        /** The route the dash planned (field frame), for a picture. */
        double[][] planned;
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
        if (o.grid) {
            HiveDash.GRID_ROUTES = true;
            ParkDash.GRID_ROUTES = true;
        }
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
                res.planned = planned;
                boolean gridRoute = field(d.getClass(), "grid").get(d) != null;
                if (gridRoute) res.plannedDesc = "grid " + res.plannedDesc;
                if (!res.routeMatched && o.force && c.log != null && !gridRoute) {
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
        System.out.println(o.describe());
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
        System.out.println(o.describe());
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

    /** A dash from a pose at rest with today's values (no log): a cell such as BLUE_AUDIENCE, or LEFT / RIGHT for a park corner. */
    static Case freeCase(double x, double y, double deg, String target, boolean red) {
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
        return c;
    }

    /**
     * Phase 2's baseline: today's planner and follower (HiveRoute, HiveDash, ParkDash with today's values) from a grid of start poses at rest, every pitch inches and
     * four headings, to the two blue cells and the two blue park corners (red is the same turned 180 deg). A start whose footprint touches a support, a flower or a
     * wall is skipped (decision 2 of the design). Prints per target the time to arrive, the refusals and the contacts, the worst starts, and writes a CSV.
     */
    /**
     * One dash on the GRID planner (GridRoutes + GridFollower, the classes the robot would run) from a free start (blue frame), with the TeleOp's lock turn when the
     * follower wants it; the same plant and Pinpoint as {@link #simulate}.
     */
    static Result simulateGrid(Case c, Options o, GridRoutes.Table table) {
        Result res = new Result();
        RunLog.clear();
        ElapsedTime.now = 0.0;
        double dt = (Double.isNaN(o.loopMs) ? c.loopMs : o.loopMs) / 1000.0;
        Plant plant = newPlant(o, c.sx, c.sy, c.sh, 0.0, 0.0, 0.0);
        plant.setWalls();
        Pinpoint pin = new Pinpoint(o.delayMs / 1000.0, o.noiseIn, o.seed);
        pin.record(-1.0, plant);
        pin.record(0.0, plant);
        GridRoutes.Route route = GridRoutes.pick(table, c.sx, c.sy, c.sh);
        if (route == null) {
            res.refused = "no grid node near the robot reaches the goal";
            return res;
        }
        res.plannedDesc = String.format(Locale.US, "grid %d legs, %.2f s predicted", route.legs(), route.predictedSec);
        res.route = route;
        GridFollower f = new GridFollower(route.xs, route.ys, route.speeds, table.aimHeading, c.hive ? HiveDash.GOAL_TOL_IN : ParkDash.GOAL_TOL_IN, c.sh);
        TargetLock lock = new TargetLock();
        HiveTarget cell = c.hive ? HiveTarget.valueOf(c.target) : null;
        boolean lockOn = false;
        double t = 0.0;
        while (t <= 10.0) {
            if (!Double.isNaN(res.tArrive) && t >= res.tArrive + COAST_WINDOW_SEC) break;
            double[] m = pin.read(t);
            GridFollower.Output out = f.update(m[0], m[1], m[2], m[3], m[4], m[5]);
            double turn;
            if (out.lockWanted && cell != null) {
                lockOn = true;
                turn = lock.turn(new Pose2d(m[0], m[1], m[2]), m[5], cell, TargetLock.farCellExtraLeftIn(cell, true), 0.0, 0.0, dt);
            } else {
                if (lockOn) {
                    lockOn = false;
                    lock.release();
                }
                turn = out.turnActive ? out.turn : 0.0;
            }
            if (out.arrived && Double.isNaN(res.tArrive)) {
                res.tArrive = t;
                res.ax = plant.x;
                res.ay = plant.y;
            }
            res.track.add(new double[]{t, plant.x, plant.y, plant.h, out.forward, out.left, turn});
            int n = Math.max(1, (int) Math.ceil(dt / o.subStepSec));
            for (int s = 0; s < n; s++) {
                plant.step(out.forward, out.left, turn, Double.isNaN(o.fixedVolts) ? 12.5 : o.fixedVolts, dt / n);
                pin.record(t + dt * (s + 1) / n, plant);
            }
            t += dt;
            ElapsedTime.now = t;
        }
        return res;
    }

    /**
     * Each test of a drivetest.log run open-loop: from the test's first RUN row with the logged pose and velocity, the plant gets each row's command and battery
     * volts (as terminal volts: no sag) until the test's last RUN / STOP / SETTLE row; prints the end and largest position errors and the end heading error, the
     * same quantities as the Python check (tools/dash: the fit's sim_wheel integrated row by row).
     */
    static void testReplay(Options o, Path log) throws IOException {
        List<String> lines = Files.readAllLines(log, StandardCharsets.UTF_8);
        String[] head = null;
        List<String[]> rows = new ArrayList<>();
        for (String l : lines) {
            if (l.startsWith("H,")) head = l.substring(2).split(",");
            else if (l.startsWith("S,") && head != null) {
                String[] p = l.substring(2).split(",");
                if (p.length == head.length) rows.add(p);
            }
        }
        Map<String, Integer> col = new HashMap<>();
        for (int i = 0; i < head.length; i++) col.put(head[i], i);
        int cT = col.get("t"), cX = col.get("x"), cY = col.get("y"), cH = col.get("hdg_deg"), cVx = col.get("vx"), cVy = col.get("vy"), cW = col.get("w_dps");
        int cF = col.get("cmd_fwd"), cL = col.get("cmd_left"), cTu = col.get("cmd_turn"), cB = col.get("vbat"), cTest = col.get("test"), cPh = col.get("phase");
        Options q = new Options();
        q.model = o.model;
        q.wheel = o.wheel;
        q.sag = false;
        q.fieldFrame = o.fieldFrame;
        q.accelMax = o.accelMax;
        q.turnBoost = o.turnBoost;
        q.gainScale = o.gainScale;
        System.out.println(q.describe());
        List<Double> ends = new ArrayList<>();
        TreeSet<Integer> tests = new TreeSet<>();
        for (String[] r : rows) if (r[cPh].equals("RUN")) tests.add(Integer.parseInt(r[cTest]));
        for (int test : tests) {
            int first = -1, last = -1;
            for (int i = 0; i < rows.size(); i++) {
                String[] r = rows.get(i);
                if (Integer.parseInt(r[cTest]) != test) continue;
                String ph = r[cPh];
                if (ph.equals("RUN") || ph.equals("STOP") || ph.equals("SETTLE")) {
                    if (first < 0) first = i;
                    last = i;
                }
            }
            List<Integer> idx = new ArrayList<>();
            for (int i = first; i <= last; i++) if (Integer.parseInt(rows.get(i)[cTest]) == test) idx.add(i);
            boolean gap = idx.size() < 10;
            for (int k = 1; k < idx.size() && !gap; k++) if (num(rows.get(idx.get(k))[cT]) - num(rows.get(idx.get(k - 1))[cT]) > 0.05) gap = true;
            if (gap) continue;
            String[] r0 = rows.get(idx.get(0));
            double h0 = Math.toRadians(num(r0[cH])), vx0 = num(r0[cVx]), vy0 = num(r0[cVy]);
            Plant pl = newPlant(q, num(r0[cX]), num(r0[cY]), h0, vx0 * Math.cos(h0) + vy0 * Math.sin(h0), -vx0 * Math.sin(h0) + vy0 * Math.cos(h0), Math.toRadians(num(r0[cW])));
            double maxErr = 0.0, hUnwrap = h0;
            for (int k = 1; k < idx.size(); k++) {
                String[] a = rows.get(idx.get(k - 1)), b = rows.get(idx.get(k));
                double dt = num(b[cT]) - num(a[cT]);
                int n = Math.max(1, (int) Math.ceil(dt / 0.005));
                for (int st = 0; st < n; st++) pl.stepFree(num(a[cF]), num(a[cL]), num(a[cTu]), num(a[cB]), dt / n);
                maxErr = Math.max(maxErr, Math.hypot(pl.x - num(b[cX]), pl.y - num(b[cY])));
                hUnwrap += normalize(Math.toRadians(num(b[cH])) - Math.toRadians(num(a[cH])));
            }
            String[] rl = rows.get(idx.get(idx.size() - 1));
            double end = Math.hypot(pl.x - num(rl[cX]), pl.y - num(rl[cY]));
            ends.add(end);
            System.out.printf(Locale.US, "  test %2d: end %5.1f in, largest %5.1f in, heading %5.1f deg%n", test, end, maxErr, Math.toDegrees(Math.abs(pl.h - hUnwrap)));
        }
        System.out.printf(Locale.US, "end-of-test position error: median %.1f in, 90th %.1f in%n", median(ends), percentile(ends, 90));
    }

    /** Runs each case (TARGET x y deg grid|today, blue frame) with the real dashes on o's plant and writes the field, the planned routes and the tracks as JSON. */
    static void exportPaths(Options o, Path out, List<String> specs) throws Exception {
        StringBuilder b = new StringBuilder("{\"plant\":\"").append(o.describe().replace("\"", "'")).append("\",\"obstacles\":[");
        for (int i = 0; i < OBSTACLES.size(); i++) {
            Obstacle ob = OBSTACLES.get(i);
            double[] k = ob.r.corners();
            b.append(i == 0 ? "" : ",").append(String.format(Locale.US, "{\"name\":\"%s\",\"pts\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]}", ob.name, k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7]));
        }
        b.append("],\"cases\":[");
        HiveRoute.Params rp = robotParams();
        for (int n = 0; n < specs.size(); n++) {
            String[] f = specs.get(n).trim().split("[ ]+");
            boolean grid = f[4].equals("grid");
            restoreDefaults();
            o.grid = grid;
            HiveDash.GRID_ROUTES = grid;
            ParkDash.GRID_ROUTES = grid;
            Case c = freeCase(Double.parseDouble(f[1]), Double.parseDouble(f[2]), Double.parseDouble(f[3]), f[0], false);
            Result r = simulate(c, o, false);
            Contacts con = r.track.isEmpty() ? null : contacts(r.track, 0.0, r.track.get(r.track.size() - 1)[0]);
            b.append(n == 0 ? "" : ",").append(String.format(Locale.US, "{\"target\":\"%s\",\"planner\":\"%s\",\"start\":[%.1f,%.1f,%.1f],\"goal\":[%.2f,%.2f],\"arrive\":%s,\"route\":\"%s\",",
                    f[0], grid ? "grid" : "today", c.sx, c.sy, Math.toDegrees(c.sh), c.gx, c.gy, Double.isNaN(r.tArrive) ? "null" : String.format(Locale.US, "%.2f", r.tArrive),
                    r.plannedDesc.replace("\"", "'")));
            b.append(String.format(Locale.US, "\"supportClear\":%.2f,\"flowerClear\":%.2f,\"touched\":\"%s\",", con == null ? 99.0 : con.supportClear(), con == null ? 99.0 : con.flowerClear(),
                    con == null ? "" : String.join(", ", con.touched()).replace("\"", "'")));
            b.append("\"plan\":[");
            if (r.planned != null) for (int i = 0; i < r.planned.length; i++) b.append(i == 0 ? "" : ",").append(String.format(Locale.US, "[%.2f,%.2f]", r.planned[i][0], r.planned[i][1]));
            b.append("],\"track\":[");
            for (int i = 0; i < r.track.size(); i++) {
                double[] t = r.track.get(i);
                double sp = i + 1 < r.track.size() ? Math.hypot(r.track.get(i + 1)[1] - t[1], r.track.get(i + 1)[2] - t[2]) / Math.max(1e-6, r.track.get(i + 1)[0] - t[0]) : 0.0;
                b.append(i == 0 ? "" : ",").append(String.format(Locale.US, "[%.3f,%.2f,%.2f,%.4f,%.1f]", t[0], t[1], t[2], t[3], sp));
            }
            b.append("],\"footprints\":[");
            double next = 0.0;
            boolean firstFp = true;
            for (double[] t : r.track) {
                if (t[0] + 1e-9 < next) continue;
                next = t[0] + 0.25;
                double[] k = HiveRoute.poseRect(t[1], t[2], t[3], rp, 0.0).corners();
                b.append(firstFp ? "" : ",").append(String.format(Locale.US, "[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]", k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7]));
                firstFp = false;
            }
            b.append("]}");
            System.out.printf(Locale.US, "%-14s %-5s from (%.0f, %.0f, %.0f deg): %s, arrival %s, support %.1f in, flower %.1f in %s%n", f[0], grid ? "grid" : "today", c.sx, c.sy, Math.toDegrees(c.sh),
                    r.plannedDesc, Double.isNaN(r.tArrive) ? "none" : String.format(Locale.US, "%.2f s", r.tArrive), con == null ? 99 : con.supportClear(), con == null ? 99 : con.flowerClear(),
                    con == null ? "" : String.join(", ", con.touched()));
        }
        b.append("]}");
        Files.write(out, b.toString().getBytes(StandardCharsets.UTF_8));
        System.out.println("wrote " + out);
    }

    /** The largest distance (in) of a track from a polyline, over the rows up to tEnd. */
    static double maxDev(List<double[]> tr, double[][] w, double tEnd) {
        double m = 0.0;
        for (double[] r : tr) if (r[0] <= tEnd) m = Math.max(m, offRoute(r[1], r[2], w));
        return m;
    }

    /**
     * The corner speed calibration (GridRoutes.CORNER_SPEED): for each angle of the table, GridFollower drives an 80 in leg from rest, a corner limited to v, and
     * 60 in on; the largest v (bisection) whose swing off the route stays within GridRoutes.E_MAX is the table's speed. No walls, no obstacles; the plant is the
     * simulation's. Sets the table in memory and prints the line for GridRoutes.
     */
    static void calibrate(Options o, boolean print) {
        double[] deg = GridRoutes.CORNER_DEG;
        double[] out = new double[deg.length];
        // measured with no corner limit in force (the speed tried is the only limit; with the old table in force a corner asked for 66 in/s was driven at the old limit)
        GridRoutes.CORNER_SPEED = new double[deg.length];
        Arrays.fill(GridRoutes.CORNER_SPEED, GridRoutes.V_MAX);
        for (int k = 0; k < deg.length; k++) {
            if (deg[k] < 1e-6) {
                out[k] = GridRoutes.V_MAX;
                continue;
            }
            double th = Math.toRadians(deg[k]);
            double lo = 5.0, hi = GridRoutes.V_MAX;
            if (cornerDev(o, th, hi) <= GridRoutes.E_MAX * GridRoutes.CORNER_SAFETY) {
                out[k] = hi;
                continue;
            }
            for (int it = 0; it < 14; it++) {
                double mid = 0.5 * (lo + hi);
                if (cornerDev(o, th, mid) <= GridRoutes.E_MAX * GridRoutes.CORNER_SAFETY) lo = mid;
                else hi = mid;
            }
            out[k] = Math.floor(lo);
        }
        // a sharper turn never gets more speed than a gentler one
        for (int k = 1; k < out.length; k++) out[k] = Math.min(out[k], out[k - 1]);
        GridRoutes.CORNER_SPEED = out;
        if (print) {
            StringBuilder b = new StringBuilder("    public static double[] CORNER_SPEED = {");
            for (int k = 0; k < out.length; k++) b.append(k == 0 ? "" : ", ").append((int) out[k]);
            System.out.println("corner speed calibration (swing within " + GridRoutes.E_MAX * GridRoutes.CORNER_SAFETY + " in), degrees " + Arrays.toString(deg) + ":");
            System.out.println(b.append("};"));
        }
    }

    static boolean debugCorner = false;

    /** The swing (in) of GridFollower through one corner of theta entered at no more than v. */
    static double cornerDev(Options o, double theta, double v) {
        double c = Math.cos(theta), s = Math.sin(theta);
        double[] xs = {0, 80, 80 + 60 * c, 80 + 120 * c}, ys = {0, 0, 60 * s, 120 * s};
        double[] sp = {Double.MAX_VALUE, v, Double.MAX_VALUE, 0.0};
        GridRoutes.limitSpeeds(xs, ys, sp, xs[3], ys[3]);
        sp[1] = Math.min(sp[1], v);
        Plant plant = newPlant(o, 0, 0, 0, 0, 0, 0);
        Pinpoint pin = new Pinpoint(0.0, o.noiseIn, o.seed);
        pin.record(0.0, plant);
        GridFollower f = new GridFollower(xs, ys, sp, Double.NaN, 2.0, 0.0);
        List<double[]> tr = new ArrayList<>();
        double dt = 0.031, t = 0.0;
        double[][] w = {{xs[0], ys[0]}, {xs[1], ys[1]}, {xs[2], ys[2]}, {xs[3], ys[3]}};
        while (t < 6.0) {
            double[] m = pin.read(t);
            GridFollower.Output out = f.update(m[0], m[1], m[2], m[3], m[4], m[5]);
            tr.add(new double[]{t, plant.x, plant.y, plant.h});
            if (debugCorner) System.out.printf(Locale.US, "  t %.2f leg %d (%.1f, %.1f, %.0f deg) %.0f in/s dev %.1f cmd (%+.2f, %+.2f, %+.2f)%n", t, out.leg, plant.x, plant.y,
                    Math.toDegrees(plant.h), Math.hypot(plant.v[0], plant.v[1]), offRoute(plant.x, plant.y, w), out.forward, out.left, out.turn);
            if (out.leg >= 3) break;   // past the corner leg: the swing is over
            int n = Math.max(1, (int) Math.ceil(dt / o.subStepSec));
            for (int k = 0; k < n; k++) {
                plant.step(out.forward, out.left, out.turnActive ? out.turn : 0.0, Double.isNaN(o.fixedVolts) ? 12.5 : o.fixedVolts, dt / n);
                pin.record(t + dt * (k + 1) / n, plant);
            }
            t += dt;
        }
        return maxDev(tr, w, t);
    }

    static void survey(Options o, double pitch, Path csv) throws Exception {
        survey(o, pitch, csv, false);
    }

    static void survey(Options o, double pitch, Path csv, boolean grid) throws Exception {
        o.force = false;
        Map<String, GridRoutes.Table> tables = new HashMap<>();
        if (grid) {
            // the tables come from the GENERATION model (always the tuned plant, as RouteGen would have it); the dashes are driven on o's plant (another plant tests robustness)
            calibrateAll(o);
            for (RouteGen.Result g : RouteGen.buildAll()) tables.put(g.table.name, RouteGen.quantize(g.table));
            System.out.println("GRID planner (GridRoutes + GridFollower), trees from RouteGen: E " + GridRoutes.E_MAX + " in tapering to " + GridRoutes.E_GOAL
                    + ", grip " + GridRoutes.A_GRIP + " in/s^2, V_MAX " + GridRoutes.V_MAX + " in/s");
        } else if (o.grid) {
            System.out.println("the REAL HiveDash / ParkDash with GRID_ROUTES on and the generated tables (GridTables)" + (o.red ? ", RED alliance (the blue starts turned 180 deg)" : ""));
        } else {
            System.out.println("TODAY'S planner (HiveRoute + HiveDash / ParkDash)" + (o.red ? ", RED alliance (the blue starts turned 180 deg)" : ""));
        }
        String[] targets = {"BLUE_REAR", "BLUE_AUDIENCE", "LEFT", "RIGHT"};
        double[] headings = {0, 90, 180, 270};
        HiveRoute.Params p = robotParams();
        Files.createDirectories(csv.toAbsolutePath().getParent());
        System.out.println(o.describe() + "; starts every " + pitch + " in, 4 headings");
        try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(csv))) {
            w.println("target,x,y,heading_deg,result,route,time_s,support_clear_in,flower_clear_in,touched");
            for (String target : targets) {
                List<Double> times = new ArrayList<>();
                int starts = 0, refused = 0, noArrival = 0, hitSupport = 0, hitFlower = 0, offRoute = 0, newRoute = 0, fellBack = 0, notGrid = 0;
                List<String> worst = new ArrayList<>();
                List<double[]> worstKey = new ArrayList<>();
                for (double x = -66; x <= 66.01; x += pitch) {
                    for (double y = -66; y <= 66.01; y += pitch) {
                        for (double hd : headings) {
                            HiveRoute.Rect me = HiveRoute.poseRect(x, y, Math.toRadians(hd), p, 1.0);
                            boolean blocked = false;
                            for (Obstacle ob : OBSTACLES) if (rectDistance(me, ob.r) <= 0) blocked = true;
                            for (double k : me.corners()) if (Math.abs(k) > WALL) blocked = true;
                            if (blocked) continue;
                            starts++;
                            restoreDefaults();
                            String tg = !o.red ? target : target.equals("BLUE_REAR") ? "RED_AUDIENCE" : target.equals("BLUE_AUDIENCE") ? "RED_REAR" : target;
                            Case c = o.red ? freeCase(-x, -y, hd + 180.0, tg, true) : freeCase(x, y, hd, target, false);
                            Result r = grid ? simulateGrid(c, o, tables.get(target)) : simulate(c, o, false);
                            boolean off = false, again = false;
                            for (RunLog.Event e : r.events) {
                                if (e.text.contains("OFF ROUTE")) off = true;
                                if (e.text.contains("new route")) again = true;
                                if (e.text.contains("grid routes not used")) fellBack++;
                            }
                            if (off) offRoute++;
                            if (again) newRoute++;
                            if (o.grid && r.refused == null && !r.plannedDesc.startsWith("grid")) notGrid++;
                            String result;
                            Contacts con = null;
                            if (r.refused != null) {
                                refused++;
                                result = "refused";
                            } else {
                                con = contacts(r.track, 0.0, r.track.get(r.track.size() - 1)[0]);
                                if (Double.isNaN(r.tArrive)) {
                                    noArrival++;
                                    result = "no arrival";
                                } else {
                                    times.add(r.tArrive);
                                    result = "arrived";
                                }
                                boolean sup = con.supportClear() < 0, fl = con.flowerClear() < 0;
                                if (sup) hitSupport++;
                                if (fl) hitFlower++;
                                if (sup || fl) {
                                    double depth = -Math.min(con.supportClear(), con.flowerClear());
                                    worstKey.add(new double[]{depth, worst.size()});
                                    worst.add(String.format(Locale.US, "from (%.0f, %.0f, %.0f deg) %s: %s by %.1f in", x, y, hd, r.plannedDesc, con.touched(), depth));
                                }
                            }
                            w.printf(Locale.US, "%s,%.0f,%.0f,%.0f,%s,%s,%s,%s,%s,%s%n", target, x, y, hd, result, r.plannedDesc.replace(',', ' '),
                                    Double.isNaN(r.tArrive) ? "" : String.format(Locale.US, "%.2f", r.tArrive),
                                    con == null ? "" : String.format(Locale.US, "%.1f", con.supportClear()), con == null ? "" : String.format(Locale.US, "%.1f", con.flowerClear()),
                                    con == null ? "" : String.join(" ", con.touched()));
                        }
                    }
                }
                System.out.printf(Locale.US, "%-14s %4d starts: %3d refused, %3d no arrival; time median %.2f s, 90th %.2f s, max %.2f s; touched a support %3d (%.0f%%), a flower %3d (%.0f%%)%n",
                        target, starts, refused, noArrival, median(times), percentile(times, 90), times.isEmpty() ? Double.NaN : java.util.Collections.max(times),
                        hitSupport, 100.0 * hitSupport / Math.max(1, starts - refused), hitFlower, 100.0 * hitFlower / Math.max(1, starts - refused));
                if (o.grid) System.out.printf(Locale.US, "               off-route guard tripped in %d, a new route in %d; not on a grid route %d (fell back %d)%n", offRoute, newRoute, notGrid, fellBack);
                worstKey.sort((a, b) -> Double.compare(b[0], a[0]));
                for (int i = 0; i < Math.min(5, worstKey.size()); i++) System.out.println("      " + worst.get((int) worstKey.get(i)[1]));
            }
        }
        System.out.println("CSV: " + csv);
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
        System.out.println(o.describe() + "; loop " + Math.round(dt * 1000) + " ms");
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

    /** The wheel plant's rest volts the corner speeds are calibrated at (a used battery, the default, a fresh one). */
    static final double[] GEN_WHEEL_VOLTS = {11.5, 12.8, 13.4};

    /**
     * The corner speed table for the generation: calibrated on the tuned plant (the 11-05 and 11-06 robot, as replayed) and on the wheel plant (the 11-07 drive test) at
     * GEN_WHEEL_VOLTS, the slowest of them at each angle; sets GridRoutes.CORNER_SPEED and prints each.
     */
    static void calibrateAll(Options o) throws IOException {
        List<Options> plants = new ArrayList<>();
        plants.add(tuned(o));
        for (double v : GEN_WHEEL_VOLTS) {
            Options w = new Options();
            w.model = Model.of("dash");
            w.wheel = new Wheel("fit 2026-11-07", Wheel.FIT_1107);
            w.fixedVolts = v;
            w.noiseIn = o.noiseIn;
            w.seed = o.seed;
            plants.add(w);
        }
        double[] min = null;
        for (Options p : plants) {
            System.out.println("calibrating on " + p.describe());
            calibrate(p, true);
            double[] c = GridRoutes.CORNER_SPEED.clone();
            if (min == null) min = c;
            else for (int k = 0; k < c.length; k++) min[k] = Math.min(min[k], c[k]);
        }
        GridRoutes.CORNER_SPEED = min;
        StringBuilder b = new StringBuilder("the slowest per angle: {");
        for (int k = 0; k < min.length; k++) b.append(k == 0 ? "" : ", ").append((int) min[k]);
        System.out.println(b.append("}"));
    }

    /** The tuned plant with o's loop, noise and seed: the plant the routes are generated and calibrated with. */
    static Options tuned(Options o) throws IOException {
        Options g = new Options();
        g.model = Model.of("dash");
        g.fieldFrame = true;
        g.accelMax = TUNED_ACCEL;
        g.turnBoost = TUNED_TURN_BOOST;
        g.fixedVolts = TUNED_VOLTS;
        g.noiseIn = o.noiseIn;
        g.seed = o.seed;
        return g;
    }

    /** The "tuned" plant (2026-11-06): grip limit in/s^2, turn boost, fixed battery volts; with the dash fit and the lag in the field frame. */
    static final double TUNED_ACCEL = 150.0, TUNED_TURN_BOOST = 1.5, TUNED_VOLTS = 12.5;
    /** The wheel plant's default rest volts (a battery partly used; the drive test's fresh battery rested at 13.4 V, the TeleOp logs of 11-05 and 11-06 read 11.8 to 12.6). */
    static final double WHEEL_REST_VOLTS = 12.8;
    /** The wheel plant: the turn uses YAW_GRIP |aw| kw of the translation limit (0.3: the dash replay and the drive test fit both prefer it to 0). */
    static double YAW_GRIP = 0.3;

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
                    if (args[i + 1].equals("wheel")) {
                        i++;
                        o.wheel = new Wheel("fit 2026-11-07", Wheel.FIT_1107);
                        o.fixedVolts = WHEEL_REST_VOLTS;
                        break;
                    }
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
                case "--wheel-model": o.wheel = new Wheel(args[i + 1], readNpy(Paths.get(args[++i]))); break;
                case "--no-sag": o.sag = false; break;
                case "--v-fade": Wheel.vFade = Double.parseDouble(args[++i]); break;
                case "--yaw-grip": YAW_GRIP = Double.parseDouble(args[++i]); break;
                case "--wheel-accel": {
                    // scales the wheel model's two translation limits (A_up, A_down), for the replay's corner swings
                    double f = Double.parseDouble(args[++i]);
                    double[] th = Wheel.FIT_1107.clone();
                    th[19] *= f;
                    th[20] *= f;
                    o.wheel = new Wheel("fit 2026-11-07, accel x " + f, th);
                    break;
                }
                case "--log-volts": o.fixedVolts = Double.NaN; break;
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
                case "--red": red = true; o.red = true; break;
                case "--grid": o.grid = true; break;
                case "--set": {
                    String[] kv = args[++i].split("=");
                    String[] cf = kv[0].split("[.]");
                    OVERRIDES.add(new String[]{cf[0], cf[1], kv[1]});
                    break;
                }
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
            case "survey":
                survey(o, rest.size() > 1 ? Double.parseDouble(rest.get(1)) : 12.0, Paths.get("build", "dashsim", "survey_current.csv"));
                break;
            case "gridrun": {
                // one grid dash with its route and a trace: gridrun x y deg TARGET
                restoreDefaults();
                Case c = freeCase(Double.parseDouble(rest.get(1)), Double.parseDouble(rest.get(2)), Double.parseDouble(rest.get(3)), rest.get(4), false);
                GridRoutes.Table tb = null;
                calibrateAll(o);
                for (RouteGen.Result g : RouteGen.buildAll()) if (g.table.name.equals(c.target)) tb = RouteGen.quantize(g.table);
                Result r = simulateGrid(c, o, tb);
                if (r.route != null) for (int i = 0; i < r.route.xs.length; i++) System.out.printf(Locale.US, "  waypoint %d (%.1f, %.1f) speed limit %s%n", i, r.route.xs[i], r.route.ys[i],
                        r.route.speeds[i] > 1e6 ? "-" : String.format(Locale.US, "%.0f in/s", r.route.speeds[i]));
                HiveRoute.Params rp = robotParams();
                double[][] wps = new double[r.route == null ? 0 : r.route.xs.length][];
                for (int i = 0; i < wps.length; i++) wps[i] = new double[]{r.route.xs[i], r.route.ys[i]};
                for (int i = 0; i < r.track.size(); i += 2) {
                    double[] s = r.track.get(i);
                    HiveRoute.Rect me = HiveRoute.poseRect(s[1], s[2], s[3], rp, 0.0);
                    double sup = Double.MAX_VALUE, flo = Double.MAX_VALUE;
                    for (Obstacle ob : OBSTACLES) {
                        if (ob.name.contains("flower")) flo = Math.min(flo, rectDistance(me, ob.r));
                        else sup = Math.min(sup, rectDistance(me, ob.r));
                    }
                    double sp = i + 1 < r.track.size() ? Math.hypot(r.track.get(i + 1)[1] - s[1], r.track.get(i + 1)[2] - s[2]) / (r.track.get(i + 1)[0] - s[0]) : 0;
                    System.out.printf(Locale.US, "  t %4.2f (%6.1f, %6.1f, %6.1f deg) %4.0f in/s off-route %4.1f in, clearance support %5.1f flower %5.1f in, cmd (%+.2f, %+.2f, %+.2f)%n", s[0], s[1], s[2],
                            Math.toDegrees(s[3]), sp, wps.length < 2 ? 0 : offRoute(s[1], s[2], wps), sup, flo, s[4], s[5], s[6]);
                }
                System.out.println("arrival " + r.tArrive + ", " + r.plannedDesc);
                break;
            }
            case "export": {
                // paths for a picture: export <out.json> then cases "TARGET x y deg grid|today", each run with the real dashes; writes the field, the routes and the tracks
                exportPaths(o, Paths.get(rest.get(1)), rest.subList(2, rest.size()));
                break;
            }
            case "testreplay": {
                // the plant open-loop through each Drive Characterization test (the cross-check of fit_drivetest.py's model): testreplay <drivetest.log>
                testReplay(o, Paths.get(rest.get(1)));
                break;
            }
            case "cornerdebug": {
                debugCorner = true;
                System.out.println("swing " + cornerDev(o, Math.toRadians(Double.parseDouble(rest.get(1))), Double.parseDouble(rest.get(2))));
                break;
            }
            case "gentables": {
                // the one command of design decision 4: calibrate the corner speeds on every generation plant (the slowest per angle), build the four trees, write GridTables.java
                calibrateAll(o);
                List<RouteGen.Result> rs = RouteGen.buildAll();
                for (RouteGen.Result g : rs) {
                    System.out.printf(Locale.US, "%-14s goal (%.1f, %.1f): %d states reach the goal from %d nodes; %.0f ms%n", g.table.name, g.table.goalX, g.table.goalY,
                            g.reachableStates, g.nodesReachable, g.ms);
                }
                Path out = Paths.get("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/GridTables.java");
                RouteGen.writeSource(rs, out, "tools/dash/run-sim.cmd gentables on " + java.time.LocalDate.now() + " (corner speeds: the slowest per angle of the tuned plant and the wheel plant at "
                        + Arrays.toString(GEN_WHEEL_VOLTS) + " V rest)");
                System.out.println("wrote " + out + " (" + Files.size(out) + " bytes); run the simulation again (it recompiles) to use it, e.g. survey 12 --grid");
                break;
            }
            case "calibrate":
                calibrate(o, true);
                break;
            case "gridsurvey":
                survey(o, rest.size() > 1 ? Double.parseDouble(rest.get(1)) : 12.0, Paths.get("build", "dashsim", "survey_grid.csv"), true);
                break;
            case "run":
                freeRun(o, Double.parseDouble(rest.get(1)), Double.parseDouble(rest.get(2)), Double.parseDouble(rest.get(3)), rest.get(4), red);
                break;
            default:
                System.out.println("usage: DashSim [replay | plant [dash] | run x y deg TARGET] [--model dash|all|file.npy] [--loop-ms N] [--delay-ms N] [--noise IN] [--frame body|field] [--no-force] [--case FILE:T0] [--tracks DIR] [--red]");
        }
    }
}
