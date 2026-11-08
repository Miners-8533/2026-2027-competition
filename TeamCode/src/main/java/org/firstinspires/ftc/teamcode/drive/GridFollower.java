package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Rotation2d;

/**
 * Follows a {@link GridRoutes.Route} (design/dash-planner-grid.md, "The grid planner"): leg by leg, each leg driven aligned with the wheels (front or rear first,
 * whichever needs less turning; on the last leg the one nearer the aim), at the speed the route allows: on a leg the target speed is
 * min(V_MAX, sqrt(v_end^2 + 2 A_DEC d)), d the distance to the leg's end and v_end the most speed allowed there (the table's corner and stop limits), turned into
 * a power by the drive model's feedforward (FF_DEAD + v / FF_GAIN) plus a little feedback on the measured speed along the leg. The last leg is the Hive Dash's:
 * straight at the goal with ALONG_KP, capped at APPROACH_POWER near it; for a cell the robot drives it TURNED TO THE AIM (it turns at the leg's start, slowed by the
 * heading error; the routes were checked that way), and the target lock takes the heading within LOCK_RADIUS_IN. (The Hive Dash's early lock is not used: it spun
 * the robot up to 180 deg at speed over the last 35 in, into the audience flower in the simulation, 2026-11-07.) The first leg is the hop from the robot to the route's start node; when it is short the robot holds the heading it had on it.
 *
 * Pure logic, field frame (the route must already be in the field frame: {@link #toField}); the caller applies the output like HiveDash's.
 */
@Config
public final class GridFollower {

    public static double MAX_POWER = 0.85;
    /** Speed to power (the drive fit of 2026-11-06: about 99 in/s per unit above a 0.11 dead zone, forward), and power per in/s of speed error. */
    public static double FF_GAIN = 92.0, FF_DEAD = 0.11, SPEED_KP = 0.006;
    /** More than this over the target speed (in/s), the power along the leg is zero so the motors BRAKE (feedback alone kept 0.18 power on at 70 in/s against 40). */
    public static double BRAKE_OVER_IN_PER_S = 3.0;
    /** Sideways correction: power per inch off the leg, per in/s of sideways speed (damping: a swing is stopped, not only pulled back), and its limit. */
    public static double CROSS_KP = 0.06, CROSS_KD = 0.012, CROSS_MAX = 0.4;
    public static double HEADING_KP = 1.5, HEADING_KD = 0.08, TURN_MAX = 0.5, ALIGN_FLOOR = 0.4;
    /** A waypoint counts as passed this close (the next leg starts). A hop shorter than HOP_KEEP_HEADING_IN keeps the robot's heading. */
    public static double PASS_TOL_IN = 3.0, HOP_KEEP_HEADING_IN = 12.0;
    /** The last leg (the Hive Dash's values). */
    public static double ALONG_KP = 0.05, MIN_FINAL_POWER = 0.08, APPROACH_POWER = 0.5, APPROACH_RADIUS_IN = 32.0;
    public static double LOCK_RADIUS_IN = 20.0;

    public static final class Output {
        public boolean translation, turnActive, lockWanted, arrived;
        public double forward, left, turn;
        /** 1-based leg being driven. */
        public int leg;
    }

    private final double[] xs, ys, speeds;
    private final boolean aim;
    private final double aimHeading, goalTolIn;
    private final Output out = new Output();
    private int leg = 0;
    private double legHeading;
    private boolean legAligned, arrived = false;
    private double offRouteIn = 0.0;

    /** The route in the field frame. aimHeading: the heading at the goal (a hive cell; the lock does the aiming), or NaN (a park corner). */
    public GridFollower(double[] xs, double[] ys, double[] speeds, double aimHeading, double goalTolIn, double startHeading) {
        this.xs = xs;
        this.ys = ys;
        this.speeds = speeds;
        this.aim = !Double.isNaN(aimHeading);
        this.aimHeading = aimHeading;
        this.goalTolIn = goalTolIn;
        startLeg(startHeading);
    }

    /** A blue-frame route in the field frame (red is turned 180 deg about the center). */
    public static double[][] toField(GridRoutes.Route r, boolean blue) {
        double s = blue ? 1.0 : -1.0;
        double[] xs = new double[r.xs.length], ys = new double[r.ys.length];
        for (int i = 0; i < xs.length; i++) {
            xs[i] = s * r.xs[i];
            ys[i] = s * r.ys[i];
        }
        return new double[][]{xs, ys, r.speeds.clone()};
    }

    public boolean arrived() {
        return arrived;
    }

    public int legs() {
        return xs.length - 1;
    }

    /** The distance (in) of the last update's pose from the leg being driven (the segment, ends included): what the off-route guard watches. */
    public double offRouteIn() {
        return offRouteIn;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void startLeg(double heading) {
        double dir = Math.atan2(ys[leg + 1] - ys[leg], xs[leg + 1] - xs[leg]);
        double len = Math.hypot(xs[leg + 1] - xs[leg], ys[leg + 1] - ys[leg]);
        boolean last = leg == xs.length - 2;
        legAligned = !(leg == 0 && xs.length > 2 && len < HOP_KEEP_HEADING_IN);
        if (!legAligned) {
            legHeading = heading;   // a short hop: hold the heading the robot has
            return;
        }
        if (last && aim) {
            legHeading = aimHeading;   // a cell's last leg: turned to the aim, the leg driven partly sideways
            return;
        }
        double ref = heading;
        double front = dir, rear = dir + Math.PI;
        legHeading = Math.abs(Rotation2d.normalize(front - ref)) <= Math.abs(Rotation2d.normalize(rear - ref)) ? front : rear;
    }

    /** One loop: the pose and field velocity (in, rad, in/s, rad/s). */
    public Output update(double x, double y, double h, double vx, double vy, double w) {
        out.translation = true;
        out.turnActive = false;
        out.lockWanted = false;
        out.forward = out.left = out.turn = 0.0;
        int legs = xs.length - 1;
        double gx = xs[legs], gy = ys[legs];
        double goalDist = Math.hypot(x - gx, y - gy);
        if (arrived) {
            out.lockWanted = aim;
            out.arrived = true;
            out.leg = legs;
            return out;
        }
        double ux = 1, uy = 0, nx = 0, ny = 1, cross = 0, remaining = 0;
        for (int guard = 0; guard < legs + 1; guard++) {
            double ax = xs[leg], ay = ys[leg], bx = xs[leg + 1], by = ys[leg + 1];
            double len = Math.hypot(bx - ax, by - ay);
            ux = len < 1e-6 ? 1.0 : (bx - ax) / len;
            uy = len < 1e-6 ? 0.0 : (by - ay) / len;
            nx = -uy;
            ny = ux;
            double along = (x - ax) * ux + (y - ay) * uy;
            cross = (x - ax) * nx + (y - ay) * ny;
            remaining = len - along;
            if (leg == legs - 1) {
                if (goalDist <= goalTolIn) {
                    arrived = true;
                    out.arrived = true;
                    out.lockWanted = aim;
                    out.leg = legs;
                    return out;
                }
                break;
            }
            if (remaining > PASS_TOL_IN && along <= len) break;
            leg++;
            startLeg(h);
        }
        boolean last = leg == legs - 1;
        {
            double ax = xs[leg], ay = ys[leg], bx = xs[leg + 1], by = ys[leg + 1];
            double len = Math.hypot(bx - ax, by - ay);
            double t = len < 1e-6 ? 0.0 : clamp(((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (len * len), 0.0, 1.0);
            offRouteIn = Math.hypot(x - (ax + t * (bx - ax)), y - (ay + t * (by - ay)));
        }
        double vfx, vfy;
        if (last) {
            double v = clamp(ALONG_KP * goalDist, MIN_FINAL_POWER, MAX_POWER);
            if (APPROACH_POWER > 0.0 && goalDist <= APPROACH_RADIUS_IN) v = Math.min(v, Math.max(APPROACH_POWER, MIN_FINAL_POWER));
            double d = Math.max(goalDist, 1e-6);
            vfx = (gx - x) / d * v;
            vfy = (gy - y) / d * v;
        } else {
            double vEnd = Math.min(GridRoutes.V_MAX, speeds[leg + 1]);
            // the speed must be down to vEnd where the next leg starts, PASS_TOL_IN before the corner (braking for the corner point itself entered corners
            // at the speed allowed 3 in out: 28 in/s into a corner limited to 10, found on the wheel plant 2026-11-07)
            double vTarget = Math.min(GridRoutes.V_MAX, Math.sqrt(vEnd * vEnd + 2.0 * GridRoutes.A_DEC * Math.max(0.0, remaining - PASS_TOL_IN)));
            double vAlong = vx * ux + vy * uy;
            double p = vTarget <= 0.5 || vAlong > vTarget + BRAKE_OVER_IN_PER_S ? 0.0 : FF_DEAD + vTarget / FF_GAIN + SPEED_KP * (vTarget - vAlong);
            p = clamp(p, 0.0, MAX_POWER);
            double vCross = vx * nx + vy * ny;
            double vC = clamp(-CROSS_KP * cross - CROSS_KD * vCross, -CROSS_MAX, CROSS_MAX);
            vfx = ux * p + nx * vC;
            vfy = uy * p + ny * vC;
        }
        double fwd = vfx * Math.cos(h) + vfy * Math.sin(h);
        double left = -vfx * Math.sin(h) + vfy * Math.cos(h);
        boolean inLock = aim && last && goalDist <= LOCK_RADIUS_IN;
        if (inLock) {
            out.lockWanted = true;
        } else {
            double err = Rotation2d.normalize(legHeading - h);
            out.turn = clamp(HEADING_KP * err - HEADING_KD * w, -TURN_MAX, TURN_MAX);
            out.turnActive = true;
            if (legAligned) {
                double scale = Math.max(ALIGN_FLOOR, Math.cos(err));
                fwd *= scale;
                left *= scale;
            }
        }
        out.forward = fwd;
        out.left = left;
        out.leg = leg + 1;
        return out;
    }
}
