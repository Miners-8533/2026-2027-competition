package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * PARK DASH (design/park-dash.md): in TeleOp - Competition the driver HOLDS D-pad up or down and the robot drives to one of the two park corners of its own loading zone, the
 * corner on the driver's left (up) or on the driver's right (down), and stays there while the D-pad is held. The ramp is stowed when the dash starts. Releasing the D-pad cancels at
 * once and gives the sticks back.
 *
 * It plans and follows the route the same way the Hive Dash does ({@link HiveRoute}: a straight line when clear, otherwise through the helper corners and the lane gates under the
 * hive; each long leg driven aligned with the wheels, front or rear leading), but it has no cell: the heading is the leg's heading, and the robot does not turn on a short leg.
 * It is its own small class so the (untested) Hive Dash is not touched. The hold at the corner is {@link HoldPosition}, which the TeleOp asks for while {@link #holdWanted()}.
 *
 * The corners (blue frame, the loading zone is x 23.9 to 46.6, y 59.1 to 70.1; red is the same turned 180 degrees) are the duo autos' park poses: the audience-side corner
 * (40, 60) is on the driver's left for either alliance, the hive-side corner (26, 60) on the right. Only a little of the robot has to be in the zone to count.
 *
 * Refused, with no movement, in backup mode / a Pinpoint fault / an unseeded pose, and when the robot is touching a hive support. This class touches no hardware.
 */
@Config
public final class ParkDash {

    // ---- the corners (blue frame; the robot's center) ----
    /** The audience-side corner (the driver's left) and the hive-side corner (the driver's right), and the y of both. */
    public static double LEFT_X = 40.0, RIGHT_X = 26.0, PARK_Y = 60.0;

    // ---- driving: the Hive Dash's values to start from ----
    public static double MAX_POWER = 0.85;   // 0.5 for the first runs (7 dashes on 2026-11-05, 0.7 to 2.0 in from the corner); 0.85 from 2026-11-05 (user), as the Hive Dash
    public static double GOAL_TOL_IN = 2.0, PASS_TOL_IN = 3.0;
    public static double ALONG_KP = 0.05, MIN_FINAL_POWER = 0.08;
    public static double CROSS_KP = 0.05, CROSS_MAX = 0.3;
    public static double HEADING_KP = 1.5, HEADING_KD = 0.08, TURN_MAX = 0.5;   // kept at 0.5 with the power at 0.85, see HiveDash.TURN_MAX
    public static double ALIGN_FLOOR = 0.4;
    public static double SAMPLE_PERIOD_SEC = 0.5;
    public static double REFUSED_SHOWN_SEC = 3.0;
    /** True: plan and log but do not drive. */
    public static boolean DRY_RUN = false;

    public enum Corner { NONE, LEFT, RIGHT }

    public enum State { IDLE, DRIVING, PARKED }

    /** What the TeleOp gives the dash each loop. */
    public static final class Input {
        public Pose2d pose;
        public PoseVelocity2d measured;
        /** The corner of the held D-pad direction, or NONE when neither (or both) is held. */
        public Corner requested = Corner.NONE;
        public boolean blue;
        /** The driver's forward direction (the same as the Hive Dash's), to tell which corner is on the driver's left. */
        public double driverForwardRad;
        /** Why the dash is not available (backup mode, a Pinpoint fault, ...), or null. */
        public String blockedWhy;
        /** The Hive Dash is running: this one does not start. */
        public boolean otherDashBusy;
    }

    /** What the dash wants done this loop. */
    public static final class Output {
        public boolean translation;
        public double forward, left;
        public boolean turnActive;
        public double turn;
        /** Stow the ramp now (once, at the start). */
        public boolean stowRamp;
        /** The D-pad was just refused (give the driver a short rumble). */
        public boolean refused;

        void reset() {
            translation = false;
            forward = left = 0.0;
            turnActive = false;
            turn = 0.0;
            stowRamp = false;
            refused = false;
        }
    }

    /** The corner a D-pad direction means. Up = the driver's left, down = the driver's right. */
    public static Corner cornerForDpad(boolean up, boolean down) {
        if (up == down) return Corner.NONE;
        return up ? Corner.LEFT : Corner.RIGHT;
    }

    /**
     * The corner's center in the field frame for this alliance. The driver looks along driverForwardRad (blue: toward -Y), so the driver's left is the field direction whose x
     * component is -sin(driverForwardRad): +X for blue, -X for red with the default directions. The corner on the driver's left is the one on that side of the two.
     */
    public static double[] cornerXY(Corner corner, boolean blue, double driverForwardRad) {
        boolean leftIsPositiveX = -Math.sin(driverForwardRad) > 0.0;
        double sign = blue ? 1.0 : -1.0;
        double xa = sign * LEFT_X, xb = sign * RIGHT_X;   // the two corners' x in the field frame
        double larger = Math.max(xa, xb), smaller = Math.min(xa, xb);
        double leftX = leftIsPositiveX ? larger : smaller;
        double rightX = leftIsPositiveX ? smaller : larger;
        return new double[]{corner == Corner.LEFT ? leftX : rightX, sign * PARK_Y};
    }

    private final double depthIn, widthIn, fromBackIn;
    private final Output out = new Output();
    private State state = State.IDLE;
    private boolean pressConsumed = false;   // this press has been used (started, refused): the D-pad must be released first
    private Corner corner = Corner.NONE;
    private double goalX, goalY;
    private HiveRoute.Plan plan;
    private int legIndex = 0;
    private double legHeading = 0.0, prevLegHeading = 0.0;
    private boolean legAligned = false;
    private final ElapsedTime dashClock = new ElapsedTime();
    private final ElapsedTime sampleClock = new ElapsedTime();
    private final ElapsedTime refusedClock = new ElapsedTime();
    private boolean everRefused = false;
    private String refusedWhy = "";
    private String status = "ready";

    public ParkDash(double depthIn, double widthIn, double fromBackIn) {
        this.depthIn = depthIn;
        this.widthIn = widthIn;
        this.fromBackIn = fromBackIn;
        HiveRoute.plan(-61.5, 15.25, 0.0, 40.0, 60.0, params());   // a throwaway plan to warm the planner (as the Hive Dash does)
    }

    public State state() {
        return state;
    }

    public String status() {
        return status;
    }

    /** True while the robot is parked at the corner with the D-pad held: the TeleOp asks {@link HoldPosition} to hold the pose. */
    public boolean holdWanted() {
        return state == State.PARKED;
    }

    /** The planner's settings: the Hive Dash's Dashboard values (the same supports, helper corners and speeds). */
    private HiveRoute.Params params() {
        HiveRoute.Params p = new HiveRoute.Params();
        p.robotHeightIn = HiveDash.ROBOT_HEIGHT_IN;
        p.marginIn = HiveDash.MARGIN_IN;
        p.depthIn = depthIn;
        p.widthIn = widthIn;
        p.fromBackIn = fromBackIn;
        p.alignMinLegIn = HiveDash.ALIGN_MIN_LEG_IN;
        p.turnPenaltyInPerDeg = HiveDash.TURN_PENALTY_IN_PER_DEG;
        p.unalignedSpeedRatio = HiveDash.UNALIGNED_SPEED_RATIO;
        p.underHive = HiveDash.UNDER_HIVE_ROUTES;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) p.points.add(new double[]{sx * HiveDash.HELPER_X, sy * HiveDash.HELPER_Y});
            p.points.add(new double[]{sx * HiveDash.GATE_X, 0.0});
        }
        return p;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Call once per loop with the loop's buffered pose; returns the (reused) output. */
    public Output update(Input in) {
        out.reset();
        boolean held = in.requested != Corner.NONE;
        if (!held) pressConsumed = false;
        switch (state) {
            case IDLE:
                if (held && !pressConsumed) {
                    pressConsumed = true;
                    tryStart(in);
                }
                break;
            case DRIVING:
            case PARKED:
                if (!held) {
                    finish(in, "the D-pad was released");
                } else if (in.requested != corner) {
                    finish(in, "the other D-pad direction was pressed");
                } else if (in.blockedWhy != null) {
                    finish(in, in.blockedWhy);
                } else if (state == State.DRIVING) {
                    drive(in);
                } else {
                    out.translation = true;   // parked: no drive power from here; the TeleOp's hold keeps the pose
                }
                break;
            default:
                break;
        }
        status = statusLine(in);
        sample(in);
        return out;
    }

    private void refuse(Corner requested, String why) {
        out.refused = true;
        refusedWhy = why;
        everRefused = true;
        refusedClock.reset();
        RunLog.event("PARK DASH refused (" + requested + "): " + why);
    }

    private void tryStart(Input in) {
        Corner requested = in.requested;
        if (in.blockedWhy != null) {
            refuse(requested, "not available: " + in.blockedWhy);
            return;
        }
        if (in.otherDashBusy) {
            refuse(requested, "the Hive Dash is running");
            return;
        }
        double[] g = cornerXY(requested, in.blue, in.driverForwardRad);
        Pose2d pose = in.pose;
        HiveRoute.Plan pl = HiveRoute.plan(pose.position.x, pose.position.y, pose.heading.toDouble(), g[0], g[1], params());
        plan = pl;
        if (!pl.ok) {
            refuse(requested, pl.refusal);
            return;
        }
        RunLog.event(String.format(Locale.US, "PARK DASH %s (%s corner): from (%.1f, %.1f, %.1f deg) to (%.1f, %.1f), %s; power %.2f", DRY_RUN ? "dry run" : "start", requested,
                pose.position.x, pose.position.y, pose.heading.toDegrees(), g[0], g[1], pl.description, MAX_POWER));
        if (DRY_RUN) return;
        corner = requested;
        goalX = g[0];
        goalY = g[1];
        legIndex = 0;
        prevLegHeading = pose.heading.toDouble();
        startLeg();
        dashClock.reset();
        sampleClock.reset();
        state = State.DRIVING;
        out.stowRamp = true;
        drive(in);
    }

    private void finish(Input in, String why) {
        Pose2d pose = in.pose;
        RunLog.event(String.format(Locale.US, "PARK DASH %s (%s) at (%.1f, %.1f, %.1f deg), %.1f in from the corner, after %.2f s", state == State.PARKED ? "released" : "cancelled", why,
                pose.position.x, pose.position.y, pose.heading.toDegrees(), Math.hypot(pose.position.x - goalX, pose.position.y - goalY), dashClock.seconds()));
        state = State.IDLE;
    }

    private void startLeg() {
        double[] a = plan.waypoints[legIndex], b = plan.waypoints[legIndex + 1];
        double dir = Math.atan2(b[1] - a[1], b[0] - a[0]);
        legAligned = plan.aligned[legIndex];
        // front or rear leading, whichever is nearer the heading the robot has
        double front = dir, rear = dir + Math.PI;
        legHeading = Math.abs(Rotation2d.normalize(front - prevLegHeading)) <= Math.abs(Rotation2d.normalize(rear - prevLegHeading)) ? front : rear;
        if (legAligned) prevLegHeading = legHeading;
        RunLog.event(String.format(Locale.US, "PARK DASH leg %d of %d: (%.1f, %.1f) -> (%.1f, %.1f), %.0f in, %s", legIndex + 1, plan.waypoints.length - 1, a[0], a[1], b[0], b[1],
                Math.hypot(b[0] - a[0], b[1] - a[1]), legAligned ? "aligned, " + (legHeading == front ? "front" : "rear") + " leading" : "short, heading kept"));
    }

    private void drive(Input in) {
        Pose2d pose = in.pose;
        double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
        int legs = plan.waypoints.length - 1;
        double goalDist = Math.hypot(x - goalX, y - goalY);
        double ux = 0, uy = 0, nx = 0, ny = 0, cross = 0, remaining = 0;
        for (int guard = 0; guard < 4; guard++) {
            double[] a = plan.waypoints[legIndex], b = plan.waypoints[legIndex + 1];
            double len = Math.hypot(b[0] - a[0], b[1] - a[1]);
            ux = len < 1e-6 ? 1.0 : (b[0] - a[0]) / len;
            uy = len < 1e-6 ? 0.0 : (b[1] - a[1]) / len;
            nx = -uy;
            ny = ux;
            double along = (x - a[0]) * ux + (y - a[1]) * uy;
            cross = (x - a[0]) * nx + (y - a[1]) * ny;
            remaining = len - along;
            boolean last = legIndex == legs - 1;
            if (last) {
                if (goalDist <= GOAL_TOL_IN) {
                    state = State.PARKED;
                    RunLog.event(String.format(Locale.US, "PARK DASH parked after %.2f s at (%.1f, %.1f, %.1f deg), %.1f in from the corner", dashClock.seconds(), x, y,
                            pose.heading.toDegrees(), goalDist));
                    out.translation = true;
                    return;
                }
                break;
            }
            if (remaining > PASS_TOL_IN && along <= len) break;
            RunLog.event(String.format(Locale.US, "PARK DASH via point (%.0f, %.0f) reached at (%.1f, %.1f), %.2f s", b[0], b[1], x, y, dashClock.seconds()));
            legIndex++;
            startLeg();
        }
        boolean last = legIndex == legs - 1;
        double vfx, vfy;
        if (last) {
            // the last leg heads straight for the corner: a robot that overshoots it or passes it to the side is driven back (power along the leg kept pushing it on past)
            double v = clamp(ALONG_KP * goalDist, MIN_FINAL_POWER, MAX_POWER);
            double d = Math.max(goalDist, 1e-6);
            vfx = (goalX - x) / d * v;
            vfy = (goalY - y) / d * v;
        } else {
            double vC = clamp(-CROSS_KP * cross, -CROSS_MAX, CROSS_MAX);
            vfx = ux * MAX_POWER + nx * vC;
            vfy = uy * MAX_POWER + ny * vC;
        }
        double fwd = vfx * Math.cos(h) + vfy * Math.sin(h);
        double left = -vfx * Math.sin(h) + vfy * Math.cos(h);
        // the heading: an aligned leg turns the robot to the leg's heading and slows it by the heading error; a short leg keeps the heading it has
        if (legAligned) {
            double err = Rotation2d.normalize(legHeading - h);
            double angVel = in.measured == null ? 0.0 : in.measured.angVel;
            out.turn = clamp(HEADING_KP * err - HEADING_KD * angVel, -TURN_MAX, TURN_MAX);
            out.turnActive = true;
            double scale = Math.max(ALIGN_FLOOR, Math.cos(err));
            fwd *= scale;
            left *= scale;
        }
        out.translation = true;
        out.forward = fwd;
        out.left = left;
    }

    private String statusLine(Input in) {
        switch (state) {
            case DRIVING:
                return String.format(Locale.US, "DRIVING to the %s corner: leg %d of %d, %.0f in to go", corner, legIndex + 1, plan.waypoints.length - 1,
                        Math.hypot(in.pose.position.x - goalX, in.pose.position.y - goalY));
            case PARKED:
                return "PARKED in the " + corner + " corner (holding while the D-pad is held)";
            default:
                if (everRefused && refusedClock.seconds() < REFUSED_SHOWN_SEC) return "REFUSED: " + refusedWhy;
                return DRY_RUN ? "ready (DRY RUN: plans, does not drive)" : "ready (hold D-pad up = left corner, down = right corner)";
        }
    }

    /** While driving, a log line every SAMPLE_PERIOD_SEC. */
    private void sample(Input in) {
        if (state != State.DRIVING || sampleClock.seconds() < SAMPLE_PERIOD_SEC) return;
        sampleClock.reset();
        Pose2d p = in.pose;
        double speed = in.measured == null ? Double.NaN : Math.hypot(in.measured.linearVel.x, in.measured.linearVel.y);
        RunLog.event(String.format(Locale.US, "PARK DASH sample: t %.2f s, pose (%.1f, %.1f, %.1f deg), speed %.1f in/s, leg %d of %d", dashClock.seconds(), p.position.x, p.position.y,
                p.heading.toDegrees(), speed, legIndex + 1, plan.waypoints.length - 1));
    }
}
