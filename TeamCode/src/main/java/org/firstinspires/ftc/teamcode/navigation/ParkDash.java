package org.firstinspires.ftc.teamcode.navigation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.robot.Footprint;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Park Dash (specs/dashes.md section 4): while D-pad up or down is held, drive to a park corner of the loading zone
 * and hold there. No cell, no lock, no launch. Pure logic. It plans with the same planner and the same values as the
 * Hive Dash, but has no corner slow-down and no approach limit.
 */
@Config
public final class ParkDash {
    public static double MAX_POWER = 0.85;
    public static double MAX_TURN = 0.5;
    public static double GOAL_TOL_IN = 2.0;
    public static double PASS_TOL_IN = 3.0;
    public static boolean DRY_RUN = false;
    /** Use the grid routes when their tables can be used. Off: never run on the robot. */
    public static boolean GRID_ROUTES = false;

    public enum State { IDLE, DRIVING, PARKED }

    public static final class In {
        public Pose pose;
        public double omega, t;
        /** Up is the corner on the driver's left, down the corner on the driver's right. */
        public boolean up, down;
        public Alliance alliance;
        public String unavailable;
        public boolean hiveDashRunning;
    }

    public static final class Out {
        public boolean active, driving;
        public double forward, left, turn;
        /** At the corner: no drive power from the dash; it asks the TeleOp for hold position. */
        public boolean parked;
        /** True on the one loop the dash starts: stow the ramp, without waiting for it. */
        public boolean stowRamp;
        public String refusal;
        public String activity;
    }

    private final Footprint fp;
    private final LegFollower follower = new LegFollower();
    private final Out out = new Out();
    private State state = State.IDLE;
    private boolean blocked, dirUp;
    private Plan plan;
    private int headingLeg = -1;
    private double leadAngle, refusedAt = -100, startedAt;
    private String refusal;

    public ParkDash(Footprint fp) { this.fp = fp; }

    public State state() { return state; }
    public boolean running() { return state != State.IDLE; }
    public Plan plan() { return plan; }

    public Out update(In in) {
        out.active = out.driving = out.parked = out.stowRamp = false;
        out.forward = out.left = out.turn = 0;
        out.activity = null;
        out.refusal = (in.t - refusedAt <= HiveDash.REFUSAL_SHOWN_S) ? refusal : null;

        boolean pressed = in.up != in.down;
        if (!pressed || (running() && in.up != dirUp)) {
            // Releasing the D-pad, or pressing the other direction, cancels at once.
            if (running()) RunLog.eventf("PARK DASH cancelled in %s after %.2f s at %s", state, in.t - startedAt, in.pose);
            state = State.IDLE;
            plan = null;
            if (!pressed) {
                blocked = false;
                return out;
            }
        }
        if (state == State.IDLE && !blocked) start(in);
        if (!running()) return out;

        out.active = true;
        if (state == State.DRIVING) {
            follower.update(in.pose.x, in.pose.y, params());
            int leg = follower.leg();
            double scale = 1.0;
            if (plan.aligned[leg]) {
                if (headingLeg != leg) {
                    headingLeg = leg;
                    leadAngle = HiveDash.lead(plan.legAngle(leg), in.pose.h);
                }
                double err = Angle.diff(leadAngle, in.pose.h);
                out.turn = Angle.clampAbs(HiveDash.HEADING_KP * err - HiveDash.HEADING_KD * in.omega, MAX_TURN);
                scale = Math.max(HiveDash.ALIGN_SPEED_FLOOR, Math.cos(err));
            }
            double fx = follower.vx() * scale, fy = follower.vy() * scale, h = in.pose.h;
            out.forward = fx * Math.cos(h) + fy * Math.sin(h);
            out.left = -fx * Math.sin(h) + fy * Math.cos(h);
            out.driving = true;
            out.activity = "PARK DASH";
            if (follower.arrived()) {
                state = State.PARKED;
                out.forward = out.left = out.turn = 0;
                out.driving = false;
                RunLog.eventf("PARK DASH arrived at %s after %.2f s", in.pose, in.t - startedAt);
            }
        }
        if (state == State.PARKED) {
            out.parked = true;
            out.activity = "PARKED";
        }
        return out;
    }

    private void start(In in) {
        dirUp = in.up;
        Vec goal = in.alliance.point(Field.parkDashCorner(in.up));
        String why = null;
        if (in.unavailable != null) why = in.unavailable;
        else if (in.hiveDashRunning) why = "the Hive Dash is running";
        else if (RoutePlanner.touchesSupport(in.pose, fp)) why = "the robot touches a hive support";
        Plan p = null;
        if (why == null) {
            if (GRID_ROUTES) {
                String g = GridRoutes.unusable(Field.parkDashCorner(in.up));
                if (g != null) {
                    RunLog.event("PARK DASH grid routes not used: " + g + "; using the route planner");
                } else {
                    // The tables can be read and pass their checks, but picking a start and the grid follower are not built.
                    String note = GridRoutes.modelNote();
                    RunLog.event("PARK DASH grid routes not used: the tables are usable" + (note == null ? "" : " (" + note + ")")
                            + ", but the grid follower is not built; using the route planner");
                }
            }
            p = RoutePlanner.plan(in.pose, goal, fp);
            if (!p.ok()) why = p.refusal;
        }
        if (why != null) {
            refusal = why;
            refusedAt = in.t;
            blocked = true;
            RunLog.event("PARK DASH refused: " + why);
            return;
        }
        plan = p;
        RunLog.eventf("PARK DASH start to %s from %s: %s%s", goal, in.pose, p.describe(), DRY_RUN ? " (DRY RUN, not driving)" : "");
        if (DRY_RUN) {
            blocked = true;
            return;
        }
        follower.start(p);
        headingLeg = -1;
        startedAt = in.t;
        state = State.DRIVING;
        out.stowRamp = true;
    }

    private LegFollower.Params params() {
        LegFollower.Params k = new LegFollower.Params();
        k.maxPower = MAX_POWER;
        k.goalTol = GOAL_TOL_IN;
        k.passTol = PASS_TOL_IN;
        k.lastLegPerInch = HiveDash.LAST_LEG_PER_IN;
        k.lastLegMin = HiveDash.LAST_LEG_MIN;
        k.sidePerInch = HiveDash.SIDE_PER_IN;
        k.sideLimit = HiveDash.SIDE_LIMIT;
        k.cornerSlow = false;
        k.approachLimit = false;
        return k;
    }
}
