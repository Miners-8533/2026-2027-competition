package org.firstinspires.ftc.teamcode.navigation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.robot.Footprint;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Hive Dash (specs/dashes.md section 3): while a D-pad direction is held, drive to the launch spot of a hive cell,
 * let the target lock aim, and launch when aligned. Pure logic: the TeleOp hands it a snapshot each loop and applies
 * what it returns. Where the launch spot is and what the aim there is come from the TeleOp, through AimSource.
 */
@Config
public final class HiveDash {
    public static double MAX_POWER = 0.85;
    public static double MAX_TURN = 0.5;
    public static double LOCK_RADIUS_IN = 20.0;
    public static double EARLY_LOCK_RADIUS_IN = 32.0;
    /** The early lock applies only this far from the field center, so not under or beside the hive. */
    public static double EARLY_LOCK_MIN_X_IN = 36.0;
    public static double GOAL_TOL_IN = 2.5;
    public static double PASS_TOL_IN = 3.0;
    public static double LAST_LEG_PER_IN = 0.05, LAST_LEG_MIN = 0.08;
    public static double SIDE_PER_IN = 0.05, SIDE_LIMIT = 0.3;
    public static double HEADING_KP = 1.5, HEADING_KD = 0.08;
    public static double ALIGN_SPEED_FLOOR = 0.4;
    public static double APPROACH_POWER = 0.5, APPROACH_RADIUS_IN = 32.0;
    public static double CORNER_POWER = 0.5, CORNER_FREE_DEG = 20.0, CORNER_FULL_DEG = 90.0, CORNER_RAMP_PER_IN = 0.03;
    public static double HOLD_BEFORE_LAUNCH_S = 0.0;
    public static double LAUNCH_START_TIMEOUT_S = 1.0, SALVO_TIMEOUT_S = 6.0;
    public static double REFUSAL_SHOWN_S = 3.0;
    public static double STOPPED_IN_S = 4.0;
    /** Plan, log and draw the route without driving. */
    public static boolean DRY_RUN = false;
    /** Draw the route on the Dashboard's field view. Can be switched off for matches. */
    public static boolean OVERLAY = true;
    /** Use the grid routes when their tables can be used. Off: never run on the robot. */
    public static boolean GRID_ROUTES = false;

    /** Where a cell's launch spot is, and the aim heading from a point. Supplied by the TeleOp from the aiming code. */
    public interface AimSource {
        Vec launchSpot(Cell cell);
        double aimHeading(Vec from, Cell cell);
    }

    public enum State { IDLE, DRIVING, ARRIVED, LAUNCHING, FINISHED }

    /** The snapshot for one loop. */
    public static final class In {
        public Pose pose;
        public double vx, vy, omega, t;
        public boolean left, right;
        public Alliance alliance;
        /** Null when a dash is available; otherwise why not (backup mode, a Pinpoint fault, the pose not seeded). */
        public String unavailable;
        public boolean parkDashRunning;
        /** A count is being read and neither count holds anything. */
        public boolean readsEmpty;
        public boolean lockAligned;
        public boolean salvoRunning;
        public int salvoCount;
    }

    /** What the TeleOp applies. */
    public static final class Out {
        /** The dash has the robot: the driver's sticks are ignored. */
        public boolean active;
        public double forward, left;
        /** The dash turns the chassis; false once the lock has taken the heading. */
        public boolean hasTurn;
        public double turn;
        public boolean wantsLock;
        /** The dash's cell overrides the nearest-cell choice; null when idle. */
        public Cell cell;
        /** The launch speed is worked out for the launch spot from the start. */
        public double launchDistance;
        public boolean requestLaunch;
        /** A refusal to show on the screen, or null. */
        public String refusal;
        /** For the match screen's activity line, or null. */
        public String activity;
    }

    private final Footprint fp;
    private final AimSource aim;
    private final LegFollower follower = new LegFollower();
    private final Out out = new Out();

    private State state = State.IDLE;
    private boolean blocked;            // refused or finished: wait for the D-pad to be released
    private boolean dirLeft;
    private Cell cell;
    private Vec goal;
    private Plan plan;
    private int headingLeg = -1;
    private double leadAngle;
    private double arrivedAt, launchAt, refusedAt = -100, startedAt;
    private int salvoCountAtLaunch;
    private boolean launchSeen, emptyLogged;
    private String refusal;

    public HiveDash(Footprint fp, AimSource aim) {
        this.fp = fp;
        this.aim = aim;
    }

    public State state() { return state; }
    public boolean running() { return state == State.DRIVING || state == State.ARRIVED || state == State.LAUNCHING; }
    /** The plan to draw, or null. */
    public Plan plan() { return plan; }

    public Out update(In in) {
        out.active = false;
        out.forward = out.left = out.turn = 0;
        out.hasTurn = false;
        out.wantsLock = false;
        out.requestLaunch = false;
        out.cell = null;
        out.activity = null;
        out.refusal = (in.t - refusedAt <= REFUSAL_SHOWN_S) ? refusal : null;

        boolean pressed = in.left != in.right;
        if (!pressed) {
            if (running()) {
                RunLog.eventf("HIVE DASH cancelled in %s after %.2f s at %s", state, in.t - startedAt, in.pose);
            }
            state = State.IDLE;
            blocked = false;
            plan = null;
            return out;
        }
        if (running() && in.left != dirLeft) {
            RunLog.event("HIVE DASH cancelled: the other direction was pressed");
            state = State.IDLE;
        }
        if (state == State.IDLE && !blocked) start(in);
        if (!running()) return out;

        out.active = true;
        out.cell = cell;
        out.launchDistance = goal.dist(cell.point());
        double goalDist = in.pose.distTo(goal.x, goal.y);

        switch (state) {
            case DRIVING: {
                follower.update(in.pose.x, in.pose.y, params());
                boolean lastLeg = follower.onLastLeg();
                out.wantsLock = goalDist <= LOCK_RADIUS_IN
                        || (lastLeg && goalDist <= EARLY_LOCK_RADIUS_IN && Math.abs(in.pose.x) >= EARLY_LOCK_MIN_X_IN);
                double scale = 1.0;
                if (!out.wantsLock && plan.aligned[follower.leg()]) {
                    if (headingLeg != follower.leg()) {
                        headingLeg = follower.leg();
                        // Front or rear leading, whichever needs less rotation; on the last leg, whichever leaves the smaller turn onto the cell.
                        double ref = lastLeg ? aim.aimHeading(goal, cell) : in.pose.h;
                        leadAngle = lead(plan.legAngle(headingLeg), ref);
                    }
                    double err = Angle.diff(leadAngle, in.pose.h);
                    out.hasTurn = true;
                    out.turn = Angle.clampAbs(HEADING_KP * err - HEADING_KD * in.omega, MAX_TURN);
                    scale = Math.max(ALIGN_SPEED_FLOOR, Math.cos(err));
                } else if (!out.wantsLock) {
                    out.hasTurn = true;     // a short leg keeps the heading it has
                    out.turn = 0;
                }
                toRobot(follower.vx() * scale, follower.vy() * scale, in.pose.h);
                out.activity = "DASH: " + cell.label();
                if (follower.arrived()) {
                    state = State.ARRIVED;
                    arrivedAt = in.t;
                    emptyLogged = false;
                    out.forward = out.left = 0;
                    RunLog.eventf("HIVE DASH arrived at %s after %.2f s, %.1f in from the spot", in.pose, in.t - startedAt, goalDist);
                }
                break;
            }
            case ARRIVED: {
                out.wantsLock = true;
                out.activity = "DASH: AIMING";
                double speed = Math.hypot(in.vx, in.vy);
                if (in.lockAligned && speed < STOPPED_IN_S && in.t - arrivedAt >= HOLD_BEFORE_LAUNCH_S) {
                    if (in.readsEmpty) {
                        if (!emptyLogged) RunLog.event("HIVE DASH no launch: the robot reads empty on arrival");
                        emptyLogged = true;
                    } else {
                        out.requestLaunch = true;
                        state = State.LAUNCHING;
                        launchAt = in.t;
                        salvoCountAtLaunch = in.salvoCount;
                        launchSeen = false;
                        RunLog.eventf("HIVE DASH launch requested at %s, %.2f s after arriving", in.pose, in.t - arrivedAt);
                    }
                }
                break;
            }
            case LAUNCHING: {
                out.wantsLock = true;
                out.activity = "DASH: LAUNCHING";
                launchSeen |= in.salvoRunning;
                if (in.salvoCount > salvoCountAtLaunch) {
                    finish(String.format(Locale.US, "done: salvo finished %.2f s after the request", in.t - launchAt));
                } else if (!launchSeen && in.t - launchAt > LAUNCH_START_TIMEOUT_S) {
                    finish("ended: the launch did not start within " + LAUNCH_START_TIMEOUT_S + " s");
                } else if (in.t - launchAt > SALVO_TIMEOUT_S) {
                    finish("ended: the salvo did not finish within " + SALVO_TIMEOUT_S + " s");
                }
                break;
            }
            default:
                break;
        }
        return out;
    }

    private void finish(String why) {
        RunLog.event("HIVE DASH " + why);
        state = State.FINISHED;
        blocked = true;
    }

    private void start(In in) {
        dirLeft = in.left;
        cell = in.alliance.cellOnDriversLeft(in.left);
        goal = aim.launchSpot(cell);
        String why = null;
        if (in.unavailable != null) why = in.unavailable;
        else if (in.parkDashRunning) why = "the Park Dash is running";
        else if (in.readsEmpty) why = "the robot reads empty";
        else if (RoutePlanner.touchesSupport(in.pose, fp)) why = "the robot touches a hive support";
        Plan p = null;
        if (why == null) {
            if (GRID_ROUTES) {
                String g = GridRoutes.unusable(in.alliance.point(goal));
                if (g != null) {
                    RunLog.event("HIVE DASH grid routes not used: " + g + "; using the route planner");
                } else {
                    // The tables can be read and pass their checks, but picking a start and the grid follower are not built.
                    String note = GridRoutes.modelNote();
                    RunLog.event("HIVE DASH grid routes not used: the tables are usable" + (note == null ? "" : " (" + note + ")")
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
            RunLog.event("HIVE DASH refused: " + why);
            return;
        }
        plan = p;
        RunLog.eventf("HIVE DASH start to %s from %s: %s%s", cell.label(), in.pose, p.describe(), DRY_RUN ? " (DRY RUN, not driving)" : "");
        if (DRY_RUN) {
            blocked = true;
            return;
        }
        follower.start(p);
        headingLeg = -1;
        startedAt = in.t;
        state = State.DRIVING;
    }

    private LegFollower.Params params() {
        LegFollower.Params k = new LegFollower.Params();
        k.maxPower = MAX_POWER;
        k.goalTol = GOAL_TOL_IN;
        k.passTol = PASS_TOL_IN;
        k.lastLegPerInch = LAST_LEG_PER_IN;
        k.lastLegMin = LAST_LEG_MIN;
        k.sidePerInch = SIDE_PER_IN;
        k.sideLimit = SIDE_LIMIT;
        k.cornerSlow = true;
        k.cornerPower = CORNER_POWER;
        k.cornerFreeDeg = CORNER_FREE_DEG;
        k.cornerFullDeg = CORNER_FULL_DEG;
        k.cornerRampPerInch = CORNER_RAMP_PER_IN;
        k.approachLimit = true;
        k.approachPower = APPROACH_POWER;
        k.approachRadius = APPROACH_RADIUS_IN;
        return k;
    }

    /** The leg's direction or its reverse, whichever is nearer to a reference heading. */
    static double lead(double legAngle, double refHeading) {
        double back = Angle.wrap(legAngle + Math.PI);
        return Math.abs(Angle.diff(legAngle, refHeading)) <= Math.abs(Angle.diff(back, refHeading)) ? legAngle : back;
    }

    private void toRobot(double fx, double fy, double h) {
        out.forward = fx * Math.cos(h) + fy * Math.sin(h);
        out.left = -fx * Math.sin(h) + fy * Math.cos(h);
    }
}
