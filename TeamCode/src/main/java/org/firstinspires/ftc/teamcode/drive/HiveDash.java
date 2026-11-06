package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.List;
import java.util.Locale;

/**
 * HIVE DASH (design/hive-dash.md): in TeleOp - Competition the driver HOLDS D-pad left or right and the robot drives to the launch spot of a hive cell (the Auto Launch
 * zone centre), target locks and launches once ALIGNED. Left / right follow the DRIVER'S view of the field (blue: left = audience cell, right = rear cell; red the other way
 * round). Releasing the D-pad cancels at once and gives the sticks back, except that a salvo already launched always finishes.
 *
 * The route comes from {@link HiveRoute} (a straight line when clear, otherwise through the helper corners and the lane gates under the hive). Each leg is driven ALIGNED with
 * the wheels (front or rear leading, a mecanum chassis is about a third faster that way than strafing: TeleOp logs 2026-11-01 to 11-03 gave strafe 0.75 of forward) at the
 * dash's power along the leg with a sideways correction; the last leg drives straight at the goal point and eases off near it, so an overshoot is driven back; within
 * LOCK_RADIUS_IN of the goal the target lock takes the heading.
 *
 * This class holds the state machine and the follower and touches no hardware: {@code TeleOpCompetition} gives it the loop's buffered pose and flags (nothing here reads a
 * sensor) and applies the {@link Output}. The dash is REFUSED, with no movement, when: backup mode / a Pinpoint fault / the pose is not seeded, the element counter reads and
 * holds nothing, or the robot is touching a hive support. If the count falls to zero on the way, the dash goes on and does not launch an empty robot on arrival.
 */
@Config
public final class HiveDash {

    // ---- driving ----
    /** The most power along a leg. 0.5 for the first runs (about 40 in/s on aligned legs, 13 dashes on 2026-11-05 with arrivals 1.5 to 2.5 in from the spot); 0.85 from 2026-11-05 (user), the power the Solo auto's cross-field legs use (58 in/s peak). At 0.85 the easing starts 17 in out (ALONG_KP 0.05). */
    public static double MAX_POWER = 0.85;
    /** Within this many inches of the goal the target lock takes the heading and the speed eases. */
    public static double LOCK_RADIUS_IN = 20.0;
    /**
     * On the last leg the target lock takes the heading this far out (and not before the robot is at least EARLY_LOCK_MIN_ABS_X from the field center, so never under or beside the hive), so the
     * turn onto the cell is done on arrival: in the 2026-11-06 log the arrival ran 0 to 0.56 s ahead of the aim because the turn only began 20 in out. 0 or less: the old radius only.
     */
    public static double EARLY_LOCK_RADIUS_IN = 32.0, EARLY_LOCK_MIN_ABS_X = 36.0;
    /** The lock is ALIGNED and the chassis stopped for this long before the launch (was AutoLaunch.ALIGNED_HOLD_SEC, 0.15 s, which the dash waited out on every arrival); 0 launches the moment both hold. */
    public static double LAUNCH_HOLD_SEC = 0.0;
    public static double GOAL_TOL_IN = 2.5;
    /** A via point counts as reached this close (the robot goes on without stopping). */
    public static double PASS_TOL_IN = 3.0;
    /** The last leg drives straight at the goal point: power per inch to the goal (the same 0.05 as DriveToPose), and the least power so it does not stall short. */
    public static double ALONG_KP = 0.05;
    public static double MIN_FINAL_POWER = 0.08;
    /** Sideways correction: power per inch off the leg, and its limit. */
    public static double CROSS_KP = 0.05, CROSS_MAX = 0.3;
    /** Heading: turn power per radian of error, per rad/s of turn rate, and its limit. */
    public static double HEADING_KP = 1.5, HEADING_KD = 0.08, TURN_MAX = 0.5;   // kept at 0.5 at the user's request when the power went to 0.85 (translation 0.85 + turn 0.5 exceeds 1, so the wheel scaling slows the robot a little in hard turns)
    /** On an aligned leg the speed is scaled by cos(heading error), but never below this. */
    public static double ALIGN_FLOOR = 0.4;
    /**
     * Overshoot control (2026-11-06; the log had dashes arriving at 20 to 55 in/s and coasting 2 to 8 in past the spot, and two that swung 17 in wide off a helper corner into the audience flower).
     * On the last leg the power is limited to APPROACH_POWER once the robot is within APPROACH_RADIUS_IN of the spot (where the aim turn starts); 0 turns it off. 0.5 is about 40 in/s, which
     * stops in about 3 in (the measured deceleration is about 240 in/s^2).
     */
    public static double APPROACH_POWER = 0.5, APPROACH_RADIUS_IN = 32.0;
    /**
     * Before a via point the power ramps down to a corner power that depends on the turn there: no slowing up to CORNER_FREE_DEG of turn, CORNER_POWER from CORNER_FULL_DEG on, in between in proportion;
     * the ramp starts CORNER_RAMP_PER_IN of power per inch out (0.03: about 12 in for 0.85 down to 0.5). CORNER_POWER 0 or CORNER_FULL_DEG below CORNER_FREE_DEG turns it off.
     */
    public static double CORNER_POWER = 0.5, CORNER_FREE_DEG = 20.0, CORNER_FULL_DEG = 90.0, CORNER_RAMP_PER_IN = 0.03;

    // ---- planning ----
    public static double ROBOT_HEIGHT_IN = 18.0;
    public static double MARGIN_IN = 1.5;
    /** Legs at least this long are driven aligned with the wheels; shorter ones are not (the turn costs more than it saves). */
    public static double ALIGN_MIN_LEG_IN = 40.0;
    public static double TURN_PENALTY_IN_PER_DEG = 0.1;
    /** Speed of an unaligned leg against an aligned one, measured 2026-11-04 from the TeleOp logs (strafe 0.75, diagonals 0.72). */
    public static double UNALIGNED_SPEED_RATIO = 0.75;
    /** True: routes between two outside points may use the lane under the hive. A robot that starts under the hive can always leave. */
    public static boolean UNDER_HIVE_ROUTES = true;
    /** Under the hive the route keeps the robot's center within (this - 8.35) in of the lane's middle line (the planner's virtual lane walls, see HiveRoute.Params.laneHalfWidthIn). 22.75 or more turns it off. */
    public static double LANE_HALF_WIDTH_IN = 12.35;
    /** The helper corners (+-HELPER_X, +-HELPER_Y) and the lane gates (+-GATE_X, 0). */
    public static double HELPER_X = 36.0, HELPER_Y = 40.0, GATE_X = 34.0;
    /** True: plan, log and draw the route but do not drive. */
    public static boolean DRY_RUN = false;

    // ---- launch and logging ----
    /** The launch is requested when the lock is ALIGNED and the chassis has stopped (AutoLaunch.MAX_SPEED_IN_PER_S) for LAUNCH_HOLD_SEC. */
    public static double LAUNCH_START_TIMEOUT_SEC = 1.0, LAUNCH_DONE_TIMEOUT_SEC = 6.0;
    public static double SAMPLE_PERIOD_SEC = 0.25;
    /** False: never send the Dashboard field overlay (a view for testing; each packet costs a few ms of JSON). Turn it off for matches. */
    public static boolean OVERLAY_ENABLED = true;
    public static double OVERLAY_PERIOD_SEC = 0.2, PLAN_SHOWN_SEC = 5.0;
    public static double REFUSED_SHOWN_SEC = 3.0;

    public enum State { IDLE, DRIVING, ARRIVED, LAUNCHING }

    /** What the TeleOp gives the dash each loop. */
    public static final class Input {
        public Pose2d pose;
        public PoseVelocity2d measured;
        /** The cell of the held D-pad direction, or null when neither (or both) is held. */
        public HiveTarget requestedCell;
        public boolean blue;
        /** Why the dash is not available (backup mode, a Pinpoint fault, ...), or null. */
        public String blockedWhy;
        /** The element counter is reading and holds nothing. */
        public boolean counterReadsEmpty;
        /** The robot may have something to launch (the counter is not reading, or it holds at least one). */
        public boolean hasElements;
        public boolean lockAligned;
        public boolean launchBusy;
    }

    /** What the dash wants done this loop. */
    public static final class Output {
        /** The dash drives the chassis (forward / left in robot powers; zero while it holds at the goal). */
        public boolean translation;
        public double forward, left;
        /** The dash turns the chassis (heading along the leg); when false and lockWanted is true the target lock turns it. */
        public boolean turnActive;
        public double turn;
        public boolean lockWanted;
        public boolean launchNow;
        /** The cell the launch speed and the lock must use for as long as the dash needs it, or null (the nearest-cell logic resumes). */
        public HiveTarget overrideCell;
        /** While overrideCell is set: the distance the launch speed must be worked out for (the launch spot to the cell), so the flywheel goes to that speed at the start of the dash; 0 otherwise. */
        public double overrideDistanceIn;
        /** The D-pad was just refused (give the driver a short rumble). */
        public boolean refused;

        void reset() {
            translation = false;
            forward = left = 0.0;
            turnActive = false;
            turn = 0.0;
            lockWanted = false;
            launchNow = false;
            overrideCell = null;
            overrideDistanceIn = 0.0;
            refused = false;
        }
    }

    /**
     * The cell a D-pad direction means, following the driver's view: the driver looks along driverForwardRad (blue: toward -Y, red: toward +Y), so the driver's left is the
     * field direction whose x component is -sin(driverForwardRad): +X (the audience wall) for blue, -X (the rear wall) for red. Null unless exactly one of the two is held.
     */
    public static HiveTarget cellForDpad(boolean left, boolean right, boolean blue, double driverForwardRad) {
        if (left == right) return null;
        boolean leftIsPositiveX = -Math.sin(driverForwardRad) > 0.0;
        HiveTarget positiveX = blue ? HiveTarget.BLUE_AUDIENCE : HiveTarget.RED_AUDIENCE;
        HiveTarget negativeX = blue ? HiveTarget.BLUE_REAR : HiveTarget.RED_REAR;
        HiveTarget leftCell = leftIsPositiveX ? positiveX : negativeX;
        HiveTarget rightCell = leftIsPositiveX ? negativeX : positiveX;
        return left ? leftCell : rightCell;
    }

    private final double depthIn, widthIn, fromBackIn;
    private final Output out = new Output();
    private State state = State.IDLE;
    private boolean pressConsumed = false;   // this press has been used (started, refused or finished): the D-pad must be released first
    private HiveTarget cell;
    private double goalX, goalY;
    private HiveRoute.Plan plan;
    private HiveRoute.Params planParams;
    private int legIndex = 0;
    private double legHeading = 0.0, prevLegHeading = 0.0;
    private boolean legAligned = false;
    private final ElapsedTime stillClock = new ElapsedTime();
    private final ElapsedTime launchClock = new ElapsedTime();
    private final ElapsedTime dashClock = new ElapsedTime();
    private final ElapsedTime sampleClock = new ElapsedTime();
    private final ElapsedTime overlayClock = new ElapsedTime();
    private final ElapsedTime planShownClock = new ElapsedTime();
    private final ElapsedTime refusedClock = new ElapsedTime();
    private boolean planShown = false, sawBusy = false, everRefused = false;
    private String refusedWhy = "";
    private Pose2d lastSamplePose;
    private String status = "ready";

    public HiveDash(double depthIn, double widthIn, double fromBackIn) {
        this.depthIn = depthIn;
        this.widthIn = widthIn;
        this.fromBackIn = fromBackIn;
        // one throwaway plan now, so the first real press does not pay for loading and warming the planner (about 25 ms cold against 0.05 ms warm, measured on a PC)
        HiveRoute.plan(-55.4, 15.25, 0.0, 55.4, 7.5, params());
    }

    public State state() {
        return state;
    }

    /** One line for the telemetry. */
    public String status() {
        return status;
    }

    private HiveRoute.Params params() {
        HiveRoute.Params p = new HiveRoute.Params();
        p.robotHeightIn = ROBOT_HEIGHT_IN;
        p.marginIn = MARGIN_IN;
        p.depthIn = depthIn;
        p.widthIn = widthIn;
        p.fromBackIn = fromBackIn;
        p.alignMinLegIn = ALIGN_MIN_LEG_IN;
        p.turnPenaltyInPerDeg = TURN_PENALTY_IN_PER_DEG;
        p.unalignedSpeedRatio = UNALIGNED_SPEED_RATIO;
        p.underHive = UNDER_HIVE_ROUTES;
        p.laneHalfWidthIn = LANE_HALF_WIDTH_IN;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) p.points.add(new double[]{sx * HELPER_X, sy * HELPER_Y});
            p.points.add(new double[]{sx * GATE_X, 0.0});
        }
        return p;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Call once per loop with the loop's buffered pose; returns the (reused) output. */
    public Output update(Input in) {
        out.reset();
        boolean held = in.requestedCell != null;
        if (!held) pressConsumed = false;
        switch (state) {
            case IDLE:
                if (held && !pressConsumed) {
                    pressConsumed = true;
                    tryStart(in);
                }
                break;
            case DRIVING:
            case ARRIVED:
                if (!held) {
                    cancel(in, "the D-pad was released");
                } else if (in.requestedCell != cell) {
                    cancel(in, "the other D-pad direction was pressed");
                } else if (in.blockedWhy != null) {
                    cancel(in, in.blockedWhy);
                } else if (state == State.DRIVING) {
                    drive(in);
                } else {
                    arrived(in);
                }
                break;
            case LAUNCHING:
                launching(in, held);
                break;
            default:
                break;
        }
        if (out.overrideCell != null) out.overrideDistanceIn = cell.distanceFrom(new Pose2d(goalX, goalY, 0.0));
        status = statusLine(in);
        sample(in);
        overlay(in);
        return out;
    }

    // ---- start / refuse / cancel ----

    private void refuse(HiveTarget requested, String why) {
        out.refused = true;
        refusedWhy = why;
        everRefused = true;
        refusedClock.reset();
        RunLog.event("HIVE DASH refused (" + requested + "): " + why);
    }

    private void tryStart(Input in) {
        HiveTarget requested = in.requestedCell;
        if (in.blockedWhy != null) {
            refuse(requested, "not available: " + in.blockedWhy);
            return;
        }
        if (in.counterReadsEmpty) {
            refuse(requested, "the robot holds nothing to launch");
            return;
        }
        HiveRoute.Params p = params();
        double gx = AutoLaunch.centerX(requested), gy = AutoLaunch.centerY(requested);
        Pose2d pose = in.pose;
        HiveRoute.Plan pl = HiveRoute.plan(pose.position.x, pose.position.y, pose.heading.toDouble(), gx, gy, p);
        planParams = p;
        plan = pl;
        planShown = true;
        planShownClock.reset();
        if (!pl.ok) {
            refuse(requested, pl.refusal);
            return;
        }
        RunLog.event(String.format(Locale.US, "HIVE DASH %s (%s): from (%.1f, %.1f, %.1f deg) to the launch spot (%.1f, %.1f), %s; under-hive routes %s, power %.2f",
                DRY_RUN ? "dry run" : "start", requested, pose.position.x, pose.position.y, pose.heading.toDegrees(), gx, gy, pl.description,
                p.underHive ? "on" : "off", MAX_POWER));
        if (DRY_RUN) return;
        cell = requested;
        goalX = gx;
        goalY = gy;
        legIndex = 0;
        prevLegHeading = pose.heading.toDouble();
        startLeg(in);
        dashClock.reset();
        sampleClock.reset();
        lastSamplePose = pose;
        state = State.DRIVING;
        out.overrideCell = cell;
        drive(in);
    }

    private void cancel(Input in, String why) {
        Pose2d pose = in.pose;
        RunLog.event(String.format(Locale.US, "HIVE DASH cancelled (%s) at (%.1f, %.1f, %.1f deg), %.1f in from the launch spot, leg %d of %d, after %.2f s",
                why, pose.position.x, pose.position.y, pose.heading.toDegrees(), Math.hypot(pose.position.x - goalX, pose.position.y - goalY), legIndex + 1,
                plan.waypoints.length - 1, dashClock.seconds()));
        state = State.IDLE;
    }

    // ---- driving ----

    private void startLeg(Input in) {
        double[] a = plan.waypoints[legIndex], b = plan.waypoints[legIndex + 1];
        double dir = Math.atan2(b[1] - a[1], b[0] - a[0]);
        legAligned = plan.aligned[legIndex];
        boolean last = legIndex == plan.waypoints.length - 2;
        // front or rear leading: the one nearer the heading the robot has (or, on the last leg, nearer the aim at the cell, so arriving rear-first already faces it)
        double ref = last ? TargetLock.aimHeadingRad(goalX, goalY, cell, TargetLock.farCellExtraLeftIn(cell, in.blue)) : prevLegHeading;
        double front = dir, rear = dir + Math.PI;
        legHeading = Math.abs(Rotation2d.normalize(front - ref)) <= Math.abs(Rotation2d.normalize(rear - ref)) ? front : rear;
        prevLegHeading = legHeading;
        RunLog.event(String.format(Locale.US, "HIVE DASH leg %d of %d: (%.1f, %.1f) -> (%.1f, %.1f), %.0f in, %s%s", legIndex + 1, plan.waypoints.length - 1, a[0], a[1], b[0], b[1],
                Math.hypot(b[0] - a[0], b[1] - a[1]), legAligned ? "aligned, " + (legHeading == front ? "front" : "rear") + " leading" : "short, not aligned", last ? ", last leg" : ""));
    }

    private void drive(Input in) {
        Pose2d pose = in.pose;
        double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
        int legs = plan.waypoints.length - 1;
        double goalDist = Math.hypot(x - goalX, y - goalY);
        double ux = 0, uy = 0, nx = 0, ny = 0, along = 0, cross = 0, remaining = 0;
        for (int guard = 0; guard < 4; guard++) {
            double[] a = plan.waypoints[legIndex], b = plan.waypoints[legIndex + 1];
            double len = Math.hypot(b[0] - a[0], b[1] - a[1]);
            ux = len < 1e-6 ? 1.0 : (b[0] - a[0]) / len;
            uy = len < 1e-6 ? 0.0 : (b[1] - a[1]) / len;
            nx = -uy;
            ny = ux;
            along = (x - a[0]) * ux + (y - a[1]) * uy;
            cross = (x - a[0]) * nx + (y - a[1]) * ny;
            remaining = len - along;
            boolean last = legIndex == legs - 1;
            if (last) {
                if (goalDist <= GOAL_TOL_IN) {
                    state = State.ARRIVED;
                    stillClock.reset();
                    RunLog.event(String.format(Locale.US, "HIVE DASH arrived after %.2f s at (%.1f, %.1f, %.1f deg), %.1f in from the launch spot", dashClock.seconds(), x, y,
                            pose.heading.toDegrees(), goalDist));
                    out.overrideCell = cell;
                    arrived(in);
                    return;
                }
                break;
            }
            if (remaining > PASS_TOL_IN && along <= len) break;
            RunLog.event(String.format(Locale.US, "HIVE DASH via point (%.0f, %.0f) reached at (%.1f, %.1f), %.2f s", b[0], b[1], x, y, dashClock.seconds()));
            legIndex++;
            startLeg(in);
        }
        boolean last = legIndex == legs - 1;
        double vfx, vfy;
        if (last) {
            // the last leg heads straight for the goal point: a robot that overshoots it or passes it to the side is driven back (power along the leg kept pushing it on past)
            double v = clamp(ALONG_KP * goalDist, MIN_FINAL_POWER, MAX_POWER);
            if (APPROACH_POWER > 0.0 && goalDist <= APPROACH_RADIUS_IN) v = Math.min(v, Math.max(APPROACH_POWER, MIN_FINAL_POWER));
            double d = Math.max(goalDist, 1e-6);
            vfx = (goalX - x) / d * v;
            vfy = (goalY - y) / d * v;
        } else {
            double vC = clamp(-CROSS_KP * cross, -CROSS_MAX, CROSS_MAX);
            double vMag = MAX_POWER;
            if (CORNER_POWER > 0.0 && CORNER_FULL_DEG > CORNER_FREE_DEG) {
                // slow down for the turn at the via point ahead: the next leg's direction against this one's
                double[] via = plan.waypoints[legIndex + 1], next = plan.waypoints[legIndex + 2];
                double turnDeg = Math.toDegrees(Math.abs(Rotation2d.normalize(Math.atan2(next[1] - via[1], next[0] - via[0]) - Math.atan2(uy, ux))));
                double f = clamp((turnDeg - CORNER_FREE_DEG) / (CORNER_FULL_DEG - CORNER_FREE_DEG), 0.0, 1.0);
                double cornerPower = MAX_POWER + (Math.min(CORNER_POWER, MAX_POWER) - MAX_POWER) * f;
                vMag = Math.min(MAX_POWER, cornerPower + CORNER_RAMP_PER_IN * Math.max(0.0, remaining));
            }
            vfx = ux * vMag + nx * vC;
            vfy = uy * vMag + ny * vC;
        }
        double fwd = vfx * Math.cos(h) + vfy * Math.sin(h);
        double left = -vfx * Math.sin(h) + vfy * Math.cos(h);
        boolean inLock = goalDist <= LOCK_RADIUS_IN
                || (last && EARLY_LOCK_RADIUS_IN > 0.0 && goalDist <= EARLY_LOCK_RADIUS_IN && Math.abs(x) >= EARLY_LOCK_MIN_ABS_X);
        out.overrideCell = cell;
        if (inLock) {
            out.lockWanted = true;
        } else {
            double target = legAligned ? legHeading : TargetLock.aimHeadingRad(goalX, goalY, cell, TargetLock.farCellExtraLeftIn(cell, in.blue));
            double err = Rotation2d.normalize(target - h);
            double angVel = in.measured == null ? 0.0 : in.measured.angVel;
            out.turn = clamp(HEADING_KP * err - HEADING_KD * angVel, -TURN_MAX, TURN_MAX);
            out.turnActive = true;
            if (legAligned) {
                double scale = Math.max(ALIGN_FLOOR, Math.cos(err));
                fwd *= scale;
                left *= scale;
            }
        }
        out.translation = true;
        out.forward = fwd;
        out.left = left;
    }

    private void arrived(Input in) {
        out.overrideCell = cell;
        out.translation = true;   // hold: no drive power
        out.lockWanted = true;
        double speed = in.measured == null ? 0.0 : Math.hypot(in.measured.linearVel.x, in.measured.linearVel.y);
        boolean still = in.lockAligned && speed <= AutoLaunch.MAX_SPEED_IN_PER_S;
        if (!still) stillClock.reset();
        if (!in.hasElements) return;   // nothing to launch: wait for the driver to let go
        if (still && stillClock.seconds() >= LAUNCH_HOLD_SEC && !in.launchBusy) {   // 'still' is needed on its own: with a hold of 0 the reset clock already reads 0 (the 2026-11-06 00:43 log launched on arrival at heading 55 deg and at up to 48 in/s)
            out.launchNow = true;
            state = State.LAUNCHING;
            sawBusy = false;
            launchClock.reset();
            RunLog.event(String.format(Locale.US, "HIVE DASH launching: aligned and stopped (%.1f in/s) %.2f s after the start", speed, dashClock.seconds()));
        }
    }

    private void launching(Input in, boolean held) {
        out.overrideCell = cell;
        if (held) {
            out.translation = true;   // hold still until the salvo is done
            out.lockWanted = true;
        }
        if (in.launchBusy) sawBusy = true;
        String done = null;
        if (sawBusy && !in.launchBusy) done = "the salvo is done";
        else if (!sawBusy && launchClock.seconds() > LAUNCH_START_TIMEOUT_SEC) done = "the launch did not start";
        else if (launchClock.seconds() > LAUNCH_DONE_TIMEOUT_SEC) done = "the salvo took too long";
        if (done != null) {
            RunLog.event(String.format(Locale.US, "HIVE DASH finished: %s, %.2f s in all%s", done, dashClock.seconds(), held ? "" : " (the D-pad had been released)"));
            state = State.IDLE;
            out.overrideCell = null;
            out.translation = false;
            out.lockWanted = false;
        }
    }

    // ---- telemetry, log, overlay ----

    private String statusLine(Input in) {
        switch (state) {
            case DRIVING:
                return String.format(Locale.US, "DRIVING to %s: leg %d of %d, %.0f in from the launch spot", cell, legIndex + 1, plan.waypoints.length - 1,
                        Math.hypot(in.pose.position.x - goalX, in.pose.position.y - goalY));
            case ARRIVED:
                return in.hasElements ? "AT THE LAUNCH SPOT (" + cell + "): aligning, then it launches" : "AT THE LAUNCH SPOT (" + cell + "): nothing to launch, let go of the D-pad";
            case LAUNCHING:
                return "LAUNCHING at " + cell;
            default:
                if (everRefused && refusedClock.seconds() < REFUSED_SHOWN_SEC) return "REFUSED: " + refusedWhy;
                return DRY_RUN ? "ready (DRY RUN: plans and draws, does not drive)" : "ready (hold D-pad left / right)";
        }
    }

    /** While driving, a log line every SAMPLE_PERIOD_SEC with the speed against the travel direction (the strafe-to-forward measurement). */
    private void sample(Input in) {
        if (state != State.DRIVING || sampleClock.seconds() < SAMPLE_PERIOD_SEC) return;
        double dt = sampleClock.seconds();
        sampleClock.reset();
        Pose2d p = in.pose;
        if (lastSamplePose != null) {
            double dx = p.position.x - lastSamplePose.position.x, dy = p.position.y - lastSamplePose.position.y;
            double speed = Math.hypot(dx, dy) / dt;
            double travel = Math.hypot(dx, dy) < 0.2 ? Double.NaN : Math.toDegrees(Rotation2d.normalize(Math.atan2(dy, dx) - p.heading.toDouble()));
            RunLog.event(String.format(Locale.US, "HIVE DASH sample: t %.2f s, pose (%.1f, %.1f, %.1f deg), speed %.1f in/s, travel %.0f deg off the front, leg %d of %d (%s)",
                    dashClock.seconds(), p.position.x, p.position.y, p.heading.toDegrees(), speed, travel, legIndex + 1, plan.waypoints.length - 1, legAligned ? "aligned" : "short"));
        }
        lastSamplePose = p;
    }

    /** The supports, the route, the goal and the robot on the Dashboard's field view while a plan is shown. Sent as its own packet at 5 per second. */
    private void overlay(Input in) {
        boolean show = state != State.IDLE || (planShown && planShownClock.seconds() < PLAN_SHOWN_SEC);
        if (!OVERLAY_ENABLED || !show || plan == null || planParams == null || overlayClock.seconds() < OVERLAY_PERIOD_SEC) return;
        overlayClock.reset();
        try {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();
            List<HiveRoute.Rect> rects = HiveRoute.planningKeepOuts(planParams);
            for (HiveRoute.Rect r : rects) {
                double[] k = r.corners();
                double[] xs = {k[0], k[2], k[4], k[6]}, ys = {k[1], k[3], k[5], k[7]};
                c.setStroke(r.lane ? "#B0BEC5" : "#E53935");
                c.strokePolygon(xs, ys);
            }
            if (plan.ok) {
                double[] xs = new double[plan.waypoints.length], ys = new double[plan.waypoints.length];
                for (int i = 0; i < xs.length; i++) {
                    xs[i] = plan.waypoints[i][0];
                    ys[i] = plan.waypoints[i][1];
                }
                c.setStroke("#1E88E5");
                c.strokePolyline(xs, ys);
                c.setStroke("#43A047");
                double[] last = plan.waypoints[plan.waypoints.length - 1];
                c.strokeCircle(last[0], last[1], LOCK_RADIUS_IN);
            }
            HiveRoute.Rect me = HiveRoute.poseRect(in.pose.position.x, in.pose.position.y, in.pose.heading.toDouble(),
                    planParams, 0.0);
            double[] k = me.corners();
            c.setStroke("#FB8C00");
            c.strokePolygon(new double[]{k[0], k[2], k[4], k[6]}, new double[]{k[1], k[3], k[5], k[7]});
            packet.put("Hive Dash", status);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } catch (RuntimeException e) {
            // the overlay is only a view: never let it stop the dash
        }
    }
}
