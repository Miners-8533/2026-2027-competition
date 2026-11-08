package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * The drive characterization routine (design/dash-planner-grid.md, "Drive characterization"): a fixed list of open-loop drive tests, each driven from a known start
 * on one alliance's HALF of the field, to fit the drive model from logs at the loop rate. Pure logic (no SDK, no hardware): {@code opmodes.tuning.DriveCharacterization}
 * gives it the pose and the deadman button each loop and sends the powers it returns; tools/dash/DashSim runs the same class against the simulated drive
 * ({@code run-sim.cmd drivetest}) to check the time and the geofence.
 *
 * Everything here is in the BLUE frame (the Blue Wall half, y &gt; 0); the opmode turns a red robot's pose 180 degrees about the field center before it calls
 * {@link #update}, and the powers it returns are robot-frame, so they need no turning.
 *
 * THE AREA: the tests run on a LANE along x at y = LANE_Y, from x = -LANE_END_X to +LANE_END_X (each run starts at the end the robot is nearer and drives to the
 * other end, so one test ends where the next starts), or at a SPOT on the lane (x = +-SPOT_X). A GEOFENCE stops a run at once (zero power, the motors brake) when
 * the robot's footprint, or a circle of its far-corner radius at the point where it would stop at FENCE_DECEL, would cross a wall (FENCE_WALL_IN), the center line
 * (y = FENCE_CENTER_LINE_IN: the other half may be in use), the hive (|x| &lt; FENCE_HIVE_X_IN, y &lt; FENCE_HIVE_Y_IN) or a flower stand grown by FENCE_FLOWER_GROW_IN.
 *
 * A TEST: drive to its start (a simple position controller, at most MOVE_MAX_POWER), wait until the robot is still (REST), then RUN its segments. A segment is a
 * constant command (along the lane, across it to the left of the travel, turn), turned into robot-frame powers once when it starts, so the powers sent are constant
 * for the segment (open loop). It ends after maxSec, or earlier when: the robot would need more than the lane left to stop at stopDecel; it has travelled
 * untilProgressIn along the lane; its speed fell below untilSpeedBelow; it has turned untilTurnedDeg.
 *
 * THE DEADMAN: nothing moves unless the deadman is held. Letting go zeroes the powers at once (PAUSED); holding it again restarts the current test from its start
 * (a run cut short is useless). The opmode's skip button ends the current test.
 */
@Config
public final class DriveTestRoutine {

    // ---- the area (blue frame) ----
    public static double LANE_Y = 46.0;
    public static double LANE_END_X = 54.0;
    public static double SPOT_X = 40.0;
    /** A run's stop is planned to end at least this far before the lane's end. */
    public static double STOP_MARGIN_IN = 6.0;
    /** The direction-change tests change direction this far along the run (past the hive's end, x about +-26). */
    public static double CHANGE_AT_IN = 80.0;

    // ---- the geofence ----
    public static double FENCE_WALL_IN = 68.0;
    public static double FENCE_CENTER_LINE_IN = 4.0;
    public static double FENCE_HIVE_X_IN = 22.5, FENCE_HIVE_Y_IN = 28.0;
    public static double FENCE_FLOWER_GROW_IN = 3.0;
    /**
     * The deceleration assumed for the predicted stop point (in/s^2). A braking robot slows at 250 or more and a coasting one at about 110; 200 lets the braking runs reach
     * their planned stop (lower values stop every one of them early) and the footprint check itself still keeps 3 in from the walls.
     */
    public static double FENCE_DECEL = 200.0;
    /** Below this turn rate (rad/s) the predicted stop uses the robot's own footprint (grown 1 in); above it, the square around its far-corner circle. */
    public static double FENCE_TURNING_RAD_PER_S = 1.0;

    // ---- driving to a test's start ----
    public static double MOVE_MAX_POWER = 0.6, MOVE_KP = 0.05, MOVE_KD = 0.004, MOVE_KS = 0.08;
    public static double TURN_KP = 1.5, TURN_KD = 0.08, TURN_MAX = 0.5, TURN_KS = 0.09;   // KS above the turn dead zone (about 0.07) so the last 2 deg do not stall
    public static double MOVE_TOL_IN = 2.0, MOVE_TOL_DEG = 2.0, MOVE_TIMEOUT_SEC = 6.0;   // 2 deg: an open-loop run 2 deg off drifts about 3.5 in over the lane
    /** Before a run the robot must be slower than REST_SPEED (in/s, and 10 deg/s) for REST_SEC (at most REST_MAX_SEC of waiting). */
    public static double REST_SPEED = 2.0, REST_SEC = 0.25, REST_MAX_SEC = 1.5;

    public enum Phase { MOVE, REST, RUN, PAUSED, DONE }

    /**
     * One constant command of a run. On the lane: along = toward the lane's far end, across = toward the alliance wall (+y in the blue frame; negative = toward the center
     * line), turn = curving toward the alliance wall (negative = toward the center line; counterclockwise when running toward +x). At a spot: the robot's own forward, left
     * and counterclockwise turn.
     */
    public static final class Seg {
        final double along, across, turn;
        double maxSec = 3.0, minSec = 0.0, stopDecel = 0.0;
        double untilProgressIn = Double.NaN, untilSpeedBelow = Double.NaN, untilTurnedDeg = Double.NaN;

        Seg(double along, double across, double turn) {
            this.along = along;
            this.across = across;
            this.turn = turn;
        }

        Seg max(double s) {
            maxSec = s;
            return this;
        }

        Seg min(double s) {
            minSec = s;
            return this;
        }

        /** Ends when the lane left is no more than the stop distance at this deceleration (plus STOP_MARGIN_IN). */
        Seg stopAt(double decel) {
            stopDecel = decel;
            return this;
        }

        Seg progress(double in) {
            untilProgressIn = in;
            return this;
        }

        Seg slowerThan(double inPerSec) {
            untilSpeedBelow = inPerSec;
            return this;
        }

        Seg turned(double deg) {
            untilTurnedDeg = deg;
            return this;
        }

        String describe() {
            return String.format(Locale.US, "along %+.2f, across %+.2f, turn %+.2f", along, across, turn);
        }
    }

    public static final class Test {
        public final String name;
        /** True: at a spot (spins, small random moves); false: a run along the lane. */
        final boolean spot;
        /** The robot's heading against the travel direction: 0 front first, 180 rear first, +-90 strafing, 45 diagonal; positive turns the front toward the alliance wall. */
        final double headingOffsetDeg;
        /** A spot test at this x (blue frame) instead of the spot nearer the robot; NaN for the nearer. */
        double spotX = Double.NaN;
        final List<Seg> segs;

        Test(String name, boolean spot, double headingOffsetDeg, Seg... segs) {
            this.name = name;
            this.spot = spot;
            this.headingOffsetDeg = headingOffsetDeg;
            this.segs = Arrays.asList(segs);
        }
    }

    private static Seg s(double along, double across, double turn) {
        return new Seg(along, across, turn);
    }

    private static Seg stop(double sec) {
        return new Seg(0.0, 0.0, 0.0).max(sec);
    }

    private static Test lane(String name, double offsetDeg, Seg... segs) {
        return new Test(name, false, offsetDeg, segs);
    }

    private static Test spot(String name, Seg... segs) {
        return new Test(name, true, 0.0, segs);
    }

    /**
     * The standard list, most important first (the open questions of the dash replay: how the robot stops, how it turns while rolling, how it changes direction
     * at speed), so a short session still gets them. About 4 minutes of driving in the simulation.
     */
    public static List<Test> standardTests() {
        List<Test> t = new ArrayList<>();
        // 1. stopping from speed: brake (zero power), coast (part power), drop to half power, reverse
        t.add(lane("stop: brake from 0.85, front first", 0, s(0.85, 0, 0).stopAt(250), stop(0.8)));
        t.add(lane("stop: brake from 0.85, rear first", 180, s(0.85, 0, 0).stopAt(250), stop(0.8)));
        t.add(lane("stop: coast on 0.2 from 0.85", 0, s(0.85, 0, 0).stopAt(80), s(0.2, 0, 0).slowerThan(12).min(0.1).max(2.0), stop(0.6)));
        t.add(lane("stop: coast on 0.2 from 0.85, rear first", 180, s(0.85, 0, 0).stopAt(80), s(0.2, 0, 0).slowerThan(12).min(0.1).max(2.0), stop(0.6)));
        t.add(lane("stop: drop from 0.85 to 0.45", 0, s(0.85, 0, 0).stopAt(55), s(0.45, 0, 0).max(0.6).stopAt(250), stop(0.6)));
        t.add(lane("stop: reverse 0.4 from 0.85", 0, s(0.85, 0, 0).stopAt(160), s(-0.4, 0, 0).slowerThan(10).min(0.05).max(0.6), stop(0.6)));
        t.add(lane("stop: brake from 0.85, strafing", -90, s(0.85, 0, 0).stopAt(250), stop(0.8)));
        t.add(lane("stop: coast on 0.2 from 0.85, strafing", -90, s(0.85, 0, 0).stopAt(80), s(0.2, 0, 0).slowerThan(12).min(0.1).max(2.0), stop(0.6)));
        t.add(lane("stop: brake from 0.6", 0, s(0.6, 0, 0).stopAt(250), stop(0.8)));
        t.add(lane("stop: coast on 0.2 from 0.6", 180, s(0.6, 0, 0).stopAt(80), s(0.2, 0, 0).slowerThan(12).min(0.1).max(2.0), stop(0.6)));
        // 2. turning while rolling (the corners of the Hive Dash: the robot turned up to 320 deg/s on a 0.5 command at speed)
        // (late in the run, curving toward the center line: past the hive's end that side has about 30 in of room, the wall side only about 14)
        t.add(lane("roll at 0.85 + turn 0.5 for 0.25 s", 0, s(0.85, 0, 0).progress(CHANGE_AT_IN).stopAt(250), s(0.85, 0, -0.5).max(0.25).stopAt(250), stop(0.8)));
        t.add(lane("roll at 0.85 + turn 0.5 for 0.25 s, rear first", 180, s(0.85, 0, 0).progress(CHANGE_AT_IN).stopAt(250), s(0.85, 0, -0.5).max(0.25).stopAt(250), stop(0.8)));
        t.add(lane("roll at 0.85 + turn 0.3 for 0.3 s", 0, s(0.85, 0, 0).progress(CHANGE_AT_IN).stopAt(250), s(0.85, 0, -0.3).max(0.3).stopAt(250), stop(0.8)));
        t.add(lane("roll at 0.85, then zero power + turn 0.5", 0, s(0.85, 0, 0).progress(CHANGE_AT_IN).stopAt(250), s(0, 0, -0.5).max(0.4), stop(0.8)));
        // 3. changing direction at speed (momentum and grip: the 17 in swing at the helper corner)
        // (late in the run and toward the center line: past the hive's end there are about 30 in of room on that side, toward the wall only about 14)
        t.add(lane("roll at 0.85, then 50 deg toward the center line", 0, s(0.85, 0, 0).progress(CHANGE_AT_IN).stopAt(250), s(0.55, -0.65, 0).max(0.3).stopAt(250), stop(0.8)));
        t.add(lane("roll at 0.85, then 50 deg toward the center line, rear first", 180, s(0.85, 0, 0).progress(CHANGE_AT_IN).stopAt(250), s(0.55, -0.65, 0).max(0.3).stopAt(250), stop(0.8)));
        t.add(lane("roll at 0.85, then a dash corner (0.2 along, 0.3 toward the center line, turn 0.5)", 0, s(0.85, 0, 0).progress(CHANGE_AT_IN).stopAt(250), s(0.2, -0.3, -0.5).max(0.3), stop(0.8)));
        // 4. steady speed against power: forward, strafe, diagonal
        t.add(lane("step forward 0.3", 0, s(0.3, 0, 0).stopAt(250).max(3.0), stop(0.6)));
        t.add(lane("step forward 0.5", 180, s(0.5, 0, 0).stopAt(250).max(3.0), stop(0.6)));
        t.add(lane("step forward 0.7", 0, s(0.7, 0, 0).stopAt(250).max(3.0), stop(0.6)));
        t.add(lane("step forward 1.0", 0, s(1.0, 0, 0).stopAt(250).max(3.0), stop(0.8)));
        t.add(lane("step strafe 0.4", -90, s(0.4, 0, 0).stopAt(250).max(3.0), stop(0.6)));
        t.add(lane("step strafe 0.7", 90, s(0.7, 0, 0).stopAt(250).max(3.0), stop(0.6)));
        t.add(lane("step strafe 1.0", -90, s(1.0, 0, 0).stopAt(250).max(3.0), stop(0.8)));
        t.add(lane("step diagonal 0.7", -45, s(0.7, 0, 0).stopAt(250).max(0.8), stop(0.6)));   // short, front toward the center line: strafing is weaker, so a diagonal drifts about 17 deg toward the front
        // 5. turning in place
        t.add(spot("spin 0.3", s(0, 0, 0.3).turned(300).max(2.0), stop(0.6)));
        t.add(spot("spin 0.5", s(0, 0, 0.5).turned(400).max(1.5), stop(0.6)));
        t.add(spot("spin -0.7", s(0, 0, -0.7).turned(450).max(1.2), stop(0.8)));
        t.add(spot("spin 1.0", s(0, 0, 1.0).turned(500).max(1.0), stop(0.8)));
        // 6. the dead zone (the smallest power that moves the robot)
        t.add(lane("creep forward 0.08 / 0.12 / 0.16", 0, s(0.08, 0, 0).max(0.7).stopAt(250), s(0.12, 0, 0).max(0.7).stopAt(250), s(0.16, 0, 0).max(0.7).stopAt(250), stop(0.4)));
        t.add(lane("creep strafe 0.08 / 0.12 / 0.16", -90, s(0.08, 0, 0).max(0.7).stopAt(250), s(0.12, 0, 0).max(0.7).stopAt(250), s(0.16, 0, 0).max(0.7).stopAt(250), stop(0.4)));
        t.add(spot("creep turn 0.05 / 0.08 / 0.12", s(0, 0, 0.05).max(0.7), s(0, 0, 0.08).max(0.7), s(0, 0, 0.12).max(0.7), stop(0.4)));
        // 7. mixed commands (a check on data the fit did not use)
        t.add(random("random A", 11));
        t.add(random("random B", 12));
        return t;
    }

    /** 12 random constant commands of 0.25 s at a spot (along +-0.5, across +-0.3, turn +-0.6); the geofence ends it early if it wanders. */
    private static Test random(String name, long seed) {
        Random r = new Random(seed);
        Seg[] segs = new Seg[13];
        for (int i = 0; i < 12; i++) segs[i] = s(r.nextDouble() - 0.5, 0.6 * (r.nextDouble() - 0.5), 1.2 * (r.nextDouble() - 0.5)).max(0.25);
        segs[12] = stop(0.6);
        Test t = new Test(name, true, 0.0, segs);
        t.spotX = SPOT_X;   // the audience end: the Blue Wall flower is near the other spot
        return t;
    }

    /** What the routine wants this loop. */
    public static final class Output {
        public double forward, left, turn;
        public Phase phase = Phase.PAUSED;
        /** 1-based; 0 before the first. */
        public int test, seg;
        public String status = "";
    }

    private final List<Test> tests;
    private final HiveRoute.Params robot;
    private final Output out = new Output();
    private int index;
    private Phase phase = Phase.MOVE, pausedFrom = Phase.MOVE;
    private double phaseStart, segStart, lastHeading, turnedRad;
    private int segIndex;
    private double laneAngle, targetX, targetY, targetH, runStartX, runStartY, runHeading;
    private double cmdF, cmdL, cmdT;
    private int aborted = 0, finished = 0;
    private String lastFence = "";
    private boolean started = false;

    /** depth, width, origin from the back: the robot's footprint (ChassisProfile). firstTest is 1-based. */
    public DriveTestRoutine(List<Test> tests, int firstTest, double depthIn, double widthIn, double fromBackIn) {
        this.tests = tests;
        this.index = Math.max(0, Math.min(tests.size(), firstTest - 1));
        robot = new HiveRoute.Params();
        robot.depthIn = depthIn;
        robot.widthIn = widthIn;
        robot.fromBackIn = fromBackIn;
        robot.robotHeightIn = 18.0;
    }

    public int count() {
        return tests.size();
    }

    public String testName(int oneBased) {
        return oneBased >= 1 && oneBased <= tests.size() ? tests.get(oneBased - 1).name : "";
    }

    public int aborted() {
        return aborted;
    }

    public int finished() {
        return finished;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * One loop. Pose and field velocity in the BLUE frame (inches, radians, in/s, rad/s); t in seconds (any origin); deadman held; skip pressed this loop.
     * Returns robot-frame powers (forward, left, counterclockwise turn), zero unless the deadman is held.
     */
    public Output update(double t, double x, double y, double h, double vx, double vy, double w, boolean deadman, boolean skip) {
        out.forward = out.left = out.turn = 0.0;
        if (index >= tests.size()) {
            if (phase != Phase.DONE) {
                phase = Phase.DONE;
                RunLog.event(String.format(Locale.US, "DRIVE TEST all done: %d finished, %d stopped by the geofence or skipped", finished, aborted));
            }
            return report(Phase.DONE, "ALL TESTS DONE (" + finished + " finished, " + aborted + " stopped or skipped): stop the opmode");
        }
        Test test = tests.get(index);
        if (!started) {
            started = true;
            beginTest(t, x, y, h);
        }
        if (skip) {
            RunLog.event(String.format(Locale.US, "DRIVE TEST %d skipped (%s) in %s", index + 1, test.name, phase));
            aborted++;
            nextTest(t, x, y, h);
            return report(Phase.PAUSED, "skipped");
        }
        if (!deadman) {
            if (phase != Phase.PAUSED) {
                pausedFrom = phase;
                phase = Phase.PAUSED;
                RunLog.event(String.format(Locale.US, "DRIVE TEST paused (the deadman was released) in test %d, %s", index + 1, pausedFrom));
            }
            return report(Phase.PAUSED, "PAUSED: hold the deadman to go on");
        }
        if (phase == Phase.PAUSED) {
            // a run cut short is useless: start the test again from its start
            RunLog.event(String.format(Locale.US, "DRIVE TEST %d %s (%s)", index + 1, pausedFrom == Phase.MOVE ? "resumed" : "restarted from its start", test.name));
            beginTest(t, x, y, h);
        }
        double speed = Math.hypot(vx, vy);
        String fence = fenceCheck(x, y, h, vx, vy, w, phase == Phase.RUN);
        if (fence != null && phase == Phase.RUN && segIndex < test.segs.size() - 1) {
            lastFence = fence;
            aborted++;
            RunLog.event(String.format(Locale.US, "DRIVE TEST %d GEOFENCE (%s) at (%.1f, %.1f, %.0f deg), %.1f in/s: zero power, test stopped", index + 1, fence, x, y,
                    Math.toDegrees(h), speed));
            segIndex = test.segs.size() - 1;   // the closing zero-power segment
            startSeg(t, h, test);
            cmdF = cmdL = cmdT = 0.0;
        }
        switch (phase) {
            case MOVE:
                move(t, x, y, h, vx, vy, w, test);
                break;
            case REST:
                if (speed > REST_SPEED || Math.abs(w) > Math.toRadians(10.0)) segStart = t;
                if (t - segStart >= REST_SEC || t - phaseStart >= REST_MAX_SEC) startRun(t, x, y, h, test);
                break;
            case RUN:
                run(t, x, y, h, vx, vy, test);
                break;
            default:
                break;
        }
        if (index >= tests.size()) return report(Phase.DONE, "ALL TESTS DONE");
        return report(phase, statusText(test));
    }

    private Output report(Phase p, String status) {
        out.phase = p;
        out.test = Math.min(index + 1, tests.size());
        out.seg = phase == Phase.RUN ? segIndex + 1 : 0;
        out.status = status;
        return out;
    }

    private String statusText(Test test) {
        switch (phase) {
            case MOVE: return "driving to the start of test " + (index + 1) + ": " + test.name;
            case REST: return "settling before test " + (index + 1);
            case RUN: return "RUNNING test " + (index + 1) + ": " + test.name + " (segment " + (segIndex + 1) + " of " + test.segs.size() + ")";
            default: return phase.toString();
        }
    }

    private void beginTest(double t, double x, double y, double h) {
        Test test = tests.get(index);
        // run toward the far end of the lane from where the robot is (so one test ends where the next starts)
        double dir = x > 0 ? -1.0 : 1.0;
        laneAngle = dir > 0 ? 0.0 : Math.PI;
        if (test.spot) {
            targetX = !Double.isNaN(test.spotX) ? test.spotX : x > 0 ? SPOT_X : -SPOT_X;
            targetH = laneAngle;
        } else {
            targetX = -dir * LANE_END_X;
            // a positive offset turns the front toward the alliance wall (+y): counterclockwise when running toward +x, clockwise toward -x
            targetH = Rotation2d.normalize(laneAngle + dir * Math.toRadians(test.headingOffsetDeg));
        }
        targetY = LANE_Y;
        phase = Phase.MOVE;
        phaseStart = t;
        RunLog.event(String.format(Locale.US, "DRIVE TEST %d of %d: %s; start (%.1f, %.1f, %.0f deg)%s", index + 1, tests.size(), test.name, targetX, targetY,
                Math.toDegrees(targetH), test.spot ? "" : dir > 0 ? ", running toward +x" : ", running toward -x"));
    }

    private void nextTest(double t, double x, double y, double h) {
        index++;
        if (index < tests.size()) beginTest(t, x, y, h);
    }

    private void move(double t, double x, double y, double h, double vx, double vy, double w, Test test) {
        double ex = targetX - x, ey = targetY - y, eh = Rotation2d.normalize(targetH - h);
        double dist = Math.hypot(ex, ey);
        boolean there = dist < MOVE_TOL_IN && Math.abs(eh) < Math.toRadians(MOVE_TOL_DEG);
        if (there || t - phaseStart > MOVE_TIMEOUT_SEC) {
            if (!there) RunLog.event(String.format(Locale.US, "DRIVE TEST %d: the start was not reached in %.0f s (%.1f in, %.0f deg off); running from here", index + 1,
                    MOVE_TIMEOUT_SEC, dist, Math.toDegrees(eh)));
            phase = Phase.REST;
            phaseStart = t;
            segStart = t;
            return;
        }
        double fx = MOVE_KP * ex - MOVE_KD * vx, fy = MOVE_KP * ey - MOVE_KD * vy;
        double mag = Math.hypot(fx, fy);
        if (mag > MOVE_MAX_POWER) {
            fx *= MOVE_MAX_POWER / mag;
            fy *= MOVE_MAX_POWER / mag;
        } else if (dist >= MOVE_TOL_IN && mag > 1e-6 && mag < MOVE_KS) {
            fx *= MOVE_KS / mag;
            fy *= MOVE_KS / mag;
        }
        double turn = clamp(TURN_KP * eh - TURN_KD * w, -TURN_MAX, TURN_MAX);
        if (Math.abs(eh) >= Math.toRadians(MOVE_TOL_DEG) && Math.abs(turn) < TURN_KS) turn = Math.copySign(TURN_KS, eh);
        out.forward = fx * Math.cos(h) + fy * Math.sin(h);
        out.left = -fx * Math.sin(h) + fy * Math.cos(h);
        out.turn = turn;
    }

    private void startRun(double t, double x, double y, double h, Test test) {
        phase = Phase.RUN;
        phaseStart = t;
        runStartX = x;
        runStartY = y;
        runHeading = h;
        if (test.spot) laneAngle = h;   // a spot test's commands are the robot's own
        segIndex = 0;
        RunLog.event(String.format(Locale.US, "DRIVE TEST %d RUN from (%.1f, %.1f, %.1f deg)", index + 1, x, y, Math.toDegrees(h)));
        startSeg(t, h, test);
    }

    private void startSeg(double t, double h, Test test) {
        Seg sg = test.segs.get(segIndex);
        double r = runHeading - laneAngle;   // the robot's heading against the travel direction, fixed for the run
        // on the lane, across is toward +y: the left of the travel direction when running toward +x, the right when toward -x
        double left = test.spot ? sg.across : sg.across * Math.cos(laneAngle);
        cmdF = sg.along * Math.cos(r) + left * Math.sin(r);
        cmdL = -sg.along * Math.sin(r) + left * Math.cos(r);
        cmdT = test.spot ? sg.turn : sg.turn * Math.cos(laneAngle);   // on the lane, positive curves toward +y
        segStart = t;
        lastHeading = h;
        turnedRad = 0.0;
        RunLog.event(String.format(Locale.US, "DRIVE TEST %d segment %d of %d: %s -> robot powers forward %+.3f, left %+.3f, turn %+.3f", index + 1, segIndex + 1,
                test.segs.size(), sg.describe(), cmdF, cmdL, cmdT));
    }

    private void run(double t, double x, double y, double h, double vx, double vy, Test test) {
        Seg sg = test.segs.get(segIndex);
        double el = t - segStart;
        turnedRad += Math.abs(Rotation2d.normalize(h - lastHeading));
        lastHeading = h;
        double ca = Math.cos(laneAngle), sa = Math.sin(laneAngle);
        double progress = (x - runStartX) * ca + (y - runStartY) * sa;
        double vAlong = vx * ca + vy * sa;
        double remaining = test.spot ? Double.MAX_VALUE : LANE_END_X - x * ca;   // the lane's far end is at x = LANE_END_X * cos(laneAngle)
        boolean end = el >= sg.maxSec;
        if (el >= sg.minSec) {
            if (sg.stopDecel > 0.0 && !test.spot && (remaining <= STOP_MARGIN_IN || (vAlong > 0 && remaining <= vAlong * vAlong / (2.0 * sg.stopDecel) + STOP_MARGIN_IN))) end = true;
            if (!Double.isNaN(sg.untilProgressIn) && progress >= sg.untilProgressIn) end = true;
            if (!Double.isNaN(sg.untilSpeedBelow) && Math.hypot(vx, vy) < sg.untilSpeedBelow) end = true;
            if (!Double.isNaN(sg.untilTurnedDeg) && turnedRad >= Math.toRadians(sg.untilTurnedDeg)) end = true;
        }
        if (end) {
            segIndex++;
            if (segIndex >= test.segs.size()) {
                finished++;
                RunLog.event(String.format(Locale.US, "DRIVE TEST %d done after %.2f s at (%.1f, %.1f, %.1f deg)", index + 1, t - phaseStart, x, y, Math.toDegrees(h)));
                nextTest(t, x, y, h);
                return;
            }
            startSeg(t, h, test);
        }
        out.forward = cmdF;
        out.left = cmdL;
        out.turn = cmdT;
    }

    /** Why the robot must stop now, or null: its footprint, or the circle at its predicted stop point, is past the fence. */
    public String fenceCheck(double x, double y, double h, double vx, double vy, double w, boolean predict) {
        String why = footprintOutside(HiveRoute.poseRect(x, y, h, robot, 0.0));
        if (why != null) return why;
        double v = Math.hypot(vx, vy);
        if (!predict || v < 5.0) return null;   // nearly still: the footprint is the whole story (the circle at a lane end would trip it)
        double k = v / (2.0 * FENCE_DECEL);
        double px = x + vx * k, py = y + vy * k, rad = robot.radiusIn();
        // the footprint at the predicted stop, or, while the robot turns fast (it may stop at any heading), the square around its far-corner circle
        HiveRoute.Rect at = Math.abs(w) < FENCE_TURNING_RAD_PER_S ? HiveRoute.poseRect(px, py, h, robot, 1.0) : new HiveRoute.Rect(px, py, rad, rad, 0.0, false);
        return footprintOutside(at, "predicted stop: ");
    }

    private String footprintOutside(HiveRoute.Rect r) {
        return footprintOutside(r, "");
    }

    private String footprintOutside(HiveRoute.Rect r, String prefix) {
        double[] k = r.corners();
        for (int i = 0; i < 8; i += 2) {
            if (Math.abs(k[i]) > FENCE_WALL_IN || k[i + 1] > FENCE_WALL_IN) return prefix + "wall";
            if (k[i + 1] < FENCE_CENTER_LINE_IN) return prefix + "center line";
        }
        if (HiveRoute.overlap(r, new HiveRoute.Rect(0.0, 0.0, FENCE_HIVE_X_IN, FENCE_HIVE_Y_IN, 0.0, false))) return prefix + "hive";
        for (HiveRoute.Rect f : HiveRoute.flowerKeepOuts(FENCE_FLOWER_GROW_IN)) if (HiveRoute.overlap(r, f)) return prefix + "flower";
        return null;
    }

    public String lastFence() {
        return lastFence;
    }
}
