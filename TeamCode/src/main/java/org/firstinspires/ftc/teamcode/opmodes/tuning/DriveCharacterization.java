package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.robot.Footprint;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * Drive Characterization (specs/support-opmodes.md section 4.3): measure the drive itself (speed, acceleration,
 * braking, coasting, turning, dead zones) to build the simulation model behind the grid routes. The robot drives 34
 * open-loop tests by itself on one alliance's half, along a lane parallel to the wall, each run starting where the
 * last ended. A log row is written on every loop; PC tools fit the drive model from it. No mechanism moves.
 *
 * Controls (gamepad 1). At INIT, D-pad right / left chooses the blue / red half (nothing moves until one is chosen)
 * and up / down the first test. The driver must hold the right bumper the whole time; letting go zeroes the power at
 * once, and holding again restarts the current test. X skips a test.
 *
 * The area (blue frame; a red robot's pose is rotated first): a lane along x at y = 46, from x = -54 to +54. A run
 * starts at the end the robot is nearer and drives to the other. Spot tests are at x = +40 or -40 on the lane.
 * On the lane a segment's command is (along the lane, across it toward the alliance wall, turn toward the alliance
 * wall); at a spot it is the robot's own (forward, left, turn). It is turned into robot-frame powers once, when the
 * segment starts.
 *
 * The robot is seeded at the TeleOp's default start of the chosen half. At a spot it faces along the lane. Before a
 * run it must be still: slower than 2 in/s and 10 deg/s for 0.25 s, with the wait at most 1.5 s. A geofence stop ends
 * the test and the next one begins. The phase column is one of MOVE, REST, RUN, PAUSED, DONE.
 *
 * SPEC GAP: after a test's last segment, and after a geofence stop, this opmode waits at zero power until the robot
 * is still before it goes on (logged as REST); the page does not say what happens between two tests.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Config
@Disabled
@TeleOp(name = "Drive Characterization", group = "3 Tuning")
public final class DriveCharacterization extends TeamOpMode {
    public static double LANE_Y = 46.0, LANE_END_X = 54.0, SPOT_X = 40.0, LANE_MARGIN_IN = 6.0;
    public static double GOTO_POWER = 0.6, GOTO_TOL_IN = 2.0, GOTO_TOL_DEG = 2.0;
    public static double STILL_IN_S = 2.0, STILL_DEG_S = 10.0, STILL_FOR_S = 0.25, STILL_MAX_S = 1.5;
    public static double FENCE_WALL_IN = 68.0, FENCE_CENTER_Y = 4.0, FENCE_HIVE_X = 22.5, FENCE_HIVE_Y = 28.0, FENCE_FLOWER_IN = 3.0, FENCE_DECEL = 200.0;
    public static double LANE_END_DECEL = 250.0, SEGMENT_S = 3.0;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.DRIVETEST; }
    @Override protected String title() { return "Drive Characterization"; }

    // ---- The tests ----

    private enum End { TIME, STOP_NEEDS, SLOWER_THAN, DISTANCE, TURNED }

    /** One constant command, with what ends it. */
    private static final class Seg {
        double along, across, turn;
        End end = End.TIME;
        double arg, minS, maxS = 3.0;
        boolean alsoLaneEnd;
        String text;

        Seg(double along, double across, double turn) {
            this.along = along;
            this.across = across;
            this.turn = turn;
        }

        boolean moving() { return along != 0 || across != 0 || turn != 0; }
    }

    private static final class Test {
        final String name;
        /** Lane tests: the robot's heading against the direction of travel, degrees. */
        final double headingDeg;
        final boolean spot;
        /** Spot tests: the spot's x, or NaN for whichever of +40 and -40 is nearer. */
        final double spotX;
        final Seg[] segs;

        Test(String name, double headingDeg, boolean spot, double spotX, Seg... segs) {
            this.name = name;
            this.headingDeg = headingDeg;
            this.spot = spot;
            this.spotX = spotX;
            this.segs = segs;
        }
    }

    private static Seg untilStopNeeds(double power, double decel) {
        Seg s = new Seg(power, 0, 0);
        s.end = End.STOP_NEEDS;
        s.arg = decel;
        s.maxS = SEGMENT_S;
        s.text = String.format(Locale.US, "%.2f until the stop needs %.0f in/s2", power, decel);
        return s;
    }

    private static Seg untilSlower(double power, double speed, double minS, double maxS) {
        Seg s = new Seg(power, 0, 0);
        s.end = End.SLOWER_THAN;
        s.arg = speed;
        s.minS = minS;
        s.maxS = maxS;
        s.text = String.format(Locale.US, "%.2f until slower than %.0f in/s (%.2f to %.1f s)", power, speed, minS, maxS);
        return s;
    }

    private static Seg forTime(double along, double across, double turn, double seconds) {
        Seg s = new Seg(along, across, turn);
        s.maxS = seconds;
        s.text = String.format(Locale.US, "(%.2f, %.2f, %.2f) for %.2f s", along, across, turn, seconds);
        return s;
    }

    private static Seg stop(double seconds) {
        Seg s = forTime(0, 0, 0, seconds);
        s.text = String.format(Locale.US, "stop %.1f s", seconds);
        return s;
    }

    private static Seg forDistance(double power, double inches) {
        Seg s = new Seg(power, 0, 0);
        s.end = End.DISTANCE;
        s.arg = inches;
        s.maxS = SEGMENT_S;
        s.alsoLaneEnd = true;
        s.text = String.format(Locale.US, "%.2f for %.0f in", power, inches);
        return s;
    }

    /** A moving segment that also ends at the lane's end. */
    private static Seg laneEnd(Seg s) {
        s.alsoLaneEnd = true;
        return s;
    }

    private static Seg step(double power, double atMostS) {
        Seg s = laneEnd(forTime(power, 0, 0, atMostS));
        s.text = String.format(Locale.US, "%.2f for at most %.1f s", power, atMostS);
        return s;
    }

    private static Seg spin(double turn, double deg, double atMostS) {
        Seg s = new Seg(0, 0, turn);
        s.end = End.TURNED;
        s.arg = deg;
        s.maxS = atMostS;
        s.text = String.format(Locale.US, "turn %.2f until turned %.0f deg, at most %.1f s", turn, deg, atMostS);
        return s;
    }

    private static Test lane(String name, double headingDeg, Seg... segs) { return new Test(name, headingDeg, false, Double.NaN, segs); }
    private static Test spot(String name, double spotX, Seg... segs) { return new Test(name, 0, true, spotX, segs); }

    private static Seg[] brake(double power) { return new Seg[] {untilStopNeeds(power, 250), stop(0.8)}; }
    private static Seg[] coast(double power) { return new Seg[] {untilStopNeeds(power, 80), untilSlower(0.2, 12, 0.1, 2.0), stop(0.6)}; }
    private static Seg[] roll(double along, double across, double turn, double seconds) {
        return new Seg[] {forDistance(0.85, 80), laneEnd(forTime(along, across, turn, seconds)), stop(0.8)};
    }
    private static Seg[] random(long seed) {
        Random r = new Random(seed);
        Seg[] s = new Seg[13];
        for (int i = 0; i < 12; i++) s[i] = forTime(r.nextDouble() - 0.5, 0.6 * (r.nextDouble() - 0.5), 1.2 * (r.nextDouble() - 0.5), 0.25);
        s[12] = stop(0.6);
        return s;
    }

    /** The 34 tests, most important first. */
    private static Test[] tests() {
        List<Test> t = new ArrayList<>();
        t.add(lane("stop: brake from 0.85, front first", 0, brake(0.85)));
        t.add(lane("stop: brake from 0.85, rear first", 180, brake(0.85)));
        t.add(lane("stop: coast on 0.2 from 0.85", 0, coast(0.85)));
        t.add(lane("stop: coast on 0.2 from 0.85, rear first", 180, coast(0.85)));
        t.add(lane("stop: drop from 0.85 to 0.45", 0, untilStopNeeds(0.85, 55), forTime(0.45, 0, 0, 0.6), stop(0.6)));
        t.add(lane("stop: reverse 0.4 from 0.85", 0, untilStopNeeds(0.85, 160), untilSlower(-0.4, 10, 0.05, 0.6), stop(0.6)));
        t.add(lane("stop: brake from 0.85, strafing", -90, brake(0.85)));
        t.add(lane("stop: coast on 0.2 from 0.85, strafing", -90, coast(0.85)));
        t.add(lane("stop: brake from 0.6", 0, brake(0.6)));
        t.add(lane("stop: coast on 0.2 from 0.6", 180, coast(0.6)));
        t.add(lane("roll at 0.85 + turn 0.5 for 0.25 s", 0, roll(0.85, 0, -0.5, 0.25)));
        t.add(lane("roll at 0.85 + turn 0.5 for 0.25 s, rear first", 180, roll(0.85, 0, -0.5, 0.25)));
        t.add(lane("roll at 0.85 + turn 0.3 for 0.3 s", 0, roll(0.85, 0, -0.3, 0.3)));
        t.add(lane("roll at 0.85, then zero power + turn 0.5", 0, roll(0, 0, -0.5, 0.4)));
        t.add(lane("roll at 0.85, then 50 deg toward the center line", 0, roll(0.55, -0.65, 0, 0.3)));
        t.add(lane("roll at 0.85, then 50 deg toward the center line, rear first", 180, roll(0.55, -0.65, 0, 0.3)));
        t.add(lane("roll at 0.85, then a dash corner", 0, roll(0.2, -0.3, -0.5, 0.3)));
        t.add(lane("step forward 0.3", 0, step(0.3, 3.0), stop(0.6)));
        t.add(lane("step forward 0.5", 180, step(0.5, 3.0), stop(0.6)));
        t.add(lane("step forward 0.7", 0, step(0.7, 3.0), stop(0.6)));
        t.add(lane("step forward 1.0", 0, step(1.0, 3.0), stop(0.8)));
        t.add(lane("step strafe 0.4", -90, step(0.4, 3.0), stop(0.6)));
        t.add(lane("step strafe 0.7", 90, step(0.7, 3.0), stop(0.6)));
        t.add(lane("step strafe 1.0", -90, step(1.0, 3.0), stop(0.8)));
        t.add(lane("step diagonal 0.7", -45, step(0.7, 0.8), stop(0.6)));
        t.add(spot("spin 0.3", Double.NaN, spin(0.3, 300, 2.0), stop(0.6)));
        t.add(spot("spin 0.5", Double.NaN, spin(0.5, 400, 1.5), stop(0.6)));
        t.add(spot("spin -0.7", Double.NaN, spin(-0.7, 450, 1.2), stop(0.8)));
        t.add(spot("spin 1.0", Double.NaN, spin(1.0, 500, 1.0), stop(0.8)));
        t.add(lane("creep forward 0.08 / 0.12 / 0.16", 0, laneEnd(forTime(0.08, 0, 0, 0.7)), laneEnd(forTime(0.12, 0, 0, 0.7)), laneEnd(forTime(0.16, 0, 0, 0.7)), stop(0.4)));
        t.add(lane("creep strafe 0.08 / 0.12 / 0.16", -90, laneEnd(forTime(0.08, 0, 0, 0.7)), laneEnd(forTime(0.12, 0, 0, 0.7)), laneEnd(forTime(0.16, 0, 0, 0.7)), stop(0.4)));
        t.add(spot("creep turn 0.05 / 0.08 / 0.12", Double.NaN, forTime(0, 0, 0.05, 0.7), forTime(0, 0, 0.08, 0.7), forTime(0, 0, 0.12, 0.7), stop(0.4)));
        t.add(spot("random A", SPOT_X, random(11)));
        t.add(spot("random B", SPOT_X, random(12)));
        return t.toArray(new Test[0]);
    }

    // ---- The run ----

    private enum Phase { IDLE, TO_START, SETTLE, RUN, AFTER, DONE }

    private Alliance alliance;
    private Footprint footprint;
    private Phase phase = Phase.IDLE;

    /** The phase column of the log: MOVE, REST, RUN, PAUSED or DONE. */
    private String phaseText() {
        switch (phase) {
            case TO_START: return "MOVE";
            case SETTLE:
            case AFTER: return "REST";
            case RUN: return "RUN";
            case DONE: return "DONE";
            default: return "PAUSED";
        }
    }
    private int testIndex, segIndex;
    // Read once per loop, for the log row.
    private Pose pose = new Pose(0, 0, 0);
    private double vx, vy, omega, vbat;
    private final double[] amps = new double[4];
    private int ampsNext;

    /** A field pose or vector in the blue frame. */
    private Pose blue(Pose p) { return alliance == Alliance.RED ? p.rotated180() : p; }

    /** Why the footprint at a blue-frame pose is outside the test area, or null. */
    private String outside(double x, double y, double h) {
        double[][] c = footprint.corners(x, y, h);
        for (int k = 0; k <= 4; k++) {
            double px = k == 4 ? x : c[k][0], py = k == 4 ? y : c[k][1];
            if (Math.abs(px) > FENCE_WALL_IN || py > FENCE_WALL_IN) return "a wall";
            if (py < FENCE_CENTER_Y) return "the center line";
            if (Math.abs(px) < FENCE_HIVE_X && py < FENCE_HIVE_Y) return "the hive";
            for (int i = 0; i < 4; i++) {
                double[] r = Field.flowerRect(i);
                if (px > r[0] - FENCE_FLOWER_IN && px < r[2] + FENCE_FLOWER_IN && py > r[1] - FENCE_FLOWER_IN && py < r[3] + FENCE_FLOWER_IN) return "a flower stand";
            }
        }
        return null;
    }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        footprint = profile.footprint();
        final Test[] tests = tests();
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, Field.startRear(profile.originToBack));
        Edge right = new Edge(), left = new Edge(), up = new Edge(), down = new Edge(), skip = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            boolean r = right.pressed(gamepad1.dpad_right), l = left.pressed(gamepad1.dpad_left);
            if (r || l) {
                alliance = r ? Alliance.BLUE : Alliance.RED;
                drive.localizer.seed(alliance.pose(Field.startRear(profile.originToBack)));
            }
            if (up.pressed(gamepad1.dpad_up)) testIndex = Math.min(tests.length - 1, testIndex + 1);
            if (down.pressed(gamepad1.dpad_down)) testIndex = Math.max(0, testIndex - 1);
            telemetry.addLine("Drive Characterization: 34 open-loop tests on one half. NO MECHANISM MOVES; the robot drives itself.");
            telemetry.addLine(alliance == null ? "CHOOSE THE HALF: D-pad right = blue, left = red. Nothing moves until then."
                    : alliance + " half. Robot at its default start, back on the wall, still.");
            telemetry.addLine("First test (D-pad up/down): " + (testIndex + 1) + " " + tests[testIndex].name);
            telemetry.addLine("Pose " + drive.pose() + (drive.localizer.seedConfirmed() ? "" : " SEED NOT CONFIRMED"));
            telemetry.addLine("After Start: HOLD the right bumper to drive; X skips a test.");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        RunLog.sampler("x,y,hdg_deg,vx,vy,w_dps,cmd_fwd,cmd_left,cmd_turn,p_lf,p_lb,p_rb,p_rf,vbat,a_lf,a_lb,a_rb,a_rf,test,seg,phase", 0, new RunLog.Sampler() {
            @Override public String row() {
                return String.format(Locale.US, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%s",
                        pose.x, pose.y, pose.headingDeg(), vx, vy, Math.toDegrees(omega), drive.cmdForward(), drive.cmdLeft(), drive.cmdTurn(),
                        drive.wheelPower(0), drive.wheelPower(1), drive.wheelPower(2), drive.wheelPower(3), vbat, amps[0], amps[1], amps[2], amps[3],
                        testIndex + 1, phase == Phase.RUN ? segIndex + 1 : 0, phaseText());
            }
        });
        RunLog.burst(true);     // a row on every loop
        if (alliance != null) RunLog.eventf("SETUP profile %s, alliance %s, start pose default %s, driver forward %.0f deg; Drive Characterization, first test %d",
                profile.name, alliance, drive.pose(), Math.toDegrees(alliance.driverForwardRad()), testIndex + 1);

        DriveToPose leg = null;
        Pose startBlue = null;
        int dir = 1;                // lane tests: +1 travels toward +x, -1 toward -x
        double stillSince = -1, segStart = 0, segTurned = 0, lastH = 0, afterSince = 0, settleAt = 0;
        double fwd = 0, lft = 0, trn = 0;
        String last = "";
        while (opModeIsActive()) {
            double t = Clock.now();
            drive.updatePose();
            pose = drive.pose();
            vx = drive.localizer.vx();
            vy = drive.localizer.vy();
            omega = drive.localizer.omega();
            vbat = battery();                               // the battery every loop
            amps[ampsNext] = drive.readAmps(ampsNext);      // one motor current per loop, in turn
            ampsNext = (ampsNext + 1) % 4;

            if (alliance == null) {
                // Nothing moves until a half is chosen. The robot must be on its start, still.
                boolean r = right.pressed(gamepad1.dpad_right), l = left.pressed(gamepad1.dpad_left);
                if (r || l) {
                    alliance = r ? Alliance.BLUE : Alliance.RED;
                    drive.localizer.seed(alliance.pose(Field.startRear(profile.originToBack)));
                    RunLog.eventf("SETUP profile %s, alliance %s, start pose default %s, driver forward %.0f deg; Drive Characterization, first test %d",
                            profile.name, alliance, drive.pose(), Math.toDegrees(alliance.driverForwardRad()), testIndex + 1);
                }
                drive.stop();
                RunLog.tick();
                telemetry.addLine("CHOOSE THE HALF: D-pad right = blue, left = red. Nothing moves until then.");
                telemetry.update();
                continue;
            }
            Pose pb = blue(pose);
            double bvx = alliance == Alliance.RED ? -vx : vx, bvy = alliance == Alliance.RED ? -vy : vy;
            boolean hold = gamepad1.right_bumper;
            Test test = testIndex < tests.length ? tests[testIndex] : null;

            if (test != null && skip.pressed(gamepad1.x)) {
                RunLog.eventf("DRIVE TEST %d SKIPPED by X: %s", testIndex + 1, test.name);
                last = "test " + (testIndex + 1) + " skipped";
                testIndex++;
                phase = Phase.IDLE;
                leg = null;
                drive.stop();
            } else if (test == null) {
                if (phase != Phase.DONE) RunLog.event("DRIVE TEST all tests done");
                phase = Phase.DONE;
                drive.stop();
            } else if (!hold) {
                // Letting go zeroes the power at once. Holding again restarts the current test.
                if (phase != Phase.IDLE) RunLog.eventf("DRIVE TEST %d released in phase %s: the test restarts when the bumper is held again", testIndex + 1, phase);
                phase = Phase.IDLE;
                leg = null;
                drive.stop();
            } else {
                switch (phase) {
                    case IDLE: {
                        // Where this test starts, from where the robot is now.
                        if (test.spot) {
                            double sx = Double.isNaN(test.spotX) ? (pb.x >= 0 ? SPOT_X : -SPOT_X) : test.spotX;
                            startBlue = Pose.deg(sx, LANE_Y, 0);
                        } else {
                            dir = pb.x <= 0 ? 1 : -1;       // start at the end the robot is nearer, drive to the other
                            startBlue = Pose.deg(-dir * LANE_END_X, LANE_Y, Angle.wrapDeg((dir > 0 ? 0 : 180) + test.headingDeg));
                        }
                        RunLog.eventf("DRIVE TEST %d of %d: %s; start %s (blue frame), %s, battery %.2f V", testIndex + 1, tests.length, test.name, startBlue,
                                test.spot ? "spot" : "lane toward " + (dir > 0 ? "+x" : "-x"), vbat);
                        leg = new DriveToPose(drive, "to the start of test " + (testIndex + 1), alliance.pose(startBlue))
                                .maxPower(GOTO_POWER).tolerance(GOTO_TOL_IN, GOTO_TOL_DEG).distanceTimeout();
                        phase = Phase.TO_START;
                        break;
                    }
                    case TO_START:
                        if (!leg.run(null)) {
                            boolean there = leg.settled();
                            leg = null;
                            drive.stop();
                            stillSince = -1;
                            // A leg that timed out short of the start is driven again: a test must not run from the wrong place.
                            phase = there ? Phase.SETTLE : Phase.IDLE;
                            settleAt = t;
                        }
                        break;
                    case SETTLE: {
                        drive.stop();
                        boolean still = Math.hypot(vx, vy) < STILL_IN_S && Math.abs(Math.toDegrees(omega)) < STILL_DEG_S;
                        if (!still) stillSince = -1;
                        else if (stillSince < 0) stillSince = t;
                        // Still for long enough, or the wait has reached its limit.
                        if ((stillSince >= 0 && t - stillSince >= STILL_FOR_S) || t - settleAt >= STILL_MAX_S) {
                            segIndex = -1;
                            phase = Phase.RUN;
                        }
                        break;
                    }
                    case RUN: {
                        Seg seg = segIndex >= 0 ? test.segs[segIndex] : null;
                        // Along the lane, in the direction of travel.
                        double vAlong = Math.max(0, bvx * dir), laneLeft = (dir * LANE_END_X - pb.x) * dir - LANE_MARGIN_IN;
                        boolean ended = seg == null;
                        if (seg != null) {
                            double el = t - segStart;
                            segTurned += Math.toDegrees(Angle.diff(pose.h, lastH));
                            lastH = pose.h;
                            switch (seg.end) {
                                case STOP_NEEDS: ended = laneLeft <= vAlong * vAlong / (2 * seg.arg); break;
                                case SLOWER_THAN: ended = el >= seg.minS && Math.hypot(vx, vy) < seg.arg; break;
                                case DISTANCE: ended = (pb.x - startBlue.x) * dir >= seg.arg; break;
                                case TURNED: ended = Math.abs(segTurned) >= seg.arg; break;
                                default: break;
                            }
                            if (el >= seg.maxS) ended = true;
                            if (!test.spot && seg.alsoLaneEnd && laneLeft <= vAlong * vAlong / (2 * LANE_END_DECEL)) ended = true;
                        }
                        if (ended) {
                            segIndex++;
                            if (segIndex >= test.segs.length) {
                                drive.stop();
                                RunLog.eventf("DRIVE TEST %d done: ended at %s (blue frame)", testIndex + 1, pb);
                                last = "test " + (testIndex + 1) + " done";
                                afterSince = t;
                                phase = Phase.AFTER;
                                break;
                            }
                            seg = test.segs[segIndex];
                            segStart = t;
                            segTurned = 0;
                            lastH = pose.h;
                            // The command is turned into robot-frame powers once, when the segment starts.
                            if (test.spot) {
                                fwd = seg.along;
                                lft = seg.across;
                                trn = seg.turn;
                            } else {
                                double fx = seg.along * dir, fy = seg.across;       // across is toward the alliance wall, +y in the blue frame
                                fwd = fx * Math.cos(pb.h) + fy * Math.sin(pb.h);
                                lft = -fx * Math.sin(pb.h) + fy * Math.cos(pb.h);
                                trn = seg.turn * dir;                              // toward the alliance wall
                            }
                            RunLog.eventf("DRIVE TEST %d segment %d of %d: %s -> robot powers forward %.3f, left %.3f, turn %.3f", testIndex + 1, segIndex + 1,
                                    test.segs.length, seg.text, fwd, lft, trn);
                        }
                        // The geofence: the footprint, or the robot where it would stop, must stay inside the area.
                        double speed = Math.hypot(bvx, bvy), stopDist = speed * speed / (2 * FENCE_DECEL);
                        String why = outside(pb.x, pb.y, pb.h);
                        if (why == null && speed > 1e-3) why = outside(pb.x + bvx / speed * stopDist, pb.y + bvy / speed * stopDist, pb.h);
                        if (why != null && seg.moving()) {
                            drive.stop();
                            RunLog.eventf("DRIVE TEST %d GEOFENCE (%s) at %s (blue frame), %.0f in/s, in segment %d", testIndex + 1, why, pb, speed, segIndex + 1);
                            last = "test " + (testIndex + 1) + " GEOFENCE: " + why;
                            afterSince = t;
                            phase = Phase.AFTER;
                            break;
                        }
                        drive.drive(fwd, lft, trn);
                        break;
                    }
                    case AFTER:
                        // Zero power until the robot is still, then the next test.
                        drive.stop();
                        if (t - afterSince >= STILL_FOR_S && Math.hypot(vx, vy) < STILL_IN_S && Math.abs(Math.toDegrees(omega)) < STILL_DEG_S) {
                            testIndex++;
                            phase = Phase.IDLE;
                        }
                        break;
                    default:
                        drive.stop();
                }
            }
            RunLog.tick();
            telemetry.addLine(test == null ? "ALL TESTS DONE" : "Test " + (testIndex + 1) + " of " + tests.length + ": " + test.name);
            telemetry.addLine(hold ? "phase " + phase + (phase == Phase.RUN ? ", segment " + (segIndex + 1) : "") : "HOLD the right bumper to drive. X skips this test.");
            telemetry.addLine(String.format(Locale.US, "%s pose %s, %.0f in/s, battery %.2f V", alliance, pose, Math.hypot(vx, vy), vbat));
            telemetry.addLine(last);
            telemetry.update();
        }
    }
}
