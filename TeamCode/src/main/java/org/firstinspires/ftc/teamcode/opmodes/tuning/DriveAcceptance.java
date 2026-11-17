package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Drive Acceptance (specs/support-opmodes.md section 4.2.1, target T2): drives the acceptance test of drive to pose
 * and reports every number of it, so a tuning session ends with a verdict and a record. The tuners find the gains;
 * this opmode judges them.
 *
 * Any clear floor: the start is (0, 0, 0 deg) wherever the robot stands at INIT. It needs 48 in clear ahead and to its
 * left, with a margin, and room to spin. No mechanism moves. The robot drives only while the right bumper is held.
 */
@Config
@TeleOp(name = "Drive Acceptance", group = "3 Tuning")
public final class DriveAcceptance extends TeamOpMode {
    public static int RUNS_PER_TEST = 10;
    public static double LONG_MOVE_IN = 48.0, SHORT_MOVE_IN = 12.0, SMALL_TURN_DEG = 5.0;
    public static double FINAL_ERROR_IN = 1.0, FINAL_ERROR_DEG = 2.0;
    public static double OVERSHOOT_IN = 1.5, OVERSHOOT_DEG = 5.0;
    public static double TRAVEL_SPEED_IN_S = 22.0, STRAFE_RATIO = 0.78, TURN_RATE_DEG_S = 120.0, TIME_MARGIN_S = 1.0;
    public static double SHORT_TEST_ALLOWED_S = 1.5;
    public static double BATTERY_AT_REST_V = 12.0;
    public static double REST_S = 0.6;
    public static double FENCE_IN = 15.0;
    public static double UNMEASURED_CAP_S = 6.0;

    private static final class Test {
        final String name;
        final Pose far;
        final double allowed;
        String verdict = "not run";
        final List<String> reasons = new ArrayList<>();
        int settled, timedOut, counted, notCounted, voided;
        double worstErrIn, worstErrDeg, sumErrIn, worstOverIn, worstOverDeg, worstTime, lowestBattery = 99;

        Test(String name, Pose far, double allowed) {
            this.name = name;
            this.far = far;
            this.allowed = allowed;
        }
    }

    private enum Phase { REST, PRE_POSITION, RUN, RETURN_AFTER_FENCE }

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Drive Acceptance"; }

    private int testIndex, run;
    private Phase phase = Phase.REST;

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
        Test[] tests = {
                new Test("48 in forward", Pose.deg(LONG_MOVE_IN, 0, 0), LONG_MOVE_IN / TRAVEL_SPEED_IN_S + TIME_MARGIN_S),
                new Test("48 in strafe left", Pose.deg(0, LONG_MOVE_IN, 0), LONG_MOVE_IN / (TRAVEL_SPEED_IN_S * STRAFE_RATIO) + TIME_MARGIN_S),
                new Test("turn 90 deg", Pose.deg(0, 0, 90), 90 / TURN_RATE_DEG_S + TIME_MARGIN_S),
                new Test("turn 180 deg", Pose.deg(0, 0, 180), 180 / TURN_RATE_DEG_S + TIME_MARGIN_S),
                new Test("12 in forward (the stall case)", Pose.deg(SHORT_MOVE_IN, 0, 0), SHORT_TEST_ALLOWED_S),
                new Test("turn 5 deg (the stall case)", Pose.deg(0, 0, SMALL_TURN_DEG), SHORT_TEST_ALLOWED_S),
        };
        Pose home = new Pose(0, 0, 0);
        Edge up = new Edge(), down = new Edge(), skip = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            if (up.pressed(gamepad1.dpad_up)) testIndex = Math.min(tests.length - 1, testIndex + 1);
            if (down.pressed(gamepad1.dpad_down)) testIndex = Math.max(0, testIndex - 1);
            telemetry.addLine("Drive Acceptance. The robot must stand still until Start.");
            telemetry.addLine("Needs 48 in clear ahead and to the left, and room to spin.");
            telemetry.addLine("First test (D-pad up/down): " + (testIndex + 1) + " " + tests[testIndex].name);
            telemetry.addLine("Pinpoint: " + drive.localizer.statusText() + (drive.localizer.seedConfirmed() ? ", seeded" : ", SEED NOT CONFIRMED"));
            telemetry.addLine("At run: HOLD right bumper to drive. X skips a test.");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        RunLog.event("ACCEPT GAINS at start: " + DriveToPose.gainsLine());
        RunLog.sampler("x,y,heading,cmd_f,cmd_l,cmd_t,test,run,phase", 0, new RunLog.Sampler() {
            @Override public String row() {
                Pose p = drive.pose();
                return String.format(Locale.US, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%s", p.x, p.y, p.headingDeg(),
                        drive.cmdForward(), drive.cmdLeft(), drive.cmdTurn(), testIndex + 1, run + 1, phase);
            }
        });
        RunLog.burst(true);      // a row on every loop

        DriveToPose leg = null;
        double phaseStart = Clock.now(), restBattery = Double.NaN;
        String last = "";
        boolean fenceFail = false;
        while (opModeIsActive() && testIndex < tests.length) {
            double t = Clock.now();
            drive.updatePose();
            Pose p = drive.pose();
            Test test = tests[testIndex];
            boolean go = gamepad1.right_bumper;
            Pose from = run % 2 == 0 ? home : test.far, to = run % 2 == 0 ? test.far : home;

            if (skip.pressed(gamepad1.x)) {
                drive.stop();
                test.verdict = "SKIPPED";
                RunLog.eventf("ACCEPT TEST %d %s: SKIPPED by the operator after %d run(s)", testIndex + 1, test.name, run);
                leg = null;
                testIndex++;
                run = 0;
                phase = Phase.REST;
                phaseStart = t;
                continue;
            }
            if (!go) {
                // Letting go stops it at once; the run in progress is void and is driven again.
                if (phase == Phase.RUN && leg != null) {
                    test.voided++;
                    RunLog.eventf("ACCEPT run %d.%d void: the bumper was released", testIndex + 1, run + 1);
                }
                drive.stop();
                leg = null;
                phase = phase == Phase.RETURN_AFTER_FENCE ? phase : Phase.REST;
                phaseStart = t;
            } else {
                switch (phase) {
                    case REST:
                        drive.stop();
                        if (t - phaseStart >= REST_S) {
                            restBattery = battery();        // read at rest
                            boolean atStart = p.distTo(from) <= DriveToPose.POSITION_TOL_IN
                                    && Math.abs(Math.toDegrees(Angle.diff(from.h, p.h))) <= DriveToPose.HEADING_TOL_DEG;
                            if (atStart) {
                                leg = measured(test, to);
                                phase = Phase.RUN;
                            } else {
                                // An unmeasured leg takes it to the run's start, so every run starts from the same place.
                                leg = new DriveToPose(drive, "to the start of the run", from).stiction(true).timeout(UNMEASURED_CAP_S);
                                phase = Phase.PRE_POSITION;
                            }
                        }
                        break;
                    case PRE_POSITION:
                        if (!leg.run(null)) {
                            leg = null;
                            phase = Phase.REST;
                            phaseStart = t;
                        }
                        break;
                    case RUN:
                        // Fence: a robot far outside the rectangle between the test's two poses ends the test as a FAIL.
                        double minX = Math.min(0, test.far.x) - FENCE_IN, maxX = Math.max(0, test.far.x) + FENCE_IN;
                        double minY = Math.min(0, test.far.y) - FENCE_IN, maxY = Math.max(0, test.far.y) + FENCE_IN;
                        if (p.x < minX || p.x > maxX || p.y < minY || p.y > maxY) {
                            drive.stop();
                            fenceFail = true;
                            RunLog.eventf("ACCEPT run %d.%d FENCE at %s: the test ends as a FAIL", testIndex + 1, run + 1, p);
                            leg = new DriveToPose(drive, "back to the start after the fence", home).stiction(true).timeout(UNMEASURED_CAP_S);
                            phase = Phase.RETURN_AFTER_FENCE;
                            break;
                        }
                        if (!leg.run(null)) {
                            boolean counted = restBattery >= BATTERY_AT_REST_V;
                            last = String.format(Locale.US, "%s in %.2f s (limit %.2f), error %.2f in %.1f deg, overshoot %.2f in %.1f deg, battery at rest %.2f V%s",
                                    leg.settled() ? "settled" : "TIMED OUT", leg.seconds(), test.allowed, leg.finalErrorIn(), leg.finalErrorDeg(),
                                    leg.overshootIn(), leg.overshootDeg(), restBattery, counted ? "" : " (NOT COUNTED: battery)");
                            RunLog.eventf("ACCEPT run %d.%d %s", testIndex + 1, run + 1, last);
                            if (counted) {
                                test.counted++;
                                if (leg.settled()) test.settled++;
                                else test.timedOut++;
                                test.worstErrIn = Math.max(test.worstErrIn, leg.finalErrorIn());
                                test.worstErrDeg = Math.max(test.worstErrDeg, Math.abs(leg.finalErrorDeg()));
                                test.sumErrIn += leg.finalErrorIn();
                                test.worstOverIn = Math.max(test.worstOverIn, leg.overshootIn());
                                test.worstOverDeg = Math.max(test.worstOverDeg, leg.overshootDeg());
                                test.worstTime = Math.max(test.worstTime, leg.seconds());
                                test.lowestBattery = Math.min(test.lowestBattery, restBattery);
                            } else {
                                test.notCounted++;
                            }
                            leg = null;
                            run++;
                            phase = Phase.REST;
                            phaseStart = t;
                        }
                        break;
                    default:
                        if (!leg.run(null)) leg = null;
                }
            }
            boolean testOver = (run >= RUNS_PER_TEST && phase == Phase.REST) || (fenceFail && leg == null && phase == Phase.RETURN_AFTER_FENCE);
            if (testOver) {
                judge(test, fenceFail, testIndex + 1);
                fenceFail = false;
                testIndex++;
                run = 0;
                phase = Phase.REST;
                phaseStart = t;
            }
            RunLog.tick();
            telemetry.addLine(go ? "DRIVING (release the right bumper to stop)" : "HOLD right bumper to drive");
            if (testIndex < tests.length) telemetry.addLine(String.format(Locale.US, "Test %d %s, run %d of %d, %s", testIndex + 1, tests[testIndex].name, run + 1, RUNS_PER_TEST, phase));
            telemetry.addLine("Last run: " + last);
            for (int i = 0; i < tests.length; i++) telemetry.addLine((i + 1) + " " + tests[i].name + ": " + tests[i].verdict);
            telemetry.update();
        }
        drive.stop();
        int pass = 0;
        StringBuilder notPass = new StringBuilder();
        boolean failed = false;
        for (int i = 0; i < tests.length; i++) {
            if (tests[i].verdict.equals("PASS")) pass++;
            else notPass.append(" ").append(i + 1).append(":").append(tests[i].verdict);
            failed |= tests[i].verdict.equals("FAIL");
        }
        RunLog.eventf("ACCEPT RESULT %s%s", pass == tests.length ? "ALL PASS" : failed ? "FAILED" : "INCOMPLETE",
                notPass.length() == 0 ? "" : "; not a pass:" + notPass);
        RunLog.eventf("ACCEPT GAINS %s (hub date %s)", DriveToPose.gainsLine(), new SimpleDateFormat("yyyy-MM-dd", Locale.US).format(new Date()));
        while (opModeIsActive()) {
            telemetry.addLine(pass == tests.length ? "ALL PASS" : failed ? "FAILED" : "INCOMPLETE");
            for (int i = 0; i < tests.length; i++) telemetry.addLine((i + 1) + " " + tests[i].name + ": " + tests[i].verdict);
            telemetry.update();
            idle();
        }
    }

    /** A measured run: one leg with the static friction term on. Its cap is long enough to tell a slow leg from one that never settles. */
    private DriveToPose measured(Test test, Pose to) {
        return new DriveToPose(drive, "accept " + test.name, to).stiction(true).timeout(Math.max(DriveToPose.TIME_CAP_S, test.allowed + 1.0));
    }

    private void judge(Test t, boolean fence, int number) {
        if (fence) t.reasons.add("the fence ended the test");
        if (t.timedOut > 0) t.reasons.add(t.timedOut + " run(s) timed out");
        if (t.worstErrIn > FINAL_ERROR_IN || t.worstErrDeg > FINAL_ERROR_DEG) t.reasons.add("final error over the limit");
        if (t.worstOverIn > OVERSHOOT_IN || t.worstOverDeg > OVERSHOOT_DEG) t.reasons.add("overshoot over the limit");
        if (t.worstTime > t.allowed) t.reasons.add("time over the limit");
        t.verdict = !t.reasons.isEmpty() ? "FAIL" : t.counted < RUNS_PER_TEST ? "INCOMPLETE" : "PASS";
        RunLog.eventf("ACCEPT TEST %d %s: %s%s; settled %d, timed out %d; final error worst %.2f in %.1f deg, mean %.2f in; overshoot worst %.2f in %.1f deg; "
                        + "time worst %.2f s against %.2f; lowest battery at rest %.2f V; not counted %d, void %d",
                number, t.name, t.verdict, t.reasons.isEmpty() ? "" : " " + t.reasons, t.settled, t.timedOut, t.worstErrIn, t.worstErrDeg,
                t.counted == 0 ? 0 : t.sumErrIn / t.counted, t.worstOverIn, t.worstOverDeg, t.worstTime, t.allowed,
                t.lowestBattery > 90 ? Double.NaN : t.lowestBattery, t.notCounted, t.voided);
    }
}
