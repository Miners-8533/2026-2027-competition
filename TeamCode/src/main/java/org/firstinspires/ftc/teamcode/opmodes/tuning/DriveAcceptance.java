package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

/**
 * DRIVE ACCEPTANCE (2026-11-17; specs/motion.md section 2.1 and specs/support-opmodes.md section 4.2, target T2): the robot drives the acceptance test of DriveToPose BY ITSELF and
 * reports every number of it, so a tuning session ends with a verdict and a record instead of an impression. Each test is one move driven RUNS_PER_TEST times, out and back in turn;
 * every run is one DriveToPose leg, measured: settled or timed out, the time, the final error, the overshoot. Each test gets a row and PASS, FAIL or INCOMPLETE against the limits below;
 * the run ends with one result line and the gains in force, ready to paste into DriveToPose.
 *
 * The tests: 48 in forward, 48 in strafe (to the robot's LEFT), a 90 deg turn, a 180 deg turn, a short 12 in move and a small 5 deg turn (the two cases that stall without the static
 * friction term). The static friction term is ON for every leg (STICTION).
 *
 * Setup: any clear floor. The start is (0, 0, 0 deg) wherever the robot stands at INIT: it needs LONG_IN (48 in) clear AHEAD and to its LEFT, plus the robot and a margin.
 * Keep the robot still from INIT to Start (the Pinpoint is reset and recalibrated). Use the tuners (Tune - Axial, Lateral, Heading) to find the gains; this opmode judges them.
 *
 * Gamepad 1:
 *   INIT                D-pad up / down = the first test (to go on from an earlier run)
 *   hold RIGHT BUMPER   the deadman: the robot drives only while it is held. Letting go stops it at once; the run in progress is void and is driven again when the bumper is held again
 *   X                   skip the current test (it is reported as SKIPPED, not as a pass)
 * Nothing else is used (no Start, no Back, no sticks). No mechanism moves.
 *
 * Safety: a fence FENCE_IN outside the rectangle a test should stay in ends that test (FAIL) and the robot drives back to the start.
 *
 * A run whose battery, read at rest just before it, is below MIN_BATTERY_V is driven and logged but NOT COUNTED; a test with fewer than RUNS_PER_TEST counted runs is INCOMPLETE.
 *
 * The log is runlogs/bringup.log: ACCEPT lines (one per run, one per test, the result, the gains), the DRIVE lines of every leg, the CONFIG lines with the gains and these limits,
 * and a row on every loop with the pose, the command sent, and the test, run and phase.
 */
@Config
@TeleOp(name = "Drive Acceptance", group = "Tuning")
public class DriveAcceptance extends LinearOpMode {

    public static int RUNS_PER_TEST = 10;
    /** The static friction term of DriveToPose on every leg (the acceptance test is of the control law WITH it). */
    public static boolean STICTION = true;
    public static double LONG_IN = 48.0, SHORT_IN = 12.0, SMALL_TURN_DEG = 5.0;

    // ---- the limits (proposed 2026-11-17; settle them at the tuning session: they are logged with every run) ----
    public static double MAX_FINAL_ERR_IN = 1.0, MAX_FINAL_ERR_DEG = 2.0;
    public static double MAX_OVERSHOOT_IN = 1.5, MAX_OVERSHOOT_DEG = 5.0;
    /** Time allowed for a long move or a turn: its length at this average speed, plus TIME_MARGIN_SEC. 22 in/s is what the autos' time caps assume at 0.6 power; a strafe is STRAFE_RATIO of it. */
    public static double TRAVEL_IN_PER_S = 22.0, STRAFE_RATIO = 0.78, TURN_DEG_PER_S = 120.0, TIME_MARGIN_SEC = 1.0;
    /** Time allowed for the short move and the small turn. */
    public static double SHORT_TIME_LIMIT_SEC = 1.5;
    /** The battery at rest must be at least this for a run to count (Pit Check's limit). */
    public static double MIN_BATTERY_V = 12.0;

    /** The pause before each run, robot stopped: the last move dies out and the battery is read at rest. */
    public static double REST_SEC = 0.6;
    /** A test ends (FAIL) when the robot's origin is this far outside the rectangle between the test's two poses. */
    public static double FENCE_IN = 15.0;
    /** Time cap of an unmeasured leg back to the start of a run. */
    public static double RETURN_TIMEOUT_SEC = 6.0;

    private enum Phase { PAUSED, RETURN, REST, RUN, DONE }

    /** One test: the move between the start pose and {@code far}, and what its runs measured. */
    private static final class Test {
        final String name;
        final Pose2d far;
        final double limitSec;
        int counted = 0, settled = 0, timedOut = 0, lowBattery = 0, voided = 0;
        double worstErrIn = 0.0, sumErrIn = 0.0, worstErrDeg = 0.0, sumErrDeg = 0.0;
        double worstOverIn = 0.0, worstOverDeg = 0.0, worstSec = 0.0;
        double lowestV = Double.NaN;
        String verdict = "", row = "";

        Test(String name, Pose2d far, double limitSec) {
            this.name = name;
            this.far = far;
            this.limitSec = limitSec;
        }
    }

    private String row = "";

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Drive Acceptance", hardwareMap);
        try {
            run();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private static List<Test> buildTests() {
        List<Test> tests = new ArrayList<>();
        double longSec = LONG_IN / TRAVEL_IN_PER_S + TIME_MARGIN_SEC;
        tests.add(new Test(String.format(Locale.US, "forward %.0f in", LONG_IN), new Pose2d(LONG_IN, 0.0, 0.0), longSec));
        tests.add(new Test(String.format(Locale.US, "strafe %.0f in", LONG_IN), new Pose2d(0.0, LONG_IN, 0.0), LONG_IN / (TRAVEL_IN_PER_S * STRAFE_RATIO) + TIME_MARGIN_SEC));
        tests.add(new Test("turn 90 deg", new Pose2d(0.0, 0.0, Math.toRadians(90.0)), 90.0 / TURN_DEG_PER_S + TIME_MARGIN_SEC));
        tests.add(new Test("turn 180 deg", new Pose2d(0.0, 0.0, Math.toRadians(180.0)), 180.0 / TURN_DEG_PER_S + TIME_MARGIN_SEC));
        tests.add(new Test(String.format(Locale.US, "short move %.0f in", SHORT_IN), new Pose2d(SHORT_IN, 0.0, 0.0), SHORT_TIME_LIMIT_SEC));
        tests.add(new Test(String.format(Locale.US, "small turn %.0f deg", SMALL_TURN_DEG), new Pose2d(0.0, 0.0, Math.toRadians(SMALL_TURN_DEG)), SHORT_TIME_LIMIT_SEC));
        return tests;
    }

    private static boolean near(Pose2d p, Pose2d target) {
        return Math.hypot(p.position.x - target.position.x, p.position.y - target.position.y) < DriveToPose.POS_TOLERANCE_IN
                && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - target.heading.toDouble()))) < DriveToPose.HEADING_TOLERANCE_DEG;
    }

    private static boolean outsideFence(Pose2d p, Test t) {
        double x = p.position.x, y = p.position.y;
        return x < Math.min(0.0, t.far.position.x) - FENCE_IN || x > Math.max(0.0, t.far.position.x) + FENCE_IN
                || y < Math.min(0.0, t.far.position.y) - FENCE_IN || y > Math.max(0.0, t.far.position.y) + FENCE_IN;
    }

    private double readBattery(VoltageSensor battery) {
        if (battery == null) return Double.NaN;
        try {
            return battery.getVoltage();
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    /** Closes a test: its verdict against the limits, its row for the screen, and its ACCEPT TEST line. {@code ended} is "" for a test that ran through, or why it did not. */
    private void finishTest(int number, Test t, String ended) {
        List<String> why = new ArrayList<>();
        if (!ended.isEmpty()) {
            t.verdict = ended.startsWith("SKIPPED") ? "SKIPPED" : "FAIL";
            why.add(ended);
        } else if (t.counted < RUNS_PER_TEST) {
            t.verdict = "INCOMPLETE";
            why.add(String.format(Locale.US, "only %d of %d runs counted (%d with the battery below %.1f V at rest)", t.counted, RUNS_PER_TEST, t.lowBattery, MIN_BATTERY_V));
        } else {
            if (t.timedOut > 0) why.add(t.timedOut + " timed out");
            if (t.worstErrIn > MAX_FINAL_ERR_IN) why.add(String.format(Locale.US, "final error %.2f in > %.2f", t.worstErrIn, MAX_FINAL_ERR_IN));
            if (t.worstErrDeg > MAX_FINAL_ERR_DEG) why.add(String.format(Locale.US, "final error %.1f deg > %.1f", t.worstErrDeg, MAX_FINAL_ERR_DEG));
            if (t.worstOverIn > MAX_OVERSHOOT_IN) why.add(String.format(Locale.US, "overshoot %.2f in > %.2f", t.worstOverIn, MAX_OVERSHOOT_IN));
            if (t.worstOverDeg > MAX_OVERSHOOT_DEG) why.add(String.format(Locale.US, "overshoot %.1f deg > %.1f", t.worstOverDeg, MAX_OVERSHOOT_DEG));
            if (t.worstSec > t.limitSec) why.add(String.format(Locale.US, "time %.2f s > %.2f", t.worstSec, t.limitSec));
            t.verdict = why.isEmpty() ? "PASS" : "FAIL";
        }
        int n = Math.max(1, t.counted);
        t.row = String.format(Locale.US, "%d %s: %s  err %.1f in/%.1f deg  over %.1f/%.1f  %.1f s", number, t.name, t.verdict, t.worstErrIn, t.worstErrDeg, t.worstOverIn, t.worstOverDeg, t.worstSec);
        RunLog.event(String.format(Locale.US, "ACCEPT TEST %d %s: %s%s | settled %d, timed out %d of %d counted runs (%d not counted: battery; %d void: deadman) | final error worst %.2f in / %.1f deg, mean %.2f in / %.1f deg"
                        + " | overshoot worst %.2f in / %.1f deg | time worst %.2f s (limit %.2f) | battery at rest lowest %.2f V",
                number, t.name, t.verdict, why.isEmpty() ? "" : " (" + String.join("; ", why) + ")", t.settled, t.timedOut, t.counted, t.lowBattery, t.voided,
                t.worstErrIn, t.worstErrDeg, t.sumErrIn / n, t.sumErrDeg / n, t.worstOverIn, t.worstOverDeg, t.worstSec, t.limitSec, t.lowestV));
    }

    private void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        final Pose2d origin = new Pose2d(0.0, 0.0, 0.0);
        final MecanumDrive drive = new MecanumDrive(hardwareMap, origin);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        VoltageSensor battery = sensors.hasNext() ? sensors.next() : null;
        int first = 1;
        final int testCount = buildTests().size();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpadUpWasPressed()) first = Math.min(testCount, first + 1);
            if (gamepad1.dpadDownWasPressed()) first = Math.max(1, first - 1);
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            boolean poseOk = near(p, origin) && loc.seedConfirmed;
            double v = readBattery(battery);
            telemetry.addData("First test (D-pad up / down)", "%d of %d: %s", first, testCount, buildTests().get(first - 1).name);
            telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   %s", loc.getDeviceStatus(), p.position.x, p.position.y, p.heading.toDegrees(),
                    poseOk ? "OK" : "WARNING: not at the start (do not move the robot after INIT; restart the opmode)");
            telemetry.addData("Battery", "%.2f V%s", v, v < MIN_BATTERY_V ? String.format(Locale.US, "   BELOW %.1f V: runs will not count", MIN_BATTERY_V) : "");
            telemetry.addData("Gamepad 1", gamepad1.getGamepadId() != Gamepad.ID_UNASSOCIATED ? "assigned (" + gamepad1.type() + ")" : "NOT ASSIGNED");
            telemetry.addData("Runs per test", "%d, static friction term %s", RUNS_PER_TEST, STICTION ? "ON" : "OFF");
            telemetry.addLine(String.format(Locale.US, "Needs %.0f in of clear floor AHEAD of the robot and to its LEFT, and room to spin.", LONG_IN + FENCE_IN));
            telemetry.addLine("After Start, HOLD THE RIGHT BUMPER to drive; let go to stop. X skips a test.");
            telemetry.update();
            sleep(40);   // the battery read each pass costs a hub round trip; INIT needs no faster
        }
        if (isStopRequested()) return;
        RunLog.started();
        List<Test> tests = buildTests();   // the Dashboard values as they are at Start
        RunLog.event(String.format(Locale.US, "SETUP Drive Acceptance: %d tests from test %d, %d runs each, static friction term %s; limits: final error %.2f in / %.1f deg, overshoot %.2f in / %.1f deg, "
                        + "time = length at %.0f in/s (strafe x %.2f) or %.0f deg/s + %.1f s, short tests %.1f s, battery at rest at least %.1f V; fence %.0f in",
                tests.size(), first, RUNS_PER_TEST, STICTION ? "ON" : "OFF", MAX_FINAL_ERR_IN, MAX_FINAL_ERR_DEG, MAX_OVERSHOOT_IN, MAX_OVERSHOOT_DEG,
                TRAVEL_IN_PER_S, STRAFE_RATIO, TURN_DEG_PER_S, TIME_MARGIN_SEC, SHORT_TIME_LIMIT_SEC, MIN_BATTERY_V, FENCE_IN));
        RunLog.event("ACCEPT GAINS at Start: " + DriveToPose.gainsText());
        RunLog.setSampler("x,y,hdg_deg,cmd_fwd,cmd_left,cmd_turn,test,run,phase", 0, () -> row);

        int ti = first - 1, runIdx = 0;
        Phase phase = Phase.PAUSED;
        Phase resume = Phase.RETURN;   // what a held deadman goes on with
        DriveToPose leg = null;
        ElapsedTime rest = new ElapsedTime();
        double restV = Double.NaN;
        String last = "";
        boolean resultLogged = false;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            boolean deadman = gamepad1.right_bumper;
            boolean skip = gamepad1.xWasPressed();
            Test t = ti < tests.size() ? tests.get(ti) : null;
            boolean poseRead = false;

            if (t == null) {
                phase = Phase.DONE;
            } else if (skip) {
                drive.stop();
                leg = null;
                finishTest(ti + 1, t, "SKIPPED after " + runIdx + " runs");
                ti++;
                runIdx = 0;
                resume = Phase.RETURN;
                phase = Phase.PAUSED;
            } else if (!deadman) {
                if (phase == Phase.RUN || phase == Phase.RETURN || phase == Phase.REST) {
                    drive.stop();
                    if (phase == Phase.RUN) {
                        t.voided++;
                        last = "run " + (runIdx + 1) + " VOID: the deadman was released";
                        RunLog.event(String.format(Locale.US, "ACCEPT run void: test %d %s, run %d of %d: the deadman was released; it is driven again", ti + 1, t.name, runIdx + 1, RUNS_PER_TEST));
                    }
                    leg = null;
                    resume = Phase.RETURN;
                }
                phase = Phase.PAUSED;
            } else {
                Pose2d from = runIdx % 2 == 0 ? origin : t.far, to = runIdx % 2 == 0 ? t.far : origin;
                if (phase == Phase.PAUSED) phase = resume;
                if (phase == Phase.RETURN) {
                    if (leg == null) {
                        if (near(drive.getPose(), from)) {
                            phase = Phase.REST;
                            rest.reset();
                        } else {
                            leg = new DriveToPose(drive, from).withStiction(STICTION).withTimeout(RETURN_TIMEOUT_SEC);
                        }
                    }
                    if (leg != null) {
                        poseRead = true;
                        if (!leg.run(packet)) {
                            leg = null;
                            phase = Phase.REST;
                            rest.reset();
                        }
                    }
                } else if (phase == Phase.REST) {
                    if (rest.seconds() >= REST_SEC) {
                        restV = readBattery(battery);
                        leg = new DriveToPose(drive, to).withStiction(STICTION).withTimeout(Math.max(DriveToPose.TIMEOUT_SEC, t.limitSec + 1.0));
                        phase = Phase.RUN;
                    }
                } else if (phase == Phase.RUN) {
                    poseRead = true;
                    if (!leg.run(packet)) {
                        boolean counts = !(restV < MIN_BATTERY_V);   // an unreadable battery (NaN) does not stop a run from counting
                        double errDeg = Math.abs(leg.getFinalHeadingErrorDeg());
                        if (counts) {
                            t.counted++;
                            if (leg.isSettled()) t.settled++; else t.timedOut++;
                            t.worstErrIn = Math.max(t.worstErrIn, leg.getFinalErrorIn());
                            t.sumErrIn += leg.getFinalErrorIn();
                            t.worstErrDeg = Math.max(t.worstErrDeg, errDeg);
                            t.sumErrDeg += errDeg;
                            t.worstOverIn = Math.max(t.worstOverIn, leg.getOvershootIn());
                            t.worstOverDeg = Math.max(t.worstOverDeg, leg.getOvershootDeg());
                            t.worstSec = Math.max(t.worstSec, leg.getElapsedSec());
                        } else {
                            t.lowBattery++;
                        }
                        if (!Double.isNaN(restV)) t.lowestV = Double.isNaN(t.lowestV) ? restV : Math.min(t.lowestV, restV);
                        last = String.format(Locale.US, "run %d %s in %.2f s, error %.2f in / %.1f deg, overshoot %.2f in / %.1f deg%s", runIdx + 1, leg.isSettled() ? "settled" : "TIMED OUT",
                                leg.getElapsedSec(), leg.getFinalErrorIn(), errDeg, leg.getOvershootIn(), leg.getOvershootDeg(), counts ? "" : "  NOT COUNTED (battery)");
                        RunLog.event(String.format(Locale.US, "ACCEPT run: test %d %s, run %d of %d (%s): %s in %.2f s (limit %.2f), final error %.2f in / %+.1f deg, overshoot %.2f in / %.1f deg, battery at rest %.2f V%s",
                                ti + 1, t.name, runIdx + 1, RUNS_PER_TEST, runIdx % 2 == 0 ? "out" : "back", leg.isSettled() ? "settled" : "TIMED OUT", leg.getElapsedSec(), t.limitSec,
                                leg.getFinalErrorIn(), leg.getFinalHeadingErrorDeg(), leg.getOvershootIn(), leg.getOvershootDeg(), restV,
                                counts ? "" : String.format(Locale.US, ", NOT COUNTED: below %.1f V", MIN_BATTERY_V)));
                        leg = null;
                        runIdx++;
                        if (runIdx >= RUNS_PER_TEST) {
                            finishTest(ti + 1, t, "");
                            ti++;
                            runIdx = 0;
                        }
                        phase = Phase.RETURN;
                    }
                }
                // the fence: a test that leaves its rectangle is over, and the robot is driven back to the start
                if (ti < tests.size() && tests.get(ti) == t && (phase == Phase.RUN || phase == Phase.RETURN) && outsideFence(drive.getPose(), t)) {
                    drive.stop();
                    leg = null;
                    Pose2d p = drive.getPose();
                    finishTest(ti + 1, t, String.format(Locale.US, "FENCE: the robot left the test's rectangle by more than %.0f in at (%.1f, %.1f) in run %d", FENCE_IN, p.position.x, p.position.y, runIdx + 1));
                    ti++;
                    runIdx = 0;
                    phase = Phase.RETURN;
                }
            }
            if (!poseRead) drive.updatePoseEstimate();   // a running leg reads the pose itself: once per loop either way

            if (phase == Phase.DONE && !resultLogged) {
                resultLogged = true;
                drive.stop();
                logResult(tests, first, "");
            }

            Pose2d pose = drive.getPose();
            PoseVelocity2d cmd = drive.getLastCommand();
            row = String.format(Locale.US, "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%s", pose.position.x, pose.position.y, pose.heading.toDegrees(),
                    cmd.linearVel.x, cmd.linearVel.y, cmd.angVel, Math.min(ti + 1, tests.size()), runIdx + 1, phase);
            dashboard.sendTelemetryPacket(packet);
            RunLog.tick();

            telemetry.addLine(phase == Phase.DONE ? "*** DONE: stop the opmode ***" : phase == Phase.PAUSED ? "*** HOLD THE RIGHT BUMPER TO DRIVE ***" : "DRIVING by itself: let go of the right bumper to stop");
            if (ti < tests.size()) {
                Test now = tests.get(ti);
                telemetry.addData("Test", "%d of %d: %s, run %d of %d (%s)", ti + 1, tests.size(), now.name, runIdx + 1, RUNS_PER_TEST,
                        phase == Phase.RETURN ? "going to its start" : phase == Phase.REST ? "resting" : phase == Phase.RUN ? "measuring" : "paused");
            } else {
                telemetry.addData("Result", resultLine(tests, first, ""));
            }
            if (!last.isEmpty()) telemetry.addData("Last", last);
            for (Test done : tests) {
                if (!done.row.isEmpty()) telemetry.addLine(done.row);
            }
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f   %s", pose.position.x, pose.position.y, pose.heading.toDegrees(), loc.getDeviceStatus());
            telemetry.addLine("Hold RB = drive, X = skip this test");
            telemetry.update();
        }
        drive.stop();
        if (!resultLogged) logResult(tests, first, "the opmode was stopped in test " + Math.min(ti + 1, tests.size()));
    }

    /** ALL PASS, or FAILED / INCOMPLETE with the tests that are not a PASS. {@code stopped} is "" or why the run ended early. */
    private static String resultLine(List<Test> tests, int first, String stopped) {
        List<String> bad = new ArrayList<>();
        int passed = 0;
        boolean failed = false;
        for (int i = first - 1; i < tests.size(); i++) {
            Test t = tests.get(i);
            if (t.verdict.equals("PASS")) {
                passed++;
            } else {
                bad.add((i + 1) + " " + t.name + (t.verdict.isEmpty() ? " NOT RUN" : " " + t.verdict));
                if (t.verdict.equals("FAIL")) failed = true;
            }
        }
        int wanted = tests.size() - (first - 1);
        if (bad.isEmpty() && stopped.isEmpty()) return "ALL PASS (" + passed + " tests" + (first > 1 ? ", from test " + first + " only" : "") + ")";
        return (failed ? "FAILED" : "INCOMPLETE") + ": " + passed + " of " + wanted + " passed; " + String.join(", ", bad) + (stopped.isEmpty() ? "" : "; " + stopped);
    }

    private static void logResult(List<Test> tests, int first, String stopped) {
        RunLog.event("ACCEPT RESULT: " + resultLine(tests, first, stopped));
        RunLog.event(String.format(Locale.US, "ACCEPT GAINS (hub clock %s, static friction term %s): %s", new SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US).format(new Date()),
                STICTION ? "ON" : "OFF", DriveToPose.gainsText()));
    }
}
