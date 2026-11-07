package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.DriveTestRoutine;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Iterator;
import java.util.List;
import java.util.Locale;

/**
 * DRIVE CHARACTERIZATION (2026-11-06; design/dash-planner-grid.md, "Drive characterization"): the robot drives a fixed list of open-loop drive tests BY ITSELF on
 * one alliance's half of the field and logs every loop, so the drive model can be fitted from clean data (stops at zero and at part power, turning while rolling,
 * changing direction at speed, steady speed against power, spins, the dead zone, random commands). The tests are {@link DriveTestRoutine} (about 2.7 minutes of
 * driving in the simulation); the geofence there keeps the robot on its own half, clear of the walls, the hive and the flowers.
 *
 * Setup: place the robot still at the TeleOp start of the half you have (BLUE: back against the rear wall, y = +16, facing the audience; RED: back against the
 * audience wall, y = -16, facing the rear wall) and init. The robot uses its own half only (blue y &gt; 0, red y &lt; 0) and needs the hive's own-side bar and the
 * flowers where the field has them (it keeps clear of where they would be either way).
 *
 * Gamepad 1:
 *   INIT          D-pad right = BLUE half, D-pad left = RED half (nothing moves until one is chosen); D-pad up / down = the first test (to go on from an earlier session)
 *   hold RIGHT BUMPER   the deadman: the robot drives the tests only while it is held; letting go stops it at once (the motors brake), holding it again restarts
 *                 the current test from its start
 *   X             skip the current test
 * Nothing else is used (no Start, no Back, no sticks). No mechanism moves.
 *
 * The log is runlogs/drivetest.log (tools/logs/pull_logs.py files it as robot-logs/DAY/HHMMSS_drivetest.log): a row on EVERY loop with the field pose, the
 * Pinpoint's field velocity, the command sent and the four wheel powers after scaling, the battery read every loop, one drive motor current per loop in turn,
 * and the test, segment and phase; DRIVE TEST events mark each test, segment, geofence stop and pause.
 */
@Config
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@TeleOp(name = "Drive Characterization", group = "Tuning")
public class DriveCharacterization extends LinearOpMode {

    public static double START_Y_IN = 16.0;
    /** Read the battery every loop (about 2 ms); false: the run log's own reading once a second. */
    public static boolean READ_VOLTAGE_EVERY_LOOP = true;
    /** Read one drive motor's current per loop, in turn (about 2 ms each); false: none. */
    public static boolean READ_CURRENTS = true;

    private String row = "";

    private static Pose2d startFor(boolean blue, double startX) {
        return blue ? new Pose2d(startX, START_Y_IN, 0.0) : new Pose2d(-startX, -START_Y_IN, Math.PI);
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.DRIVETEST, "Drive Characterization", hardwareMap);
        try {
            run();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        double startX = -(FieldGeometry.HALF_IN - profile.originFromBackIn);   // back against the wall

        boolean blue = true, chosen = false;
        Pose2d expected = startFor(true, startX);
        MecanumDrive drive = new MecanumDrive(hardwareMap, expected);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        DcMotorEx[] motors = {drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront};
        Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        VoltageSensor battery = sensors.hasNext() ? sensors.next() : null;
        List<DriveTestRoutine.Test> tests = DriveTestRoutine.standardTests();
        int first = 1;

        while (!isStarted() && !isStopRequested()) {
            boolean wantBlue = gamepad1.dpad_right, wantRed = gamepad1.dpad_left && !gamepad1.dpad_right;
            if ((wantBlue || wantRed) && (!chosen || wantBlue != blue)) {
                blue = wantBlue;
                chosen = true;
                expected = startFor(blue, startX);
                drive.setPose(expected);
            }
            if (gamepad1.dpadUpWasPressed()) first = Math.min(tests.size(), first + 1);
            if (gamepad1.dpadDownWasPressed()) first = Math.max(1, first - 1);
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            boolean poseOk = Math.hypot(p.position.x - expected.position.x, p.position.y - expected.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - expected.heading.toDouble()))) < 3.0;
            telemetry.addData("Half", chosen ? (blue ? "BLUE (y > 0)" : "RED (y < 0)") + String.format(Locale.US, ", start (%.1f, %.1f, %.0f deg)",
                    expected.position.x, expected.position.y, expected.heading.toDegrees()) : "NOT CHOSEN: D-pad right = BLUE half, D-pad left = RED half");
            telemetry.addData("First test (D-pad up / down)", "%d of %d: %s", first, tests.size(), tests.get(first - 1).name);
            telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   %s", loc.getDeviceStatus(), p.position.x, p.position.y, p.heading.toDegrees(),
                    poseOk && loc.seedConfirmed ? "OK" : "WARNING: does not match the start (do not move the robot after INIT; restart the opmode)");
            telemetry.addData("Gamepad 1", gamepad1.getGamepadId() != Gamepad.ID_UNASSOCIATED ? "assigned (" + gamepad1.type() + ")" : "NOT ASSIGNED");
            telemetry.addLine("Place the robot still at the start of the chosen half, then Start. After Start, HOLD THE RIGHT BUMPER to drive; let go to stop.");
            telemetry.addLine("Keep everyone off the half: the robot drives at full power by itself.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.event(String.format(Locale.US, "SETUP profile %s, half %s, start (%.1f, %.1f, %.1f deg), first test %d of %d, footprint %.2f x %.2f in (origin %.2f from the back)",
                profile.name, chosen ? (blue ? "BLUE" : "RED") : "NOT CHOSEN", expected.position.x, expected.position.y, expected.heading.toDegrees(), first, tests.size(),
                profile.robotDepthIn, profile.robotWidthIn, profile.originFromBackIn));
        RunLog.setSampler("x,y,hdg_deg,vx,vy,w_dps,cmd_fwd,cmd_left,cmd_turn,p_lf,p_lb,p_rb,p_rf,vbat,a_lf,a_lb,a_rb,a_rf,test,seg,phase", 0, () -> row);

        DriveTestRoutine routine = new DriveTestRoutine(tests, first, profile.robotDepthIn, profile.robotWidthIn, profile.originFromBackIn);
        ElapsedTime clock = new ElapsedTime();
        double[] amps = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
        int ampIndex = 0;
        double vbat = Double.NaN;
        double drivingSec = 0.0, lastT = 0.0;

        while (opModeIsActive()) {
            if (!chosen && (gamepad1.dpad_right || gamepad1.dpad_left)) {
                blue = gamepad1.dpad_right;
                chosen = true;
                expected = startFor(blue, startX);
                drive.setPose(expected);
                RunLog.event(String.format(Locale.US, "HALF chosen after Start: %s (%.1f, %.1f, %.1f deg)", blue ? "BLUE" : "RED",
                        expected.position.x, expected.position.y, expected.heading.toDegrees()));
            }
            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            double t = clock.seconds();
            double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
            double vx = vel == null ? 0.0 : vel.linearVel.x, vy = vel == null ? 0.0 : vel.linearVel.y, w = vel == null ? 0.0 : vel.angVel;
            // the routine works in the blue frame: a red robot's pose is turned 180 deg about the field center (the powers are robot-frame and need no turning)
            double s = blue ? 1.0 : -1.0;
            boolean deadman = chosen && gamepad1.right_bumper;
            boolean skip = gamepad1.xWasPressed() && chosen;
            DriveTestRoutine.Output out = routine.update(t, s * x, s * y, blue ? h : Rotation2d.normalize(h + Math.PI), s * vx, s * vy, w, deadman, skip);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(out.forward, out.left), out.turn));
            if (out.phase == DriveTestRoutine.Phase.RUN) drivingSec += t - lastT;
            lastT = t;

            // what MecanumDrive sends: the wheel powers over the largest when it passes 1
            double lf = out.forward - out.left - out.turn, lb = out.forward + out.left - out.turn, rb = out.forward - out.left + out.turn, rf = out.forward + out.left + out.turn;
            double mx = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rb), Math.abs(rf))));
            if (READ_VOLTAGE_EVERY_LOOP && battery != null) {
                try {
                    vbat = battery.getVoltage();
                } catch (RuntimeException e) {
                    vbat = Double.NaN;
                }
            }
            if (READ_CURRENTS) {
                try {
                    amps[ampIndex] = motors[ampIndex].getCurrent(CurrentUnit.AMPS);
                } catch (RuntimeException e) {
                    amps[ampIndex] = Double.NaN;
                }
                ampIndex = (ampIndex + 1) % 4;
            }
            row = String.format(Locale.US, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%s",
                    x, y, Math.toDegrees(h), vx, vy, Math.toDegrees(w), out.forward, out.left, out.turn, lf / mx, lb / mx, rb / mx, rf / mx, vbat,
                    amps[0], amps[1], amps[2], amps[3], out.test, out.seg, out.phase);
            RunLog.tick();

            if (!chosen) telemetry.addLine("*** CHOOSE THE HALF: D-pad right = BLUE, D-pad left = RED (nothing drives until you do) ***");
            telemetry.addLine(out.phase == DriveTestRoutine.Phase.PAUSED ? "*** HOLD THE RIGHT BUMPER TO DRIVE ***" : out.phase == DriveTestRoutine.Phase.DONE ? "*** DONE: stop the opmode ***"
                    : "DRIVING by itself: let go of the right bumper to stop");
            telemetry.addData("Test", "%d of %d: %s", out.test, routine.count(), routine.testName(out.test));
            telemetry.addData("Now", out.status);
            telemetry.addData("Time", "%d:%02d since Start, %.0f s of test driving; %d done, %d stopped or skipped", (int) (t / 60), (int) (t % 60), drivingSec,
                    routine.finished(), routine.aborted());
            if (!routine.lastFence().isEmpty()) telemetry.addData("Last geofence stop", routine.lastFence());
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f   %s", x, y, Math.toDegrees(h), loc.getDeviceStatus());
            telemetry.addData("Battery", "%.2f V", vbat);
            telemetry.addLine("Hold RB = drive, X = skip this test");
            telemetry.update();
        }
        drive.stop();
    }
}
