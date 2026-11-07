package org.firstinspires.ftc.teamcode.opmodes.autobuild;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpCompetition;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Auto Waypoint Marker: for building autos. The robot starts in one of the two known start positions, chosen at INIT; you drive it
 * to points on the field, press A at each one to MARK the pose, and use the marks to write the auto routines afterwards.
 *
 * Start positions (the same as TeleOp - Competition): BLUE at X = -(72 - depth / 2) (the rear wall), Y = +16, facing +X;
 * RED at X = +(72 - depth / 2) (the audience wall), Y = -16, facing -X. The robot must be placed there, still, before INIT
 * is finished, and must not be moved until Start (the Pinpoint calibrates at INIT).
 *
 * Gamepad 1 (one driver):
 *   INIT                 D-pad right = BLUE start, D-pad left = RED start. If none was chosen by Start, nothing drives until you
 *                        choose (the robot must not have moved); once chosen the start cannot be changed in the run
 *   left stick           drive, field-oriented: up = away from the driver (red faces +Y, blue faces -Y, like the TeleOp)
 *   right stick X        turn
 *   right trigger (hold) precision mode (30%)
 *   A                    MARK the current pose as the next numbered point (the screen and the log list it)
 *   X / Y                deploy / stow the ramp (each mark records whether the ramp was deployed)
 *   B                    CHECK mark, numbered separately from the waypoints: drive the robot to a spot whose real position you know (for example
 *                        squared against a wall or into a corner) and press B, then compare the logged pose with the real one afterwards
 *   left + right bumper  (hold 1 s) reseed the pose to the chosen start, if the pose is lost
 *
 * Each mark is shown on the screen and written to the run log as a MARK event with a ready-to-paste line, e.g.
 *   MARK 3: x 12.5, y -30.2, heading 90.0 deg, ramp deployed | new Pose2d(12.5, -30.2, Math.toRadians(90.0))
 * The log is runlogs/waypoints.log on the hub (pull it with tools/logs/pull_logs.py; its Marks column counts them), and the pose is
 * sampled every 100 ms so the driven path is there too. There is no undo: an extra or wrong mark is dealt with when reading the log.
 *
 * No mechanism but the ramp is used. Not for matches.
 */
@Config
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@TeleOp(name = "Auto Waypoint Marker", group = "Auto build")
public class AutoWaypointMarker extends LinearOpMode {

    public static double START_Y_IN = 16.0;
    public static double RESEED_HOLD_SEC = 1.0;
    public static long SAMPLE_PERIOD_MS = 100;
    /** How many of the latest marks the screen lists. */
    public static int SHOWN_MARKS = 8;


    private double logX, logY, logHeadingDeg;
    private boolean logRampDeployed;
    private int logMarks;

    private static Pose2d startFor(boolean blue, double startX) {
        return blue ? new Pose2d(startX, START_Y_IN, 0.0) : new Pose2d(-startX, -START_Y_IN, Math.PI);
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.WAYPOINTS, "Auto Waypoint Marker", hardwareMap);
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

        // built in INIT with the BLUE start; re-seeded when a start is chosen (the robot has not moved)
        boolean blue = true, chosen = false;
        Pose2d expected = startFor(true, startX);
        MecanumDrive drive = new MecanumDrive(hardwareMap, expected);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Ramp ramp = hardwareMap.tryGet(com.qualcomm.robotcore.hardware.Servo.class, "depolLeft") != null
                && hardwareMap.tryGet(com.qualcomm.robotcore.hardware.Servo.class, "depolRight") != null ? new Ramp(hardwareMap) : null;

        while (!isStarted() && !isStopRequested()) {
            boolean wantBlue = gamepad1.dpad_right, wantRed = gamepad1.dpad_left && !gamepad1.dpad_right;
            if ((wantBlue || wantRed) && (!chosen || wantBlue != blue)) {
                blue = wantBlue;
                chosen = true;
                expected = startFor(blue, startX);
                drive.setPose(expected);
            }
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            boolean poseOk = Math.hypot(p.position.x - expected.position.x, p.position.y - expected.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - expected.heading.toDouble()))) < 3.0;
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Start", chosen ? (blue ? "BLUE" : "RED") + String.format(Locale.US, "  (%.1f, %.1f, %.0f deg)",
                    expected.position.x, expected.position.y, expected.heading.toDegrees())
                    : "NOT CHOSEN: D-pad right = BLUE start, D-pad left = RED start");
            telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   %s", loc.getDeviceStatus(), p.position.x, p.position.y,
                    p.heading.toDegrees(), poseOk && loc.seedConfirmed ? "OK" : "WARNING: does not match the start (do not move the robot after INIT; restart the opmode)");
            telemetry.addData("Ramp", ramp == null ? "not configured" : ramp.isConfigured() ? "ready" : "positions not measured: it will not move");
            telemetry.addData("Gamepad 1", gamepad1.getGamepadId() != Gamepad.ID_UNASSOCIATED ? "assigned (" + gamepad1.type() + ")" : "NOT ASSIGNED: press Start + A on it");
            telemetry.addLine("Place the robot still at the chosen start. Then press Start on the Driver Station.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.event(String.format(Locale.US, "SETUP profile %s, start %s (%.1f, %.1f, %.1f deg)", profile.name,
                chosen ? (blue ? "BLUE" : "RED") : "NOT CHOSEN", expected.position.x, expected.position.y, expected.heading.toDegrees()));
        RunLog.setSampler("x,y,hdg_deg,ramp,marks", SAMPLE_PERIOD_MS,
                () -> String.format(Locale.US, "%.2f,%.2f,%.1f,%s,%d", logX, logY, logHeadingDeg, logRampDeployed ? "deployed" : "stowed", logMarks));

        List<String> marks = new ArrayList<>();
        boolean reseedDown = false, reseeded = false;
        int checks = 0;
        ElapsedTime reseedHold = new ElapsedTime();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            logX = pose.position.x;
            logY = pose.position.y;
            logHeadingDeg = pose.heading.toDegrees();
            logRampDeployed = ramp != null && ramp.isDeployedTarget();
            logMarks = marks.size();

            if (!chosen && (gamepad1.dpad_right || gamepad1.dpad_left)) {
                blue = gamepad1.dpad_right;
                chosen = true;
                expected = startFor(blue, startX);
                drive.setPose(expected);
                pose = expected;
                RunLog.event(String.format(Locale.US, "START chosen after Start: %s (%.1f, %.1f, %.1f deg)", blue ? "BLUE" : "RED",
                        expected.position.x, expected.position.y, expected.heading.toDegrees()));
            }

            if (chosen) {
                double driverForwardRad = Math.toRadians(blue ? TeleOpCompetition.BLUE_DRIVER_FORWARD_DEG : TeleOpCompetition.RED_DRIVER_FORWARD_DEG);
                double scale = gamepad1.right_trigger > TeleOpCompetition.TRIGGER_THRESHOLD ? TeleOpCompetition.PRECISION_SCALE : 1.0;
                double up = -gamepad1.left_stick_y * scale;
                double right = gamepad1.left_stick_x * scale;
                double fieldX = up * Math.cos(driverForwardRad) + right * Math.cos(driverForwardRad - Math.PI / 2.0);
                double fieldY = up * Math.sin(driverForwardRad) + right * Math.sin(driverForwardRad - Math.PI / 2.0);
                double heading = pose.heading.toDouble();
                double robotForward = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
                double robotLeft = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);
                double turn = -gamepad1.right_stick_x * TeleOpCompetition.TURN_SCALE * scale;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotForward, robotLeft), turn));
            } else {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
            }

            if (ramp != null) {
                if (gamepad1.xWasPressed()) {
                    ramp.deploy();
                    RunLog.event("RAMP deploy");
                }
                if (gamepad1.yWasPressed()) {
                    ramp.stow();
                    RunLog.event("RAMP stow");
                }
            }

            if (gamepad1.aWasPressed() && chosen) {
                String text = String.format(Locale.US, "MARK %d: x %.1f, y %.1f, heading %.1f deg, ramp %s | new Pose2d(%.1f, %.1f, Math.toRadians(%.1f))",
                        marks.size() + 1, pose.position.x, pose.position.y, pose.heading.toDegrees(), logRampDeployed ? "deployed" : "stowed",
                        pose.position.x, pose.position.y, pose.heading.toDegrees());
                marks.add(text);
                RunLog.event(text);
            }

            if (gamepad1.bWasPressed() && chosen) {
                checks++;
                RunLog.event(String.format(Locale.US, "CHECK %d: x %.1f, y %.1f, heading %.1f deg, start %s", checks, pose.position.x, pose.position.y,
                        pose.heading.toDegrees(), blue ? "BLUE" : "RED"));
            }

            if (chosen && gamepad1.left_bumper && gamepad1.right_bumper) {
                if (!reseedDown) {
                    reseedDown = true;
                    reseeded = false;
                    reseedHold.reset();
                }
                if (!reseeded && reseedHold.seconds() >= RESEED_HOLD_SEC) {
                    drive.setPose(startFor(blue, startX));
                    reseeded = true;
                    RunLog.event(String.format(Locale.US, "POSE reseeded to the start (was %.1f, %.1f, %.1f deg)",
                            pose.position.x, pose.position.y, pose.heading.toDegrees()));
                }
            } else {
                reseedDown = false;
            }
            RunLog.tick();

            if (!chosen) telemetry.addLine("*** CHOOSE THE START: D-pad right = BLUE, D-pad left = RED (nothing drives until you do) ***");
            telemetry.addData("Start", chosen ? (blue ? "BLUE" : "RED") : "not chosen");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f", pose.position.x, pose.position.y, pose.heading.toDegrees());
            telemetry.addData("Pinpoint", loc.getDeviceStatus());
            telemetry.addData("Ramp", ramp == null ? "not configured" : (logRampDeployed ? "deployed" : "stowed"));
            if (reseedDown && !reseeded) telemetry.addData("Reseed pose", "keep holding LB + RB... %.1f s", RESEED_HOLD_SEC - reseedHold.seconds());
            telemetry.addData("Marks", marks.size());
            for (int i = Math.max(0, marks.size() - SHOWN_MARKS); i < marks.size(); i++) telemetry.addLine(marks.get(i));
            telemetry.addData("Checks", checks);
            telemetry.addLine("A = MARK this pose, B = CHECK mark (a spot whose real position you know), X = deploy ramp, Y = stow ramp, hold LB + RB = reseed to the start, right trigger = precision");
            telemetry.update();
        }
        drive.stop();
    }
}
