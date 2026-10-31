package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;

/**
 * Draft integrated TeleOp: a driver (gamepad 1) drives the chassis and an operator (gamepad 2) runs the
 * mechanisms through the {@link Mechanisms} coordinator, which enforces the interlocks in
 * design/mechanisms.md. Mechanisms the robot does not have yet are skipped.
 *
 * Alliance: in INIT press D-pad right = BLUE (default) or D-pad left = RED on gamepad 1. This sets the
 * start pose and which HIVE's cells the flywheel aims at.
 *
 * Gamepad 1 (driver):
 *   left stick / right stick X   drive / turn (robot oriented)     right bumper: precision (30%)
 *   Back                         reseed the pose to the start pose
 *
 * Gamepad 2 (operator):
 *   right trigger (press)        intake on / off (a toggle; not gated on the pusher's position)
 *   left trigger (hold)          outtake
 *   A                            fire a salvo (spins up and fires when ready)
 *   B                            reject (pusher reverse + outtake)
 *   X / Y                        deploy / stow the ramp (stow waits until clear of a flower)
 *   right bumper / left bumper   flywheel spin up / stop
 *   D-pad up / down              manual flywheel setpoint +/- 25 ticks/s (used while the shot map is empty)
 *   D-pad left or right          the open CELL is the HIVE's other side (after a tip)
 *   Back                         clear a pusher stall
 *
 * Not yet done: the pose is not carried over from an auto (it starts at the start pose below), and the
 * chassis is robot oriented, not field oriented.
 */
@Config
@TeleOp(name = "TeleOp - Draft (mechanisms)", group = "TeleOp")
public class TeleOpDraft extends LinearOpMode {

    private static final double FIELD_HALF_IN = 72.0;
    public static double START_Y_IN = 12.75;       // blue; red mirrors to the negative. X comes from the profile's depth (touching the rear wall)
    public static double START_HEADING_DEG = 0.0;

    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;
    private static final double TRIGGER_THRESHOLD = 0.3;
    private static final double SETPOINT_STEP = 25.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);

        boolean blue = true;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right) blue = true;
            if (gamepad1.dpad_left) blue = false;
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addLine("Gamepad 1: D-pad right = BLUE, D-pad left = RED, then press Start.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        double startX = -(FIELD_HALF_IN - profile.robotDepthIn / 2.0);
        Pose2d startPose = new Pose2d(startX, blue ? START_Y_IN : -START_Y_IN, Math.toRadians(START_HEADING_DEG));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);
        mechanisms.setOpenCell(blue ? HiveTarget.BLUE_REAR : HiveTarget.RED_AUDIENCE);
        mechanisms.setBlueAlliance(blue);
        boolean prevIntakeTrigger = false;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();

            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale),
                    -gamepad1.right_stick_x * TURN_SCALE * scale));
            if (gamepad1.backWasPressed()) drive.setPose(startPose);

            boolean intakeTrigger = gamepad2.right_trigger > TRIGGER_THRESHOLD;
            if (intakeTrigger && !prevIntakeTrigger) mechanisms.toggleIntake();
            prevIntakeTrigger = intakeTrigger;
            mechanisms.setManualOuttake(gamepad2.left_trigger > TRIGGER_THRESHOLD);
            if (gamepad2.aWasPressed()) mechanisms.requestFire();
            if (gamepad2.bWasPressed()) mechanisms.requestReject();
            if (gamepad2.xWasPressed()) mechanisms.requestDeploy();
            if (gamepad2.yWasPressed()) mechanisms.requestStow();
            if (gamepad2.rightBumperWasPressed()) mechanisms.toggleSpin();
            if (gamepad2.leftBumperWasPressed()) mechanisms.stopFlywheel();
            if (gamepad2.dpadUpWasPressed()) mechanisms.adjustManualSetpoint(SETPOINT_STEP);
            if (gamepad2.dpadDownWasPressed()) mechanisms.adjustManualSetpoint(-SETPOINT_STEP);
            if (gamepad2.dpadLeftWasPressed() || gamepad2.dpadRightWasPressed()) {
                mechanisms.setOpenCell(mechanisms.getOpenCell().otherSide());
            }
            if (gamepad2.backWasPressed()) mechanisms.clearPusherStall();

            mechanisms.update(pose);

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, pose.heading.toDegrees());
            mechanisms.log(telemetry);
            telemetry.addLine("GP2: RT intake on/off, LT outtake (hold), A fire, B reject, X deploy, Y stow, RB/LB spin/stop, D-pad speed / cell");
            telemetry.update();
        }
        drive.stop();
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
