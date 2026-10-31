package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.TransportGate;

/**
 * The competition TeleOp: a driver (gamepad 1) drives the chassis FIELD-oriented and an operator (gamepad 2) runs
 * the mechanisms through the {@link Mechanisms} coordinator.
 *
 * Start pose: if an auto ran recently (see AUTO_POSE_MAX_AGE_SEC) the TeleOp starts from the pose and alliance the
 * auto saved in {@link PoseStorage}; otherwise from the alliance's default start pose. The INIT screen says which.
 * It also reports whether both gamepads are assigned (Start + A on the driver's controller, Start + B on the
 * operator's), because a gamepad that is not assigned silently does nothing.
 *
 * INIT (gamepad 1):
 *   D-pad right / left   BLUE / RED alliance (only when there is no auto pose; an auto pose fixes the alliance)
 *   Back                 discard the auto pose and use the default start pose (then the D-pad picks the alliance)
 *
 * Gamepad 1 (driver):
 *   left stick           drive, field-oriented: up = away from the driver, right = the driver's right
 *   right stick X        turn
 *   right bumper (hold)  precision mode (30%)
 *   Y                    reset the heading: point the robot AWAY from you first, then press
 *   Back (hold 1 s)      reseed the pose to the default start pose (held, so it cannot happen by accident)
 *
 * Gamepad 2 (operator):
 *   right trigger        intake on / off (a toggle)
 *   left trigger (hold)  outtake
 *   A                    fire a salvo (spins up and fires when ready)
 *   B                    reject (gate reverse + outtake)
 *   X / Y                deploy / stow the depollinator ramp (stow waits until clear of a flower)
 *   right bumper         flywheel on / off (toggle). It runs by default, from Start, and keeps running after salvos
 *   left bumper          flywheel stop (the right bumper turns it back on)
 *   D-pad up / down      manual flywheel setpoint +/- 25 ticks/s (used while the shot map is empty)
 *   D-pad left or right  the open CELL is the HIVE's other side (after a tip)
 *   Back                 clear a gate fault
 *
 * "Away from the driver" is a field direction fixed by the alliance: drivers stand behind their alliance wall, so
 * blue faces -Y (-90 deg) and red faces +Y (+90 deg). If a stick direction is wrong at the field, correct
 * BLUE_DRIVER_FORWARD_DEG / RED_DRIVER_FORWARD_DEG in the Dashboard.
 */
@Config
@TeleOp(name = "TeleOp - Competition", group = "Competition")
public class TeleOpCompetition extends LinearOpMode {

    public static double START_Y_IN = 12.75;       // blue; red mirrors to the negative. X comes from the profile's depth
    public static double START_HEADING_DEG = 0.0;
    /** Field heading (deg) of the direction each driver faces, i.e. "stick up". */
    public static double BLUE_DRIVER_FORWARD_DEG = -90.0;
    public static double RED_DRIVER_FORWARD_DEG = 90.0;
    /** An auto pose older than this is ignored (a leftover from an earlier run). */
    public static double AUTO_POSE_MAX_AGE_SEC = 300.0;
    public static double RESEED_HOLD_SEC = 1.0;
    public static double TURN_SCALE = 0.5;
    public static double PRECISION_SCALE = 0.30;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double SETPOINT_STEP = 25.0;

    private static final double FIELD_HALF_IN = 72.0;

    private static boolean assigned(Gamepad gamepad) {
        return gamepad.getGamepadId() != Gamepad.ID_UNASSOCIATED;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);

        boolean useAutoPose = PoseStorage.isFresh(AUTO_POSE_MAX_AGE_SEC);
        boolean blue = useAutoPose ? PoseStorage.isBlue() : true;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.backWasPressed() && useAutoPose) {
                useAutoPose = false;
                PoseStorage.clear();
                blue = true;
            }
            if (!useAutoPose) {
                if (gamepad1.dpad_right) blue = true;
                if (gamepad1.dpad_left) blue = false;
            }
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Start pose", useAutoPose
                    ? String.format("AUTO pose, saved %.0f s ago (Back on gamepad 1 discards it)", PoseStorage.ageSec())
                    : "default start pose (rear wall)");
            telemetry.addData("Gamepad 1 (driver)", assigned(gamepad1) ? "assigned" : "NOT ASSIGNED: press Start + A on it");
            telemetry.addData("Gamepad 2 (operator)", assigned(gamepad2) ? "assigned" : "NOT ASSIGNED: press Start + B on it");
            telemetry.addLine(useAutoPose ? "Press Start on the Driver Station when ready."
                    : "Gamepad 1: D-pad right = BLUE, D-pad left = RED, then press Start on the Driver Station.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        double startX = -(FIELD_HALF_IN - profile.robotDepthIn / 2.0);
        Pose2d defaultStart = new Pose2d(startX, blue ? START_Y_IN : -START_Y_IN, Math.toRadians(START_HEADING_DEG));
        Pose2d startPose = useAutoPose ? PoseStorage.pose() : defaultStart;
        double driverForwardRad = Math.toRadians(blue ? BLUE_DRIVER_FORWARD_DEG : RED_DRIVER_FORWARD_DEG);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, TransportGate.Reference.CONTINUE);
        mechanisms.setOpenCell(blue ? HiveTarget.BLUE_REAR : HiveTarget.RED_AUDIENCE);
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel runs the whole time ...
        mechanisms.setSpinRequested(true);         // ... from the start, until the operator stops it (left bumper)

        boolean prevIntakeTrigger = false;
        boolean backDown = false, reseeded = false;
        ElapsedTime backHold = new ElapsedTime();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            PoseStorage.save(pose, blue);
            double heading = pose.heading.toDouble();

            // driver: stick up = away from the driver, stick right = the driver's right, both in field terms
            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
            double up = -gamepad1.left_stick_y * scale;
            double right = gamepad1.left_stick_x * scale;
            double fieldX = up * Math.cos(driverForwardRad) + right * Math.cos(driverForwardRad - Math.PI / 2.0);
            double fieldY = up * Math.sin(driverForwardRad) + right * Math.sin(driverForwardRad - Math.PI / 2.0);
            double robotForward = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
            double robotLeft = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotForward, robotLeft),
                    -gamepad1.right_stick_x * TURN_SCALE * scale));

            if (gamepad1.yWasPressed()) drive.setPose(new Pose2d(pose.position, driverForwardRad));
            if (gamepad1.back) {
                if (!backDown) {
                    backDown = true;
                    reseeded = false;
                    backHold.reset();
                }
                if (!reseeded && backHold.seconds() >= RESEED_HOLD_SEC) {
                    drive.setPose(defaultStart);
                    reseeded = true;
                }
            } else {
                backDown = false;
            }

            // operator
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
            if (gamepad2.backWasPressed()) mechanisms.clearGateStall();

            mechanisms.update(pose);

            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, pose.heading.toDegrees());
            if (gamepad1.back && !reseeded) {
                telemetry.addData("Reseed pose", "keep holding Back... %.1f s", RESEED_HOLD_SEC - backHold.seconds());
            }
            mechanisms.log(telemetry);
            telemetry.addLine("GP1: stick drive (field), RB precision, Y reset heading (face away first), Back held = reseed");
            telemetry.addLine("GP2: RT intake on/off, LT outtake (hold), A fire, B reject, X deploy, Y stow, RB/LB spin/stop, D-pad speed / cell, Back clear fault");
            telemetry.update();
        }
        drive.stop();
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
