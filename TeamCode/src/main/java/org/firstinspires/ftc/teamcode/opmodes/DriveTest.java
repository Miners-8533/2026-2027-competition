package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;

/**
 * Bring-up test for the new {@code drive} package. Robot-oriented mecanum drive from
 * gamepad 1, with pose + velocity from the Pinpoint shown on telemetry / FTC Dashboard.
 *
 * Checks:
 *   - all four motors spin the right way (forward stick = robot forward, etc.)
 *   - Pinpoint reports READY and the pose updates as you push the robot / drive it
 *   - heading increases when the robot turns counter-clockwise
 *
 * Controls:
 *   left stick   - translate (forward / strafe)
 *   right stick X - turn
 *   right bumper - hold for precision (30% power)
 *   options      - reset pose to (0, 0, 0)
 */
@TeleOp(name = "Drive Test - Mecanum", group = "Drivetrain")
public class DriveTest extends LinearOpMode {

    private static final double TURN_SCALE = 0.5;      // matches 2025 feel
    private static final double PRECISION_SCALE = 0.30;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Initialized. Pinpoint calibrating - keep the robot still.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;

            double forward = -gamepad1.left_stick_y * scale;
            double strafe = -gamepad1.left_stick_x * scale;
            double turn = -gamepad1.right_stick_x * TURN_SCALE * scale;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), turn));

            if (gamepad1.optionsWasPressed()) {
                drive.setPose(new Pose2d(0, 0, 0));
            }

            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();

            telemetry.addData("status", ((PinpointLocalizer) drive.localizer).getDeviceStatus());
            telemetry.addData("x (in)", "%.2f", pose.position.x);
            telemetry.addData("y (in)", "%.2f", pose.position.y);
            telemetry.addData("heading (deg)", "%.1f", pose.heading.toDegrees());
            telemetry.addData("vel (in/s)", "%.1f, %.1f", vel.linearVel.x, vel.linearVel.y);
            telemetry.addData("omega (deg/s)", "%.1f", Math.toDegrees(vel.angVel));
            telemetry.addData("cmd", "fwd %.2f  str %.2f  turn %.2f", forward, strafe, turn);
            telemetry.update();
        }
    }
}
