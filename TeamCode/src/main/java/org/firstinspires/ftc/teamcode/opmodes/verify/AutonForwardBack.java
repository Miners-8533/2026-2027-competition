package org.firstinspires.ftc.teamcode.opmodes.verify;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.motion.SleepAction;

/**
 * Path-following check: drive straight forward DISTANCE_IN, pause PAUSE_SEC, drive back to the
 * start pose, done. The start pose is (0, 0, 0) wherever you place the robot, heading held at 0.
 *
 * Mark the robot's start position on the floor with tape (and the 48 in point if you like).
 * The drive motors have no encoders, so the Pinpoint is the only sensor: at the end, compare
 * how far the robot really is from the tape mark to the pose error shown here. If the Pinpoint
 * says ~0 but the robot is off, the pod scale or offsets are wrong. If both agree, the gains
 * are what limit the accuracy.
 *
 * Needs about 4 ft of clear floor ahead. DriveToPose gives up on a leg after its TIMEOUT_SEC.
 */
@Config
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@Autonomous(name = "Auton Verify - Forward 48 and back", group = "Auto verify")
public class AutonForwardBack extends LinearOpMode {

    public static double DISTANCE_IN = 48.0;
    public static double PAUSE_SEC = 5.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d start = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        telemetry.addLine("Mark the start position with tape. Keep still while the Pinpoint calibrates.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Pose2d far = new Pose2d(DISTANCE_IN, 0, 0);
        Actions.runBlocking(this, new SequentialAction(
                new DriveToPose(drive, far),
                new SleepAction(PAUSE_SEC),
                new DriveToPose(drive, start)));

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            telemetry.addLine("Done. Measure the robot's real distance from the tape mark.");
            telemetry.addData("Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f",
                    p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.addData("Error from start (in, in, deg)", "%.2f, %.2f, %.1f",
                    p.position.x - start.position.x, p.position.y - start.position.y,
                    Math.toDegrees(p.heading.toDouble() - start.heading.toDouble()));
            telemetry.update();
        }
    }
}
