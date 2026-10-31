package org.firstinspires.ftc.teamcode.opmodes.bringup;

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
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.ShotMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds the shot map on the real robot. Drive to a distance from the open CELL, set the flywheel speed
 * until shots score, and record the pair. The recorded points are interpolated live (so you can test
 * the map as you go) and printed as lines to paste into ShotMap.TABLE.
 *
 *   Left stick / right stick X   drive / turn (robot oriented)      Right bumper: precision (30%)
 *   X                            flywheel on / off
 *   D-pad up / down              setpoint +/- STEP ticks/s (ignored while auto speed is on)
 *   Y                            cycle the target CELL (blue rear, blue audience, red rear, red audience)
 *   B                            record (distance, setpoint) as a point (replaces a point within 1 in)
 *   A                            remove the most recently recorded point
 *   Left bumper                  auto speed on / off: the setpoint follows the recorded points
 *   Back                      reseed the pose to the start pose below
 *
 * Distance comes from the Pinpoint pose to the target (set the start pose to where you place the robot),
 * or set MANUAL_DISTANCE_IN above 0 in the Dashboard to type a measured distance instead.
 * The hood is fixed, so speed depends on distance only. Same speed for pollen and nectar.
 */
@Config
@TeleOp(name = "Shot Map Tuner", group = "Bring-up")
public class ShotMapTuner extends LinearOpMode {

    private static final double FIELD_HALF_IN = 72.0;
    public static double START_Y_IN = 12.75;   // X comes from the profile's depth (touching the rear wall)
    public static double START_HEADING_DEG = 0.0;
    public static double MANUAL_DISTANCE_IN = 0.0;
    public static double STEP = 25.0;

    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        double startX = -(FIELD_HALF_IN - profile.robotDepthIn / 2.0);
        Pose2d startPose = new Pose2d(startX, START_Y_IN, Math.toRadians(START_HEADING_DEG));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Flywheel flywheel = new Flywheel(hardwareMap);

        HiveTarget target = HiveTarget.BLUE_REAR;
        boolean spinning = false, autoSpeed = false;
        double setpoint = 400.0;
        List<double[]> points = new ArrayList<>();
        double[] lastRecorded = null;

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Shot Map Tuner. Keep still while the Pinpoint calibrates.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();

            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale),
                    -gamepad1.right_stick_x * TURN_SCALE * scale));

            if (gamepad1.backWasPressed()) {
                drive.setPose(new Pose2d(startX, START_Y_IN, Math.toRadians(START_HEADING_DEG)));
            }
            if (gamepad1.xWasPressed()) spinning = !spinning;
            if (gamepad1.yWasPressed()) target = target.next();
            if (gamepad1.leftBumperWasPressed()) autoSpeed = !autoSpeed;
            if (!autoSpeed) {
                if (gamepad1.dpadUpWasPressed()) setpoint += STEP;
                if (gamepad1.dpadDownWasPressed()) setpoint = Math.max(0.0, setpoint - STEP);
            }

            double distance = MANUAL_DISTANCE_IN > 0 ? MANUAL_DISTANCE_IN : target.distanceFrom(pose);

            if (gamepad1.bWasPressed()) {
                points.removeIf(p -> Math.abs(p[0] - distance) < 1.0);
                lastRecorded = new double[]{distance, setpoint};
                points.add(lastRecorded);
                points.sort((a, b) -> Double.compare(a[0], b[0]));
            }
            if (gamepad1.aWasPressed() && lastRecorded != null) {
                points.remove(lastRecorded);
                lastRecorded = null;
            }

            double mapSpeed = ShotMap.interpolate(points, distance);
            if (autoSpeed && !Double.isNaN(mapSpeed)) setpoint = mapSpeed;

            flywheel.setTargetVelocity(spinning ? setpoint : 0.0);
            flywheel.update();

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Target CELL", target);
            telemetry.addData("Distance (in)", "%.1f   %s", distance, MANUAL_DISTANCE_IN > 0 ? "(manual)" : "(from pose)");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, pose.heading.toDegrees());
            telemetry.addData("Setpoint (ticks/s)", "%.0f   %s", setpoint, autoSpeed ? "(auto, from points)" : "(manual)");
            telemetry.addData("Recorded-points speed here", Double.isNaN(mapSpeed) ? "no points yet" : String.format("%.0f", mapSpeed));
            telemetry.addData("Saved ShotMap.TABLE", ShotMap.isEmpty() ? "empty" : ShotMap.TABLE.length + " points");
            flywheel.log(telemetry);
            telemetry.addLine("--- paste into ShotMap.TABLE ---");
            for (double[] p : points) {
                telemetry.addLine(String.format("        {%.1f, %.0f},", p[0], p[1]));
            }
            telemetry.addLine("X spin, Y target, B record, A undo, LB auto speed, D-pad speed, Back reseed pose");
            telemetry.update();
        }
        drive.stop();
        flywheel.stop();
    }
}
