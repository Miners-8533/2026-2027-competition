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
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.ShotMap;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Builds the shot map on the real robot. Drive to a distance from the open CELL, set the flywheel speed, fire real elements until they
 * score, and record the pair. The recorded points are interpolated live (so you can test the map as you go) and printed as lines to
 * paste into ShotMap.TABLE (and written to the log). It runs the real mechanisms (the {@link Mechanisms} coordinator), so a shot is
 * the same as in the match: the flywheel must be ready, the pusher turns, the intake and the element counter work, and the pusher check runs
 * after a salvo.
 *
 * Gamepad 1:
 *   Left stick / right stick X   drive / turn (robot oriented)      Right bumper (hold): precision (30%)
 *   Right trigger                intake on / off (toggle; stops itself at 4 elements)      Left trigger (hold): outtake
 *   X                            flywheel on / off
 *   D-pad up / down              setpoint +/- STEP ticks/s (ignored while auto speed is on)
 *   Y                            FIRE a salvo (spins up if needed, fires when the flywheel is ready)
 *   B                            record (distance, setpoint) as a point (replaces a point within 1 in)
 *   A                            remove the most recently recorded point
 *   D-pad left / right           cycle the target CELL (blue rear, blue audience, red rear, red audience)
 *   Left bumper                  auto speed on / off: the setpoint follows the recorded points
 *   Back                         reseed the pose to the start pose below (also clears a pusher fault)
 *
 * Distance comes from the Pinpoint pose to the target (set the start pose to where you place the robot), or set MANUAL_DISTANCE_IN above
 * 0 in the Dashboard to type a measured distance instead. The hood is fixed, so speed depends on distance only. Same speed for pollen and nectar.
 * The pusher is assumed to be where the last opmode left it (the saved zero); the start-up pusher check verifies it after Start if
 * the track is empty.
 */
@Config
@TeleOp(name = "Shot Map Tuner", group = "Bring-up")
public class ShotMapTuner extends LinearOpMode {

    private static final double FIELD_HALF_IN = 72.0;
    public static double START_Y_IN = 12.75;   // X comes from the profile's depth (touching the rear wall)
    public static double START_HEADING_DEG = 0.0;
    public static double MANUAL_DISTANCE_IN = 0.0;
    public static double STEP = 25.0;
    public static double START_SETPOINT = 1000.0;
    public static double TRIGGER_THRESHOLD = 0.3;

    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Shot Map Tuner", hardwareMap);
        try {
            runTuner();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runTuner() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        double startX = -(FIELD_HALF_IN - profile.robotDepthIn / 2.0);
        Pose2d startPose = new Pose2d(startX, START_Y_IN, Math.toRadians(START_HEADING_DEG));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);
        mechanisms.setUseShotMap(false);            // the tuner sets the speed itself
        mechanisms.setAutoStopAfterSalvo(false);    // the flywheel stays on between shots until X
        mechanisms.setBlueAlliance(true);

        HiveTarget target = HiveTarget.BLUE_REAR;
        boolean spinning = false, autoSpeed = false, prevIntakeTrigger = false;
        double setpoint = START_SETPOINT;
        mechanisms.setManualSetpoint(setpoint);
        List<double[]> points = new ArrayList<>();
        double[] lastRecorded = null;

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addData("Mechanisms present", mechanisms.describe());
        telemetry.addLine("Shot Map Tuner. Keep still while the Pinpoint calibrates; the pusher is assumed at its saved zero.");
        telemetry.update();
        waitForStart();
        RunLog.started();
        RunLog.event("SETUP " + mechanisms.describe());

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();

            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale),
                    -gamepad1.right_stick_x * TURN_SCALE * scale));

            if (gamepad1.backWasPressed()) {
                drive.setPose(new Pose2d(startX, START_Y_IN, Math.toRadians(START_HEADING_DEG)));
                mechanisms.clearPusherStall();
            }
            if (gamepad1.xWasPressed()) spinning = !spinning;
            if (gamepad1.dpadRightWasPressed()) {
                target = target.next();
                RunLog.event("TARGET " + target);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                HiveTarget[] all = HiveTarget.values();
                target = all[(target.ordinal() + all.length - 1) % all.length];
                RunLog.event("TARGET " + target);
            }
            if (gamepad1.leftBumperWasPressed()) autoSpeed = !autoSpeed;
            if (!autoSpeed) {
                if (gamepad1.dpadUpWasPressed()) setpoint += STEP;
                if (gamepad1.dpadDownWasPressed()) setpoint = Math.max(0.0, setpoint - STEP);
            }

            boolean intakeTrigger = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            if (intakeTrigger && !prevIntakeTrigger) mechanisms.toggleIntake();
            prevIntakeTrigger = intakeTrigger;
            mechanisms.setManualOuttake(gamepad1.left_trigger > TRIGGER_THRESHOLD);
            if (gamepad1.yWasPressed()) mechanisms.requestFire();

            double distance = MANUAL_DISTANCE_IN > 0 ? MANUAL_DISTANCE_IN : target.distanceFrom(pose);

            if (gamepad1.bWasPressed()) {
                points.removeIf(p -> Math.abs(p[0] - distance) < 1.0);
                lastRecorded = new double[]{distance, setpoint};
                points.add(lastRecorded);
                points.sort((a, b) -> Double.compare(a[0], b[0]));
                RunLog.event(String.format(Locale.US, "RECORD %.1f in to %s at %.0f ticks/s", distance, target, setpoint));
            }
            if (gamepad1.aWasPressed() && lastRecorded != null) {
                points.remove(lastRecorded);
                RunLog.event(String.format(Locale.US, "UNDO the point %.1f in at %.0f ticks/s", lastRecorded[0], lastRecorded[1]));
                lastRecorded = null;
            }

            double mapSpeed = ShotMap.interpolate(points, distance);
            if (autoSpeed && !Double.isNaN(mapSpeed)) setpoint = mapSpeed;

            mechanisms.setOpenCell(target);
            mechanisms.setManualSetpoint(setpoint);
            mechanisms.setSpinRequested(spinning);
            mechanisms.update(pose);
            RunLog.tick();

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Target CELL", target);
            telemetry.addData("Distance (in)", "%.1f   %s", distance, MANUAL_DISTANCE_IN > 0 ? "(manual)" : "(from pose)");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, pose.heading.toDegrees());
            telemetry.addData("Setpoint (ticks/s)", "%.0f   %s", setpoint, autoSpeed ? "(auto, from points)" : "(manual)");
            telemetry.addData("Recorded-points speed here", Double.isNaN(mapSpeed) ? "no points yet" : String.format("%.0f", mapSpeed));
            telemetry.addData("Saved ShotMap.TABLE", ShotMap.isEmpty() ? "empty" : ShotMap.TABLE.length + " points");
            mechanisms.log(telemetry);
            telemetry.addLine("--- paste into ShotMap.TABLE ---");
            for (double[] p : points) {
                telemetry.addLine(String.format("        {%.1f, %.0f},", p[0], p[1]));
            }
            telemetry.addLine("RT intake, Y FIRE, X spin, B record, A undo, D-pad up/down speed, D-pad left/right target, LB auto speed, Back reseed pose");
            telemetry.update();
        }
        StringBuilder table = new StringBuilder("SHOT MAP at the end of the run, paste into ShotMap.TABLE:");
        for (double[] p : points) table.append(String.format(Locale.US, "\n        {%.1f, %.0f},", p[0], p[1]));
        RunLog.event(table.toString().replace('\n', ' '));
        drive.stop();
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
