package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.LaunchMap;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Builds the launch map on the real robot. Drive to a distance from the open CELL, set the flywheel speed, launch real elements until they
 * score, and record the pair. The recorded points are interpolated live (so you can test the map as you go) and printed as lines to
 * paste into LaunchMap.TABLE (and written to the log). It runs the real mechanisms (the {@link Mechanisms} coordinator), so a launch is
 * the same as in the match: the flywheel must be ready, the pusher turns, the intake and the element counter work, and the pusher check runs
 * after a salvo.
 *
 * Gamepad 1:
 *   Left stick / right stick X   drive / turn (robot oriented)      Right bumper (hold): precision (30%)
 *   Right trigger                intake on / off (toggle; stops itself at 4 elements)      Left trigger (hold): outtake
 *   X                            flywheel on / off
 *   D-pad up / down              setpoint +/- STEP ticks/s (ignored while auto speed is on)
 *   Y                            LAUNCH a salvo (spins up if needed, launches when the flywheel is ready)
 *   B                            record (distance, setpoint) as a point (replaces a point within 1 in)
 *   A                            remove the most recently recorded point
 *   D-pad left / right           cycle the target CELL (blue rear, blue audience, red rear, red audience)
 *   Left bumper                  auto speed on / off: the setpoint follows the recorded points
 *   Back                         clear a pusher fault; HOLD it for 1 s to reseed the pose to the start pose below
 *
 * Start pose: START_X_IN, START_Y_IN, START_HEADING_DEG (field coordinates: X toward the audience wall, Y toward the Blue Wall, heading 0 = facing
 * +X, 180 = facing the rear wall). The alliance does not move it. The target starts as the CELL nearest the start pose. INIT (gamepad 1): D-pad
 * right = BLUE, D-pad left = RED, which sets which nectar counts as the opponent's.
 *
 * Distance comes from the Pinpoint pose to the target (set the start pose to where you place the robot), or set MANUAL_DISTANCE_IN above
 * 0 in the Dashboard to type a measured distance instead. The hood is fixed, so speed depends on distance only. Same speed for pollen and nectar.
 * The pusher is assumed to be where the last opmode left it (the saved zero); the start-up pusher check verifies it after Start if
 * the track is empty.
 */
@Config
@TeleOp(name = "Launch Map Tuner", group = "Bring-up")
public class LaunchMapTuner extends LinearOpMode {

    /**
     * The start pose for the 2026-11-01 launch map session: against the AUDIENCE wall (X = 72 less half the 17 in robot depth), Y = -16.5, facing the
     * rear wall (180 deg). The HIVE centerlines are Y = +12.75 (blue) and -12.75 (red). The same for either alliance.
     */
    public static double START_X_IN = 63.4;   // FieldGeometry.HALF_IN (71) - the origin's 7.6 in from the back edge (back against the audience wall, facing -X)
    public static double START_Y_IN = -16.5;
    public static double START_HEADING_DEG = 180.0;
    public static double MANUAL_DISTANCE_IN = 0.0;
    public static double STEP = 20.0;   // the flywheel's velocity reads in steps of 20 ticks/s
    public static double START_SETPOINT = 1000.0;
    public static double TRIGGER_THRESHOLD = 0.3;

    public static double RESEED_HOLD_SEC = 1.0;
    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;

    /** What the log's sample rows show: where the robot is, what it aims at, and the setpoint. */
    private double logX, logY, logHeadingDeg, logDistance, logSetpoint;
    private String logTarget = "";

    private static Pose2d startPose() {
        return new Pose2d(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
    }

    /** The CELL closest to a pose, the starting target. */
    private static HiveTarget nearestCell(Pose2d pose) {
        HiveTarget best = HiveTarget.BLUE_REAR;
        for (HiveTarget t : HiveTarget.values()) {
            if (t.distanceFrom(pose) < best.distanceFrom(pose)) best = t;
        }
        return best;
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Launch Map Tuner", hardwareMap);
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose());
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);
        mechanisms.setUseLaunchMap(false);            // the tuner sets the speed itself
        mechanisms.setAutoStopAfterSalvo(false);    // the flywheel stays on between launches until X

        boolean blue = true;
        HiveTarget target = nearestCell(startPose());
        boolean spinning = false, autoSpeed = false, prevIntakeTrigger = false;
        double setpoint = START_SETPOINT;
        mechanisms.setManualSetpoint(setpoint);
        List<double[]> points = new ArrayList<>();
        double[] lastRecorded = null;

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addData("Mechanisms present", mechanisms.describe());
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right) blue = true;
            if (gamepad1.dpad_left) blue = false;
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();   // reads only; nothing moves in INIT
            Pose2d p = drive.getPose();
            Pose2d want = startPose();
            boolean poseOk = Math.hypot(p.position.x - want.position.x, p.position.y - want.position.y) < 1.5;
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Alliance (D-pad right = BLUE, left = RED)", blue ? "BLUE" : "RED");
            telemetry.addData("Start pose to use", "X %.1f, Y %.2f in, heading %.0f deg (START_X_IN, START_Y_IN, START_HEADING_DEG in the Dashboard)",
                    want.position.x, want.position.y, Math.toDegrees(want.heading.toDouble()));
            telemetry.addData("Pinpoint pose", "%.1f, %.1f, %.0f deg   %s", p.position.x, p.position.y, p.heading.toDegrees(), poseOk ? "OK" : "WARNING: not at the start pose");
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addLine("Keep still while the Pinpoint calibrates; the pusher is assumed at its saved zero. Then press Start.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        mechanisms.setBlueAlliance(blue);
        RunLog.started();
        RunLog.event("SETUP " + (blue ? "BLUE" : "RED") + " start, " + mechanisms.describe());
        RunLog.setSampler("x,y,hdg_deg,target,distance,setpoint," + Mechanisms.SAMPLE_COLUMNS, 200,
                () -> String.format(Locale.US, "%.1f,%.1f,%.1f,%s,%.1f,%.0f,%s", logX, logY, logHeadingDeg, logTarget, logDistance, logSetpoint,
                        mechanisms.sampleRow()));
        boolean backDown = false, reseeded = false;
        ElapsedTime backHold = new ElapsedTime();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();

            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale),
                    -gamepad1.right_stick_x * TURN_SCALE * scale));

            if (gamepad1.backWasPressed()) mechanisms.clearPusherStall();
            if (gamepad1.back) {
                if (!backDown) {
                    backDown = true;
                    reseeded = false;
                    backHold.reset();
                }
                if (!reseeded && backHold.seconds() >= RESEED_HOLD_SEC) {
                    drive.setPose(startPose());
                    reseeded = true;
                    RunLog.event("POSE reseeded to the start pose");
                }
            } else {
                backDown = false;
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
            if (gamepad1.yWasPressed()) {
                RunLog.event(String.format(Locale.US, "LAUNCH pressed: pose %.1f, %.1f, %.0f deg, %s at %.1f in%s, setpoint %.0f",
                        pose.position.x, pose.position.y, pose.heading.toDegrees(), target,
                        MANUAL_DISTANCE_IN > 0 ? MANUAL_DISTANCE_IN : target.distanceFrom(pose), MANUAL_DISTANCE_IN > 0 ? " (typed)" : "", setpoint));
                mechanisms.requestLaunch();
            }

            double distance = MANUAL_DISTANCE_IN > 0 ? MANUAL_DISTANCE_IN : target.distanceFrom(pose);

            if (gamepad1.bWasPressed()) {
                points.removeIf(p -> Math.abs(p[0] - distance) < 1.0);
                lastRecorded = new double[]{distance, setpoint};
                points.add(lastRecorded);
                points.sort((a, b) -> Double.compare(a[0], b[0]));
                RunLog.event(String.format(Locale.US, "RECORD %.1f in to %s at %.0f ticks/s (%s; pose %.1f, %.1f, %.0f deg)", distance, target, setpoint,
                        MANUAL_DISTANCE_IN > 0 ? "distance TYPED" : "distance from the pose", pose.position.x, pose.position.y, pose.heading.toDegrees()));
            }
            if (gamepad1.aWasPressed() && lastRecorded != null) {
                points.remove(lastRecorded);
                RunLog.event(String.format(Locale.US, "UNDO the point %.1f in at %.0f ticks/s", lastRecorded[0], lastRecorded[1]));
                lastRecorded = null;
            }

            double mapSpeed = LaunchMap.interpolate(points, distance);
            if (autoSpeed && !Double.isNaN(mapSpeed)) setpoint = mapSpeed;

            logX = pose.position.x;
            logY = pose.position.y;
            logHeadingDeg = pose.heading.toDegrees();
            logDistance = distance;
            logSetpoint = setpoint;
            logTarget = target.name();
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
            telemetry.addData("Saved LaunchMap.TABLE", LaunchMap.isEmpty() ? "empty" : LaunchMap.TABLE.length + " points");
            mechanisms.log(telemetry);
            telemetry.addLine("--- paste into LaunchMap.TABLE ---");
            for (double[] p : points) {
                telemetry.addLine(String.format("        {%.1f, %.0f},", p[0], p[1]));
            }
            telemetry.addLine("RT intake, Y LAUNCH, X spin, B record, A undo, D-pad up/down speed, D-pad left/right target, LB auto speed, Back clear fault (hold = reseed pose)");
            telemetry.update();
        }
        StringBuilder table = new StringBuilder("LAUNCH MAP at the end of the run, paste into LaunchMap.TABLE:");
        for (double[] p : points) table.append(String.format(Locale.US, "\n        {%.1f, %.0f},", p[0], p[1]));
        RunLog.event(table.toString().replace('\n', ' '));
        drive.stop();
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
