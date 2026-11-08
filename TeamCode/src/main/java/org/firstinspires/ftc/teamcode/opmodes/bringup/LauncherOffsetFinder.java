package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Finds where the launcher really points, so the launcher offset (and any hive or heading offset) can be measured instead of inferred from missed launches.
 * Target lock is OFF: the driver drives the robot (field-oriented, as in the TeleOp), lines the launch up with the centerline of a HIVE cell by hand, and launches;
 * a mark on a good salvo writes the robot's pose at that launch, the cell, and what the pose says about the launcher. Do it at each cell, and from the blue and the
 * red start poses (both headings) to see whether the effect is symmetric.
 *
 * What a mark records (the pose at the last LAUNCH, since the robot may have moved by the time the mark is pressed): the pose, the cell the robot was
 * nearest, the distance, the bearing, and how far to the LEFT of the robot's front line the cell's center sits (distance * sin(bearing - heading)). If the salvo
 * went into the cell with the launcher's line through its center, that number is the launcher's sideways offset from the pose point (positive = the launcher is
 * left of the origin; TargetLock.LAUNCHER_OFFSET_LEFT_IN is the value in use) plus the distance times any angle error of the launch. At one distance the two cannot be
 * told apart; marks at two distances along the same line can (the offset is the same at both, the angle error grows with the distance).
 *
 * INIT (gamepad 1): D-pad right = BLUE start (back against the rear wall facing +X: (-HALF + 7.6, +16)), D-pad left = RED start (back against the audience wall
 * facing -X: (+HALF - 7.6, -16)), the same default start poses as TeleOp - Competition (FieldGeometry.HALF_IN is the wall distance). Place the robot there, still.
 *
 * Gamepad 1 (one driver does everything; neither Start nor Back is used):
 *   left stick / right stick X   drive, field-oriented (stick up = away from the driver of the chosen start's alliance, as in the TeleOp) / turn
 *   left bumper (hold)           precision drive (30%), for lining up
 *   right trigger (hold)         intake IN, even when full (the intake also runs IN by itself until the robot holds 4, like the TeleOp)
 *   left trigger (hold)          intake OUT
 *   A                            launch a salvo
 *   Y                            MARK a good salvo
 *   D-pad left / right           MARK a salvo that landed LEFT / RIGHT of the cell
 *   X (hold 1 s)                 reseed the pose to the INIT start pose (put the robot on that spot first)
 *
 * The target for the flywheel speed is the HIVE cell nearest the robot (all four, no alliance). The flywheel runs from Start and the speed trim is not used.
 * The log is the BRINGUP kind (bringup.log); read it with tools/logs/launcher_offset.py. Procedure: design/launcher-offset-finder.md.
 */
@Config
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@TeleOp(name = "Launcher Offset Finder", group = "Bring-up")
public class LauncherOffsetFinder extends LinearOpMode {

    public static double START_Y_IN = 16.0;
    public static double TURN_SCALE = 0.5;
    public static double PRECISION_SCALE = 0.30;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double RESEED_HOLD_SEC = 1.0;
    public static long SAMPLE_PERIOD_MS = 100;

    private String logCell = "-";
    private double logX, logY, logHeadingDeg, logDist, logLeftIn;

    private static Pose2d startPose(boolean blue, double startX) {
        return blue ? new Pose2d(startX, START_Y_IN, 0.0) : new Pose2d(-startX, -START_Y_IN, Math.PI);
    }

    private static HiveTarget nearestCell(Pose2d pose) {
        HiveTarget best = HiveTarget.BLUE_REAR;
        double bestDist = Double.MAX_VALUE;
        for (HiveTarget t : HiveTarget.values()) {
            double d = t.distanceFrom(pose);
            if (d < bestDist) {
                bestDist = d;
                best = t;
            }
        }
        return best;
    }

    /** {distance in, bearing deg, error deg (+ = the cell is counterclockwise of the front), the cell's center left of the front line in} */
    private static double[] aim(Pose2d pose, HiveTarget t) {
        double dx = t.x - pose.position.x, dy = t.y - pose.position.y;
        double dist = Math.hypot(dx, dy);
        double bearing = Math.atan2(dy, dx);
        double err = Rotation2d.normalize(bearing - pose.heading.toDouble());
        return new double[]{dist, Math.toDegrees(bearing), Math.toDegrees(err), dist * Math.sin(err)};
    }

    private String describe(String label, Pose2d pose, HiveTarget cell, Mechanisms mechanisms) {
        double[] a = aim(pose, cell);
        return String.format(Locale.US, "%s pose (%.2f, %.2f, %.1f deg), cell %s (%.2f, %.2f) %.1f in, bearing %.1f deg, cell center %+.1f deg / %+.2f in left of the front line"
                        + " (launcher offset estimate if the salvo went through the center: %+.2f in; in use %+.1f), setpoint %.0f",
                label, pose.position.x, pose.position.y, pose.heading.toDegrees(), cell, cell.x, cell.y, a[0], a[1], a[2], a[3], a[3],
                TargetLock.LAUNCHER_OFFSET_LEFT_IN, mechanisms.getLastSetpoint());
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Launcher Offset Finder", hardwareMap);
        try {
            find();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void find() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        double startX = -(FieldGeometry.HALF_IN - profile.originFromBackIn);   // blue: back against the rear wall

        boolean blue = true, seededBlue = true;
        Pose2d start = startPose(blue, startX);
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right) blue = true;
            if (gamepad1.dpad_left) blue = false;
            if (blue != seededBlue) {
                seededBlue = blue;
                start = startPose(blue, startX);
                drive.setPose(start);
            }
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            boolean poseOk = loc.seedConfirmed
                    && Math.hypot(p.position.x - start.position.x, p.position.y - start.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - start.heading.toDouble()))) < 3.0;
            telemetry.addLine("Launcher Offset Finder. Target lock is OFF. Place the robot at the start pose, still.");
            telemetry.addData("Start", blue ? "BLUE: back against the rear wall, facing +X, y +" + START_Y_IN : "RED: back against the audience wall, facing -X, y -" + START_Y_IN);
            telemetry.addData("Start pose", "%.2f, %.2f, %.1f deg   (walls at +-%.1f in)", start.position.x, start.position.y, start.heading.toDegrees(), FieldGeometry.HALF_IN);
            telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   %s", loc.getDeviceStatus(), p.position.x, p.position.y, p.heading.toDegrees(),
                    poseOk ? "OK" : "WARNING: does not match the start pose (do not move the robot after INIT; restart the opmode)");
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addLine("Gamepad 1: D-pad right = BLUE start, D-pad left = RED start, then press Start on the Driver Station.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();

        final boolean isBlue = blue;
        final Pose2d defaultStart = start;
        double driverForwardRad = Math.toRadians(isBlue ? org.firstinspires.ftc.teamcode.opmodes.TeleOpCompetition.BLUE_DRIVER_FORWARD_DEG
                : org.firstinspires.ftc.teamcode.opmodes.TeleOpCompetition.RED_DRIVER_FORWARD_DEG);
        mechanisms.setBlueAlliance(isBlue);
        mechanisms.setAutoTarget(false);            // the target is set below: the cell nearest the robot
        mechanisms.setIntakeAlwaysOn(true);
        mechanisms.setAutoStopAfterSalvo(false);
        mechanisms.setSpinRequested(true);
        RunLog.event(String.format(Locale.US, "SETUP profile %s, start %s (%.2f, %.2f, %.1f deg), walls at +-%.1f in, LAUNCHER_OFFSET_LEFT_IN %+.1f; %s", profile.name,
                isBlue ? "BLUE" : "RED", start.position.x, start.position.y, start.heading.toDegrees(), FieldGeometry.HALF_IN, TargetLock.LAUNCHER_OFFSET_LEFT_IN, mechanisms.describe()));
        RunLog.setSampler("x,y,hdg_deg,cell,dist_in,cell_left_in," + Mechanisms.SAMPLE_COLUMNS, SAMPLE_PERIOD_MS,
                () -> String.format(Locale.US, "%.2f,%.2f,%.1f,%s,%.1f,%.2f,%s", logX, logY, logHeadingDeg, logCell, logDist, logLeftIn, mechanisms.sampleRow()));

        String lastLaunch = "none yet";
        int marksGood = 0;
        boolean reseedDown = false, reseeded = false;
        ElapsedTime reseedHold = new ElapsedTime();

        while (opModeIsActive()) {
            PoseVelocity2d measured = drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            HiveTarget cell = nearestCell(pose);
            mechanisms.setOpenCell(cell);
            double[] a = aim(pose, cell);
            logX = pose.position.x;
            logY = pose.position.y;
            logHeadingDeg = pose.heading.toDegrees();
            logCell = cell.name();
            logDist = a[0];
            logLeftIn = a[3];

            // drive, field-oriented like the TeleOp (stick up = away from the driver)
            double scale = gamepad1.left_bumper ? PRECISION_SCALE : 1.0;
            double up = -gamepad1.left_stick_y * scale, right = gamepad1.left_stick_x * scale;
            double heading = pose.heading.toDouble();
            double fieldX = up * Math.cos(driverForwardRad) + right * Math.cos(driverForwardRad - Math.PI / 2.0);
            double fieldY = up * Math.sin(driverForwardRad) + right * Math.sin(driverForwardRad - Math.PI / 2.0);
            double forward = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
            double left = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, left), -gamepad1.right_stick_x * TURN_SCALE * scale));

            mechanisms.setManualIntake(gamepad1.right_trigger > TRIGGER_THRESHOLD);
            mechanisms.setManualOuttake(gamepad1.left_trigger > TRIGGER_THRESHOLD);
            if (gamepad1.aWasPressed()) {
                lastLaunch = describe("LAUNCH", pose, cell, mechanisms);
                RunLog.event(lastLaunch);
                mechanisms.requestLaunch();
            }
            boolean markGood = gamepad1.yWasPressed(), markLeft = gamepad1.dpadLeftWasPressed(), markRight = gamepad1.dpadRightWasPressed();
            if (markGood || markLeft || markRight) {
                if (markGood) marksGood++;
                RunLog.event("MARK " + (markGood ? "good" : markLeft ? "landed LEFT" : "landed RIGHT") + " | AT THE LAST LAUNCH: " + lastLaunch
                        + " | NOW: " + describe("", pose, cell, mechanisms));
            }
            if (gamepad1.x) {
                if (!reseedDown) {
                    reseedDown = true;
                    reseeded = false;
                    reseedHold.reset();
                }
                if (!reseeded && reseedHold.seconds() >= RESEED_HOLD_SEC) {
                    drive.setPose(defaultStart);
                    reseeded = true;
                    RunLog.event(String.format(Locale.US, "RESEED: pose reseeded to the start (was %.2f, %.2f, %.1f deg)", pose.position.x, pose.position.y, pose.heading.toDegrees()));
                }
            } else {
                reseedDown = false;
            }

            mechanisms.update(pose);
            RunLog.tick();

            telemetry.addData("Start", isBlue ? "BLUE (facing +X)" : "RED (facing -X)");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.2f / %.2f / %.1f", pose.position.x, pose.position.y, pose.heading.toDegrees());
            telemetry.addData("Nearest cell", "%s at %.1f in, bearing %.1f deg", cell, a[0], a[1]);
            telemetry.addData("Cell center left of the front line", "%+.2f in   (%+.1f deg)   <- the launcher offset if the launch goes through the center", a[3], a[2]);
            telemetry.addData("Flywheel setpoint", "%.0f ticks/s", mechanisms.getLastSetpoint());
            telemetry.addData("Good marks so far", marksGood);
            if (reseedDown && !reseeded) telemetry.addData("Reseed pose", "keep holding X... %.1f s", RESEED_HOLD_SEC - reseedHold.seconds());
            mechanisms.log(telemetry);
            telemetry.addLine("GP1: stick drive (LB precision), RT intake IN, LT intake OUT, A launch, Y MARK good, D-pad left / right MARK landed LEFT / RIGHT, hold X 1 s = reseed the pose to the start");
            telemetry.update();
        }
        drive.stop();
        mechanisms.setManualIntake(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
