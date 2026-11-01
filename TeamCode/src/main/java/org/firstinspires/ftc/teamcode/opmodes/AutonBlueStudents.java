package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.motion.InstantAction;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.motion.SleepAction;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;

import java.util.Locale;

/**
 * Solo Blue auto designed by the students. The shots, the intake and the ramp are the real mechanisms (the
 * {@link Mechanisms} coordinator, updated every loop). The flywheel spins from Start; a salvo fires as soon as it is ready.
 *
 * Plan, in field coordinates (origin at field center, X toward the audience, Y toward the
 * Blue Wall, heading 0 = +X, counterclockwise positive):
 *   1. Start on the rear wall at Y=16 facing +X. Shoot all 4 preload (tip 1, overflow).
 *   2. Rotate 90 deg so the BACK of the robot faces the Blue Wall. The intake is on the back and
 *      the shooter on the front, so the garden run must go backward into the corner: heading -90
 *      (clockwise turn). The students' notes said counterclockwise (heading +90), which would
 *      lead with the shooter end; GARDEN_HEADING_DEG = 90 restores that.
 *   3. Drive to (X=rear wall, Y=50), intake ON, drive to the blue corner, intake OFF.
 *      This collects the blue GARDEN's 4 POLLEN along the rear wall.
 *   4. Drive to (24, 38) facing +X, clear of the HIVE.
 *   5. Drive to the audience wall at Y=11 facing -X (180 deg), toward the blue HIVE's now
 *      upward audience-side CELL. Shoot 4.
 *   6. Drive to (51, 24), intake ON, drive to (59, 24) facing 180 deg at the audience-wall
 *      FLOWER, intake OFF, collecting its 4 POLLEN.
 *   7. Back to the shooting spot, shoot 4 again (8 equivalents in the cell, tip 2). End.
 *
 * Mechanisms: the intake stops by itself at 4 elements (the element counter). The ramp is deployed on the way to the
 * flower (deploying turns the intake on) and stowed at the shooting spot, which the path puts clear of the flower, without
 * the pose check the operator's stow uses. The open CELL is set to the rear side for the first volley and the audience
 * side after the first tip. The pusher is assumed to be at HOLD at INIT (so it must be, at power-on) and keeps that zero
 * for the TeleOp.
 *
 * Wall-touching coordinates come from the chassis profile's robot depth: a robot of depth D
 * touching a wall has its center D/2 from it, so "rear wall" is X = -(72 - D/2) and the corner
 * pickup is Y = 72 - D/2. The students' notes used +-64 (a 16 in robot); an 18 in robot gives +-63.
 *
 * Heads-up for the first run: rotating a 14x18 in robot in place at a wall swings its corners
 * about 11.4 in from its center, more than the 9 in to the wall, so step 2 can clip the wall.
 * TURN_CLEARANCE_IN moves the turn that many inches away from the wall; it defaults to 0,
 * exactly as drawn in the notes.
 */
@Config
@Autonomous(name = "Auton - Blue Students", group = "Auto")
public class AutonBlueStudents extends LinearOpMode {

    private static final double FIELD_HALF_IN = 72.0;

    public static double TURN_CLEARANCE_IN = 0.0;
    public static double GARDEN_HEADING_DEG = -90.0;   // back intake leads toward +Y; 90 = shooter end leads

    /** Seconds to keep driving the intake in after reaching the flower, before leaving. */
    public static double FLOWER_DWELL_SEC = 0.8;

    private static Pose2d pose(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    private Action note(String message) {
        return new InstantAction(() -> {
            telemetry.log().add(message);
            RunLog.event("AUTO " + message);
        });
    }

    private Action shoot(Mechanisms mechanisms, String label) {
        return new SequentialAction(
                note("Fire: " + label),
                mechanisms.fire());
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.AUTO, "Auton - Blue Students", hardwareMap);
        try {
            runAuto();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runAuto() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        double wall = FIELD_HALF_IN - profile.robotDepthIn / 2.0;

        Pose2d start = pose(-wall, 16, 0);
        Pose2d facingBlue = pose(-wall + TURN_CLEARANCE_IN, 16, GARDEN_HEADING_DEG);
        Pose2d gardenStart = pose(-wall, 50, GARDEN_HEADING_DEG);
        Pose2d gardenEnd = pose(-wall, wall, GARDEN_HEADING_DEG);
        Pose2d clearOfHive = pose(24, 38, 0);
        Pose2d shootSpot = pose(wall, 11, 180);
        Pose2d flowerNear = pose(51, 24, 180);
        Pose2d flowerAt = pose(59, 24, 180);

        MecanumDrive drive = new MecanumDrive(hardwareMap, start);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);

        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();   // reads only; nothing moves in INIT
            Pose2d p = drive.getPose();
            boolean poseOk = loc.seedConfirmed
                    && Math.hypot(p.position.x - start.position.x, p.position.y - start.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - start.heading.toDouble()))) < 3.0;
            telemetry.addLine("Auton - Blue Students. Rear wall at Y=16, facing +X. Keep still.");
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Pinpoint status", loc.getDeviceStatus());
            telemetry.addData("Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f",
                    p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.addData("Expected start pose", "%.2f, %.2f, %.1f",
                    start.position.x, start.position.y, start.heading.toDegrees());
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addData("Preload", mechanisms.counter == null ? "no element counter"
                    : mechanisms.counter.isValid() ? mechanisms.counter.count() + " elements seen (expect 4)   " + mechanisms.counter.stackString()
                    : "counter NOT READING");
            telemetry.addLine(poseOk ? "Pose OK." : "WARNING: Pinpoint pose does not match the start pose. Do NOT start; restart the opmode.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.event(String.format("SETUP profile %s, start pose %.1f, %.1f, %.1f deg", profile.name,
                drive.getPose().position.x, drive.getPose().position.y, drive.getPose().heading.toDegrees()));
        RunLog.setSampler("x,y,hdg_deg," + Mechanisms.SAMPLE_COLUMNS, 100, () -> {
            Pose2d p = drive.getPose();
            return String.format(Locale.US, "%.2f,%.2f,%.1f,%s", p.position.x, p.position.y, p.heading.toDegrees(), mechanisms.sampleRow());
        });

        mechanisms.setBlueAlliance(true);
        mechanisms.setOpenCell(HiveTarget.BLUE_REAR);   // as staged; the first tip flips it
        mechanisms.setAutoStopAfterSalvo(false);        // the flywheel stays up for the next volley ...
        mechanisms.setSpinRequested(true);              // ... and spins from Start so the first volley fires at once

        Action auto = new SequentialAction(
                shoot(mechanisms, "preload, tip 1"),
                new DriveToPose(drive, facingBlue),
                new DriveToPose(drive, gardenStart),
                note("Intake ON: blue garden"),
                mechanisms.setIntake(true),
                new DriveToPose(drive, gardenEnd),
                note("Intake OFF"),
                mechanisms.setIntake(false),
                new DriveToPose(drive, clearOfHive),
                new InstantAction(() -> mechanisms.setOpenCell(HiveTarget.BLUE_AUDIENCE)),   // the tip flipped the cell
                new DriveToPose(drive, shootSpot),
                shoot(mechanisms, "volley 2"),
                new InstantAction(mechanisms::requestDeploy),   // deploys while driving; deploying turns the intake on
                new DriveToPose(drive, flowerNear),
                note("Ramp deployed, intake ON: flower"),
                new DriveToPose(drive, flowerAt),
                new SleepAction(FLOWER_DWELL_SEC),
                note("Leaving the flower"),
                new DriveToPose(drive, shootSpot),
                mechanisms.stowNow(),
                shoot(mechanisms, "volley 3, tip 2")
        );

        Actions.runBlocking(this, auto, () -> mechanisms.update(drive.getPose()));
        RunLog.event(isStopRequested() ? "AUTO stopped before it finished" : "AUTO actions finished");

        // hand the final pose (and the alliance) to the TeleOp, whether the auto finished or was stopped
        drive.updatePoseEstimate();
        PoseStorage.save(drive.getPose(), true);

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            PoseStorage.save(p, true);
            mechanisms.update(p);
            RunLog.tick();
            telemetry.addLine("Auto complete.");
            telemetry.addData("Final Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f",
                    p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.addData("Shooting spot (in, in, deg)", "%.2f, %.2f, %.1f",
                    shootSpot.position.x, shootSpot.position.y, shootSpot.heading.toDegrees());
            telemetry.update();
        }
    }
}
