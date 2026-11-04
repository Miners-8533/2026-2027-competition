package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
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
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;

import java.util.Locale;

/**
 * Rudimentary BIOBUZZ auto: drive off the wall (LEAVE), then launch the preload with the real flywheel and pusher (the
 * {@link Mechanisms} coordinator, updated every loop; the flywheel spins from Start). Assumes the launcher can make the first
 * shot from near the starting position itself - no approach-to-the-HIVE drive needed.
 * Drivetrain + sequencing only; this is a bring-up test for {@link DriveToPose} as much
 * as it is a real auto skeleton.
 *
 * Models firing the 4 preloaded POLLEN into the blue upward-facing CELL, which already
 * holds 3 seeded NECTAR (6 of the 8 "pollen-equivalents" needed to tip, per
 * BIOBUZZ/analysis/four-cycle-auto-score-estimate.md) - an "overflow" first HIVE TIP
 * that needs only 2 of the 4 preloaded POLLEN.
 *
 * Field coordinates follow the official FTC field coordinate system (origin at field
 * center, same "inverted" orientation BIOBUZZ shares with last year's DECODE field):
 * Red Wall at Y=-72, Blue Wall at Y=+72, audience wall at X=+72, heading 0 = facing +X,
 * CCW positive. See BIOBUZZ/rules/field-elements.md and the manual's Figure 9-4/9-5/10-2.
 *
 * HIVE geometry (manual Figures 9-8, 9-9, 9-10, 10-2): each HIVE rocks about a pivot axis
 * running along Y, with its two CELLs at either end, so the CELL openings face along X
 * (toward the audience or rear wall) - not toward the Red/Blue Walls. The red and blue
 * HIVEs sit side by side 25.5 in apart, so the blue HIVE is centered at Y=+12.75. As
 * staged (Fig 10-2), blue's seeded NECTAR are in the blue HIVE's REAR-side CELL (its
 * opening faces -X, tilted up 30 deg), so the robot starts against the REAR wall facing
 * +X, lined up with the blue HIVE. After a TIP the blue HIVE's other (audience-side)
 * CELL is the upward one, so any follow-up volley would need the opposite side.
 *
 * Caveat: assumes the eventual launcher can reach the CELL opening (about 32 in away in X,
 * opening 53.5-65.6 in above the tiles) from the starting position. Unverified until a
 * real launcher exists and the robot can be tested on an actual field.
 */
@Config
@Autonomous(name = "Auton - Hive Volley (Blue)", group = "Auto")
public class AutonHiveVolley extends LinearOpMode {

    // --- field geometry (measured from the manual, not tunable) ---

    // --- start pose: blue alliance, touching the REAR wall, lined up with the blue HIVE in Y ---
    // Robot depth (the dimension perpendicular to the wall) comes from the chassis profile.
    public static double START_Y_IN = 12.75;        // blue HIVE center: half of the 25.5 in HIVE center-to-center
    public static double START_HEADING_DEG = 0.0;   // facing +X, into the field toward the blue CELL opening

    // --- tunable waypoint (placeholder - re-tune once there's real field/robot access) ---
    public static double LEAVE_DISTANCE_IN = 6.0;     // just enough to clear the perimeter wall

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.AUTO, "Auton - Hive Volley (Blue)", hardwareMap);
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
        double startX = -(FieldGeometry.HALF_IN - profile.originFromBackIn);   // back against the rear wall
        Pose2d startPose = new Pose2d(startX, START_Y_IN, Math.toRadians(START_HEADING_DEG));

        Pose2d leavePose = new Pose2d(startX + LEAVE_DISTANCE_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);
        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();   // reads only; nothing moves in INIT
            Pose2d p = drive.getPose();
            boolean poseOk = loc.seedConfirmed
                    && Math.hypot(p.position.x - startPose.position.x, p.position.y - startPose.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - startPose.heading.toDouble()))) < 3.0;
            telemetry.addLine("Auton - Hive Volley (Blue). Rear wall, centered on the blue HIVE, facing +X. Keep still.");
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Pinpoint status", loc.getDeviceStatus());
            telemetry.addData("Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f",
                    p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.addData("Expected start pose", "%.2f, %.2f, %.1f",
                    startPose.position.x, startPose.position.y, startPose.heading.toDegrees());
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
        mechanisms.setOpenCell(HiveTarget.BLUE_REAR);   // as staged
        mechanisms.setAutoStopAfterSalvo(false);
        mechanisms.setSpinRequested(true);              // spins from Start so the volley launches as soon as the robot is in place

        Action auto = new SequentialAction(
                new DriveToPose(drive, leavePose),           // LEAVE
                new InstantAction(() -> {
                    telemetry.log().add("Launch: preload -> blue CELL");
                    RunLog.event("AUTO launch: preload -> blue CELL");
                }),
                mechanisms.launch()
        );

        Actions.runBlocking(this, auto, () -> mechanisms.update(drive.getPose()));
        RunLog.event(isStopRequested() ? "AUTO stopped before it finished" : "AUTO actions finished");

        // hand the final pose (and the alliance) to the TeleOp, whether the auto finished or was stopped
        drive.updatePoseEstimate();
        PoseStorage.save(drive.getPose(), true);

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }
}
