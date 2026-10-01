package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.motion.InstantAction;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.motion.SleepAction;

/**
 * Rudimentary BIOBUZZ auto: drive off the wall, approach a firing position in front of
 * the HIVE, then stand in for a launch volley with sleeps (no launcher/intake exists on
 * this chassis yet - see the separate {@code shooter-prototype} branch). Drivetrain +
 * sequencing only; this is a bring-up test for {@link DriveToPose} chaining as much as
 * it is a real auto skeleton.
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
 * Caveat: the firing position/heading below assume the HIVE's upward CELL is approached
 * head-on from the Blue Wall side. Nobody has confirmed which way the CELL opening
 * actually faces yet (the manual only says the downward CELL "points at" a FLOWER, which
 * sit on the audience/rear walls, not the Red/Blue walls) - this is a placeholder
 * approach vector, not a verified shooting angle. Revisit once a real launcher exists
 * and the robot can be bench-tested on an actual field.
 */
@Config
@Autonomous(name = "Auton - Hive Volley (Blue)", group = "Auto")
public class AutonHiveVolley extends LinearOpMode {

    // --- field geometry (measured from the manual, not tunable) ---
    private static final double FIELD_HALF_IN = 72.0;           // Section 9.2, 144x144in field
    private static final double HIVE_HALF_WIDTH_IN = 49.46 / 2; // Section 9.6.1 frame width (Y-extent)

    // --- start pose: blue alliance, touching the Blue Wall, centered on the HIVE in X ---
    // Robot is 14(W) x 18(D) in; D (18in) is the dimension perpendicular to the wall.
    public static double ROBOT_DEPTH_IN = 18.0;
    public static double START_X_IN = 0.0;
    public static double START_HEADING_DEG = -90.0; // facing -Y, into the field toward the HIVE

    // --- tunable waypoints (placeholders - re-tune once there's real field/robot access) ---
    public static double LEAVE_DISTANCE_IN = 6.0;     // just enough to clear the perimeter wall
    public static double FIRING_STANDOFF_IN = 24.0;   // stand-off from the HIVE's near (Y) edge

    // --- simulated volley timing (placeholders, stand in for a real launcher) ---
    public static double SPIN_UP_SEC = 1.5;
    public static double FIRE_SEC = 1.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double startY = FIELD_HALF_IN - ROBOT_DEPTH_IN / 2.0;
        Pose2d startPose = new Pose2d(START_X_IN, startY, Math.toRadians(START_HEADING_DEG));

        Pose2d leavePose = new Pose2d(START_X_IN, startY - LEAVE_DISTANCE_IN, Math.toRadians(START_HEADING_DEG));

        double firingY = HIVE_HALF_WIDTH_IN + FIRING_STANDOFF_IN;
        Pose2d firingPose = new Pose2d(START_X_IN, firingY, Math.toRadians(START_HEADING_DEG));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Auton - Hive Volley (Blue). Keep still while the Pinpoint calibrates.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Action auto = new SequentialAction(
                new DriveToPose(drive, leavePose),           // LEAVE
                new DriveToPose(drive, firingPose),          // approach firing position
                new InstantAction(() -> telemetry.log().add("Simulated spin-up...")),
                new SleepAction(SPIN_UP_SEC),
                new InstantAction(() -> telemetry.log().add("Simulated fire x4 (preload -> blue CELL)")),
                new SleepAction(FIRE_SEC)
        );

        Actions.runBlocking(this, auto);

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }
}
