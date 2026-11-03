package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.motion.AimAtCell;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.motion.InstantAction;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.motion.SleepAction;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Solo auto (our partner just parks and stays out of the way). One path, written for BLUE from poses marked with the Auto Waypoint Marker
 * (2026-11-02) and snapped to the nearest 90 deg; RED is the same path turned 180 deg about the field center, (x, y, heading) to
 * (-x, -y, heading + 180), the field's symmetry. The alliance is picked in INIT with gamepad 1 (D-pad right = BLUE, left = RED).
 *
 * The robot finishes each action before it starts the next one (a drive settles, a salvo completes, the ramp settles). The flywheel runs from
 * Start and the shot map follows the distance to the open cell, so it is already up to speed when the robot gets to a shot spot.
 *
 *   1. Start (the TeleOp's start pose: back against the wall, y 16): turn onto the rear cell and shoot the preload.
 *   2. Garden pre-position (heading 90, facing the Blue Wall), then the garden pickup with the intake running, driving along the rear wall.
 *   3. To the shot spot for the audience cell (the open cell after the first tip), aim, shoot.
 *   4. Deploy the ramp on the way to the flower pre-position (heading 0) and wait for it to settle; drive onto the flower with the intake running;
 *      pause there until the robot holds 4 elements (at most FLOWER_PAUSE_MAX_SEC).
 *   5. Stow the ramp, back to the same shot spot, aim, shoot.
 *   6. Park (via a point level with the shot spot so the path stays clear of the flower at the audience wall).
 *
 * The shot spot's heading is not snapped: it is the heading that points the shooter's line at the cell from there ({@link TargetLock#aimHeadingRad},
 * with the shooter offset and the far-cell extra), and {@link AimAtCell} takes out what is left before each shot. The straight run from the garden
 * to the shot spot has no intermediate point (assumed clear of the HIVE support; watch the first runs slowly).
 *
 * The final pose is saved for the TeleOp, which starts from it when it is fresh.
 */
@Config
@Autonomous(name = "Auton - Solo", group = "Auto")
public class AutonSolo extends LinearOpMode {

    private static final double FIELD_HALF_IN = 72.0;

    // ---- the path, BLUE frame: x, y in inches, heading in degrees (the start pose comes from the chassis profile) ----
    public static double START_Y_IN = 16.0;
    public static double GARDEN_PRE_X = -64.6, GARDEN_PRE_Y = 37.1, GARDEN_HEADING_DEG = 90.0;
    public static double GARDEN_END_X = -64.7, GARDEN_END_Y = 60.3;
    /** The one shot spot for the audience cell (the marks were (62.9, 9.5) and (62.3, 7.4)); its heading is the computed aim. */
    public static double SHOT_X = 62.6, SHOT_Y = 8.5;
    public static double FLOWER_PRE_X = 42.0, FLOWER_PRE_Y = 21.3, FLOWER_HEADING_DEG = 0.0;
    public static double FLOWER_AT_X = 54.3, FLOWER_AT_Y = 21.3;
    public static double PARK_X = 50.1, PARK_Y = 54.0, PARK_HEADING_DEG = 180.0;
    /** Drive to a point level with the shot spot (x = PARK_X) before the park run, so the path does not pass the flower at the audience wall. */
    public static boolean PARK_VIA_POINT = true;

    // ---- timing ----
    /** Seconds to keep the intake running at the end of the garden pickup (0 = stop on arrival). */
    public static double GARDEN_DWELL_SEC = 0.0;
    /** At the flower: pause until the robot holds 4 elements, but no longer than this. */
    public static double FLOWER_PAUSE_MAX_SEC = 2.0;
    /** Hard cap for each drive leg (the DriveToPose default of 4 s is too short for the long diagonal). */
    public static double LEG_TIMEOUT_SEC = 8.0;

    private static Pose2d pose(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    /** The blue-frame pose as this alliance sees it: red is the blue pose turned 180 deg about the field center. */
    private static Pose2d forAlliance(Pose2d p, boolean blue) {
        return blue ? p : new Pose2d(-p.position.x, -p.position.y, p.heading.toDouble() + Math.PI);
    }

    private Action note(String message) {
        return new InstantAction(() -> {
            telemetry.log().add(message);
            RunLog.event("AUTO " + message);
        });
    }

    private Action leg(MecanumDrive drive, Pose2d target) {
        return new DriveToPose(drive, target).withTimeout(LEG_TIMEOUT_SEC);
    }

    /** Aim at the cell with the lock, shoot one salvo, and finish when it is done. */
    private Action shoot(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label) {
        return new SequentialAction(
                note("Aim and fire: " + label),
                new AimAtCell(drive, cell, blue),
                mechanisms.fire());
    }

    /** Waits until the robot holds 4 elements, or maxSec passes. */
    private Action waitUntilFull(Mechanisms mechanisms, double maxSec) {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    timer.reset();
                }
                boolean done = mechanisms.isFull() || timer.seconds() >= maxSec;
                if (done) {
                    RunLog.event(String.format(Locale.US, "AUTO flower pause over after %.2f s, robot %s", timer.seconds(), mechanisms.isFull() ? "full" : "NOT full"));
                }
                return !done;
            }
        };
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.AUTO, "Auton - Solo", hardwareMap);
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
        double startX = -(FIELD_HALF_IN - profile.originFromBackIn);   // back against the rear wall (blue frame)

        boolean blue = true, seededBlue = true;
        Pose2d start = forAlliance(pose(startX, START_Y_IN, 0.0), blue);
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right) blue = true;
            if (gamepad1.dpad_left) blue = false;
            if (blue != seededBlue) {
                seededBlue = blue;
                start = forAlliance(pose(startX, START_Y_IN, 0.0), blue);
                drive.setPose(start);
            }
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();   // reads only; nothing moves in INIT
            Pose2d p = drive.getPose();
            boolean poseOk = loc.seedConfirmed
                    && Math.hypot(p.position.x - start.position.x, p.position.y - start.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - start.heading.toDouble()))) < 3.0;
            telemetry.addLine("Auton - Solo. Back against the " + (blue ? "REAR" : "AUDIENCE") + " wall at y " + (blue ? "+" : "-") + START_Y_IN + ". Keep still.");
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Pinpoint status", loc.getDeviceStatus());
            telemetry.addData("Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f", p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.addData("Expected start pose", "%.2f, %.2f, %.1f", start.position.x, start.position.y, start.heading.toDegrees());
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addData("Preload", mechanisms.counter == null ? "no element counter"
                    : mechanisms.counter.isValid() ? mechanisms.counter.count() + " elements seen (expect 4)   " + mechanisms.counter.stackString()
                    : "counter NOT READING");
            telemetry.addLine(poseOk ? "Pose OK." : "WARNING: Pinpoint pose does not match the start pose. Do NOT start; restart the opmode.");
            telemetry.addLine("Gamepad 1: D-pad right = BLUE, D-pad left = RED, then press Start on the Driver Station.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();

        // the path for this alliance
        final boolean isBlue = blue;
        HiveTarget firstCell = isBlue ? HiveTarget.BLUE_REAR : HiveTarget.RED_AUDIENCE;    // as staged; the first tip flips it
        HiveTarget secondCell = firstCell.otherSide();                                     // blue: audience, red: rear
        Pose2d gardenPre = forAlliance(pose(GARDEN_PRE_X, GARDEN_PRE_Y, GARDEN_HEADING_DEG), isBlue);
        Pose2d gardenEnd = forAlliance(pose(GARDEN_END_X, GARDEN_END_Y, GARDEN_HEADING_DEG), isBlue);
        Pose2d shotSpotBlueFrame = pose(SHOT_X, SHOT_Y, 0.0);
        Pose2d shotXY = forAlliance(shotSpotBlueFrame, isBlue);
        double shotHeading = TargetLock.aimHeadingRad(shotXY.position.x, shotXY.position.y, secondCell, TargetLock.farCellExtraLeftIn(secondCell, isBlue));
        Pose2d shotSpot = new Pose2d(shotXY.position.x, shotXY.position.y, shotHeading);
        Pose2d flowerPre = forAlliance(pose(FLOWER_PRE_X, FLOWER_PRE_Y, FLOWER_HEADING_DEG), isBlue);
        Pose2d flowerAt = forAlliance(pose(FLOWER_AT_X, FLOWER_AT_Y, FLOWER_HEADING_DEG), isBlue);
        Pose2d parkVia = forAlliance(pose(PARK_X, SHOT_Y, PARK_HEADING_DEG), isBlue);
        Pose2d park = forAlliance(pose(PARK_X, PARK_Y, PARK_HEADING_DEG), isBlue);

        RunLog.event(String.format(Locale.US, "SETUP profile %s, alliance %s, start pose (%.1f, %.1f, %.1f deg), cells %s then %s, shot spot (%.1f, %.1f, %.1f deg), %s",
                profile.name, isBlue ? "BLUE" : "RED", drive.getPose().position.x, drive.getPose().position.y, drive.getPose().heading.toDegrees(),
                firstCell, secondCell, shotSpot.position.x, shotSpot.position.y, shotSpot.heading.toDegrees(), mechanisms.describe()));
        RunLog.setSampler("x,y,hdg_deg," + Mechanisms.SAMPLE_COLUMNS, 100, () -> {
            Pose2d p = drive.getPose();
            return String.format(Locale.US, "%.2f,%.2f,%.1f,%s", p.position.x, p.position.y, p.heading.toDegrees(), mechanisms.sampleRow());
        });

        mechanisms.setBlueAlliance(isBlue);
        mechanisms.setOpenCell(firstCell);
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel stays up between shots ...
        mechanisms.setSpinRequested(true);         // ... and spins from Start, so it is at speed on arrival at each shot spot

        Action auto = new SequentialAction(
                shoot(drive, mechanisms, firstCell, isBlue, "preload"),

                note("Garden pre-position"),
                leg(drive, gardenPre),
                note("Intake ON: garden"),
                mechanisms.setIntake(true),
                leg(drive, gardenEnd),
                new SleepAction(GARDEN_DWELL_SEC),
                note("Intake OFF"),
                mechanisms.setIntake(false),

                new InstantAction(() -> mechanisms.setOpenCell(secondCell)),   // the tip flipped the cell; the flywheel follows the new distance on the way
                note("To the shot spot"),
                leg(drive, shotSpot),
                shoot(drive, mechanisms, secondCell, isBlue, "garden pollen"),

                new InstantAction(mechanisms::requestDeploy),                   // deploys on the way
                note("Flower pre-position"),
                leg(drive, flowerPre),
                mechanisms.deploy(),                                            // fully deployed before it goes on
                note("Intake ON: flower"),
                mechanisms.setIntake(true),
                leg(drive, flowerAt),
                waitUntilFull(mechanisms, FLOWER_PAUSE_MAX_SEC),
                mechanisms.setIntake(false),

                new InstantAction(mechanisms::requestStow),                     // waits by itself until the robot is clear of the flower
                note("Back to the shot spot"),
                leg(drive, shotSpot),
                mechanisms.stowNow(),
                shoot(drive, mechanisms, secondCell, isBlue, "flower pollen"),

                note("Park"),
                PARK_VIA_POINT ? leg(drive, parkVia) : new InstantAction(() -> { }),
                leg(drive, park),
                new InstantAction(mechanisms::stopFlywheel)
        );

        Actions.runBlocking(this, auto, () -> mechanisms.update(drive.getPose()));
        RunLog.event(isStopRequested() ? "AUTO stopped before it finished" : "AUTO actions finished");

        // hand the final pose (and the alliance) to the TeleOp, whether the auto finished or was stopped
        drive.updatePoseEstimate();
        PoseStorage.save(drive.getPose(), isBlue);

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            PoseStorage.save(p, isBlue);
            mechanisms.update(p);
            RunLog.tick();
            telemetry.addLine("Auto complete.");
            telemetry.addData("Final Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f", p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.update();
        }
    }
}
