package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
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
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
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
 *   3. Around the outside of the hive's support rails via a helper point (HELPER_X, HELPER_Y), then to the shot spot for the audience cell (the open cell
 *      after the first tip), with the intake still running so every element gets in (it stops by itself at 4, and is switched off on arrival); aim, shoot.
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


    // ---- the path, BLUE frame: x, y in inches, heading in degrees (the start pose comes from the chassis profile) ----
    // The x values are the waypoint marks of 2026-11-02 + 1.0: those were recorded in a pose frame seeded at x = -64.4 (a 72 in field half-width), and the walls are
    // at 71 (start x = -63.4), so the same physical places read 1.0 more. (Odometry only adds displacement to the seed, so shifting every x is exact; y is unchanged.)
    public static double START_Y_IN = 16.0;
    /**
     * The garden is approached at an ANGLE, the front (intake mouth) turned 40 deg toward the rear wall (heading 130), so the pollen lying along the wall are caught between
     * the angled mouth and the wall instead of being plowed ahead in a line (the straight approach at heading 90 did that, 2026-11-03). At 130 deg the front corners swing out
     * about 11.6 in in x and 12 in in y from the origin, so the poses are 13.5 in from the rear wall (the wall-side corner about 2 in off it) and the end is at y 57.5, not 60.3.
     * (Red: the same, turned 180 deg.) The pre-position and the end keep the same heading so the robot translates along the wall at that angle.
     */
    public static double GARDEN_PRE_X = -57.5, GARDEN_PRE_Y = 37.1, GARDEN_HEADING_DEG = 130.0;
    public static double GARDEN_END_X = -57.5, GARDEN_END_Y = 57.5;
    /** The one shot spot for the audience cell (the marks were (62.9, 9.5) and (62.3, 7.4), +1.0 in x); its heading is the computed aim. */
    public static double SHOT_X = 63.6, SHOT_Y = 8.5;
    /** The flower poses are 3 in toward +Y of the marks (21.3 to 24.3, user 2026-11-03: the pollen did not come out), and the pre-position is 5 in farther back (43 to 38) for a run-up. */
    public static double FLOWER_PRE_X = 38.0, FLOWER_PRE_Y = 24.3, FLOWER_HEADING_DEG = 0.0;
    public static double FLOWER_AT_X = 55.3, FLOWER_AT_Y = 24.3;
    /**
     * The flower is RAMMED: the drive's target is FLOWER_RAM_OVERSHOOT_IN past FLOWER_AT_X, so the error stays large and the power stays at FLOWER_RAM_POWER until the robot
     * meets the flower, and the leg ends after FLOWER_RAM_SEC (it never reaches the target: the log says TIMED OUT, which is expected). The normal drive uses 0.05 x the
     * error, so a 12 in leg never passed 0.6.
     */
    public static double FLOWER_RAM_POWER = 0.9, FLOWER_RAM_OVERSHOOT_IN = 12.0, FLOWER_RAM_SEC = 0.8;
    public static double PARK_X = 51.1, PARK_Y = 54.0, PARK_HEADING_DEG = 180.0;
    /**
     * The helper point between the garden and the shot spot, to go around the OUTSIDE of the hive's two support rails (along the row borders 2|3 and 4|5, columns C to D,
     * so y = +-23.7 from x = -23.7 to +23.7; the 2026-11-03 run stopped with its back corner on the rail end at about (-23.7, 23.7)). (36, 40) keeps the robot's
     * footprint at least 8 in from the rail end (controller simulation) and the path 145 in long. Its heading is the shot spot's aim heading.
     */
    public static double HELPER_X = 36.0, HELPER_Y = 40.0;
    /** Drive to a point level with the shot spot (x = PARK_X) before the park run, so the path does not pass the flower at the audience wall. */
    public static boolean PARK_VIA_POINT = true;

    // ---- timing ----
    /** At the flower: pause until the robot holds 4 elements, but no longer than this. */
    public static double FLOWER_PAUSE_MAX_SEC = 2.0;
    /** Each drive leg's hard cap is max(TIMEOUT_MIN_SEC, TIMEOUT_BASE_SEC + distance / TIMEOUT_IN_PER_S) + TIMEOUT_TURN_EXTRA_SEC for a turn of over 120 deg, never more than TIMEOUT_MAX_SEC. */
    public static double TIMEOUT_BASE_SEC = 1.0, TIMEOUT_IN_PER_S = 22.0, TIMEOUT_MIN_SEC = 1.5, TIMEOUT_TURN_EXTRA_SEC = 0.5, TIMEOUT_MAX_SEC = 8.0;
    /** Pass-through points (the garden, the helper, the park via point) only need to get near: a leg ends within this position and heading. The shot spots and the flower keep the tight DriveToPose tolerance. */
    public static double COARSE_POS_IN = 3.0, COARSE_HEADING_DEG = 10.0;
    /**
     * The garden pickup leg (along the rear wall, intake on) is driven slowly so the pollen get in: this is the most power any axis may use (the normal limit is 0.6;
     * speed is about 60 in/s per unit of power, so 0.25 is about 15 in/s, a 23 in leg in about 2 s). Raise it if the pickup is fine slower than needed, lower it if pollen are missed.
     */
    public static double GARDEN_PICKUP_MAX_POWER = 0.25;
    /** The leg's hard cap is worked out with a speed of this many in/s per unit of power (a little under the real speed, so it only fires when the robot is stuck). */
    public static double SLOW_IN_PER_S_PER_POWER = 40.0;

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

    /** Seconds since Start, and the finished steps as "label: seconds", for the timing lines and the summary in the log. */
    private final ElapsedTime autoClock = new ElapsedTime();
    private final List<String> timings = new ArrayList<>();

    /** Runs the action and logs how long it took, and when it ended, once it finishes (an "AUTO TIMING" line, one per step). */
    private Action timed(String label, Action inner) {
        return new Action() {
            private boolean started = false;
            private double begin;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    begin = autoClock.seconds();
                }
                boolean more = inner.run(packet);
                if (!more) {
                    double now = autoClock.seconds();
                    RunLog.event(String.format(Locale.US, "AUTO TIMING %-30s %5.2f s   (%5.2f to %5.2f s into the auto)", label, now - begin, begin, now));
                    timings.add(String.format(Locale.US, "%s %.2f", label, now - begin));
                }
                return more;
            }
        };
    }

    /** A drive leg that must settle (the shot spots, the flower): the tight DriveToPose tolerance. */
    private Action leg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC));
    }

    /** A pass-through leg: ends as soon as the robot is near. */
    private Action passLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withTolerance(COARSE_POS_IN, COARSE_HEADING_DEG)
                .withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC));
    }

    /** Drives hard at a target beyond an obstacle for a fixed time (the flower ram). */
    private Action ramLeg(String label, MecanumDrive drive, Pose2d target, double power, double seconds) {
        return timed("drive: " + label, new DriveToPose(drive, target).withMaxOutput(power).withTolerance(0.5, 5.0).withTimeout(seconds));
    }

    /** A slow pass-through leg (the garden pickup): the power is capped, and the hard cap uses the slower speed. */
    private Action slowLeg(String label, MecanumDrive drive, Pose2d target, double maxPower) {
        return timed("drive: " + label, new DriveToPose(drive, target).withMaxOutput(maxPower).withTolerance(COARSE_POS_IN, COARSE_HEADING_DEG)
                .withDistanceTimeout(TIMEOUT_BASE_SEC, SLOW_IN_PER_S_PER_POWER * maxPower, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC));
    }

    /** Commands the ramp only if it is not already heading there (a repeated command restarts its travel timer), then waits until it has settled. */
    private Action rampTo(Mechanisms mechanisms, boolean deployed) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (mechanisms.ramp == null || !mechanisms.ramp.isConfigured()) return false;
                if (!started) {
                    started = true;
                    if (mechanisms.ramp.isDeployedTarget() != deployed) {
                        if (deployed) mechanisms.requestDeploy();
                        else mechanisms.requestStowNow();
                    }
                }
                return !mechanisms.ramp.isSettled();
            }
        };
    }

    /** Aim at the cell with the lock, shoot one salvo, and finish when it is done. */
    private Action shoot(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label) {
        return new SequentialAction(
                note("Aim and fire: " + label),
                timed("aim: " + label, new AimAtCell(drive, cell, blue)),
                timed("fire (wait + salvo): " + label, mechanisms.fire()));
    }

    /** Waits until the robot holds 4 elements, or maxSec passes. */
    private Action waitUntilFull(Mechanisms mechanisms, double maxSec) {
        return timed("flower pause", new Action() {
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
        });
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
        double startX = -(FieldGeometry.HALF_IN - profile.originFromBackIn);   // back against the rear wall (blue frame)

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
        autoClock.reset();

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
        double blueShotHeadingDeg = Math.toDegrees(TargetLock.aimHeadingRad(SHOT_X, SHOT_Y, HiveTarget.BLUE_AUDIENCE, TargetLock.farCellExtraLeftIn(HiveTarget.BLUE_AUDIENCE, true)));
        Pose2d helper = forAlliance(pose(HELPER_X, HELPER_Y, blueShotHeadingDeg), isBlue);   // outside the support rails, at the shot spot's heading
        Pose2d flowerPre = forAlliance(pose(FLOWER_PRE_X, FLOWER_PRE_Y, FLOWER_HEADING_DEG), isBlue);
        Pose2d flowerRam = forAlliance(pose(FLOWER_AT_X + FLOWER_RAM_OVERSHOOT_IN, FLOWER_AT_Y, FLOWER_HEADING_DEG), isBlue);   // past the flower: the ram target
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
                passLeg("garden pre-position", drive, gardenPre),
                note("Intake ON: garden, slow"),
                mechanisms.setIntake(true),
                slowLeg("garden pickup (slow)", drive, gardenEnd, GARDEN_PICKUP_MAX_POWER),

                new InstantAction(() -> mechanisms.setOpenCell(secondCell)),   // the tip flipped the cell; the flywheel follows the new distance on the way
                note("Around the outside of the hive support rails, intake still ON so every element gets in"),
                passLeg("helper (outside the support rails)", drive, helper),
                note("To the shot spot"),
                leg("shot spot 1", drive, shotSpot),
                note("Intake OFF"),
                mechanisms.setIntake(false),
                shoot(drive, mechanisms, secondCell, isBlue, "garden pollen"),

                new InstantAction(mechanisms::requestDeploy),                   // deploys on the way
                note("Flower pre-position"),
                leg("flower pre-position", drive, flowerPre),
                timed("ramp deploy (wait)", rampTo(mechanisms, true)),         // fully deployed before it goes on; no second command
                note("Intake ON: flower"),
                mechanisms.setIntake(true),
                ramLeg("ram the flower", drive, flowerRam, FLOWER_RAM_POWER, FLOWER_RAM_SEC),
                waitUntilFull(mechanisms, FLOWER_PAUSE_MAX_SEC),
                mechanisms.setIntake(false),

                new InstantAction(mechanisms::requestStow),                     // waits by itself until the robot is clear of the flower
                note("Back to the shot spot"),
                leg("shot spot 2", drive, shotSpot),
                timed("ramp stow (wait)", rampTo(mechanisms, false)),
                shoot(drive, mechanisms, secondCell, isBlue, "flower pollen"),

                note("Park"),
                PARK_VIA_POINT ? passLeg("park via point", drive, parkVia) : new InstantAction(() -> { }),
                leg("park", drive, park),
                new InstantAction(mechanisms::stopFlywheel)
        );

        Actions.runBlocking(this, auto, () -> mechanisms.update(drive.getPose()));
        RunLog.event(isStopRequested() ? "AUTO stopped before it finished" : "AUTO actions finished");
        RunLog.event(String.format(Locale.US, "AUTO TIMING total %.2f s into the auto; steps (s): %s", autoClock.seconds(), String.join("; ", timings)));

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
