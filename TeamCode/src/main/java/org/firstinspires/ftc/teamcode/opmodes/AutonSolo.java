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
import org.firstinspires.ftc.teamcode.motion.Jostle;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.motion.SquareOnWall;
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
 * Start and the launch map follows the distance to the open cell, so it is already up to speed when the robot gets to a launch spot.
 *
 *   1. Start (the TeleOp's start pose: back against the wall, y 16): turn onto the rear cell and launch the preload.
 *   2. Garden pre-position (heading 90, facing the Blue Wall), then the garden pickup with the intake running, driving along the rear wall.
 *   3. Around the outside of the hive's support rails via a helper point (HELPER_X, HELPER_Y), then to the launch spot (on the flower's line, LAUNCH_X, LAUNCH_Y) for the audience cell (the open cell
 *      after the first tip), with the intake still running so every element gets in (it stops by itself at 4, and is switched off on arrival); aim, launch.
 *   4. Deploy the ramp (after the salvo, or during it with RAMP_DEPLOY_DURING_LAUNCH) while backing to the flower pre-position and turning to heading 0, wait for it to settle;
 *      drive onto the flower with the intake running; jostle it (back and push, repeated: see Jostle) until the robot holds 4 elements (at most FLOWER_PAUSE_MAX_SEC).
 *   5. Stow the ramp, back off along the flower's line to the launch spot while turning onto the cell (FLOWER_TURN_WHILE_BACKING), aim, launch.
 *   6. Park (straight from the launch spot; PARK_VIA_POINT can add a point level with it).
 *
 * The launch spot's heading is not snapped: it is the heading that points the launcher's line at the cell from there ({@link TargetLock#aimHeadingRad},
 * with the launcher offset and the far-cell extra), and {@link AimAtCell} takes out what is left before each launch. The straight run from the garden
 * to the launch spot has no intermediate point (assumed clear of the HIVE support; watch the first runs slowly).
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
     * The garden is approached at an ANGLE, the front (intake mouth) turned 15 deg toward the rear wall (heading 105 since 2026-11-03, was 130), so the pollen lying along the wall are caught between
     * the angled mouth and the wall instead of being plowed ahead in a line (the straight approach at heading 90 did that, 2026-11-03). At 105 deg the front corners swing out
     * about 9.2 in in x and 11.3 in in y from the origin, so the poses are 11.2 in from the rear wall (the wall-side corner about 2 in off it) and the end is at y 58.5; the
     * pre-position is nearer the Blue Wall (y 46, was 37.1) so the slow leg is short and reaches the pollen (the 2026-11-03 run reached only y 53.8 in 3.1 s).
     * (Red: the same, turned 180 deg.) The pre-position and the end keep the same heading so the robot translates along the wall at that angle.
     */
    public static double GARDEN_PRE_X = -59.8, GARDEN_PRE_Y = 46.0, GARDEN_HEADING_DEG = 105.0;
    public static double GARDEN_END_X = -59.8, GARDEN_END_Y = 58.5;
    /**
     * After the garden the robot squares on the rear wall: it turns to heading 90 (its left side to the wall) SQUARE_GAP_IN off the wall at the end's y, then strafes into the wall
     * (SquareOnWall), then drives forward into the Blue Wall (red: the Red Wall) to square on that too. The pose is not reseeded; the log lines say how far the pose was from the flush heading, x and y.
     */
    public static double SQUARE_GAP_IN = 3.0;
    /**
     * The one launch spot for the audience cell, used for BOTH the garden pollen and the flower pollen. Since 2026-11-03 it is on the flower's line (y = FLOWER_AT_Y), 50 in from
     * the audience wall side: 31.6 in from the cell (the launch map minimum is 28.6 in plus 3 in of leeway; x 47.8 would be 29.6), about 25 deg off the cell's axis. It was (63.6, 8.5) at the wall, 6 deg off-axis.
     * Its heading is the computed aim.
     */
    public static double LAUNCH_X = 50.0, LAUNCH_Y = 26.3;
    /** Deploy the ramp before the garden salvo (true: only if the launcher is known to work with the ramp down) or right after it, while the robot drives to the flower pre-position (false). */
    public static boolean RAMP_DEPLOY_DURING_LAUNCH = false;
    /**
     * Standing aim corrections for the two salvos from the launch spot, degrees, positive = counterclockwise (the launch lands further left of where it landed), negative = clockwise (further right);
     * the same for either alliance. If a salvo lands LEFT, make its trim negative. 2026-11-03: the garden salvo landed left (the aim step ended +2.2 to +2.3 deg, the robot already pointed to the
     * right of the aim) and the flower salvo went in (+0.9 deg). The AIM log line shows the trim.
     */
    public static double GARDEN_AIM_TRIM_DEG = -3.0, FLOWER_AIM_TRIM_DEG = -3.0;   // 2 deg clockwise from 2026-11-03, 1 more on 2026-11-04 (the launch was still a little left)
    /**
     * After the jostle the robot first backs STRAIGHT out (heading held 0) to FLOWER_AT_X - FLOWER_BACKOFF_IN (x 46.3, ending within BACKOUT_TOL_IN) and only then does the turn move to the launch spot (false, the default).
     * True goes straight to the launch spot turning as it backs: it caught the edge of the ramp on the flower on 2026-11-03 (the turn started at x 53.4, 4 in from the launch spot).
     */
    public static boolean FLOWER_TURN_WHILE_BACKING = false;
    /**
     * The ramp stows during the turn from the flower to the launch spot, as soon as the robot has turned this many degrees from the flower heading (the intake mouth is then facing away
     * from the flower and the robot is already 5 in or more back). The old stow waited for the robot to be 24 in from the flower centre, which the launch spot (18.7 in) never is, so the
     * ramp stowed only after arrival and the auto waited 0.81 s for it (2026-11-03 21:59 run).
     */
    public static double STOW_TURN_DEG = 45.0;
    /** The flower poses are 3 in toward +Y of the marks (21.3 to 24.3, user 2026-11-03: the pollen did not come out), and the pre-position is 5 in farther back (43 to 38) for a run-up. */
    public static double FLOWER_PRE_X = 38.0, FLOWER_PRE_Y = 26.3, FLOWER_HEADING_DEG = 0.0;   // y 21.3 (the marks) +3 then +2 more (2026-11-03)
    public static double FLOWER_AT_X = 55.3, FLOWER_AT_Y = 26.3;
    /**
     * The flower is RAMMED: the drive's target is FLOWER_RAM_OVERSHOOT_IN past FLOWER_AT_X, so the error stays large and the power stays at FLOWER_RAM_POWER until the robot
     * meets the flower, and the leg ends after FLOWER_RAM_SEC (it never reaches the target: the log says TIMED OUT, which is expected). The normal drive uses 0.05 x the
     * error, so a 12 in leg never passed 0.6.
     */
    public static double FLOWER_RAM_POWER = 0.6, FLOWER_RAM_OVERSHOOT_IN = 12.0, FLOWER_RAM_SEC = 0.6;   // 0.9 / 0.8 rammed hard on 2026-11-03 and was not enough; the jostle does the work
    /** After the ram the robot jostles (back FLOWER jostle BACK_IN, push, repeat: see Jostle) until it is full, for at most FLOWER_PAUSE_MAX_SEC; then it backs off this far before turning. */
    public static double FLOWER_BACKOFF_IN = 9.0;   // 10 from the old turn at the flower; now the straight back-out before the turn (x 46.3), which ends within BACKOUT_TOL_IN of it
    public static double BACKOUT_TOL_IN = 1.5;
    public static double PARK_X = 51.1, PARK_Y = 54.0, PARK_HEADING_DEG = 180.0;
    /**
     * The park run uses full power and is driven at a target PARK_OVERSHOOT_IN past the park pose (in the +Y direction of travel, blue frame; red is turned 180 deg), because the drive stalls about
     * 1.9 in short of its target (no static friction term) and the 2026-11-03 runs ended 1.86 in short at the timeout. The leg ends when within PARK_POS_TOL_IN of that shifted target, so the robot
     * has pushed through to the park pose by then, or after PARK_TIMEOUT_SEC.
     */
    public static double PARK_MAX_POWER = 1.0, PARK_OVERSHOOT_IN = 2.0, PARK_POS_TOL_IN = 2.5, PARK_HEADING_TOL_DEG = 5.0, PARK_TIMEOUT_SEC = 3.0;
    /**
     * The helper point between the garden and the launch spot, to go around the OUTSIDE of the hive's two support rails (along the row borders 2|3 and 4|5, columns C to D,
     * so y = +-23.7 from x = -23.7 to +23.7; the 2026-11-03 run stopped with its back corner on the rail end at about (-23.7, 23.7)). (36, 40) keeps the robot's
     * footprint at least 8 in from the rail end (controller simulation) and the path 145 in long. Its heading is the launch spot's aim heading.
     */
    public static double HELPER_X = 36.0, HELPER_Y = 40.0;
    /** Drive to a point level with the launch spot (x = PARK_X, y = LAUNCH_Y) before the park run. Off since the launch spot moved onto the flower's line: the run to the park is straight up the field, 15 in clear of the flower. */
    public static boolean PARK_VIA_POINT = false;

    // ---- timing ----
    /** At the flower: pause until the robot holds 4 elements, but no longer than this. */
    public static double FLOWER_PAUSE_MAX_SEC = 9.0;   // the jostle's own time limit; it is really bounded by JOSTLE_DEADLINE_SEC. It no longer stops early at 3 held: all 4 matter more (2026-11-04)
    /**
     * The jostle ends at this many seconds into the auto at the latest, whether or not the robot is full, so the back-out, the launch spot, the salvo and the park (about 5 s in the 2026-11-03 runs) still fit in
     * the 30 s with 2 s to spare. The robot gathers from the flower until it is full or this time, then moves on to the launch.
     */
    public static double JOSTLE_DEADLINE_SEC = 23.0;
    /** Each drive leg's hard cap is max(TIMEOUT_MIN_SEC, TIMEOUT_BASE_SEC + distance / TIMEOUT_IN_PER_S) + TIMEOUT_TURN_EXTRA_SEC for a turn of over 120 deg, never more than TIMEOUT_MAX_SEC. */
    public static double TIMEOUT_BASE_SEC = 1.0, TIMEOUT_IN_PER_S = 22.0, TIMEOUT_MIN_SEC = 1.5, TIMEOUT_TURN_EXTRA_SEC = 0.5, TIMEOUT_MAX_SEC = 8.0;
    /** Pass-through points (the garden, the helper, the park via point) only need to get near: a leg ends within this position and heading. The launch spots and the flower keep the tight DriveToPose tolerance. */
    public static double COARSE_POS_IN = 3.0, COARSE_HEADING_DEG = 10.0;
    /**
     * The garden pickup leg (along the rear wall, intake on) is driven slowly so the pollen get in: this is the most power any axis may use (the normal limit is 0.6;
     * speed is about 60 in/s per unit of power, so 0.25 is about 15 in/s, a 23 in leg in about 2 s). Raise it if the pickup is fine slower than needed, lower it if pollen are missed.
     */
    /**
     * The cross-field legs (the helper leg and the one after it, 100 and 42 in) use this power limit instead of 0.6 (the leg's hard cap speed scales with it). 0.6 took 2.9 s for the
     * 101 in leg on 2026-11-03.
     */
    public static double CROSS_FIELD_MAX_POWER = 0.85;
    /** The launch spots only need to be near (the aim step turns onto the cell): a leg ends within this position and heading; the tight 1 in / 2 deg tolerance ran two legs to their timeout each time. */
    public static double LAUNCH_POS_TOL_IN = 2.5, LAUNCH_HEADING_TOL_DEG = 6.0;
    public static double GARDEN_PICKUP_MAX_POWER = 0.35;   // 0.25 moved only about 5.6 in/s on 2026-11-03 and missed the pollen
    /** The leg's hard cap is worked out with a speed of this many in/s per unit of power (a little under the real speed, so it only launches when the robot is stuck). */
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

    /** A drive leg that must settle (the launch spots, the flower): the tight DriveToPose tolerance. */
    private Action leg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC));
    }

    /** A fast pass-through leg (the cross-field move): the power is capped at maxPower and the hard cap's speed scales with it. */
    private Action crossLeg(String label, MecanumDrive drive, Pose2d target, double maxPower, boolean pass) {
        DriveToPose d = new DriveToPose(drive, target).withMaxOutput(maxPower).withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S * maxPower / 0.6, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC);
        d.withTolerance(pass ? COARSE_POS_IN : LAUNCH_POS_TOL_IN, pass ? COARSE_HEADING_DEG : LAUNCH_HEADING_TOL_DEG);
        return timed("drive: " + label, d);
    }

    /** A launch-spot leg: near is enough (LAUNCH_POS_TOL_IN, LAUNCH_HEADING_TOL_DEG). */
    private Action launchLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withTolerance(LAUNCH_POS_TOL_IN, LAUNCH_HEADING_TOL_DEG)
                .withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC));
    }

    /** A pass-through leg: ends as soon as the robot is near. */
    private Action passLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withTolerance(COARSE_POS_IN, COARSE_HEADING_DEG)
                .withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC));
    }

    /** A straight back-out: the leg ends within BACKOUT_TOL_IN of the target (a pass-through leg ended 3 in away, which is not far enough from the flower). */
    private Action backOutLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withTolerance(BACKOUT_TOL_IN, COARSE_HEADING_DEG)
                .withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC));
    }

    /** Runs {@code inner} (a drive) and commands the ramp to stow as soon as the robot has turned STOW_TURN_DEG from flowerHeadingRad (once). */
    private Action stowWhenTurned(Action inner, MecanumDrive drive, Mechanisms mechanisms, double flowerHeadingRad) {
        return new Action() {
            private boolean stowed = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                boolean running = inner.run(packet);
                if (!stowed) {
                    Pose2d p = drive.getPose();
                    double turnedDeg = Math.toDegrees(Math.abs(Rotation2d.normalize(p.heading.toDouble() - flowerHeadingRad)));
                    if (turnedDeg >= STOW_TURN_DEG) {
                        stowed = true;
                        mechanisms.requestStowNow();
                        RunLog.event(String.format(Locale.US, "AUTO ramp stow during the turn: %.0f deg from the flower heading, pose (%.1f, %.1f)", turnedDeg, p.position.x, p.position.y));
                    }
                }
                return running;
            }
        };
    }

    /** The park run: full power, a tolerance wide enough to end once through the stall, a generous time limit. */
    private Action parkLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withMaxOutput(PARK_MAX_POWER).withTolerance(PARK_POS_TOL_IN, PARK_HEADING_TOL_DEG).withTimeout(PARK_TIMEOUT_SEC));
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

    /** Aim at the cell with the lock, launch one salvo, and finish when it is done. */
    private Action launch(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label) {
        return launch(drive, mechanisms, cell, blue, label, 0.0, false);
    }

    /** The jostle is done when the robot is full or JOSTLE_DEADLINE_SEC into the auto has passed (logged once). */
    private java.util.function.BooleanSupplier jostleDone(Mechanisms mechanisms) {
        return new java.util.function.BooleanSupplier() {
            private boolean logged = false;

            @Override
            public boolean getAsBoolean() {
                if (mechanisms.isFull()) return true;
                if (autoClock.seconds() >= JOSTLE_DEADLINE_SEC) {
                    if (!logged) {
                        logged = true;
                        RunLog.event(String.format(Locale.US, "AUTO jostle deadline reached at %.2f s into the auto: moving on to the launch with %d held", autoClock.seconds(),
                                mechanisms.counter == null ? -1 : mechanisms.counter.count()));
                    }
                    return true;
                }
                return false;
            }
        };
    }

    /** skipIfEmpty: the aim and the salvo are skipped (and logged) when the element counter is reading and holds nothing; any other count launches. */
    private Action launch(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label, double trimDeg, boolean skipIfEmpty) {
        Action real = launchNow(drive, mechanisms, cell, blue, label, trimDeg);
        if (!skipIfEmpty) return real;
        return new Action() {
            private boolean decided = false, skip = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!decided) {
                    decided = true;
                    skip = mechanisms.counter != null && mechanisms.counter.isValid() && mechanisms.counter.count() == 0;
                    if (skip) RunLog.event("AUTO SKIP salvo: " + label + " (the robot holds nothing)");
                }
                return !skip && real.run(packet);
            }
        };
    }

    private Action launchNow(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label, double trimDeg) {
        return new SequentialAction(
                note("Aim and launch: " + label),
                timed("aim: " + label, new AimAtCell(drive, cell, blue, trimDeg)),
                timed("launch (wait + salvo): " + label, mechanisms.launch()));
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
        RunLog.event(String.format(Locale.US, "PINPOINT START mode %s, seedConfirmed %s, bring-up %.0f ms: %s", loc.startMode, loc.seedConfirmed, loc.bringUpMs, loc.startNote));

        // the path for this alliance
        final boolean isBlue = blue;
        HiveTarget firstCell = isBlue ? HiveTarget.BLUE_REAR : HiveTarget.RED_AUDIENCE;    // as staged; the first tip flips it
        HiveTarget secondCell = firstCell.otherSide();                                     // blue: audience, red: rear
        Pose2d gardenPre = forAlliance(pose(GARDEN_PRE_X, GARDEN_PRE_Y, GARDEN_HEADING_DEG), isBlue);
        Pose2d gardenEnd = forAlliance(pose(GARDEN_END_X, GARDEN_END_Y, GARDEN_HEADING_DEG), isBlue);
        double sideIn = profile.originFromSideIn();   // the origin to the robot's side, for the wall-flush x
        double squareX = -(FieldGeometry.HALF_IN - sideIn - SQUARE_GAP_IN);
        Pose2d squarePre = forAlliance(pose(squareX, GARDEN_END_Y, 90.0), isBlue);   // heading 90: the left side to the rear wall (red: the audience wall)
        double squareFlushX = (isBlue ? -1.0 : 1.0) * (FieldGeometry.HALF_IN - sideIn);
        double squareFlushY = (isBlue ? 1.0 : -1.0) * (FieldGeometry.HALF_IN - profile.originFromFrontIn());   // the front (intake) against the Blue Wall (red: the Red Wall)
        double squareHeadingRad = Math.toRadians(isBlue ? 90.0 : 270.0);
        Pose2d launchSpotBlueFrame = pose(LAUNCH_X, LAUNCH_Y, 0.0);
        Pose2d launchXY = forAlliance(launchSpotBlueFrame, isBlue);
        double launchHeading = TargetLock.aimHeadingRad(launchXY.position.x, launchXY.position.y, secondCell, TargetLock.farCellExtraLeftIn(secondCell, isBlue));
        Pose2d launchSpot = new Pose2d(launchXY.position.x, launchXY.position.y, launchHeading);
        double blueLaunchHeadingDeg = Math.toDegrees(TargetLock.aimHeadingRad(LAUNCH_X, LAUNCH_Y, HiveTarget.BLUE_AUDIENCE, TargetLock.farCellExtraLeftIn(HiveTarget.BLUE_AUDIENCE, true)));
        Pose2d helper = forAlliance(pose(HELPER_X, HELPER_Y, blueLaunchHeadingDeg), isBlue);   // outside the support rails, at the launch spot's heading
        Pose2d flowerPre = forAlliance(pose(FLOWER_PRE_X, FLOWER_PRE_Y, FLOWER_HEADING_DEG), isBlue);
        Pose2d flowerBack = forAlliance(pose(FLOWER_AT_X - FLOWER_BACKOFF_IN, FLOWER_AT_Y, FLOWER_HEADING_DEG), isBlue);   // clear of the flower before the 180 deg turn
        Pose2d flowerRam = forAlliance(pose(FLOWER_AT_X + FLOWER_RAM_OVERSHOOT_IN, FLOWER_AT_Y, FLOWER_HEADING_DEG), isBlue);   // past the flower: the ram target
        Pose2d parkVia = forAlliance(pose(PARK_X, LAUNCH_Y, PARK_HEADING_DEG), isBlue);
        Pose2d park = forAlliance(pose(PARK_X, PARK_Y + PARK_OVERSHOOT_IN, PARK_HEADING_DEG), isBlue);   // pushed through the stall: see PARK_OVERSHOOT_IN

        RunLog.event(String.format(Locale.US, "SETUP profile %s, alliance %s, start pose (%.1f, %.1f, %.1f deg), cells %s then %s, launch spot (%.1f, %.1f, %.1f deg), %s",
                profile.name, isBlue ? "BLUE" : "RED", drive.getPose().position.x, drive.getPose().position.y, drive.getPose().heading.toDegrees(),
                firstCell, secondCell, launchSpot.position.x, launchSpot.position.y, launchSpot.heading.toDegrees(), mechanisms.describe()));
        RunLog.setSampler("x,y,hdg_deg," + Mechanisms.SAMPLE_COLUMNS, 100, () -> {
            Pose2d p = drive.getPose();
            return String.format(Locale.US, "%.2f,%.2f,%.1f,%s", p.position.x, p.position.y, p.heading.toDegrees(), mechanisms.sampleRow());
        });

        mechanisms.setBlueAlliance(isBlue);
        mechanisms.setOpenCell(firstCell);
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel stays up between launches ...
        mechanisms.setSpinRequested(true);         // ... and spins from Start, so it is at speed on arrival at each launch spot

        Action auto = new SequentialAction(
                launch(drive, mechanisms, firstCell, isBlue, "preload"),

                note("Garden pre-position"),
                passLeg("garden pre-position", drive, gardenPre),
                note("Intake ON: garden, slow"),
                mechanisms.setIntake(true),
                slowLeg("garden pickup (slow)", drive, gardenEnd, GARDEN_PICKUP_MAX_POWER),
                note("Square on the wall"),
                passLeg("square approach", drive, squarePre),
                timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY)),

                new InstantAction(() -> mechanisms.setOpenCell(secondCell)),   // the tip flipped the cell; the flywheel follows the new distance on the way
                note("Around the outside of the hive support rails, intake still ON so every element gets in"),
                crossLeg("helper (outside the support rails)", drive, helper, CROSS_FIELD_MAX_POWER, true),
                note("To the launch spot"),
                crossLeg("launch spot 1", drive, launchSpot, CROSS_FIELD_MAX_POWER, false),
                note("Intake OFF"),
                mechanisms.setIntake(false),
                RAMP_DEPLOY_DURING_LAUNCH ? new InstantAction(mechanisms::requestDeploy) : new InstantAction(() -> { }),   // deploys while it launches
                launch(drive, mechanisms, secondCell, isBlue, "garden pollen", GARDEN_AIM_TRIM_DEG, true),

                RAMP_DEPLOY_DURING_LAUNCH ? new InstantAction(() -> { }) : new InstantAction(mechanisms::requestDeploy),   // deploys on the way
                note("Back to the flower pre-position, turning to the flower"),
                leg("flower pre-position", drive, flowerPre),
                timed("ramp deploy (wait)", rampTo(mechanisms, true)),         // fully deployed before it goes on; no second command
                note("Intake ON: flower"),
                mechanisms.setIntake(true),
                ramLeg("ram the flower", drive, flowerRam, FLOWER_RAM_POWER, FLOWER_RAM_SEC),
                timed("flower jostle", new Jostle(drive, jostleDone(mechanisms), () -> mechanisms.counter == null ? 0 : mechanisms.counter.count(), FLOWER_PAUSE_MAX_SEC)),
                note("Back off the flower to the launch spot (intake still ON)"),
                FLOWER_TURN_WHILE_BACKING ? new InstantAction(() -> { }) : backOutLeg("flower back-out (straight)", drive, flowerBack),
                stowWhenTurned(launchLeg("launch spot 2", drive, launchSpot), drive, mechanisms, flowerPre.heading.toDouble()),   // the ramp stows during the turn
                mechanisms.setIntake(false),
                timed("ramp stow (wait)", rampTo(mechanisms, false)),
                launch(drive, mechanisms, secondCell, isBlue, "flower pollen", FLOWER_AIM_TRIM_DEG, true),

                note("Park"),
                PARK_VIA_POINT ? passLeg("park via point", drive, parkVia) : new InstantAction(() -> { }),
                parkLeg("park (full power)", drive, park),
                new InstantAction(mechanisms::stopFlywheel)
        );

        Actions.runBlocking(this, auto, () -> mechanisms.update(drive.getPose()));
        RunLog.event(isStopRequested() ? "AUTO stopped before it finished" : "AUTO actions finished");
        RunLog.event(String.format(Locale.US, "AUTO TIMING total %.2f s into the auto; steps (s): %s", autoClock.seconds(), String.join("; ", timings)));

        // hand the final pose (and the alliance) to the TeleOp, whether the auto finished or was stopped
        drive.updatePoseEstimate();
        PoseStorage.save(drive.getPose(), isBlue);
        RunLog.event(String.format(Locale.US, "PINPOINT handed to the TeleOP: pose (%.2f, %.2f, %.1f deg), alliance %s, device %s (the TeleOp adopts the device's own pose if it starts within the age limit)",
                drive.getPose().position.x, drive.getPose().position.y, drive.getPose().heading.toDegrees(), isBlue ? "BLUE" : "RED", loc.getDeviceStatus()));

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
