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
import org.firstinspires.ftc.teamcode.drive.LaunchGridMap;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
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
 * SOLO AUTO ON THE 2-D LAUNCH GRID (2026-11-19, the user's decision; not run on the robot). Until then the audience-cell launch spot (for the garden pollen and the flower pollen) was (50.0, 26.3), which the
 * grid measured as a NO SHOT zone (the nearest measured spot, (48, 24), could not score). It is now (48.0, 36.0), the grid's measured spot 18 of the far cell (GOOD, 35.3 in from the cell, 12 in from the
 * nearest no-shot spot, 12 in from the flower pose and 12.6 in from the helper point). From there the flywheel speed and the aim come from the grid (the grid is switched on after the preload, which keeps the
 * launch map and its trim from the start wall, where the grid only extrapolates), and the aim is the grid's: the bearing plus the spot's residual, with no trim (the old -2.5 deg trims were for the
 * formula aim at the old spot). The legs, the flower, the final gather and the park are unchanged. The earlier version is in the git history (before the commit that made this change).
 *
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
 *   6. FINAL GATHER (2026-11-05, FINAL_FLOWER_ENABLED, always runs when enabled): around the outside of the hive via the helper point to the flower on the robot's own
 *      alliance wall (the Blue Wall flower for blue), ram, a one-cycle jostle (ends by FINAL_JOSTLE_DEADLINE_SEC), back out, the ramp stows, and park in the hive-side corner of the loading zone
 *      (FINAL_PARK_X / Y); the intake is held IN from the ram to the park and the pollen stay for the TeleOp. THIRD GARDEN SALVO (2026-11-19, user, THIRD_SALVO_ENABLED): before that park the robot drives to the
 *      grid's BLUE_AUDIENCE spot 2 (36.3, 47.7) and launches what it holds into the same cell as the garden and flower salvos; the jostle ends by THIRD_JOSTLE_DEADLINE_SEC; the park is tried after it. Otherwise: the old park (straight from the launch spot; PARK_VIA_POINT can add a point level with it).
 *
 * The launch spot's heading is not snapped: it is the heading that points the launcher's line at the cell from there ({@link TargetLock#aimHeadingRad},
 * with the launcher offset; the far-cell extra is not used in the autos from 2026-11-17, each launch's trim holds its whole allowance), and {@link AimAtCell} takes out what is left before each launch. The straight run from the garden
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
     * THE GARDEN IN ONE MOVEMENT (2026-11-19, user; the MIDDLE version): ONE leg from the start straight to the END of the garden pickup (GARDEN_END, at the angled heading GARDEN_HEADING_DEG), the last
     * GARDEN_SLOW_ZONE_IN inches limited to GARDEN_SLOW_POWER (12.5 in: four pollen of about 2.8 in end to end plus margin), then the squaring approach and the squaring as before. It replaces the angled
     * pre-position leg and the slow pickup leg (about 1.9 s together on the 2026-11-07 Solo runs). The first idea, the garden of Duo Daryl's first version (one leg to the squaring pose at heading 90), ended 2.9 to
     * 3.5 in off the wall on 2026-11-07: the side squaring was 'stopped by the wall' with the pose still 2.9 in from the flush x (the angled garden's runs ended 0.6 in off), probably pollen pinched between the
     * side and the wall; the angled approach (the mouth turned toward the wall) keeps them in front of the mouth, so this version keeps the angle. Not run. False: the pre-position and the pickup as two legs.
     */
    public static boolean GARDEN_ONE_LEG = true;
    public static double GARDEN_SLOW_ZONE_IN = 12.5, GARDEN_SLOW_POWER = 0.5;
    /**
     * The one launch spot for the audience cell, used for BOTH the garden pollen and the flower pollen. Since 2026-11-03 it is on the flower's line (y = FLOWER_AT_Y), 50 in from
     * the audience wall side: 31.6 in from the cell (the launch map minimum is 28.6 in plus 3 in of leeway; x 47.8 would be 29.6), about 25 deg off the cell's axis. It was (63.6, 8.5) at the wall, 6 deg off-axis.
     * Its heading is the computed aim.
     */
    public static double LAUNCH_X = 48.0, LAUNCH_Y = 36.0;   // the grid's measured far-cell spot 18 (was 50.0, 26.3, a NO SHOT zone of the grid)
    /** Deploy the ramp before the garden salvo (true: only if the launcher is known to work with the ramp down) or right after it, while the robot drives to the flower pre-position (false). */
    public static boolean RAMP_DEPLOY_DURING_LAUNCH = false;
    /**
     * Standing aim corrections for the two salvos from the launch spot, degrees, positive = counterclockwise (the launch lands further left of where it landed), negative = clockwise (further right);
     * the same for either alliance. If a salvo lands LEFT, make its trim negative. 2026-11-03: the garden salvo landed left (the aim step ended +2.2 to +2.3 deg, the robot already pointed to the
     * right of the aim) and the flower salvo went in (+0.9 deg). The AIM log line shows the trim.
     */
    // 2026-11-17 (user): ONE trim per launch. The far-cell extra (TargetLock.FAR_CELL_EXTRA_LEFT_IN) is no longer applied in the autos: at this spot it turned the robot about +5.5 deg,
    // against the -8.0 deg these trims then held. -2.5 is the net of the two, so the launches aim where they did (identical at the spot itself; up to about half a degree different when
    // the leg ends 2.5 in off it, because the extra's angle depended on the distance). The history after the values is of the -8.0.
    public static double GARDEN_AIM_TRIM_DEG = 0.0, FLOWER_AIM_TRIM_DEG = 0.0;   // 0 with the grid aim (the old -2.5 belonged to the formula aim at the old spot); a standing correction on top of the grid's residual, degrees, + = counterclockwise   // 2 deg clockwise from 2026-11-03, 1 more on 2026-11-04 (the launch was still a little left), 1.5 more on 2026-11-07 (the red run's rear-cell salvos landed left), and 1.5 more later that day, then 2 more (-8.0) together with the aim breakaway, which removes the stall that left the robot about 1 deg short of the trimmed aim (the blue runs still landed left; the aim step stops about 1 deg short of the trimmed aim, so the real shift is about 1 deg less than the trim)
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
    /**
     * The final gather's wall flower legs (2026-11-05, from the 23:50 and 23:59 logs): the pre-position stalled 1.9 in short of its 1 in tolerance and ran to its 3.07 s timeout both times, and the back-out
     * stalled 1.7 in short of 1.5 in (1.53 s timeout); the P term gives only about 0.1 power that close, which does not move the robot. The pre-position ends in a 12 in ram and the back-out only has to clear
     * the flower, so near is enough.
     */
    public static double WALL_PRE_POS_TOL_IN = 2.5, WALL_PRE_HEADING_TOL_DEG = 4.0, WALL_BACKOUT_TOL_IN = 2.5;
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
    /**
     * FINAL GATHER (2026-11-05, user): after the flower salvo the robot gathers the flower on its OWN alliance wall (the Blue Wall flower for blue, the audience-wall flower's poses turned 90 deg
     * counterclockwise, as the Duo Garden Side does; red is turned 180 deg), keeps what it gets for the TeleOp, and parks in the OTHER (hive-side) corner of the loading zone at (FINAL_PARK_X, FINAL_PARK_Y)
     * instead of the old park beside the audience flower. It always runs when enabled (the start-time guard was removed 2026-11-06, user: only the jostle deadline is needed), the jostle there ends at
     * FINAL_JOSTLE_DEADLINE_SEC so the park has its time (about 1.0 s back-out plus 1.5 s park), and the intake is held IN from the ram to the park. Nominal: about 29 s with a full one-cycle jostle.
     */
    public static boolean FINAL_FLOWER_ENABLED = true;
    public static double FINAL_JOSTLE_DEADLINE_SEC = 26.5;
    /** FINAL_PARK_Y 53.1 (2026-11-07, user): the front edge (9.88 in ahead of the origin) ends 8 in from the wall (it was 60, 1.1 in from it) and still 3.9 in inside the loading zone (it starts at y 59.1). */
    public static double FINAL_PARK_X = 26.0, FINAL_PARK_Y = 53.1, FINAL_PARK_HEADING_DEG = 90.0, FINAL_PARK_OVERSHOOT_IN = 0.0;
    /** The helper point outside the support rails for the trip back across, and the heading it is taken at (along the leg, toward -X). Nudge of the wall flower poses (blue frame, before the alliance turn). */
    public static double FINAL_HELPER_HEADING_DEG = 180.0;
    public static double FINAL_FLOWER_DX_IN = 0.0, FINAL_FLOWER_DY_IN = 0.0;
    /**
     * THIRD GARDEN SALVO (2026-11-19, user: one near salvo (the preload), then THREE garden salvos): after the wall flower the robot launches what it holds into the SAME cell as the garden and flower salvos
     * (blue: the audience cell), and THEN tries to park in the hive-side corner whether or not there is time left (the auto simply ends at 30 s). The launch spot is the 2-D launch grid's measured spot 2 of
     * BLUE_AUDIENCE, (36.3, 47.7): 38.0 in from the cell, speed 1350, residual +4.35 deg, a straight run along y about 47 from the wall flower's back-out (-26.3, 46.3), 62.6 in, clear of the hive, and
     * 11.6 in from the park pose (spot 3, 11 in nearer the hive, is NO SHOT). Red is the same turned 180 deg. The jostle ends at THIRD_JOSTLE_DEADLINE_SEC so the back-out, the run (about 2 s), the aim and the
     * salvo (1.4 s) end near 29 s; the park after it is likely cut by the end of the auto. False: no third salvo, the park as before (and the jostle ends at FINAL_JOSTLE_DEADLINE_SEC).
     */
    public static boolean THIRD_SALVO_ENABLED = true;
    public static double THIRD_LAUNCH_X = 36.3, THIRD_LAUNCH_Y = 47.7, THIRD_JOSTLE_DEADLINE_SEC = 24.0, THIRD_AIM_TRIM_DEG = 0.0;

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
    public static double CROSS_FIELD_MAX_POWER = 1.0;   // 0.85 until 2026-11-05 (3.01 s for the helper leg); the dashes run at 0.85 without trouble and this leg is outside the support rails
    /**
     * Every other leg (garden and flower pre-positions, the square approach, the back-outs, the launch spot after the flower) runs at this power limit instead of the drive's 0.6 (2026-11-05, to gain time;
     * the hard cap's speed scales with it). Only a leg longer than about 1/kP times this (17 in at 0.85) ever reaches it. 0.6 restores the old speed.
     */
    public static double LEG_MAX_POWER = 0.85;
    /** Pass-through legs (the garden pre-position, the square approach, the helper legs) end the moment they are within the coarse tolerance instead of waiting the drive's 0.20 s settle; the next leg takes over at speed. */
    public static double PASS_SETTLE_SEC = 0.0;
    /** The launch spots only need to be near (the aim step turns onto the cell): a leg ends within this position and heading; the tight 1 in / 2 deg tolerance ran two legs to their timeout each time. */
    public static double LAUNCH_POS_TOL_IN = 2.5, LAUNCH_HEADING_TOL_DEG = 6.0;
    public static double GARDEN_PICKUP_MAX_POWER = 0.35;   // 0.25 moved only about 5.6 in/s on 2026-11-03 and missed the pollen
    /** The leg's hard cap is worked out with a speed of this many in/s per unit of power (a little under the real speed, so it only launches when the robot is stuck). */
    public static double SLOW_IN_PER_S_PER_POWER = 40.0;

    private static Pose2d pose(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    /** The blue-frame pose as this alliance sees it: red is the blue pose turned 180 deg about the field center. */
    /** A blue-frame pose turned 90 deg counterclockwise about the field center (the audience-wall flower's poses become the Blue Wall flower's). */
    private static Pose2d rotate90(Pose2d p) {
        return new Pose2d(-p.position.y, p.position.x, p.heading.toDouble() + Math.PI / 2.0);
    }

    /** Adds the Dashboard nudge to a blue-frame wall flower pose. */
    private static Pose2d wallNudge(Pose2d p) {
        return new Pose2d(p.position.x + FINAL_FLOWER_DX_IN, p.position.y + FINAL_FLOWER_DY_IN, p.heading.toDouble());
    }

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
        return timed("drive: " + label, legDrive(drive, target));
    }

    /** A drive at LEG_MAX_POWER whose hard cap's speed scales with the power. */
    private DriveToPose legDrive(MecanumDrive drive, Pose2d target) {
        return new DriveToPose(drive, target).withMaxOutput(LEG_MAX_POWER)
                .withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S * LEG_MAX_POWER / 0.6, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC);
    }

    /** A fast pass-through leg (the cross-field move): the power is capped at maxPower and the hard cap's speed scales with it. */
    private Action crossLeg(String label, MecanumDrive drive, Pose2d target, double maxPower, boolean pass) {
        DriveToPose d = new DriveToPose(drive, target).withMaxOutput(maxPower).withDistanceTimeout(TIMEOUT_BASE_SEC, TIMEOUT_IN_PER_S * maxPower / 0.6, TIMEOUT_MIN_SEC, TIMEOUT_TURN_EXTRA_SEC, TIMEOUT_MAX_SEC);
        d.withTolerance(pass ? COARSE_POS_IN : LAUNCH_POS_TOL_IN, pass ? COARSE_HEADING_DEG : LAUNCH_HEADING_TOL_DEG);
        if (pass) d.withSettle(PASS_SETTLE_SEC);
        return timed("drive: " + label, d);
    }

    /** A launch-spot leg: near is enough (LAUNCH_POS_TOL_IN, LAUNCH_HEADING_TOL_DEG). */
    private Action launchLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, legDrive(drive, target).withTolerance(LAUNCH_POS_TOL_IN, LAUNCH_HEADING_TOL_DEG));
    }

    /** A pass-through leg: ends as soon as the robot is near. */
    private Action passLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, legDrive(drive, target).withTolerance(COARSE_POS_IN, COARSE_HEADING_DEG).withSettle(PASS_SETTLE_SEC));
    }

    /** The garden in one movement: one leg to the end of the pickup, the last GARDEN_SLOW_ZONE_IN inches at GARDEN_SLOW_POWER. */
    private Action gardenLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, legDrive(drive, target).withTolerance(COARSE_POS_IN, COARSE_HEADING_DEG).withSlowZone(GARDEN_SLOW_ZONE_IN, GARDEN_SLOW_POWER).withSettle(PASS_SETTLE_SEC));
    }

    /** A straight back-out: the leg ends within BACKOUT_TOL_IN of the target (a pass-through leg ended 3 in away, which is not far enough from the flower). */
    private Action backOutLeg(String label, MecanumDrive drive, Pose2d target) {
        return backOutLeg(label, drive, target, BACKOUT_TOL_IN);
    }

    private Action backOutLeg(String label, MecanumDrive drive, Pose2d target, double tolIn) {
        return timed("drive: " + label, legDrive(drive, target).withTolerance(tolIn, COARSE_HEADING_DEG));
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

    /** Decides once, when it first runs, which of two actions to run. */
    private Action choose(java.util.function.BooleanSupplier condition, Action ifTrue, Action ifFalse) {
        return new Action() {
            private Action chosen = null;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (chosen == null) chosen = condition.getAsBoolean() ? ifTrue : ifFalse;
                return chosen.run(packet);
            }
        };
    }

    /** Aim at the cell with the lock, launch one salvo, and finish when it is done. */
    private Action launch(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label) {
        return launch(drive, mechanisms, cell, blue, label, 0.0);
    }

    /** The jostle is done when the robot is full or JOSTLE_DEADLINE_SEC into the auto has passed (logged once). */
    private java.util.function.BooleanSupplier jostleDone(Mechanisms mechanisms) {
        return jostleDone(mechanisms, JOSTLE_DEADLINE_SEC);
    }

    private java.util.function.BooleanSupplier jostleDone(Mechanisms mechanisms, double deadlineSec) {
        return new java.util.function.BooleanSupplier() {
            private boolean logged = false;

            @Override
            public boolean getAsBoolean() {
                if (mechanisms.isFull()) return true;
                if (autoClock.seconds() >= deadlineSec) {
                    if (!logged) {
                        logged = true;
                        RunLog.event(String.format(Locale.US, "AUTO jostle deadline (%.1f s) reached at %.2f s into the auto: moving on with %d held", deadlineSec, autoClock.seconds(),
                                mechanisms.getHeldCount()));
                    }
                    return true;
                }
                return false;
            }
        };
    }

    /** Aim at the cell and launch one salvo, ALWAYS (the user, 2026-11-06: whatever the element counter says; a counter that under-reads must never throw a salvo away). */
    private Action launch(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label, double trimDeg) {
        return launch(drive, mechanisms, cell, blue, label, trimDeg, false);
    }

    /** grid: aim as the TeleOp's lock does with the 2-D launch grid (the bearing plus the grid's residual at this pose), else the formula aim. */
    private Action launch(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label, double trimDeg, boolean grid) {
        AimAtCell aim = new AimAtCell(drive, cell, blue, trimDeg, 0.0);
        if (grid) aim.withGrid();
        return new SequentialAction(
                note("Aim and launch: " + label),
                timed("aim: " + label, aim),   // 0.0: no far-cell extra in the autos (2026-11-17), the trim is the whole allowance
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
                    : mechanisms.counter.isValid() ? mechanisms.counter.count() + " elements seen (expect 3 with the 4 loaded: CS1, CS3 and CS4; CS2 is on the feed ramp)   " + mechanisms.counter.stackString()
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
        // the launch spots' headings include the salvo's standing aim trim (2026-11-07), so the leg arrives pointed at the trimmed aim and the aim step only fine-tunes (the trim is the same clockwise degrees for either alliance)
        // the grid's aim at the launch spot: the bearing to the cell plus the spot's residual (the formula aim where the grid has no data)
        LaunchGridMap.Result gridAtSpot = LaunchGridMap.lookup(secondCell, launchXY.position.x, launchXY.position.y);
        double launchHeading = gridAtSpot.hasValues()
                ? Math.atan2(secondCell.y - launchXY.position.y, secondCell.x - launchXY.position.x) + Math.toRadians(gridAtSpot.residualDeg)
                : TargetLock.aimHeadingRad(launchXY.position.x, launchXY.position.y, secondCell, 0.0);
        Pose2d launchSpot = new Pose2d(launchXY.position.x, launchXY.position.y, launchHeading + Math.toRadians(GARDEN_AIM_TRIM_DEG));
        Pose2d launchSpotFlower = new Pose2d(launchXY.position.x, launchXY.position.y, launchHeading + Math.toRadians(FLOWER_AIM_TRIM_DEG));
        LaunchGridMap.Result gridBlue = LaunchGridMap.lookup(HiveTarget.BLUE_AUDIENCE, LAUNCH_X, LAUNCH_Y);
        double blueLaunchHeadingDeg = (gridBlue.hasValues()
                ? Math.toDegrees(Math.atan2(HiveTarget.BLUE_AUDIENCE.y - LAUNCH_Y, HiveTarget.BLUE_AUDIENCE.x - LAUNCH_X)) + gridBlue.residualDeg
                : Math.toDegrees(TargetLock.aimHeadingRad(LAUNCH_X, LAUNCH_Y, HiveTarget.BLUE_AUDIENCE, 0.0))) + GARDEN_AIM_TRIM_DEG;
        Pose2d helper = forAlliance(pose(HELPER_X, HELPER_Y, blueLaunchHeadingDeg), isBlue);   // outside the support rails, at the launch spot's heading
        Pose2d flowerPre = forAlliance(pose(FLOWER_PRE_X, FLOWER_PRE_Y, FLOWER_HEADING_DEG), isBlue);
        Pose2d flowerBack = forAlliance(pose(FLOWER_AT_X - FLOWER_BACKOFF_IN, FLOWER_AT_Y, FLOWER_HEADING_DEG), isBlue);   // clear of the flower before the 180 deg turn
        Pose2d flowerRam = forAlliance(pose(FLOWER_AT_X + FLOWER_RAM_OVERSHOOT_IN, FLOWER_AT_Y, FLOWER_HEADING_DEG), isBlue);   // past the flower: the ram target
        Pose2d parkVia = forAlliance(pose(PARK_X, LAUNCH_Y, PARK_HEADING_DEG), isBlue);
        Pose2d park = forAlliance(pose(PARK_X, PARK_Y + PARK_OVERSHOOT_IN, PARK_HEADING_DEG), isBlue);   // pushed through the stall: see PARK_OVERSHOOT_IN
        // the final gather: the alliance-wall flower (the audience flower's poses turned 90 deg, plus the Dashboard nudge) and the hive-side corner of the loading zone
        Pose2d wallPre = forAlliance(wallNudge(rotate90(pose(FLOWER_PRE_X, FLOWER_PRE_Y, FLOWER_HEADING_DEG))), isBlue);
        Pose2d wallRam = forAlliance(wallNudge(rotate90(pose(FLOWER_AT_X + FLOWER_RAM_OVERSHOOT_IN, FLOWER_AT_Y, FLOWER_HEADING_DEG))), isBlue);
        Pose2d wallBack = forAlliance(wallNudge(rotate90(pose(FLOWER_AT_X - FLOWER_BACKOFF_IN, FLOWER_AT_Y, FLOWER_HEADING_DEG))), isBlue);
        Pose2d finalHelper = forAlliance(pose(HELPER_X, HELPER_Y, FINAL_HELPER_HEADING_DEG), isBlue);
        // the third salvo's spot: the grid's measured BLUE_AUDIENCE spot 2, the aim the bearing to the cell plus the spot's residual (the formula aim where the grid has no data)
        HiveTarget thirdCell = secondCell;   // the same cell as the garden and flower salvos (user, 2026-11-19)
        Pose2d thirdXY = forAlliance(pose(THIRD_LAUNCH_X, THIRD_LAUNCH_Y, 0.0), isBlue);
        LaunchGridMap.Result gridThird = LaunchGridMap.lookup(thirdCell, thirdXY.position.x, thirdXY.position.y);
        double thirdHeading = gridThird.hasValues()
                ? Math.atan2(thirdCell.y - thirdXY.position.y, thirdCell.x - thirdXY.position.x) + Math.toRadians(gridThird.residualDeg)
                : TargetLock.aimHeadingRad(thirdXY.position.x, thirdXY.position.y, thirdCell, 0.0);
        Pose2d thirdSpot = new Pose2d(thirdXY.position.x, thirdXY.position.y, thirdHeading + Math.toRadians(THIRD_AIM_TRIM_DEG));
        RunLog.event("AUTO third launch spot " + THIRD_LAUNCH_X + ", " + THIRD_LAUNCH_Y + " (blue frame) " + (THIRD_SALVO_ENABLED ? "ON" : "OFF (the park runs)") + " in the grid: " + gridThird.describe());
        Pose2d finalPark = forAlliance(pose(FINAL_PARK_X, FINAL_PARK_Y + FINAL_PARK_OVERSHOOT_IN, FINAL_PARK_HEADING_DEG), isBlue);

        RunLog.event("AUTO launch spot " + LAUNCH_X + ", " + LAUNCH_Y + " (blue frame) in the grid: " + gridAtSpot.describe());
        RunLog.event(String.format(Locale.US, "SETUP profile %s, alliance %s, start pose (%.1f, %.1f, %.1f deg), cells %s then %s, launch spot (%.1f, %.1f, %.1f deg), %s",
                profile.name, isBlue ? "BLUE" : "RED", drive.getPose().position.x, drive.getPose().position.y, drive.getPose().heading.toDegrees(),
                firstCell, secondCell, launchSpot.position.x, launchSpot.position.y, launchSpot.heading.toDegrees(), mechanisms.describe()));
        RunLog.setSampler("x,y,hdg_deg,cmd_fwd,cmd_left,cmd_turn," + Mechanisms.SAMPLE_COLUMNS + ",a_lf,a_lb,a_rb,a_rf", 100, () -> {
            Pose2d p = drive.getPose();
            PoseVelocity2d dc = drive.getLastCommand();   // the drive command in the sample rows (from 2026-11-07), as in the TeleOp: what the legs and the jostle sent
            return String.format(Locale.US, "%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%s", p.position.x, p.position.y, p.heading.toDegrees(), dc.linearVel.x, dc.linearVel.y, dc.angVel, mechanisms.sampleRow() + "," + drive.wheelAmpsRow());
        });

        mechanisms.setBlueAlliance(isBlue);
        mechanisms.setIntakeAlwaysOn(true);   // the intake runs IN for the whole auto (user, 2026-11-07), whatever the count says; only a reject or an outtake runs it OUT
        mechanisms.setOpenCell(firstCell);
        mechanisms.setJamRetries(Pusher.JAM_RETRIES_AUTO);   // one jam back-off per pusher move: the autos are tight on time
        PoseStorage.saveAims(null);   // this auto reset and seeded the Pinpoint: the remembered aims an earlier TeleOp saved corrected the old pose's drift
        mechanisms.setHeldByEntries(Mechanisms.AUTO_PRELOAD_HELD, "the auto's preload");   // CS5 never saw the preload come in: the count starts at 4, not 0 (2026-11-17)
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel stays up between launches ...
        mechanisms.setSpinRequested(true);         // ... and spins from Start, so it is at speed on arrival at each launch spot

        Action auto = new SequentialAction(
                new InstantAction(mechanisms::preparePusherForLaunch),   // the pusher moves to the lock position while the flywheel spins up: the preload salvo turns 310 deg instead of 400
                launch(drive, mechanisms, firstCell, isBlue, "preload"),
                new InstantAction(() -> mechanisms.setLaunchGrid(true, "Auton - Solo, after the preload")),   // from here the flywheel speed follows the 2-D launch grid (the preload kept the launch map)

                GARDEN_ONE_LEG
                        ? new SequentialAction(
                                note("Intake ON: garden in ONE leg from the start to the end of the pickup at the angled heading, the last " + GARDEN_SLOW_ZONE_IN + " in at limited power"),
                                mechanisms.setIntake(true),
                                gardenLeg("garden (start to the pickup end)", drive, gardenEnd),
                                note("Square on the wall"),
                                passLeg("square approach", drive, squarePre),
                                timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY)))
                        : new SequentialAction(
                                note("Garden pre-position"),
                                passLeg("garden pre-position", drive, gardenPre),
                                note("Intake ON: garden, slow"),
                                mechanisms.setIntake(true),
                                slowLeg("garden pickup (slow)", drive, gardenEnd, GARDEN_PICKUP_MAX_POWER),
                                note("Square on the wall"),
                                passLeg("square approach", drive, squarePre),
                                timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY))),

                new InstantAction(() -> mechanisms.setOpenCell(secondCell)),   // the tip flipped the cell; the flywheel follows the new distance on the way
                note("Around the outside of the hive support rails, intake still ON so every element gets in"),
                crossLeg("helper (outside the support rails)", drive, helper, CROSS_FIELD_MAX_POWER, true),
                note("To the launch spot"),
                crossLeg("launch spot 1", drive, launchSpot, CROSS_FIELD_MAX_POWER, false),
                RAMP_DEPLOY_DURING_LAUNCH ? new InstantAction(mechanisms::requestDeploy) : new InstantAction(() -> { }),   // deploys while it launches
                launch(drive, mechanisms, secondCell, isBlue, "garden pollen", GARDEN_AIM_TRIM_DEG, true),

                RAMP_DEPLOY_DURING_LAUNCH ? new InstantAction(() -> { }) : new InstantAction(mechanisms::requestDeploy),   // deploys on the way
                note("Back to the flower pre-position, turning to the flower"),
                leg("flower pre-position", drive, flowerPre),
                timed("ramp deploy (wait)", rampTo(mechanisms, true)),         // fully deployed before it goes on; no second command
                note("Intake ON: flower (held IN, whatever the count, until the flower salvo is done)"),
                mechanisms.setIntake(true),
                mechanisms.forceIntakeIn(true),
                ramLeg("ram the flower", drive, flowerRam, FLOWER_RAM_POWER, FLOWER_RAM_SEC),
                timed("flower jostle", new Jostle(drive, jostleDone(mechanisms), () -> mechanisms.counter == null ? 0 : mechanisms.counter.count(), FLOWER_PAUSE_MAX_SEC)),
                note("Back off the flower to the launch spot (intake still ON)"),
                FLOWER_TURN_WHILE_BACKING ? new InstantAction(() -> { }) : backOutLeg("flower back-out (straight)", drive, flowerBack),
                stowWhenTurned(launchLeg("launch spot 2", drive, launchSpotFlower), drive, mechanisms, flowerPre.heading.toDouble()),   // the ramp stows during the turn
                timed("ramp stow (wait)", rampTo(mechanisms, false)),
                launch(drive, mechanisms, secondCell, isBlue, "flower pollen", FLOWER_AIM_TRIM_DEG, true),

                choose(() -> {
                            boolean go = FINAL_FLOWER_ENABLED;
                            RunLog.event(String.format(Locale.US, "AUTO %s at %.2f s into the auto",
                                    go ? "FINAL GATHER starts" : "FINAL GATHER disabled: the old park runs", autoClock.seconds()));
                            return go;
                        },
                        new SequentialAction(
                                note("Final gather: the alliance-wall flower, then the hive-side corner"),
                                new InstantAction(mechanisms::requestDeploy),   // deploys on the way
                                crossLeg("final helper (outside the support rails)", drive, finalHelper, CROSS_FIELD_MAX_POWER, true),
                                timed("drive: wall flower pre-position", legDrive(drive, wallPre).withTolerance(WALL_PRE_POS_TOL_IN, WALL_PRE_HEADING_TOL_DEG)),
                                timed("ramp deploy (wait)", rampTo(mechanisms, true)),
                                note("Intake ON: wall flower (held IN until the park)"),
                                mechanisms.setIntake(true),
                                mechanisms.forceIntakeIn(true),
                                ramLeg("ram the wall flower", drive, wallRam, FLOWER_RAM_POWER, FLOWER_RAM_SEC),
                                timed("wall flower jostle", new Jostle(drive, jostleDone(mechanisms, THIRD_SALVO_ENABLED ? THIRD_JOSTLE_DEADLINE_SEC : FINAL_JOSTLE_DEADLINE_SEC),
                                        () -> mechanisms.counter == null ? 0 : mechanisms.counter.count(), FLOWER_PAUSE_MAX_SEC)),
                                backOutLeg("wall flower back-out (straight)", drive, wallBack, WALL_BACKOUT_TOL_IN),
                                new InstantAction(mechanisms::requestStowNow),   // clear of the flower: the ramp stows on the way to the corner (or to the third launch spot)
                                THIRD_SALVO_ENABLED
                                        ? new SequentialAction(
                                                note("Third garden salvo: the wall flower's pollen into the same cell, then the park is tried"),
                                                crossLeg("third launch spot", drive, thirdSpot, CROSS_FIELD_MAX_POWER, false),
                                                timed("ramp stow (wait)", rampTo(mechanisms, false)),
                                                launch(drive, mechanisms, thirdCell, isBlue, "wall flower pollen", THIRD_AIM_TRIM_DEG, true))
                                        : new InstantAction(() -> { }),
                                parkLeg("park (hive-side corner, full power)", drive, finalPark),   // tried even when little time is left: the auto ends at 30 s
                                timed("ramp stow (wait)", rampTo(mechanisms, false))
                                ),
                        new SequentialAction(
                                note("Park"),
                                PARK_VIA_POINT ? passLeg("park via point", drive, parkVia) : new InstantAction(() -> { }),
                                parkLeg("park (full power)", drive, park))),
                new InstantAction(mechanisms::stopFlywheel)
        );

        Actions.runBlocking(this, auto, () -> {
            mechanisms.update(drive.getPose());
            PoseStorage.saveHeld(mechanisms.getHeldForHandOver(), mechanisms.getLoadOrder());   // every loop, so a stopped or crashed auto still hands its count to the TeleOp
        });
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
            PoseStorage.saveHeld(mechanisms.getHeldForHandOver(), mechanisms.getLoadOrder());
            RunLog.tick();
            telemetry.addLine("Auto complete.");
            telemetry.addData("Final Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f", p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.update();
        }
        // the one way out, as in the TeleOp (2026-11-17): the drive, then every mechanism off at once, with no update() after it
        drive.stop();
        mechanisms.stopAll();
    }
}
