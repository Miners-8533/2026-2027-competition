package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

/**
 * The Dashboard values of the two DUO autos, "Auton - Duo Flower Side" and "Auton - Duo Garden Side" (design/duo-autos.md). They are a matched pair: the alliance partner runs the other one,
 * and the two take turns tipping the blue hive. All poses are in the BLUE frame (x, y inches, heading degrees; origin at the field center, +X toward the audience wall, +Y toward the Blue Wall);
 * red is the same path turned 180 degrees about the center. Most numbers are copied from {@code AutonSolo}, whose tuned values they start from; these are separate Dashboard values so tuning
 * one auto does not move the other.
 *
 * The sequence (T = the tip time, 4 s): G (garden side, the rear wall) launches its preload and tips the hive (the open cell starts with 3 nectar, so 3 pollen tip it); F (flower side, the audience
 * wall) then launches its preload and its flower pollen into the freshly opened audience cell, which needs 8 pollen to tip; G then launches its flower pollen and its garden pollen into the rear
 * cell, 8 again. Each robot only launches at the cell on its own start side. A robot cannot see its partner, so the waits are absolute times from Start.
 */
@Config
public final class DuoAutos {

    private DuoAutos() {}

    // ---- the waits (seconds from Start), tuned from the first logged runs ----
    /** F's preload launch is not before this: G's preload salvo ends at about 2.7 s, then the 4 s tip, then 0.5 s of margin. */
    public static double F_FIRST_LAUNCH_NOT_BEFORE_SEC = 7.2;
    /** G's flower-pollen launch is not before this: F's second salvo ends at about 20 s, then the 4 s tip, plus margin (a first guess: read it from the first runs). */
    public static double G_SECOND_LAUNCH_NOT_BEFORE_SEC = 25.0;
    /** Each robot's flower jostle ends by this many seconds into the auto at the latest (it also ends when the robot is full). */
    public static double F_JOSTLE_DEADLINE_SEC = 18.0, G_JOSTLE_DEADLINE_SEC = 20.0;

    // ---- the starts ----
    /** F starts at the audience wall (the blue frame y; the launch zone's y is 7.5), back flush, heading 180. G starts at the rear wall like the solo auto. */
    public static double F_START_Y_IN = 7.5;
    public static double G_START_Y_IN = 16.0;

    // ---- the launch spots (blue frame, x, y; the heading is the computed aim): F in the audience-side launch zone at its start y, G in the rear one a little off its start ----
    public static double F_LAUNCH_X = 61.5, F_LAUNCH_Y = 7.5;
    public static double G_LAUNCH_X = -61.5, G_LAUNCH_Y = 15.25;

    // ---- aim trims (degrees, positive = counterclockwise, negative = clockwise), one per launch; the AIM log line shows them ----
    public static double F_PRELOAD_TRIM_DEG = 0.0, F_FLOWER_TRIM_DEG = 0.0;
    public static double G_PRELOAD_TRIM_DEG = 0.0, G_FLOWER_TRIM_DEG = 0.0, G_GARDEN_TRIM_DEG = 0.0;

    // ---- the audience-wall flower (F), the solo auto's numbers, blue frame ----
    public static double FLOWER_PRE_X = 38.0, FLOWER_PRE_Y = 26.3, FLOWER_HEADING_DEG = 0.0;
    public static double FLOWER_AT_X = 55.3, FLOWER_AT_Y = 26.3;
    public static double FLOWER_RAM_POWER = 0.6, FLOWER_RAM_OVERSHOOT_IN = 12.0, FLOWER_RAM_SEC = 0.6;
    public static double FLOWER_BACKOFF_IN = 9.0, BACKOUT_TOL_IN = 1.5;
    /** The ramp stows once the robot has turned this far from the flower heading on the way to the launch spot. */
    public static double STOW_TURN_DEG = 45.0;
    public static double FLOWER_PAUSE_MAX_SEC = 12.0;

    // ---- the Blue Wall flower (G): the audience-wall flower turned 90 degrees about the field center, plus a nudge to refine on the field ----
    public static double WALL_FLOWER_DX_IN = 0.0, WALL_FLOWER_DY_IN = 0.0;

    // ---- the garden (G), the solo auto's numbers ----
    public static double GARDEN_PRE_X = -59.8, GARDEN_PRE_Y = 46.0, GARDEN_HEADING_DEG = 105.0;
    public static double GARDEN_END_X = -59.8, GARDEN_END_Y = 58.5;
    public static double SQUARE_GAP_IN = 3.0;
    public static double GARDEN_PICKUP_MAX_POWER = 0.35;

    // ---- the parks: both in the blue loading zone (x 23.9 to 46.6, y 59.1 to 70.1), facing the Blue Wall, F at the audience-side corner and G at the inner one ----
    public static double F_PARK_X = 40.0, F_PARK_Y = 60.0, G_PARK_X = 26.0, G_PARK_Y = 60.0, PARK_HEADING_DEG = 90.0;
    /** The park run uses full power; a target PARK_OVERSHOOT_IN past the pose pushes through the stall (0 here: with the front toward the wall a target 1 in further would press the wall). */
    public static double PARK_MAX_POWER = 1.0, PARK_OVERSHOOT_IN = 0.0, PARK_POS_TOL_IN = 2.5, PARK_HEADING_TOL_DEG = 5.0, PARK_TIMEOUT_SEC = 3.0;
    /** G's way to its park goes around the hive supports through this point (blue frame; the solo auto's helper, mirrored: G leaves from the rear side). */
    public static double HELPER_X = -36.0, HELPER_Y = 40.0;
    /** F's way to its park goes through this point first (clear of the audience-wall flower and the hive's support rails); heading is the park heading. */
    public static double F_PARK_VIA_X = 46.0, F_PARK_VIA_Y = 32.0;

    // ---- legs, as in the solo auto ----
    public static double TIMEOUT_BASE_SEC = 1.0, TIMEOUT_IN_PER_S = 22.0, TIMEOUT_MIN_SEC = 1.5, TIMEOUT_TURN_EXTRA_SEC = 0.5, TIMEOUT_MAX_SEC = 8.0;
    public static double COARSE_POS_IN = 3.0, COARSE_HEADING_DEG = 10.0;
    public static double LAUNCH_POS_TOL_IN = 2.5, LAUNCH_HEADING_TOL_DEG = 6.0;
    public static double CROSS_FIELD_MAX_POWER = 0.85;
    public static double SLOW_IN_PER_S_PER_POWER = 40.0;
}
