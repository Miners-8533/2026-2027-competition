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
    public static double F_PARK_X = 40.0, F_PARK_Y = 53.1, G_PARK_X = 26.0, G_PARK_Y = 53.1, PARK_HEADING_DEG = 90.0;
    /** The park run uses full power; a target PARK_OVERSHOOT_IN past the pose pushes through the stall (0 here: with the front toward the wall a target 1 in further would press the wall). */
    public static double PARK_MAX_POWER = 1.0, PARK_OVERSHOOT_IN = 0.0, PARK_POS_TOL_IN = 2.5, PARK_HEADING_TOL_DEG = 5.0, PARK_TIMEOUT_SEC = 3.0;
    /** G's way to its park goes around the hive supports through this point (blue frame; the solo auto's helper, mirrored: G leaves from the rear side). */
    public static double HELPER_X = -36.0, HELPER_Y = 40.0;
    /** F's way to its park goes through this point first (clear of the audience-wall flower and the hive's support rails); heading is the park heading. */
    public static double F_PARK_VIA_X = 46.0, F_PARK_VIA_Y = 32.0;

    // ---- Auton - Duo Primary (2026-11-06, design/duo-autos.md): the rear-wall start of G with a longer program ----
    /**
     * The launch spot for the AUDIENCE cell after the Blue Wall flower: the solo auto's, on the audience flower's line (the flower gather follows it), the far cell so the aim takes the far-cell
     * extra. The launch spot for the REAR cell after the audience flower and after the garden: its mirror, (-50, 26.3), 31.6 in from the rear cell like the audience one is from its cell, 22 in
     * from the garden's pre-position. Blue frame, heading is the computed aim.
     */
    public static double P_AUD_LAUNCH_X = 50.0, P_AUD_LAUNCH_Y = 26.3;
    public static double P_REAR_LAUNCH_X = -50.0, P_REAR_LAUNCH_Y = 26.3;
    /** The helper point outside the hive's support rails on the audience side, for the trips between the Blue Wall side and the audience side (the west one is HELPER_X / HELPER_Y). */
    public static double P_EAST_HELPER_X = 36.0, P_EAST_HELPER_Y = 40.0, P_HELPER_HEADING_DEG = 180.0;
    public static double P_PRELOAD_TRIM_DEG = 0.0, P_WALL_TRIM_DEG = -8.0, P_AUD_FLOWER_TRIM_DEG = 0.0, P_GARDEN_TRIM_DEG = 0.0; // the salvos into the audience cell from (50, 26.3) (blue; the far cell from the rear-wall start) use -8.0, the solo auto's total trim there (user, 2026-11-07: the solo's landed left; -4.5 still landed left)
    /** Each flower jostle ends by this many seconds into the auto at the latest (it also ends after its one back-and-forth or when the robot is full). */
    public static double P_WALL_JOSTLE_DEADLINE_SEC = 12.0, P_AUD_JOSTLE_DEADLINE_SEC = 21.0;
    /**
     * A step that starts after this many seconds into the auto is skipped (logged as AUTO SKIP) so the park still happens: the audience flower (gather and back-out) and the garden (gather, return
     * and salvo). 99 = never skipped. The garden's is 18 s since 2026-11-06 (the first full run reached it at 21.8 s and lost the park); the audience flower's stays 99 (it started at 10.4 s).
     */
    public static double P_AUD_FLOWER_LATEST_START_SEC = 99.0, P_GARDEN_LATEST_START_SEC = 18.0;   // the garden at 18 s: the 01:57 run reached it at 21.8 s, ran its salvo to 28.1 s and the park was cut off at 30 s; garden step 6.3 s + park 3.0 s need a start by about 20.7 s, 18 leaves margin. The user (2026-11-06): skip the garden to protect the park for now
    /**
     * The Primary's path speeds, the solo auto's (2026-11-05): every ordinary leg at P_LEG_MAX_POWER (the drive's default is 0.6), the helper and cross-field legs at P_CROSS_MAX_POWER, and the
     * pass-through legs end the moment they are near (P_PASS_SETTLE_SEC instead of the drive's 0.20 s). The other two duo autos keep their old speeds.
     */
    public static double P_LEG_MAX_POWER = 0.85, P_CROSS_MAX_POWER = 1.0, P_PASS_SETTLE_SEC = 0.0;
    /** The Blue Wall flower's pre-position and back-out stalled 1.7 to 1.9 in short of the solo auto's tolerances (2026-11-05 logs): the same near-enough tolerances here. */
    public static double WALL_PRE_POS_TOL_IN = 2.5, WALL_PRE_HEADING_TOL_DEG = 4.0, WALL_BACKOUT_TOL_IN = 2.5;

    // ---- Auton - Duo Daryl (2026-11-06, design/duo-autos.md): the rear-wall start, garden first ----
    /**
     * The partner's window (the user, 2026-11-06): our preload ends at about 3 s, the hive tip takes 4 s, plus a 2 s buffer: the partner shoots at about 9 s and must be parked by about 18 s. The second audience
     * shot (the Blue Wall flower's pollen) is therefore not before D_SECOND_SHOT_NOT_BEFORE_SEC (an ABSOLUTE time from Start, not a relative pause): the robot does not leave the flower before that time minus
     * D_RETURN_TRAVEL_SEC (the drive back, about 2.4 s) so it does not arrive early, and waits at the spot if it still did. The nominal arrival is about 20 s, so this normally does not wait at all.
     */
    public static double D_SECOND_SHOT_NOT_BEFORE_SEC = 18.0, D_RETURN_TRAVEL_SEC = 2.4;
    /** The garden leg runs from the start straight to the wall squaring pose; the last D_GARDEN_SLOW_ZONE_IN inches (4 pollen of about 2.8 in end to end is 11.2 in, plus margin) are limited to D_GARDEN_SLOW_POWER (0.35 in the solo auto, relaxed to 0.5 by the user). */
    public static double D_GARDEN_SLOW_ZONE_IN = 12.5, D_GARDEN_SLOW_POWER = 0.5;
    public static double D_PRELOAD_TRIM_DEG = 0.0, D_GARDEN_TRIM_DEG = -8.0, D_WALL_TRIM_DEG = -8.0; // the salvos into the audience cell from (50, 26.3) (blue; the far cell from the rear-wall start) use -8.0, the solo auto's total trim there (user, 2026-11-07: the solo's landed left; -4.5 still landed left)
    /** The Blue Wall flower's jostle ends by this many seconds into the auto at the latest; the audience flower's likewise, and the audience flower is skipped when it would start later than the latest start (so the park keeps its time). */
    public static double D_WALL_JOSTLE_DEADLINE_SEC = 19.0, D_AUD_JOSTLE_DEADLINE_SEC = 26.0, D_AUD_FLOWER_LATEST_START_SEC = 22.5;

    // ---- legs, as in the solo auto ----
    public static double TIMEOUT_BASE_SEC = 1.0, TIMEOUT_IN_PER_S = 22.0, TIMEOUT_MIN_SEC = 1.5, TIMEOUT_TURN_EXTRA_SEC = 0.5, TIMEOUT_MAX_SEC = 8.0;
    public static double COARSE_POS_IN = 3.0, COARSE_HEADING_DEG = 10.0;
    public static double LAUNCH_POS_TOL_IN = 2.5, LAUNCH_HEADING_TOL_DEG = 6.0;
    // ---- Cybugs (2026-11-07, built from Daryl) ----
    /** The salvo from the rear launch spot at the start wall into the rear cell (the cell near the start): the preload from beside it needs no trim. */
    public static double C_REAR_TRIM_DEG = 0.0;
    /** A pause at the rear launch spot before the last salvo, the flywheel spinning (user, 2026-11-07: 2 s, then 1 s). */
    public static double C_LAST_SALVO_WAIT_SEC = 1.0;
    public static double CROSS_FIELD_MAX_POWER = 0.85;
    public static double SLOW_IN_PER_S_PER_POWER = 40.0;
}
