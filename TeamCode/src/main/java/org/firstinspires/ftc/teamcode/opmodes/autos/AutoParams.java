package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;

/**
 * Tunable values of the autonomous programs (specs/autos.md section 8). Times are seconds since Start.
 *
 * The -2.5 deg trims are the net of two older allowances at the far cell from (50, 26.3). Each auto launch has one
 * trim that holds all of its allowance; the far-cell extra is not applied in any auto.
 */
@Config
public final class AutoParams {
    private AutoParams() { }

    // Solo
    public static double SOLO_JOSTLE_DEADLINE = 23.0, SOLO_FINAL_JOSTLE_DEADLINE = 26.5;
    public static double SOLO_GARDEN_TRIM_DEG = -2.5, SOLO_FLOWER_TRIM_DEG = -2.5;
    /** Off replaces the final gather and the park with a plain park. */
    public static boolean SOLO_FINAL_GATHER = true;
    public static double SOLO_JOSTLE_LIMIT = 9.0, DUO_JOSTLE_LIMIT = 12.0;

    // Daryl
    public static double DARYL_SECOND_LAUNCH_NOT_BEFORE = 18.0, DARYL_RETURN_TRAVEL = 2.4;
    public static double DARYL_WALL_JOSTLE_DEADLINE = 19.0, DARYL_AUDIENCE_JOSTLE_DEADLINE = 26.0, DARYL_AUDIENCE_FLOWER_LATEST_START = 22.5;
    public static double AUDIENCE_CELL_TRIM_DEG = -2.5;

    // Cybugs
    public static double CYBUGS_WAIT_BEFORE_LAST_SALVO = 1.0, CYBUGS_REAR_TRIM_DEG = 0.0;

    // Garden Side and Flower Side: first guesses, to be tuned from logged runs.
    public static double GARDEN_SIDE_SECOND_LAUNCH_NOT_BEFORE = 25.0, GARDEN_SIDE_JOSTLE_DEADLINE = 20.0;
    public static double FLOWER_SIDE_FIRST_LAUNCH_NOT_BEFORE = 7.2, FLOWER_SIDE_JOSTLE_DEADLINE = 18.0;

    // Primary
    public static double PRIMARY_WALL_JOSTLE_DEADLINE = 12.0, PRIMARY_AUDIENCE_JOSTLE_DEADLINE = 21.0, PRIMARY_GARDEN_LATEST_START = 18.0;
}
