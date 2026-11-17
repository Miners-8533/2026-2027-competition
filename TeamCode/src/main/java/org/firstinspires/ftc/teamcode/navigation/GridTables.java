package org.firstinspires.ftc.teamcode.navigation;

/**
 * Grid route tables. GENERATED SOURCE in the team's process: the dash simulator builds it from the fitted drive
 * model (specs/dashes.md section 5.1, specs/tools.md). It must not be edited by hand.
 *
 * The class name and the field names are the page's; GridRoutes reads them:
 *   GEOMETRY      the lattice the tables were built on: "pitch 6.0 in, half span 66.0 in, 16 directions"
 *   MODEL         the motion values the tables were built with
 *   CORNER_SPEED  the eleven speed limits alone, in in/s, for a corner of 0, 10, 20, 30, 45, 60, 75, 90, 120, 150 and
 *                 180 deg in that order; the angles are in GridRoutes, not in this file
 *   NAMES         the goals: BLUE_REAR, BLUE_AUDIENCE, LEFT, RIGHT
 *   GOAL_X, GOAL_Y  each goal's position, blue frame
 *   AIM           the heading at each goal in radians; not a number for a park corner
 *   NEXT          per goal, Base64: the next state of every state, 2 bytes each, high byte first, 0xFFFF = none
 *   COST          per goal, Base64: the time to the goal in milliseconds, 2 bytes each, high byte first, 0xFFFF = unreachable
 *   SPEED         per goal, Base64: the speed at the end of the leg in half inches per second, 1 byte each
 *
 * The tables are data, not text, and are not in the specs. Until the generator has been run for this branch there are
 * none: GridRoutes answers "no tables" and the dashes use the route planner.
 */
public final class GridTables {
    private GridTables() { }

    public static final String BUILT_FROM = "nothing: not generated in the from-specs rebuild";

    public static final String GEOMETRY = "";
    public static final String MODEL = "";
    public static final double[] CORNER_SPEED = {
    };
    public static final String[] NAMES = {
    };
    public static final double[] GOAL_X = {
    };
    public static final double[] GOAL_Y = {
    };
    public static final double[] AIM = {
    };
    public static final String[] NEXT = {
    };
    public static final String[] COST = {
    };
    public static final String[] SPEED = {
    };
}
