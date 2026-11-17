package org.firstinspires.ftc.teamcode.navigation;

/**
 * Grid route tables. GENERATED SOURCE in the team's process: the dash simulator builds it from the fitted drive
 * model (specs/dashes.md section 5.1, specs/tools.md). It must not be edited by hand.
 *
 * What the file holds is on the page; GridRoutes reads it:
 *   GEOMETRY      the lattice the tables were built on: "pitch 6.0 in, half span 66.0 in, 16 directions"
 *   MODEL         the motion values the tables were built with
 *   CORNER_SPEED  rows of {corner angle deg, speed limit in/s}
 *   GOAL          the goals' names: BLUE_REAR, BLUE_AUDIENCE, LEFT, RIGHT
 *   GOAL_POSE     per goal {x, y, heading at the goal in radians}; the heading is NaN for a park corner
 *   NEXT          per goal, Base64: the next state of every state, 2 bytes each, high byte first, 0xFFFF = none
 *   TIME_MS       per goal, Base64: the time to the goal in milliseconds, 2 bytes each, high byte first, 0xFFFF = unreachable
 *   END_SPEED     per goal, Base64: the speed at the end of the leg in half inches per second, 1 byte each
 *
 * SPEC GAP: the page gives what the generated file holds, not the names of its fields; the names above are this
 * rebuild's, and a generator for this branch writes them.
 *
 * The tables are data, not text, and are not in the specs. Until the generator has been run for this branch there are
 * none: GridRoutes answers "no tables" and the dashes use the route planner.
 */
public final class GridRouteData {
    private GridRouteData() { }

    public static final String BUILT_FROM = "nothing: not generated in the from-specs rebuild";

    public static final String GEOMETRY = "";
    public static final String MODEL = "";
    public static final double[][] CORNER_SPEED = {
    };
    public static final String[] GOAL = {
    };
    public static final double[][] GOAL_POSE = {
    };
    public static final String[] NEXT = {
    };
    public static final String[] TIME_MS = {
    };
    public static final String[] END_SPEED = {
    };
}
