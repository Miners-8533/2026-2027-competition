package org.firstinspires.ftc.teamcode.aiming;

/**
 * Launch grid data. GENERATED SOURCE in the team's process: a PC tool builds it from a tuning run's log
 * (specs/tools.md section 1). It must not be edited by hand.
 *
 * The layout is the table maker's (specs/aiming.md section 4.1): one row per spot of the BLUE REAR cell,
 * {x, y, speed ticks/s, aim residual deg, shot possible (1/0), estimated (1/0)}. The tool also writes a list of
 * triangles; this lookup works the triangles out itself and does not need it.
 *
 * The measured spots are data, not text: they are in tools/launchgrid/data and are not in the specs. Until the table
 * maker has been run for this branch the table is empty, so every lookup answers "no data" and the launch map is used.
 */
public final class LaunchGridData {
    private LaunchGridData() { }

    public static final String BUILT_FROM = "nothing: not generated in the from-specs rebuild";

    public static final double[][] BLUE_REAR = {
    };
}
