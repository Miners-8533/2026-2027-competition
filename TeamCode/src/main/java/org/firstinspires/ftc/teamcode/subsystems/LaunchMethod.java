package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Where the flywheel speed and the aim of a launch come from in the TeleOp (2026-11-18): chosen at INIT (gamepad 1 Y cycles, TeleOpCompetition.LAUNCH_METHOD) and, during a
 * practice run, cycled by the operator (left bumper + A). The autos always use {@link #MAP}.
 *
 *  - {@link #MAP}: the speed from the 1-D launch map ({@link LaunchMap}, by distance) and the aim from the formula (TargetLock.aimHeadingRad: the launcher offset and the far-cell extra).
 *  - {@link #GRID}: the speed and the aim from the 2-D launch grid (drive.LaunchGridMap: the measured spots, blended).
 *  - {@link #EQUATION}: the speed and the aim from the launch equation (drive.LaunchEquation: the grid's measured spots fitted by a curve in the distance and a corrected aim).
 */
public enum LaunchMethod {
    MAP("1-D LAUNCH MAP", "speed from the launch map by distance, aim from the formula"),
    GRID("2-D LAUNCH GRID", "speed and aim from the measured spots"),
    EQUATION("LAUNCH EQUATION", "speed from a curve in the distance, aim from the corrected formula");

    /** The name on the screens and in the log. */
    public final String label;
    public final String detail;

    LaunchMethod(String label, String detail) {
        this.label = label;
        this.detail = detail;
    }

    /** The next method, round and round (the INIT choice and the operator's cycle). */
    public LaunchMethod next() {
        LaunchMethod[] all = values();
        return all[(ordinal() + 1) % all.length];
    }
}
