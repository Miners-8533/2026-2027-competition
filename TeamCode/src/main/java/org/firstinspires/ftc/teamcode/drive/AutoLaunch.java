package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

/**
 * Auto-launch (a prototype of the competition TeleOp, switched on and off by the operator's right bumper): while the target lock is on and the
 * robot is inside the SHOT ZONE of the cell it is locked on, the TeleOp stops the chassis (the driver's translation is ignored) and, once the lock
 * is ALIGNED and the robot has stopped, fires one salvo by itself. The zones keep the shot roughly straight on to the cell and not too close.
 *
 * A zone is a circle of ZONE_RADIUS_IN around a good shot spot for the cell, about 40 in out along the cell's axis (inside the shot map's flat part,
 * 32 to 47 in) with the shooter's line on that axis: ROBOT_1's shooter sits 2.5 in right of the centerline, so the rear cell's spot is at
 * y = 12.75 + 2.5 = 15.25 (heading 0), and the audience cell's at y = 12.75 - 2.5 - TargetLock.FAR_CELL_EXTRA = 7.5 (heading 180). The red zones are the
 * blue ones turned 180 deg about the field center. Chosen 2026-11-03 from the marked shots of 2026-11-02; the audience zone is from the geometry
 * (the marks there do not separate yet), so re-check it with marks.
 */
@Config
public final class AutoLaunch {

    /** Radius of every zone, inches (about 30 to 50 in from the cell, within about 14 deg of straight on). */
    public static double ZONE_RADIUS_IN = 10.0;
    /** The chassis counts as stopped below this speed, inches per second. */
    public static double MAX_SPEED_IN_PER_S = 4.0;
    /** ALIGNED and stopped for this long before the shot. */
    public static double ALIGNED_HOLD_SEC = 0.15;
    /** The chassis is held for at most this long after entering the zone; if the shot has not been taken by then, the driver gets the robot back. */
    public static double FREEZE_MAX_SEC = 2.5;

    public static double BLUE_REAR_X = -61.5, BLUE_REAR_Y = 15.25;
    public static double BLUE_AUDIENCE_X = 61.5, BLUE_AUDIENCE_Y = 7.5;
    public static double RED_AUDIENCE_X = 61.5, RED_AUDIENCE_Y = -15.25;
    public static double RED_REAR_X = -61.5, RED_REAR_Y = -7.5;

    private AutoLaunch() {}

    public static double centerX(HiveTarget cell) {
        switch (cell) {
            case BLUE_REAR: return BLUE_REAR_X;
            case BLUE_AUDIENCE: return BLUE_AUDIENCE_X;
            case RED_AUDIENCE: return RED_AUDIENCE_X;
            default: return RED_REAR_X;
        }
    }

    public static double centerY(HiveTarget cell) {
        switch (cell) {
            case BLUE_REAR: return BLUE_REAR_Y;
            case BLUE_AUDIENCE: return BLUE_AUDIENCE_Y;
            case RED_AUDIENCE: return RED_AUDIENCE_Y;
            default: return RED_REAR_Y;
        }
    }

    /** How far (inches) the point is from the center of the cell's shot zone. */
    public static double distanceFromZone(HiveTarget cell, double x, double y) {
        return Math.hypot(x - centerX(cell), y - centerY(cell));
    }

    public static boolean inZone(HiveTarget cell, double x, double y) {
        return distanceFromZone(cell, x, y) <= ZONE_RADIUS_IN;
    }
}
