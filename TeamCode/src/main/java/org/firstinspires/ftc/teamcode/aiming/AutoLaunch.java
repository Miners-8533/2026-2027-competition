package org.firstinspires.ftc.teamcode.aiming;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.math.Vec;

/**
 * Auto-launch zones (specs/aiming.md section 3, specs/teleop.md section 6). Each cell has a launch zone: a circle
 * around its launch spot. While armed and locked, a robot that enters the zone of the locked cell is stopped, and one
 * salvo goes once the lock is aligned and the robot is still. Pure logic.
 */
@Config
public final class AutoLaunch {
    public static double ZONE_RADIUS_IN = 10.0;
    public static double STOPPED_IN_S = 4.0;
    public static double ALIGNED_STOPPED_S = 0.15;
    public static double HELD_MAX_S = 2.5;

    /** How far a point is from the center of a cell's launch zone. */
    public static double zoneDistance(Cell cell, double x, double y) {
        Vec spot = Aim.launchSpot(cell);
        return Math.hypot(x - spot.x, y - spot.y);
    }

    public static boolean inZone(Cell cell, double x, double y) {
        return zoneDistance(cell, x, y) <= ZONE_RADIUS_IN;
    }

    private boolean actedThisLock, gaveUp, inZone;
    private double enteredAt, stillSince = -1;

    /** What the TeleOp applies this loop. */
    public boolean holdChassis, requestLaunch;
    /** A log line when something happened this loop, else null. */
    public String event;

    /**
     * @param armed      the operator switched it on, or the robot is FULL in FULL AUTO; never in backup mode
     * @param lockOn     the target lock is on
     * @param readsEmpty both counts read and both hold nothing
     */
    public void update(boolean armed, boolean lockOn, boolean readsEmpty, Cell cell, double x, double y, boolean lockAligned, double speed, double t) {
        holdChassis = false;
        requestLaunch = false;
        event = null;
        if (!lockOn) {
            // Releasing the lock re-arms it.
            actedThisLock = false;
            gaveUp = false;
            inZone = false;
            stillSince = -1;
            return;
        }
        if (!armed || actedThisLock || gaveUp || readsEmpty) {
            inZone = false;
            stillSince = -1;
            return;
        }
        boolean in = inZone(cell, x, y);
        if (in && !inZone) {
            enteredAt = t;
            stillSince = -1;
            event = "AUTO LAUNCH entered the zone of " + cell.label() + ": chassis held";
        }
        inZone = in;
        if (!in) return;
        if (t - enteredAt > HELD_MAX_S) {
            gaveUp = true;
            event = "AUTO LAUNCH gave up after " + HELD_MAX_S + " s: not aligned and still; the driver has the robot back";
            return;
        }
        holdChassis = true;
        if (lockAligned && speed < STOPPED_IN_S) {
            if (stillSince < 0) stillSince = t;
            if (t - stillSince >= ALIGNED_STOPPED_S) {
                requestLaunch = true;
                actedThisLock = true;
                event = "AUTO LAUNCH launching at " + cell.label();
            }
        } else {
            stillSince = -1;
        }
    }
}
