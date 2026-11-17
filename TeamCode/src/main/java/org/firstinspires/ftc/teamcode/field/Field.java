package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;

/**
 * The one field model (specs/field.md, target T7). Pure data: the frame and walls, the hive supports, the flower stands,
 * the garden and loading zone, the start poses, the launch spots, the park corners and the route points.
 * Everything is in the blue frame; Alliance rotates it for red. Every position is defined here once.
 */
@Config
public final class Field {
    private Field() { }

    // ---- 1. Frame and walls ----
    /** Wall position on both axes. The team measured about 142 in between wall faces; re-check at each venue. */
    public static double HALF_IN = 71.0;

    // ---- 2. Hive supports (field CAD), symmetric in both signs ----
    public static final double BAR_HALF_X = 19.47, BAR_Y_NEAR = 22.75, BAR_Y_FAR = 24.73, BAR_HEIGHT = 2.15;
    public static final double LEG_FOOT_X = 18.56, LEG_FOOT_Y = 23.81, LEG_FOOT_H = 0.53;
    public static final double LEG_LEAN_X = 0.445, LEG_LEAN_Y = 0.273, LEG_THICKNESS = 1.14;

    /** Where a leg is at a height: {|x|, |y|}. */
    public static double[] legAtHeight(double h) {
        double up = Math.max(0, h - LEG_FOOT_H);
        return new double[] {LEG_FOOT_X - LEG_LEAN_X * up, LEG_FOOT_Y - LEG_LEAN_Y * up};
    }

    // ---- 3. Flowers, garden, loading zone ----
    public static final double FLOWER_OUT = 68.45, FLOWER_SIDE = 23.4;
    /** A stand is this long along its wall and reaches this far out from it. */
    public static final double FLOWER_ALONG = 5.8, FLOWER_DEPTH = 4.8;

    /** Opening centers: audience wall, Blue Wall, rear wall, Red Wall. Blue's are the first two. */
    public static Vec[] flowerCenters() {
        return new Vec[] {
                new Vec(FLOWER_OUT, FLOWER_SIDE), new Vec(-FLOWER_SIDE, FLOWER_OUT),
                new Vec(-FLOWER_OUT, -FLOWER_SIDE), new Vec(FLOWER_SIDE, -FLOWER_OUT)};
    }

    /** A flower stand as a rectangle {xMin, yMin, xMax, yMax}, against its wall. */
    public static double[] flowerRect(int i) {
        Vec c = flowerCenters()[i];
        double a = FLOWER_ALONG / 2;
        if (Math.abs(c.x) > Math.abs(c.y)) {
            double wall = Math.signum(c.x) * HALF_IN, inner = wall - Math.signum(c.x) * FLOWER_DEPTH;
            return new double[] {Math.min(wall, inner), c.y - a, Math.max(wall, inner), c.y + a};
        }
        double wall = Math.signum(c.y) * HALF_IN, inner = wall - Math.signum(c.y) * FLOWER_DEPTH;
        return new double[] {c.x - a, Math.min(wall, inner), c.x + a, Math.max(wall, inner)};
    }

    /** Blue garden: against the rear wall. */
    public static final double GARDEN_Y_MIN = 47, GARDEN_Y_MAX = 70;
    /** Blue loading zone {xMin, yMin, xMax, yMax}, against the Blue Wall. */
    public static final double[] LOADING_ZONE = {23.9, 59.1, 46.6, 70.1};

    // ---- 4. Positions the software drives to (blue frame) ----
    public static double START_REAR_Y = 16.0;
    public static double START_AUDIENCE_Y = 7.5;

    /** Rear-wall start: the back flush against the wall, facing +X. */
    public static Pose startRear(double originToBack) {
        return new Pose(-(HALF_IN - originToBack), START_REAR_Y, 0);
    }

    /** Audience-wall start, facing -X. */
    public static Pose startAudience(double originToBack) {
        return new Pose(HALF_IN - originToBack, START_AUDIENCE_Y, Math.PI);
    }

    /** How far out from the field center a launch spot stands: about 34 in from its cell, 8 in clear of the wall behind. */
    public static double LAUNCH_SPOT_X = 55.4;

    /**
     * A cell's launch spot: square to the wall on the cell's open side, with the launcher's line through the cell.
     * The spot is worked out, not stored (target T1): the caller gives the launcher's sideways position (positive to the
     * robot's left) including any allowance for that cell, so the spot moves when the allowance does.
     */
    public static Vec launchSpot(Cell cell, double launcherLeftIn) {
        double leftY = Math.cos(cell.faceHeading());   // the y part of the robot's left, standing square: +1 facing +X, -1 facing -X
        return new Vec(cell.openSign() * LAUNCH_SPOT_X, cell.y() - leftY * launcherLeftIn);
    }

    /** Park corners for the TeleOp Park Dash. */
    public static double PARK_AUDIENCE_X = 40, PARK_HIVE_X = 26, PARK_DASH_Y = 60;
    /** The autos park 6.9 in short of the Park Dash corners on purpose: they stop with the front edge 8 in from the wall. */
    public static double PARK_AUTO_Y = 53.1;

    public static Vec parkDashCorner(boolean audienceSide) {
        return new Vec(audienceSide ? PARK_AUDIENCE_X : PARK_HIVE_X, PARK_DASH_Y);
    }

    public static Pose parkAuto(boolean audienceSide) {
        return new Pose(audienceSide ? PARK_AUDIENCE_X : PARK_HIVE_X, PARK_AUTO_Y, Math.PI / 2);
    }

    /** Route points: helper corners around the outside of the hive, and the gates of the lane under it. */
    public static double HELPER_X = 36, HELPER_Y = 40, GATE_X = 34;

    public static Vec helperEast() { return new Vec(HELPER_X, HELPER_Y); }
    public static Vec helperWest() { return new Vec(-HELPER_X, HELPER_Y); }

    /** All six route points, the same for either alliance (the set is symmetric). */
    public static Vec[] routePoints() {
        return new Vec[] {
                new Vec(HELPER_X, HELPER_Y), new Vec(-HELPER_X, HELPER_Y), new Vec(-HELPER_X, -HELPER_Y), new Vec(HELPER_X, -HELPER_Y),
                new Vec(GATE_X, 0), new Vec(-GATE_X, 0)};
    }

    // ---- Helpers ----
    public static boolean insideWalls(double x, double y, double slack) {
        return Math.abs(x) <= HALF_IN + slack && Math.abs(y) <= HALF_IN + slack;
    }
}
