package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;

/**
 * The poses the autonomous programs drive to (specs/autos.md sections 2 and 8), in the blue frame. Part of the field
 * model (target T7): pure data, so the robot code and the PC checks read the same definition and nobody keeps a copy.
 */
@Config
public final class AutoPoses {
    private AutoPoses() { }

    // ---- Garden pickup ----
    /** The intake mouth angled toward the rear wall, so the pollen lying along the wall are caught, not plowed ahead. */
    public static double GARDEN_X = -59.8, GARDEN_PRE_Y = 46.0, GARDEN_END_Y = 58.5, GARDEN_HEADING_DEG = 105.0;
    /** The square approach stops with the robot's side this far off the rear wall. */
    public static double SQUARE_GAP_IN = 3.0;

    public static Pose gardenPre() { return Pose.deg(GARDEN_X, GARDEN_PRE_Y, GARDEN_HEADING_DEG); }
    public static Pose gardenEnd() { return Pose.deg(GARDEN_X, GARDEN_END_Y, GARDEN_HEADING_DEG); }

    /** Facing the Blue Wall, the left side a gap off the rear wall. */
    public static Pose squareApproach(double halfWidth) {
        return Pose.deg(-(Field.HALF_IN - SQUARE_GAP_IN - halfWidth), GARDEN_END_Y, 90);
    }

    /** What a robot pressed into the rear-wall and Blue Wall corner should read. */
    public static Pose squareFlush(double halfWidth, double originToFront) {
        return Pose.deg(-(Field.HALF_IN - halfWidth), Field.HALF_IN - originToFront, 90);
    }

    // ---- Flower gathers ----
    public static double FLOWER_PRE_X = 38.0, FLOWER_X = 55.3, FLOWER_Y = 26.3;
    /** The Blue Wall flower poses are a rotation of the tuned audience-wall numbers, not measured: a nudge for the Dashboard. */
    public static double WALL_FLOWER_NUDGE_X = 0.0, WALL_FLOWER_NUDGE_Y = 0.0;
    public static double RAM_BEYOND_IN = 12.0, BACK_OUT_IN = 9.0;

    /** @param wall false = the audience-wall flower (facing 0 deg); true = the Blue Wall flower (facing 90 deg) */
    public static Pose flowerPre(boolean wall) {
        return wall ? Pose.deg(-FLOWER_Y + WALL_FLOWER_NUDGE_X, FLOWER_PRE_X + WALL_FLOWER_NUDGE_Y, 90) : Pose.deg(FLOWER_PRE_X, FLOWER_Y, 0);
    }

    public static Pose flower(boolean wall) {
        return wall ? Pose.deg(-FLOWER_Y + WALL_FLOWER_NUDGE_X, FLOWER_X + WALL_FLOWER_NUDGE_Y, 90) : Pose.deg(FLOWER_X, FLOWER_Y, 0);
    }

    /** The ram drives at a target beyond the flower. */
    public static Pose ramTarget(boolean wall) {
        Pose f = flower(wall);
        return new Pose(f.x + RAM_BEYOND_IN * Math.cos(f.h), f.y + RAM_BEYOND_IN * Math.sin(f.h), f.h);
    }

    // ---- Launch spots of the autos ----
    /** Away from the walls, on the line of the flower pickups: 31.6 in from the cell. */
    public static double LAUNCH_X = 50.0, LAUNCH_Y = 26.3;
    public static double REAR_AT_START_X = -61.5, REAR_AT_START_Y = 15.25;
    public static double AUDIENCE_AT_WALL_X = 61.5, AUDIENCE_AT_WALL_Y = 7.5;

    public static Vec audienceLaunch() { return new Vec(LAUNCH_X, LAUNCH_Y); }
    public static Vec rearLaunch() { return new Vec(-LAUNCH_X, LAUNCH_Y); }
    public static Vec rearAtStart() { return new Vec(REAR_AT_START_X, REAR_AT_START_Y); }
    public static Vec audienceAtWall() { return new Vec(AUDIENCE_AT_WALL_X, AUDIENCE_AT_WALL_Y); }

    // ---- Other points ----
    /** The Flower Side program passes through here on its way to park. */
    public static Vec flowerSideVia() { return new Vec(46, 32); }
    /** The Solo program's plain park, used when its final gather is switched off. */
    public static Pose soloPlainPark() { return Pose.deg(51.1, 54.0, 180); }
}
