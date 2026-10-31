package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.math.Pose2d;

/**
 * Carries the robot's field pose and alliance from one opmode to the next (an auto hands its final pose to the
 * TeleOp). It is a static, so it lives as long as the robot app does and is empty after an app restart. A saved
 * pose is only trusted while it is fresh: a pose left over from a practice run an hour ago must not become the
 * start of a match, so callers pass the oldest age they will accept.
 *
 * The TeleOp also saves its pose every loop, so a TeleOp that has to be restarted mid-match picks up where the
 * robot is, not where the auto ended.
 */
public final class PoseStorage {

    private static Pose2d pose = null;
    private static boolean blue = true;
    private static long savedAtNanos = 0L;

    private PoseStorage() {}

    public static void save(Pose2d p, boolean isBlue) {
        pose = p;
        blue = isBlue;
        savedAtNanos = System.nanoTime();
    }

    /** True if a pose was saved no more than {@code maxAgeSec} seconds ago. */
    public static boolean isFresh(double maxAgeSec) {
        return pose != null && ageSec() <= maxAgeSec;
    }

    /** Seconds since the last save, or infinity if nothing was saved. */
    public static double ageSec() {
        return pose == null ? Double.POSITIVE_INFINITY : (System.nanoTime() - savedAtNanos) / 1e9;
    }

    /** The saved pose, or null if nothing is saved. */
    public static Pose2d pose() {
        return pose;
    }

    public static boolean isBlue() {
        return blue;
    }

    public static void clear() {
        pose = null;
    }
}
