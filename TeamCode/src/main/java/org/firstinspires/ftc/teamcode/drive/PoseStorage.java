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
 *
 * The CS5 HELD COUNT travels the same way (2026-11-17, user: the count must be remembered from the auto to the TeleOp and through an unexpected TeleOp restart;
 * specs/element-counting.md, target T3). The autos and the TeleOp save it every loop; a TeleOp starts from it while it is fresh. It has its own save time because the
 * TeleOp stops saving the POSE in backup mode and still knows its count. The operator's REMEMBERED AIMS travel too ({@link #saveAims}). {@link #clear()} drops all three.
 */
public final class PoseStorage {

    private static Pose2d pose = null;
    private static boolean blue = true;
    private static long savedAtNanos = 0L;
    private static int held = -1;                 // the CS5 held count, -1 = none saved (no CS5 in the configuration, or nothing saved yet)
    private static long heldSavedAtNanos = 0L;
    private static double[] aims = null;          // the operator's remembered aim per cell, degrees, by HiveTarget ordinal; null = none saved
    private static long aimsSavedAtNanos = 0L;

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
        held = -1;
        aims = null;
    }

    /**
     * Saves the operator's remembered aim per cell (2026-11-17, user: a TeleOp restarted in a match adopts the drifted pose, so it needs the trims that correct that pose; specs/aiming.md,
     * target T1). The TeleOp saves them every loop beside the pose; null drops them (an auto does that: it reseeds the Pinpoint, and the old trims belonged to the old pose).
     */
    public static void saveAims(double[] byCell) {
        aims = byCell == null ? null : byCell.clone();
        aimsSavedAtNanos = System.nanoTime();
    }

    /** True if remembered aims were saved no more than {@code maxAgeSec} seconds ago. */
    public static boolean isAimsFresh(double maxAgeSec) {
        return aims != null && (System.nanoTime() - aimsSavedAtNanos) / 1e9 <= maxAgeSec;
    }

    /** A copy of the saved remembered aims, or null if none are saved. */
    public static double[] aims() {
        return aims == null ? null : aims.clone();
    }

    /** Saves the CS5 held count (elements entered since the track was last emptied); a negative count means there is none to hand over and clears the saved one. */
    public static void saveHeld(int count) {
        held = count < 0 ? -1 : count;
        heldSavedAtNanos = System.nanoTime();
    }

    /** True if a held count was saved no more than {@code maxAgeSec} seconds ago. */
    public static boolean isHeldFresh(double maxAgeSec) {
        return held >= 0 && heldAgeSec() <= maxAgeSec;
    }

    /** Seconds since the held count was last saved, or infinity if none is saved. */
    public static double heldAgeSec() {
        return held < 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - heldSavedAtNanos) / 1e9;
    }

    /** The saved held count, or -1 if none is saved. */
    public static int held() {
        return held;
    }
}
