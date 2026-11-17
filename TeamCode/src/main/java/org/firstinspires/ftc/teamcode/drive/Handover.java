package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * State handed from one opmode to the next (specs/drive.md section 5, specs/architecture.md section 5).
 * Held in memory for the life of the robot app; an app restart empties it.
 *
 * The alliance is kept as a flag and the remembered aims as a plain array (one per hive cell, in the Cell order)
 * so that this package needs nothing from the field model.
 */
public final class Handover {
    private Handover() { }

    private static Pose pose;
    private static boolean red;
    private static long poseAt;
    private static int heldCount = -1;
    private static long countAt;
    private static double[] aims;

    private static double ageS(long at) { return (System.nanoTime() - at) / 1e9; }

    // ---- Pose and alliance ----
    public static synchronized void savePose(Pose p, boolean isRed) {
        pose = p;
        red = isRed;
        poseAt = System.nanoTime();
    }

    /** The saved pose if it is no older than the caller accepts, else null. */
    public static synchronized Pose freshPose(double maxAgeS) {
        return pose != null && ageS(poseAt) <= maxAgeS ? pose : null;
    }

    public static synchronized boolean red() { return red; }

    public static synchronized double poseAgeS() { return pose == null ? Double.NaN : ageS(poseAt); }

    // ---- CS5 held count: its own save time, because the TeleOp keeps saving it in backup mode ----
    public static synchronized void saveCount(int held) {
        heldCount = held;
        countAt = System.nanoTime();
    }

    /** The saved held count if fresh, else -1. */
    public static synchronized int freshCount(double maxAgeS) {
        return heldCount >= 0 && ageS(countAt) <= maxAgeS ? heldCount : -1;
    }

    public static synchronized double countAgeS() { return heldCount < 0 ? Double.NaN : ageS(countAt); }

    // ---- The operator's remembered aims: they correct the drift of the saved pose ----
    public static synchronized void saveAims(double[] perCellDeg) { aims = perCellDeg == null ? null : perCellDeg.clone(); }

    public static synchronized double[] aims() { return aims == null ? null : aims.clone(); }

    /** An auto drops them: it has reseeded the Pinpoint. */
    public static synchronized void dropAims() { aims = null; }

    /** Discards everything the last opmode saved: the pose, the aims that went with it, and the held count. */
    public static synchronized void discard() {
        pose = null;
        aims = null;
        heldCount = -1;
    }
}
