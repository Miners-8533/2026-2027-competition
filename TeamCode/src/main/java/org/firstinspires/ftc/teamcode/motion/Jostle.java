package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;
import java.util.function.BooleanSupplier;

/**
 * Jostles an obstacle (the flower) that has to be shaken to give up its pollen: from where the robot is pressed against it, backs off BACK_IN inches (or for BACK_MAX_SEC),
 * pushes forward again for PUSH_SEC, and repeats until {@code done} is true (the robot is full) or MAX_SEC passes. The heading is held at the heading it had when the jostle
 * started, with a small limit on the turn power so the obstacle can still square the robot. The drive is in the robot's frame (forward is the robot's front), so it works
 * the same for either alliance. A sustained push gave the pollen on 2026-11-03 where a single ram did not.
 */
@Config
public final class Jostle implements Action {

    public static double BACK_IN = 1.0;   // 2.0 moved a little too far on 2026-11-03: halved
    public static double POWER = 0.5;
    public static double PUSH_SEC = 0.2;   // halved from 0.35
    public static double BACK_MAX_SEC = 0.3;   // halved from 0.5
    public static double HEADING_KP = 1.5;
    public static double HEADING_MAX_TURN = 0.15;

    private final MecanumDrive drive;
    private final BooleanSupplier done;
    private final double maxSec;
    private final ElapsedTime total = new ElapsedTime();
    private final ElapsedTime phase = new ElapsedTime();
    private boolean started = false;
    private boolean backing = true;
    private double holdHeading;
    private Pose2d backStart;
    private int cycles = 0;

    public Jostle(MecanumDrive drive, BooleanSupplier done, double maxSec) {
        this.drive = drive;
        this.done = done;
        this.maxSec = maxSec;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        drive.updatePoseEstimate();
        Pose2d pose = drive.getPose();
        if (!started) {
            started = true;
            total.reset();
            phase.reset();
            holdHeading = pose.heading.toDouble();
            backStart = pose;
            RunLog.event(String.format(Locale.US, "AUTO JOSTLE start at (%.1f, %.1f, %.1f deg): back %.1f in, push %.2f s, power %.2f, for at most %.1f s",
                    pose.position.x, pose.position.y, pose.heading.toDegrees(), BACK_IN, PUSH_SEC, POWER, maxSec));
        }
        boolean full = done.getAsBoolean();
        if (full || total.seconds() >= maxSec) {
            drive.stop();
            RunLog.event(String.format(Locale.US, "AUTO JOSTLE ended after %.2f s and %d cycles: %s, pose (%.1f, %.1f, %.1f deg)", total.seconds(), cycles,
                    full ? "the robot is full" : "TIMED OUT, not full", pose.position.x, pose.position.y, pose.heading.toDegrees()));
            return false;
        }
        double turn = Math.max(-HEADING_MAX_TURN, Math.min(HEADING_MAX_TURN, HEADING_KP * Rotation2d.normalize(holdHeading - pose.heading.toDouble())));
        if (backing) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-POWER, 0.0), turn));
            double moved = Math.hypot(pose.position.x - backStart.position.x, pose.position.y - backStart.position.y);
            if (moved >= BACK_IN || phase.seconds() >= BACK_MAX_SEC) {
                backing = false;
                phase.reset();
            }
        } else {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(POWER, 0.0), turn));
            if (phase.seconds() >= PUSH_SEC) {
                backing = true;
                backStart = pose;
                phase.reset();
                cycles++;
            }
        }
        return true;
    }
}
