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

/**
 * Squares the robot on a wall: strafes LEFT (robot frame, so the left side goes into the wall) at PUSH_POWER, holding the heading it started with (the turn power is
 * limited to HEADING_MAX_TURN so the wall can still square the robot; with no hold at all the first run on 2026-11-03 rotated 11 deg and stopped short of the wall).
 * It ends when the robot has stopped moving (below STOP_SPEED_IN_PER_S for STOPPED_HOLD_SEC, after at least MIN_SEC) or after MAX_SEC. Then it drives FORWARD at FORWARD_POWER
 * (still leaning into the side wall at SIDE_HOLD_POWER) to square on the wall in front, the Blue Wall for blue, until it stops (FORWARD_MIN_SEC to FORWARD_MAX_SEC): on
 * 2026-11-03 the strafe squared the robot on the rear wall but it still needed about 2 in forward to square on the Blue Wall. The pose is NOT changed (the user did not want a reseed): the log line only records where the pose said the
 * robot was before and after, against the heading and the x the wall-flush pose should have, so the drift can be read from the log.
 */
@Config
public final class SquareOnWall implements Action {

    public static double PUSH_POWER = 0.5;   // 0.3 moved only about 3 in/s and never reached the wall in 0.9 s
    public static double MIN_SEC = 0.5;
    public static double MAX_SEC = 1.4;
    public static double STOP_SPEED_IN_PER_S = 1.0;
    public static double FORWARD_POWER = 0.4;
    public static double SIDE_HOLD_POWER = 0.2;
    public static double FORWARD_MIN_SEC = 0.25;
    public static double FORWARD_MAX_SEC = 0.8;
    public static double HEADING_KP = 1.5;
    public static double HEADING_MAX_TURN = 0.15;
    public static double STOPPED_HOLD_SEC = 0.15;

    private final MecanumDrive drive;
    private final double expectedHeadingRad;
    private final double expectedX;
    private final double expectedY;
    private int phase = 0;   // 0 into the side wall, 1 forward into the front wall
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime stopped = new ElapsedTime();
    private boolean started = false;
    private Pose2d before;
    private double holdHeading;

    /** expectedHeadingRad, expectedX and expectedY: the heading, x and y the pose would read if the robot is flush in the corner (for the log only). */
    public SquareOnWall(MecanumDrive drive, double expectedHeadingRad, double expectedX, double expectedY) {
        this.drive = drive;
        this.expectedHeadingRad = expectedHeadingRad;
        this.expectedX = expectedX;
        this.expectedY = expectedY;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            started = true;
            timer.reset();
            stopped.reset();
            before = drive.getPose();
            holdHeading = before.heading.toDouble();
        }
        PoseVelocity2d measured = drive.updatePoseEstimate();
        double speed = measured == null ? 0.0 : Math.hypot(measured.linearVel.x, measured.linearVel.y);
        double turn = Math.max(-HEADING_MAX_TURN, Math.min(HEADING_MAX_TURN, HEADING_KP * Rotation2d.normalize(holdHeading - drive.getPose().heading.toDouble())));
        if (phase == 0) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, PUSH_POWER), turn));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(FORWARD_POWER, SIDE_HOLD_POWER), turn));
        }
        if (speed > STOP_SPEED_IN_PER_S) stopped.reset();
        double minSec = phase == 0 ? MIN_SEC : FORWARD_MIN_SEC, maxSec = phase == 0 ? MAX_SEC : FORWARD_MAX_SEC;
        boolean stoppedOk = timer.seconds() >= minSec && stopped.seconds() >= STOPPED_HOLD_SEC;
        boolean done = stoppedOk || timer.seconds() >= maxSec;
        if (!done) return true;
        Pose2d after = drive.getPose();
        RunLog.event(String.format(Locale.US, "AUTO SQUARE %s after %.2f s (%s): pose before (%.1f, %.1f, %.1f deg), after (%.1f, %.1f, %.1f deg); "
                        + "heading %+.1f deg, x %+.1f in, y %+.1f in from the flush pose (%.1f deg, x %.1f, y %.1f); the pose was NOT reseeded",
                phase == 0 ? "on the side wall" : "on the front wall", timer.seconds(), stoppedOk ? "stopped by the wall" : "TIMED OUT, still moving",
                before.position.x, before.position.y, before.heading.toDegrees(), after.position.x, after.position.y, after.heading.toDegrees(),
                Math.toDegrees(Rotation2d.normalize(after.heading.toDouble() - expectedHeadingRad)), after.position.x - expectedX, after.position.y - expectedY,
                Math.toDegrees(expectedHeadingRad), expectedX, expectedY));
        if (phase == 0) {
            phase = 1;
            before = after;
            timer.reset();
            stopped.reset();
            return true;
        }
        drive.stop();
        return false;
    }
}
