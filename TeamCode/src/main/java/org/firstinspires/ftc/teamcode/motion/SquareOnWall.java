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
 * Squares the robot on a wall: strafes LEFT (robot frame, so the left side goes into the wall) at PUSH_POWER with no heading hold, so the wall squares the robot
 * (whichever side corner touches first, the wheels keep pushing until the other one does). It ends when the robot has stopped moving (below STOP_SPEED_IN_PER_S for
 * STOPPED_HOLD_SEC, after at least MIN_SEC) or after MAX_SEC. The pose is NOT changed (the user did not want a reseed): the log line only records where the pose said the
 * robot was before and after, against the heading and the x the wall-flush pose should have, so the drift can be read from the log.
 */
@Config
public final class SquareOnWall implements Action {

    public static double PUSH_POWER = 0.3;
    public static double MIN_SEC = 0.3;
    public static double MAX_SEC = 0.9;
    public static double STOP_SPEED_IN_PER_S = 2.0;
    public static double STOPPED_HOLD_SEC = 0.15;

    private final MecanumDrive drive;
    private final double expectedHeadingRad;
    private final double expectedX;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime stopped = new ElapsedTime();
    private boolean started = false;
    private Pose2d before;

    /** expectedHeadingRad and expectedX: the heading and x the pose would read if the robot is flush on the wall (for the log only). */
    public SquareOnWall(MecanumDrive drive, double expectedHeadingRad, double expectedX) {
        this.drive = drive;
        this.expectedHeadingRad = expectedHeadingRad;
        this.expectedX = expectedX;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            started = true;
            timer.reset();
            stopped.reset();
            before = drive.getPose();
        }
        PoseVelocity2d measured = drive.updatePoseEstimate();
        double speed = measured == null ? 0.0 : Math.hypot(measured.linearVel.x, measured.linearVel.y);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, PUSH_POWER), 0.0));
        if (speed > STOP_SPEED_IN_PER_S) stopped.reset();
        boolean stoppedOk = timer.seconds() >= MIN_SEC && stopped.seconds() >= STOPPED_HOLD_SEC;
        boolean done = stoppedOk || timer.seconds() >= MAX_SEC;
        if (done) {
            drive.stop();
            Pose2d after = drive.getPose();
            RunLog.event(String.format(Locale.US, "AUTO SQUARE on the wall after %.2f s (%s): pose before (%.1f, %.1f, %.1f deg), after (%.1f, %.1f, %.1f deg); "
                            + "heading %+.1f deg and x %+.1f in from the wall-flush pose (%.1f deg, x %.1f); the pose was NOT reseeded",
                    timer.seconds(), stoppedOk ? "stopped by the wall" : "TIMED OUT, still moving", before.position.x, before.position.y, before.heading.toDegrees(),
                    after.position.x, after.position.y, after.heading.toDegrees(),
                    Math.toDegrees(Rotation2d.normalize(after.heading.toDouble() - expectedHeadingRad)), after.position.x - expectedX,
                    Math.toDegrees(expectedHeadingRad), expectedX));
        }
        return !done;
    }
}
