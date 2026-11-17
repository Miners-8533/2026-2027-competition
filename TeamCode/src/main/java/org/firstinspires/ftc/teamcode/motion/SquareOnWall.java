package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Square on a wall (specs/motion.md section 4): press the robot into a corner so its heading and position are
 * physically known, after the garden pickup. First the side wall (strafe left), then the front wall.
 * The pose is not reseeded; each phase logs the pose against the pose a flush robot should read, so odometry drift
 * can be read from the logs.
 */
@Config
public final class SquareOnWall implements Action {
    public static double PUSH_POWER = 0.5;
    public static double SIDE_MIN_S = 0.5, SIDE_MAX_S = 1.4;
    public static double FORWARD_POWER = 0.4, SIDE_HOLD_POWER = 0.2;
    public static double FORWARD_MIN_S = 0.25, FORWARD_MAX_S = 0.8;
    public static double STOPPED_IN_S = 1.0, STOPPED_FOR_S = 0.15;
    public static double HEADING_KP = 1.5, TURN_LIMIT = 0.15;

    /** The pose a flush robot should read, for the log. */
    public interface Expected { Pose get(); }

    private final MecanumDrive drive;
    private final Expected expected;
    private int phase;                 // 0 = side wall, 1 = front wall, 2 = done
    private double phaseStart = -1, stoppedSince = -1, heading;
    private Pose before;

    public SquareOnWall(MecanumDrive drive, Expected expected) {
        this.drive = drive;
        this.expected = expected;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        double t = Clock.now();
        Pose p = drive.pose();
        if (phaseStart < 0) {
            phaseStart = t;
            heading = p.h;      // hold the starting heading, with a small turn limit so the wall can still square the robot
            before = p;
        }
        boolean slow = drive.localizer.speed() < STOPPED_IN_S;
        if (!slow) stoppedSince = -1;
        else if (stoppedSince < 0) stoppedSince = t;
        double el = t - phaseStart;
        double min = phase == 0 ? SIDE_MIN_S : FORWARD_MIN_S, max = phase == 0 ? SIDE_MAX_S : FORWARD_MAX_S;
        boolean stopped = el >= min && stoppedSince >= 0 && t - stoppedSince >= STOPPED_FOR_S;
        if (stopped || el >= max) {
            Pose flush = expected.get();
            RunLog.eventf("AUTO SQUARE %s wall %s in %.2f s: before %s, after %s, differs from flush %s by %.1f, %.1f in, %.1f deg",
                    phase == 0 ? "side" : "front", stopped ? "stopped" : "time limit", el, before, p, flush,
                    p.x - flush.x, p.y - flush.y, Math.toDegrees(Angle.diff(p.h, flush.h)));
            phase++;
            phaseStart = t;
            stoppedSince = -1;
            before = p;
            if (phase >= 2) {
                drive.stop();
                return false;
            }
        }
        double turn = Angle.clampAbs(HEADING_KP * Angle.diff(heading, p.h), TURN_LIMIT);
        if (phase == 0) drive.drive(0, PUSH_POWER, turn);
        else drive.drive(FORWARD_POWER, SIDE_HOLD_POWER, turn);
        return true;
    }
}
