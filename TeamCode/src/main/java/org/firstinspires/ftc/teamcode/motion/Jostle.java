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
import java.util.function.IntSupplier;

/**
 * Jostles an obstacle (the flower) that has to be shaken to give up its pollen. The contact point is ANCHORED: it is where the robot is when the jostle starts (pressed against the
 * obstacle). Each cycle backs off BACK_IN inches behind that point (or for BACK_MAX_SEC), pushes forward until it is back at the point (at least PUSH_SEC, at most PUSH_MAX_SEC), and then
 * keeps pressing for PRESS_HOLD_SEC (the robot stays under the flower a little longer before it pulls back). Measuring from where the previous cycle ended let the robot creep from x 54.3 to
 * 51.9 over 5 cycles on 2026-11-03. It repeats until {@code done} is true (the robot is full), or MIN_COUNT_TO_STOP or more are held and nothing new has come in for QUIET_SEC, or the time
 * limit passes. The heading is held at the heading it had when the jostle started, with a small limit on the turn power so the obstacle can still square the robot. The drive is in the
 * robot's frame (forward is the robot's front), so it works the same for either alliance. A sustained push gave the pollen on 2026-11-03 where a single ram did not.
 */
@Config
public final class Jostle implements Action {

    public static double BACK_IN = 1.0;   // 2.0 moved a little too far on 2026-11-03: halved
    public static double POWER = 0.5;
    public static double PUSH_SEC = 0.2;   // the least a push lasts (halved from 0.35)
    public static double PUSH_MAX_SEC = 0.6;
    /** Once back at the contact point the robot keeps pressing this long before it backs off again. */
    public static double PRESS_HOLD_SEC = 0.4;
    public static double CONTACT_TOL_IN = 0.3;
    /** The jostle may stop early when this many or more are held and nothing new has come in for QUIET_SEC. 5 = never (a robot holds at most 4): getting all 4 matters more than the time (2026-11-04). */
    public static int MIN_COUNT_TO_STOP = 5;
    public static double QUIET_SEC = 0.8;
    public static double BACK_MAX_SEC = 0.3;   // halved from 0.5
    public static double HEADING_KP = 1.5;
    public static double HEADING_MAX_TURN = 0.15;

    private final MecanumDrive drive;
    private final BooleanSupplier done;
    private final IntSupplier count;
    private final ElapsedTime sinceGain = new ElapsedTime();
    private int lastCount = 0;
    private final double maxSec;
    private final ElapsedTime total = new ElapsedTime();
    private final ElapsedTime phase = new ElapsedTime();
    private boolean started = false;
    private enum Mode { BACK, PUSH, PRESS }

    private Mode mode = Mode.BACK;
    private double holdHeading;
    private Pose2d contact;   // the anchor: where the jostle started
    private int cycles = 0;

    public Jostle(MecanumDrive drive, BooleanSupplier done, IntSupplier count, double maxSec) {
        this.drive = drive;
        this.done = done;
        this.count = count;
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
            contact = pose;
            lastCount = count.getAsInt();
            sinceGain.reset();
            RunLog.event(String.format(Locale.US, "AUTO JOSTLE start at (%.1f, %.1f, %.1f deg): back %.1f in from this point, press %.1f s each cycle, power %.2f, for at most %.1f s",
                    pose.position.x, pose.position.y, pose.heading.toDegrees(), BACK_IN, PRESS_HOLD_SEC, POWER, maxSec));
        }
        boolean full = done.getAsBoolean();
        int held = count.getAsInt();
        if (held > lastCount) sinceGain.reset();
        lastCount = held;
        boolean quiet = held >= MIN_COUNT_TO_STOP && sinceGain.seconds() >= QUIET_SEC;
        if (full || quiet || total.seconds() >= maxSec) {
            drive.stop();
            RunLog.event(String.format(Locale.US, "AUTO JOSTLE ended after %.2f s and %d cycles: %s, pose (%.1f, %.1f, %.1f deg)", total.seconds(), cycles,
                    full ? "the robot is full" : quiet ? "nothing new for the quiet time with " + held + " held" : "TIMED OUT, not full", pose.position.x, pose.position.y, pose.heading.toDegrees()));
            return false;
        }
        double turn = Math.max(-HEADING_MAX_TURN, Math.min(HEADING_MAX_TURN, HEADING_KP * Rotation2d.normalize(holdHeading - pose.heading.toDouble())));
        // how far along the robot's heading it is from the anchor: negative = backed off
        double along = (pose.position.x - contact.position.x) * Math.cos(holdHeading) + (pose.position.y - contact.position.y) * Math.sin(holdHeading);
        switch (mode) {
            case BACK:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-POWER, 0.0), turn));
                if (along <= -BACK_IN || phase.seconds() >= BACK_MAX_SEC) {
                    mode = Mode.PUSH;
                    phase.reset();
                }
                break;
            case PUSH:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(POWER, 0.0), turn));
                if ((phase.seconds() >= PUSH_SEC && along >= -CONTACT_TOL_IN) || phase.seconds() >= PUSH_MAX_SEC) {
                    mode = Mode.PRESS;
                    phase.reset();
                }
                break;
            default:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(POWER, 0.0), turn));
                if (phase.seconds() >= PRESS_HOLD_SEC) {
                    mode = Mode.BACK;
                    phase.reset();
                    cycles++;
                }
                break;
        }
        return true;
    }
}
