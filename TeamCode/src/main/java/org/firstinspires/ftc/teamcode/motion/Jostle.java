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
 * obstacle). First the robot keeps pressing for START_HOLD_SEC (2026-11-05: a pause after squaring on the wall, so the pollen can come in), then each cycle backs off BACK_IN inches
 * (a small shake: 0.25 in since 2026-11-05) behind that point at BACK_POWER (or for BACK_MAX_SEC), pushes forward until it is back at the point (at least PUSH_SEC, at most PUSH_MAX_SEC), and then
 * keeps pressing for PRESS_HOLD_SEC (the long pause on the forward stroke: the robot stays under the flower before it pulls back). Measuring from where the previous cycle ended let the robot creep from x 54.3 to
 * 51.9 over 5 cycles on 2026-11-03. It repeats until {@code done} is true (the robot is full), or MAX_CYCLES cycles are done (ONE since 2026-11-05), or MIN_COUNT_TO_STOP or more are held and nothing new has come in for
 * QUIET_SEC, or the time limit passes. The heading is held at the heading it had when the jostle started, with a small limit on the turn power so the obstacle can still square the robot. The drive is in the
 * robot's frame (forward is the robot's front), so it works the same for either alliance. A sustained push gave the pollen on 2026-11-03 where a single ram did not.
 */
@Config
public final class Jostle implements Action {

    /** The pause at the start, pressing against the flower, before the first back stroke (2026-11-05, user). */
    public static double START_HOLD_SEC = 0.0;   // 0 from 2026-11-20 (user: no opening press, the ram already presses against the flower); it was 1.0 s
    public static double BACK_IN = 0.5;   // 2.0 moved a little too far on 2026-11-03: halved to 1.0; 0.25 from 2026-11-05 (user): a small shake; 0.5 from 2026-11-07 (user: the 0.25 in shake was hardly visible)
    public static double POWER = 0.5;
    /** The back stroke is gentle so that it does not overshoot the small BACK_IN (at 0.5 power the robot moves about an inch before the next loop sees it). Raise it if the robot does not move back. */
    public static double BACK_POWER = 0.3;   // 0.3 again from 2026-11-19 night (user: a lower back-stroke power): at 0.4 the stroke is cut at 0.5 in but the robot brakes on to a deepest back-off of 1.5 to 2 in (median 1.95 in the 11-19 logs; the braking distance goes with the square of the speed), about 0.8 in less expected at 0.3; the Hartland runs of 2026-11-07 once did not move back at 0.3, so BACK_MAX_SEC is 0.4 now
    // earlier history of this value: 0.3 until 2026-11-07: the robot did not move back at the audience flower in some runs (user); 0.45 for a moment, then 0.4 (user: less overshoot)
    public static double PUSH_SEC = 0.2;   // the least a push lasts (halved from 0.35)
    public static double PUSH_MAX_SEC = 0.6;
    /** Once back at the contact point the robot keeps pressing this long before it backs off again (0.4 s before 2026-11-05). */
    public static double PRESS_HOLD_SEC = 1.0;
    /** The jostle ends after this many back-and-forth cycles (a back stroke, the push and the press), 2026-11-05 (user: it works much better now, one is enough). 0 = no limit. */
    public static int MAX_CYCLES = 1;
    public static double CONTACT_TOL_IN = 0.3;
    /** The jostle may stop early when this many or more are held and nothing new has come in for QUIET_SEC. 5 = never (a robot holds at most 4): getting all 4 matters more than the time (2026-11-04). */
    public static int MIN_COUNT_TO_STOP = 5;
    public static double QUIET_SEC = 0.8;
    /**
     * THE STROKES BY TIME (2026-11-19 night, user): the back stroke is BACK_SEC (0.2 s) at BACK_POWER (0.3) and the push is PUSH_TIMED_SEC (0.4 s) at POWER (0.5), then the press (PRESS_HOLD_SEC) is the wait. The
     * distance rules below (BACK_IN, CONTACT_TOL_IN, PUSH_SEC, PUSH_MAX_SEC) are used only when the flags are false. Why: the 0.5 in distance cut left the robot braking on to 1.5 to 2 in behind the contact point.
     */
    public static boolean BACK_BY_TIME = true, PUSH_BY_TIME = true;
    public static double BACK_SEC = 0.1, PUSH_TIMED_SEC = 0.2;   // 0.1 and 0.2 from 2026-11-20 (user; they were 0.2 and 0.4: at 0.2 s the robot braked on to a deepest back-off of 2.03 in, 1.01 in behind the contact point at the end of the command)
    public static double BACK_MAX_SEC = 0.4;   // halved from 0.5
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
    private enum Mode { HOLD, BACK, PUSH, PRESS }

    private Mode mode = Mode.HOLD;
    private double holdHeading;
    private Pose2d contact;   // the anchor: where the jostle started
    private int cycles = 0;
    private double minAlong = 0.0;   // the deepest back-off reached in this stroke (for the log)

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
            RunLog.event(String.format(Locale.US, "AUTO JOSTLE start at (%.1f, %.1f, %.1f deg): press %.1f s first, then back %.2f in (power %.2f) from this point and press %.1f s each cycle, power %.2f, %s, for at most %.1f s",
                    pose.position.x, pose.position.y, pose.heading.toDegrees(), START_HOLD_SEC, BACK_IN, BACK_POWER, PRESS_HOLD_SEC, POWER, MAX_CYCLES > 0 ? "at most " + MAX_CYCLES + " cycle(s)" : "no cycle limit", maxSec));
        }
        boolean full = done.getAsBoolean();
        int held = count.getAsInt();
        if (held > lastCount) sinceGain.reset();
        lastCount = held;
        boolean quiet = held >= MIN_COUNT_TO_STOP && sinceGain.seconds() >= QUIET_SEC;
        boolean cyclesDone = MAX_CYCLES > 0 && cycles >= MAX_CYCLES;
        if (full || quiet || cyclesDone || total.seconds() >= maxSec) {
            drive.stop();
            RunLog.event(String.format(Locale.US, "AUTO JOSTLE ended after %.2f s and %d cycles: %s, pose (%.1f, %.1f, %.1f deg)", total.seconds(), cycles,
                    full ? "the robot is full" : quiet ? "nothing new for the quiet time with " + held + " held" : cyclesDone ? "the " + MAX_CYCLES + " back-and-forth cycle(s) are done, " + held + " held" : "TIMED OUT, not full", pose.position.x, pose.position.y, pose.heading.toDegrees()));
            return false;
        }
        double turn = Math.max(-HEADING_MAX_TURN, Math.min(HEADING_MAX_TURN, HEADING_KP * Rotation2d.normalize(holdHeading - pose.heading.toDouble())));
        // how far along the robot's heading it is from the anchor: negative = backed off
        double along = (pose.position.x - contact.position.x) * Math.cos(holdHeading) + (pose.position.y - contact.position.y) * Math.sin(holdHeading);
        switch (mode) {
            case HOLD:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(POWER, 0.0), turn));   // keep pressing against the flower
                if (phase.seconds() >= START_HOLD_SEC) {
                    mode = Mode.BACK;
                    phase.reset();
                }
                break;
            case BACK:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-BACK_POWER, 0.0), turn));
                minAlong = Math.min(minAlong, along);
                if (BACK_BY_TIME ? phase.seconds() >= BACK_SEC : (along <= -BACK_IN || phase.seconds() >= BACK_MAX_SEC)) {
                    RunLog.event(String.format(Locale.US, "AUTO JOSTLE back stroke %d: commanded %.2f for %.2f s, %.2f in behind the contact point (wanted %.2f), ended by %s",
                            cycles + 1, -BACK_POWER, phase.seconds(), -along, BACK_IN, BACK_BY_TIME ? "the time (BACK_SEC " + BACK_SEC + " s)" : along <= -BACK_IN ? "the distance" : "the time limit: it did not move back far enough"));
                    mode = Mode.PUSH;
                    phase.reset();
                }
                break;
            case PUSH:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(POWER, 0.0), turn));
                minAlong = Math.min(minAlong, along);
                if (PUSH_BY_TIME ? phase.seconds() >= PUSH_TIMED_SEC : ((phase.seconds() >= PUSH_SEC && along >= -CONTACT_TOL_IN) || phase.seconds() >= PUSH_MAX_SEC)) {
                    RunLog.event(String.format(Locale.US, "AUTO JOSTLE push %d: commanded %.2f for %.2f s, back at %.2f in from the contact point; the deepest back-off was %.2f in",
                            cycles + 1, POWER, phase.seconds(), -along, -minAlong));
                    minAlong = 0.0;
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
