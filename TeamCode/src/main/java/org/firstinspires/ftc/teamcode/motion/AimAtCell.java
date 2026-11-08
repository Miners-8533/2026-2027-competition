package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Turns the robot in place with the {@link TargetLock} (the same law and the same launcher-offset compensation as the TeleOp's lock, including the
 * far-cell extra) until it is ALIGNED on the cell for {@link #HOLD_SEC}, or {@link #TIMEOUT_SEC} passes, then stops the drive. The auto drives to
 * a pose whose heading is already the aim (see {@link TargetLock#aimHeadingRad}); this takes out what is left, a degree or two, before the launch.
 */
@Config
public final class AimAtCell implements Action {

    /** Aligned for this long counts as done. */
    public static double HOLD_SEC = 0.15;
    /** Gives up and lets the launch go ahead after this long (1.5 s before 2026-11-03, when the error sat at +2.0 to +2.2 deg and ran it out twice). */
    public static double TIMEOUT_SEC = 0.6;
    /**
     * Aligned means within this many degrees (the TeleOp lock's own ALIGNED_DEG is 2.0) and turning slower than TargetLock.ALIGNED_MAX_RATE_DEG_PER_S. 1.0 since 2026-11-06 (the user; it was 2.5): the Solo
     * auto's logs of 11-05 and 11-06 had every aim end at +1.5 to +2.1 deg or -1.9 to -2.1 deg, cut short as soon as it was inside 2.5 deg and nearly stopped (about 1.1 in of lateral error at the cell). The turn law's
     * own dead band (TargetLock.DEADBAND_DEG, 1.0) stops the turn at 1 deg, so this ends the aim when it gets there; if the turn stalls short (a TIMED OUT aim line) raise this or TIMEOUT_SEC.
     */
    public static double ALIGNED_DEG = 1.0;
    /**
     * BREAKAWAY (2026-11-07): the turn law's output near the end of an aim (about 1.2 x error + 0.05, so 0.07 to 0.11 for 1 to 2 deg) is under the drive's dead zone, and the robot sat still at 1 to 2 deg
     * short of the aim for the whole 0.6 s (the 11-07 logs: turn command -0.09, rate 0 deg/s, every aim TIMED OUT). While the robot is outside the turn dead band and turning slower than
     * STALL_RATE_DEG_PER_S, the turn command is raised to at least BREAKAWAY_POWER; once it moves, the normal law (smaller) takes over and it coasts in. 0 = off.
     */
    public static double BREAKAWAY_POWER = 0.14;
    public static double STALL_RATE_DEG_PER_S = 6.0;

    private final MecanumDrive drive;
    private final HiveTarget cell;
    private final boolean blue;
    private final double trimDeg;
    private int kicks = 0;   // loops in which the breakaway raised the turn command (for the AIM log line)
    /** NaN: the lock's own far-cell rule ({@link TargetLock#farCellExtraLeftIn}); a number: this extra sideways offset in inches instead (the duo autos launch only at the cell on their own start side, which needs none). */
    private final double extraLeftIn;
    private final TargetLock lock = new TargetLock();
    private final ElapsedTime timeout = new ElapsedTime();
    private final ElapsedTime held = new ElapsedTime();
    private final ElapsedTime loop = new ElapsedTime();
    private boolean started = false;

    public AimAtCell(MecanumDrive drive, HiveTarget cell, boolean blue) {
        this(drive, cell, blue, 0.0);
    }

    /** trimDeg: a standing aim correction for this launch, degrees, positive = counterclockwise (left), negative = clockwise (right); the same for either alliance. */
    public AimAtCell(MecanumDrive drive, HiveTarget cell, boolean blue, double trimDeg) {
        this(drive, cell, blue, trimDeg, Double.NaN);
    }

    public AimAtCell(MecanumDrive drive, HiveTarget cell, boolean blue, double trimDeg, double extraLeftIn) {
        this.drive = drive;
        this.cell = cell;
        this.blue = blue;
        this.trimDeg = trimDeg;
        this.extraLeftIn = extraLeftIn;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            started = true;
            timeout.reset();
            held.reset();
            loop.reset();
        }
        double dt = Math.min(0.1, Math.max(0.001, loop.seconds()));
        loop.reset();
        PoseVelocity2d measured = drive.updatePoseEstimate();
        Pose2d pose = drive.getPose();
        double turn = lock.turn(pose, measured == null ? 0.0 : measured.angVel, cell, Double.isNaN(extraLeftIn) ? TargetLock.farCellExtraLeftIn(cell, blue) : extraLeftIn, trimDeg, 0.0, dt);
        double rateDegPerS = measured == null ? 0.0 : Math.toDegrees(measured.angVel);
        if (BREAKAWAY_POWER > 0.0 && turn != 0.0 && Math.abs(turn) < BREAKAWAY_POWER && Math.abs(rateDegPerS) < STALL_RATE_DEG_PER_S) {
            turn = Math.copySign(BREAKAWAY_POWER, turn);
            kicks++;
        }
        drive.setDrivePowers(new PoseVelocity2d(Vector2d.zero(), turn));
        boolean near = Math.abs(lock.errorDeg()) <= ALIGNED_DEG && Math.abs(rateDegPerS) <= TargetLock.ALIGNED_MAX_RATE_DEG_PER_S;
        if (!near) held.reset();
        packet.put("aim/err_deg", lock.errorDeg());
        boolean aligned = near && held.seconds() >= HOLD_SEC;
        boolean done = aligned || timeout.seconds() >= TIMEOUT_SEC;
        if (done) {
            drive.stop();
            RunLog.event(String.format(java.util.Locale.US, "AIM %s at %s after %.2f s: error %+.1f deg (trim %+.1f deg, %d breakaway loops), pose (%.1f, %.1f, %.1f deg)",
                    aligned ? "aligned" : "TIMED OUT", cell, timeout.seconds(), lock.errorDeg(), trimDeg, kicks,
                    pose.position.x, pose.position.y, pose.heading.toDegrees()));
            lock.release();
        }
        return !done;
    }
}
