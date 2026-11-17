package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.aiming.TargetLock;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Aim at a cell (specs/motion.md section 3): in an auto, turn in place onto a hive cell just before a launch.
 * The leg before it already ends at the aim heading; this removes the last degree or two. It uses the target lock's
 * turn law with no translation. The launch goes ahead whether it aligned or timed out.
 */
@Config
public final class AimAtCell implements Action {
    public static double ALIGNED_DEG = 1.0;
    public static double HOLD_S = 0.15;
    public static double TIME_CAP_S = 0.6;
    /** 0 = off. */
    public static double BREAKAWAY_POWER = 0.14;
    public static double STALL_RATE_DEG_S = 6.0;

    /** The cell is read when the aim starts, so it can follow the alliance chosen at INIT. */
    public interface CellSource { Cell get(); }

    private final MecanumDrive drive;
    private final CellSource cell;
    private final double trimDeg;
    private final boolean withAllowance;

    private double startT = -1, alignedSince = -1;
    private int breakawayLoops;

    /**
     * @param trimDeg       the aim trim for this launch, positive counterclockwise
     * @param withAllowance false in the autos (one trim holds all of a launch's allowance); true gives the TeleOp's aim
     */
    public AimAtCell(MecanumDrive drive, CellSource cell, double trimDeg, boolean withAllowance) {
        this.drive = drive;
        this.cell = cell;
        this.trimDeg = trimDeg;
        this.withAllowance = withAllowance;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        double t = Clock.now();
        if (startT < 0) startT = t;
        Pose p = drive.pose();
        Cell c = cell.get();
        double aim = withAllowance ? Aim.forCell(p.x, p.y, c, trimDeg, 0) : Aim.forAuto(p.x, p.y, c, trimDeg);
        double err = Angle.diff(aim, p.h), errDeg = Math.toDegrees(err);
        double omega = drive.localizer.omega(), rateDeg = Math.abs(Math.toDegrees(omega));
        boolean aligned = Math.abs(errDeg) <= ALIGNED_DEG && rateDeg < TargetLock.ALIGNED_RATE_DEG_S;
        if (aligned) {
            if (alignedSince < 0) alignedSince = t;
        } else {
            alignedSince = -1;
        }
        boolean held = aligned && t - alignedSince >= HOLD_S, timedOut = t - startT >= TIME_CAP_S;
        if (held || timedOut) {
            drive.stop();
            RunLog.eventf("AIM %s at %s in %.2f s: error %.1f deg, trim %.1f deg, %d breakaway loop(s), at %s",
                    held ? "aligned" : "TIMED OUT", c.label(), t - startT, errDeg, trimDeg, breakawayLoops, p);
            return false;
        }
        double turn = TargetLock.turnLaw(err, omega);
        // Breakaway: outside the dead band, a command too small to move a robot that is not turning is raised.
        if (BREAKAWAY_POWER > 0 && Math.abs(errDeg) > TargetLock.DEAD_BAND_DEG && Math.abs(turn) < BREAKAWAY_POWER && rateDeg < STALL_RATE_DEG_S) {
            turn = Math.signum(err) * BREAKAWAY_POWER;
            breakawayLoops++;
        }
        drive.drive(0, 0, turn);
        return true;
    }
}
