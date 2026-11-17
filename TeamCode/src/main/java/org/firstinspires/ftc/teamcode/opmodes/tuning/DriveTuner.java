package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The drive tuners (specs/support-opmodes.md section 4.2): each drives a repeating pattern with drive to pose so its
 * gains can be adjusted live in the Dashboard. B is a kill switch; A goes on. They write a run log: the gains at the
 * start and whenever a Dashboard edit changes them, every leg's DRIVE lines, and a row every 50 ms.
 *
 * Hidden: remove the @Disabled line of the one wanted to bring it back.
 */
public abstract class DriveTuner extends TeamOpMode {
    /** The poses the pattern visits in turn, from (0, 0, 0) where the robot stands at INIT. */
    protected abstract Pose[] pattern();

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
        while (opModeInInit()) {
            drive.updatePose();
            telemetry.addLine(title() + ": the robot must stand still until Start.");
            telemetry.addLine("B stops it (kill switch); A goes on. Edit the DriveToPose gains in the Dashboard.");
            telemetry.addLine("Pinpoint: " + drive.localizer.statusText());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        String gains = DriveToPose.gainsLine();
        RunLog.event("TUNE gains " + gains);
        RunLog.sampler("x,y,hdg_deg,cmd_fwd,cmd_left,cmd_turn", 0.050, new RunLog.Sampler() {
            @Override public String row() {
                Pose p = drive.pose();
                return String.format(Locale.US, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", p.x, p.y, p.headingDeg(), drive.cmdForward(), drive.cmdLeft(), drive.cmdTurn());
            }
        });
        Pose[] pts = pattern();
        int i = 0;
        boolean killed = false;
        DriveToPose leg = null;
        String last = "";
        while (opModeIsActive()) {
            drive.updatePose();
            if (gamepad1.b) {
                killed = true;
                leg = null;
                drive.stop();
            } else if (gamepad1.a) {
                killed = false;
            }
            String now = DriveToPose.gainsLine();
            if (!now.equals(gains)) {
                gains = now;
                RunLog.event("TUNE gains " + gains);
            }
            if (!killed) {
                if (leg == null) leg = new DriveToPose(drive, title() + " leg " + i, pts[i % pts.length]);
                if (!leg.run(null)) {
                    last = String.format(Locale.US, "%s in %.2f s, error %.2f in %.1f deg, overshoot %.2f in %.1f deg", leg.settled() ? "settled" : "TIMED OUT",
                            leg.seconds(), leg.finalErrorIn(), leg.finalErrorDeg(), leg.overshootIn(), leg.overshootDeg());
                    leg = null;
                    i++;
                }
            }
            RunLog.tick();
            telemetry.addLine(killed ? "STOPPED (A goes on)" : "running; B stops");
            telemetry.addLine("Pose " + drive.pose());
            telemetry.addLine("Last leg: " + last);
            telemetry.update();
        }
    }

    @Disabled
    @TeleOp(name = "Tune - Axial", group = "3 Tuning")
    public static final class Axial extends DriveTuner {
        @Override protected String title() { return "Tune - Axial"; }
        @Override protected Pose[] pattern() { return new Pose[] {Pose.deg(48, 0, 0), Pose.deg(0, 0, 0)}; }
    }

    @Disabled
    @TeleOp(name = "Tune - Lateral", group = "3 Tuning")
    public static final class Lateral extends DriveTuner {
        @Override protected String title() { return "Tune - Lateral"; }
        @Override protected Pose[] pattern() { return new Pose[] {Pose.deg(0, 48, 0), Pose.deg(0, 0, 0)}; }
    }

    @Disabled
    @TeleOp(name = "Tune - Heading", group = "3 Tuning")
    public static final class Heading extends DriveTuner {
        @Override protected String title() { return "Tune - Heading"; }
        @Override protected Pose[] pattern() { return new Pose[] {Pose.deg(0, 0, 90), Pose.deg(0, 0, 180), Pose.deg(0, 0, 90), Pose.deg(0, 0, 0)}; }
    }
}
