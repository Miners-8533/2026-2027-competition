package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.motion.ActionRunner;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The verify autos (specs/support-opmodes.md section 4.2): drive 48 in and back, forward or sideways, to confirm the
 * tuned gains by the final error. They write no log; they are for comparing the Pinpoint's answer with a tape mark on
 * the floor. Hidden.
 */
public abstract class AutonVerify extends TeamOpMode {
    protected abstract Pose far();

    @Override protected RunLog.Kind logKind() { return null; }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
        while (opModeInInit()) {
            drive.updatePose();
            telemetry.addLine(title() + ": put a tape mark at the robot, keep it still, then press Start.");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        final DriveToPose out = new DriveToPose(drive, "verify out", far()), back = new DriveToPose(drive, "verify back", new Pose(0, 0, 0));
        ActionRunner.run(this, drive, Actions.sequence(out, Actions.sleep(1.0), back), new ActionRunner.EveryLoop() {
            @Override public void run() { }
        });
        while (opModeIsActive()) {
            drive.updatePose();
            telemetry.addLine(String.format(Locale.US, "Out: %s, error %.2f in %.1f deg", out.settled() ? "settled" : "TIMED OUT", out.finalErrorIn(), out.finalErrorDeg()));
            telemetry.addLine(String.format(Locale.US, "Back: %s, error %.2f in %.1f deg", back.settled() ? "settled" : "TIMED OUT", back.finalErrorIn(), back.finalErrorDeg()));
            telemetry.addLine("Pinpoint says " + drive.pose() + ". Compare with the tape mark.");
            telemetry.update();
            idle();
        }
    }

    @Disabled
    @Autonomous(name = "Auton Verify - Forward 48 and back", group = "4 Auto verify")
    public static final class Forward extends AutonVerify {
        @Override protected String title() { return "Auton Verify - Forward 48 and back"; }
        @Override protected Pose far() { return Pose.deg(48, 0, 0); }
    }

    @Disabled
    @Autonomous(name = "Auton Verify - Lateral 48 and back", group = "4 Auto verify")
    public static final class Lateral extends AutonVerify {
        @Override protected String title() { return "Auton Verify - Lateral 48 and back"; }
        @Override protected Pose far() { return Pose.deg(0, 48, 0); }
    }
}
