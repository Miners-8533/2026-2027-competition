package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Tune the HEADING PID. Turns in place through a varied pattern, forever, holding
 * X and Y at 0 so only heading moves. The pattern mixes regimes on purpose:
 *   large slews (+/-135), small moves (+/-20), and a near-180 flip (175 -> -175,
 *   which is only a 10 deg move the short way - checks wrap-around handling).
 *
 * Graph {@code dtp/errH_deg} and adjust {@code DriveToPose.HEADING_kP / kD / kS}:
 *   - big moves reveal kP (settle speed) and kD (overshoot)
 *   - small moves reveal kS (does it finish the last couple degrees, or stall?)
 *
 * Robot can spin in place - a bit of clear space around it. gamepad1.B = kill switch.
 */
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@TeleOp(name = "Tune - Heading (turn pattern)", group = "Tuning")
public class HeadingTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        // a run log from 2026-11-17 (runlogs/bringup.log): the gains in force (CONFIG and TUNE lines), every leg's DRIVE lines with its time, final error and overshoot, and a row every 50 ms
        RunLog.open(RunLog.Kind.BRINGUP, "Tune - Heading", hardwareMap);
        try {
            run();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Heading tuner. Keep still while the Pinpoint calibrates.");
        telemetry.addLine("Give the robot room to spin. START to run, B to kill.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        RunLog.started();

        double[] headingsDeg = {0, 135, -135, 20, -20, 175, -175, 90, -90};
        Pose2d[] pattern = new Pose2d[headingsDeg.length];
        for (int i = 0; i < headingsDeg.length; i++) {
            pattern[i] = new Pose2d(0, 0, Math.toRadians(headingsDeg[i]));
        }

        Actions.cycle(this, drive, pattern);
    }
}
