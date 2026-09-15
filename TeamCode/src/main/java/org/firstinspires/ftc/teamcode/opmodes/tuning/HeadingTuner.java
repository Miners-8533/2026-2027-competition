package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Actions;

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
@TeleOp(name = "Tune - Heading (turn pattern)", group = "Tuning")
public class HeadingTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Heading tuner. Keep still while the Pinpoint calibrates.");
        telemetry.addLine("Give the robot room to spin. START to run, B to kill.");
        telemetry.update();
        waitForStart();

        double[] headingsDeg = {0, 135, -135, 20, -20, 175, -175, 90, -90};
        Pose2d[] pattern = new Pose2d[headingsDeg.length];
        for (int i = 0; i < headingsDeg.length; i++) {
            pattern[i] = new Pose2d(0, 0, Math.toRadians(headingsDeg[i]));
        }

        Actions.cycle(this, drive, pattern);
    }
}
