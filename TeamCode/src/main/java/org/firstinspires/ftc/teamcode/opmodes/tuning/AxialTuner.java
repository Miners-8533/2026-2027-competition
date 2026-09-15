package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;

/**
 * Tune the AXIAL (field-X, forward/back) PID. Drives 48 in forward, then back to the
 * start, forever. Heading and Y are held at 0, so this isolates the axial controller.
 *
 * On the FTC Dashboard: graph {@code dtp/errX_in} over time and adjust
 * {@code DriveToPose.AXIAL_kP / kD / kS} in the config panel.
 *   - too little kP: slow, stops short          - too much kP: overshoot / oscillation
 *   - add kD to kill overshoot                  - add a little kS if it stalls near target
 *
 * Give it ~5 ft of clear space ahead. gamepad1.B = kill switch.
 */
@TeleOp(name = "Tune - Axial (fwd/back 48in)", group = "Tuning")
public class AxialTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Axial tuner. Keep still while the Pinpoint calibrates.");
        telemetry.addLine("Clear 5+ ft ahead. START to run, B to kill.");
        telemetry.update();
        waitForStart();

        Actions.cycle(this, drive,
                new Pose2d(0, 0, 0),
                new Pose2d(48, 0, 0));
    }
}
