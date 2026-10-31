package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Bring-up test for the intake roller. It runs only while a button is held, as in the real
 * design, and starts at 30% of the tuned power so the first run is gentle.
 *
 *   A (hold)           take elements IN
 *   B (hold)           spit elements OUT
 *   D-pad up / down    power scale up / down by 0.1 (0.1 to 1.0)
 *   Back            reset the peak-current reading
 *
 * Checks, in order:
 *   1. Hold A: the roller pulls elements toward the transport. If it spits them out instead,
 *      flip intakeDirection for this chassis in ChassisProfile.
 *   2. Hold B: the reverse of that, at the (lower) outtake power.
 *   3. Raise the scale and watch the current. Feed it a pollen and a nectar by hand and note the
 *      peak current with and without an element; a jam threshold, if we add one, comes from this.
 *   4. The intake powers themselves (Intake.IN_POWER / OUT_POWER, start 1.0 / -0.5) are tuned
 *      live in the FTC Dashboard.
 *
 * Needs a motor named "intake" in the robot configuration. Keep fingers clear of the roller.
 */
@TeleOp(name = "Intake Test", group = "Bring-up")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        double scale = 0.3;
        intake.setPowerScale(scale);
        double peakAmps = 0;

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Intake Test ready. Hold A = in, B = out. Starts at 30% power.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) scale = Math.min(1.0, scale + 0.1);
            if (gamepad1.dpadDownWasPressed()) scale = Math.max(0.1, scale - 0.1);
            if (gamepad1.backWasPressed()) peakAmps = 0;
            intake.setPowerScale(scale);

            Intake.Mode wanted = Intake.Mode.OFF;
            if (gamepad1.a && !gamepad1.b) wanted = Intake.Mode.IN;
            else if (gamepad1.b && !gamepad1.a) wanted = Intake.Mode.OUT;
            intake.setMode(wanted);
            intake.update();

            peakAmps = Math.max(peakAmps, intake.getCurrentAmps());

            telemetry.addData("Chassis profile", profile.label());
            intake.log(telemetry);
            telemetry.addData("Power scale", "%.1f   (IN %.2f, OUT %.2f at full scale)",
                    scale, Intake.IN_POWER, Intake.OUT_POWER);
            telemetry.addData("Peak current since reset (A)", "%.2f", peakAmps);
            telemetry.addLine("Hold A = in, B = out. D-pad up/down = power scale. Back = reset peak.");
            telemetry.update();
        }
        intake.setMode(Intake.Mode.OFF);
    }
}
