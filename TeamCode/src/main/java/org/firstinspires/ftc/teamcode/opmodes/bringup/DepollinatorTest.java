package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Depollinator;

/**
 * Finds the depollinator's servo positions and tests the deploy / stow motion. The servo positions are
 * not known yet, so NOTHING is commanded until you press a jog button, and the first jog snaps that servo
 * to about 0.5 (the 0.5 start plus that press), so keep the ramp and your hands clear and make sure BOTH
 * servos are set to the same mode.
 *
 *   Left / right bumper   select the left / right servo
 *   D-pad up / down       selected servo +/- 0.01
 *   D-pad left / right    selected servo -/+ 0.05 (coarse)
 *   A                     record the selected servo's position as STOWED
 *   B                     record it as DEPLOYED
 *   X / Y                 deploy / stow using the recorded positions, with the MAX_RATE slew
 *                         (needs all four recorded; both servos are assumed to be where you last jogged them)
 *
 * Find STOWED with the ramp folded inside the starting cube, DEPLOYED with the ramp lowered to just short
 * of the floor / flower opening (not pressing, so a torque servo is not stalled). Then copy the printed
 * values into ChassisProfile (depolStowedLeft, ...), and tune Depollinator.MAX_RATE for a gentle lowering.
 *
 * Needs servos named "depolLeft" and "depolRight".
 */
@TeleOp(name = "Depollinator Test", group = "Bring-up")
public class DepollinatorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Depollinator depollinator = new Depollinator(hardwareMap);

        boolean selectedLeft = true;
        double[] jogPosition = {0.5, 0.5};            // left, right: where a servo goes on its first jog
        boolean[] commanded = {false, false};
        double[] stowed = {Double.NaN, Double.NaN};
        double[] deployed = {Double.NaN, Double.NaN};

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Depollinator Test. Nothing is commanded until you jog. The first jog snaps that servo to about 0.5.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.leftBumperWasPressed()) selectedLeft = true;
            if (gamepad1.rightBumperWasPressed()) selectedLeft = false;
            int i = selectedLeft ? 0 : 1;

            double delta = 0;
            if (gamepad1.dpadUpWasPressed()) delta = 0.01;
            if (gamepad1.dpadDownWasPressed()) delta = -0.01;
            if (gamepad1.dpadRightWasPressed()) delta = 0.05;
            if (gamepad1.dpadLeftWasPressed()) delta = -0.05;
            if (delta != 0 || (!commanded[i] && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right))) {
                jogPosition[i] = Math.max(0.0, Math.min(1.0, jogPosition[i] + delta));
                depollinator.jog(selectedLeft, jogPosition[i]);
                commanded[i] = true;
            }

            if (gamepad1.aWasPressed() && commanded[i]) stowed[i] = jogPosition[i];
            if (gamepad1.bWasPressed() && commanded[i]) deployed[i] = jogPosition[i];

            boolean allRecorded = !(Double.isNaN(stowed[0]) || Double.isNaN(stowed[1])
                    || Double.isNaN(deployed[0]) || Double.isNaN(deployed[1]));
            if ((gamepad1.xWasPressed() || gamepad1.yWasPressed()) && allRecorded && commanded[0] && commanded[1]) {
                depollinator.setPositions(stowed[0], stowed[1], deployed[0], deployed[1]);
                depollinator.assumeCurrentPositions(jogPosition[0], jogPosition[1]);
                if (gamepad1.x) depollinator.deploy(); else depollinator.stow();
            }

            depollinator.update();
            if (depollinator.isConfigured()) {
                jogPosition[0] = depollinator.getCommandedPosition(true);
                jogPosition[1] = depollinator.getCommandedPosition(false);
            }

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Selected servo", selectedLeft ? "LEFT" : "RIGHT");
            telemetry.addData("Jog position L / R", "%s / %s",
                    commanded[0] ? String.format("%.3f", jogPosition[0]) : "not commanded",
                    commanded[1] ? String.format("%.3f", jogPosition[1]) : "not commanded");
            telemetry.addData("Recorded STOWED L / R", "%.3f / %.3f", stowed[0], stowed[1]);
            telemetry.addData("Recorded DEPLOYED L / R", "%.3f / %.3f", deployed[0], deployed[1]);
            depollinator.log(telemetry);
            telemetry.addLine("--- copy into ChassisProfile ---");
            telemetry.addLine(String.format("depolStowedLeft %.3f, depolStowedRight %.3f, depolDeployedLeft %.3f, depolDeployedRight %.3f",
                    stowed[0], stowed[1], deployed[0], deployed[1]));
            telemetry.addLine("Bumpers select, D-pad jog, A stowed, B deployed, X deploy, Y stow");
            telemetry.update();
        }
    }
}
