package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;

/**
 * Finds the ramp's servo positions and its travel time. Uses the raw servos, not the Ramp
 * class, so it works before any position is known. The positions are not known yet, so NOTHING is commanded
 * until you press a trigger or a jog button. A trigger snaps both servos to a mirrored end at once, and the
 * first jog snaps that servo to about 0.5, so keep the ramp and your hands clear and make sure BOTH servos are
 * set to the same mode.
 *
 *   Left / right bumper   select the left / right servo
 *   D-pad up / down       selected servo +/- 0.01
 *   D-pad left / right    selected servo -/+ 0.05 (coarse)
 *   Left / right trigger  BOTH servos to a mirrored end: left trigger = (left 0.0, right 1.0), right trigger =
 *                         (left 1.0, right 0.0), to see which end is up
 *   A                     record BOTH servos' current positions as STOWED
 *   B                     record both as DEPLOYED
 *   X / Y                 send both servos to their recorded DEPLOYED / STOWED positions and start a timer
 *                         (needs STOWED and DEPLOYED recorded and both servos commanded once)
 *   Back               press when the ramp has stopped moving: records the elapsed time as TRAVEL_SEC
 *
 * Find STOWED with the ramp folded inside the starting cube, DEPLOYED with the ramp lowered to just short
 * of the floor / flower opening (not pressing, so a torque servo is not stalled). Then copy the printed
 * values into ChassisProfile (rampStowedLeft, ...) and Ramp.TRAVEL_SEC. The servos move at their
 * own full speed, so watch how the ramp lands: if it lands too hard, that is the case for adding a slew back.
 *
 * The two servos are tied together and mirrored, so they are always driven as a pair with OPPOSITE values
 * (left v, right 1 - v). Sending both the same value would make them fight, so use the triggers for the ends and
 * the jog buttons only for small corrections of one servo.
 *
 * Needs servos named "depolLeft" and "depolRight".
 */
@TeleOp(name = "Ramp Test", group = "Bring-up")
public class RampTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Servo[] servos = {hardwareMap.get(Servo.class, "depolLeft"), hardwareMap.get(Servo.class, "depolRight")};

        boolean selectedLeft = true;
        double[] position = {0.5, 0.5};
        boolean[] commanded = {false, false};
        double[] stowed = {Double.NaN, Double.NaN};
        double[] deployed = {Double.NaN, Double.NaN};
        ElapsedTime moveTimer = new ElapsedTime();
        boolean timing = false;
        double travelSec = Double.NaN;
        boolean prevLeftTrigger = false, prevRightTrigger = false;

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Ramp Test. Nothing is commanded until you jog. The first jog snaps that servo to about 0.5.");
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
            if (delta != 0) {
                position[i] = Math.max(0.0, Math.min(1.0, position[i] + delta));
                servos[i].setPosition(position[i]);
                commanded[i] = true;
            }

            boolean leftTrigger = gamepad1.left_trigger > 0.5;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;
            // the servos are tied together and mirrored: a pair command is (left, right) = (v, 1 - v), never the same value
            if (leftTrigger && !prevLeftTrigger) {
                for (int s = 0; s < 2; s++) {
                    position[s] = s == 0 ? 0.0 : 1.0;
                    servos[s].setPosition(position[s]);
                    commanded[s] = true;
                }
            }
            if (rightTrigger && !prevRightTrigger) {
                for (int s = 0; s < 2; s++) {
                    position[s] = s == 0 ? 1.0 : 0.0;
                    servos[s].setPosition(position[s]);
                    commanded[s] = true;
                }
            }
            prevLeftTrigger = leftTrigger;
            prevRightTrigger = rightTrigger;

            // the servos move one ramp together, so a record takes BOTH servos' current positions
            if (gamepad1.aWasPressed() && commanded[0] && commanded[1]) {
                stowed[0] = position[0];
                stowed[1] = position[1];
            }
            if (gamepad1.bWasPressed() && commanded[0] && commanded[1]) {
                deployed[0] = position[0];
                deployed[1] = position[1];
            }

            boolean allRecorded = !(Double.isNaN(stowed[0]) || Double.isNaN(stowed[1])
                    || Double.isNaN(deployed[0]) || Double.isNaN(deployed[1]));
            boolean deploy = gamepad1.xWasPressed();
            boolean stow = gamepad1.yWasPressed();
            if ((deploy || stow) && allRecorded && commanded[0] && commanded[1]) {
                double[] target = deploy ? deployed : stowed;
                for (int s = 0; s < 2; s++) {
                    position[s] = target[s];
                    servos[s].setPosition(target[s]);
                }
                moveTimer.reset();
                timing = true;
            }
            if (gamepad1.backWasPressed() && timing) {
                travelSec = moveTimer.seconds();
                timing = false;
            }

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Selected servo", selectedLeft ? "LEFT" : "RIGHT");
            telemetry.addData("Position L / R", "%s / %s",
                    commanded[0] ? String.format("%.3f", position[0]) : "not commanded",
                    commanded[1] ? String.format("%.3f", position[1]) : "not commanded");
            telemetry.addData("Recorded STOWED L / R", "%.3f / %.3f", stowed[0], stowed[1]);
            telemetry.addData("Recorded DEPLOYED L / R", "%.3f / %.3f", deployed[0], deployed[1]);
            telemetry.addData("Timer", timing ? String.format("%.2f s, press Back when the ramp stops", moveTimer.seconds()) : "idle");
            telemetry.addLine("--- copy into ChassisProfile and Ramp.java ---");
            telemetry.addLine(String.format("rampStowedLeft %.3f, rampStowedRight %.3f, rampDeployedLeft %.3f, rampDeployedRight %.3f",
                    stowed[0], stowed[1], deployed[0], deployed[1]));
            telemetry.addLine(String.format("TRAVEL_SEC = %.2f", travelSec));
            telemetry.addLine("Triggers = mirrored pair ends, A / B record both, bumpers + D-pad fine-tune one servo, X deploy, Y stow, Back stop the timer");
            telemetry.update();
        }
    }
}
