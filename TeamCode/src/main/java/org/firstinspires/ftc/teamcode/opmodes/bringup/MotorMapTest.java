package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Spins ONE drive motor at a time so wiring, config names and directions can be checked
 * wheel by wheel. Uses the raw hardware map (not MecanumDrive) so it works even if the
 * Pinpoint is missing or misconfigured. The drive motors have no encoders on this chassis,
 * so nothing here reads one - you judge by watching the wheel.
 *
 * Hold a button to spin that wheel at TEST_POWER; hold the left bumper as well to reverse.
 *   X = leftFront    Y = rightFront
 *   A = leftBack     B = rightBack
 *
 * Pass: the named wheel (and only that wheel) turns so the robot would be driven FORWARD -
 * the top of the wheel moves toward the front of the robot. Put the robot on blocks first.
 *
 * Directions here must match MecanumDrive (left side REVERSE, right side FORWARD).
 */
@Config
@TeleOp(name = "Motor Map Test", group = "Bring-up")
public class MotorMapTest extends LinearOpMode {

    public static double TEST_POWER = 0.3;

    private static final String[] NAMES = {"leftFront", "rightFront", "leftBack", "rightBack"};
    private static final String[] BUTTONS = {"X", "Y", "A", "B"};
    private static final DcMotorSimple.Direction[] DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD};

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx[] motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, NAMES[i]);
            if (motors[i] != null) {
                motors[i].setDirection(DIRECTIONS[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setPower(0.0);
            }
        }

        telemetry.addLine("Motor Map Test ready. Robot on blocks, wheels clear.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean[] pressed = {gamepad1.x, gamepad1.y, gamepad1.a, gamepad1.b};
            double sign = gamepad1.left_bumper ? -1.0 : 1.0;

            telemetry.addLine("Hold a button to spin one wheel. Left bumper = reverse.");
            telemetry.addLine("Pass: only the named wheel turns, robot-FORWARD direction.");
            for (int i = 0; i < 4; i++) {
                if (motors[i] == null) {
                    telemetry.addData("[" + BUTTONS[i] + "] " + NAMES[i], "NOT FOUND in the robot configuration");
                    continue;
                }
                double power = pressed[i] ? sign * TEST_POWER : 0.0;
                motors[i].setPower(power);
                telemetry.addData("[" + BUTTONS[i] + "] " + NAMES[i],
                        "%s  power %+.2f", DIRECTIONS[i], power);
            }
            telemetry.update();
        }

        for (DcMotorEx m : motors) {
            if (m != null) m.setPower(0.0);
        }
    }
}
