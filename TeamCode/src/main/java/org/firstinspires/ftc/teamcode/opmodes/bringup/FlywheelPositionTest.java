package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelPosition;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Flywheel Position Test (PROTOTYPE, branch flywheel-servo-prototype, 2026-11-15): finds the two positions of the servo that moves the FLYWHEEL, one for launching POLLEN and one for NECTAR, and which direction
 * the servo should run, WITHOUT the servo's channel being decided: it lists every servo in the Robot Configuration and lets you pick one in INIT. Nothing is commanded at Start; the servo is unpowered until the first
 * press. THE FIRST COMMAND MOVES THE SERVO TO START_POSITION (0.50): keep the flywheel and anything near it clear, and use the coarse step with care near the ends of its travel.
 *
 * INIT (gamepad 1):  D-pad left / right  pick the servo (the one named FlywheelPosition.SERVO_NAME, "fly_pos", is picked first when it exists)
 * Gamepad 1:  A  go to the POLLEN position and adjust that one      B  go to the NECTAR position and adjust that one      Y  switch between them
 *             D-pad up / down  +/- FINE_STEP (0.01) on the position being adjusted (LB held: COARSE_STEP 0.05); the servo follows at once
 *             X  reverse the servo's direction (the servo moves to the mirror position)
 * Screen and log: the values to paste into FlywheelPosition (POLLEN_POSITION, NECTAR_POSITION, REVERSED) and the name to give the servo in the Robot Configuration (SERVO_NAME). Watch the mechanism, find where each
 * element launches best, and time how long the servo takes between the two (FlywheelPosition.TRAVEL_SEC).
 */
@Config
@TeleOp(name = "Flywheel Position Test", group = "Bring-up")
public class FlywheelPositionTest extends LinearOpMode {

    /** The position the first command goes to for a position that is not set yet. */
    public static double START_POSITION = 0.50;
    public static double FINE_STEP = 0.01;
    public static double COARSE_STEP = 0.05;

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Flywheel Position Test", hardwareMap);
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

        List<String> names = new ArrayList<>();
        for (Map.Entry<String, Servo> e : hardwareMap.servo.entrySet()) names.add(e.getKey());
        int pick = names.indexOf(FlywheelPosition.SERVO_NAME);
        if (pick < 0) {
            for (int i = 0; i < names.size(); i++) {
                if (!names.get(i).startsWith("depol")) {
                    pick = i;
                    break;   // not the ramp's servos
                }
            }
        }
        if (pick < 0) pick = 0;

        while (!isStarted() && !isStopRequested()) {
            if (!names.isEmpty()) {
                if (gamepad1.dpadRightWasPressed()) pick = (pick + 1) % names.size();
                if (gamepad1.dpadLeftWasPressed()) pick = (pick + names.size() - 1) % names.size();
            }
            telemetry.addData("Servos in the Robot Configuration", names.isEmpty() ? "NONE: add the flywheel's servo in the configuration, then restart" : names.toString());
            telemetry.addData("Servo to test (D-pad left / right)", names.isEmpty() ? "-" : names.get(pick));
            telemetry.addLine("The ramp's servos are depolLeft and depolRight: do not pick those. Nothing moves until you press a button after Start; the first command goes to START_POSITION.");
            telemetry.update();
        }
        if (isStopRequested() || names.isEmpty()) return;

        String name = names.get(pick);
        Servo servo = hardwareMap.get(Servo.class, name);
        boolean reversed = false;
        double pollen = Double.NaN, nectar = Double.NaN;
        boolean adjustingPollen = true, commanded = false;
        RunLog.started();
        RunLog.event("SETUP flywheel position test: servo " + name + " of " + names);

        while (opModeIsActive()) {
            boolean moved = false;
            if (gamepad1.aWasPressed()) {
                adjustingPollen = true;
                if (Double.isNaN(pollen)) pollen = START_POSITION;
                moved = true;
            }
            if (gamepad1.bWasPressed()) {
                adjustingPollen = false;
                if (Double.isNaN(nectar)) nectar = START_POSITION;
                moved = true;
            }
            if (gamepad1.yWasPressed()) {
                adjustingPollen = !adjustingPollen;
                if (adjustingPollen && Double.isNaN(pollen)) pollen = START_POSITION;
                if (!adjustingPollen && Double.isNaN(nectar)) nectar = START_POSITION;
                moved = true;
            }
            double step = gamepad1.left_bumper ? COARSE_STEP : FINE_STEP;
            double delta = 0.0;
            if (gamepad1.dpadUpWasPressed()) delta += step;
            if (gamepad1.dpadDownWasPressed()) delta -= step;
            if (delta != 0.0 && commanded) {
                if (adjustingPollen) pollen = clamp(pollen + delta);
                else nectar = clamp(nectar + delta);
                moved = true;
            }
            if (gamepad1.xWasPressed()) {
                reversed = !reversed;
                servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                RunLog.event("REVERSED -> " + reversed);
                moved = commanded;
            }
            if (moved) {
                double pos = adjustingPollen ? pollen : nectar;
                servo.setPosition(pos);
                commanded = true;
                RunLog.event(String.format(Locale.US, "SERVO %s -> %s at %.3f (reversed %s) | POLLEN %s, NECTAR %s", name, adjustingPollen ? "POLLEN" : "NECTAR", pos, reversed, fmt(pollen), fmt(nectar)));
            }
            telemetry.addData("Servo", name + (commanded ? "" : "   (not commanded yet: press A, B or Y; the first command goes to " + String.format(Locale.US, "%.2f", START_POSITION) + ")"));
            telemetry.addData("Adjusting", adjustingPollen ? "POLLEN" : "NECTAR");
            telemetry.addData("POLLEN position", fmt(pollen));
            telemetry.addData("NECTAR position", fmt(nectar));
            telemetry.addData("Direction", reversed ? "REVERSED" : "forward");
            telemetry.addLine("--- paste into FlywheelPosition (Dashboard or the source) and name the servo in the Robot Configuration ---");
            telemetry.addLine(String.format(Locale.US, "SERVO_NAME = \"%s\";  REVERSED = %s;  POLLEN_POSITION = %s;  NECTAR_POSITION = %s;", name, reversed, fmt(pollen), fmt(nectar)));
            telemetry.addLine("A pollen, B nectar, Y switch, D-pad up/down +/- 0.01 (LB: 0.05), X reverse");
            telemetry.update();
        }
        RunLog.event(String.format(Locale.US, "FLYWHEEL POSITIONS at the end: SERVO_NAME \"%s\", REVERSED %s, POLLEN_POSITION %s, NECTAR_POSITION %s", name, reversed, fmt(pollen), fmt(nectar)));
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "not set" : String.format(Locale.US, "%.3f", v);
    }
}
