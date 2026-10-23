package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.TransportGate;

/**
 * Bring-up and calibration for the transport gate. The gate has no hard stops and a relative encoder, so
 * two numbers must be measured before it can do anything but jog: ticks per revolution and where the
 * push sweep ends. Do these in order:
 *
 *   1. Jog the arm (left stick) to the position where its back limits the transport to 4 elements. This
 *      is HOLD. Press A to declare "HOLD is here".
 *   2. Jog FORWARD exactly one full revolution back to the same mark, press B: records TICKS_PER_REV.
 *      Check the direction first: forward must be the way the arm pushes elements toward the flywheel.
 *      If not, flip gateDirection for this chassis in ChassisProfile.
 *   3. Jog to the angle where the push sweep should end (arm through the ramp, last element handed to the
 *      flywheel), press X: records SWEEP_END_DEG.
 *   4. Y runs a full salvo, RB runs a reject (reverse then forward to HOLD), LB returns to HOLD.
 *      D-pad up/down changes the sweep speed, left/right the return speed (30 deg/s per press).
 *   Options clears a stall. The recorded values last only for this session: copy the lines the
 *   opmode prints into TransportGate.java.
 *
 * Needs a motor named "gate". Jog slowly; the arm has no stops, so mind the elements and the ramp.
 */
@Config
@TeleOp(name = "Transport Gate Test", group = "Bring-up")
public class TransportGateTest extends LinearOpMode {

    public static double JOG_MAX_POWER = 0.3;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        TransportGate gate = new TransportGate(hardwareMap, TransportGate.Reference.ASSUME_AT_HOLD);
        String lastEvent = "-";

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Transport Gate Test. The gate is assumed to be at HOLD right now.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double stick = -gamepad1.left_stick_y;
            if (Math.abs(stick) > 0.05) {
                gate.jog(stick * JOG_MAX_POWER);
            } else if (gate.getState() == TransportGate.State.JOG) {
                gate.jog(0.0);
            }

            if (gamepad1.aWasPressed()) {
                gate.markHoldHere();
                lastEvent = "HOLD marked here (zero set).";
            }
            if (gamepad1.bWasPressed()) {
                double ticks = Math.abs(gate.getUnwrappedTicks());
                if (ticks < 100) {
                    lastEvent = "B ignored: jog one full revolution forward from HOLD first.";
                } else {
                    TransportGate.TICKS_PER_REV = ticks;
                    lastEvent = String.format("TICKS_PER_REV = %.1f", ticks);
                }
            }
            if (gamepad1.xWasPressed()) {
                if (!gate.isCalibrated()) {
                    lastEvent = "X ignored: set TICKS_PER_REV first (B).";
                } else {
                    TransportGate.SWEEP_END_DEG = gate.getAngleDeg();
                    lastEvent = String.format("SWEEP_END_DEG = %.1f", TransportGate.SWEEP_END_DEG);
                }
            }
            if (gamepad1.yWasPressed()) {
                gate.startSalvo();
                lastEvent = gate.isSweepConfigured() ? "Salvo started." : "Salvo ignored: set TICKS_PER_REV (B) and SWEEP_END_DEG (X).";
            }
            if (gamepad1.rightBumperWasPressed()) {
                gate.startReject();
                lastEvent = gate.isCalibrated() ? "Reject started." : "Reject ignored: set TICKS_PER_REV first.";
            }
            if (gamepad1.leftBumperWasPressed()) {
                gate.hold();
                lastEvent = "Returning to HOLD.";
            }
            if (gamepad1.optionsWasPressed()) {
                gate.clearStall();
                lastEvent = "Stall cleared.";
            }
            if (gamepad1.dpadUpWasPressed()) TransportGate.SWEEP_DEG_PER_SEC += 30;
            if (gamepad1.dpadDownWasPressed()) TransportGate.SWEEP_DEG_PER_SEC = Math.max(30, TransportGate.SWEEP_DEG_PER_SEC - 30);
            if (gamepad1.dpadRightWasPressed()) TransportGate.RETURN_DEG_PER_SEC += 30;
            if (gamepad1.dpadLeftWasPressed()) TransportGate.RETURN_DEG_PER_SEC = Math.max(30, TransportGate.RETURN_DEG_PER_SEC - 30);

            gate.update();

            telemetry.addData("Chassis profile", profile.label());
            gate.log(telemetry);
            telemetry.addData("Last event", lastEvent);
            telemetry.addLine("--- copy into TransportGate.java when happy ---");
            telemetry.addLine(String.format("TICKS_PER_REV = %.1f;   SWEEP_END_DEG = %.1f;", TransportGate.TICKS_PER_REV, TransportGate.SWEEP_END_DEG));
            telemetry.addLine(String.format("SWEEP_DEG_PER_SEC = %.0f;   RETURN_DEG_PER_SEC = %.0f;", TransportGate.SWEEP_DEG_PER_SEC, TransportGate.RETURN_DEG_PER_SEC));
            telemetry.addLine("Stick jog, A mark HOLD, B ticks/rev, X sweep end, Y salvo, RB reject, LB hold, Options clear stall");
            telemetry.update();
        }
        gate.jog(0.0);
        gate.update();
    }
}
