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
 *   2. Jog FORWARD a whole number of full revolutions back to the same mark, press B: records TICKS_PER_REV.
 *      Back picks 1, 5 or 10 revolutions; more revolutions is more precise (a mark misalignment is spread over them).
 *      Check the direction first: forward must be the way the arm pushes elements toward the flywheel.
 *      If not, flip gateDirection for this chassis in ChassisProfile.
 *   3. Jog to the angle where the push sweep should end (arm through the ramp, last element handed to the
 *      flywheel), press X: records SWEEP_END_DEG. If that is the HOLD mark itself (the sweep is the whole
 *      turn) it records 360.
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
        gate.jog(0.0);   // do nothing until told: no automatic hold at start, so an unverified encoder sign cannot run away
        String lastEvent = "-";
        int lastRaw = gate.getRawTicks();
        double forwardRaw = 0;
        int calRevs = 1;

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Transport Gate Test. The gate is assumed to be at HOLD right now.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double stick = -gamepad1.left_stick_y;
            int raw = gate.getRawTicks();
            if (stick > 0.05) forwardRaw += raw - lastRaw;
            lastRaw = raw;
            if (Math.abs(stick) > 0.05) {
                gate.jog(stick * JOG_MAX_POWER);
            } else if (gate.getState() == TransportGate.State.JOG) {
                gate.jog(0.0);
            }

            if (gamepad1.aWasPressed()) {
                gate.markHoldHere();
                lastEvent = "HOLD marked here (zero set).";
            }
            if (gamepad1.backWasPressed()) calRevs = calRevs == 1 ? 5 : calRevs == 5 ? 10 : 1;
            if (gamepad1.bWasPressed()) {
                double ticks = Math.abs(gate.getUnwrappedTicks());
                if (ticks < 100) {
                    lastEvent = "B ignored: jog the arm forward from the HOLD mark first.";
                } else {
                    TransportGate.TICKS_PER_REV = ticks / calRevs;
                    lastEvent = String.format("TICKS_PER_REV = %.1f  (%.0f ticks over %d revolutions)", ticks / calRevs, ticks, calRevs);
                }
            }
            if (gamepad1.xWasPressed()) {
                if (!gate.isCalibrated()) {
                    lastEvent = "X ignored: set TICKS_PER_REV first (B).";
                } else {
                    double angle = gate.getAngleDeg();
                    boolean atHoldMark = angle < 5.0 || angle > 355.0;
                    TransportGate.SWEEP_END_DEG = atHoldMark ? 360.0 : angle;
                    lastEvent = String.format("SWEEP_END_DEG = %.1f%s", TransportGate.SWEEP_END_DEG,
                            atHoldMark ? "  (at the HOLD mark: the sweep runs the whole turn)" : "");
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
            if (Math.abs(forwardRaw) < 200) {
                telemetry.addData("Encoder sign check", "jog forward (stick up) a bit; raw movement so far %.0f ticks", forwardRaw);
            } else if (forwardRaw < 0 && TransportGate.ENCODER_SIGN > 0) {
                telemetry.addData("Encoder sign check", "WRONG: the encoder counts DOWN going forward. Set TransportGate.ENCODER_SIGN = -1.");
            } else if (forwardRaw > 0 && TransportGate.ENCODER_SIGN < 0) {
                telemetry.addData("Encoder sign check", "WRONG: the encoder counts UP going forward. Set TransportGate.ENCODER_SIGN = +1.");
            } else {
                telemetry.addData("Encoder sign check", "OK (raw %.0f ticks forward, ENCODER_SIGN %+.0f)", forwardRaw, TransportGate.ENCODER_SIGN);
            }
            if (gate.isCalibrated()) {
                telemetry.addData("Revolutions (ticks / TICKS_PER_REV)", "%.3f   (jog N full turns to the mark: this should read N)",
                        gate.getUnwrappedTicks() / TransportGate.TICKS_PER_REV);
            }
            telemetry.addData("B records ticks per rev over", "%d revolution(s)   (Back changes it: 1, 5, 10)", calRevs);
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
