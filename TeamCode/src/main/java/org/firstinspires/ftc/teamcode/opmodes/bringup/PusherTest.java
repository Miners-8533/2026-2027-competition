package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;

/**
 * Bring-up and calibration for the pusher. The pusher has no hard stops and a relative encoder, and
 * every move is "run at a power, cut it at a position threshold", so there is one number to measure
 * (ticks per revolution) and a few to calibrate (powers and where to cut). In order:
 *
 *   1. Jog the arm (left stick, up = forward) to the position where its back limits the transport to 4
 *      elements. This is HOLD. Press A to declare "HOLD is here".
 *   2. Jog FORWARD a whole number of full revolutions back to the same mark, press B: records TICKS_PER_REV.
 *      D-pad right picks 1, 5 or 10 revolutions; more is more precise. Check the direction first: forward must be
 *      the way the arm pushes elements toward the flywheel (if not, flip pusherDirection in ChassisProfile).
 *   3. Y runs a full salvo (HOLD, one turn forward, HOLD), RB a reject, LB returns to the nearest HOLD.
 *      After each move read "Error to nearest HOLD" once the arm stops: + means it stopped SHORT of the
 *      mark, - means it went PAST. Adjust and repeat.
 *   4. X selects which setting the D-pad up / down changes: sweep power, approach power, stop-early degrees,
 *      reject power. If it stops short, lower STOP_EARLY_DEG or raise APPROACH_POWER a little; if it goes
 *      past, raise STOP_EARLY_DEG or lower the powers. Back clears a fault.
 *
 * The values last only for this session: copy the printed lines into Pusher.java. Needs a motor named
 * "gate". Jog slowly; the arm has no stops, so mind the elements and the ramp.
 */
@Config
@TeleOp(name = "Pusher Test", group = "Bring-up")
public class PusherTest extends LinearOpMode {

    public static double JOG_MAX_POWER = 0.3;

    private static final String[] SETTING_NAMES = {"SWEEP_POWER", "APPROACH_POWER", "STOP_EARLY_DEG", "REJECT_POWER"};

    private static double getSetting(int i) {
        switch (i) {
            case 0: return Pusher.SWEEP_POWER;
            case 1: return Pusher.APPROACH_POWER;
            case 2: return Pusher.STOP_EARLY_DEG;
            default: return Pusher.REJECT_POWER;
        }
    }

    private static void changeSetting(int i, int direction) {
        double step = i == 2 ? 1.0 : 0.05;
        double max = i == 2 ? 30.0 : 1.0;
        double value = Math.max(i == 2 ? 0.0 : 0.05, Math.min(max, getSetting(i) + direction * step));
        switch (i) {
            case 0: Pusher.SWEEP_POWER = value; break;
            case 1: Pusher.APPROACH_POWER = value; break;
            case 2: Pusher.STOP_EARLY_DEG = value; break;
            default: Pusher.REJECT_POWER = value; break;
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Pusher pusher = new Pusher(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);
        pusher.jog(0.0);   // do nothing until told
        String lastEvent = "-";
        int lastRaw = pusher.getRawTicks();
        double forwardRaw = 0;
        int calRevs = 1;
        int setting = 0;

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Pusher Test. The pusher is assumed to be at HOLD right now.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double stick = -gamepad1.left_stick_y;
            int raw = pusher.getRawTicks();
            if (stick > 0.05) forwardRaw += raw - lastRaw;
            lastRaw = raw;
            if (Math.abs(stick) > 0.05) {
                pusher.jog(stick * JOG_MAX_POWER);
            } else if (pusher.getState() == Pusher.State.JOG) {
                pusher.jog(0.0);
            }

            if (gamepad1.aWasPressed()) {
                pusher.markHoldHere();
                lastEvent = "HOLD marked here (zero set).";
            }
            if (gamepad1.dpadRightWasPressed()) calRevs = calRevs == 1 ? 5 : calRevs == 5 ? 10 : 1;
            if (gamepad1.bWasPressed()) {
                double ticks = Math.abs(pusher.getUnwrappedTicks());
                if (ticks < 100) {
                    lastEvent = "B ignored: jog the arm forward from the HOLD mark first.";
                } else {
                    Pusher.TICKS_PER_REV = ticks / calRevs;
                    lastEvent = String.format("TICKS_PER_REV = %.1f  (%.0f ticks over %d revolutions)", ticks / calRevs, ticks, calRevs);
                }
            }
            if (gamepad1.yWasPressed()) {
                pusher.startSalvo();
                lastEvent = pusher.isCalibrated() ? "Salvo started." : "Salvo ignored: set TICKS_PER_REV first.";
            }
            if (gamepad1.rightBumperWasPressed()) {
                pusher.startReject();
                lastEvent = pusher.isCalibrated() ? "Reject started." : "Reject ignored: set TICKS_PER_REV first.";
            }
            if (gamepad1.leftBumperWasPressed()) {
                pusher.hold();
                lastEvent = "Going to the nearest HOLD.";
            }
            if (gamepad1.backWasPressed()) {
                pusher.clearStall();
                lastEvent = "Fault cleared.";
            }
            if (gamepad1.xWasPressed()) setting = (setting + 1) % SETTING_NAMES.length;
            if (gamepad1.dpadUpWasPressed()) changeSetting(setting, +1);
            if (gamepad1.dpadDownWasPressed()) changeSetting(setting, -1);

            pusher.update();

            telemetry.addData("Chassis profile", profile.label());
            pusher.log(telemetry);
            if (Math.abs(forwardRaw) < 200) {
                telemetry.addData("Encoder sign check", "jog forward (stick up) a bit; raw movement so far %.0f ticks", forwardRaw);
            } else if (forwardRaw < 0 && Pusher.ENCODER_SIGN > 0) {
                telemetry.addData("Encoder sign check", "WRONG: the encoder counts DOWN going forward. Set Pusher.ENCODER_SIGN = -1.");
            } else if (forwardRaw > 0 && Pusher.ENCODER_SIGN < 0) {
                telemetry.addData("Encoder sign check", "WRONG: the encoder counts UP going forward. Set Pusher.ENCODER_SIGN = +1.");
            } else {
                telemetry.addData("Encoder sign check", "OK (raw %.0f ticks forward, ENCODER_SIGN %+.0f)", forwardRaw, Pusher.ENCODER_SIGN);
            }
            if (pusher.isCalibrated()) {
                telemetry.addData("Revolutions (ticks / TICKS_PER_REV)", "%.3f   (jog N full turns to the mark: this should read N)",
                        pusher.getUnwrappedTicks() / Pusher.TICKS_PER_REV);
            }
            telemetry.addData("B records ticks per rev over", "%d revolution(s)   (D-pad right changes it: 1, 5, 10)", calRevs);
            telemetry.addData("D-pad up/down changes", "%s = %.2f   (X selects)", SETTING_NAMES[setting], getSetting(setting));
            telemetry.addData("Last event", lastEvent);
            telemetry.addLine("--- copy into Pusher.java when happy ---");
            telemetry.addLine(String.format("TICKS_PER_REV = %.1f;   SWEEP_POWER = %.2f;   APPROACH_POWER = %.2f;",
                    Pusher.TICKS_PER_REV, Pusher.SWEEP_POWER, Pusher.APPROACH_POWER));
            telemetry.addLine(String.format("APPROACH_DEG = %.0f;   STOP_EARLY_DEG = %.0f;   REJECT_POWER = %.2f;",
                    Pusher.APPROACH_DEG, Pusher.STOP_EARLY_DEG, Pusher.REJECT_POWER));
            telemetry.addLine("Stick jog, A mark HOLD, B ticks/rev, Y salvo, RB reject, LB hold, X setting, D-pad up/down adjust, Back clear fault");
            telemetry.update();
        }
        pusher.jog(0.0);
        pusher.update();
    }
}
