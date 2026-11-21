package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * RAMP TEST (2026-11-21, the user, at the Novi Qualifier): the ramp by hand, nothing else. X deploys, Y stows, on either gamepad, as often as you like (the same buttons the operator has in the competition
 * TeleOp). Nothing else on the robot runs: no drive, no intake, no flywheel, no pusher. Nothing moves in INIT or at Start: the ramp servos are first commanded when a button is pressed (until then they are
 * unpowered and the screen says the position is unknown). The positions are the ones in ChassisProfile (the stowed and deployed position of each servo); the screen shows them. Never uses Start or Back.
 */
@TeleOp(name = "Ramp Test", group = "Bring-up")
public class RampTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Ramp Test", hardwareMap);
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
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Ramp ramp = new Ramp(hardwareMap, false);   // false: nothing is commanded until a button is pressed
        String positions = String.format(Locale.US, "stowed left %.2f / right %.2f, deployed left %.2f / right %.2f",
                profile.rampStowedLeft, profile.rampStowedRight, profile.rampDeployedLeft, profile.rampDeployedRight);
        RunLog.event("SETUP profile " + profile.name + ", ramp " + (ramp.isConfigured() ? positions : "NOT CONFIGURED (it will not move)"));

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("RAMP TEST. Keep hands clear of the ramp. Nothing moves until you press X or Y after Start.");
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Servo positions", ramp.isConfigured() ? positions : "NOT CONFIGURED: the ramp will not move");
            telemetry.addLine("After Start (either gamepad): X = DEPLOY, Y = STOW.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();

        boolean lastX = false, lastY = false;
        int deploys = 0, stows = 0;
        while (opModeIsActive()) {
            boolean x = gamepad1.x || gamepad2.x, y = gamepad1.y || gamepad2.y;
            if (x && !lastX) {
                ramp.deploy();
                deploys++;
                RunLog.event(String.format(Locale.US, "COMMAND ramp DEPLOY (left %.2f, right %.2f), number %d", profile.rampDeployedLeft, profile.rampDeployedRight, deploys));
            } else if (y && !lastY) {
                ramp.stow();
                stows++;
                RunLog.event(String.format(Locale.US, "COMMAND ramp STOW (left %.2f, right %.2f), number %d", profile.rampStowedLeft, profile.rampStowedRight, stows));
            }
            lastX = x;
            lastY = y;

            telemetry.addLine("RAMP TEST (either gamepad): X = DEPLOY, Y = STOW");
            ramp.log(telemetry);
            telemetry.addData("Servo positions", positions);
            telemetry.addData("Commands", "%d deploy, %d stow", deploys, stows);
            telemetry.update();
        }
    }
}
