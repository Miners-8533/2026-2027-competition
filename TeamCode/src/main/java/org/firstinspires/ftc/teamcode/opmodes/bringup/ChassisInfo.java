package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;

import java.util.List;

/**
 * Shows which chassis profile this Control Hub gets, and the hub ids to register. Touches no
 * motors or sensors, and never throws, so it works on a hub that no profile lists yet.
 *
 * To register a hub: run this on it, then add one of the ids shown (the serial is preferred) to
 * that chassis's list in {@link ChassisProfile}. Rerun to confirm "Matched".
 */
@TeleOp(name = "Chassis Info", group = "Bring-up")
public class ChassisInfo extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<String> ids = ChassisProfile.currentHubIds(hardwareMap);
        ChassisProfile matched = ChassisProfile.match(hardwareMap);

        while (!isStopRequested()) {
            telemetry.addData("Matched profile", matched == null ? "NONE - this hub is not registered" : matched.name);
            if (matched != null) {
                telemetry.addData("Motor directions (left / right)", "%s / %s", matched.leftDirection, matched.rightDirection);
                telemetry.addData("Pod directions (X / Y)", "%s / %s", matched.podXDirection, matched.podYDirection);
                telemetry.addData("Pod offsets x / y (mm)", "%.1f / %.1f", matched.xOffsetMM, matched.yOffsetMM);
                telemetry.addData("Robot depth x width (in)", "%.1f x %.1f", matched.robotDepthIn, matched.robotWidthIn);
            }
            telemetry.addLine("--- hub ids (register one in ChassisProfile) ---");
            if (ids.isEmpty()) {
                telemetry.addLine("No id could be read from this hub.");
            }
            for (int i = 0; i < ids.size(); i++) {
                telemetry.addData("id " + (i + 1), "\"" + ids.get(i) + "\"");
            }
            telemetry.update();
        }
    }
}
