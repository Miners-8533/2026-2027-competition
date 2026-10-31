package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;

/**
 * Hand-push check of the Pinpoint. The drive motors are set to FLOAT and never powered, so
 * you can push the robot around by hand. A guided three-step check judges each push:
 *   1. push FORWARD ~6 in   -> expect +X
 *   2. push LEFT ~6 in      -> expect +Y
 *   3. rotate CCW ~45 deg   -> expect + heading (counterclockwise seen from above)
 * Each step reports PASS or a likely cause. Raw pod ticks are always shown. The pods and
 * offsets come from {@link PinpointLocalizer#PARAMS}; edit them live in the FTC Dashboard
 * and press Back to re-apply them and rerun the check.
 *
 * Gamepad 1:  A = skip the current step   Back = re-apply PARAMS, zero the pose, restart
 */
@TeleOp(name = "Pinpoint Push Test", group = "Bring-up")
public class PinpointPushTest extends LinearOpMode {

    private static final double TRANSLATE_DONE_IN = 6.0;
    private static final double ROTATE_DONE_DEG = 45.0;
    private static final String[] MOTOR_NAMES = {"leftFront", "rightFront", "leftBack", "rightBack"};
    private static final String[] STEP_TEXT = {
            "Push the robot straight FORWARD about 6 in",
            "Push the robot straight LEFT about 6 in",
            "Rotate the robot COUNTERCLOCKWISE (seen from above) about 45 deg"};

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (String name : MOTOR_NAMES) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, name);
            if (m != null) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m.setPower(0.0);
            }
        }

        PinpointLocalizer loc = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Pinpoint Push Test. Keep the robot still while the Pinpoint calibrates.");
        telemetry.update();
        waitForStart();

        String[] verdicts = new String[3];
        int step = 0;
        double startX = 0, startY = 0, lastHeading = 0, headingAccum = 0;

        while (opModeIsActive()) {
            loc.update();
            Pose2d pose = loc.getPose();
            double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
            headingAccum += Rotation2d.normalize(h - lastHeading);
            lastHeading = h;

            if (gamepad1.backWasPressed()) {
                loc.applyParams();
                loc.setPose(new Pose2d(0, 0, 0));
                step = 0;
                verdicts = new String[3];
                startX = 0; startY = 0; headingAccum = 0; lastHeading = 0;
            } else if (gamepad1.aWasPressed() && step < 3) {
                verdicts[step] = "SKIPPED";
                step++;
                startX = x; startY = y; headingAccum = 0;
            } else if (step < 2 && Math.hypot(x - startX, y - startY) >= TRANSLATE_DONE_IN) {
                verdicts[step] = judgeTranslation(step == 0, x - startX, y - startY);
                step++;
                startX = x; startY = y; headingAccum = 0;
            } else if (step == 2 && Math.abs(Math.toDegrees(headingAccum)) >= ROTATE_DONE_DEG) {
                verdicts[2] = headingAccum > 0
                        ? "PASS  counterclockwise registered as + heading"
                        : "FAIL  counterclockwise registered as - heading: heading sign is reversed, check how the Pinpoint is mounted";
                step = 3;
            }

            GoBildaPinpointDriver.DeviceStatus status = loc.getDeviceStatus();
            telemetry.addData("Pinpoint status", status + (status.name().equals("READY") ? "" : "   <-- not READY, check cables"));
            telemetry.addData("Update rate (Hz)", "%.0f", loc.pinpoint.getFrequency());
            telemetry.addData("x / y (in)", "%.2f / %.2f", x, y);
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(h));
            telemetry.addData("raw pod ticks X / Y", "%d / %d", loc.pinpoint.getEncoderX(), loc.pinpoint.getEncoderY());
            telemetry.addLine("---");
            telemetry.addData("Now", step < 3 ? STEP_TEXT[step] : "All steps done. Back = rerun.");
            for (int i = 0; i < 3; i++) {
                telemetry.addData("Step " + (i + 1), verdicts[i] == null ? "-" : verdicts[i]);
            }
            telemetry.addLine("A = skip step   Back = re-apply PARAMS and restart");
            telemetry.update();
        }
    }

    private static String judgeTranslation(boolean forward, double dx, double dy) {
        boolean onX = Math.abs(dx) >= Math.abs(dy);
        if (forward) {
            if (onX && dx > 0) return "PASS  forward registered as +X";
            if (onX) return "FAIL  forward registered as -X: flip podXDirection in this chassis's ChassisProfile (or PARAMS.xDirection in the Dashboard to try it)";
            return "FAIL  forward registered on Y: the X and Y pods look swapped";
        }
        if (!onX && dy > 0) return "PASS  left registered as +Y";
        if (!onX) return "FAIL  left registered as -Y: flip podYDirection in this chassis's ChassisProfile (or PARAMS.yDirection in the Dashboard to try it)";
        return "FAIL  left registered on X: the X and Y pods look swapped";
    }
}
