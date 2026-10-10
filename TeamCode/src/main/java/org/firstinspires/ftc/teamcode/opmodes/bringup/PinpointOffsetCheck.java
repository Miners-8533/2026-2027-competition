package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;

/**
 * Spins the robot in place and measures how far the Pinpoint's reported position wanders.
 * If the pod offsets in {@link PinpointLocalizer#PARAMS} are right, the reported position
 * stays put while the robot rotates about the tracking center. A wrong offset makes the
 * position trace a circle whose diameter is twice the offset error, and a full turn closes
 * back to the start. So the number to chase toward zero is the WOBBLE.
 *
 * Math: a constant error e (length units, robot frame, per radian turned) traces
 *   P(theta) = (ex sin t - ey (1 - cos t),  ex (1 - cos t) + ey sin t),
 * so at a half turn P = (-2 ey, 2 ex). The forward component ex tracks an xOffset error
 * (X pod sideways offset) and the strafe component ey tracks a yOffset error.
 * The SIGN of the correction has not been verified on hardware: nudge the offset by a few mm
 * in the FTC Dashboard, press A to rerun, and keep the direction that shrinks the wobble.
 *
 * Also check by eye: after the spins the robot should face the way it started (line it up with
 * a floor tape line). If not, the heading scale/IMU is off, which offsets cannot fix.
 *
 * Gamepad 1:  A = start / rerun (re-applies PARAMS)   B = stop immediately
 * Needs clear floor around the robot. Run Motor Map Test and Pinpoint Push Test first.
 */
@Config
@TeleOp(name = "Pinpoint Offset Check", group = "Bring-up")
public class PinpointOffsetCheck extends LinearOpMode {

    public static double SPIN_POWER = 0.25;
    public static int ROTATIONS = 3;
    public static double TIMEOUT_SEC = 45.0;

    private enum State { IDLE, SPINNING, DONE }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;

        telemetry.addLine("Pinpoint Offset Check. Keep still while the Pinpoint calibrates.");
        telemetry.update();
        waitForStart();

        State state = State.IDLE;
        String outcome = "";
        ElapsedTime clock = new ElapsedTime();
        double lastHeading = 0, unwrapped = 0, maxDist = 0, closure = 0;
        double oddX = 0, oddY = 0;
        int oddCount = 0, half = 1;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            double x = pose.position.x, y = pose.position.y;

            if (state == State.SPINNING) {
                unwrapped += Rotation2d.normalize(pose.heading.toDouble() - lastHeading);
                lastHeading = pose.heading.toDouble();
                maxDist = Math.max(maxDist, Math.hypot(x, y));

                if (unwrapped >= half * Math.PI) {
                    if (half % 2 == 1) { oddX += x; oddY += y; oddCount++; }
                    else { closure = Math.max(closure, Math.hypot(x, y)); }
                    half++;
                }

                if (gamepad1.b) {
                    drive.stop(); state = State.DONE; outcome = "Stopped by B.";
                } else if (unwrapped >= ROTATIONS * 2 * Math.PI) {
                    drive.stop(); state = State.DONE; outcome = "Finished " + ROTATIONS + " turns.";
                } else if (unwrapped < Math.toRadians(-30)) {
                    drive.stop(); state = State.DONE;
                    outcome = "ABORTED: heading went the wrong way. Check Motor Map Test and Pinpoint Push Test.";
                } else if (clock.seconds() > 2.0 && unwrapped < Math.toRadians(10)) {
                    drive.stop(); state = State.DONE;
                    outcome = "ABORTED: almost no heading change after 2 s. Wheels not turning, or Pinpoint not reporting.";
                } else if (clock.seconds() > TIMEOUT_SEC) {
                    drive.stop(); state = State.DONE; outcome = "ABORTED: timed out.";
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), SPIN_POWER));
                }
            } else if (gamepad1.aWasPressed()) {
                loc.applyParams();
                drive.setPose(new Pose2d(0, 0, 0));
                lastHeading = 0; unwrapped = 0; maxDist = 0; closure = 0;
                oddX = 0; oddY = 0; oddCount = 0; half = 1;
                outcome = "";
                clock.reset();
                state = State.SPINNING;
            }

            telemetry.addData("State", state);
            telemetry.addData("Pinpoint status", loc.getDeviceStatus());
            telemetry.addData("x / y (in)", "%.2f / %.2f", x, y);
            telemetry.addData("turned (deg)", "%.0f of %d", Math.toDegrees(unwrapped), ROTATIONS * 360);
            telemetry.addData("wobble so far (in)", "%.2f", maxDist);
            telemetry.addData("offsets in use (mm)", "x %.1f   y %.1f",
                    PinpointLocalizer.PARAMS.xOffsetMM, PinpointLocalizer.PARAMS.yOffsetMM);
            if (state == State.IDLE) {
                telemetry.addLine("Clear floor around the robot, line it up with a floor line. A = spin, B = stop.");
            }
            if (state == State.DONE) {
                telemetry.addLine(outcome);
                if (oddCount > 0) {
                    double ex = oddY / oddCount / 2.0, ey = -oddX / oddCount / 2.0;
                    telemetry.addData("wobble, max distance from start (in)", "%.2f   (= 2 x offset error)", maxDist);
                    telemetry.addData("est. forward-axis error (mm)", "%.1f   -> xOffset (sign unverified)", ex * 25.4);
                    telemetry.addData("est. strafe-axis error (mm)", "%.1f   -> yOffset (sign unverified)", ey * 25.4);
                }
                telemetry.addData("closure error at full turns (in)", "%.2f   (pods/scale, not offsets)", closure);
                telemetry.addLine("Check by eye: does the robot face its starting direction? A = rerun.");
            }
            telemetry.update();
        }
        drive.stop();
    }
}
