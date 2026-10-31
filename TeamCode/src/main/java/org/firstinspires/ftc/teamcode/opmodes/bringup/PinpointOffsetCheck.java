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

import java.util.ArrayList;
import java.util.List;

/**
 * Spins the robot in place and measures how far the Pinpoint's reported position wanders.
 * If the pod offsets in {@link PinpointLocalizer#PARAMS} are right, the reported position
 * stays put while the robot rotates about the tracking center. A wrong offset makes the
 * position trace a circle whose diameter is twice the offset error, repeating exactly every
 * full turn.
 *
 * Two things move the reported position, and they must be separated:
 *   - offset error: periodic, closes back to the start every full turn
 *   - real sliding: an open-loop mecanum spin never stays on one spot, so the robot really
 *     drifts across the floor. The Pinpoint reports that correctly, and it does NOT repeat.
 * The position is sampled at every half turn (P0, P1, ...). Each odd sample (a half turn
 * into a revolution) is corrected for steady drift by subtracting the average of its
 * neighbors, P_k - (P_{k-1} + P_{k+1}) / 2. What remains is the offset-error circle at its
 * far point, P = (-2 ey, 2 ex), so ex = Py / 2 and ey = -Px / 2. The forward component ex
 * tracks an xOffset error and the strafe component ey tracks a yOffset error. The net drift
 * per full turn is reported separately.
 * The SIGN of the correction has not been verified on hardware: nudge the offset by a few mm
 * in the FTC Dashboard, press A to rerun, and keep the direction that shrinks the error.
 * Measuring the pod positions with a tape is the more reliable check.
 *
 * Also check by eye: after the spins the robot should face the way it started (line it up with
 * a floor tape line). If not, the heading scale/IMU is off, which offsets cannot fix.
 *
 * Gamepad 1:  A = start / rerun (re-applies PARAMS)   B = stop immediately
 * Needs clear floor around the robot. Run Pinpoint Push Test first.
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
        double lastHeading = 0, unwrapped = 0, maxDist = 0;
        List<double[]> halfTurnPoints = new ArrayList<>();   // index k = position at k half turns

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            double x = pose.position.x, y = pose.position.y;

            if (state == State.SPINNING) {
                unwrapped += Rotation2d.normalize(pose.heading.toDouble() - lastHeading);
                lastHeading = pose.heading.toDouble();
                maxDist = Math.max(maxDist, Math.hypot(x, y));

                if (unwrapped >= halfTurnPoints.size() * Math.PI) {
                    halfTurnPoints.add(new double[]{x, y});
                }

                if (gamepad1.b) {
                    drive.stop(); state = State.DONE; outcome = "Stopped by B.";
                } else if (unwrapped >= ROTATIONS * 2 * Math.PI) {
                    drive.stop(); state = State.DONE; outcome = "Finished " + ROTATIONS + " turns.";
                } else if (unwrapped < Math.toRadians(-30)) {
                    drive.stop(); state = State.DONE;
                    outcome = "ABORTED: heading went the wrong way. Check the motor directions and run Pinpoint Push Test.";
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
                lastHeading = 0; unwrapped = 0; maxDist = 0;
                halfTurnPoints.clear();
                halfTurnPoints.add(new double[]{0, 0});     // P0 is the origin; the next point is the first half turn
                outcome = "";
                clock.reset();
                state = State.SPINNING;
            }

            telemetry.addData("State", state);
            telemetry.addData("Pinpoint status", loc.getDeviceStatus());
            telemetry.addData("x / y (in)", "%.2f / %.2f", x, y);
            telemetry.addData("turned (deg)", "%.0f of %d", Math.toDegrees(unwrapped), ROTATIONS * 360);
            telemetry.addData("farthest from start (in)", "%.2f", maxDist);
            telemetry.addData("offsets in use (mm)", "x %.1f   y %.1f",
                    PinpointLocalizer.PARAMS.xOffsetMM, PinpointLocalizer.PARAMS.yOffsetMM);
            if (state == State.IDLE) {
                telemetry.addLine("Clear floor around the robot, line it up with a floor line. A = spin, B = stop.");
            }
            if (state == State.DONE) {
                telemetry.addLine(outcome);
                reportResults(halfTurnPoints);
                telemetry.addLine("Check by eye: does the robot face its starting direction? A = rerun.");
            }
            telemetry.update();
        }
        drive.stop();
    }

    private void reportResults(List<double[]> pts) {
        int sumCount = 0;
        double cx = 0, cy = 0;
        for (int k = 1; k + 1 < pts.size(); k += 2) {
            cx += pts.get(k)[0] - (pts.get(k - 1)[0] + pts.get(k + 1)[0]) / 2.0;
            cy += pts.get(k)[1] - (pts.get(k - 1)[1] + pts.get(k + 1)[1]) / 2.0;
            sumCount++;
        }
        int lastEven = (pts.size() - 1) / 2 * 2;
        if (lastEven >= 2) {
            double drift = Math.hypot(pts.get(lastEven)[0], pts.get(lastEven)[1]) / (lastEven / 2.0);
            telemetry.addData("net drift per full turn (in)", "%.2f   (real sliding, not offsets)", drift);
        }
        if (sumCount == 0) {
            telemetry.addLine("Not enough turns for an offset estimate (needs at least 1 full turn).");
            return;
        }
        double ex = cy / sumCount / 2.0, ey = -cx / sumCount / 2.0;     // inches
        telemetry.addData("offset-only wobble (in)", "%.2f   (= 2 x offset error, drift removed)", 2 * Math.hypot(ex, ey));
        telemetry.addData("est. forward-axis error (mm)", "%.1f   -> xOffset = %.1f or %.1f",
                ex * 25.4, PinpointLocalizer.PARAMS.xOffsetMM - ex * 25.4, PinpointLocalizer.PARAMS.xOffsetMM + ex * 25.4);
        telemetry.addData("est. strafe-axis error (mm)", "%.1f   -> yOffset = %.1f or %.1f",
                ey * 25.4, PinpointLocalizer.PARAMS.yOffsetMM - ey * 25.4, PinpointLocalizer.PARAMS.yOffsetMM + ey * 25.4);
        telemetry.addLine("Sign unverified: measure the pods with a tape, or nudge and rerun.");
    }
}
