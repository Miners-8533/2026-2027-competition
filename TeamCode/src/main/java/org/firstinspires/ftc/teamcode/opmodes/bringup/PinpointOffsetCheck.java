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
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

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
 * Everything is also written to runlogs/bringup.log (pull it with adb): the offsets in use, each half-turn position, and the final estimates.
 *
 * Needs clear floor around the robot. Run Pinpoint Push Test first.
 */
@Config
// TEMPORARILY ENABLED 2026-11-03 (was @Disabled, with the import): the spin test of the CAD-based pod offsets in ChassisProfile.ROBOT_1
@TeleOp(name = "Pinpoint Offset Check", group = "Bring-up")
public class PinpointOffsetCheck extends LinearOpMode {

    public static double SPIN_POWER = 0.25;
    public static int ROTATIONS = 3;
    public static double TIMEOUT_SEC = 45.0;

    private enum State { IDLE, SPINNING, DONE }

    private double logX, logY, logTurnedDeg;
    private String logState = "";

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Pinpoint Offset Check", hardwareMap);
        try {
            runCheck();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runCheck() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;

        telemetry.addLine("Pinpoint Offset Check. Keep still while the Pinpoint calibrates.");
        telemetry.update();
        waitForStart();
        RunLog.started();
        RunLog.event(String.format(Locale.US, "SETUP offsets x %.1f mm, y %.1f mm, pods %s, SPIN_POWER %.2f, ROTATIONS %d",
                PinpointLocalizer.PARAMS.xOffsetMM, PinpointLocalizer.PARAMS.yOffsetMM, PinpointLocalizer.PARAMS.podType, SPIN_POWER, ROTATIONS));
        RunLog.setSampler("state,x_in,y_in,turned_deg,pod_x_ticks,pod_y_ticks", 100,
                () -> String.format(Locale.US, "%s,%.2f,%.2f,%.1f,%d,%d", logState, logX, logY, logTurnedDeg,
                        loc.pinpoint.getEncoderX(), loc.pinpoint.getEncoderY()));

        State state = State.IDLE;
        boolean doneLogged = false;
        String outcome = "";
        ElapsedTime clock = new ElapsedTime();
        double lastHeading = 0, unwrapped = 0, maxDist = 0;
        List<double[]> halfTurnPoints = new ArrayList<>();   // index k = position at k half turns

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            double x = pose.position.x, y = pose.position.y;
            logX = x;
            logY = y;
            logTurnedDeg = Math.toDegrees(unwrapped);
            logState = state.name();
            RunLog.tick();

            if (state == State.SPINNING) {
                unwrapped += Rotation2d.normalize(pose.heading.toDouble() - lastHeading);
                lastHeading = pose.heading.toDouble();
                maxDist = Math.max(maxDist, Math.hypot(x, y));

                if (unwrapped >= halfTurnPoints.size() * Math.PI) {
                    halfTurnPoints.add(new double[]{x, y});
                    RunLog.event(String.format(Locale.US, "HALF TURN %d at x %.2f in, y %.2f in", halfTurnPoints.size() - 1, x, y));
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
                doneLogged = false;
                RunLog.event(String.format(Locale.US, "START with offsets x %.1f mm, y %.1f mm", PinpointLocalizer.PARAMS.xOffsetMM, PinpointLocalizer.PARAMS.yOffsetMM));
            }

            if (state == State.DONE && !doneLogged) {
                doneLogged = true;
                RunLog.event("DONE: " + outcome + String.format(Locale.US, " turned %.0f deg, farthest %.2f in, final pose %.2f, %.2f in",
                        Math.toDegrees(unwrapped), maxDist, x, y));
                logResults(halfTurnPoints);
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

    /** The same numbers reportResults shows, written to the log. */
    private void logResults(List<double[]> pts) {
        int sumCount = 0;
        double cx = 0, cy = 0;
        for (int k = 1; k + 1 < pts.size(); k += 2) {
            cx += pts.get(k)[0] - (pts.get(k - 1)[0] + pts.get(k + 1)[0]) / 2.0;
            cy += pts.get(k)[1] - (pts.get(k - 1)[1] + pts.get(k + 1)[1]) / 2.0;
            sumCount++;
        }
        int lastEven = (pts.size() - 1) / 2 * 2;
        if (lastEven >= 2) {
            RunLog.event(String.format(Locale.US, "RESULT net drift per full turn %.2f in (real sliding, not offsets)",
                    Math.hypot(pts.get(lastEven)[0], pts.get(lastEven)[1]) / (lastEven / 2.0)));
        }
        if (sumCount == 0) {
            RunLog.event("RESULT not enough turns for an offset estimate");
            return;
        }
        double ex = cy / sumCount / 2.0, ey = -cx / sumCount / 2.0;
        RunLog.event(String.format(Locale.US, "RESULT offset-only wobble %.2f in; forward-axis error %.1f mm -> xOffset %.1f or %.1f; strafe-axis error %.1f mm -> yOffset %.1f or %.1f (sign unverified)",
                2 * Math.hypot(ex, ey), ex * 25.4, PinpointLocalizer.PARAMS.xOffsetMM - ex * 25.4, PinpointLocalizer.PARAMS.xOffsetMM + ex * 25.4,
                ey * 25.4, PinpointLocalizer.PARAMS.yOffsetMM - ey * 25.4, PinpointLocalizer.PARAMS.yOffsetMM + ey * 25.4));
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
