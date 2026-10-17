package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;

/**
 * Open-loop response test: ONE clean command at a time, no stick mixing, so any sideways motion
 * or heading change during a forward run (and so on) is drift, not driver input.
 *
 *   D-pad up / down     forward / backward
 *   D-pad left / right  strafe left / strafe right
 *   Left / right bumper rotate counterclockwise / clockwise
 *   X / B               power down / up by 0.1, between 0.1 and 1.0 (while idle)
 *   A                   toggle HOLD (runs while held) / PULSE (runs PULSE_SEC per press)
 *   Options             clear the results table and zero the pose
 *
 * Two inputs at once output nothing. A run ends when the input is released (or the pulse
 * ends); the drive brakes, waits SETTLE_SEC for the robot to stop, then the Pinpoint's start-to-end
 * displacement is resolved in the robot's STARTING frame and added to the results table.
 *   moved  - distance travelled in the commanded direction (deg for rotation), positive = correct way
 *   cross  - sideways drift: left (+) for forward/backward, forward (+) for strafes; for a
 *            rotation it is the total translation (wander) instead
 *   heading- heading change during the run
 * The Pinpoint is the only sensor, so confirm a few runs against a tape measure.
 *
 * Needs open floor. Start at low POWER; the step is instant (no ramp), which is the point.
 */
@Config
@TeleOp(name = "Open Loop Test (D-pad)", group = "Bring-up")
public class OpenLoopTest extends LinearOpMode {

    public static double POWER = 0.3;
    public static double PULSE_SEC = 1.0;
    public static double SETTLE_SEC = 0.7;

    private enum Cmd { FORWARD, BACKWARD, LEFT, RIGHT, CCW, CW, NONE }
    private enum State { IDLE, RUNNING, SETTLING }

    private int inputCount;

    private Cmd readCommand() {
        Cmd found = Cmd.NONE;
        inputCount = 0;
        if (gamepad1.dpad_up) { found = Cmd.FORWARD; inputCount++; }
        if (gamepad1.dpad_down) { found = Cmd.BACKWARD; inputCount++; }
        if (gamepad1.dpad_left) { found = Cmd.LEFT; inputCount++; }
        if (gamepad1.dpad_right) { found = Cmd.RIGHT; inputCount++; }
        if (gamepad1.left_bumper) { found = Cmd.CCW; inputCount++; }
        if (gamepad1.right_bumper) { found = Cmd.CW; inputCount++; }
        return inputCount == 1 ? found : Cmd.NONE;
    }

    private static void apply(MecanumDrive drive, Cmd cmd, double p) {
        double forward = 0, left = 0, turn = 0;
        switch (cmd) {
            case FORWARD: forward = p; break;
            case BACKWARD: forward = -p; break;
            case LEFT: left = p; break;
            case RIGHT: left = -p; break;
            case CCW: turn = p; break;
            case CW: turn = -p; break;
            default: break;
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, left), turn));
    }

    private static String summarize(Cmd cmd, Pose2d p0, Pose2d p1, double headingDeg, double runSec, double power) {
        double h0 = p0.heading.toDouble();
        double dx = p1.position.x - p0.position.x, dy = p1.position.y - p0.position.y;
        double along = dx * Math.cos(h0) + dy * Math.sin(h0);
        double left = -dx * Math.sin(h0) + dy * Math.cos(h0);

        if (cmd == Cmd.CCW || cmd == Cmd.CW) {
            double turned = cmd == Cmd.CCW ? headingDeg : -headingDeg;
            return String.format("p %.2f  turned %+.1f deg  wander %.1f in  %.2f s  %.0f deg/s",
                    power, turned, Math.hypot(along, left), runSec, runSec > 0 ? turned / runSec : 0);
        }
        double moved, cross;
        switch (cmd) {
            case FORWARD: moved = along; cross = left; break;
            case BACKWARD: moved = -along; cross = left; break;
            case LEFT: moved = left; cross = along; break;
            default: moved = -left; cross = along; break;
        }
        return String.format("p %.2f  moved %+.1f in  cross %+.1f in (%.0f%%)  heading %+.1f deg  %.2f s  %.1f in/s",
                power, moved, cross, 100 * Math.abs(cross) / Math.max(Math.abs(moved), 0.1),
                headingDeg, runSec, runSec > 0 ? moved / runSec : 0);
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Open Loop Test. Keep still while the Pinpoint calibrates.");
        telemetry.update();
        waitForStart();

        State state = State.IDLE;
        Cmd runCmd = Cmd.NONE;
        boolean armed = true, holdMode = true;
        ElapsedTime phase = new ElapsedTime();
        Pose2d p0 = new Pose2d(0, 0, 0);
        double runSec = 0, lastHeading = 0, unwrappedDeg = 0, power = 0;
        String[] results = new String[6];

        while (opModeIsActive()) {
            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            double h = pose.heading.toDouble();
            if (state == State.IDLE) {
                unwrappedDeg = 0;
            } else {
                unwrappedDeg += Math.toDegrees(Rotation2d.normalize(h - lastHeading));
            }
            lastHeading = h;

            Cmd cmd = readCommand();
            if (cmd == Cmd.NONE && inputCount == 0) armed = true;

            if (gamepad1.optionsWasPressed()) {
                results = new String[6];
                drive.setPose(new Pose2d(0, 0, 0));
            }
            if (gamepad1.aWasPressed()) holdMode = !holdMode;
            if (state == State.IDLE) {
                if (gamepad1.xWasPressed()) POWER = Math.max(0.1, POWER - 0.1);
                if (gamepad1.bWasPressed()) POWER = Math.min(1.0, POWER + 0.1);
            }

            switch (state) {
                case IDLE:
                    drive.stop();
                    if (cmd != Cmd.NONE && armed) {
                        runCmd = cmd; p0 = pose; power = POWER; armed = false;
                        phase.reset();
                        state = State.RUNNING;
                    }
                    break;
                case RUNNING:
                    apply(drive, runCmd, power);
                    if (holdMode ? cmd != runCmd : phase.seconds() >= PULSE_SEC) {
                        runSec = phase.seconds();
                        drive.stop();
                        phase.reset();
                        state = State.SETTLING;
                    }
                    break;
                case SETTLING:
                    drive.stop();
                    if (phase.seconds() >= SETTLE_SEC) {
                        results[runCmd.ordinal()] = summarize(runCmd, p0, pose, unwrappedDeg, runSec, power);
                        state = State.IDLE;
                    }
                    break;
            }

            telemetry.addData("State", state + (state == State.RUNNING ? " " + runCmd : ""));
            telemetry.addData("Mode / power", "%s   POWER %.2f   PULSE_SEC %.1f", holdMode ? "HOLD" : "PULSE", POWER, PULSE_SEC);
            if (inputCount > 1) telemetry.addLine("More than one input held: output is zero. Press one at a time.");
            if (state == State.IDLE && !armed) telemetry.addLine("Release every input to arm the next run.");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, Math.toDegrees(h));
            telemetry.addData("Velocity x / y (in/s), turn (deg/s)", "%.1f / %.1f / %.0f",
                    vel.linearVel.x, vel.linearVel.y, Math.toDegrees(vel.angVel));
            telemetry.addLine("--- last result per command ---");
            String[] names = {"Forward", "Backward", "Strafe left", "Strafe right", "Rotate CCW", "Rotate CW"};
            for (int i = 0; i < 6; i++) {
                telemetry.addData(names[i], results[i] == null ? "-" : results[i]);
            }
            telemetry.addLine("Dpad = drive, bumpers = rotate, X/B = power, A = hold/pulse, Options = clear");
            telemetry.update();
        }
        drive.stop();
    }
}
