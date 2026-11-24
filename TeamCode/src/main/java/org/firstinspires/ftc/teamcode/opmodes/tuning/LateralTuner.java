package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Tune the LATERAL (field-Y, left/right strafe) PID. Strafes 48 in left, then back,
 * forever. Heading and X are held at 0, isolating the lateral controller.
 *
 * Strafing has less traction than driving straight, so LATERAL gains usually end up
 * a bit higher than AXIAL. Graph {@code dtp/errY_in} and adjust
 * {@code DriveToPose.LATERAL_kP / kD / kS}.
 *
 * Give it ~5 ft of clear space to the left. gamepad1.B = kill switch.
 */
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@TeleOp(name = "Tune - Lateral (strafe 48in)", group = "Tuning")
public class LateralTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        // a run log from 2026-11-17 (runlogs/bringup.log): the gains in force (CONFIG and TUNE lines), every leg's DRIVE lines with its time, final error and overshoot, and a row every 50 ms
        RunLog.open(RunLog.Kind.BRINGUP, "Tune - Lateral", hardwareMap);
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Lateral tuner. Keep still while the Pinpoint calibrates.");
        telemetry.addLine("Clear 5+ ft to the left. START to run, B to kill.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        RunLog.started();

        Actions.cycle(this, drive,
                new Pose2d(0, 0, 0),
                new Pose2d(0, 48, 0));
    }
}
