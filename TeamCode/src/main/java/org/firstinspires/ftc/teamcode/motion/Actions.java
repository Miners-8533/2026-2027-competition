package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;

/** Helpers for running {@link Action}s from a LinearOpMode. */
public final class Actions {

    private Actions() {}

    /**
     * Runs {@code action} to completion, one {@link Action#run} call per loop, sending
     * each loop's telemetry packet to FTC Dashboard. Stops early if the OpMode stops.
     */
    public static void runBlocking(LinearOpMode opMode, Action action) {
        runBlocking(opMode, action, () -> {});
    }

    /**
     * As {@link #runBlocking(LinearOpMode, Action)}, and also calls {@code everyLoop} once per
     * loop after the action runs. Autos pass the mechanisms' {@code update()} here so flywheel
     * speed control and gate position control keep running between and during actions.
     */
    public static void runBlocking(LinearOpMode opMode, Action action, Runnable everyLoop) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        boolean running = true;
        while (running && opMode.opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);
            everyLoop.run();
            dashboard.sendTelemetryPacket(packet);
        }
    }

    /**
     * Tuning aid: drives {@code waypoints} in a loop forever (0, 1, 2, ..., 0, 1, ...)
     * with a fresh {@link DriveToPose} for each leg. Watch the {@code dtp/*} values on
     * the FTC Dashboard graph and adjust the {@link DriveToPose} gains live.
     *
     * gamepad1.B is a kill switch: stops the drive and ends the OpMode.
     */
    public static void cycle(LinearOpMode opMode, MecanumDrive drive, Pose2d... waypoints) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int index = 0;
        DriveToPose leg = new DriveToPose(drive, waypoints[0]);

        while (opMode.opModeIsActive()) {
            if (opMode.gamepad1.b) {
                drive.stop();
                opMode.requestOpModeStop();
                return;
            }
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("cycle/waypoint", index);
            if (!leg.run(packet)) {
                index = (index + 1) % waypoints.length;
                leg = new DriveToPose(drive, waypoints[index]);
            }
            dashboard.sendTelemetryPacket(packet);
        }
        drive.stop();
    }
}
