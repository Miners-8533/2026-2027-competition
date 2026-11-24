package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.RunLog;

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
     * speed control and pusher position control keep running between and during actions.
     */
    public static void runBlocking(LinearOpMode opMode, Action action, Runnable everyLoop) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        boolean running = true;
        while (running && opMode.opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            long profiledAction = LoopProfiler.start();
            running = action.run(packet);
            LoopProfiler.stop("action", profiledAction);   // a drive leg's pose read and drive writes (pose, drive_write) are inside this one
            everyLoop.run();
            long profiledPacket = LoopProfiler.start();
            dashboard.sendTelemetryPacket(packet);
            LoopProfiler.stop("dash_packet", profiledPacket);
            RunLog.tick();
        }
    }

    /**
     * Tuning aid: drives {@code waypoints} in a loop forever (0, 1, 2, ..., 0, 1, ...)
     * with a fresh {@link DriveToPose} for each leg. Watch the {@code dtp/*} values on
     * the FTC Dashboard graph and adjust the {@link DriveToPose} gains live.
     *
     * gamepad1.B is a kill switch: stops the drive and ends the OpMode.
     *
     * With a run log open (the tuners open one from 2026-11-17) every leg writes its DRIVE lines (time, final error, overshoot), and the gains are logged at the start and again
     * whenever a Dashboard edit has changed them, so a tuning session leaves a record of which gains gave which legs.
     */
    public static void cycle(LinearOpMode opMode, MecanumDrive drive, Pose2d... waypoints) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int index = 0;
        DriveToPose leg = new DriveToPose(drive, waypoints[0]);
        String gains = DriveToPose.gainsText();
        RunLog.event("TUNE gains: " + gains);
        final int[] at = {0};
        RunLog.setSampler("x,y,hdg_deg,cmd_fwd,cmd_left,cmd_turn,waypoint", 50, () -> {
            Pose2d p = drive.getPose();   // the pose the leg read this loop: the row reads nothing from the hub
            org.firstinspires.ftc.teamcode.math.PoseVelocity2d c = drive.getLastCommand();
            return String.format(java.util.Locale.US, "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d", p.position.x, p.position.y, p.heading.toDegrees(), c.linearVel.x, c.linearVel.y, c.angVel, at[0]);
        });

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
                at[0] = index;
                leg = new DriveToPose(drive, waypoints[index]);
                String now = DriveToPose.gainsText();
                if (!now.equals(gains)) {
                    gains = now;
                    RunLog.event("TUNE gains changed: " + gains);
                }
            }
            dashboard.sendTelemetryPacket(packet);
            RunLog.tick();
        }
        drive.stop();
    }
}
