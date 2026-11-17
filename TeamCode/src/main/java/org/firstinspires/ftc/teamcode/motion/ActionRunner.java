package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * The action runner (specs/motion.md section 1). An auto runs one action to completion. Each loop: read the pose once,
 * run the action, then the opmode's every-loop work (the mechanisms update), then send the telemetry packet, then give
 * the run log its tick. It stops as soon as the opmode stops.
 */
public final class ActionRunner {
    private ActionRunner() { }

    /** The opmode's every-loop work. */
    public interface EveryLoop { void run(); }

    public static void run(LinearOpMode op, MecanumDrive drive, Action action, EveryLoop everyLoop) {
        FtcDashboard dash = FtcDashboard.getInstance();
        while (op.opModeIsActive()) {
            drive.updatePose();
            TelemetryPacket packet = new TelemetryPacket();
            LoopProfiler.begin("action");
            boolean more = action.run(packet);
            LoopProfiler.end("action");
            everyLoop.run();
            LoopProfiler.begin("dash_packet");
            if (dash != null) dash.sendTelemetryPacket(packet);
            LoopProfiler.end("dash_packet");
            RunLog.tick();
            if (!more) return;
        }
    }
}
