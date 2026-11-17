package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Localizer;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Pinpoint Push Test (specs/support-opmodes.md section 4.1): for when a pod or the Pinpoint is moved or rewired.
 * Push the robot forward, sideways and around by hand; the opmode checks the sign of each axis and so the pod
 * directions. No motor is driven. Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@TeleOp(name = "Pinpoint Push Test", group = "2 Bring-up")
public final class PinpointPushTest extends TeamOpMode {
    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Pinpoint Push Test"; }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        MecanumDrive.bulkAuto(hardwareMap);
        Localizer loc = Localizer.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
        telemetry.addLine("Pinpoint Push Test: keep the robot still, then press Start.");
        telemetry.addLine(loc.note());
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        markStarted();
        RunLog.event("PINPOINT " + loc.note());
        double maxX = 0, minX = 0, maxY = 0, minY = 0, maxH = 0, minH = 0;
        boolean loggedX = false, loggedY = false, loggedH = false;
        while (opModeIsActive()) {
            loc.update();
            Pose p = loc.pose();
            maxX = Math.max(maxX, p.x);
            minX = Math.min(minX, p.x);
            maxY = Math.max(maxY, p.y);
            minY = Math.min(minY, p.y);
            maxH = Math.max(maxH, p.headingDeg());
            minH = Math.min(minH, p.headingDeg());
            String x = verdict(maxX, minX, 6), y = verdict(maxY, minY, 6), h = verdict(maxH, minH, 30);
            if (!loggedX && !x.startsWith("push")) { loggedX = true; RunLog.event("PINPOINT push test forward axis: " + x); }
            if (!loggedY && !y.startsWith("push")) { loggedY = true; RunLog.event("PINPOINT push test sideways axis: " + y); }
            if (!loggedH && !h.startsWith("push")) { loggedH = true; RunLog.event("PINPOINT push test turn: " + h); }
            RunLog.tick();
            telemetry.addLine(String.format(Locale.US, "pose %.1f, %.1f, %.0f deg, status %s", p.x, p.y, p.headingDeg(), loc.statusText()));
            telemetry.addLine("1. Push FORWARD 6 in (toward the intake): " + x);
            telemetry.addLine("2. Push to the robot's LEFT 6 in: " + y);
            telemetry.addLine("3. Turn COUNTERCLOCKWISE 30 deg (seen from above): " + h);
            telemetry.addLine("WRONG WAY means that pod's direction in the chassis profile is reversed.");
            telemetry.update();
        }
    }

    private static String verdict(double max, double min, double need) {
        if (max >= need && max >= -min) return "OK (reads +" + String.format(Locale.US, "%.1f", max) + ")";
        if (-min >= need) return "WRONG WAY (reads " + String.format(Locale.US, "%.1f", min) + ")";
        return "push...";
    }
}
