package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Localizer;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Pinpoint Push Test (specs/support-opmodes.md sections 3.2 and 4.1): for when a pod or the Pinpoint is moved or
 * rewired. Three guided pushes by hand, with the drive motors floating: forward about 6 in (expect +x), left about
 * 6 in (expect +y), counterclockwise about 45 deg (expect + heading). Each checks the sign of one axis and so a pod
 * direction. No motor is driven.
 *
 * Controls (gamepad 1): A skips a step; Back applies the settings again, zeroes the pose and restarts (one of the six
 * support opmodes that use Back).
 *
 * A step is judged once the push reads 6 in, or the turn 45 deg.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@TeleOp(name = "Pinpoint Push Test", group = "2 Bring-up")
public final class PinpointPushTest extends TeamOpMode {
    private static final double JUDGE_IN = 6.0, JUDGE_DEG = 45.0;
    private static final String[] ASK = {
            "1. Push FORWARD about 6 in (toward the intake): expect +x",
            "2. Push to the robot's LEFT about 6 in: expect +y",
            "3. Turn COUNTERCLOCKWISE about 45 deg (seen from above): expect + heading"};
    private static final String[] AXIS = {"forward (x pod)", "left (y pod)", "turn (heading)"};

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Pinpoint Push Test"; }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        MecanumDrive.bulkAuto(hardwareMap);
        // The drive motors float, so the robot can be pushed by hand.
        for (String name : new String[] {ChassisProfile.LEFT_FRONT, ChassisProfile.LEFT_BACK, ChassisProfile.RIGHT_BACK, ChassisProfile.RIGHT_FRONT}) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, name);
            if (m != null) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                m.setPower(0);
            }
        }
        Localizer loc = Localizer.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
        telemetry.addLine("Pinpoint Push Test: keep the robot still, then press Start.");
        telemetry.addLine(loc.note());
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        markStarted();
        RunLog.event("PINPOINT " + loc.note());
        Edge skip = new Edge(), again = new Edge();
        String[] result = new String[3];
        int step = 0;
        Pose from = loc.pose();
        double turned = 0, lastH = from.h;
        while (opModeIsActive()) {
            loc.update();
            Pose p = loc.pose();
            turned += Math.toDegrees(Angle.diff(p.h, lastH));
            lastH = p.h;
            if (again.pressed(gamepad1.back)) {
                // Apply the settings again, zero the pose and restart.
                loc = Localizer.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
                RunLog.event("PINPOINT push test restarted: " + loc.note());
                result = new String[3];
                step = 0;
                from = loc.pose();
                turned = 0;
                lastH = from.h;
                continue;
            }
            // Movement since the step began, in the robot's frame as it stood then.
            double dx = p.x - from.x, dy = p.y - from.y;
            double fwd = dx * Math.cos(from.h) + dy * Math.sin(from.h), left = -dx * Math.sin(from.h) + dy * Math.cos(from.h);
            if (step < 3) {
                double reading = step == 0 ? fwd : step == 1 ? left : turned, need = step == 2 ? JUDGE_DEG : JUDGE_IN;
                String verdict = null;
                if (skip.pressed(gamepad1.a)) verdict = "SKIPPED";
                else if (reading >= need) verdict = "OK";
                else if (reading <= -need) verdict = "WRONG WAY: that pod's direction in the chassis profile is reversed";
                if (verdict != null) {
                    result[step] = String.format(Locale.US, "%s (moved forward %+.1f in, left %+.1f in, turned %+.0f deg)", verdict, fwd, left, turned);
                    RunLog.eventf("PINPOINT push test %s: %s", AXIS[step], result[step]);
                    step++;
                    from = p;
                    turned = 0;
                    if (step == 3) RunLog.eventf("PINPOINT push test done: %s | %s | %s", result[0], result[1], result[2]);
                }
            }
            RunLog.tick();
            telemetry.addLine(String.format(Locale.US, "pose %.1f, %.1f, %.0f deg, status %s", p.x, p.y, p.headingDeg(), loc.statusText()));
            for (int i = 0; i < 3; i++) {
                telemetry.addLine((i == step ? "> " : "  ") + ASK[i] + (result[i] != null ? ": " + result[i] : i == step
                        ? String.format(Locale.US, ": reads %+.1f", i == 0 ? fwd : i == 1 ? left : turned) : ""));
            }
            telemetry.addLine(step < 3 ? "A skips this step. Back applies the settings again and restarts." : "Done. Back applies the settings again and restarts.");
            telemetry.update();
        }
    }
}
