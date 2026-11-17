package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Pinpoint Offset Check (specs/support-opmodes.md sections 3.2 and 4.1): for when a pod or the Pinpoint is moved.
 * Spin the robot in place several turns; the wobble of the reported position and the per-axis error estimates give
 * the pod offsets to put in the chassis profile.
 *
 * Controls (gamepad 1): A starts or reruns the spin; B stops at once. The position is sampled at every half turn.
 *
 * The robot spins 3 turns at 0.25 turn power (45 s at most) and the position is sampled at every half turn. Each
 * sample taken half a turn into a revolution, less the mean of the samples before and after it (which removes a
 * steady drift), is the far point of the offset error's circle, (-2 ey, 2 ex): ex is the error along the forward axis,
 * which follows the x offset, and ey along the strafe axis, which follows the y offset. The net drift per full turn
 * is reported apart. The sign of the correction has not been verified on hardware: nudge an offset a few millimetres,
 * rerun, and keep the direction that shrinks the error.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Config
@Disabled
@TeleOp(name = "Pinpoint Offset Check", group = "2 Bring-up")
public final class PinpointOffsetCheck extends TeamOpMode {
    public static double TURN_POWER = 0.25, TIMEOUT_S = 45.0;
    public static int TURNS = 3;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Pinpoint Offset Check"; }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, new Pose(0, 0, 0));
        while (opModeInInit()) {
            drive.updatePose();
            telemetry.addLine("Pinpoint Offset Check: clear floor, room to spin. Keep the robot still until Start.");
            telemetry.addLine("A starts or reruns the spin (" + TURNS + " turns at " + TURN_POWER + "); B stops at once.");
            telemetry.addLine("Pinpoint: " + drive.localizer.statusText());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        RunLog.eventf("PINPOINT offset check: configured pod offsets x %.1f mm, y %.1f mm", profile.xPodOffsetMm, profile.yPodOffsetMm);
        RunLog.sampler("x,y,hdg_deg,turned_deg,cmd_turn", 0.050, new RunLog.Sampler() {
            @Override public String row() {
                Pose p = drive.pose();
                return String.format(Locale.US, "%.2f,%.2f,%.2f,%.1f,%.2f", p.x, p.y, p.headingDeg(), turned, drive.cmdTurn());
            }
        });
        Edge a = new Edge(), b = new Edge();
        boolean spinning = false;
        List<double[]> samples = new ArrayList<>();     // {x, y, heading rad} at every half turn
        double lastH = 0, sumX = 0, sumY = 0, maxX = 0, minX = 0, maxY = 0, minY = 0;
        int n = 0;
        String result = "";
        while (opModeIsActive()) {
            drive.updatePose();
            Pose p = drive.pose();
            if (b.pressed(gamepad1.b) && spinning) {
                spinning = false;
                drive.stop();
                RunLog.eventf("PINPOINT offset check stopped by B after %.0f deg", turned);
                result = "stopped by B";
            } else if (a.pressed(gamepad1.a)) {
                spinning = true;
                spinStartNs = System.nanoTime();
                turned = 0;
                lastH = p.h;
                samples.clear();
                samples.add(new double[] {p.x, p.y, p.h});
                sumX = sumY = 0;
                n = 0;
                maxX = minX = p.x;
                maxY = minY = p.y;
                RunLog.eventf("PINPOINT offset check spin started at %s: %d turns at %.2f turn power", p, TURNS, TURN_POWER);
            }
            if (spinning) {
                turned += Math.toDegrees(Angle.diff(p.h, lastH));
                lastH = p.h;
                sumX += p.x;
                sumY += p.y;
                n++;
                maxX = Math.max(maxX, p.x);
                minX = Math.min(minX, p.x);
                maxY = Math.max(maxY, p.y);
                minY = Math.min(minY, p.y);
                if (Math.abs(turned) >= 180.0 * samples.size()) {
                    samples.add(new double[] {p.x, p.y, p.h});
                    RunLog.eventf("PINPOINT offset check half turn %d: position (%.2f, %.2f), heading %.1f deg", samples.size() - 1, p.x, p.y, p.headingDeg());
                }
                if (samples.size() - 1 < 2 * TURNS && (System.nanoTime() - spinStartNs) / 1e9 > TIMEOUT_S) {
                    spinning = false;
                    drive.stop();
                    result = "given up after " + TIMEOUT_S + " s";
                    RunLog.eventf("PINPOINT offset check given up after %.0f s and %.0f deg", TIMEOUT_S, Math.abs(turned));
                } else if (samples.size() - 1 >= 2 * TURNS) {
                    spinning = false;
                    drive.stop();
                    // Each sample half a turn into a revolution, less the mean of its neighbors, is (-2 ey, 2 ex).
                    double ex = 0, ey = 0;
                    int odd = 0;
                    for (int k = 1; k + 1 < samples.size(); k += 2) {
                        double[] before = samples.get(k - 1), at = samples.get(k), after = samples.get(k + 1);
                        double cx = at[0] - (before[0] + after[0]) / 2, cy = at[1] - (before[1] + after[1]) / 2;
                        ex += cy / 2;
                        ey += -cx / 2;
                        odd++;
                    }
                    ex /= Math.max(1, odd);
                    ey /= Math.max(1, odd);
                    double[] first = samples.get(0), end = samples.get(samples.size() - 1);
                    double wobble = Math.max(maxX - minX, maxY - minY) / 2;
                    result = String.format(Locale.US, "wobble %.2f in; offset error along the forward axis (x offset) %+.1f mm, along the strafe axis (y offset) %+.1f mm; "
                                    + "net drift per turn (%.2f, %.2f) in; configured offsets x %.1f mm, y %.1f mm. The sign of the correction is not verified: nudge and rerun",
                            wobble, ex * 25.4, ey * 25.4, (end[0] - first[0]) / TURNS, (end[1] - first[1]) / TURNS, profile.xPodOffsetMm, profile.yPodOffsetMm);
                    RunLog.event("PINPOINT offset check result: " + result);
                } else {
                    drive.drive(0, 0, TURN_POWER);
                }
            } else {
                drive.stop();
            }
            RunLog.tick();
            telemetry.addLine(spinning ? String.format(Locale.US, "spinning: %.0f of %d deg (B stops)", Math.abs(turned), 360 * TURNS) : "A starts or reruns the spin");
            telemetry.addLine("Pose " + p + ", Pinpoint " + drive.localizer.statusText());
            telemetry.addLine(result);
            telemetry.update();
        }
    }

    private double turned;
    private long spinStartNs;
}
