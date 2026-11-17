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
 * SPEC GAP: the page does not give the spin's power or number of turns, nor how the estimates are worked out.
 * Chosen here: 0.35 turn power, 5 turns. The estimate is derived, not taken from the page: if the X pod really sits
 * dx farther left than its configured offset and the Y pod dy farther forward, a robot turning on the spot is
 * reported moving at (-w dx, +w dy) in its own frame, and over a half turn that adds up to (-2 dy, -2 dx) in the frame
 * the robot had when the half turn began. So dy = -(reported x move) / 2 and dx = -(reported y move) / 2, averaged
 * over the half turns. A drift of the real spin center changes sign with every half turn and averages out over an
 * even number of them. Check the result by putting the new offsets in and spinning again: the wobble should shrink.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Config
@Disabled
@TeleOp(name = "Pinpoint Offset Check", group = "2 Bring-up")
public final class PinpointOffsetCheck extends TeamOpMode {
    public static double TURN_POWER = 0.35;
    public static int TURNS = 5;

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
                if (samples.size() - 1 >= 2 * TURNS) {
                    spinning = false;
                    drive.stop();
                    // Each half turn's reported move, in the frame the robot had when that half turn began.
                    double mx = 0, my = 0;
                    int halves = samples.size() - 1;
                    for (int k = 0; k < halves; k++) {
                        double[] s0 = samples.get(k), s1 = samples.get(k + 1);
                        double dx = s1[0] - s0[0], dy = s1[1] - s0[1], c = Math.cos(s0[2]), s = Math.sin(s0[2]);
                        mx += dx * c + dy * s;
                        my += -dx * s + dy * c;
                    }
                    mx /= halves;
                    my /= halves;
                    double podXErrIn = -my / 2, podYErrIn = -mx / 2;
                    double wobble = Math.max(maxX - minX, maxY - minY) / 2;
                    result = String.format(Locale.US, "wobble %.2f in (mean position %.2f, %.2f); estimated errors: x pod %+.1f mm, y pod %+.1f mm; try offsets x %.1f mm, y %.1f mm (configured %.1f, %.1f)",
                            wobble, sumX / n, sumY / n, podXErrIn * 25.4, podYErrIn * 25.4,
                            profile.xPodOffsetMm + podXErrIn * 25.4, profile.yPodOffsetMm + podYErrIn * 25.4, profile.xPodOffsetMm, profile.yPodOffsetMm);
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
}
