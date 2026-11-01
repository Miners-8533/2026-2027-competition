package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;

/**
 * Test shots with the flywheel and the pusher together, with no chassis or other mechanism needed
 * (hub configuration: "gate" and "fly_motor_enc"). Everything is manual; nothing
 * runs behind your back, and the flywheel keeps spinning between shots until you stop it.
 *
 *   A                  flywheel on / off
 *   D-pad up / down    shot speed +/- SPEED_STEP ticks/s     D-pad left / right: -/+ COARSE_STEP
 *   Y                  FIRE: spins up if needed, then runs a salvo as soon as the flywheel is ready
 *                      and the pusher has stopped, wherever it rests (gives up after FIRE_TIMEOUT_SEC)
 *   RB                 salvo NOW, without waiting for the flywheel (a dry run of the pusher)
 *   X                  reverse: the pusher's reject move (330 deg back onto the nearest HOLD)
 *   LB                 pusher back to the nearest HOLD
 *   Left stick         jog the pusher (up = forward) to place it, then B to set HOLD there
 *   B                  declare the pusher's current position to be HOLD
 *   Back            clear a pusher or flywheel fault, and cancel a queued fire
 *
 * After each salvo it shows the flywheel speed when the pusher started, the lowest speed during the salvo, and
 * the salvo time: that is the data for choosing the shot speed. The pusher's TICKS_PER_REV, powers and
 * STOP_EARLY_DEG and the flywheel's FEED_FORWARD come from their classes.
 *
 * Starts in a do-nothing pusher state, so press LB (or Y) to bring the pusher to a HOLD. Keep clear of the wheel and
 * the arm; the pusher has no stops.
 */
@Config
@TeleOp(name = "Shot Test (pusher + flywheel)", group = "Bring-up")
public class ShotTest extends LinearOpMode {

    public static double START_SPEED = 800.0;
    public static double SPEED_STEP = 25.0;
    public static double COARSE_STEP = 100.0;
    public static double FIRE_TIMEOUT_SEC = 4.0;
    public static double JOG_MAX_POWER = 0.3;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Pusher pusher = new Pusher(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);
        pusher.jog(0.0);

        boolean spinning = false;
        double setpoint = START_SPEED;
        boolean fireQueued = false;
        ElapsedTime fireTimer = new ElapsedTime();

        boolean shotActive = false;
        double shotStartSpeed = 0, shotMinSpeed = 0, shotSetpoint = 0;
        ElapsedTime shotTimer = new ElapsedTime();
        int shots = 0;
        String lastShot = "-";
        String lastEvent = "-";

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Shot Test. The pusher is assumed to be at HOLD right now. Flywheel and pusher stay still until you press something.");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        double loopMs = 0.0, loopMaxMs = 0.0;
        while (opModeIsActive()) {
            double thisLoopMs = loopTimer.milliseconds();
            loopTimer.reset();
            loopMs = 0.95 * loopMs + 0.05 * thisLoopMs;
            loopMaxMs = Math.max(0.99 * loopMaxMs, thisLoopMs);
            Pusher.State pusherState = pusher.getState();
            boolean pusherFree = pusherState == Pusher.State.JOG || pusherState == Pusher.State.HOLD;

            double stick = -gamepad1.left_stick_y;
            if (pusherFree && Math.abs(stick) > 0.05) {
                pusher.jog(stick * JOG_MAX_POWER);
            } else if (pusherState == Pusher.State.JOG) {
                pusher.jog(0.0);
            }

            if (gamepad1.bWasPressed() && pusherFree) {
                pusher.markHoldHere();
                lastEvent = "HOLD set at the pusher's current position.";
            }
            if (gamepad1.aWasPressed()) {
                spinning = !spinning;
                if (!spinning) fireQueued = false;
            }
            if (gamepad1.dpadUpWasPressed()) setpoint += SPEED_STEP;
            if (gamepad1.dpadDownWasPressed()) setpoint = Math.max(0.0, setpoint - SPEED_STEP);
            if (gamepad1.dpadRightWasPressed()) setpoint += COARSE_STEP;
            if (gamepad1.dpadLeftWasPressed()) setpoint = Math.max(0.0, setpoint - COARSE_STEP);

            if (gamepad1.leftBumperWasPressed()) {
                pusher.hold();
                lastEvent = "Pusher going to the nearest HOLD.";
            }
            if (gamepad1.xWasPressed()) {
                pusher.startReject();
                lastEvent = pusher.isCalibrated() ? "Reverse (reject) started." : "Reverse ignored: the pusher is not calibrated.";
            }
            if (gamepad1.rightBumperWasPressed()) {
                if (pusher.getState() == Pusher.State.JOG) pusher.hold();
                pusher.startSalvo();
                lastEvent = "Dry salvo started (not waiting for the flywheel).";
            }
            if (gamepad1.yWasPressed()) {
                if (!pusher.isCalibrated()) {
                    lastEvent = "Fire ignored: the pusher is not calibrated.";
                } else {
                    if (pusher.getState() == Pusher.State.JOG) pusher.hold();
                    spinning = true;
                    fireQueued = true;
                    fireTimer.reset();
                    lastEvent = "Fire queued: waiting for the flywheel and the pusher.";
                }
            }
            if (gamepad1.backWasPressed()) {
                pusher.clearStall();
                fireQueued = false;
                lastEvent = "Pusher fault cleared, fire cancelled.";
            }

            flywheel.setTargetVelocity(spinning ? setpoint : 0.0);

            if (fireQueued) {
                if (fireTimer.seconds() > FIRE_TIMEOUT_SEC) {
                    fireQueued = false;
                    lastEvent = "Fire dropped: the flywheel or the pusher was not ready in time.";
                } else if (flywheel.isReady() && pusher.isStopped()) {
                    shotStartSpeed = flywheel.getFilteredVelocity();
                    shotMinSpeed = shotStartSpeed;
                    shotSetpoint = setpoint;
                    shotTimer.reset();
                    pusher.startSalvo();
                    shotActive = true;
                    fireQueued = false;
                    lastEvent = "Salvo started.";
                }
            }

            flywheel.update();
            pusher.update();

            if (shotActive) {
                shotMinSpeed = Math.min(shotMinSpeed, flywheel.getVelocity());
                boolean salvoDone = pusher.getState() != Pusher.State.SALVO;
                if (salvoDone) {
                    shotActive = false;
                    shots++;
                    lastShot = String.format("#%d  setpoint %.0f, at release %.0f, lowest %.0f (%.0f%% of setpoint), salvo %.2f s%s",
                            shots, shotSetpoint, shotStartSpeed, shotMinSpeed, 100.0 * shotMinSpeed / Math.max(shotSetpoint, 1.0),
                            shotTimer.seconds(), pusher.isStalled() ? "  PUSHER FAULT" : "");
                }
            }

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Flywheel", "%s   shot speed %.0f ticks/s", spinning ? "ON" : "off", setpoint);
            flywheel.log(telemetry);
            pusher.log(telemetry);
            telemetry.addData("Fire", fireQueued
                    ? String.format("QUEUED %.1f s: flywheel %s, pusher %s", fireTimer.seconds(),
                            flywheel.isReady() ? "ready" : "NOT ready",
                            pusher.isStopped() ? "stopped" : "NOT stopped (" + pusher.getState() + ")")
                    : shotActive ? "salvo running" : "idle");
            telemetry.addData("Loop (ms)", "%.1f avg, %.1f recent max", loopMs, loopMaxMs);
            telemetry.addData("Last shot", lastShot);
            telemetry.addData("Last event", lastEvent);
            telemetry.addLine("A flywheel, D-pad speed, Y fire, RB dry salvo, X reverse, LB hold, stick jog, B set HOLD, Back clear");
            telemetry.update();
        }
        flywheel.stop();
        pusher.jog(0.0);
        pusher.update();
    }
}
