package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TransportGate;

/**
 * Test shots with the flywheel and the transport gate together, with no chassis or other mechanism needed
 * (hub configuration: "gate", "fly_motor_enc", and optionally "fly_follow"). Everything is manual; nothing
 * runs behind your back, and the flywheel keeps spinning between shots until you stop it.
 *
 *   A                  flywheel on / off
 *   D-pad up / down    shot speed +/- SPEED_STEP ticks/s     D-pad left / right: -/+ COARSE_STEP
 *   Y                  FIRE: spins up if needed, then runs a salvo as soon as the flywheel is ready
 *                      and the gate has stopped, wherever it rests (gives up after FIRE_TIMEOUT_SEC)
 *   RB                 salvo NOW, without waiting for the flywheel (a dry run of the gate)
 *   X                  reverse: the gate's reject move (330 deg back onto the nearest HOLD)
 *   LB                 gate back to the nearest HOLD
 *   Left stick         jog the gate (up = forward) to place it, then B to set HOLD there
 *   B                  declare the gate's current position to be HOLD
 *   Options            clear a gate or flywheel fault, and cancel a queued fire
 *
 * After each salvo it shows the flywheel speed when the gate started, the lowest speed during the salvo, and
 * the salvo time: that is the data for choosing the shot speed. The gate's TICKS_PER_REV, powers and
 * STOP_EARLY_DEG (see Transport Gate Test) and the flywheel's FEED_FORWARD come from their classes.
 *
 * Starts in a do-nothing gate state, so press LB (or Y) to bring the gate to a HOLD. Keep clear of the wheel and
 * the arm; the gate has no stops.
 */
@Config
@TeleOp(name = "Shot Test (gate + flywheel)", group = "Bring-up")
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
        TransportGate gate = new TransportGate(hardwareMap, TransportGate.Reference.ASSUME_AT_HOLD);
        gate.jog(0.0);

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
        telemetry.addLine("Shot Test. The gate is assumed to be at HOLD right now. Flywheel and gate stay still until you press something.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            TransportGate.State gateState = gate.getState();
            boolean gateFree = gateState == TransportGate.State.JOG || gateState == TransportGate.State.HOLD;

            double stick = -gamepad1.left_stick_y;
            if (gateFree && Math.abs(stick) > 0.05) {
                gate.jog(stick * JOG_MAX_POWER);
            } else if (gateState == TransportGate.State.JOG) {
                gate.jog(0.0);
            }

            if (gamepad1.bWasPressed() && gateFree) {
                gate.markHoldHere();
                lastEvent = "HOLD set at the gate's current position.";
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
                gate.hold();
                lastEvent = "Gate going to the nearest HOLD.";
            }
            if (gamepad1.xWasPressed()) {
                gate.startReject();
                lastEvent = gate.isCalibrated() ? "Reverse (reject) started." : "Reverse ignored: the gate is not calibrated.";
            }
            if (gamepad1.rightBumperWasPressed()) {
                if (gate.getState() == TransportGate.State.JOG) gate.hold();
                gate.startSalvo();
                lastEvent = "Dry salvo started (not waiting for the flywheel).";
            }
            if (gamepad1.yWasPressed()) {
                if (!gate.isCalibrated()) {
                    lastEvent = "Fire ignored: the gate is not calibrated.";
                } else {
                    if (gate.getState() == TransportGate.State.JOG) gate.hold();
                    spinning = true;
                    fireQueued = true;
                    fireTimer.reset();
                    lastEvent = "Fire queued: waiting for the flywheel and the gate.";
                }
            }
            if (gamepad1.optionsWasPressed()) {
                gate.clearStall();
                flywheel.clearFault();
                fireQueued = false;
                lastEvent = "Faults cleared, fire cancelled.";
            }

            flywheel.setTargetVelocity(spinning ? setpoint : 0.0);

            if (fireQueued) {
                if (fireTimer.seconds() > FIRE_TIMEOUT_SEC) {
                    fireQueued = false;
                    lastEvent = "Fire dropped: the flywheel or the gate was not ready in time.";
                } else if (flywheel.isReady() && gate.isStopped()) {
                    shotStartSpeed = flywheel.getFilteredVelocity();
                    shotMinSpeed = shotStartSpeed;
                    shotSetpoint = setpoint;
                    shotTimer.reset();
                    gate.startSalvo();
                    shotActive = true;
                    fireQueued = false;
                    lastEvent = "Salvo started.";
                }
            }

            flywheel.update();
            gate.update();

            if (shotActive) {
                shotMinSpeed = Math.min(shotMinSpeed, flywheel.getVelocity());
                boolean salvoDone = gate.getState() != TransportGate.State.SALVO;
                if (salvoDone) {
                    shotActive = false;
                    shots++;
                    lastShot = String.format("#%d  setpoint %.0f, at release %.0f, lowest %.0f (%.0f%% of setpoint), salvo %.2f s%s",
                            shots, shotSetpoint, shotStartSpeed, shotMinSpeed, 100.0 * shotMinSpeed / Math.max(shotSetpoint, 1.0),
                            shotTimer.seconds(), gate.isStalled() ? "  GATE FAULT" : "");
                }
            }

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Flywheel", "%s   shot speed %.0f ticks/s", spinning ? "ON" : "off", setpoint);
            flywheel.log(telemetry);
            gate.log(telemetry);
            telemetry.addData("Fire", fireQueued
                    ? String.format("QUEUED %.1f s: flywheel %s, gate %s", fireTimer.seconds(),
                            flywheel.isReady() ? "ready" : "NOT ready",
                            gate.isStopped() ? "stopped" : "NOT stopped (" + gate.getState() + ")")
                    : shotActive ? "salvo running" : "idle");
            telemetry.addData("Last shot", lastShot);
            telemetry.addData("Last event", lastEvent);
            telemetry.addLine("A flywheel, D-pad speed, Y fire, RB dry salvo, X reverse, LB hold, stick jog, B set HOLD, Options clear");
            telemetry.update();
        }
        flywheel.stop();
        gate.jog(0.0);
        gate.update();
    }
}
