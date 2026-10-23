package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.InstantAction;

/**
 * The shooter flywheel: one motor with an encoder, hood fixed. Design notes: design/mechanisms.md.
 *
 * Control is last season's bang-bang plus feedforward: full power while below the setpoint, a single
 * FEED_FORWARD motor command (0 to 1) at or above it, nothing when the target is 0. Plain on purpose;
 * add compensation only after this has been tried. It does NOT wait for recovery between balls: the
 * transport gate sets the ball spacing, and {@link #isReady()} is meant to be checked before the
 * FIRST ball of a salvo only.
 *
 * Call {@link #update()} once per loop (autos pass it to Actions.runBlocking). Velocity is in
 * encoder ticks per second. The motor is the encoder motor "fly_motor_enc"; a second, follower motor
 * can be added later if rig data (speed dip and recovery over a 4-ball burst) shows one is not enough.
 */
@Config
public class Flywheel {

    /** Motor command (0 to 1) applied once the speed is at or above the setpoint. Placeholder: tune on the rig. */
    public static double FEED_FORWARD = 0.2;
    /** Filtered speed, as a fraction of the setpoint, that counts as ready. */
    public static double READY_FRACTION = 0.95;

    private static final int FILTER_SAMPLES = 5;

    private final DcMotorEx motor;
    private final double[] window = new double[FILTER_SAMPLES];
    private int windowIndex = 0;
    private double windowSum = 0.0;
    private double targetVelocity = 0.0;

    public Flywheel(HardwareMap hardwareMap) {
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "fly_motor_enc");
        motor.setDirection(profile.flywheelDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0.0);
    }

    /** Ticks per second; 0 or less turns the flywheel off. */
    public void setTargetVelocity(double ticksPerSecond) {
        targetVelocity = Math.max(0.0, ticksPerSecond);
    }

    public void stop() {
        targetVelocity = 0.0;
        motor.setPower(0.0);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getFilteredVelocity() {
        return windowSum / FILTER_SAMPLES;
    }

    /** True once the filtered speed has reached READY_FRACTION of a nonzero setpoint. */
    public boolean isReady() {
        return targetVelocity > 0.0 && getFilteredVelocity() >= targetVelocity * READY_FRACTION;
    }

    /** Call once per loop. */
    public void update() {
        double velocity = motor.getVelocity();
        windowSum += velocity - window[windowIndex];
        window[windowIndex] = velocity;
        windowIndex = (windowIndex + 1) % FILTER_SAMPLES;

        if (targetVelocity <= 0.0) {
            motor.setPower(0.0);
        } else if (velocity > targetVelocity) {
            motor.setPower(Math.max(0.0, Math.min(1.0, FEED_FORWARD)));
        } else {
            motor.setPower(1.0);
        }
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Flywheel target (ticks/s)", "%.0f", targetVelocity);
        telemetry.addData("Flywheel velocity (ticks/s)", "%.0f   filtered %.0f", getVelocity(), getFilteredVelocity());
        telemetry.addData("Flywheel ready", isReady());
        telemetry.addData("Flywheel power (+/-%FS)", "%.2f", motor.getPower());
        telemetry.addData("Flywheel current (A)", "%.2f", getCurrentAmps());
    }

    /** Action: set the target and finish as soon as the flywheel is ready, or after the timeout. */
    public Action spinUp(double ticksPerSecond, double timeoutSeconds) {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    timer.reset();
                    setTargetVelocity(ticksPerSecond);
                }
                packet.put("flywheel/velocity", getFilteredVelocity());
                packet.put("flywheel/target", targetVelocity);
                return !isReady() && timer.seconds() < timeoutSeconds;
            }
        };
    }

    /** Action: set the target and finish in the same loop (does not wait for speed). */
    public Action setTarget(double ticksPerSecond) {
        return new InstantAction(() -> setTargetVelocity(ticksPerSecond));
    }

    /** Action: stop the flywheel. */
    public Action stopAction() {
        return new InstantAction(this::stop);
    }
}
