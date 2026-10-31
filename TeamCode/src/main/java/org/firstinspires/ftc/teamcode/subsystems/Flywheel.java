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
 * The shooter flywheel: one encoder motor "fly_motor_enc", hood fixed. (The optional second motor "fly_follow"
 * is commented out for now; search for "fly_follow" to restore it.) Design notes: design/mechanisms.md.
 *
 * Control is last season's bang-bang plus feedforward: full power while below the setpoint, a single
 * FEED_FORWARD motor command (0 to 1) at or above it, nothing when the target is 0. Plain on purpose;
 * add compensation only after this has been tried. It does NOT wait for recovery between balls: the
 * pusher sets the ball spacing, and {@link #isReady()} is meant to be checked before the
 * FIRST ball of a salvo only.
 *
 * The motor runs WITHOUT the hub's closed-loop encoder modes, so a power is a plain duty cycle (a closed-loop mode
 * would turn the same number into a speed target, and runs away if the encoder counts opposite to its motor). The
 * encoder is only a sensor:
 * ENCODER_SIGN makes the speed positive when the wheel turns the shooting way, and a speed that reads
 * strongly negative while spinning stops the flywheel with a fault instead of letting it run at full power.
 *
 * Call {@link #update()} once per loop (autos pass it to Actions.runBlocking). Velocity is in encoder ticks
 * per second.
 */
@Config
public class Flywheel {

    /** Motor command (0 to 1) applied once the speed is at or above the setpoint. Placeholder: tune on the rig. */
    public static double FEED_FORWARD = 0.2;
    /** Filtered speed, as a fraction of the setpoint, that counts as ready. */
    public static double READY_FRACTION = 0.95;
    /** +1 if the SDK encoder count rises when the wheel turns the shooting way, -1 if it falls. */
    public static double ENCODER_SIGN = 1.0;
    /** A speed below minus this (ticks/s) for REVERSED_TIME_S while spinning is taken as a reversed encoder. */
    public static double REVERSED_TICKS_PER_SEC = 100.0;
    public static double REVERSED_TIME_S = 0.5;
    /** How often (ms) the motor currents are refreshed for the display. */
    public static double CURRENT_REFRESH_MS = 250.0;

    private static final int FILTER_SAMPLES = 5;

    private final DcMotorEx motor;
    // The optional second motor "fly_follow" is commented out for now (removed from the robot); the lines below
    // marked "fly_follow" put it back.
    // private final DcMotorEx follower;   // null when there is no "fly_follow" motor
    private final double[] window = new double[FILTER_SAMPLES];
    private final ElapsedTime reversedTimer = new ElapsedTime();
    private int windowIndex = 0;
    private double windowSum = 0.0;
    private double targetVelocity = 0.0;
    private boolean reversedSeen = false;
    private boolean faulted = false;

    // Hub calls are blocking round trips, so the hub is read in one place (the velocity, at the top of update(), and the
    // currents a few times a second for the display) and written only when the power changes.
    private final ElapsedTime currentTimer = new ElapsedTime();
    private double velocity = 0.0;
    private double currentA = 0.0;
    // private double followerCurrentA = 0.0;   // fly_follow
    private double lastPower = 0.0;

    public Flywheel(HardwareMap hardwareMap) {
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "fly_motor_enc");
        motor.setDirection(profile.flywheelDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);

        // fly_follow
        // follower = hardwareMap.tryGet(DcMotorEx.class, "fly_follow");
        // if (follower != null) {
        //     follower.setDirection(profile.flywheelFollowerDirection);
        //     follower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //     follower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //     follower.setPower(0.0);
        // }
    }

    // fly_follow
    // public boolean hasFollower() {
    //     return follower != null;
    // }

    /** Write the power only when it changed (each write is a blocking round trip), unless forced. */
    private void setPower(double power, boolean force) {
        if (force || power != lastPower) {
            motor.setPower(power);
            // if (follower != null) follower.setPower(power);   // fly_follow
            lastPower = power;
        }
    }

    private void setPower(double power) {
        setPower(power, false);
    }

    /** Ticks per second; 0 or less turns the flywheel off. */
    public void setTargetVelocity(double ticksPerSecond) {
        targetVelocity = Math.max(0.0, ticksPerSecond);
    }

    public void stop() {
        targetVelocity = 0.0;
        setPower(0.0, true);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Speed in ticks per second, positive when the wheel turns the shooting way (after ENCODER_SIGN). */
    public double getVelocity() {
        return velocity;
    }

    public double getFilteredVelocity() {
        return windowSum / FILTER_SAMPLES;
    }

    /** True once the filtered speed has reached READY_FRACTION of a nonzero setpoint. */
    public boolean isReady() {
        return !faulted && targetVelocity > 0.0 && getFilteredVelocity() >= targetVelocity * READY_FRACTION;
    }

    /** True if the flywheel was stopped because the encoder reads backward while spinning. */
    public boolean isFaulted() {
        return faulted;
    }

    public void clearFault() {
        faulted = false;
        reversedSeen = false;
    }

    /** Call once per loop. Reads the encoder once, up front; getVelocity() and the filter use that reading. */
    public void update() {
        velocity = ENCODER_SIGN * motor.getVelocity();
        if (currentTimer.milliseconds() > CURRENT_REFRESH_MS) {
            currentA = motor.getCurrent(CurrentUnit.AMPS);
            // followerCurrentA = follower != null ? follower.getCurrent(CurrentUnit.AMPS) : 0.0;   // fly_follow
            currentTimer.reset();
        }
        windowSum += velocity - window[windowIndex];
        window[windowIndex] = velocity;
        windowIndex = (windowIndex + 1) % FILTER_SAMPLES;

        if (targetVelocity > 0.0 && velocity < -REVERSED_TICKS_PER_SEC) {
            if (!reversedSeen) {
                reversedSeen = true;
                reversedTimer.reset();
            } else if (reversedTimer.seconds() > REVERSED_TIME_S) {
                faulted = true;
            }
        } else {
            reversedSeen = false;
        }

        if (faulted || targetVelocity <= 0.0) {
            setPower(0.0);
        } else if (velocity > targetVelocity) {
            setPower(Math.max(0.0, Math.min(1.0, FEED_FORWARD)));
        } else {
            setPower(1.0);
        }
    }

    /** Motor current as of the last refresh (a few times a second; for the display, not for control). */
    public double getCurrentAmps() {
        return currentA;
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Flywheel target (ticks/s)", "%.0f", targetVelocity);
        telemetry.addData("Flywheel velocity (ticks/s)", "%.0f   filtered %.0f", getVelocity(), getFilteredVelocity());
        telemetry.addData("Flywheel ready", isReady());
        telemetry.addData("Flywheel power (+/-%FS)", "%.2f", lastPower);
        telemetry.addData("Flywheel current (A)", "%.2f", getCurrentAmps());
        // fly_follow
        // telemetry.addData("Flywheel power (+/-%FS)", "%.2f   (%s)", lastPower, follower != null ? "2 motors" : "1 motor");
        // telemetry.addData("Flywheel current (A)", "%.2f%s", getCurrentAmps(), follower != null
        //         ? String.format("   follower %.2f", followerCurrentA) : "");
        if (faulted) {
            telemetry.addLine("FLYWHEEL FAULT: the encoder reads BACKWARD while spinning. Set Flywheel.ENCODER_SIGN = -1 (Back clears).");
        }
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
                return !isReady() && !faulted && timer.seconds() < timeoutSeconds;
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
