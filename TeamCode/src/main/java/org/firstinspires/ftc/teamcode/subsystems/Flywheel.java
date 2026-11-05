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
 * The launcher flywheel: one encoder motor "fly_motor_enc", hood fixed. (The optional second motor "fly_follow"
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
 * ENCODER_SIGN makes the speed positive when the wheel turns the launching way. A dead or reversed encoder is a
 * wiring problem for the pit check to find. If the speed sensor DIES in a match this class notices (see
 * {@link #isEncoderLost}): the motor commanded at 0.9 or more for LOST_DETECT_SEC while the speed never reaches
 * LOST_SPEED_MAX. It then latches, for the rest of the run, into an open-loop mode at a fixed power for
 * LOST_BASE_SETPOINT ticks/s (LOST_BASE_POWER, a plain power with no battery compensation: the voltage is too noisy and sags with
 * the other motors) that the speed trim can adjust.
 * {@link #setOpenLoop} is the older manual open-loop switch (power = target / OPEN_LOOP_TOP_SPEED).
 *
 * Call {@link #update()} once per loop (autos pass it to Actions.runBlocking). Velocity is in encoder ticks
 * per second.
 */
@Config
public class Flywheel {

    /**
     * Motor command (0 to 1) applied once the speed is at or above the setpoint. It must not exceed the power that HOLDS the slowest launch-map speed
     * (1325 ticks/s): the 2026-11-02 logs put that at about 0.38 on average, 0.37 on a battery above 12.3 V, and 0.29 to 0.35 for some warm stretches, so
     * 0.35 (the user's choice, 2026-11-03; 0.2 before) is at the edge: if the speed creeps above the setpoint at the slowest entry on a fresh battery, lower it.
     */
    public static double FEED_FORWARD = 0.35;
    /**
     * The speed control: 0 = the bang-bang above (full power below the setpoint, FEED_FORWARD at or above it; the default and what every match so far used), 1 = FEED-FORWARD PLUS P
     * (developed 2026-11-04, NOT yet run on the robot, switch it in from the Dashboard to try it): full power until the filtered speed reaches FFP_SPINUP_BELOW_FRACTION of the setpoint, then
     * power = FFP_BASE_POWER + FFP_POWER_PER_TICK x (setpoint - FFP_BASE_SPEED) + FFP_KP x error + FFP_KI x the error's integral, clamped to 0..1 (so above the setpoint it coasts at
     * 0 instead of holding 0.35, which also brings the speed down faster after the setpoint drops). The feed-forward is the average power of the bang-bang at steady speeds (about 0.44 at
     * 1350, 2026-11-03 logs). tools/flywheel/ffp_sim.py compares the two on a plant fitted to the logs: the idle ripple falls from a standard deviation of 1.1 percent (95th percentile +3.1) to
     * about 0.3 percent (+0.5), a pollen takes the same speed away but it comes back in about 0.2 s instead of 0.05 s; kP 0.012 hunts in the simulation, so stay at 0.005 or below.
     */
    public static int CONTROL_MODE = 0;
    public static double FFP_BASE_SPEED = 1350.0, FFP_BASE_POWER = 0.44, FFP_POWER_PER_TICK = 4.3e-4;
    public static double FFP_KP = 0.005, FFP_KI = 0.002, FFP_MAX_INTEGRAL_POWER = 0.15;
    public static double FFP_SPINUP_BELOW_FRACTION = 0.92;
    /**
     * The integral is cleared only when the setpoint steps by more than this fraction (or to or from 0). The setpoint follows the distance every loop, so clearing it on
     * every change of more than 1 tick/s (as before) cleared it on nearly every loop while the robot moved, and the I term never built up.
     */
    public static double FFP_INTEGRAL_RESET_FRACTION = 0.05;
    /** True: the P term uses the 5-loop average speed (steadier, a little late); false: the raw hub speed. */
    public static boolean FFP_USE_FILTERED = true;
    /**
     * LAUNCH BAND (2026-11-04): a salvo may only START while the filtered speed is between these fractions of the setpoint (checked once, at the start; a salvo that has started
     * goes on whatever the speed does). 0.96 / 1.04 from the 268 salvos of 11-02 and 11-03: the speed at the start was within about -2 to +4 percent in 96 percent of them,
     * the other 12 were -5 to -7 (a first salvo after the spin-up) and +5 to +16 (after the setpoint dropped when the driver moved in: the wheel only coasts down, and READY_FRACTION has no upper limit).
     */
    public static double LAUNCH_BAND_LOW_FRACTION = 0.96;
    public static double LAUNCH_BAND_HIGH_FRACTION = 1.04;
    /** Filtered speed, as a fraction of the setpoint, that counts as ready. */
    public static double READY_FRACTION = 0.95;
    /** +1 if the SDK encoder count rises when the wheel turns the launching way, -1 if it falls. */
    public static double ENCODER_SIGN = 1.0;
    /** Open-loop override: power = target / this (about the speed at full power, measured on the bench). */
    public static double OPEN_LOOP_TOP_SPEED = 2200.0;
    /** Open-loop override: seconds after the flywheel is turned on until it counts as ready (there is no speed to check). */
    public static double OPEN_LOOP_READY_SEC = 1.5;
    /** Encoder-lost detection: on or off. */
    public static boolean LOST_DETECT_ENABLED = true;
    /** The encoder counts as lost when the motor has been commanded at LOST_POWER_MIN or more for this long with the speed always under LOST_SPEED_MAX. */
    public static double LOST_DETECT_SEC = 1.0;
    public static double LOST_POWER_MIN = 0.9;
    public static double LOST_SPEED_MAX = 50.0;
    /** Encoder-lost mode: the speed (ticks/s) the base power is for, and the base power itself. 0.40 is the average fly_power of the bang-bang at a steady 1350 (14,709 log rows, 12 TeleOp runs of 2026-11-02, speed within 1.5 % of the setpoint, no salvo): 0.38 to 0.42 per run, 0.41 below 12 V and 0.375 above, drifting down as the wheel warms. The old placeholder 0.61 (1350 / 2200) would have spun about 50 % too fast. */
    public static double LOST_BASE_SETPOINT = 1350.0;
    public static double LOST_BASE_POWER = 0.40;
    /** Encoder-lost mode: seconds after the switch until the flywheel counts as ready (it must slow from full power to the base power first). */
    public static double LOST_READY_SEC = 2.0;
    /** How often (ms) the motor currents are refreshed for the display. */
    public static double CURRENT_REFRESH_MS = 250.0;

    private static final int FILTER_SAMPLES = 5;

    private final DcMotorEx motor;
    // The optional second motor "fly_follow" is commented out for now (removed from the robot); the lines below
    // marked "fly_follow" put it back.
    // private final DcMotorEx follower;   // null when there is no "fly_follow" motor
    private final double[] window = new double[FILTER_SAMPLES];
    private final ElapsedTime openLoopTimer = new ElapsedTime();
    private int windowIndex = 0;
    private double windowSum = 0.0;
    private double targetVelocity = 0.0;
    private double ffpIntegral = 0.0;
    private final ElapsedTime ffpTimer = new ElapsedTime();
    private boolean openLoop = false;
    private boolean openLoopSpinning = false;
    private boolean encoderLost = false;
    private String lostReason = "";
    private boolean lostWindowActive = false;
    private final ElapsedTime lostTimer = new ElapsedTime();

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
        double next = Math.max(0.0, ticksPerSecond);
        boolean onOff = (next <= 0.0) != (targetVelocity <= 0.0);
        if (onOff || Math.abs(next - targetVelocity) > FFP_INTEGRAL_RESET_FRACTION * Math.max(next, targetVelocity)) ffpIntegral = 0.0;
        targetVelocity = next;
    }

    public void stop() {
        targetVelocity = 0.0;
        setPower(0.0, true);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Speed in ticks per second, positive when the wheel turns the launching way (after ENCODER_SIGN). */
    public double getVelocity() {
        return velocity;
    }

    public double getFilteredVelocity() {
        return windowSum / FILTER_SAMPLES;
    }

    /** True once the filtered speed has reached READY_FRACTION of a nonzero setpoint. */
    public boolean isReady() {
        if (openLoop) {
            return targetVelocity > 0.0 && openLoopSpinning
                    && openLoopTimer.seconds() >= (encoderLost ? LOST_READY_SEC : OPEN_LOOP_READY_SEC);
        }
        return targetVelocity > 0.0 && getFilteredVelocity() >= targetVelocity * READY_FRACTION;
    }

    /**
     * True when a salvo may start: the filtered speed is inside the launch band (LAUNCH_BAND_LOW_FRACTION to LAUNCH_BAND_HIGH_FRACTION of the setpoint). With the speed sensor
     * overridden or lost there is no speed to check, so it is the same as {@link #isReady}.
     */
    public boolean isInLaunchBand() {
        if (openLoop) return isReady();
        double v = getFilteredVelocity();
        return targetVelocity > 0.0 && v >= targetVelocity * LAUNCH_BAND_LOW_FRACTION && v <= targetVelocity * LAUNCH_BAND_HIGH_FRACTION;
    }

    /** "in the band", "BELOW the band" or "ABOVE the band", for the log and the notices. */
    public String launchBandState() {
        if (openLoop || targetVelocity <= 0.0) return isReady() ? "in the band" : "BELOW the band";
        double v = getFilteredVelocity();
        return v < targetVelocity * LAUNCH_BAND_LOW_FRACTION ? "BELOW the band" : v > targetVelocity * LAUNCH_BAND_HIGH_FRACTION ? "ABOVE the band" : "in the band";
    }

    /**
     * The override for a failed speed sensor: run at a power set from the target (target / OPEN_LOOP_TOP_SPEED) and treat the
     * flywheel as ready OPEN_LOOP_READY_SEC after it is turned on, ignoring the measured speed.
     */
    public void setOpenLoop(boolean on) {
        if (on != openLoop) {
            openLoop = on;
            openLoopSpinning = false;
        }
    }

    public boolean isOpenLoop() {
        return openLoop;
    }

    /** True once the speed sensor has been judged dead (latched for the rest of the run): the flywheel then runs open loop at a fixed power. */
    public boolean isEncoderLost() {
        return encoderLost;
    }

    public String getLostReason() {
        return lostReason;
    }

    /** The power for the lost-encoder mode: the base power, moved by the difference from the base speed. No battery compensation. */
    private double lostPower() {
        double p = LOST_BASE_POWER + (targetVelocity - LOST_BASE_SETPOINT) / OPEN_LOOP_TOP_SPEED;
        return p;
    }

    /** Judges the speed sensor dead: commanded hard for LOST_DETECT_SEC with the speed never reaching LOST_SPEED_MAX (an unplugged or reversed encoder). */
    private void detectEncoderLost() {
        boolean commanded = targetVelocity > 0.0 && lastPower >= LOST_POWER_MIN;
        if (!commanded || velocity >= LOST_SPEED_MAX) {
            lostWindowActive = false;
            return;
        }
        if (!lostWindowActive) {
            lostWindowActive = true;
            lostTimer.reset();
        } else if (lostTimer.seconds() >= LOST_DETECT_SEC) {
            encoderLost = true;
            openLoop = true;
            openLoopSpinning = false;
            lostReason = String.format(java.util.Locale.US, "commanded %.2f for %.1f s with the speed under %.0f ticks/s (now %.0f)",
                    lastPower, lostTimer.seconds(), LOST_SPEED_MAX, velocity);
        }
    }

    /** Call once per loop. Reads the encoder once, up front; getVelocity() and the filter use that reading. */
    public void update() {
        velocity = ENCODER_SIGN * motor.getVelocity();
        if (LOST_DETECT_ENABLED && !encoderLost && !openLoop) detectEncoderLost();
        if (currentTimer.milliseconds() > CURRENT_REFRESH_MS) {
            currentA = motor.getCurrent(CurrentUnit.AMPS);
            // followerCurrentA = follower != null ? follower.getCurrent(CurrentUnit.AMPS) : 0.0;   // fly_follow
            currentTimer.reset();
        }
        windowSum += velocity - window[windowIndex];
        window[windowIndex] = velocity;
        windowIndex = (windowIndex + 1) % FILTER_SAMPLES;

        if (targetVelocity <= 0.0) {
            openLoopSpinning = false;
        } else if (openLoop && !openLoopSpinning) {
            openLoopSpinning = true;
            openLoopTimer.reset();
        }

        if (targetVelocity <= 0.0) {
            setPower(0.0);
        } else if (openLoop) {
            setPower(Math.max(0.0, Math.min(1.0, encoderLost ? lostPower() : targetVelocity / OPEN_LOOP_TOP_SPEED)));
        } else if (CONTROL_MODE == 1) {
            setPower(feedForwardPlusP());
        } else if (velocity > targetVelocity) {
            setPower(Math.max(0.0, Math.min(1.0, FEED_FORWARD)));
        } else {
            setPower(1.0);
        }
    }

    /** CONTROL_MODE 1: full power while spinning up, then the feed-forward for the setpoint plus P and a little I on the speed error (see CONTROL_MODE). */
    private double feedForwardPlusP() {
        double dt = Math.min(0.1, ffpTimer.seconds());
        ffpTimer.reset();
        double v = FFP_USE_FILTERED ? getFilteredVelocity() : velocity;
        if (v < FFP_SPINUP_BELOW_FRACTION * targetVelocity) {
            ffpIntegral = 0.0;
            return 1.0;
        }
        double ff = FFP_BASE_POWER + FFP_POWER_PER_TICK * (targetVelocity - FFP_BASE_SPEED);
        double error = targetVelocity - v;
        double limit = FFP_KI > 1e-9 ? FFP_MAX_INTEGRAL_POWER / FFP_KI : 0.0;
        ffpIntegral = Math.max(-limit, Math.min(limit, ffpIntegral + error * dt));
        return Math.max(0.0, Math.min(1.0, ff + FFP_KP * error + FFP_KI * ffpIntegral));
    }

    /** The motor command last written, 0 to 1 (the bang-bang switches it between 1.0 and FEED_FORWARD, so average it over time). */
    public double getLastPower() {
        return lastPower;
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
        if (encoderLost) {
            telemetry.addLine("FLYWHEEL ENCODER LOST: open loop at a fixed power (" + lostReason + ")");
        } else if (openLoop) {
            telemetry.addLine("FLYWHEEL OPEN LOOP (override): the speed sensor is ignored");
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
