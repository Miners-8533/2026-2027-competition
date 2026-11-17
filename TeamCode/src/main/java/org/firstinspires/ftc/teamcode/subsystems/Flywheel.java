package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;

import java.util.Locale;

/**
 * The flywheel (specs/flywheel.md sections 1 to 3). Bang-bang with a feedforward: full power at or below the target,
 * the feedforward power above it. Never braked. Speeds are encoder ticks per second.
 * A lost speed sensor switches it to open loop at a fixed power; it can recover twice per run.
 */
@Config
public final class Flywheel {
    /** Must not exceed the power that holds the slowest launch speed, or the wheel creeps above its setpoint. */
    public static double FEEDFORWARD = 0.35;
    public static double READY_FRACTION = 0.95;
    public static double BAND_LOW = 0.96, BAND_HIGH = 1.04;
    public static int FILTER_LENGTH = 5;
    public static int ENCODER_SIGN = 1;
    public static double LOST_COMMAND = 0.9, LOST_FOR_S = 1.0, LOST_SPEED = 50;
    public static double LOST_BASE_POWER = 0.40, LOST_BASE_SPEED = 1350, LOST_READY_S = 2.0;
    public static double RECOVER_FRACTION = 0.5, RECOVER_FOR_S = 1.0;
    public static int RECOVER_MAX = 2;
    public static double TOP_SPEED = 2200;
    public static double CURRENT_REFRESH_MS = 250;

    private final DcMotorEx motor;
    private double target, speed, filtered, command, sentCommand = Double.NaN, amps, lastAmpsAt = -1;
    private final double[] window = new double[16];
    private int windowN, windowAt;

    private boolean lost;
    private double lostAt, highCommandSince = -1, recoverSince = -1;
    private int recoveries;
    private String report;      // said once when it enters lost mode or recovers

    private Flywheel(DcMotorEx motor, ChassisProfile profile) {
        this.motor = motor;
        motor.setDirection(profile.flywheelReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Null when the motor is not in the hub configuration. */
    public static Flywheel create(HardwareMap hw, ChassisProfile profile) {
        DcMotorEx m = hw.tryGet(DcMotorEx.class, ChassisProfile.FLYWHEEL);
        return m == null ? null : new Flywheel(m, profile);
    }

    /** Target speed in ticks per second; 0 or less means off. */
    public void setTarget(double ticksPerS) { target = Math.max(0, ticksPerS); }

    public void update(double t) {
        speed = ENCODER_SIGN * motor.getVelocity();
        int len = Math.max(1, Math.min(window.length, FILTER_LENGTH));
        window[windowAt % len] = speed;
        windowAt++;
        windowN = Math.min(len, windowN + 1);
        double sum = 0;
        for (int i = 0; i < windowN; i++) sum += window[i];
        filtered = sum / windowN;

        if (!lost) {
            command = target <= 0 ? 0 : (speed <= target ? 1.0 : FEEDFORWARD);
            // Lost: commanded hard for a while and the speed never showed. Any real reading restarts the window.
            if (command >= LOST_COMMAND && speed < LOST_SPEED) {
                if (highCommandSince < 0) highCommandSince = t;
                if (t - highCommandSince >= LOST_FOR_S) {
                    lost = true;
                    lostAt = t;
                    recoverSince = -1;
                    report = String.format(Locale.US, "FLYWHEEL ENCODER LOST: command %.2f for %.1f s and the speed reads %.0f ticks/s; open loop at a fixed power",
                            command, t - highCommandSince, speed);
                }
            } else {
                highCommandSince = -1;
            }
        }
        if (lost) {
            command = target <= 0 ? 0 : Math.max(0, Math.min(1, LOST_BASE_POWER + (target - LOST_BASE_SPEED) / TOP_SPEED));
            // A hub that stopped answering for a moment looks the same as a dead encoder: keep reading the speed.
            if (recoveries < RECOVER_MAX && target > 0 && speed >= RECOVER_FRACTION * target) {
                if (recoverSince < 0) recoverSince = t;
                if (t - recoverSince >= RECOVER_FOR_S) {
                    lost = false;
                    recoveries++;
                    highCommandSince = -1;
                    report = String.format(Locale.US, "FLYWHEEL encoder back: %.0f ticks/s for %.1f s; closed loop again (recovery %d of %d)",
                            speed, t - recoverSince, recoveries, RECOVER_MAX);
                }
            } else {
                recoverSince = -1;
            }
        }
        if (command != sentCommand) {
            motor.setPower(command);
            sentCommand = command;
        }
        if ((t - lastAmpsAt) * 1000 >= CURRENT_REFRESH_MS) {
            lastAmpsAt = t;
            try {
                amps = motor.getCurrent(CurrentUnit.AMPS);
            } catch (Throwable ignored) { }
        }
        this.now = t;
    }

    private double now;

    public double target() { return target; }
    public double speed() { return speed; }
    /** The mean of the last readings. Readiness and the launch band use it. */
    public double filtered() { return filtered; }
    public double command() { return command; }
    public double amps() { return amps; }
    public boolean lost() { return lost; }

    public boolean ready() {
        if (target <= 0) return false;
        if (lost) return now - lostAt >= LOST_READY_S;
        return filtered >= READY_FRACTION * target;
    }

    /** The upper limit matters: a wheel still coasting down from a higher setpoint launches long. */
    public boolean inBand() {
        if (target <= 0) return false;
        if (lost) return ready();
        return filtered >= BAND_LOW * target && filtered <= BAND_HIGH * target;
    }

    /** The speed the wheel is driven toward in lost mode, whatever the distance. */
    public static double lostBaseSpeed() { return LOST_BASE_SPEED; }

    /** A line to log once, when it entered lost mode or recovered; null otherwise. */
    public String takeReport() {
        String r = report;
        report = null;
        return r;
    }

    /** For the end of an opmode: power off at once; the wheel coasts. */
    public void stopNow() {
        target = 0;
        command = 0;
        motor.setPower(0);
        sentCommand = 0;
    }
}
