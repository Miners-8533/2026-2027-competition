package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * The intake (specs/intake-and-ramp.md section 1). It only does what it is told: a mode, OFF, IN or OUT.
 * Its current is watched and reported to the log; nothing acts on the reports.
 */
@Config
public final class Intake {
    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -0.5;
    public static double CURRENT_REFRESH_MS = 100;
    public static double HIGH_AMPS = 6.0, HIGH_FOR_S = 0.5;
    public static double LOW_AMPS = 0.4, LOW_FOR_S = 1.0;

    public enum Mode { OFF, IN, OUT }

    private final DcMotorEx motor;
    private Mode mode = Mode.OFF;
    private double scale = 1.0;
    private double sentPower = Double.NaN;
    private double amps, lastAmpsAt = -1;

    // Current watch: one stretch of each kind at a time.
    private double highSince = -1, lowSince = -1, highPeak;
    private boolean highOpen, lowOpen;

    private Intake(DcMotorEx motor, ChassisProfile profile) {
        this.motor = motor;
        motor.setDirection(profile.intakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Null when the motor is not in the hub configuration. */
    public static Intake create(HardwareMap hw, ChassisProfile profile) {
        DcMotorEx m = hw.tryGet(DcMotorEx.class, ChassisProfile.INTAKE);
        return m == null ? null : new Intake(m, profile);
    }

    public void set(Mode m) { mode = m; }
    public Mode mode() { return mode; }
    public void setScale(double s) { scale = Math.max(0, Math.min(1, s)); }
    public double amps() { return amps; }

    public void update(double t) {
        double p = (mode == Mode.IN ? IN_POWER : mode == Mode.OUT ? OUT_POWER : 0) * scale;
        if (p != sentPower) {
            motor.setPower(p);
            sentPower = p;
        }
        if (mode == Mode.OFF) {
            // While it is off nothing is read and the value is 0.
            amps = 0;
            closeStretches(t, "the intake was switched off");
            return;
        }
        if ((t - lastAmpsAt) * 1000 < CURRENT_REFRESH_MS) return;
        lastAmpsAt = t;
        try {
            amps = motor.getCurrent(CurrentUnit.AMPS);
        } catch (Throwable e) {
            return;
        }
        // High current: a possible jam or stall.
        if (amps >= HIGH_AMPS) {
            if (highSince < 0) {
                highSince = t;
                highPeak = amps;
            }
            highPeak = Math.max(highPeak, amps);
            if (!highOpen && t - highSince >= HIGH_FOR_S) {
                highOpen = true;
                RunLog.eventf("INTAKE high current %.1f A for %.1f s while %s: possible jam or stall", amps, t - highSince, mode);
            }
        } else if (highSince >= 0) {
            if (highOpen) RunLog.eventf("INTAKE high current ended after %.1f s, peak %.1f A", t - highSince, highPeak);
            highSince = -1;
            highOpen = false;
        }
        // Low current while IN: a roller that is not turning or not coupled.
        if (mode == Mode.IN && amps < LOW_AMPS) {
            if (lowSince < 0) lowSince = t;
            if (!lowOpen && t - lowSince >= LOW_FOR_S) {
                lowOpen = true;
                RunLog.eventf("INTAKE low current %.2f A for %.1f s while IN: roller not turning or not coupled?", amps, t - lowSince);
            }
        } else if (lowSince >= 0) {
            if (lowOpen) RunLog.eventf("INTAKE low current ended after %.1f s", t - lowSince);
            lowSince = -1;
            lowOpen = false;
        }
    }

    private void closeStretches(double t, String why) {
        if (highOpen) RunLog.eventf("INTAKE high current ended after %.1f s, peak %.1f A: %s", t - highSince, highPeak, why);
        if (lowOpen) RunLog.eventf("INTAKE low current ended after %.1f s: %s", t - lowSince, why);
        highSince = lowSince = -1;
        highOpen = lowOpen = false;
    }

    /** For the end of an opmode: off at once. */
    public void stopNow() {
        mode = Mode.OFF;
        motor.setPower(0);
        sentPower = 0;
    }
}
