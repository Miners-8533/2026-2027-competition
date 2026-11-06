package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.InstantAction;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.motion.SleepAction;

/**
 * The intake roller: one motor, no encoder, open loop, at the front of the robot, the same end as the launcher (the robot
 * launches out of the side it intakes). Takes pollen and nectar. Design notes: design/mechanisms.md.
 *
 * It runs ONLY while someone asks it to (never always-on), so it cannot pick up stray balls
 * and push the robot over the 4-element carry limit. Jams are the driver's job for now. Since 2026-11-05 (the intake failed mechanically in the 23:07 Solo run and the
 * log had nothing to show why) its motor current is read every CURRENT_REFRESH_MS while it runs, logged as the sample rows' intake_amps column, and a long stretch of high or
 * low current while it is commanded is written to the run log (INTAKE HIGH CURRENT / INTAKE LOW CURRENT, with the end of the stretch and its peak).
 *
 * Hardware config: a motor named "intake". Its direction is per chassis (ChassisProfile).
 */
@Config
public class Intake {

    public enum Mode { OFF, IN, OUT }

    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -0.5;
    /** The motor current is read this often (ms) while the intake runs (one hub read each; the log rows are 100 ms apart). */
    public static double CURRENT_REFRESH_MS = 100.0;
    /** More than HIGH_AMPS for HIGH_SEC while the intake is commanded: a jam or a stall (the free-running current is about 1.0 A in and 1.5 A out, Pit Check 2026-11-05). A guess to refine from the logs. */
    public static double HIGH_AMPS = 6.0, HIGH_SEC = 0.5;
    /** Less than LOW_AMPS for LOW_SEC while it is commanded IN: the roller is not turning or not coupled (a broken belt or a slipping coupler). A guess to refine from the logs. */
    public static double LOW_AMPS = 0.4, LOW_SEC = 1.0;

    private final DcMotorEx motor;
    private Mode mode = Mode.OFF;
    private double powerScale = 1.0;
    // the current watch
    private double currentA = 0.0;
    private long currentAtNs = 0L;
    private long highSinceNs = 0L, lowSinceNs = 0L;
    private boolean highActive = false, lowActive = false;
    private double highPeakA = 0.0, lowPeakA = 0.0;
    private String note = null;

    public Intake(HardwareMap hardwareMap) {
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(profile.intakeDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
    }

    /** Takes effect immediately. */
    public void setMode(Mode mode) {
        this.mode = mode;
        apply();
    }

    public Mode getMode() {
        return mode;
    }

    /** Scales both powers, 0 to 1. For gentle bring-up and precision use; 1.0 is full tuned power. */
    public void setPowerScale(double scale) {
        powerScale = Math.max(0.0, Math.min(1.0, scale));
        apply();
    }

    /** Call once per loop, like the other subsystems. Currently just reapplies the power. */
    public void update() {
        apply();
        refreshCurrentAndWatch();
    }

    public double getCommandedPower() {
        switch (mode) {
            case IN: return IN_POWER * powerScale;
            case OUT: return OUT_POWER * powerScale;
            default: return 0.0;
        }
    }

    /** A fresh read of the motor current (one hub read per call; for telemetry and bring-up). The run log uses {@link #getCachedCurrentAmps()}. */
    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /** The motor current as of the last refresh in {@link #update()} (0 while the intake is off); no hub read. */
    public double getCachedCurrentAmps() {
        return currentA;
    }

    /** What the current watch has to say (a high or low stretch began or ended), or null; returns it once. For the run log. */
    public String takeNote() {
        String n = note;
        note = null;
        return n;
    }

    private void say(String text) {
        note = note == null ? text : note + "; " + text;
    }

    private void refreshCurrentAndWatch() {
        long now = System.nanoTime();
        if (mode == Mode.OFF) {
            currentA = 0.0;   // no read while it is off
            currentAtNs = now;
            endStretches(now);
            return;
        }
        if (now - currentAtNs < CURRENT_REFRESH_MS * 1e6) return;
        currentA = motor.getCurrent(CurrentUnit.AMPS);
        currentAtNs = now;
        if (currentA >= HIGH_AMPS) {
            if (highSinceNs == 0L) {
                highSinceNs = now;
                highPeakA = 0.0;
            }
            highPeakA = Math.max(highPeakA, currentA);
            if (!highActive && (now - highSinceNs) / 1e9 >= HIGH_SEC) {
                highActive = true;
                say(String.format(java.util.Locale.US, "INTAKE HIGH CURRENT: %.1f A for %.1f s while %s (a jam or a stall? normal is about 1.0 A in and 1.5 A out)", currentA, (now - highSinceNs) / 1e9, mode));
            }
        } else {
            if (highActive) say(String.format(java.util.Locale.US, "INTAKE current back under %.0f A after %.1f s of high current, peak %.1f A", HIGH_AMPS, (now - highSinceNs) / 1e9, highPeakA));
            highSinceNs = 0L;
            highActive = false;
        }
        if (mode == Mode.IN && currentA < LOW_AMPS) {
            if (lowSinceNs == 0L) {
                lowSinceNs = now;
                lowPeakA = 0.0;
            }
            lowPeakA = Math.max(lowPeakA, currentA);
            if (!lowActive && (now - lowSinceNs) / 1e9 >= LOW_SEC) {
                lowActive = true;
                say(String.format(java.util.Locale.US, "INTAKE LOW CURRENT: %.2f A for %.1f s while commanded IN (the roller is not turning or not coupled: a broken belt or a slipping coupler?)", currentA, (now - lowSinceNs) / 1e9));
            }
        } else {
            if (lowActive) say(String.format(java.util.Locale.US, "INTAKE current back over %.1f A after %.1f s of low current", LOW_AMPS, (now - lowSinceNs) / 1e9));
            lowSinceNs = 0L;
            lowActive = false;
        }
    }

    private void endStretches(long now) {
        if (highActive) say(String.format(java.util.Locale.US, "INTAKE stopped after %.1f s of high current, peak %.1f A", (now - highSinceNs) / 1e9, highPeakA));
        if (lowActive) say(String.format(java.util.Locale.US, "INTAKE stopped after %.1f s of low current", (now - lowSinceNs) / 1e9));
        highSinceNs = lowSinceNs = 0L;
        highActive = lowActive = false;
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Intake mode", mode);
        telemetry.addData("Intake power (+/-%FS)", "%.2f", getCommandedPower());
        telemetry.addData("Intake current (A)", "%.2f", getCurrentAmps());
    }

    /** Action: set the mode and finish in the same loop. */
    public Action set(Mode mode) {
        return new InstantAction(() -> setMode(mode));
    }

    /** Action: run in this mode for the given seconds, then turn off. */
    public Action pulse(Mode mode, double seconds) {
        return new SequentialAction(set(mode), new SleepAction(seconds), set(Mode.OFF));
    }

    private void apply() {
        motor.setPower(getCommandedPower());
    }
}
