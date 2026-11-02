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
 * The intake roller: one motor, no encoder, open loop, at the front of the robot, the same end as the shooter (the robot
 * shoots out of the side it intakes). Takes pollen and nectar. Design notes: design/mechanisms.md.
 *
 * It runs ONLY while someone asks it to (never always-on), so it cannot pick up stray balls
 * and push the robot over the 4-element carry limit. Jams are the driver's job for now; the
 * current is exposed so a threshold can be chosen later if automatic jam handling is added.
 *
 * Hardware config: a motor named "intake". Its direction is per chassis (ChassisProfile).
 */
@Config
public class Intake {

    public enum Mode { OFF, IN, OUT }

    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -0.5;

    private final DcMotorEx motor;
    private Mode mode = Mode.OFF;
    private double powerScale = 1.0;

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
    }

    public double getCommandedPower() {
        switch (mode) {
            case IN: return IN_POWER * powerScale;
            case OUT: return OUT_POWER * powerScale;
            default: return 0.0;
        }
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
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
