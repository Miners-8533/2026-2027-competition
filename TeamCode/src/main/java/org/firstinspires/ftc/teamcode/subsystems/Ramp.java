package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.Clock;

/**
 * The ramp (specs/intake-and-ramp.md section 2): two servos, two commands. Each sets both servos straight to their
 * positions from the chassis profile. There is no feedback, so "settled" is a timer.
 */
@Config
public final class Ramp {
    public static double TRAVEL_S = 0.8;
    /** A placeholder: the real ramp reach was never measured. */
    public static double CLEAR_OF_FLOWER_IN = 24.0;

    public enum Target { UNKNOWN, STOWED, DEPLOYED }

    private final Servo left, right;
    private final ChassisProfile profile;
    private Target target = Target.UNKNOWN;
    private double commandedAt = -100;

    private Ramp(Servo left, Servo right, ChassisProfile profile) {
        this.left = left;
        this.right = right;
        this.profile = profile;
    }

    /**
     * Null unless both servos are configured.
     *
     * @param stowAtInit true for an auto or a test opmode (the robot must start stowed); false for the competition
     *                   TeleOp, which may command nothing in INIT (rule G403): the position is unknown until the stow at Start.
     */
    public static Ramp create(HardwareMap hw, ChassisProfile profile, boolean stowAtInit) {
        Servo l = hw.tryGet(Servo.class, ChassisProfile.RAMP_LEFT), r = hw.tryGet(Servo.class, ChassisProfile.RAMP_RIGHT);
        if (l == null || r == null) return null;
        Ramp ramp = new Ramp(l, r, profile);
        if (stowAtInit) ramp.stow();
        return ramp;
    }

    /** False when a position is missing from the chassis profile: the ramp then never moves. */
    public boolean configured() { return profile.rampSet(); }

    public void deploy() {
        if (!configured()) return;
        left.setPosition(profile.rampDeployLeft);
        right.setPosition(profile.rampDeployRight);
        if (target != Target.DEPLOYED) commandedAt = Clock.now();
        target = Target.DEPLOYED;
    }

    public void stow() {
        if (!configured()) return;
        left.setPosition(profile.rampStowLeft);
        right.setPosition(profile.rampStowRight);
        if (target != Target.STOWED) commandedAt = Clock.now();
        target = Target.STOWED;
    }

    public Target target() { return target; }
    public boolean deployed() { return target == Target.DEPLOYED; }

    /** The travel time has passed since the last command. */
    public boolean settled() { return Clock.now() - commandedAt >= TRAVEL_S; }

    /** Whether the robot's origin is farther than the clear distance from all four flower centers (the field model's). */
    public static boolean clearOfFlowers(double x, double y) {
        for (Vec f : Field.flowerCenters()) {
            if (Math.hypot(x - f.x, y - f.y) < CLEAR_OF_FLOWER_IN) return false;
        }
        return true;
    }
}
