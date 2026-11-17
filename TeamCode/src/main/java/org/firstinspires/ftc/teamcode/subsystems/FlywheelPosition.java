package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.Clock;

/**
 * The flywheel position servo (specs/flywheel.md section 4). It holds the flywheel in the position the launch speeds
 * were measured at. This branch only ever commands POLLEN. Nothing is commanded when it is built: the position is
 * unknown until the first command. There is no feedback, so "in place" is a timer from the last command.
 */
@Config
public final class FlywheelPosition {
    public static double POLLEN = 0.45;
    public static double NECTAR = 0.30;
    public static double TO_POLLEN_S = 0.10;
    public static double TO_NECTAR_S = 0.25;

    private final Servo servo;
    private boolean commanded;
    private double readyAt;

    private FlywheelPosition(Servo servo) { this.servo = servo; }

    /** Null when the servo is not configured: the robot runs without it. */
    public static FlywheelPosition create(HardwareMap hw) {
        Servo s = hw.tryGet(Servo.class, ChassisProfile.FLY_POS);
        return s == null ? null : new FlywheelPosition(s);
    }

    public void commandPollen() {
        servo.setPosition(POLLEN);
        if (!commanded) {
            // The first command may come from anywhere: allow the longer travel.
            readyAt = Clock.now() + Math.max(TO_POLLEN_S, TO_NECTAR_S);
            commanded = true;
        }
    }

    public boolean commanded() { return commanded; }

    /** A launch waits for this. */
    public boolean inPlace() { return commanded && Clock.now() >= readyAt; }
}
