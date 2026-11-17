package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The servo that moves the FLYWHEEL between two positions, one for launching POLLEN and one for launching NECTAR. On this branch (integration, 2026-11-16, user) it only HOLDS THE POLLEN position: the launch map
 * was tuned there, a nectar jams at the pollen position and a pollen misses from the nectar one, so nothing here switches it. The switch between the two is the prototype on the branch flywheel-servo-prototype
 * (with its design notes, design/flywheel-position.md, and the Flywheel Position Test that measured the positions; neither is on this branch).
 *
 * Deliberately plain, like the {@link Ramp}: a position servo set straight to one of two positions (0 to 1), no slew, no feedback. The positions are NaN until they are measured (Flywheel Position Test),
 * and until both are set nothing moves (set on 2026-11-15: pollen 0.45, nectar 0.30). With no position feedback "settled" means the travel time of the last command (250 ms to nectar, 100 ms to pollen) since the command, so a launch waits for it (Mechanisms holds a launch while the servo is moving).
 * This class commands nothing when it is built (the servo sags like the ramp's when an opmode ends, and its position is unknown until the first command); the first {@link #set} commands it. Who calls it
 * (Mechanisms): fromHardware commands POLLEN for the autos and the tuners when they are built (INIT), the competition TeleOp commands it at Start (holdFlywheelPollenAtStart; nothing moves in its INIT), and
 * update() commands it on the first update if nothing has.
 * The hardware configuration needs a servo named SERVO_NAME (the channel is not decided yet: add it in the Robot Configuration under that name, or change SERVO_NAME in the Dashboard before INIT).
 */
@Config
public class FlywheelPosition {

    /** The launched element decides the position. */
    public enum Element { POLLEN, NECTAR }

    /** The Robot Configuration name of the servo. Read when this is built (INIT). */
    public static String SERVO_NAME = "fly_pos";
    /** Servo direction: true reverses it (swaps which end the positions are measured from). Read when this is built. */
    public static boolean REVERSED = false;   // the positions below were found with the servo running FORWARD (Flywheel Position Test, 2026-11-15 19:58)
    /** The two positions (0 to 1). NaN: not measured yet, nothing moves. Measured with the Flywheel Position Test on 2026-11-15 (the servo forward): pollen 0.45, nectar 0.30. */
    public static double POLLEN_POSITION = 0.45;
    public static double NECTAR_POSITION = 0.30;
    /**
     * Seconds after a command until the flywheel counts as in place (no servo feedback; timed by the user on 2026-11-15): 250 ms from pollen to nectar, 100 ms from nectar to pollen. The time depends on the direction of the
     * move, so a launch waits for the one it needs; the first command (the position was unknown) waits for the longer. Add a margin here if a launch still goes before the flywheel is in place.
     */
    public static double TRAVEL_TO_NECTAR_SEC = 0.25;
    public static double TRAVEL_TO_POLLEN_SEC = 0.10;

    private final Servo servo;
    private final ElapsedTime sinceCommand = new ElapsedTime();
    private Element target = null;      // null until the first command
    private double lastPosition = Double.NaN;
    private double travelSec = 0.0;      // the travel time of the last command

    private FlywheelPosition(Servo servo) {
        this.servo = servo;
        servo.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        sinceCommand.reset();
    }

    /** The servo named SERVO_NAME, or null when the configuration has none (the robot runs without it). */
    public static FlywheelPosition tryCreate(HardwareMap hardwareMap) {
        Servo s = hardwareMap.tryGet(Servo.class, SERVO_NAME);
        return s == null ? null : new FlywheelPosition(s);
    }

    /** True once both positions are known. Nothing moves before that. */
    public boolean isConfigured() {
        return !(Double.isNaN(POLLEN_POSITION) || Double.isNaN(NECTAR_POSITION));
    }

    /** Moves the flywheel to the position for this element. Ignored until both positions are measured. */
    public void set(Element element) {
        if (!isConfigured()) return;
        double pos = Math.max(0.0, Math.min(1.0, element == Element.POLLEN ? POLLEN_POSITION : NECTAR_POSITION));
        if (element == target && pos == lastPosition) return;
        servo.setPosition(pos);
        travelSec = target == null ? Math.max(TRAVEL_TO_NECTAR_SEC, TRAVEL_TO_POLLEN_SEC) : element == Element.NECTAR ? TRAVEL_TO_NECTAR_SEC : TRAVEL_TO_POLLEN_SEC;   // the first command: the position was unknown
        target = element;
        lastPosition = pos;
        sinceCommand.reset();
        RunLog.event(String.format(Locale.US, "FLYWHEEL POSITION -> %s (servo %s at %.3f)", element, SERVO_NAME, pos));
    }

    /** The element the flywheel was last sent to, or null before the first command. */
    public Element target() {
        return target;
    }

    /** The travel time of the last command has passed (true before any command: nothing is moving). */
    public boolean isSettled() {
        return target == null || sinceCommand.seconds() >= travelSec;
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Flywheel position", !isConfigured() ? "NOT CONFIGURED: positions not measured, it will not move"
                : target == null ? "position UNKNOWN (not commanded yet)"
                : target + " (servo " + SERVO_NAME + " at " + String.format(Locale.US, "%.3f", lastPosition) + ")" + (isSettled() ? " (settled)" : " (moving)"));
    }
}
