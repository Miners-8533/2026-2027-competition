package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.math.Pose2d;

/**
 * The ramp: two goBILDA torque servos lower a ramp so pollen from a FLOWER's bottom opening
 * rolls straight into the intake. Two positions only, STOWED and DEPLOYED. Design notes:
 * design/mechanisms.md.
 *
 * Deliberately plain, like last year's hood and stop servos: each command sets both servos straight to
 * their position (the plain Servo interface, no slew, no pulse range). Each servo has its own stowed and
 * deployed position (0 to 1) in {@link ChassisProfile}; they are NaN until measured, and until they are
 * filled in the ramp will not move (set from the Ramp Test, since deleted). With no position feedback,
 * "settled" simply means TRAVEL_SEC has passed since the last command, so TRAVEL_SEC is measured once.
 *
 * The robot starts STOWED (it must fit the 18 in starting cube), so the constructor commands the stowed
 * positions (except for the competition TeleOp, which builds it with commandStowed false and stows at Start: see the constructor).
 * The robot must drive out from under the flower before stowing: {@link #isClearOfFlower}
 * provides the check; the coordinator decides what to do about it. Deploying should switch the intake on and
 * stowing off, also the coordinator's job. Both servos must be set to the same mode (goBILDA dual-mode servos
 * can be positional, continuous or multi-turn). Hardware config: servos named "depolLeft" and "depolRight".
 */
@Config
public class Ramp {

    /** Seconds after a command until the ramp counts as in place. Measured 2026-10-31 with the Ramp Test (since deleted): 0.8 s up or down (no servo feedback, so this is only a timer). */
    public static double TRAVEL_SEC = 0.8;
    /** Robot-center distance from a flower center beyond which the ramp is considered clear. Placeholder: set from the real ramp length. */
    public static double CLEAR_DISTANCE_IN = 24.0;

    /** Flower centers {x, y}, field inches, read from the manual's figures (about +/-1 in); 3.4 in out from the wall is an estimate. */
    private static final double[][] FLOWERS = {
            {-68.6, -24.0},   // rear wall
            {-24.0, 68.6},    // Blue Wall
            {24.0, -68.6},    // Red Wall
            {68.6, 24.0},     // audience wall
    };

    private final Servo left;
    private final Servo right;
    private final double stowedLeft, stowedRight, deployedLeft, deployedRight;
    private final ElapsedTime sinceCommand = new ElapsedTime();
    private boolean deployedTarget = false;
    /** Whether a position has been commanded since this object was built. Until then the ramp is unpowered and its position is unknown. */
    private boolean commanded = false;

    /** The autos and test opmodes: the constructor commands the stowed positions (the robot starts stowed, and no match rule limits INIT). */
    public Ramp(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /**
     * With commandStowed false nothing is commanded here: the servos sag when an opmode ends, and an auto may end with the ramp stuck or deployed,
     * so the competition TeleOp must not move it in INIT (G403: no powered movement in the transition). The first {@link #stow()} or
     * {@link #deploy()} (the TeleOp stows at Start) commands it; until then {@link #isPositionKnown()} is false.
     */
    public Ramp(HardwareMap hardwareMap, boolean commandStowed) {
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        stowedLeft = profile.rampStowedLeft;
        stowedRight = profile.rampStowedRight;
        deployedLeft = profile.rampDeployedLeft;
        deployedRight = profile.rampDeployedRight;

        left = hardwareMap.get(Servo.class, "depolLeft");
        right = hardwareMap.get(Servo.class, "depolRight");

        if (commandStowed && isConfigured()) {
            left.setPosition(stowedLeft);
            right.setPosition(stowedRight);
            commanded = true;
        }
        sinceCommand.reset();
    }

    /** False until the first stow or deploy command (a Ramp built with commandStowed false): the servos are unpowered and the position is unknown. */
    public boolean isPositionKnown() {
        return commanded;
    }

    /** True once all four positions are known. Nothing moves before that. */
    public boolean isConfigured() {
        return !(Double.isNaN(stowedLeft) || Double.isNaN(stowedRight)
                || Double.isNaN(deployedLeft) || Double.isNaN(deployedRight));
    }

    public void deploy() {
        if (!isConfigured()) return;
        deployedTarget = true;
        commanded = true;
        left.setPosition(deployedLeft);
        right.setPosition(deployedRight);
        sinceCommand.reset();
    }

    public void stow() {
        if (!isConfigured()) return;
        deployedTarget = false;
        commanded = true;
        left.setPosition(stowedLeft);
        right.setPosition(stowedRight);
        sinceCommand.reset();
    }

    public boolean isDeployedTarget() {
        return deployedTarget;
    }

    /** TRAVEL_SEC has passed since the last command. */
    public boolean isSettled() {
        return isConfigured() && sinceCommand.seconds() >= TRAVEL_SEC;
    }

    /** True if the robot center is farther than CLEAR_DISTANCE_IN from every flower. */
    public static boolean isClearOfFlower(Pose2d pose) {
        for (double[] f : FLOWERS) {
            if (Math.hypot(f[0] - pose.position.x, f[1] - pose.position.y) <= CLEAR_DISTANCE_IN) return false;
        }
        return true;
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Ramp", !isConfigured() ? "NOT CONFIGURED: positions not measured, it will not move"
                : !commanded ? "position UNKNOWN (not commanded yet)"
                : (deployedTarget ? "target DEPLOYED" : "target STOWED") + (isSettled() ? " (settled)" : " (moving)"));
    }
}
