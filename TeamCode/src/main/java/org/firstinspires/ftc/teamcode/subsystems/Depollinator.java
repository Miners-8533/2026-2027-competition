package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Action;

/**
 * The depollinator: two goBILDA torque servos lower a ramp so pollen from a FLOWER's bottom opening
 * rolls straight into the intake. Two positions only, STOWED and DEPLOYED. Design notes:
 * design/mechanisms.md.
 *
 * Each servo has its own stowed and deployed position (0 to 1) in {@link ChassisProfile}; the pair is
 * mirrored simply by having different numbers. They are NaN until measured, and until they are filled
 * in the depollinator will not move (use the Depollinator Test opmode). Motion is rate limited to
 * MAX_RATE position units per second so the ramp lowers gently, and because there is no position
 * feedback, "done" means the commanded positions have reached their targets for SETTLE_SEC.
 *
 * The robot starts STOWED (it must fit the 18 in starting cube), so the constructor commands the stowed
 * positions. The robot must drive out from under the flower before stowing: {@link #isClearOfFlower}
 * provides the check; whether stow is blocked or only warned about is left to the caller.
 *
 * Deploying should switch the intake on and stowing off (a coordinator's job). Both servos must be set to
 * the same mode (goBILDA dual-mode servos can be positional, continuous or multi-turn). Call
 * {@link #update()} once per loop (autos pass it to Actions.runBlocking). Hardware config: servos named
 * "depolLeft" and "depolRight".
 */
@Config
public class Depollinator {

    /** Servo position units per second, applied to the slew. Placeholder: tune so the ramp lowers gently. */
    public static double MAX_RATE = 1.0;
    public static double SETTLE_SEC = 0.15;
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
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime atTargetTimer = new ElapsedTime();

    private double stowedLeft, stowedRight, deployedLeft, deployedRight;
    private double leftPosition = Double.NaN, rightPosition = Double.NaN;
    private double leftTarget = Double.NaN, rightTarget = Double.NaN;
    private boolean deployedTarget = false;

    public Depollinator(HardwareMap hardwareMap) {
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        stowedLeft = profile.depolStowedLeft;
        stowedRight = profile.depolStowedRight;
        deployedLeft = profile.depolDeployedLeft;
        deployedRight = profile.depolDeployedRight;

        left = hardwareMap.get(Servo.class, "depolLeft");
        right = hardwareMap.get(Servo.class, "depolRight");

        if (isConfigured()) {
            leftPosition = leftTarget = stowedLeft;
            rightPosition = rightTarget = stowedRight;
            left.setPosition(leftPosition);
            right.setPosition(rightPosition);
        }
        loopTimer.reset();
        atTargetTimer.reset();
    }

    /** True once all four positions are known. Nothing moves before that. */
    public boolean isConfigured() {
        return !(Double.isNaN(stowedLeft) || Double.isNaN(stowedRight)
                || Double.isNaN(deployedLeft) || Double.isNaN(deployedRight));
    }

    /** Override the positions for this session (bring-up). Does not command anything by itself. */
    public void setPositions(double stowedLeft, double stowedRight, double deployedLeft, double deployedRight) {
        this.stowedLeft = stowedLeft;
        this.stowedRight = stowedRight;
        this.deployedLeft = deployedLeft;
        this.deployedRight = deployedRight;
    }

    /** Tell the slew where the servos physically are now (bring-up), so the first move starts from there. */
    public void assumeCurrentPositions(double leftNow, double rightNow) {
        leftPosition = leftTarget = leftNow;
        rightPosition = rightTarget = rightNow;
    }

    /** Bring-up: command one servo straight to a position, bypassing the slew and the stored positions. */
    public void jog(boolean isLeft, double position) {
        double clamped = Math.max(0.0, Math.min(1.0, position));
        if (isLeft) {
            leftPosition = leftTarget = clamped;
            left.setPosition(clamped);
        } else {
            rightPosition = rightTarget = clamped;
            right.setPosition(clamped);
        }
    }

    public double getCommandedPosition(boolean isLeft) {
        return isLeft ? leftPosition : rightPosition;
    }

    public void deploy() {
        if (!isConfigured()) return;
        deployedTarget = true;
        leftTarget = deployedLeft;
        rightTarget = deployedRight;
    }

    public void stow() {
        if (!isConfigured()) return;
        deployedTarget = false;
        leftTarget = stowedLeft;
        rightTarget = stowedRight;
    }

    public boolean isDeployedTarget() {
        return deployedTarget;
    }

    private boolean atTarget() {
        return isConfigured() && !Double.isNaN(leftPosition) && !Double.isNaN(rightPosition)
                && Math.abs(leftPosition - leftTarget) < 1e-6 && Math.abs(rightPosition - rightTarget) < 1e-6;
    }

    /** Both servos have reached their targets and stayed there for SETTLE_SEC. */
    public boolean isSettled() {
        return atTarget() && atTargetTimer.seconds() >= SETTLE_SEC;
    }

    /** Seconds a full stowed-to-deployed move takes at MAX_RATE. */
    public double estimatedTravelSec() {
        if (!isConfigured() || MAX_RATE <= 0.0) return Double.NaN;
        return Math.max(Math.abs(deployedLeft - stowedLeft), Math.abs(deployedRight - stowedRight)) / MAX_RATE;
    }

    /** Call once per loop: slews each servo toward its target at MAX_RATE. */
    public void update() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();
        if (!isConfigured() || Double.isNaN(leftPosition) || Double.isNaN(rightPosition)) return;

        double step = MAX_RATE * dt;
        double newLeft = move(leftPosition, leftTarget, step);
        double newRight = move(rightPosition, rightTarget, step);
        if (newLeft != leftPosition) left.setPosition(newLeft);
        if (newRight != rightPosition) right.setPosition(newRight);
        leftPosition = newLeft;
        rightPosition = newRight;
        if (!atTarget()) atTargetTimer.reset();
    }

    private static double move(double current, double target, double step) {
        double delta = target - current;
        if (Math.abs(delta) <= step) return target;
        return current + Math.copySign(step, delta);
    }

    /** True if the robot center is farther than CLEAR_DISTANCE_IN from every flower. */
    public static boolean isClearOfFlower(Pose2d pose) {
        for (double[] f : FLOWERS) {
            if (Math.hypot(f[0] - pose.position.x, f[1] - pose.position.y) <= CLEAR_DISTANCE_IN) return false;
        }
        return true;
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Depollinator", !isConfigured() ? "NOT CONFIGURED: positions not measured, it will not move"
                : (deployedTarget ? "target DEPLOYED" : "target STOWED") + (isSettled() ? " (settled)" : " (moving)"));
        telemetry.addData("Servo positions L / R", "%.3f / %.3f", leftPosition, rightPosition);
        telemetry.addData("Full travel at MAX_RATE (s)", "%.2f", estimatedTravelSec());
    }

    /** Action: deploy and finish once the servos have settled. Finishes at once if not configured. */
    public Action deployAction() {
        return transition(true);
    }

    /** Action: stow and finish once the servos have settled. Finishes at once if not configured. */
    public Action stowAction() {
        return transition(false);
    }

    private Action transition(boolean deploy) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    if (deploy) deploy(); else stow();
                }
                return isConfigured() && !isSettled();
            }
        };
    }
}
