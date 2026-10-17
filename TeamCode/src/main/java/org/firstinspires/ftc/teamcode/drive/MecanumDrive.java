package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;

/**
 * Minimal mecanum drivetrain: four motors, open-loop power mixing, plus a {@link Localizer}
 * for the field pose. No trajectory following here - path/pose control will live in
 * {@code org.firstinspires.ftc.teamcode.motion} (the team's own framework, not yet built).
 *
 * From-scratch replacement for last season's Road Runner {@code MecanumDrive}. The
 * power-mixing math is intentionally identical to what Road Runner's
 * {@code MecanumKinematics(1).inverse(...)} produced, so drive feel carries over.
 *
 * Hardware config names: "leftFront", "leftBack", "rightFront", "rightBack", "pinpoint".
 */
@Config
public final class MecanumDrive {

    public static class Params {
        /** Global scale on all wheel powers (e.g. a "slow"/precision mode multiplier). */
        public double powerScale = 1.0;
    }

    public static Params PARAMS = new Params();

    /**
     * Directions that make a positive power turn each wheel so the robot is driven forward.
     * Per chassis. Currently set for the first draft of the new chassis (left REVERSE, right
     * FORWARD, the same pair as the 2025 robot). The earlier prototype chassis needed the
     * opposite pair (left FORWARD, right REVERSE): Motor Map Test showed every wheel of it
     * driving the robot backward with this one. Rerun Motor Map Test on any new chassis.
     */
    public static final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public final Localizer localizer;

    private PoseVelocity2d lastCommand = PoseVelocity2d.zero();

    public MecanumDrive(HardwareMap hardwareMap, Pose2d initialPose) {
        // Bulk reads: one round trip per loop for all motor encoders / currents.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(LEFT_DIRECTION);
        leftBack.setDirection(LEFT_DIRECTION);
        rightFront.setDirection(RIGHT_DIRECTION);
        rightBack.setDirection(RIGHT_DIRECTION);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        localizer = new PinpointLocalizer(hardwareMap, initialPose);
    }

    /**
     * Open-loop drive. Components are powers in [-1, 1]:
     *   linearVel.x = forward, linearVel.y = left/strafe, angVel = counter-clockwise turn.
     * Wheel powers are normalized so the largest magnitude never exceeds 1.
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        lastCommand = powers;

        double x = powers.linearVel.x;
        double y = powers.linearVel.y;
        double w = powers.angVel;

        double lf = x - y - w;
        double lb = x + y - w;
        double rb = x - y + w;
        double rf = x + y + w;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(lb), Math.max(Math.abs(rb), Math.abs(rf)))));

        double s = PARAMS.powerScale / max;
        leftFront.setPower(lf * s);
        leftBack.setPower(lb * s);
        rightBack.setPower(rb * s);
        rightFront.setPower(rf * s);
    }

    public void stop() {
        setDrivePowers(PoseVelocity2d.zero());
    }

    /** Advance the pose estimate. Call once per loop, before reading {@link #getPose()}. */
    public PoseVelocity2d updatePoseEstimate() {
        return localizer.update();
    }

    public Pose2d getPose() {
        return localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        localizer.setPose(pose);
    }

    public PoseVelocity2d getLastCommand() {
        return lastCommand;
    }
}
