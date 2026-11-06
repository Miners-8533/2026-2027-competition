package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

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
        /**
         * A wheel power within this of the last one SENT is not sent (each hub write blocks for a few ms). 0 sends every change. A stop (0 after a non-zero power)
         * is always sent, and a power is re-sent at least every refreshMs so the hub's own refresh and the motor controller never go long without a command.
         */
        public double writeDeadband = 0.01;
        public double refreshMs = 200.0;
    }

    public static Params PARAMS = new Params();

    public final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public final Localizer localizer;
    /** Per-chassis values (motor directions here, pod settings in the localizer), picked by hub. */
    public final ChassisProfile profile;

    private PoseVelocity2d lastCommand = PoseVelocity2d.zero();

    /** The autos and test opmodes: the Pinpoint is reset (position zeroed, IMU recalibrated: the robot must be still) and seeded with initialPose. */
    public MecanumDrive(HardwareMap hardwareMap, Pose2d initialPose) {
        this(hardwareMap, initialPose, false);
    }

    /**
     * With adoptPinpointPose true the Pinpoint is NOT reset or written to: its own pose, kept since the last opmode, is used (the TeleOp after an auto;
     * initialPose is then only the pose the auto saved, used to check the device and as the fallback if it looks restarted). See
     * {@link PinpointLocalizer#PinpointLocalizer(HardwareMap, Pose2d, boolean)}.
     */
    public MecanumDrive(HardwareMap hardwareMap, Pose2d initialPose, boolean adoptPinpointPose) {
        // Bulk reads: one round trip per loop for all motor encoders / currents.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        profile = ChassisProfile.detect(hardwareMap);
        leftFront.setDirection(profile.leftDirection);
        leftBack.setDirection(profile.leftDirection);
        rightFront.setDirection(profile.rightDirection);
        rightBack.setDirection(profile.rightDirection);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        localizer = new PinpointLocalizer(hardwareMap, initialPose, adoptPinpointPose);
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
        long t0 = LoopProfiler.start();   // the hub writes (the SDK itself skips only a power exactly equal to the last one; see writePower)
        long now = System.nanoTime();
        writePower(0, leftFront, lf * s, now);
        writePower(1, leftBack, lb * s, now);
        writePower(2, rightBack, rb * s, now);
        writePower(3, rightFront, rf * s, now);
        LoopProfiler.stop("drive_write", t0);
    }

    private final double[] sentPower = new double[4];   // the last power actually sent to each motor
    private final long[] sentAtNs = new long[4];         // 0 = nothing sent yet

    /** Sends a wheel power unless it is within PARAMS.writeDeadband of the last one sent (a stop and the periodic refresh are always sent). */
    private void writePower(int i, DcMotorEx motor, double power, long nowNs) {
        double dead = PARAMS.writeDeadband;
        boolean skip = dead > 0.0 && sentAtNs[i] != 0L
                && Math.abs(power - sentPower[i]) < dead
                && !(power == 0.0 && sentPower[i] != 0.0)
                && (nowNs - sentAtNs[i]) < PARAMS.refreshMs * 1e6;
        if (skip) {
            LoopProfiler.count("drive_skipped");
            return;
        }
        motor.setPower(power);
        sentPower[i] = power;
        sentAtNs[i] = nowNs;
        LoopProfiler.count("drive_sent");
    }

    public void stop() {
        setDrivePowers(PoseVelocity2d.zero());
    }

    /** Advance the pose estimate. Call once per loop, before reading {@link #getPose()}. */
    public PoseVelocity2d updatePoseEstimate() {
        long t0 = LoopProfiler.start();
        try {
            return localizer.update();
        } finally {
            LoopProfiler.stop("pose", t0);   // the Pinpoint read
        }
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
