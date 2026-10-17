package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;

/**
 * {@link Localizer} backed by a goBILDA Pinpoint odometry computer (2x dead wheels + IMU,
 * fused on the sensor). The Pinpoint tracks the field pose itself, so this class is a thin
 * adapter: push a known pose down with {@link #setPose}, read it back in {@link #update}.
 *
 * Replaces last season's Road-Runner-coupled PinpointLocalizer.
 * Hardware config: a "I2C Device" / "goBILDA Pinpoint" named "pinpoint".
 */
@Config
public final class PinpointLocalizer implements Localizer {

    public static class Params {
        /**
         * Pod offsets from the robot's tracking center, in millimetres.
         *   xOffset: sideways distance to the X (forward) pod - left of center is POSITIVE.
         *   yOffset: forward distance to the Y (strafe) pod   - forward of center is POSITIVE.
         *
         * Tuned on the 2026 prototype chassis (pods on the rear right) with Pinpoint Offset Check:
         * x stayed at last year's -55.0, y moved from -85.5 to -75.6 (offset-only wobble
         * 0.72 in -> 0.33 in; remaining error is at the ~3 mm noise floor).
         * >>> A different chassis needs its own offsets: rerun Pinpoint Offset Check. <<<
         */
        public double xOffsetMM = -55.0;
        public double yOffsetMM = -75.6;

        public GoBildaPinpointDriver.GoBildaOdometryPods podType =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        /**
         * Pod encoder directions, per chassis. First-draft new chassis: Pinpoint Push Test reported
         * forward as -X and left as -Y with REVERSED/REVERSED, so both are FORWARD here. The earlier
         * prototype chassis (and the 2025 robot) used REVERSED/REVERSED.
         */
        public GoBildaPinpointDriver.EncoderDirection xDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    public static Params PARAMS = new Params();

    private static final long CALIBRATION_MIN_MS = 300;       // the device takes about 0.25 s
    private static final long CALIBRATION_TIMEOUT_MS = 2000;
    private static final int SEED_ATTEMPTS = 5;
    private static final double SEED_TOLERANCE_IN = 0.5;
    private static final double SEED_TOLERANCE_RAD = Math.toRadians(2.0);

    public final GoBildaPinpointDriver pinpoint;
    /** False if the device never reported the requested start pose back; the pose is then NOT trustworthy. */
    public final boolean seedConfirmed;
    private Pose2d pose;
    private PoseVelocity2d velocity = PoseVelocity2d.zero();

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        applyParams();

        // Zeroes the position and recalibrates the IMU - the robot must be stationary.
        pinpoint.resetPosAndIMU();
        // The reset runs on the device for ~0.25 s and zeroes the position when it processes.
        // A pose written before then is lost (a hardware run seeded at (-63, 12.75) behaved as
        // if it were at (0, 0)), so wait it out, then write the pose and read it back.
        waitForCalibration();
        seedConfirmed = seedPose(initialPose);
    }

    private void waitForCalibration() {
        sleepMs(CALIBRATION_MIN_MS);
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < CALIBRATION_TIMEOUT_MS) {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) return;
            sleepMs(10);
        }
    }

    private boolean seedPose(Pose2d initialPose) {
        for (int attempt = 0; attempt < SEED_ATTEMPTS; attempt++) {
            setPose(initialPose);
            sleepMs(30);
            pinpoint.update();
            double dx = pinpoint.getPosX(DistanceUnit.INCH) - initialPose.position.x;
            double dy = pinpoint.getPosY(DistanceUnit.INCH) - initialPose.position.y;
            double dh = Rotation2d.normalize(pinpoint.getHeading(AngleUnit.RADIANS) - initialPose.heading.toDouble());
            if (Math.hypot(dx, dy) <= SEED_TOLERANCE_IN && Math.abs(dh) <= SEED_TOLERANCE_RAD) return true;
        }
        return false;
    }

    private static void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** Push offsets, pod type and encoder directions from {@link #PARAMS} to the device. Bring-up opmodes call this to apply Dashboard edits. */
    public void applyParams() {
        pinpoint.setOffsets(PARAMS.xOffsetMM, PARAMS.yOffsetMM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PARAMS.podType);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, pose.position.x, pose.position.y,
                AngleUnit.RADIANS, pose.heading.toDouble()));
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    public PoseVelocity2d getVelocity() {
        return velocity;
    }

    @Override
    public PoseVelocity2d update() {
        pinpoint.update();

        pose = new Pose2d(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                Rotation2d.normalize(pinpoint.getHeading(AngleUnit.RADIANS)));

        velocity = new PoseVelocity2d(
                new Vector2d(
                        pinpoint.getVelX(DistanceUnit.INCH),
                        pinpoint.getVelY(DistanceUnit.INCH)),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        return velocity;
    }

    /** Raw Pinpoint device status - useful for telemetry / fault detection. */
    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return pinpoint.getDeviceStatus();
    }
}
