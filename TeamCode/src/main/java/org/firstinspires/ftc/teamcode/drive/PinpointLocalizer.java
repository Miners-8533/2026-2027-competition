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

import java.util.Locale;

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
         * These are overwritten from the detected {@link ChassisProfile} every time this localizer is
         * constructed, so the real per-chassis values live there. Edit them in the FTC Dashboard
         * while an opmode runs to try a change; it is lost at the next init.
         */
        public double xOffsetMM = -55.0;
        public double yOffsetMM = -75.6;

        public GoBildaPinpointDriver.GoBildaOdometryPods podType =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        /** Pod encoder directions; also overwritten from the {@link ChassisProfile} at construction. */
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

    /** Adopting the device's own pose (the TeleOp after an auto): how long to wait for READY, and the "the device just restarted" test. */
    private static final long ADOPT_READY_TIMEOUT_MS = 500;
    private static final double ADOPT_ZERO_TOL_IN = 1.0;
    private static final double ADOPT_ZERO_TOL_RAD = Math.toRadians(2.0);
    private static final double ADOPT_SAVED_FAR_FROM_ORIGIN_IN = 3.0;
    private static final double ADOPT_SAVED_FAR_FROM_ZERO_RAD = Math.toRadians(5.0);
    private static final double ADOPT_FIELD_MARGIN_IN = 6.0;

    /** How the device was brought up. */
    public enum StartMode {
        /** The device was reset (position zeroed, IMU recalibrated) and seeded with the requested pose: the autos, and a TeleOp with no fresh auto pose. */
        RESET_AND_SEED,
        /** Nothing was written to the device: its own pose, kept since the last opmode, is used as it is. */
        ADOPTED,
        /** Adoption was asked for but the device did not look usable, so it was reset and seeded with the saved pose instead. */
        RESEEDED_AFTER_FAILED_ADOPT
    }

    public final GoBildaPinpointDriver pinpoint;
    /** False if the device never reported the requested start pose back; the pose is then NOT trustworthy. (True when the device's pose was adopted.) */
    public final boolean seedConfirmed;
    /** How the device was brought up, and one line saying why and with what numbers (for the INIT screen and the run log). */
    public final StartMode startMode;
    public final String startNote;
    /** The device's own pose when it was adopted, else null. */
    public final Pose2d adoptedDevicePose;
    /** How long the bring-up took (the reset path waits for the IMU calibration). */
    public final double bringUpMs;
    private Pose2d pose;
    private PoseVelocity2d velocity = PoseVelocity2d.zero();

    /** Reset the device and seed it with the pose: the autos. The robot must be still. */
    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        this(hardwareMap, initialPose, false);
    }

    /**
     * With tryAdopt false: reset the device (zero the position, recalibrate the IMU: the robot must be still) and seed it with seedPose.
     * With tryAdopt true (the TeleOp after an auto, whose pose is seedPose): write NOTHING to the device, so the pose it has kept since the auto,
     * including any bump in the transition, is used as it is and the IMU is not recalibrated. If the device is not READY within 500 ms, reads
     * (0, 0, 0) while seedPose is somewhere else (it restarted), or reads a pose outside the field, it is reset and seeded with seedPose instead.
     */
    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d seedPose, boolean tryAdopt) {
        long startNs = System.nanoTime();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        PARAMS.xOffsetMM = profile.xOffsetMM;
        PARAMS.yOffsetMM = profile.yOffsetMM;
        PARAMS.xDirection = profile.podXDirection;
        PARAMS.yDirection = profile.podYDirection;

        Adoption adoption = tryAdopt ? checkAdoption(seedPose) : null;
        if (adoption != null && adoption.ok) {
            // nothing is written to the device: the offsets, directions and resolution are still in it from the auto
            pose = adoption.devicePose;
            seedConfirmed = true;
            startMode = StartMode.ADOPTED;
            startNote = adoption.note;
            adoptedDevicePose = adoption.devicePose;
        } else {
            applyParams();
            // Zeroes the position and recalibrates the IMU - the robot must be stationary.
            pinpoint.resetPosAndIMU();
            // The reset runs on the device for ~0.25 s and zeroes the position when it processes.
            // A pose written before then is lost (a hardware run seeded at (-63, 12.75) behaved as
            // if it were at (0, 0)), so wait it out, then write the pose and read it back.
            waitForCalibration();
            seedConfirmed = seedPose(seedPose);
            startMode = adoption != null ? StartMode.RESEEDED_AFTER_FAILED_ADOPT : StartMode.RESET_AND_SEED;
            startNote = (adoption != null ? adoption.note + "; then " : "") + String.format(Locale.US,
                    "reset (position zeroed, IMU recalibrated) and seeded to (%.2f, %.2f, %.1f deg), read back %s, device %s",
                    seedPose.position.x, seedPose.position.y, seedPose.heading.toDegrees(), seedConfirmed ? "OK" : "NOT CONFIRMED", pinpoint.getDeviceStatus());
            adoptedDevicePose = null;
        }
        bringUpMs = (System.nanoTime() - startNs) / 1e6;
    }

    private static final class Adoption {
        boolean ok = false;
        Pose2d devicePose = null;
        String note = "";
    }

    /** Reads the device without writing to it and decides whether its pose can be used as it is. */
    private Adoption checkAdoption(Pose2d saved) {
        Adoption a = new Adoption();
        ElapsedTime timer = new ElapsedTime();
        GoBildaPinpointDriver.DeviceStatus status;
        while (true) {
            pinpoint.update();
            status = pinpoint.getDeviceStatus();
            if (status == GoBildaPinpointDriver.DeviceStatus.READY || timer.milliseconds() >= ADOPT_READY_TIMEOUT_MS) break;
            sleepMs(10);
        }
        double x = pinpoint.getPosX(DistanceUnit.INCH), y = pinpoint.getPosY(DistanceUnit.INCH);
        double h = Rotation2d.normalize(pinpoint.getHeading(AngleUnit.RADIANS));
        double dist = Math.hypot(x - saved.position.x, y - saved.position.y);
        double dHeadingDeg = Math.toDegrees(Rotation2d.normalize(h - saved.heading.toDouble()));
        String device = String.format(Locale.US,
                "device %s after %.0f ms, pose (%.2f, %.2f, %.1f deg), loop %d us (%.0f Hz), firmware %d, encoders %d / %d; the saved pose (%.2f, %.2f, %.1f deg) differs by %.1f in, %.1f deg",
                status, timer.milliseconds(), x, y, Math.toDegrees(h), pinpoint.getLoopTime(), pinpoint.getFrequency(), pinpoint.getDeviceVersion(),
                pinpoint.getEncoderX(), pinpoint.getEncoderY(), saved.position.x, saved.position.y, saved.heading.toDegrees(), dist, dHeadingDeg);
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            a.note = "NOT adopted, the device is not READY: " + device;
            return a;
        }
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(h)) {
            a.note = "NOT adopted, a NaN reading: " + device;
            return a;
        }
        boolean readsZero = Math.hypot(x, y) <= ADOPT_ZERO_TOL_IN && Math.abs(h) <= ADOPT_ZERO_TOL_RAD;
        boolean savedIsNearZero = Math.hypot(saved.position.x, saved.position.y) <= ADOPT_SAVED_FAR_FROM_ORIGIN_IN
                && Math.abs(Rotation2d.normalize(saved.heading.toDouble())) <= ADOPT_SAVED_FAR_FROM_ZERO_RAD;
        if (readsZero && !savedIsNearZero) {
            a.note = "NOT adopted, the device reads (0, 0, 0) but the saved pose is elsewhere, so it probably restarted: " + device;
            return a;
        }
        double limit = FieldGeometry.HALF_IN + ADOPT_FIELD_MARGIN_IN;
        if (Math.abs(x) > limit || Math.abs(y) > limit) {
            a.note = "NOT adopted, the device pose is outside the field: " + device;
            return a;
        }
        a.ok = true;
        a.devicePose = new Pose2d(x, y, h);
        a.note = "adopted (nothing written to the device, IMU not recalibrated): " + device;
        return a;
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
