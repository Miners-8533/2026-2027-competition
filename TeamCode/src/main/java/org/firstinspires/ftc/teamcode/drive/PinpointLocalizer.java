package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

        public GoBildaPinpointDriver.EncoderDirection xDirection =
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection =
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver pinpoint;
    private Pose2d pose;
    private PoseVelocity2d velocity = PoseVelocity2d.zero();

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        applyParams();

        // Recalibrates the IMU - the robot must be stationary. waitForStart() in the
        // OpMode gives it time to finish before the match begins.
        pinpoint.resetPosAndIMU();

        this.pose = initialPose;
        setPose(initialPose);
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
