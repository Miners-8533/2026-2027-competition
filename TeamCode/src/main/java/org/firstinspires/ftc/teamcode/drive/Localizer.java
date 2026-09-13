package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;

/**
 * Field-pose estimator. Replaces the Road Runner {@code Localizer} interface.
 *
 * Implementations own the sensor(s) and integrate them into a field pose.
 * {@link #update()} must be called exactly once per control loop.
 */
public interface Localizer {

    /** Force the pose estimate to a known value (e.g. at auton start, or from an AprilTag fix). */
    void setPose(Pose2d pose);

    /** Latest field pose estimate. */
    Pose2d getPose();

    /**
     * Read sensors, advance the estimate, and return the current field-frame velocity.
     * Call exactly once per loop, before reading {@link #getPose()}.
     */
    PoseVelocity2d update();
}
