package org.firstinspires.ftc.teamcode.math;

/**
 * A robot-frame velocity command / measurement: linear velocity plus angular velocity.
 * From-scratch replacement for Road Runner's {@code PoseVelocity2d}.
 *
 * When used as a drive command from {@code MecanumDrive.setDrivePowers}, the
 * components are unitless powers in [-1, 1]:
 *   linearVel.x = forward power, linearVel.y = left/strafe power, angVel = CCW turn power.
 */
public final class PoseVelocity2d {
    public final Vector2d linearVel;
    public final double angVel;

    public PoseVelocity2d(Vector2d linearVel, double angVel) {
        this.linearVel = linearVel;
        this.angVel = angVel;
    }

    public static PoseVelocity2d zero() {
        return new PoseVelocity2d(Vector2d.zero(), 0.0);
    }

    @Override
    public String toString() {
        return String.format("PoseVelocity2d{%s, w=%.2f}", linearVel, angVel);
    }
}
