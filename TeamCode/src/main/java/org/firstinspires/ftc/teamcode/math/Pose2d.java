package org.firstinspires.ftc.teamcode.math;

/**
 * Immutable robot pose on the field: a position plus a heading.
 * From-scratch replacement for Road Runner's {@code Pose2d}.
 *
 * Field coordinates are inches; heading is radians CCW-positive.
 */
public final class Pose2d {
    public final Vector2d position;
    public final Rotation2d heading;

    public Pose2d(Vector2d position, Rotation2d heading) {
        this.position = position;
        this.heading = heading;
    }

    public Pose2d(Vector2d position, double headingRad) {
        this(position, new Rotation2d(headingRad));
    }

    public Pose2d(double x, double y, double headingRad) {
        this(new Vector2d(x, y), new Rotation2d(headingRad));
    }

    public static Pose2d zero() {
        return new Pose2d(0.0, 0.0, 0.0);
    }

    public double getX() {
        return position.x;
    }

    public double getY() {
        return position.y;
    }

    public double getHeading() {
        return heading.toDouble();
    }

    public Pose2d withHeading(double headingRad) {
        return new Pose2d(position, new Rotation2d(headingRad));
    }

    public Pose2d withPosition(Vector2d p) {
        return new Pose2d(p, heading);
    }

    /** Pose of {@code this} expressed relative to {@code reference} (reference-frame error). */
    public Pose2d relativeTo(Pose2d reference) {
        Vector2d delta = position.minus(reference.position);
        return new Pose2d(
                delta.rotated(-reference.heading.toDouble()),
                heading.minus(reference.heading).toDouble());
    }

    @Override
    public String toString() {
        return String.format("Pose2d{%s, %s}", position, heading);
    }
}
