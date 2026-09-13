package org.firstinspires.ftc.teamcode.math;

/**
 * Immutable 2D vector. Field convention (matches our odometry / field coordinates):
 *   +x = away from the audience / "forward" for a robot at 0 heading
 *   +y = to the left
 *
 * From-scratch replacement for the Road Runner {@code Vector2d}. Kept deliberately
 * small - add only what the robot code actually needs.
 */
public final class Vector2d {
    public final double x;
    public final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2d zero() {
        return new Vector2d(0.0, 0.0);
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(x - other.x, y - other.y);
    }

    public Vector2d times(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    public Vector2d div(double scalar) {
        return new Vector2d(x / scalar, y / scalar);
    }

    public Vector2d unaryMinus() {
        return new Vector2d(-x, -y);
    }

    public double dot(Vector2d other) {
        return x * other.x + y * other.y;
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public double sqrNorm() {
        return x * x + y * y;
    }

    /** Heading of this vector, radians in (-pi, pi]. */
    public double angle() {
        return Math.atan2(y, x);
    }

    /** Rotate this vector by {@code angleRad} counter-clockwise. */
    public Vector2d rotated(double angleRad) {
        double c = Math.cos(angleRad);
        double s = Math.sin(angleRad);
        return new Vector2d(x * c - y * s, x * s + y * c);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }
}
