package org.firstinspires.ftc.teamcode.math;

/**
 * Immutable heading, stored as radians but with cached cos/sin for cheap rotations.
 * From-scratch replacement for Road Runner's {@code Rotation2d}.
 */
public final class Rotation2d {
    private final double radians;
    public final double real; // cos
    public final double imag; // sin

    public Rotation2d(double radians) {
        this.radians = normalize(radians);
        this.real = Math.cos(this.radians);
        this.imag = Math.sin(this.radians);
    }

    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    /** Radians in (-pi, pi]. */
    public double toDouble() {
        return radians;
    }

    public double toDegrees() {
        return Math.toDegrees(radians);
    }

    public Rotation2d plus(double deltaRad) {
        return new Rotation2d(radians + deltaRad);
    }

    public Rotation2d minus(Rotation2d other) {
        return new Rotation2d(radians - other.radians);
    }

    /** Signed shortest angular distance from {@code this} to {@code target}, radians. */
    public double distanceTo(Rotation2d target) {
        return normalize(target.radians - radians);
    }

    /** Rotate a vector by this rotation (counter-clockwise). */
    public Vector2d apply(Vector2d v) {
        return new Vector2d(real * v.x - imag * v.y, imag * v.x + real * v.y);
    }

    /** Wrap an angle to (-pi, pi]. */
    public static double normalize(double angleRad) {
        double a = angleRad % (2.0 * Math.PI);
        if (a <= -Math.PI) {
            a += 2.0 * Math.PI;
        } else if (a > Math.PI) {
            a -= 2.0 * Math.PI;
        }
        return a;
    }

    @Override
    public String toString() {
        return String.format("%.1f deg", toDegrees());
    }
}
