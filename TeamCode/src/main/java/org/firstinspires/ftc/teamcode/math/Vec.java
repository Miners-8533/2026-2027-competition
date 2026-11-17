package org.firstinspires.ftc.teamcode.math;

/** A 2D vector or point, inches. Immutable. */
public final class Vec {
    public final double x, y;

    public Vec(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vec plus(Vec o) { return new Vec(x + o.x, y + o.y); }
    public Vec minus(Vec o) { return new Vec(x - o.x, y - o.y); }
    public Vec times(double k) { return new Vec(x * k, y * k); }
    public double dot(Vec o) { return x * o.x + y * o.y; }
    public double cross(Vec o) { return x * o.y - y * o.x; }
    public double norm() { return Math.hypot(x, y); }
    public double dist(Vec o) { return Math.hypot(x - o.x, y - o.y); }
    public double angle() { return Math.atan2(y, x); }

    public Vec unit() {
        double n = norm();
        return n < 1e-9 ? new Vec(1, 0) : new Vec(x / n, y / n);
    }

    /** Rotated counterclockwise by the angle. */
    public Vec rotated(double rad) {
        double c = Math.cos(rad), s = Math.sin(rad);
        return new Vec(x * c - y * s, x * s + y * c);
    }

    /** The left normal (this turned 90 degrees counterclockwise). */
    public Vec left() { return new Vec(-y, x); }

    public Vec rotated180() { return new Vec(-x, -y); }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "(%.1f, %.1f)", x, y);
    }
}
