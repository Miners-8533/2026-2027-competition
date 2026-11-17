package org.firstinspires.ftc.teamcode.math;

import java.util.Locale;

/** A field pose: inches, heading in radians (0 = facing +X, counterclockwise positive). Immutable. */
public final class Pose {
    public final double x, y, h;

    public Pose(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.h = headingRad;
    }

    public static Pose deg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    public double headingDeg() {
        return Math.toDegrees(h);
    }

    public double distTo(double px, double py) {
        return Math.hypot(px - x, py - y);
    }

    public double distTo(Pose o) {
        return distTo(o.x, o.y);
    }

    public Vec vec() {
        return new Vec(x, y);
    }

    /** The same pose for the other alliance: rotated 180 degrees about the field center. */
    public Pose rotated180() {
        return new Pose(-x, -y, Angle.wrap(h + Math.PI));
    }

    public Pose withHeading(double headingRad) {
        return new Pose(x, y, headingRad);
    }

    public boolean isFinite() {
        return !(Double.isNaN(x) || Double.isInfinite(x) || Double.isNaN(y) || Double.isInfinite(y) || Double.isNaN(h) || Double.isInfinite(h));
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "(%.1f, %.1f, %.1f deg)", x, y, headingDeg());
    }
}
