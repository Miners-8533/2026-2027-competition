package org.firstinspires.ftc.teamcode.math;

/** Angle and number helpers. Angles are radians unless a name says Deg. */
public final class Angle {
    private Angle() { }

    /** Wraps to (-pi, pi]. */
    public static double wrap(double rad) {
        double a = rad % (2 * Math.PI);
        if (a <= -Math.PI) a += 2 * Math.PI;
        else if (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }

    /** Wraps to (-180, 180]. */
    public static double wrapDeg(double deg) {
        double a = deg % 360.0;
        if (a <= -180.0) a += 360.0;
        else if (a > 180.0) a -= 360.0;
        return a;
    }

    /** The short way round from current to target. */
    public static double diff(double target, double current) {
        return wrap(target - current);
    }

    public static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    public static double clampAbs(double v, double limit) {
        return clamp(v, -Math.abs(limit), Math.abs(limit));
    }
}
