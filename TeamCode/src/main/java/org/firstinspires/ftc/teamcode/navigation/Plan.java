package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.math.Vec;

/** A planned route: waypoints from the robot to the goal, or a refusal with a reason. */
public final class Plan {
    /** points[0] is where the robot was when the plan was made; the last is the goal. */
    public final Vec[] points;
    /** Per leg: long enough to be driven with the robot aligned to it. */
    public final boolean[] aligned;
    public final double cost;
    public final String refusal;

    Plan(Vec[] points, boolean[] aligned, double cost) {
        this.points = points;
        this.aligned = aligned;
        this.cost = cost;
        this.refusal = null;
    }

    Plan(String refusal) {
        this.points = new Vec[0];
        this.aligned = new boolean[0];
        this.cost = Double.NaN;
        this.refusal = refusal;
    }

    public boolean ok() { return refusal == null; }

    public int legs() { return Math.max(0, points.length - 1); }

    public Vec goal() { return points[points.length - 1]; }

    /** Direction of a leg, radians. */
    public double legAngle(int leg) {
        return points[leg + 1].minus(points[leg]).angle();
    }

    /** How much the direction of travel changes at the end of a leg, degrees, 0 to 180. 0 for the last leg. */
    public double turnAfterDeg(int leg) {
        if (leg >= legs() - 1) return 0;
        double d = Math.toDegrees(legAngle(leg + 1) - legAngle(leg));
        d = Math.abs(((d + 540) % 360) - 180);
        return d;
    }

    public double length() {
        double s = 0;
        for (int i = 0; i < legs(); i++) s += points[i].dist(points[i + 1]);
        return s;
    }

    public String describe() {
        if (!ok()) return "refused: " + refusal;
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < points.length; i++) {
            if (i > 0) sb.append(aligned[i - 1] ? " => " : " -> ");
            sb.append(points[i]);
        }
        return sb.append(String.format(java.util.Locale.US, ", %.0f in, %d via", length(), Math.max(0, points.length - 2))).toString();
    }
}
