package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Vec;

/**
 * The leg follower shared by both dashes (specs/dashes.md section 2). Each loop it answers: which leg the robot is on,
 * whether the goal is reached, and a translation command in the field frame. The dash decides the heading.
 */
public final class LegFollower {
    /** The values a dash follows with. */
    public static final class Params {
        public double maxPower, goalTol, passTol;
        public double lastLegPerInch, lastLegMin;
        public double sidePerInch, sideLimit;
        /** Hive Dash only: slow down before a via point, by the turn there. */
        public boolean cornerSlow;
        public double cornerPower, cornerFreeDeg, cornerFullDeg, cornerRampPerInch;
        /** Hive Dash only: cap the power near the goal. */
        public boolean approachLimit;
        public double approachPower, approachRadius;
    }

    private Plan plan;
    private int leg;
    private boolean arrived;
    private double vx, vy;

    public void start(Plan p) {
        plan = p;
        leg = 0;
        arrived = false;
        vx = vy = 0;
    }

    public Plan plan() { return plan; }
    public int leg() { return leg; }
    public boolean onLastLeg() { return plan != null && leg >= plan.legs() - 1; }
    public boolean arrived() { return arrived; }
    public double vx() { return vx; }
    public double vy() { return vy; }
    /** Distance from the robot to the goal at the last update. */
    public double goalDistance;

    public void update(double x, double y, Params k) {
        vx = vy = 0;
        if (plan == null || !plan.ok()) return;
        Vec p = new Vec(x, y);
        int lastLeg = plan.legs() - 1;
        // A via point counts as reached when the robot is within the pass tolerance of it along the leg, or past it. It does not stop there.
        while (leg < lastLeg) {
            Vec a = plan.points[leg], b = plan.points[leg + 1];
            Vec u = b.minus(a).unit();
            if (b.minus(p).dot(u) <= k.passTol) leg++;
            else break;
        }
        Vec goal = plan.goal();
        goalDistance = p.dist(goal);
        if (leg < lastLeg) {
            Vec a = plan.points[leg], b = plan.points[leg + 1];
            Vec u = b.minus(a).unit(), n = u.left();
            double power = k.maxPower;
            if (k.cornerSlow) {
                double span = Math.max(1e-6, k.cornerFullDeg - k.cornerFreeDeg);
                double frac = Angle.clamp((plan.turnAfterDeg(leg) - k.cornerFreeDeg) / span, 0, 1);
                double corner = k.maxPower - (k.maxPower - k.cornerPower) * frac;
                double toVia = Math.max(0, b.minus(p).dot(u));
                power = Math.min(power, corner + k.cornerRampPerInch * toVia);
            }
            if (k.approachLimit && goalDistance < k.approachRadius) power = Math.min(power, k.approachPower);
            double off = p.minus(a).dot(n);                 // distance to the left of the leg's line
            double side = Angle.clampAbs(-k.sidePerInch * off, k.sideLimit);
            vx = u.x * power + n.x * side;
            vy = u.y * power + n.y * side;
        } else {
            // The last leg: straight at the goal, power proportional to the distance, never below a minimum,
            // so a robot that overshoots or passes to the side is driven back.
            arrived = goalDistance <= k.goalTol;
            if (arrived) return;
            double power = Angle.clamp(k.lastLegPerInch * goalDistance, k.lastLegMin, k.maxPower);
            if (k.approachLimit && goalDistance < k.approachRadius) power = Math.min(power, k.approachPower);
            Vec u = goal.minus(p).unit();
            vx = u.x * power;
            vy = u.y * power;
        }
    }
}
