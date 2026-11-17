package org.firstinspires.ftc.teamcode.navigation;

import java.util.ArrayList;
import java.util.List;

/** Plane geometry for the route planner. A polygon is double[n][2], convex, in order. */
final class Geom {
    private Geom() { }

    static double[][] rect(double xMin, double yMin, double xMax, double yMax) {
        return new double[][] {{xMin, yMin}, {xMax, yMin}, {xMax, yMax}, {xMin, yMax}};
    }

    /** A rectangle around the segment a-b, reaching halfWidth to each side. */
    static double[][] band(double ax, double ay, double bx, double by, double halfWidth) {
        double dx = bx - ax, dy = by - ay, len = Math.hypot(dx, dy);
        if (len < 1e-9) return rect(ax - halfWidth, ay - halfWidth, ax + halfWidth, ay + halfWidth);
        double nx = -dy / len * halfWidth, ny = dx / len * halfWidth;
        return new double[][] {{ax + nx, ay + ny}, {bx + nx, by + ny}, {bx - nx, by - ny}, {ax - nx, ay - ny}};
    }

    static double pointSeg(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
        double t = len2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2));
        return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
    }

    private static double cross(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

    static boolean segsIntersect(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
        double d1 = cross(bx - ax, by - ay, cx - ax, cy - ay), d2 = cross(bx - ax, by - ay, dx - ax, dy - ay);
        double d3 = cross(dx - cx, dy - cy, ax - cx, ay - cy), d4 = cross(dx - cx, dy - cy, bx - cx, by - cy);
        return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
    }

    static double segSeg(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
        if (segsIntersect(ax, ay, bx, by, cx, cy, dx, dy)) return 0;
        return Math.min(Math.min(pointSeg(ax, ay, cx, cy, dx, dy), pointSeg(bx, by, cx, cy, dx, dy)),
                Math.min(pointSeg(cx, cy, ax, ay, bx, by), pointSeg(dx, dy, ax, ay, bx, by)));
    }

    static boolean inside(double[][] poly, double px, double py) {
        if (poly.length < 3) return false;
        int sign = 0;
        for (int i = 0; i < poly.length; i++) {
            double[] a = poly[i], b = poly[(i + 1) % poly.length];
            double c = cross(b[0] - a[0], b[1] - a[1], px - a[0], py - a[1]);
            if (Math.abs(c) < 1e-12) continue;
            int s = c > 0 ? 1 : -1;
            if (sign == 0) sign = s;
            else if (s != sign) return false;
        }
        return true;
    }

    /** Distance from a point to a polygon (0 inside). */
    static double polyPoint(double[][] poly, double px, double py) {
        if (poly.length == 0) return Double.MAX_VALUE;
        if (inside(poly, px, py)) return 0;
        double best = Double.MAX_VALUE;
        for (int i = 0; i < poly.length; i++) {
            double[] a = poly[i], b = poly[(i + 1) % poly.length];
            best = Math.min(best, pointSeg(px, py, a[0], a[1], b[0], b[1]));
        }
        return best;
    }

    /** Distance from a segment to a polygon (0 when they touch). */
    static double polySeg(double[][] poly, double ax, double ay, double bx, double by) {
        if (poly.length == 0) return Double.MAX_VALUE;
        if (inside(poly, ax, ay) || inside(poly, bx, by)) return 0;
        double best = Double.MAX_VALUE;
        for (int i = 0; i < poly.length; i++) {
            double[] c = poly[i], d = poly[(i + 1) % poly.length];
            best = Math.min(best, segSeg(ax, ay, bx, by, c[0], c[1], d[0], d[1]));
        }
        return best;
    }

    /** Distance between two convex polygons (0 when they touch). */
    static double polyPoly(double[][] p, double[][] q) {
        if (p.length == 0 || q.length == 0) return Double.MAX_VALUE;
        if (inside(p, q[0][0], q[0][1]) || inside(q, p[0][0], p[0][1])) return 0;
        double best = Double.MAX_VALUE;
        for (int i = 0; i < p.length; i++) {
            double[] a = p[i], b = p[(i + 1) % p.length];
            best = Math.min(best, polySeg(q, a[0], a[1], b[0], b[1]));
        }
        return best;
    }

    /** The part of a polygon on the side of the line through (ox, oy) that the direction (ux, uy) points to. May be empty. */
    static double[][] clip(double[][] poly, double ox, double oy, double ux, double uy) {
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < poly.length; i++) {
            double[] a = poly[i], b = poly[(i + 1) % poly.length];
            double da = (a[0] - ox) * ux + (a[1] - oy) * uy, db = (b[0] - ox) * ux + (b[1] - oy) * uy;
            if (da >= 0) out.add(a);
            if ((da >= 0) != (db >= 0)) {
                double t = da / (da - db);
                out.add(new double[] {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])});
            }
        }
        return out.toArray(new double[0][]);
    }
}
