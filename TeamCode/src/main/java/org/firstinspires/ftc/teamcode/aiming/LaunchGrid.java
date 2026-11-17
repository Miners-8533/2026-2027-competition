package org.firstinspires.ftc.teamcode.aiming;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

import java.util.ArrayList;
import java.util.List;

/**
 * The launch grid (specs/aiming.md section 4): measured flywheel speed and aim residual per field position, instead of
 * "speed from distance, aim from a formula". Off unless the operator switches it on in the TeleOp. Pure logic.
 *
 * Only the blue rear cell is measured. The blue audience cell is a seed (the spots mirrored about x = 0); the red cells
 * are the blue data rotated 180 degrees.
 */
@Config
public final class LaunchGrid {
    private LaunchGrid() { }

    /** The switch. The TeleOp's operator control flips it; nothing else turns it on. */
    public static boolean ENABLED = false;

    public enum Zone { INTERPOLATED, EXTRAPOLATED, NO_SHOT, NO_DATA }

    public static final class Result {
        public final Zone zone;
        public final double speed;
        /** The heading that scored minus the bearing to the cell's center, radians. */
        public final double residualRad;
        /** True for a cell whose data is mirrored, not measured. */
        public final boolean seed;

        Result(Zone zone, double speed, double residualRad, boolean seed) {
            this.zone = zone;
            this.speed = speed;
            this.residualRad = residualRad;
            this.seed = seed;
        }

        public boolean hasValues() { return zone != Zone.NO_DATA; }
    }

    private static double[][] spots;      // rows of LaunchGridData.BLUE_REAR
    private static int[][] triangles;

    private static synchronized void prepare() {
        if (spots != null) return;
        spots = LaunchGridData.BLUE_REAR;
        triangles = spots.length >= 3 ? triangulate(spots) : new int[0][];
    }

    /** For a PC check: use other rows than the generated table. */
    public static synchronized void setData(double[][] rows) {
        spots = rows;
        triangles = rows.length >= 3 ? triangulate(rows) : new int[0][];
    }

    /** The grid's answer for a cell at a position. */
    public static Result lookup(Cell cell, double x, double y) {
        prepare();
        // Bring the query into the measured cell's frame.
        boolean seed = false;
        double sign = 1;
        Cell c = cell;
        if (c.alliance == Alliance.RED) {
            c = c.rotated();
            x = -x;
            y = -y;
        }
        if (c == Cell.BLUE_AUDIENCE) {
            x = -x;          // mirrored about x = 0, keeping the ball's path: the residual changes sign
            sign = -1;
            seed = true;
        }
        if (spots.length == 0) return new Result(Zone.NO_DATA, Double.NaN, Double.NaN, seed);

        int nearest = nearest(x, y, false);
        boolean nearNoShot = spots[nearest][4] < 0.5;
        // Inside a triangle of measured spots: a linear blend of the three; a no-shot spot does not contribute.
        for (int[] t : triangles) {
            double[] w = barycentric(t, x, y);
            if (w == null) continue;
            double[] v = blend(new int[] {t[0], t[1], t[2]}, w);
            if (v == null) break;
            return new Result(nearNoShot ? Zone.NO_SHOT : Zone.INTERPOLATED, v[0], sign * Math.toRadians(v[1]), seed);
        }
        // Outside: the value at the nearest point of the data's outline, at any distance.
        double best = Double.MAX_VALUE;
        double[] bestV = null;
        for (int[] t : triangles) {
            for (int e = 0; e < 3; e++) {
                int a = t[e], b = t[(e + 1) % 3];
                if (!outlineEdge(a, b)) continue;
                double ax = spots[a][0], ay = spots[a][1], bx = spots[b][0], by = spots[b][1];
                double len2 = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
                double s = len2 < 1e-9 ? 0 : Math.max(0, Math.min(1, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / len2));
                double d = Math.hypot(x - (ax + s * (bx - ax)), y - (ay + s * (by - ay)));
                if (d < best) {
                    double[] v = blend(new int[] {a, b}, new double[] {1 - s, s});
                    if (v != null) {
                        best = d;
                        bestV = v;
                    }
                }
            }
        }
        if (bestV == null) {
            int good = nearest(x, y, true);
            if (good < 0) return new Result(Zone.NO_SHOT, Double.NaN, Double.NaN, seed);
            bestV = new double[] {spots[good][2], spots[good][3]};
        }
        return new Result(nearNoShot ? Zone.NO_SHOT : Zone.EXTRAPOLATED, bestV[0], sign * Math.toRadians(bestV[1]), seed);
    }

    private static int nearest(double x, double y, boolean goodOnly) {
        int best = -1;
        double bd = Double.MAX_VALUE;
        for (int i = 0; i < spots.length; i++) {
            if (goodOnly && spots[i][4] < 0.5) continue;
            double d = Math.hypot(x - spots[i][0], y - spots[i][1]);
            if (d < bd) {
                bd = d;
                best = i;
            }
        }
        return best;
    }

    /** Speed and residual (deg) blended over spots with weights; no-shot spots are left out. Null if none is left. */
    private static double[] blend(int[] idx, double[] w) {
        double sw = 0, speed = 0, res = 0;
        for (int i = 0; i < idx.length; i++) {
            double[] s = spots[idx[i]];
            if (s[4] < 0.5) continue;
            sw += w[i];
            speed += w[i] * s[2];
            res += w[i] * s[3];
        }
        if (sw < 1e-6) {
            // Every corner is a no-shot spot, or the point sits on one: take the nearest good corner by weight order.
            return null;
        }
        return new double[] {speed / sw, res / sw};
    }

    private static double[] barycentric(int[] t, double x, double y) {
        double x1 = spots[t[0]][0], y1 = spots[t[0]][1], x2 = spots[t[1]][0], y2 = spots[t[1]][1], x3 = spots[t[2]][0], y3 = spots[t[2]][1];
        double det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
        if (Math.abs(det) < 1e-9) return null;
        double a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det;
        double b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det;
        double c = 1 - a - b;
        double eps = -1e-9;
        return (a >= eps && b >= eps && c >= eps) ? new double[] {a, b, c} : null;
    }

    /** An edge on the outline belongs to exactly one triangle. */
    private static boolean outlineEdge(int a, int b) {
        int n = 0;
        for (int[] t : triangles) {
            for (int e = 0; e < 3; e++) {
                int p = t[e], q = t[(e + 1) % 3];
                if ((p == a && q == b) || (p == b && q == a)) n++;
            }
        }
        return n == 1;
    }

    /** Delaunay triangulation (Bowyer-Watson) of the spots' positions. */
    static int[][] triangulate(double[][] pts) {
        int n = pts.length;
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : pts) {
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
        }
        double d = Math.max(maxX - minX, maxY - minY) * 20 + 100, mx = (minX + maxX) / 2, my = (minY + maxY) / 2;
        final double[][] v = new double[n + 3][];
        for (int i = 0; i < n; i++) v[i] = pts[i];
        v[n] = new double[] {mx - d, my - d};
        v[n + 1] = new double[] {mx + d, my - d};
        v[n + 2] = new double[] {mx, my + d};
        List<int[]> tris = new ArrayList<>();
        tris.add(new int[] {n, n + 1, n + 2});
        for (int i = 0; i < n; i++) {
            List<int[]> bad = new ArrayList<>();
            for (int[] t : tris) if (inCircle(v, t, v[i][0], v[i][1])) bad.add(t);
            List<int[]> edges = new ArrayList<>();
            for (int[] t : bad) {
                for (int e = 0; e < 3; e++) {
                    int a = t[e], b = t[(e + 1) % 3];
                    boolean shared = false;
                    for (int[] o : bad) {
                        if (o == t) continue;
                        for (int f = 0; f < 3 && !shared; f++) {
                            int p = o[f], q = o[(f + 1) % 3];
                            shared = (p == a && q == b) || (p == b && q == a);
                        }
                        if (shared) break;
                    }
                    if (!shared) edges.add(new int[] {a, b});
                }
            }
            tris.removeAll(bad);
            for (int[] e : edges) tris.add(new int[] {e[0], e[1], i});
        }
        List<int[]> out = new ArrayList<>();
        for (int[] t : tris) if (t[0] < n && t[1] < n && t[2] < n) out.add(t);
        return out.toArray(new int[0][]);
    }

    private static boolean inCircle(double[][] v, int[] t, double px, double py) {
        double ax = v[t[0]][0] - px, ay = v[t[0]][1] - py, bx = v[t[1]][0] - px, by = v[t[1]][1] - py, cx = v[t[2]][0] - px, cy = v[t[2]][1] - py;
        double det = (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay) + (cx * cx + cy * cy) * (ax * by - bx * ay);
        double orient = (v[t[1]][0] - v[t[0]][0]) * (v[t[2]][1] - v[t[0]][1]) - (v[t[2]][0] - v[t[0]][0]) * (v[t[1]][1] - v[t[0]][1]);
        return orient > 0 ? det > 0 : det < 0;
    }
}
