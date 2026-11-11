import org.firstinspires.ftc.teamcode.drive.HiveRoute;
import org.firstinspires.ftc.teamcode.drive.LaunchGrid;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

import java.util.List;
import java.util.Locale;

/**
 * PC check of LaunchGrid (the Launch Grid Tuner's spots), run by tools/hive/run-check.cmd. Prints, for the near (red audience) and far (red rear) cells, the
 * spots in visiting order and whether the router (HiveRoute.plan) can route every hop of the sweep, starting from the usual start pose.
 * Fails (exit 1) when a grid is empty, a spot violates the rules, or a hop between consecutive spots cannot be routed.
 */
public class LaunchGridCheck {

    static final double LAUNCHER_OFFSET_LEFT_IN = -2.5, FAR_EXTRA_LEFT_IN = -3.0;   // TargetLock's values at 2026-11-10

    static HiveRoute.Params params() {
        HiveRoute.Params p = new HiveRoute.Params();
        p.robotHeightIn = 18.0;
        p.marginIn = 1.5;
        p.flowerClearanceIn = 3.0;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) p.points.add(new double[]{sx * 36.0, sy * 40.0});
            p.points.add(new double[]{sx * 34.0, 0.0});
        }
        return p;
    }

    static int fails = 0;

    static void check(boolean ok, String what) {
        if (!ok) {
            fails++;
            System.out.println("FAIL: " + what);
        }
    }

    public static void main(String[] args) {
        double pitch = args.length > 0 ? Double.parseDouble(args[0]) : 12.0;
        HiveRoute.Params p = params();
        double startX = 63.4, startY = -16.5, startH = Math.PI;
        for (HiveTarget cell : new HiveTarget[]{HiveTarget.RED_AUDIENCE, HiveTarget.RED_REAR}) {
            double extra = cell.x < 0 ? FAR_EXTRA_LEFT_IN : 0.0;   // red: the rear cell is the far one
            LaunchGrid.Aim aim = (x, y) -> {
                double dx = cell.x - x, dy = cell.y - y;
                double par = Math.asin(Math.max(-0.9, Math.min(0.9, (LAUNCHER_OFFSET_LEFT_IN + extra) / Math.hypot(dx, dy))));
                return Math.atan2(dy, dx) - par;
            };
            List<LaunchGrid.Point> pts = LaunchGrid.generate(cell, pitch, 75.0, 12.0, 71.0, aim, p);
            LaunchGrid.order(pts, startX, startY);
            System.out.printf(Locale.US, "%n== %s (open toward %s), pitch %.0f in: %d spots%n", cell, LaunchGrid.openSide(cell) > 0 ? "audience" : "rear", pitch, pts.size());
            check(!pts.isEmpty(), cell + " grid is empty");
            double px = startX, py = startY, ph = startH;
            int refused = 0, direct = 0;
            double total = 0;
            int i = 0;
            for (LaunchGrid.Point q : pts) {
                // rules
                check(Math.hypot(q.x - cell.x, q.y - cell.y) >= 12.0, "too close " + q.x + "," + q.y);
                check((q.x - cell.x) * LaunchGrid.openSide(cell) > 0, "wrong side " + q.x + "," + q.y);
                check(Math.max(Math.abs(q.x), Math.abs(q.y)) <= 71.0, "outside the walls " + q.x + "," + q.y);
                HiveRoute.Plan plan = HiveRoute.plan(px, py, ph, q.x, q.y, p);
                String hop;
                if (plan.ok) {
                    hop = String.format(Locale.US, "%.0f in, %d leg(s)", plan.lengthIn, plan.waypoints.length - 1);
                    total += plan.lengthIn;
                } else if (LaunchGrid.straightHopClear(px, py, ph, q.x, q.y, q.headingRad, 71.0, p)) {
                    double d = Math.hypot(q.x - px, q.y - py);
                    hop = String.format(Locale.US, "router refused (%s); a straight hop of %.0f in is clear", plan.refusal, d);
                    direct++;
                    total += d;
                } else {
                    hop = "BLOCKED: " + plan.refusal + " and the straight hop is not clear either";
                    refused++;
                }
                System.out.printf(Locale.US, "%2d (%6.1f, %6.1f) heading %6.1f deg, %5.1f in from the cell%s   hop from (%.0f, %.0f): %s%n", ++i, q.x, q.y,
                        Math.toDegrees(q.headingRad), q.distanceIn, q.slid ? "  [slid in]" : "", px, py, hop);
                px = q.x;
                py = q.y;
                ph = q.headingRad;
            }
            System.out.printf(Locale.US, "hops: %d routed by the router, %d by a clear straight hop (router refused), %d blocked; total length %.0f in%n", pts.size() - direct - refused, direct, refused, total);
            check(refused == 0, cell + ": " + refused + " hops can be neither routed nor driven straight");
        }
        System.out.println(fails == 0 ? "\nLaunchGridCheck: all OK" : "\nLaunchGridCheck: " + fails + " FAILED");
        if (fails > 0) System.exit(1);
    }
}
