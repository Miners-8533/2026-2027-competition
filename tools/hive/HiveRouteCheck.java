import org.firstinspires.ftc.teamcode.drive.HiveRoute;
import org.firstinspires.ftc.teamcode.drive.HiveRoute.Params;
import org.firstinspires.ftc.teamcode.drive.HiveRoute.Plan;
import org.firstinspires.ftc.teamcode.drive.HiveRoute.Rect;

import java.util.List;

/**
 * Checks of the Hive Dash router on a PC (no robot, no SDK). Run with tools/hive/run-check.cmd (or the commands inside it).
 * Exit code 0 = every check passed.
 */
public class HiveRouteCheck {

    static int fails = 0, checks = 0;

    static void check(boolean ok, String what) {
        checks++;
        if (!ok) {
            fails++;
            System.out.println("FAIL: " + what);
        } else {
            System.out.println("pass: " + what);
        }
    }

    static Params params(boolean underHive) {
        Params p = new Params();
        p.underHive = underHive;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) p.points.add(new double[]{sx * 36.0, sy * 40.0});
            p.points.add(new double[]{sx * 34.0, 0.0});
        }
        return p;
    }

    /** Samples the footprint along every leg (aligned legs: front or rear leading, either; short legs: the far-corner circle) against the real supports, no margin. */
    static boolean routeIsSafe(Plan plan, Params p) {
        List<Rect> real = HiveRoute.keepOuts(p, false);
        for (int k = 0; k + 1 < plan.waypoints.length; k++) {
            double ax = plan.waypoints[k][0], ay = plan.waypoints[k][1], bx = plan.waypoints[k + 1][0], by = plan.waypoints[k + 1][1];
            double len = Math.hypot(bx - ax, by - ay), dir = Math.atan2(by - ay, bx - ax);
            for (double s = 0.0; s <= len; s += 0.25) {
                double x = ax + (bx - ax) * s / Math.max(len, 1e-9), y = ay + (by - ay) * s / Math.max(len, 1e-9);
                if (plan.aligned[k]) {
                    if (HiveRoute.touchingSupport(x, y, dir, p) || HiveRoute.touchingSupport(x, y, dir + Math.PI, p)) return false;
                } else {
                    for (Rect r : real) if (r.distanceTo(x, y) <= p.radiusIn()) return false;
                }
            }
        }
        return true;
    }

    public static void main(String[] args) {
        Params on = params(true), off = params(false);
        double[][] zones = {{-55.4, 15.25}, {55.4, 7.5}, {-55.4, -7.5}, {55.4, -15.25}};   // blue rear, blue audience, red rear, red audience

        // geometry: the lane under the hive
        check(!HiveRoute.touchingSupport(10.78, 0.0, 0.0, on), "the lane at the narrowest x (10.78) on y = 0, aligned, is clear");
        boolean rotatedOk = true;
        for (int deg = 0; deg < 360; deg += 15) if (HiveRoute.touchingSupport(10.78, 0.0, Math.toRadians(deg), on)) rotatedOk = false;
        check(rotatedOk, "the lane at the narrowest x on y = 0 is clear at every heading");
        check(!HiveRoute.touchingSupport(0.0, 0.0, Math.toRadians(90), on), "the field origin, facing +Y, is clear");
        check(HiveRoute.touchingSupport(10.78, 14.0, 0.0, on), "a robot at y 14 under the narrowest point touches a leg (its top corner at y 20.85, beyond the leg face at 18.47)");
        check(HiveRoute.touchingSupport(0.0, 17.5, 0.0, on), "a robot pressed against a bar is touching");

        // routes
        Plan p1 = HiveRoute.plan(-55.4, 15.25, 0.0, 55.4, 7.5, on);
        System.out.println("rear zone -> audience zone, under-hive on: " + p1.description);
        check(p1.ok && p1.description.contains("(34, 0)") && !p1.description.contains("(36, 40)"), "rear zone to audience zone goes through the lane (the gate on the audience side) when under-hive routes are on");
        Plan p2 = HiveRoute.plan(-55.4, 15.25, 0.0, 55.4, 7.5, off);
        System.out.println("rear zone -> audience zone, under-hive off: " + p2.description);
        check(p2.ok && p2.description.contains("(-36, 40)") && p2.description.contains("(36, 40)"), "with under-hive routes off the same trip goes around the outside");
        check(p1.ok && p2.ok && p1.lengthIn < p2.lengthIn, "the lane route is shorter than the outside route (" + String.format("%.0f", p1.lengthIn) + " against " + String.format("%.0f", p2.lengthIn) + " in)");

        Plan p3 = HiveRoute.plan(0.0, 0.0, 0.0, 55.4, 7.5, on);
        check(p3.ok && p3.description.startsWith("direct"), "from the field origin to the audience zone: " + p3.description);
        Plan p4 = HiveRoute.plan(0.0, 0.0, 0.0, 55.4, 7.5, off);
        check(p4.ok, "from the field origin with under-hive routes off, the robot can still leave: " + p4.description);
        Plan p5 = HiveRoute.plan(0.0, 17.5, 0.0, 55.4, 7.5, on);
        check(!p5.ok, "a robot pressed against a bar is refused: " + p5.description);
        Plan p6 = HiveRoute.plan(30.0, 24.0, 0.0, -55.4, 15.25, on);
        check(p6.ok, "from beside the bar end (30, 24) to the rear zone: " + p6.description);

        // every start to every zone, both settings: a route exists and is safe
        int total = 0, bad = 0;
        double[][] starts = {{-36, 40}, {36, 40}, {-36, -40}, {36, -40}, {-34, 0}, {34, 0}, {0, 0}, {0, 8}, {-60, 60}, {60, 60}, {-60, -60}, {60, -60}, {0, 50}, {0, -50}, {45, 20}, {-45, -20}};
        for (boolean under : new boolean[]{true, false}) {
            Params p = under ? on : off;
            for (double[] s : starts) {
                for (double[] g : zones) {
                    for (double h = 0.0; h < 360.0; h += 90.0) {
                        Plan pl = HiveRoute.plan(s[0], s[1], Math.toRadians(h), g[0], g[1], p);
                        total++;
                        if (!pl.ok) {
                            System.out.println("  no route: start (" + s[0] + ", " + s[1] + ") heading " + h + " -> (" + g[0] + ", " + g[1] + ") under=" + under + ": " + pl.refusal);
                            bad++;
                        } else if (!routeIsSafe(pl, p)) {
                            System.out.println("  UNSAFE route: start (" + s[0] + ", " + s[1] + ") -> (" + g[0] + ", " + g[1] + ") " + pl.description);
                            bad++;
                        }
                    }
                }
            }
        }
        check(bad == 0, "all " + total + " start / goal / heading / setting combinations have a safe route (" + bad + " problems)");

        // a start beside the legs under the hive may be refused, but never crashes and never routes through a support
        int tight = 0, tightRouted = 0;
        for (double x = -30.0; x <= 30.0; x += 5.0) {
            for (double y = -22.0; y <= 22.0; y += 2.0) {
                if (HiveRoute.touchingSupport(x, y, 0.0, on)) continue;
                tight++;
                Plan pl = HiveRoute.plan(x, y, 0.0, 55.4, 7.5, on);
                if (pl.ok) { tightRouted++; if (!routeIsSafe(pl, on)) check(false, "unsafe route from (" + x + ", " + y + ")"); }
            }
        }
        System.out.println("starts under or beside the hive that are not touching a support: " + tightRouted + " of " + tight + " have a route");
        check(tightRouted > tight / 2, "most starts under or beside the hive have a route");

        // symmetry: the same trip turned 180 degrees about the field center (blue to red) has the same length
        Plan b = HiveRoute.plan(-55.4, 15.25, 0.0, 55.4, 7.5, on), r = HiveRoute.plan(55.4, -15.25, Math.PI, -55.4, -7.5, on);
        check(b.ok && r.ok && Math.abs(b.lengthIn - r.lengthIn) < 1e-6, "blue and red (turned 180 deg) plan the same length");

        // a lower robot never has less room than an 18 in one
        Params low = params(true);
        low.robotHeightIn = 10.0;
        boolean lowOk = true;
        for (double y = 0.0; y <= 20.0; y += 0.5) if (HiveRoute.touchingSupport(10.78, y, 0.0, low) && !HiveRoute.touchingSupport(10.78, y, 0.0, on)) lowOk = false;
        check(lowOk, "a 10 in robot never touches where an 18 in robot does not");

        // Park Dash: from every free start on a 6 in grid, a safe route to each of the four park corners (blue and red, ParkDash.LEFT_X 40 / RIGHT_X 26, y 60)
        double[][] corners = {{40.0, 60.0}, {26.0, 60.0}, {-40.0, -60.0}, {-26.0, -60.0}};
        int parkTrips = 0, parkNoRoute = 0, parkUnsafe = 0;
        for (double sx = -66.0; sx <= 66.0; sx += 6.0) {
            for (double sy = -66.0; sy <= 66.0; sy += 6.0) {
                if (HiveRoute.touchingSupport(sx, sy, 0.0, on) || HiveRoute.touchingSupport(sx, sy, Math.PI / 2.0, on)) continue;
                for (double[] g : corners) {
                    Plan pl = HiveRoute.plan(sx, sy, 0.0, g[0], g[1], on);
                    parkTrips++;
                    if (!pl.ok) parkNoRoute++;
                    else if (!routeIsSafe(pl, on)) parkUnsafe++;
                }
            }
        }
        // the Park Dash of 2026-11-05 20:05: from (25.0, 37.5), 22 in from the right corner, it planned 126 in via (-36, 40) because a short leg's box reached back over the support bar
        Plan logged = HiveRoute.plan(25.0, 37.5, Math.toRadians(-1.6), 26.0, 60.0, on);
        check(logged.ok && logged.waypoints.length == 2, "a start 22 in from the park corner with the bar behind it plans a direct leg, not a helper-point detour (" + logged.description + ")");

        // lane centring (2026-11-05): a route that goes under the hive keeps the robot's center near the lane's middle line while it is between the bars. The first leg of a start that is already
        // under the hive is exempt. Limit: laneHalfWidth - (half width + margin) = 12.35 - 8.35 = 4 in, plus 1 in for the sampling.
        double laneLimit = on.laneHalfWidthIn - (on.widthIn / 2.0 + on.marginIn) + 1.0;
        int laneTrips = 0, laneOff = 0;
        double worstOffset = 0.0;
        for (double sx = -66.0; sx <= 66.0; sx += 6.0) {
            for (double sy = -66.0; sy <= 66.0; sy += 6.0) {
                if (HiveRoute.touchingSupport(sx, sy, 0.0, on) || HiveRoute.touchingSupport(sx, sy, Math.PI / 2.0, on)) continue;
                boolean startUnder = Math.abs(sx) <= HiveRoute.BAR_HALF_LENGTH_IN && Math.abs(sy) < HiveRoute.BAR_INNER_Y_IN;
                double[][] goals = {{-55.4, 15.25}, {55.4, 7.5}, {-55.4, -7.5}, {55.4, -15.25}, {40.0, 60.0}, {26.0, 60.0}, {-40.0, -60.0}, {-26.0, -60.0}};
                for (double[] g : goals) {
                    Plan pl = HiveRoute.plan(sx, sy, 0.0, g[0], g[1], on);
                    if (!pl.ok) continue;
                    double offset = 0.0;
                    for (int k = (startUnder ? 1 : 0); k + 1 < pl.waypoints.length; k++) {
                        double ax = pl.waypoints[k][0], ay = pl.waypoints[k][1], bx = pl.waypoints[k + 1][0], by = pl.waypoints[k + 1][1];
                        double len = Math.hypot(bx - ax, by - ay);
                        for (double u = 0.0; u <= len; u += 0.5) {
                            double x = ax + (bx - ax) * u / Math.max(len, 1e-9), y = ay + (by - ay) * u / Math.max(len, 1e-9);
                            if (Math.abs(x) <= HiveRoute.BAR_HALF_LENGTH_IN && Math.abs(y) < HiveRoute.BAR_INNER_Y_IN) offset = Math.max(offset, Math.abs(y));   // inside the lane (between the bars)
                        }
                    }
                    laneTrips++;
                    worstOffset = Math.max(worstOffset, offset);
                    if (offset > laneLimit) laneOff++;
                }
            }
        }
        System.out.println("lane centring: " + laneTrips + " trips, the largest distance of a route from the lane's middle line between the bars: " + String.format("%.1f", worstOffset) + " in (limit " + String.format("%.1f", laneLimit) + "), " + laneOff + " over");
        check(laneOff == 0, "every route that goes under the hive stays within " + String.format("%.0f", laneLimit) + " in of the lane's middle line between the bars");
        Plan loggedLane = HiveRoute.plan(58.2, -41.0, Math.toRadians(-94.5), -55.4, 15.3, on);
        check(loggedLane.ok && !loggedLane.description.equals("via (34, 0), 144 in") && (loggedLane.description.contains("(-34, 0)") || !loggedLane.description.contains("(34, 0)")),
                "the 2026-11-05 dash that hit a support (from (58.2, -41.0) to the blue rear zone via the near gate only) no longer takes the diagonal under the hive (" + loggedLane.description + ")");

        System.out.println("park corner trips planned: " + parkTrips + ", with no route: " + parkNoRoute + ", unsafe: " + parkUnsafe);
        check(parkUnsafe == 0, "no planned route to a park corner touches a support");
        check(parkNoRoute * 20 < parkTrips, "at least 95 percent of free starts have a route to a park corner");

        // flowers (2026-11-06): no planned route touches a flower stand, from any free start that is not itself next to one
        List<Rect> flowers = HiveRoute.flowerKeepOuts();
        double[][] allGoals = {{-55.4, 15.25}, {55.4, 7.5}, {-55.4, -7.5}, {55.4, -15.25}, {40.0, 60.0}, {26.0, 60.0}, {-40.0, -60.0}, {-26.0, -60.0}};
        int flowerTrips = 0, flowerTouch = 0;
        double minClear = 1e9;
        for (double sx = -66.0; sx <= 66.0; sx += 6.0) {
            for (double sy = -66.0; sy <= 66.0; sy += 6.0) {
                if (HiveRoute.touchingSupport(sx, sy, 0.0, on) || HiveRoute.touchingSupport(sx, sy, Math.PI / 2.0, on)) continue;
                boolean nextToFlower = false;
                for (Rect f : flowers) if (f.distanceTo(sx, sy) <= on.radiusIn() + on.marginIn + 2.0) nextToFlower = true;
                if (nextToFlower) continue;
                for (double[] g : allGoals) {
                    Plan pl = HiveRoute.plan(sx, sy, 0.0, g[0], g[1], on);
                    if (!pl.ok) continue;
                    flowerTrips++;
                    boolean touched = false;
                    for (int k = 0; k + 1 < pl.waypoints.length && !touched; k++) {
                        double ax = pl.waypoints[k][0], ay = pl.waypoints[k][1], bx = pl.waypoints[k + 1][0], by = pl.waypoints[k + 1][1];
                        double len = Math.hypot(bx - ax, by - ay), dir = Math.atan2(by - ay, bx - ax);
                        for (double u = 0.0; u <= len && !touched; u += 0.5) {
                            double x = ax + (bx - ax) * u / Math.max(len, 1e-9), y = ay + (by - ay) * u / Math.max(len, 1e-9);
                            for (Rect f : flowers) {
                                if (pl.aligned[k]) {
                                    if (HiveRoute.overlap(HiveRoute.poseRect(x, y, dir, on, 0.0), f) || HiveRoute.overlap(HiveRoute.poseRect(x, y, dir + Math.PI, on, 0.0), f)) touched = true;
                                } else if (f.distanceTo(x, y) <= on.radiusIn()) {
                                    touched = true;
                                }
                                if (pl.aligned[k]) minClear = Math.min(minClear, f.distanceTo(x, y) - Math.max(on.frontIn(), on.fromBackIn));
                            }
                        }
                    }
                    if (touched) flowerTouch++;
                }
            }
        }
        System.out.println("flower check: " + flowerTrips + " trips from free starts not next to a flower, " + flowerTouch + " touch one");
        check(flowerTouch == 0, "no planned route to a launch spot or a park corner touches a flower stand");
        // the 2026-11-06 dashes to the rear cell that passed 7.8 to 9.9 in from the rear flower's center (from the log): they must clear it now
        double[][] loggedStarts = {{47.7, -58.4, -87.8}, {43.0, -48.3, 4.7}, {-61.3, -57.1, -128.0}, {-66.0, -57.7, -106.6}, {-61.7, -59.1, -92.7}};
        for (double[] l : loggedStarts) {
            Plan pl = HiveRoute.plan(l[0], l[1], Math.toRadians(l[2]), -55.4, 15.25, on);
            double worst = 1e9;
            if (pl.ok) {
                for (int k = 0; k + 1 < pl.waypoints.length; k++) {
                    double ax = pl.waypoints[k][0], ay = pl.waypoints[k][1], bx = pl.waypoints[k + 1][0], by = pl.waypoints[k + 1][1];
                    double len = Math.hypot(bx - ax, by - ay);
                    for (double u = 0.0; u <= len; u += 0.5) {
                        double x = ax + (bx - ax) * u / Math.max(len, 1e-9), y = ay + (by - ay) * u / Math.max(len, 1e-9);
                        if (k == 0 && Math.hypot(x - l[0], y - l[1]) < 3.0) continue;
                        worst = Math.min(worst, flowers.get(2).distanceTo(x, y));
                    }
                }
            }
            System.out.println(String.format("logged start (%.1f, %.1f): %s, the center comes within %.1f in of the rear flower stand (robot radius %.1f)", l[0], l[1], pl.ok ? pl.description : pl.refusal, worst, on.radiusIn()));
            check(pl.ok && worst > on.radiusIn(), "the logged dash from (" + l[0] + ", " + l[1] + ") to the rear cell clears the rear flower by more than the robot's radius");
        }
        // a robot that has just gathered from a flower stands next to it: it must still get a route to the cell on its side
        double[][] atFlower = {{57.0, 24.0, 0.0, 55.4, 7.5}, {-57.0, -24.0, Math.PI, -55.4, -7.5}, {57.0, 24.0, 0.0, -55.4, 15.25}};
        for (double[] a : atFlower) {
            Plan pl = HiveRoute.plan(a[0], a[1], a[2], a[3], a[4], on);
            check(pl.ok, "a start next to a flower stand (" + a[0] + ", " + a[1] + ") has a route to (" + a[3] + ", " + a[4] + "): " + (pl.ok ? pl.description : pl.refusal));
        }

        System.out.println(checks + " checks, " + fails + " failed");
        System.exit(fails == 0 ? 0 : 1);
    }
}
