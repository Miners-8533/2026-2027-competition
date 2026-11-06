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
        double[][] zones = {{-61.5, 15.25}, {61.5, 7.5}, {-61.5, -7.5}, {61.5, -15.25}};   // blue rear, blue audience, red rear, red audience

        // geometry: the lane under the hive
        check(!HiveRoute.touchingSupport(10.78, 0.0, 0.0, on), "the lane at the narrowest x (10.78) on y = 0, aligned, is clear");
        boolean rotatedOk = true;
        for (int deg = 0; deg < 360; deg += 15) if (HiveRoute.touchingSupport(10.78, 0.0, Math.toRadians(deg), on)) rotatedOk = false;
        check(rotatedOk, "the lane at the narrowest x on y = 0 is clear at every heading");
        check(!HiveRoute.touchingSupport(0.0, 0.0, Math.toRadians(90), on), "the field origin, facing +Y, is clear");
        check(HiveRoute.touchingSupport(10.78, 14.0, 0.0, on), "a robot at y 14 under the narrowest point touches a leg (its top corner at y 20.85, beyond the leg face at 18.47)");
        check(HiveRoute.touchingSupport(0.0, 17.5, 0.0, on), "a robot pressed against a bar is touching");

        // routes
        Plan p1 = HiveRoute.plan(-61.5, 15.25, 0.0, 61.5, 7.5, on);
        System.out.println("rear zone -> audience zone, under-hive on: " + p1.description);
        check(p1.ok && p1.description.contains("(34, 0)") && !p1.description.contains("(36, 40)"), "rear zone to audience zone goes through the lane (the gate on the audience side) when under-hive routes are on");
        Plan p2 = HiveRoute.plan(-61.5, 15.25, 0.0, 61.5, 7.5, off);
        System.out.println("rear zone -> audience zone, under-hive off: " + p2.description);
        check(p2.ok && p2.description.contains("(-36, 40)") && p2.description.contains("(36, 40)"), "with under-hive routes off the same trip goes around the outside");
        check(p1.ok && p2.ok && p1.lengthIn < p2.lengthIn, "the lane route is shorter than the outside route (" + String.format("%.0f", p1.lengthIn) + " against " + String.format("%.0f", p2.lengthIn) + " in)");

        Plan p3 = HiveRoute.plan(0.0, 0.0, 0.0, 61.5, 7.5, on);
        check(p3.ok && p3.description.startsWith("direct"), "from the field origin to the audience zone: " + p3.description);
        Plan p4 = HiveRoute.plan(0.0, 0.0, 0.0, 61.5, 7.5, off);
        check(p4.ok, "from the field origin with under-hive routes off, the robot can still leave: " + p4.description);
        Plan p5 = HiveRoute.plan(0.0, 17.5, 0.0, 61.5, 7.5, on);
        check(!p5.ok, "a robot pressed against a bar is refused: " + p5.description);
        Plan p6 = HiveRoute.plan(30.0, 24.0, 0.0, -61.5, 15.25, on);
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
                Plan pl = HiveRoute.plan(x, y, 0.0, 61.5, 7.5, on);
                if (pl.ok) { tightRouted++; if (!routeIsSafe(pl, on)) check(false, "unsafe route from (" + x + ", " + y + ")"); }
            }
        }
        System.out.println("starts under or beside the hive that are not touching a support: " + tightRouted + " of " + tight + " have a route");
        check(tightRouted > tight / 2, "most starts under or beside the hive have a route");

        // symmetry: the same trip turned 180 degrees about the field center (blue to red) has the same length
        Plan b = HiveRoute.plan(-61.5, 15.25, 0.0, 61.5, 7.5, on), r = HiveRoute.plan(61.5, -15.25, Math.PI, -61.5, -7.5, on);
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

        System.out.println("park corner trips planned: " + parkTrips + ", with no route: " + parkNoRoute + ", unsafe: " + parkUnsafe);
        check(parkUnsafe == 0, "no planned route to a park corner touches a support");
        check(parkNoRoute * 20 < parkTrips, "at least 95 percent of free starts have a route to a park corner");

        System.out.println(checks + " checks, " + fails + " failed");
        System.exit(fails == 0 ? 0 : 1);
    }
}
