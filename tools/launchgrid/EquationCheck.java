import org.firstinspires.ftc.teamcode.drive.LaunchEquation;
import org.firstinspires.ftc.teamcode.drive.LaunchGridData;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.LaunchMap;

import java.util.Locale;

/**
 * PC check of LaunchEquation (2026-11-18), run by tools/launchgrid/run-check.cmd after GridMapCheck: the equation against the measured spots of the 2-D launch grid's data (the blue
 * rear cell: the speed and the aim it was fitted to, each spot printed), against the seeded blue audience spots (the mirror rule is the grid's own), the red cells as the blue ones
 * turned 180 degrees, the speed curve (rising with the distance, held at the ends, near the 1-D launch map), a hand-worked aim, and an answer everywhere on the field with no jumps.
 */
public class EquationCheck {
    static int fails = 0;

    static void check(boolean ok, String what) {
        if (!ok) {
            fails++;
            System.out.println("FAIL: " + what);
        }
    }

    /** rms and worst of the equation's error over a cell's MEASURED good spots (not the estimated ones): {speed rms, speed worst, aim rms, aim worst, n}. */
    static double[] against(HiveTarget cell, double[][] spots, boolean print) {
        double ss = 0, sw = 0, as = 0, aw = 0;
        int n = 0;
        for (double[] s : spots) {
            if (s[4] < 0.5 || s[5] > 0.5) continue;
            double d = Math.hypot(cell.x - s[0], cell.y - s[1]);
            double es = LaunchEquation.speedFor(d) - s[2], ea = LaunchEquation.residualDeg(cell, s[0], s[1]) - s[3];
            if (print) System.out.println(String.format(Locale.US, "  (%6.1f, %6.1f) %5.1f in: speed %4.0f, equation %4.0f (%+3.0f) | aim residual %+5.1f, equation %+5.1f (%+4.1f)",
                    s[0], s[1], d, s[2], LaunchEquation.speedFor(d), es, s[3], s[3] + ea, ea));
            ss += es * es;
            as += ea * ea;
            sw = Math.max(sw, Math.abs(es));
            aw = Math.max(aw, Math.abs(ea));
            n++;
        }
        return new double[]{Math.sqrt(ss / n), sw, Math.sqrt(as / n), aw, n};
    }

    public static void main(String[] args) {
        // 1. the measured spots of the blue rear cell (what the equation was fitted to)
        System.out.println("BLUE_REAR measured spots: the data, the equation, (the equation minus the data)");
        double[] r = against(HiveTarget.BLUE_REAR, LaunchGridData.BLUE_REAR, true);
        System.out.println(String.format(Locale.US, "BLUE_REAR, %.0f measured spots: speed rms %.1f ticks/s, worst %.0f | aim rms %.2f deg, worst %.1f", r[4], r[0], r[1], r[2], r[3]));
        check(r[4] >= 15, "too few measured spots to check against: " + r[4]);
        check(r[0] < 20.0 && r[1] < 40.0, "the speed curve does not fit the measured spots (refit LaunchEquation.SPEED_*)");
        check(r[2] < 3.0 && r[3] < 6.5, "the aim does not fit the measured spots (refit LaunchEquation.AIM_*)");

        // 2. the seeded blue audience cell: the same mirror rule, so about the same error
        double[] a = against(HiveTarget.BLUE_AUDIENCE, LaunchGridData.BLUE_AUDIENCE, false);
        System.out.println(String.format(Locale.US, "BLUE_AUDIENCE (the grid's SEED, mirrored), %.0f spots: speed rms %.1f ticks/s, worst %.0f | aim rms %.2f deg, worst %.1f", a[4], a[0], a[1], a[2], a[3]));
        check(a[0] < 20.0 && a[2] < 3.5 && a[3] < 8.0, "the audience cell's mirror does not follow the grid's seed");

        // 3. the red cells are the blue ones turned 180 degrees
        for (double x = -66; x <= 66; x += 6) {
            for (double y = -66; y <= 66; y += 6) {
                check(Math.abs(LaunchEquation.residualDeg(HiveTarget.BLUE_REAR, x, y) - LaunchEquation.residualDeg(HiveTarget.RED_AUDIENCE, -x, -y)) < 1e-9, "red audience is not the blue rear turned at " + x + ", " + y);
                check(Math.abs(LaunchEquation.residualDeg(HiveTarget.BLUE_AUDIENCE, x, y) - LaunchEquation.residualDeg(HiveTarget.RED_REAR, -x, -y)) < 1e-9, "red rear is not the blue audience turned at " + x + ", " + y);
            }
        }

        // 4. the speed curve: rises with the distance, held at the ends, near the 1-D launch map over the map's range
        double prev = LaunchEquation.speedFor(0.0), worstMap = 0.0;
        for (double d = 0.5; d <= 140.0; d += 0.5) {
            double v = LaunchEquation.speedFor(d);
            check(v >= prev - 1e-9 && v > 1250.0 && v < 1750.0, String.format(Locale.US, "speed %.0f at %.1f in (before it %.0f)", v, d, prev));
            prev = v;
            if (d >= LaunchMap.TABLE[0][0] && d <= LaunchMap.TABLE[LaunchMap.TABLE.length - 1][0]) worstMap = Math.max(worstMap, Math.abs(v - LaunchMap.velocityFor(d)));
        }
        check(LaunchEquation.speedFor(5.0) == LaunchEquation.speedFor(LaunchEquation.DIST_MIN_IN) && LaunchEquation.speedFor(130.0) == LaunchEquation.speedFor(LaunchEquation.DIST_MAX_IN), "the speed is not held outside the fitted distances");
        System.out.println(String.format(Locale.US, "speed: %.0f at %.0f in to %.0f at %.0f in; furthest from the 1-D launch map over the map's range: %.0f ticks/s",
                LaunchEquation.speedFor(LaunchEquation.DIST_MIN_IN), LaunchEquation.DIST_MIN_IN, LaunchEquation.speedFor(LaunchEquation.DIST_MAX_IN), LaunchEquation.DIST_MAX_IN, worstMap));
        check(worstMap < 50.0, "the speed curve is " + worstMap + " ticks/s from the 1-D launch map somewhere");

        // 5. a hand-worked aim: 30 in straight out from the blue rear cell the aim point is 30 + depth away, so the residual is asin(offset / that) + bias
        double hand = Math.toDegrees(Math.asin(-LaunchEquation.LAUNCHER_OFFSET_LEFT_IN / (30.0 + LaunchEquation.AIM_DEPTH_IN))) + LaunchEquation.AIM_BIAS_DEG;
        double got = LaunchEquation.residualDeg(HiveTarget.BLUE_REAR, HiveTarget.BLUE_REAR.x - 30.0, HiveTarget.BLUE_REAR.y);
        check(Math.abs(got - hand) < 1e-6, "straight out from the rear cell: " + got + " deg, by hand " + hand);

        // 6. an answer everywhere, and no jump over the launching side of each cell (more than 15 in out from the opening face, more than 20 in from the cell)
        double worstStep = 0.0;
        for (HiveTarget c : HiveTarget.values()) {
            double out = Math.signum(c.x);
            for (double x = -70; x <= 70; x += 1.0) {
                for (double y = -70; y <= 70; y += 1.0) {
                    double res = LaunchEquation.residualDeg(c, x, y);
                    check(!Double.isNaN(res) && Math.abs(res) <= 180.0, c + " at " + x + ", " + y + ": " + res);
                    if ((x - c.x) * out > 15.0 && Math.hypot(x - c.x, y - c.y) > 20.0 && x + 1.0 <= 70) {
                        worstStep = Math.max(worstStep, Math.abs(res - LaunchEquation.residualDeg(c, x + 1.0, y)));
                        if (y + 1.0 <= 70) worstStep = Math.max(worstStep, Math.abs(res - LaunchEquation.residualDeg(c, x, y + 1.0)));
                    }
                }
            }
        }
        System.out.println(String.format(Locale.US, "aim: the most the residual changes over 1 in on the launching sides: %.2f deg", worstStep));
        check(worstStep < 2.0, "the aim jumps " + worstStep + " deg over 1 in");

        System.out.println(fails == 0 ? "\nEquationCheck: all OK" : "\nEquationCheck: " + fails + " FAILED");
        if (fails > 0) System.exit(1);
    }
}
