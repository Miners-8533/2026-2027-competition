import org.firstinspires.ftc.teamcode.drive.LaunchGridData;
import org.firstinspires.ftc.teamcode.drive.LaunchGridMap;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

import java.util.Locale;

/**
 * PC check of LaunchGridMap (branch launch-grid-2d), run by tools/launchgrid/run-check.cmd: every spot (measured or estimated) gives back its own speed and residual, the hive side of the
 * recorded spots is a NO_SHOT zone that still gives a speed and an aim, outside the spots the nearest edge is held, EVERY point of the field gets a speed and an aim (the 2-D method never
 * falls back to the formula), the red cells are the blue data turned 180 degrees, the seeded cell says so; prints a map of the zones.
 */
public class GridMapCheck {
    static int fails = 0;

    static void check(boolean ok, String what) {
        if (!ok) {
            fails++;
            System.out.println("FAIL: " + what);
        }
    }

    static String sym(LaunchGridMap.Result r) {
        switch (r.zone) {
            case INTERPOLATED: return "g";
            case EXTRAPOLATED: return "e";
            case NO_SHOT: return "X";
            default: return "?";
        }
    }

    public static void main(String[] args) {
        // 1. every recorded spot of both measured cells gives itself back (2026-11-18: the far cell is measured too)
        for (HiveTarget cellUnderTest : new HiveTarget[]{HiveTarget.BLUE_REAR, HiveTarget.BLUE_AUDIENCE}) {
            for (double[] s : cellUnderTest == HiveTarget.BLUE_REAR ? LaunchGridData.BLUE_REAR : LaunchGridData.BLUE_AUDIENCE) {
                LaunchGridMap.Result r = LaunchGridMap.lookup(cellUnderTest, s[0], s[1]);
                if (s[4] > 0.5) {
                    check(r.hasValues() && Math.abs(r.speed - s[2]) < 1.0 && Math.abs(r.residualDeg - s[3]) < 0.05, String.format(Locale.US, "%s spot (%.1f, %.1f) gave %s", cellUnderTest, s[0], s[1], r.describe()));
                } else {
                    check(r.zone == LaunchGridMap.Zone.NO_SHOT && r.hasValues(), String.format(Locale.US, "%s no-shot spot (%.1f, %.1f) gave %s", cellUnderTest, s[0], s[1], r.describe()));
                }
            }
        }
        // 2. zones
        LaunchGridMap.Result inner = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, -29.0, 12.0);
        check(inner.zone == LaunchGridMap.Zone.NO_SHOT && inner.hasValues(), "between the no-shot column and the hive should be NO_SHOT with values: " + inner.describe());
        LaunchGridMap.Result wall = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, -63.0, 24.0);
        check(wall.zone == LaunchGridMap.Zone.EXTRAPOLATED && wall.hasValues() && wall.estimated, "toward the wall should be EXTRAPOLATED with values: " + wall.describe());
        LaunchGridMap.Result far = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, -68.0, 24.0);
        check(far.hasValues() && far.zone == LaunchGridMap.Zone.EXTRAPOLATED, "at the wall: " + far.describe());
        LaunchGridMap.Result nowhere = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, 40.0, -30.0);
        check(nowhere.hasValues(), "the other half of the field still gets an answer (no fallback): " + nowhere.describe());
        // a point between two wall spots has values (2026-11-18: every spot of the blue rear cell was measured by then, so none is estimated any more; an estimated spot would be flagged)
        LaunchGridMap.Result est = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, -58.0, -42.0);
        check(est.hasValues(), "between the wall spots: " + est.describe());
        // no point of the field has no answer, for any cell
        for (HiveTarget c : HiveTarget.values()) {
            for (double x = -70; x <= 70; x += 5) {
                for (double y = -70; y <= 70; y += 5) {
                    LaunchGridMap.Result r = LaunchGridMap.lookup(c, x, y);
                    check(r.hasValues() && !Double.isInfinite(r.speed), String.format(Locale.US, "%s at (%.0f, %.0f): %s", c, x, y, r.describe()));
                }
            }
        }
        // 3. the red cells are the blue data turned 180 degrees
        LaunchGridMap.Result a = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, -48.0, 0.0), b = LaunchGridMap.lookup(HiveTarget.RED_AUDIENCE, 48.0, 0.0);
        check(a.hasValues() && b.hasValues() && Math.abs(a.speed - b.speed) < 1e-9 && Math.abs(a.residualDeg - b.residualDeg) < 1e-9 && b.rotated, "red audience is the blue rear turned: " + b.describe());
        LaunchGridMap.Result a2 = LaunchGridMap.lookup(HiveTarget.BLUE_AUDIENCE, 48.0, -24.0), b2 = LaunchGridMap.lookup(HiveTarget.RED_REAR, -48.0, 24.0);
        check(a2.hasValues() && b2.hasValues() && Math.abs(a2.speed - b2.speed) < 1e-9 && Math.abs(a2.residualDeg - b2.residualDeg) < 1e-9 && b2.rotated, "red rear is the blue audience turned: " + b2.describe());
        // 4. the far cell is measured now (it was a seed until 2026-11-18)
        LaunchGridMap.Result measured = LaunchGridMap.lookup(HiveTarget.BLUE_AUDIENCE, 48.0, -24.0);
        check(!measured.seeded && measured.hasValues(), "the blue audience cell is measured, not seeded: " + measured.describe());
        // 5. continuity: a small step changes the speed by little
        double worst = 0.0;
        for (double x = -58.0; x <= -40.0; x += 0.5) {
            LaunchGridMap.Result p = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, x, 10.0), q = LaunchGridMap.lookup(HiveTarget.BLUE_REAR, x + 0.5, 10.0);
            // the no-shot zone is left out (2026-11-18): its speed is the nearest spot's, a step between spots, and no shot is expected there
            if (p.hasValues() && q.hasValues() && p.zone != LaunchGridMap.Zone.NO_SHOT && q.zone != LaunchGridMap.Zone.NO_SHOT) worst = Math.max(worst, Math.abs(p.speed - q.speed));
        }
        check(worst < 30.0, "speed jumps " + worst + " ticks/s between 0.5 in steps");

        // the zone maps (every 4 in): g interpolated, e extrapolated, X no shot, . no data; y down the page from +64
        for (HiveTarget cell : new HiveTarget[]{HiveTarget.BLUE_REAR, HiveTarget.BLUE_AUDIENCE}) {
            System.out.println("\n" + cell + (cell == HiveTarget.BLUE_AUDIENCE && LaunchGridData.BLUE_AUDIENCE_SEEDED ? " (SEEDED)" : "") + ": zones, x from -68 to 68 step 4 across, y from 64 to -64 down (g interpolated, e extrapolated or held from the edge, X no shot zone)");
            for (double y = 64; y >= -64; y -= 4) {
                StringBuilder sb = new StringBuilder(String.format("%4.0f ", y));
                for (double x = -68; x <= 68; x += 4) sb.append(sym(LaunchGridMap.lookup(cell, x, y)));
                System.out.println(sb);
            }
        }
        System.out.println(fails == 0 ? "\nGridMapCheck: all OK" : "\nGridMapCheck: " + fails + " FAILED");
        if (fails > 0) System.exit(1);
    }
}
