import org.firstinspires.ftc.teamcode.drive.HiveRoute;
import org.firstinspires.ftc.teamcode.drive.HiveRoute.Params;
import org.firstinspires.ftc.teamcode.drive.HiveRoute.Rect;

import java.util.List;

/**
 * Checks on a PC that the legs of the two duo autos (blue frame, the DuoAutos defaults) and the Solo auto's final gather (the AutonSolo defaults) keep the robot's footprint (17.48 x 13.70 in, the heading turning evenly along each leg) off the hive's
 * support rails, and report the least clearance on each leg. Red is the same turned 180 deg, so blue is enough. Exit code 0 = no leg touches a support.
 * Run with tools/hive/run-check.cmd. The numbers below are copies of the DuoAutos defaults: change both together.
 */
public class DuoPathCheck {

    static final double DEPTH = 17.48, WIDTH = 13.70, FROM_BACK = 7.598;
    static int fails = 0;

    /** Poses are {x, y, heading deg}. */
    static void leg(String name, double[] a, double[] b, List<Rect> rects) {
        double len = Math.hypot(b[0] - a[0], b[1] - a[1]);
        double turn = ((b[2] - a[2]) % 360.0 + 540.0) % 360.0 - 180.0;
        double least = 1e9;
        int steps = Math.max(2, (int) (Math.max(len, Math.abs(turn) / 4.0) / 0.5));
        for (int i = 0; i <= steps; i++) {
            double t = i / (double) steps;
            double x = a[0] + (b[0] - a[0]) * t, y = a[1] + (b[1] - a[1]) * t, h = Math.toRadians(a[2] + turn * t);
            double c = Math.cos(h), s = Math.sin(h);
            for (double u = -FROM_BACK; u <= DEPTH - FROM_BACK + 1e-9; u += 1.0) {
                for (double v : new double[]{-WIDTH / 2.0, WIDTH / 2.0}) least = Math.min(least, clearance(x + u * c - v * s, y + u * s + v * c, rects));
            }
            for (double v = -WIDTH / 2.0; v <= WIDTH / 2.0 + 1e-9; v += 1.0) {
                for (double u : new double[]{-FROM_BACK, DEPTH - FROM_BACK}) least = Math.min(least, clearance(x + u * c - v * s, y + u * s + v * c, rects));
            }
        }
        boolean ok = least > 0.5;
        if (!ok) fails++;
        System.out.printf("%s %-46s least clearance to a support rail %5.1f in%n", ok ? "pass:" : "FAIL:", name, least);
    }

    static double clearance(double px, double py, List<Rect> rects) {
        double least = 1e9;
        for (Rect r : rects) least = Math.min(least, r.distanceTo(px, py));
        return least;
    }

    public static void main(String[] args) {
        Params p = new Params();
        p.marginIn = 0.0;
        p.underHive = false;
        List<Rect> real = HiveRoute.keepOuts(p, false);

        // Garden side (G)
        double[] gStart = {-63.4, 16.0, 0.0}, gLaunch = {-61.5, 15.25, 0.0}, wallPre = {-26.3, 38.0, 90.0}, wallAt = {-26.3, 55.3, 90.0}, wallRam = {-26.3, 67.3, 90.0}, wallBack = {-26.3, 46.3, 90.0};
        double[] gardenPre = {-59.8, 46.0, 105.0}, gardenEnd = {-59.8, 58.5, 105.0}, squarePre = {-62.0, 58.5, 90.0}, helper = {-36.0, 40.0, 90.0}, gPark = {26.0, 60.0, 90.0};
        leg("G launch spot 1 -> wall flower pre", gLaunch, wallPre, real);
        leg("G wall flower pre -> ram target", wallPre, wallRam, real);
        leg("G wall flower back-out", wallAt, wallBack, real);
        leg("G back-out -> launch spot (turning to aim)", wallBack, gLaunch, real);
        leg("G launch spot -> garden pre", gLaunch, gardenPre, real);
        leg("G garden pickup", gardenPre, gardenEnd, real);
        leg("G garden end -> square approach", gardenEnd, squarePre, real);
        leg("G square -> launch spot 2", squarePre, gLaunch, real);
        leg("G launch spot -> park helper", gLaunch, helper, real);
        leg("G park helper -> park", helper, gPark, real);

        // Flower side (F)
        double[] fLaunch = {61.5, 7.5, 172.0}, flowerPre = {38.0, 26.3, 0.0}, flowerRam = {67.3, 26.3, 0.0}, flowerBack = {46.3, 26.3, 0.0}, parkVia = {46.0, 32.0, 90.0}, fPark = {40.0, 60.0, 90.0};
        leg("F launch spot -> flower pre", new double[]{61.5, 7.5, 180.0}, flowerPre, real);
        leg("F flower pre -> ram target", flowerPre, flowerRam, real);
        leg("F back-out -> launch spot (turning to aim)", flowerBack, fLaunch, real);
        leg("F launch spot -> park via", fLaunch, parkVia, real);
        leg("F park via -> park", parkVia, fPark, real);

        // Solo final gather (2026-11-05): launch spot 2, around the outside via the helper, the Blue Wall flower, back out, the hive-side corner (the AutonSolo defaults)
        double[] sLaunch2 = {50.0, 26.3, -145.0}, sHelper = {36.0, 40.0, 180.0}, sWallPre = {-26.3, 38.0, 90.0}, sWallRam = {-26.3, 67.3, 90.0}, sWallAt = {-26.3, 55.3, 90.0}, sWallBack = {-26.3, 46.3, 90.0}, sPark = {26.0, 60.0, 90.0};
        leg("S launch spot 2 -> final helper", sLaunch2, sHelper, real);
        leg("S final helper -> wall flower pre", sHelper, sWallPre, real);
        leg("S wall flower pre -> ram target", sWallPre, sWallRam, real);
        leg("S wall flower back-out", sWallAt, sWallBack, real);
        leg("S back-out -> hive-side corner park", sWallBack, sPark, real);

        // Duo Primary (2026-11-06): the legs of AutonDuoPrimary with the DuoAutos defaults, against the supports AND the four flower stands (grown 2 in); the ram and back-out legs end at a flower on purpose, so they use the supports only
        List<Rect> withFlowers = new java.util.ArrayList<>(real);
        for (Rect f : HiveRoute.flowerKeepOuts(2.0)) withFlowers.add(f);
        double[] pStart = {-63.4, 16.0, 0.0}, pEastHelper = {36.0, 40.0, 180.0}, pAud = {50.0, 26.3, -170.0}, pRear = {-50.0, 26.3, -26.0}, pWestHelper = {-36.0, 40.0, 180.0};
        leg("P start -> wall flower pre", pStart, wallPre, withFlowers);
        leg("P wall flower pre -> ram target", wallPre, wallRam, real);
        leg("P wall flower back-out -> east helper", wallBack, pEastHelper, withFlowers);
        leg("P east helper -> audience launch spot", pEastHelper, pAud, withFlowers);
        leg("P audience launch spot -> flower pre", pAud, flowerPre, withFlowers);
        leg("P audience flower pre -> ram target", flowerPre, flowerRam, real);
        leg("P audience flower back-out -> east helper", flowerBack, pEastHelper, withFlowers);
        leg("P east helper -> west helper", pEastHelper, pWestHelper, withFlowers);
        leg("P west helper -> rear launch spot", pWestHelper, pRear, withFlowers);
        leg("P rear launch spot -> garden pre", pRear, gardenPre, withFlowers);
        leg("P garden square -> rear launch spot", squarePre, pRear, withFlowers);
        leg("P rear launch spot -> west helper", pRear, pWestHelper, withFlowers);
        leg("P west helper -> park", pWestHelper, gPark, withFlowers);

        System.out.println(fails == 0 ? "All duo and Solo legs clear of the supports." : fails + " leg(s) touch a support.");
        System.exit(fails == 0 ? 0 : 1);
    }
}
