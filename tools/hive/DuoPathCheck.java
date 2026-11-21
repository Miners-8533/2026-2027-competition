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
        double[] gardenPre = {-59.8, 46.0, 105.0}, gardenEnd = {-59.8, 58.5, 105.0}, squarePre = {-62.0, 58.5, 90.0}, helper = {-36.0, 40.0, 90.0}, gPark = {26.0, 53.1, 90.0};
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
        // 2026-11-21: the flower salvo from the Solo launch spot (56, 36) with the grid's aim (-142.1 deg), the park (40, 55.1) straight from it (it was (61.5, 7.5, 172) and the via point (46, 32))
        double[] fLaunch = {56.0, 36.0, -142.1}, flowerPre = {38.0, 26.3, 0.0}, flowerRam = {67.3, 26.3, 0.0}, flowerBack = {46.3, 26.3, 0.0}, fPark = {40.0, 55.1, 90.0};
        leg("F start -> flower pre", new double[]{63.4, 7.5, 180.0}, flowerPre, real);
        leg("F flower pre -> ram target", flowerPre, flowerRam, real);
        leg("F back-out -> launch spot (turning to aim)", flowerBack, fLaunch, real);
        leg("F launch spot -> park", fLaunch, fPark, real);

        // Solo final gather (2026-11-05): launch spot 2, around the outside via the helper, the Blue Wall flower, back out, the hive-side corner (the AutonSolo defaults)
        double[] sLaunch2 = {50.0, 26.3, -152.5}, sHelper = {36.0, 40.0, 180.0}, sWallPre = {-26.3, 38.0, 90.0}, sWallRam = {-26.3, 67.3, 90.0}, sWallAt = {-26.3, 55.3, 90.0}, sWallBack = {-26.3, 46.3, 90.0}, sPark = {26.0, 60.0, 90.0};
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

        // Duo Daryl (2026-11-06): the garden (the solo auto's from 2026-11-07, the one-leg version before), then around to the audience spot, the Blue Wall flower, back, the audience flower and the park
        double[] dSquare = {-61.15, 58.5, 90.0}, dEastHelper = {36.0, 40.0, 180.0};
        double[] dGardenPre = {-59.8, 46.0, 105.0}, dGardenEnd = {-59.8, 58.5, 105.0}, dSquarePre = {-62.0, 58.5, 90.0};   // the solo auto garden (from 2026-11-07)
        leg("D start -> garden pre-position", pStart, dGardenPre, withFlowers);
        leg("D garden pickup", dGardenPre, dGardenEnd, withFlowers);
        leg("D garden end -> square approach", dGardenEnd, dSquarePre, withFlowers);
        leg("D squared on the wall -> east helper (turned 90 -> 180 in the first 25 in)", new double[]{-40.0, 56.0, 180.0}, dEastHelper, withFlowers);
        leg("D squared on the wall -> turned, first 25 in (supports only)", new double[]{-64.0, 60.5, 90.0}, new double[]{-40.0, 56.0, 180.0}, real);
        leg("D east helper -> audience launch spot", dEastHelper, pAud, withFlowers);
        leg("D audience launch spot -> east helper", pAud, dEastHelper, withFlowers);
        leg("D east helper -> wall flower pre", dEastHelper, wallPre, withFlowers);
        // 2026-11-21: the finish is the Blue Wall flower's pollen into the original cell from the rear launch spot (-55.6, 26.2), then the park (22, 55.1, 30 deg) through (-20, 44); the second audience salvo and the audience flower are gone
        double[] dRear = {-55.6, 26.2, -11.7}, dParkVia = {-20.0, 44.0, 30.0}, dPark = {22.0, 55.1, 30.0};
        leg("D wall flower back-out -> rear launch spot", new double[]{-26.3, 46.3, 90.0}, dRear, withFlowers);
        leg("D rear launch spot -> park via", dRear, dParkVia, withFlowers);
        leg("D park via -> park", dParkVia, dPark, withFlowers);
        // Cybugs (2026-11-07): the Blue Wall flower, the rear launch spot at the start wall, then the park through the helper
        double[] cRear = {-61.5, 15.25, -4.0};
        leg("C wall flower back-out -> rear launch spot", new double[]{-26.3, 46.3, 90.0}, cRear, withFlowers);
        leg("C rear launch spot -> park helper", cRear, helper, withFlowers);
        leg("C park helper -> park", helper, gPark, withFlowers);

        // Duo Garden Side from 2026-11-21: the start straight to the Blue Wall flower, the Triple Tip's rear launch spot, the Solo's garden and back, the park through the via point (the "G" legs above are the old path)
        double[] g2Rear = {-55.6, 26.2, -11.7}, g2GardenEnd = {-59.8, 58.5, 105.0}, g2Square = {-64.15, 61.1, 90.0}, g2Via = {-20.0, 44.0, 30.0}, g2Park = {22.0, 55.1, 30.0};
        leg("G2 start -> wall flower pre", gStart, wallPre, withFlowers);
        leg("G2 wall flower back-out -> rear launch spot", wallBack, g2Rear, withFlowers);
        leg("G2 rear launch spot -> garden end", g2Rear, g2GardenEnd, withFlowers);
        leg("G2 squared on the wall -> rear launch spot", g2Square, g2Rear, withFlowers);
        leg("G2 rear launch spot -> park via", g2Rear, g2Via, withFlowers);
        leg("G2 park via -> park", g2Via, g2Park, withFlowers);

        // Blaser's Gambit (2026-11-21): the audience-wall start, the audience flower, the Solo launch spot, the Blue Wall flower, the rear launch spot, the garden and back, the park through the via point
        double[] bStart = {63.4, 7.5, 180.0}, bAud = {56.0, 36.0, -142.1}, bRear = {-55.6, 26.2, -11.7}, bGardenEnd = {-59.8, 58.5, 105.0}, bSquare = {-64.15, 61.1, 90.0}, bVia = {-20.0, 44.0, 30.0}, bPark = {22.0, 55.1, 30.0};
        leg("B start -> flower pre", bStart, flowerPre, real);
        leg("B flower back-out -> audience launch spot", flowerBack, bAud, real);
        leg("B audience launch spot -> east helper", bAud, dEastHelper, withFlowers);
        leg("B east helper -> wall flower pre", dEastHelper, wallPre, withFlowers);
        leg("B wall flower back-out -> rear launch spot", wallBack, bRear, withFlowers);
        leg("B rear launch spot -> garden end", bRear, bGardenEnd, withFlowers);
        leg("B squared on the wall -> rear launch spot", bSquare, bRear, withFlowers);
        leg("B rear launch spot -> park via", bRear, bVia, withFlowers);
        leg("B park via -> park", bVia, bPark, withFlowers);

        System.out.println(fails == 0 ? "All duo and Solo legs clear of the supports." : fails + " leg(s) touch a support.");
        System.exit(fails == 0 ? 0 : 1);
    }
}
