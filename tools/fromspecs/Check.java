import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.aiming.LaunchGrid;
import org.firstinspires.ftc.teamcode.aiming.TargetLock;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.navigation.HiveDash;
import org.firstinspires.ftc.teamcode.navigation.HoldPosition;
import org.firstinspires.ftc.teamcode.navigation.ParkDash;
import org.firstinspires.ftc.teamcode.navigation.Plan;
import org.firstinspires.ftc.teamcode.navigation.RoutePlanner;
import org.firstinspires.ftc.teamcode.robot.Footprint;
import org.firstinspires.ftc.teamcode.subsystems.EntryRule;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class Check {
    static Footprint fp = Footprint.ROBOT_1;

    public static void main(String[] a) throws Exception {
        System.out.println("== launch spots (specs: blue rear (-55.4,15.25), blue audience worked out 7.25 (stored 7.5), red audience (55.4,-15.25), red rear -7.25)");
        for (Cell c : Cell.values()) {
            System.out.println("  " + c + " " + Aim.launchSpot(c) + " far=" + c.isFar() + String.format(" %.1f in from its cell", Aim.launchSpot(c).dist(c.point())));
        }
        System.out.println("== auto trim conversion at (50,26.3), audience cell: old = extra -3.0 in and -8.0 deg; new = no extra and -2.5 deg");
        double old = Aim.geometry(50, 26.3, Cell.BLUE_AUDIENCE, -3.0) + Math.toRadians(-8.0), neu = Aim.forAuto(50, 26.3, Cell.BLUE_AUDIENCE, -2.5);
        System.out.printf("  old %.3f deg, new %.3f deg, difference %.3f deg%n", Math.toDegrees(old), Math.toDegrees(neu), Math.toDegrees(Angle.diff(old, neu)));
        System.out.printf("  the far-cell extra alone turns %.2f deg%n",
                Math.toDegrees(Angle.diff(Aim.geometry(50, 26.3, Cell.BLUE_AUDIENCE, -3.0), Aim.geometry(50, 26.3, Cell.BLUE_AUDIENCE, 0))));
        System.out.println("== aim sanity");
        System.out.printf("  blue rear from its spot: aim %.2f deg (0 expected), miss at heading 0: %.2f in (0 expected)%n",
                Math.toDegrees(Aim.forCell(-55.4, 15.25, Cell.BLUE_REAR, 0, 0)), Aim.missIn(-55.4, 15.25, 0, Cell.BLUE_REAR));
        Vec sp = Aim.launchSpot(Cell.BLUE_AUDIENCE);
        System.out.printf("  blue audience from its spot: aim %.2f deg (180 expected)%n", Math.toDegrees(Aim.forCell(sp.x, sp.y, Cell.BLUE_AUDIENCE, 0, 0)));
        System.out.printf("  red symmetry: %.3f vs %.3f%n", Math.toDegrees(Aim.forCell(40, -30, Cell.RED_AUDIENCE, 0, 0)),
                Math.toDegrees(Angle.wrap(Aim.forCell(-40, 30, Cell.BLUE_REAR, 0, 0) + Math.PI)));
        TargetLock tl = new TargetLock();
        double turn = tl.update(new Pose(-55, -10, 0), 0, Cell.BLUE_REAR, 0, 0, 0.03, Double.NaN);
        System.out.printf("  lock: heading 0, cell up-left: turn %.3f (positive expected), error %.1f deg%n", turn, tl.errorDeg());

        System.out.println("== route planner survey: starts on a 6 in grid, every goal");
        Vec[] goals = {Aim.launchSpot(Cell.BLUE_REAR), Aim.launchSpot(Cell.BLUE_AUDIENCE), Field.parkDashCorner(true), Field.parkDashCorner(false),
                Aim.launchSpot(Cell.RED_REAR), Aim.launchSpot(Cell.RED_AUDIENCE), Field.parkDashCorner(true).rotated180(), Field.parkDashCorner(false).rotated180()};
        boolean[] spot = {true, true, false, false, true, true, false, false};
        for (int g = 0; g < goals.length; g++) {
            int ok = 0, refused = 0, skipped = 0, direct = 0;
            int[] vias = new int[4];
            double longest = 0;
            StringBuilder refusedAt = new StringBuilder();
            for (double x = -60; x <= 60; x += 6) {
                for (double y = -60; y <= 60; y += 6) {
                    Pose p = new Pose(x, y, 0);
                    if (RoutePlanner.touchesSupport(p, fp)) {
                        skipped++;
                        continue;
                    }
                    Plan pl = RoutePlanner.plan(p.vec(), goals[g], spot[g], fp);
                    if (pl.ok()) {
                        ok++;
                        vias[pl.legs() - 1]++;
                        longest = Math.max(longest, pl.length());
                    } else {
                        refused++;
                        if (refused <= 12) refusedAt.append(" (").append((int) x).append(',').append((int) y).append(')');
                    }
                }
            }
            System.out.printf("  goal %s: %d routes (direct %d, 1 via %d, 2 via %d, 3 via %d), %d refused, %d starts touching a support, longest %.0f in%n",
                    goals[g], ok, vias[0], vias[1], vias[2], vias[3], refused, skipped, longest);
            if (refused > 0) System.out.println("     refused at:" + refusedAt);
        }
        for (double[] s : new double[][] {{-55, 15}, {-50, -50}, {0, 0}, {50, 50}, {-60, 60}, {60, -20}, {-30, 0}, {0, 40}}) {
            Plan pl = RoutePlanner.plan(new Vec(s[0], s[1]), Aim.launchSpot(Cell.BLUE_AUDIENCE), true, fp);
            System.out.println("  from (" + s[0] + "," + s[1] + ") to the blue audience spot: " + pl.describe());
        }
        System.out.println("  from (55,7) to park, hive side: " + RoutePlanner.plan(new Vec(55, 7), Field.parkDashCorner(false), false, fp).describe());
        System.out.println("  from (-55,15) to park, audience side: " + RoutePlanner.plan(new Vec(-55, 15), Field.parkDashCorner(true), false, fp).describe());
        System.out.println("  from (0,-50) to park, audience side: " + RoutePlanner.plan(new Vec(0, -50), Field.parkDashCorner(true), false, fp).describe());

        System.out.println("== entry rule replay against the recorded runs");
        for (String f : new String[] {"2026-11-14_132704_cs5_proximity.csv", "2026-11-14_141611_teleop_cs5_proximity.csv", "2026-11-14_152200_teleop_cs5_proximity.csv"}) {
            for (boolean edge : new boolean[] {true, false}) {
                EntryRule.EDGE_COUNTING = edge;
                EntryRule r = new EntryRule();
                int n = 0, rows = 0;
                String first = null, line;
                BufferedReader br = new BufferedReader(new FileReader(a[0] + "/" + f));
                while ((line = br.readLine()) != null) {
                    if (line.startsWith("#")) {
                        if (first == null) first = line;
                        continue;
                    }
                    if (line.startsWith("t_sec")) continue;
                    String[] q = line.split(",");
                    rows++;
                    if (r.sample(Double.parseDouble(q[0]), (int) Double.parseDouble(q[1])) != null) n++;
                }
                br.close();
                if (edge) System.out.println("  " + first);
                System.out.printf("     %s rule: %d entries from %d samples, empty level %.0f%n", edge ? "edge" : "peak", n, rows, r.level());
            }
        }
        EntryRule.EDGE_COUNTING = true;

        System.out.println("== launch grid lookup with a made-up 3x3 lattice (speed 1300+10i+5j, residual 2+i deg, spot (2,2) no shot)");
        List<double[]> rows = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) rows.add(new double[] {-60 + i * 12 + 0.3 * j, j * 12 - 0.2 * i, 1300 + 10 * i + 5 * j, 2 + i, (i == 2 && j == 2) ? 0 : 1, 0});
        }
        LaunchGrid.setData(rows.toArray(new double[0][]));
        for (double[] q : new double[][] {{-60, 0}, {-54, 6}, {-48, 12}, {-20, 12}, {-36.4, 23.6}, {-70, -10}}) {
            LaunchGrid.Result r = LaunchGrid.lookup(Cell.BLUE_REAR, q[0], q[1]);
            System.out.printf("  (%.1f,%.1f): %s speed %.1f residual %.2f deg%n", q[0], q[1], r.zone, r.speed, Math.toDegrees(r.residualRad));
        }
        LaunchGrid.Result rr = LaunchGrid.lookup(Cell.RED_AUDIENCE, 54, -6), ba = LaunchGrid.lookup(Cell.BLUE_AUDIENCE, 54, 6);
        System.out.printf("  red audience (54,-6): %s %.1f %.2f; blue audience seed (54,6): %s %.1f %.2f seed=%b%n",
                rr.zone, rr.speed, Math.toDegrees(rr.residualRad), ba.zone, ba.speed, Math.toDegrees(ba.residualRad), ba.seed);

        dashSim();
    }

    /** A crude kinematic robot: speed follows the command with a lag. Catches sign errors and state machine stalls, not clearances. */
    static void dashSim() {
        System.out.println("== dash simulation (crude plant: 80 in/s and 450 deg/s at full command, 0.3 s lag)");
        final double[] aims = new double[4];
        HiveDash.AimSource src = new HiveDash.AimSource() {
            @Override public Vec launchSpot(Cell cell) { return Aim.launchSpot(cell); }
            @Override public double aimHeading(Vec from, Cell cell) { return Aim.forCell(from.x, from.y, cell, 0, 0); }
        };
        double[][] starts = {{-55, 15, 0}, {50, 50, -90}, {-50, -50, 90}, {0, 0, 0}, {-60, 60, 180}};
        for (Alliance al : Alliance.values()) {
            for (double[] s : starts) {
                for (boolean left : new boolean[] {true, false}) {
                    HiveDash dash = new HiveDash(fp, src);
                    TargetLock lock = new TargetLock();
                    HoldPosition hold = new HoldPosition();
                    HiveDash.In in = new HiveDash.In();
                    double x = s[0], y = s[1], h = Math.toRadians(s[2]), vx = 0, vy = 0, w = 0, t = 0, dt = 0.03;
                    int salvoCount = 0;
                    double salvoEnd = -1, minClear = 99;
                    String result = "timeout";
                    Cell cell = null;
                    while (t < 15) {
                        in.pose = new Pose(x, y, h);
                        in.vx = vx;
                        in.vy = vy;
                        in.omega = w;
                        in.t = t;
                        in.left = left;
                        in.right = !left;
                        in.alliance = al;
                        in.unavailable = null;
                        in.readsEmpty = false;
                        in.lockAligned = lock.aligned();
                        in.salvoRunning = salvoEnd >= 0;
                        in.salvoCount = salvoCount;
                        HiveDash.Out o = dash.update(in);
                        if (o.refusal != null) {
                            result = "refused: " + o.refusal;
                            break;
                        }
                        if (dash.state() == HiveDash.State.FINISHED) {
                            result = String.format("launched at %.1f s from (%.1f, %.1f, %.0f deg), %.1f in from the spot, lock error %.1f deg", t, x, y, Math.toDegrees(h),
                                    Math.hypot(x - Aim.launchSpot(cell).x, y - Aim.launchSpot(cell).y), lock.errorDeg());
                            break;
                        }
                        cell = o.cell;
                        double f = o.forward, l = o.left, turn = o.hasTurn ? o.turn : 0;
                        if (o.wantsLock) turn = lock.update(in.pose, w, cell, 0, 0, dt, Double.NaN);
                        else lock.release();
                        if (o.requestLaunch) salvoEnd = t + 1.2;
                        if (salvoEnd >= 0 && t >= salvoEnd) {
                            salvoEnd = -1;
                            salvoCount++;
                        }
                        hold.update(in.pose, vx, vy, w, salvoEnd >= 0, false, false, false, t, dt);
                        if (hold.active) {
                            f = hold.forward;
                            l = hold.left;
                        }
                        // plant
                        double tvx = (f * Math.cos(h) - l * Math.sin(h)) * 80, tvy = (f * Math.sin(h) + l * Math.cos(h)) * 80 * 0.78, tw = turn * Math.toRadians(450);
                        double k = dt / 0.3;
                        vx += (tvx - vx) * k;
                        vy += (tvy - vy) * k;
                        w += (tw - w) * k;
                        x += vx * dt;
                        y += vy * dt;
                        h = Angle.wrap(h + w * dt);
                        if (RoutePlanner.touchesSupport(new Pose(x, y, h), fp)) {
                            result = String.format("TOUCHED A SUPPORT at (%.1f, %.1f) t=%.1f", x, y, t);
                            break;
                        }
                        t += dt;
                    }
                    System.out.printf("  %s from (%.0f,%.0f,%.0f) D-pad %s -> %s: %s%n", al, s[0], s[1], s[2], left ? "left" : "right", cell == null ? "-" : cell.label(), result);
                }
            }
        }
        // Park Dash
        for (double[] s : starts) {
            ParkDash park = new ParkDash(fp);
            ParkDash.In in = new ParkDash.In();
            double x = s[0], y = s[1], h = Math.toRadians(s[2]), vx = 0, vy = 0, w = 0, t = 0, dt = 0.03;
            String result = "timeout";
            while (t < 15) {
                in.pose = new Pose(x, y, h);
                in.omega = w;
                in.t = t;
                in.up = true;
                in.alliance = Alliance.BLUE;
                ParkDash.Out o = park.update(in);
                if (o.refusal != null) {
                    result = "refused: " + o.refusal;
                    break;
                }
                if (o.parked) {
                    result = String.format("parked at %.1f s at (%.1f, %.1f, %.0f deg)", t, x, y, Math.toDegrees(h));
                    break;
                }
                double f = o.forward, l = o.left, turn = o.turn;
                double tvx = (f * Math.cos(h) - l * Math.sin(h)) * 80, tvy = (f * Math.sin(h) + l * Math.cos(h)) * 80 * 0.78, tw = turn * Math.toRadians(450);
                double k = dt / 0.3;
                vx += (tvx - vx) * k;
                vy += (tvy - vy) * k;
                w += (tw - w) * k;
                x += vx * dt;
                y += vy * dt;
                h = Angle.wrap(h + w * dt);
                if (RoutePlanner.touchesSupport(new Pose(x, y, h), fp)) {
                    result = String.format("TOUCHED A SUPPORT at (%.1f, %.1f) t=%.1f", x, y, t);
                    break;
                }
                t += dt;
            }
            System.out.printf("  BLUE park dash up from (%.0f,%.0f,%.0f): %s%n", s[0], s[1], s[2], result);
        }
    }
}
