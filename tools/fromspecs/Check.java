import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.aiming.LaunchGrid;
import org.firstinspires.ftc.teamcode.aiming.LaunchGridSpots;
import org.firstinspires.ftc.teamcode.aiming.TargetLock;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.navigation.GridRoutes;
import org.firstinspires.ftc.teamcode.navigation.HiveDash;
import org.firstinspires.ftc.teamcode.navigation.HoldPosition;
import org.firstinspires.ftc.teamcode.navigation.ParkDash;
import org.firstinspires.ftc.teamcode.navigation.Plan;
import org.firstinspires.ftc.teamcode.navigation.RoutePlanner;
import org.firstinspires.ftc.teamcode.robot.Footprint;
import org.firstinspires.ftc.teamcode.subsystems.EntryRule;
import org.firstinspires.ftc.teamcode.subsystems.HeldCount;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSequencer;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Base64;
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
                    Plan pl = RoutePlanner.plan(p.vec(), goals[g], fp);
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
            Plan pl = RoutePlanner.plan(new Vec(s[0], s[1]), Aim.launchSpot(Cell.BLUE_AUDIENCE), fp);
            System.out.println("  from (" + s[0] + "," + s[1] + ") to the blue audience spot: " + pl.describe());
        }
        System.out.println("  from (55,7) to park, hive side: " + RoutePlanner.plan(new Vec(55, 7), Field.parkDashCorner(false), fp).describe());
        System.out.println("  from (-55,15) to park, audience side: " + RoutePlanner.plan(new Vec(-55, 15), Field.parkDashCorner(true), fp).describe());
        System.out.println("  from (0,-50) to park, audience side: " + RoutePlanner.plan(new Vec(0, -50), Field.parkDashCorner(true), fp).describe());

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
        legChecks();
        gridSpotChecks();
        gridRouteChecks();
        partChecks();
        runLogChecks(a[0]);
        System.out.println(failures == 0 ? "== all checks with an expected answer passed" : "== " + failures + " CHECK(S) FAILED");
        if (failures > 0) System.exit(1);
    }

    static int failures;

    static void expect(boolean ok, String what) {
        System.out.println("  " + (ok ? "ok    " : "FAILED") + " " + what);
        if (!ok) failures++;
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

    /** The leg check as specs/dashes.md section 1.2 now gives it: swept rectangles, with the exact exemptions. */
    static void legChecks() {
        System.out.println("== leg check (specs/dashes.md 1.2): rectangles and exemptions");
        List<RoutePlanner.Obstacle> obs = RoutePlanner.obstacles();
        Vec gateE = new Vec(34, 0), gateW = new Vec(-34, 0);
        // Between two route points: the lane is passable on its middle line and nowhere else.
        expect(RoutePlanner.legOk(gateW, gateE, false, false, fp, obs), "gate to gate along y = 0 is clear");
        expect(RoutePlanner.legOk(new Vec(-34, 3.9), new Vec(34, 3.9), false, false, fp, obs), "a lane leg 3.9 in off the middle line is still clear (walls keep the center within about 4 in)");
        expect(!RoutePlanner.legOk(new Vec(-34, 4.2), new Vec(34, 4.2), false, false, fp, obs), "a lane leg 4.2 in off the middle line touches a lane wall");
        // A leg from the robot checks the walls only from 10 in along the leg onward; one of 10 in or less ignores them.
        expect(!RoutePlanner.legOk(new Vec(-40, 6), new Vec(34, 0), true, false, fp, obs), "from the robot at (-40, 6) into the lane: refused, it is not centered");
        expect(RoutePlanner.legOk(new Vec(-40, 0), new Vec(34, 0), true, false, fp, obs), "from the robot at (-40, 0) into the lane: clear");
        expect(RoutePlanner.legOk(new Vec(-28, 8), new Vec(-28, 0), true, false, fp, obs), "an 8 in leg from the robot at the lane's mouth ignores the walls");
        expect(!RoutePlanner.legOk(new Vec(-28, 8), new Vec(-28, 0), false, false, fp, obs), "the same 8 in leg between route points does not");
        // A robot that starts inside the box between the two bars may always leave: the whole first leg ignores the walls.
        expect(RoutePlanner.legOk(new Vec(0, 6), new Vec(34, 0), true, false, fp, obs), "from between the bars, 6 in off the middle line, out to the gate: clear");
        expect(!RoutePlanner.legOk(new Vec(0, 6), new Vec(34, 0), false, false, fp, obs), "the same leg not from the robot touches a wall");
        // A flower just gathered from is ignored on a leg from the robot that ends farther from it.
        Vec atFlower = new Vec(-23.4, 52), away = new Vec(-36, 40);
        expect(RoutePlanner.legOk(atFlower, away, true, false, fp, obs), "from the Blue Wall flower (robot 11.2 in from the grown stand) away to the west helper: clear");
        expect(!RoutePlanner.legOk(atFlower, away, false, false, fp, obs), "the same leg not from the robot touches the flower stand");
        // A leg that ends at the goal: against the flower stands the rectangle ends at the goal, and the robot there is its far-corner circle with no margin.
        Vec spot = Aim.launchSpot(Cell.BLUE_AUDIENCE);
        expect(RoutePlanner.legOk(gateE, spot, false, true, fp, obs), "east gate to the audience launch spot as the goal: clear");
        expect(!RoutePlanner.legOk(gateE, spot, false, false, fp, obs), "the same leg not ending at the goal is refused (the rectangle reaches past the spot into the flower stand)");
        expect(!RoutePlanner.legOk(new Vec(36, 40), spot, false, true, fp, obs), "east helper to the audience launch spot (38 in, so not aligned): the wide rectangle touches the flower stand");
        // A short leg from the robot is boxed from the start onward; the robot's own circle is checked without margin.
        expect(!RoutePlanner.legOk(new Vec(0, 36), new Vec(0, 46), true, false, fp, obs), "from the robot 11.3 in outside a bar: its own circle (12.0 in) touches the bar");
        expect(RoutePlanner.legOk(new Vec(0, 38), new Vec(0, 48), true, false, fp, obs), "from the robot 13.3 in outside a bar, moving away: clear, nothing is checked behind the start");
        expect(!RoutePlanner.legOk(new Vec(0, 48), new Vec(0, 38), true, false, fp, obs), "the same leg driven toward the bar is refused (the box reaches past its end)");
    }

    /** The launch grid's spot generator (specs/aiming.md section 4.4). */
    static void gridSpotChecks() {
        System.out.println("== launch grid spots (specs/aiming.md 4.4): about 30 per cell, as a serpentine sweep");
        LaunchGridSpots.Fit fit = new LaunchGridSpots.Fit() {
            @Override public boolean clearOfObstacles(double x, double y, double heading) {
                return RoutePlanner.bodyClear(new Pose(x, y, heading), fp, RoutePlanner.MARGIN_IN);
            }

            @Override public double[] overWalls(double x, double y, double heading, double inset) {
                double ox = 0, oy = 0, limit = Field.HALF_IN - inset;
                for (double[] c : fp.corners(x, y, heading)) {
                    ox = Math.max(ox, Math.abs(c[0]) - limit);
                    oy = Math.max(oy, Math.abs(c[1]) - limit);
                }
                return new double[] {ox, oy};
            }
        };
        for (Cell c : new Cell[] {Cell.BLUE_REAR, Cell.BLUE_AUDIENCE}) {
            List<LaunchGridSpots.Spot> spots = LaunchGridSpots.spots(c, fit, -63.4, 16.5);
            int slid = 0;
            boolean allRules = true, fits = true;
            StringBuilder sb = new StringBuilder();
            for (LaunchGridSpots.Spot v : spots) {
                if (v.slid) slid++;
                allRules &= (v.x - c.x()) * c.openSign() > 0 && Aim.distance(v.x, v.y, c) >= 12
                        && Math.abs(Math.toDegrees(Angle.diff(Aim.bearing(v.x, v.y, c), c.faceHeading()))) <= 75;
                double[] over = fit.overWalls(v.x, v.y, LaunchGridSpots.aimAt(v.x, v.y, c), 3.0);
                fits &= over[0] <= 1e-6 && over[1] <= 1e-6;
                if (sb.length() < 150) sb.append(String.format(java.util.Locale.US, " (%.1f, %.1f)%s", v.x, v.y, v.slid ? "*" : ""));
            }
            expect(fits, c + ": every spot, slid or not, fits inside the walls less 3 in at its aim heading");
            double nearest = Double.MAX_VALUE;
            for (LaunchGridSpots.Spot v : spots) nearest = Math.min(nearest, Math.abs(v.x - -63.4));
            expect(!spots.isEmpty() && Math.abs(spots.get(0).x - -63.4) <= nearest + 8.1, c + ": the sweep begins in the column nearest the start");
            System.out.println("  " + c + ": " + spots.size() + " spots, " + slid + " slid off a wall; first:" + sb);
            expect(spots.size() >= 20 && spots.size() <= 40 && allRules, c + ": every spot is on the open side, at least 12 in from the cell and within 75 deg of the opening's normal");
        }
    }

    static String b64(byte[] b) { return Base64.getEncoder().encodeToString(b); }

    /** The reading of the grid route tables and the checks that decide whether they may be used (specs/dashes.md 5.1), on a made-up table. */
    static void gridRouteChecks() {
        System.out.println("== grid route tables (specs/dashes.md 5.1): numbering, reading and the checks");
        expect(GridRoutes.NODES_PER_AXIS == 23 && GridRoutes.STATES == 8993, "23 nodes per axis, 8,993 states per goal");
        expect(GridRoutes.node(22, 22) == 528 && GridRoutes.state(528, 16) == 8992, "node (22, 22) is 528 and its final state is 8992");
        Vec n = GridRoutes.nodePoint(GridRoutes.node(3, 20));
        expect(n.x == -48 && n.y == 54, "node (3, 20) is at (-48, 54)");
        expect(GridRoutes.stepFrom(GridRoutes.node(5, 5), 5) == GridRoutes.node(4, 7), "direction 5 is the step (-1, 2)");
        expect(GridRoutes.stepFrom(GridRoutes.node(0, 0), 8) == -1, "a step off the lattice leads nowhere");
        expect(GridRoutes.unusable(new Vec(40, 60)) != null, "this build has no generated tables: " + GridRoutes.unusable(new Vec(40, 60)));

        // A made-up tree for one goal at (3, 1): along the row y = 0 in direction 0 to node (11, 11), then the final state.
        byte[] next = new byte[2 * GridRoutes.STATES], time = new byte[2 * GridRoutes.STATES], speed = new byte[GridRoutes.STATES];
        java.util.Arrays.fill(next, (byte) 0xFF);
        java.util.Arrays.fill(time, (byte) 0xFF);
        for (int i = 0; i <= 11; i++) {
            int s = GridRoutes.state(GridRoutes.node(i, 11), i < 11 ? 0 : GridRoutes.FINAL_DIRECTION);
            int nx = i < 10 ? GridRoutes.state(GridRoutes.node(i + 1, 11), 0) : i == 10 ? GridRoutes.state(GridRoutes.node(11, 11), GridRoutes.FINAL_DIRECTION) : 0xFFFF;
            int ms = (12 - i) * 150;
            next[2 * s] = (byte) (nx >> 8);
            next[2 * s + 1] = (byte) nx;
            time[2 * s] = (byte) (ms >> 8);
            time[2 * s + 1] = (byte) ms;
            speed[s] = (byte) (i < 11 ? 131 : 0);
        }
        double[] corner = {66, 60, 44, 36, 24, 20, 20, 20, 20, 20, 20};
        String[] goal = {"LEFT"}, nx = {b64(next)}, tm = {b64(time)}, sp = {b64(speed)};
        double[] gx = {3}, gy = {1}, aim = {Double.NaN};
        String wrong = GridRoutes.setData("pitch 4.0 in, half span 66.0 in, 16 directions", "model A", corner, goal, gx, gy, aim, nx, tm, sp);
        expect(wrong != null, "a table built on another lattice is not used: " + wrong);
        String shortTable = GridRoutes.setData(GridRoutes.geometry(), "model A", corner, goal, gx, gy, aim, new String[] {b64(new byte[100])}, tm, sp);
        expect(shortTable != null, "a table of the wrong size is not used: " + shortTable);
        String fewCorners = GridRoutes.setData(GridRoutes.geometry(), "model A", new double[] {66, 60, 44}, goal, gx, gy, aim, nx, tm, sp);
        expect(fewCorners != null, "a corner speed table without the eleven angles is not used: " + fewCorners);
        String good = GridRoutes.setData(GridRoutes.geometry(), "model A", corner, goal, gx, gy, aim, nx, tm, sp);
        expect(good == null, "a table with this code's geometry text (\"" + GridRoutes.geometry() + "\") is read");
        expect(GridRoutes.unusable(new Vec(3, 1)) == null, "its goal at (3, 1) can be used");
        expect(GridRoutes.unusable(new Vec(3, 1.9)) == null, "a goal within 1 in of the table's still has its table");
        expect(GridRoutes.unusable(new Vec(3, 2.5)) != null, "a goal that has moved more than 1 in has none: " + GridRoutes.unusable(new Vec(3, 2.5)));
        expect(GridRoutes.modelNote() != null, "a model text that differs is reported and the tables are still used: " + GridRoutes.modelNote());
        GridRoutes.setData(GridRoutes.geometry(), GridRoutes.MODEL, corner, goal, gx, gy, aim, nx, tm, sp);
        expect(GridRoutes.modelNote() == null && GridRoutes.MODEL.startsWith("V_MAX 66.0, A_DEC 120.0, A_GRIP 150.0"), "the page's model text is this code's, and then nothing is reported");
        GridRoutes.Table t = GridRoutes.table(new Vec(3, 1));
        int start = GridRoutes.state(GridRoutes.node(0, 11), 0);
        List<Vec> route = t.trace(start);
        expect(route != null && route.size() == 13 && route.get(0).x == -66 && route.get(11).x == 0 && route.get(12).x == 3 && route.get(12).y == 1,
                "the route from node (0, 11) is 11 lattice steps and the final leg to the goal");
        expect(Math.abs(t.seconds(start) - 1.8) < 1e-9 && t.endSpeed(start) == 65.5, "time 1.8 s (milliseconds, high byte first) and end speed 65.5 in/s (half inches per second)");
        expect(t.trace(GridRoutes.state(GridRoutes.node(0, 0), 0)) == null && Double.isNaN(t.seconds(0)), "an unreachable state (0xFFFF) has no route and no time");
        expect(Math.abs(GridRoutes.cornerSpeed(25) - 40) < 1e-9 && GridRoutes.cornerSpeed(0) == 66 && GridRoutes.cornerSpeed(170) == 20,
                "corner speed: 66 at 0 deg, 40 at 25 deg (between 44 and 36), 20 from 60 deg on");
        GridRoutes.setData("", "", null, null, null, null, null, null, null, null);
    }

    /** The launch sequencer and the held count as specs/mechanisms.md sections 3 and 16.3 now give them. */
    static void partChecks() {
        System.out.println("== launch sequencer and held count (specs/mechanisms.md 3 and 16.3)");
        LaunchSequencer seq = new LaunchSequencer();
        seq.queue(0);
        expect(seq.queuedStep(0.0, true, true, true, 1350, 1350) == LaunchSequencer.Event.START, "a launch whose conditions hold starts in the update it was requested in");
        seq.salvoEnded(1.2);
        expect(seq.salvoCount() == 1 && !seq.busy(), "a salvo that ends (the pusher stopped, or gave up) counts as a salvo");
        seq.queue(2);
        seq.queuedStep(2.0, true, true, true, 1350, 1350);
        seq.cancel(2.5);
        expect(seq.salvoCount() == 1 && !seq.busy(), "a salvo dropped by a pusher fault does not count");
        expect(seq.burst(3.0) && !seq.burst(3.2), "burst logging lasts 0.6 s after the salvo");
        seq.queue(10);
        expect(seq.queuedStep(13.5, true, true, true, 1350, 1350) == LaunchSequencer.Event.DROPPED, "a queued launch is dropped on its timeout before its conditions are tested");
        HeldCount held = new HeldCount();
        held.start(4);
        int taken = held.salvoStarted(0);
        held.onEntry(5.0);
        held.salvoIncomplete();
        expect(taken == 4 && held.held() == 5 && held.tally() == 0, "an incomplete salvo puts back the count (4) plus what entered since (1), and the tally");
    }

    static List<String> lines(File f) throws Exception {
        return Files.readAllLines(f.toPath(), StandardCharsets.UTF_8);
    }

    /** The run log's first line, start and end markers, crash lines and the discarded INIT-only run (specs/logging.md 1.2), written on the PC. */
    static void runLogChecks(String dataDir) throws Exception {
        System.out.println("== run log (specs/logging.md 1.2): first line, RUN STARTED, RUN ENDED, CRASH, the INIT-only run");
        File dir = new File("build/fromspecs/runlogs");
        if (dir.exists()) for (File f : dir.listFiles()) f.delete();
        RunLog.DIR = dir.getPath();
        File latest = new File(dir, "bringup.log"), prev = new File(dir, "bringup_prev.log"), prev2 = new File(dir, "bringup_prev2.log");

        RunLog.open(RunLog.Kind.BRINGUP, "Check run 1");        // an INIT that is stopped without starting
        RunLog.event("SETUP something at INIT");
        RunLog.close();
        List<String> a = lines(latest);
        expect(a.get(0).matches("# Check run 1, opened \\d{4}-\\d{2}-\\d{2} \\d{2}:\\d{2}:\\d{2} \\(hub clock\\)\\. Previous bringup run: none"), "first line: " + a.get(0));
        expect(a.get(1).startsWith("# Build: "), "second line is the build stamp");
        expect(a.get(a.size() - 1).matches("E,[0-9.]+,RUN ENDED \\(normal\\)"), "last line: " + a.get(a.size() - 1));

        RunLog.open(RunLog.Kind.BRINGUP, "Check run 2");        // a normal run
        RunLog.started();
        RunLog.sampler("a,b", 0, new RunLog.Sampler() {
            @Override public String row() { return "1,2"; }
        });
        RunLog.tick();
        RunLog.close();
        a = lines(latest);
        expect(a.get(0).endsWith("Previous bringup run: init only, stopped before start") && !prev.exists(),
                "the INIT-only run is named in the next first line, deleted, and the other runs are not shifted");
        boolean started = false, header = false, sample = false;
        for (String s : a) {
            started |= s.matches("E,[0-9.]+,RUN STARTED");
            header |= s.equals("H,t,loop_ms,loop_max_ms,volt,a,b");
            sample |= s.matches("S,[0-9.]+,[0-9.]*,[0-9.]*,,1,2");
        }
        expect(started && header && sample, "RUN STARTED, the H line with the four common columns, and an S row");

        RunLog.open(RunLog.Kind.BRINGUP, "Check run 3");        // a crash at INIT
        RunLog.crash(new IllegalStateException("made up, with a comma"));
        RunLog.close();
        a = lines(latest);
        expect(a.get(0).endsWith("Previous bringup run: ended normally") && prev.exists(), "after a normal run: \"ended normally\", and that run is shifted to bringup_prev.log");
        int crashAt = -1, traceLines = 0;
        for (int i = 0; i < a.size(); i++) {
            if (a.get(i).contains(",CRASH: java.lang.IllegalStateException: made up, with a comma")) crashAt = i;
            else if (crashAt >= 0 && a.get(i).matches("E,[0-9.]+,at .*")) traceLines++;
        }
        expect(crashAt >= 0 && traceLines >= 1 && traceLines <= 60, "CRASH: with the exception, then " + traceLines + " stack trace lines, each its own E line (at most 60)");
        expect(a.get(a.size() - 1).matches("E,[0-9.]+,RUN ENDED \\(after a crash\\)"), "last line: " + a.get(a.size() - 1));

        RunLog.open(RunLog.Kind.BRINGUP, "Check run 4");        // stopped hard: no end marker
        RunLog.started();
        RunLog.event("something");
        Thread.sleep(1600);                                     // the writer flushes about once a second
        a = lines(latest);
        expect(a.get(0).endsWith("Previous bringup run: CRASHED (see the exception in bringup_prev.log)") && prev2.exists(),
                "a crashed INIT is kept: " + a.get(0).substring(a.get(0).indexOf("Previous")));
        // Leave run 4 without closing it, as a hard stop would, and open the next on top of a copy of it.
        File hard = new File(dir, "teleop.log");
        Files.copy(latest.toPath(), hard.toPath());
        RunLog.close();
        RunLog.open(RunLog.Kind.TELEOP, "Check run 5");
        RunLog.close();
        a = lines(hard);
        expect(a.get(0).endsWith("Previous teleop run: NO END MARKER: the app stopped hard or the robot lost power"), "a log without RUN ENDED: " + a.get(0).substring(a.get(0).indexOf("Previous")));
        expect(new File(dir, "bringup.log").exists() && new File(dir, "teleop_prev.log").exists(), "kinds never touch each other's files");

        // The coordinator's column names and their order (specs/mechanisms.md section 14), read from the source.
        String src = new String(Files.readAllBytes(new File("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Mechanisms.java").toPath()), StandardCharsets.UTF_8)
                .replace("\"\n            + \"", "").replace("\"\r\n            + \"", "");
        expect(src.contains("\"setpoint,fly_target,fly_vel,fly_ready,pusher_state,pusher_deg,pusher_vel,pusher_amps,intake_mode,ramp,count,slots,launch,warnings,overrides,check,"
                + "fly_power,fly_amps,pusher_lock,pusher_hold,intake_amps,cs5_entries,cs5_held,cs5_full,cs5_prox,cs5f,launched_total\""), "the coordinator's 27 column names, in the page's order");
    }
}
