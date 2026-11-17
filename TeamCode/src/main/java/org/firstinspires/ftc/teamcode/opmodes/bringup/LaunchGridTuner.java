package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.aiming.LaunchGridSpots;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.navigation.Plan;
import org.firstinspires.ftc.teamcode.navigation.RoutePlanner;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.robot.Footprint;
import org.firstinspires.ftc.teamcode.subsystems.LaunchMap;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Launch Grid Tuner (specs/support-opmodes.md section 2.2): collect the data for the two-dimensional launch grid
 * (specs/aiming.md section 4): aim and speed per field position for one hive cell. The alliance is assumed blue.
 *
 * While the right trigger is held the robot drives itself to the next spot of the grid, already pointed at the cell
 * with today's aim and at the launch map's speed. The operator loads, fires, and trims the angle (the robot re-aims
 * by itself) and the speed until the salvo scores, then accepts the spot, or marks it as "no shot possible", or goes
 * back one spot. The route to each spot comes from the dash planner; where the planner refuses a spot beside a flower
 * stand, a short straight hop is used if it is clear, otherwise the operator drives by hand.
 *
 * Controls (gamepad 1). At INIT, D-pad left / right chooses NEAR (the rear cell) / FAR (the audience cell); the start
 * is against the rear wall, at (-63.4, 16.5, 0 deg). Right trigger (hold): drive to the next spot; letting go stops
 * at once. Sticks: drive by hand, robot-oriented, while the trigger is not held; right bumper precision (30 percent).
 * D-pad left / right: aim left / right by 0.5 deg; the robot turns by itself. D-pad up / down: speed, 20 ticks/s a
 * step. Y: fire. A / B / X: accept the spot / no shot is possible here / back one spot. Left trigger (hold): intake
 * OUT. Back: clear a pusher fault; held 1 s, reseed the pose to the start.
 *
 * Settings read at INIT: START_INDEX, RETEST_SPOTS, EXTRA_SPOTS and CARRY_TRIMS, below.
 * Log lines: FIRE, GRID POINT, GRID TABLE and TRIM, in the page's wording, and RE-AIM OK / RE-AIM STUCK.
 */
@Config
@TeleOp(name = "Launch Grid Tuner", group = "2 Bring-up")
public final class LaunchGridTuner extends TeamOpMode {
    /** Start the sweep at this position in the list of spots that will be visited, counted from 0; past the end means the last. The earlier spots stay in the list. */
    public static int START_INDEX = 0;
    /** Visit only these spots: a comma list of spot numbers with ranges, such as "8-10,22-30". Empty = every spot. The spots keep their numbers. */
    public static String RETEST_SPOTS = "";
    /** Extra positions as "x,y;x,y", visited last and numbered from 101. Each must pass the grid's rules. */
    public static String EXTRA_SPOTS = "";
    /** Start a spot with the previous spot's trims; otherwise they start at 0. */
    public static boolean CARRY_TRIMS = false;
    public static double START_Y = 16.5;
    public static double DRIVE_POWER = 0.5, AT_SPOT_IN = 4.0, MAX_AIM_TRIM_DEG = 20.0;
    public static double REAIM_POWER = 0.3, REAIM_TOL_DEG = 0.25, REAIM_STUCK_DEG = 0.4, REAIM_CAP_S = 2.5, REAIM_HOLD_IN = 1.0;
    private static final double SPEED_STEP = 20, AIM_STEP_DEG = 0.5, PRECISION = 0.30, TRIGGER = 0.3, RESEED_HOLD_S = 1.0, HOP_STEP_IN = 0.5;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Launch Grid Tuner"; }

    /** One spot of the sweep and what was found there. */
    private static final class Spot {
        int number;
        double x, y;
        boolean slid;
        /** The spot's distance from the cell, and today's aim from it with no trim. */
        double distance, geometricRad;
        char status = ' ';      // ' ' not done, 'G' good, 'N' no shot possible
        int salvos;
        double speedTrim, aimTrimDeg, doneSpeed = Double.NaN;

        double mapSpeed() { return LaunchMap.speed(distance); }
        double aimDeg() { return Math.toDegrees(geometricRad) + aimTrimDeg; }
    }

    private Footprint footprint;
    private Cell cell;
    private final List<String> leftOut = new ArrayList<>();

    private final LaunchGridSpots.Fit fit = new LaunchGridSpots.Fit() {
        @Override public boolean clearOfObstacles(double x, double y, double heading) {
            return RoutePlanner.bodyClear(new Pose(x, y, heading), footprint, RoutePlanner.MARGIN_IN);
        }

        @Override public double[] overWalls(double x, double y, double heading, double inset) {
            double ox = 0, oy = 0, limit = Field.HALF_IN - inset;
            for (double[] c : footprint.corners(x, y, heading)) {
                ox = Math.max(ox, Math.abs(c[0]) - limit);
                oy = Math.max(oy, Math.abs(c[1]) - limit);
            }
            return new double[] {ox, oy};
        }
    };

    private Spot spot(int number, double x, double y, boolean slid) {
        Spot s = new Spot();
        s.number = number;
        s.x = x;
        s.y = y;
        s.slid = slid;
        s.distance = Aim.distance(x, y, cell);
        s.geometricRad = LaunchGridSpots.aimAt(x, y, cell);
        return s;
    }

    /** The spot numbers of RETEST_SPOTS: a comma list with ranges. */
    private static List<Integer> retest() {
        List<Integer> out = new ArrayList<>();
        for (String part : RETEST_SPOTS.split(",")) {
            String p = part.trim();
            if (p.isEmpty()) continue;
            try {
                int dash = p.indexOf('-', 1);
                int from = Integer.parseInt((dash < 0 ? p : p.substring(0, dash)).trim());
                int to = dash < 0 ? from : Integer.parseInt(p.substring(dash + 1).trim());
                for (int n = from; n <= to; n++) out.add(n);
            } catch (NumberFormatException ignored) { }
        }
        return out;
    }

    private List<Spot> buildSweep(Pose start) {
        List<Spot> all = new ArrayList<>();
        int number = 0;
        for (LaunchGridSpots.Spot g : LaunchGridSpots.spots(cell, fit, start.x, start.y)) all.add(spot(++number, g.x, g.y, g.slid));
        // The extra positions, visited last and numbered from 101.
        leftOut.clear();
        int extra = 100;
        for (String pair : EXTRA_SPOTS.split(";")) {
            String[] xy = pair.trim().split(",");
            if (xy.length != 2) continue;
            extra++;
            try {
                double x = Double.parseDouble(xy[0].trim()), y = Double.parseDouble(xy[1].trim());
                String why = LaunchGridSpots.invalid(x, y, cell, fit);
                if (why == null) all.add(spot(extra, x, y, false));
                else leftOut.add(extra + " (" + pair.trim() + "): " + why);
            } catch (NumberFormatException e) {
                leftOut.add(extra + " (" + pair.trim() + "): not two numbers");
            }
        }
        List<Integer> only = retest();
        List<Spot> sweep = new ArrayList<>();
        for (Spot s : all) if (only.isEmpty() || only.contains(s.number)) sweep.add(s);
        return sweep;
    }

    /**
     * The short straight hop: the robot's rectangle, grown by the planner's margin and turning steadily from its
     * present heading to the spot's, touches no support and no flower stand at any half inch of the line, and stays
     * inside the walls less 3 in.
     */
    private boolean hopClear(Pose from, double toX, double toY, double heading) {
        double len = from.distTo(toX, toY), turn = Angle.diff(heading, from.h), limit = Field.HALF_IN - LaunchGridSpots.WALL_INSET_IN;
        int n = Math.max(1, (int) Math.ceil(len / HOP_STEP_IN));
        for (int k = 0; k <= n; k++) {
            double f = (double) k / n;
            Pose p = new Pose(from.x + (toX - from.x) * f, from.y + (toY - from.y) * f, from.h + turn * f);
            if (!RoutePlanner.standsClear(p, footprint, RoutePlanner.MARGIN_IN)) return false;
            for (double[] c : footprint.corners(p.x, p.y, p.h)) {
                if (Math.abs(c[0]) > limit || Math.abs(c[1]) > limit) return false;
            }
        }
        return true;
    }

    /** The TRIM line, at every trim. */
    private void logTrim(Spot s, Pose p) {
        RunLog.eventf("TRIM spot %d: angle %+.1f deg, speed %+.0f ticks/s (aim heading %.1f deg)", s.number, s.aimTrimDeg, s.speedTrim,
                Math.toDegrees(Aim.forCell(p.x, p.y, cell, s.aimTrimDeg, 0)));
    }

    private String pointLine(Spot s, int total, Pose p) {
        String found = s.status == 'N' ? "NO SHOT possible"
                : String.format(Locale.US, "GOOD: speed %.0f ticks/s (map %.0f %+.0f trim), aim heading %.1f deg (geometric %.1f %+.1f trim)",
                s.mapSpeed() + s.speedTrim, s.mapSpeed(), s.speedTrim, s.aimDeg(), Math.toDegrees(s.geometricRad), s.aimTrimDeg);
        return String.format(Locale.US, "GRID POINT %d of %d: spot (%.1f, %.1f)%s, %.1f in, %s, %d salvo(s) fired%s, pose (%.1f, %.1f, %.1f deg)",
                s.number, total, s.x, s.y, s.slid ? " slid" : "", s.distance, found, s.salvos, s.status == 'G' && s.salvos == 0 ? " (UNVERIFIED)" : "",
                p.x, p.y, p.headingDeg());
    }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        footprint = profile.footprint();
        Alliance alliance = Alliance.BLUE;      // the alliance is assumed blue
        Pose start = new Pose(Field.startRear(profile.originToBack).x, START_Y, 0);
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, start);
        Mechanisms.Settings ms = new Mechanisms.Settings();
        ms.intakeAlwaysOn = true;
        ms.flywheelStopsAfterSalvo = false;
        ms.flyPosAtInit = true;
        mech = new Mechanisms(hardwareMap, profile, alliance, ms);
        cell = Cell.BLUE_REAR;
        List<Spot> sweep = buildSweep(start);
        Edge l = new Edge(), r = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            mech.initLoop();
            boolean left = l.pressed(gamepad1.dpad_left), right = r.pressed(gamepad1.dpad_right);
            if (left || right) {
                cell = left ? Cell.BLUE_REAR : Cell.BLUE_AUDIENCE;
                sweep = buildSweep(start);
            }
            telemetry.addLine("Launch Grid Tuner, BLUE. D-pad left = NEAR (the rear cell), right = FAR (the audience cell): " + (cell.rear ? "NEAR" : "FAR"));
            telemetry.addLine("Start against the rear wall at " + start + ", still. Pose " + drive.pose() + (drive.localizer.seedConfirmed() ? "" : " SEED NOT CONFIRMED"));
            telemetry.addLine(sweep.size() + " spots will be visited" + (sweep.isEmpty() ? "" : ", starting at spot " + sweep.get(Math.max(0, Math.min(START_INDEX, sweep.size() - 1))).number)
                    + ". START_INDEX " + START_INDEX + ", RETEST_SPOTS \"" + RETEST_SPOTS + "\", CARRY_TRIMS " + CARRY_TRIMS);
            for (String s : leftOut) telemetry.addLine("extra spot left out: " + s);
            telemetry.addLine("Found: " + mech.found());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        sweep = buildSweep(start);
        RunLog.eventf("SETUP profile %s, alliance %s, start pose default %s, driver forward %.0f deg; Launch Grid Tuner, cell %s (%s), %d spots (START_INDEX %d, RETEST_SPOTS \"%s\", EXTRA_SPOTS \"%s\", CARRY_TRIMS %b)",
                profile.name, alliance, drive.pose(), Math.toDegrees(alliance.driverForwardRad()), cell, cell.rear ? "NEAR" : "FAR", sweep.size(),
                START_INDEX, RETEST_SPOTS, EXTRA_SPOTS, CARRY_TRIMS);
        for (String s : leftOut) RunLog.event("GRID extra spot left out: " + s);
        mech.setTargetCell(cell);
        mech.setSpin(true);
        Edge aimL = new Edge(), aimR = new Edge(), up = new Edge(), down = new Edge(), fire = new Edge(), accept = new Edge(), noShot = new Edge(), backOne = new Edge(), back = new Edge();
        Hold reseed = new Hold();
        int index = Math.max(0, Math.min(START_INDEX, sweep.size() - 1));        // the earlier spots stay in the list: X can go back to them
        List<Pose> route = null;        // the poses still to pass on the way to the spot
        DriveToPose leg = null, reAim = null;
        double reAimFrom = 0, reAimTarget = 0;
        boolean wasSalvo = false, wasHeld = false;
        String status = "", last = "";
        while (opModeIsActive()) {
            double t = Clock.now();
            drive.updatePose();
            Pose p = drive.pose();
            Spot spot = index < sweep.size() ? sweep.get(index) : null;
            boolean atSpot = spot != null && p.distTo(spot.x, spot.y) <= AT_SPOT_IN;       // driven there, or by hand to within this
            // The speed is the launch map's for the spot, plus the spot's trim.
            mech.setManualSetpoint(spot == null ? Double.NaN : spot.mapSpeed());
            mech.setSpeedTrim(spot == null ? 0 : spot.speedTrim);
            mech.setOuttake(gamepad1.left_trigger > TRIGGER);
            if (back.pressed(gamepad1.back)) mech.clearPusherStall();
            if (reseed.held(gamepad1.back, t, RESEED_HOLD_S)) {
                drive.stop();
                drive.localizer.setPose(start);
                RunLog.eventf("PINPOINT reseeded by the driver to %s", start);
            }

            // ---- Driving: the trigger drives to the spot; a re-aim turns by itself; otherwise the sticks ----
            boolean held = gamepad1.right_trigger > TRIGGER && spot != null;
            if (held) {
                reAim = null;
                if (!wasHeld) {
                    // Planned again each time the trigger is pulled, from where the robot is. The robot arrives already pointed at the cell.
                    double h = Angle.wrap(spot.geometricRad + Math.toRadians(spot.aimTrimDeg));
                    Plan plan = RoutePlanner.plan(p, new Vec(spot.x, spot.y), footprint);
                    route = new ArrayList<>();
                    if (plan.ok()) {
                        for (int k = 1; k < plan.points.length; k++) route.add(new Pose(plan.points[k].x, plan.points[k].y, h));
                        status = "route: " + plan.describe();
                    } else if (hopClear(p, spot.x, spot.y, h)) {
                        route.add(new Pose(spot.x, spot.y, h));
                        status = "the planner refused (" + plan.refusal + "): a short straight hop";
                    } else {
                        route = null;
                        status = "BLOCKED: " + plan.refusal + " and a straight hop is not clear: drive by hand";
                    }
                    RunLog.eventf("GRID drive to spot %d at (%.1f, %.1f) from %s: %s", spot.number, spot.x, spot.y, p, status);
                    leg = null;
                }
                if (leg == null && route != null && !route.isEmpty()) {
                    boolean lastLeg = route.size() == 1;
                    leg = new DriveToPose(drive, "to grid spot " + spot.number, route.get(0)).maxPower(DRIVE_POWER).distanceTimeout().stiction(lastLeg);
                    if (!lastLeg) leg.tolerance(3.0, 10.0).settle(0);        // a via point is passed, not stopped at
                }
                if (leg != null) {
                    if (!leg.run(null)) {
                        leg = null;
                        route.remove(0);
                        if (route.isEmpty()) {
                            route = null;
                            drive.stop();
                        }
                    }
                } else {
                    drive.stop();
                }
            } else {
                if (wasHeld) {
                    // Letting go stops at once.
                    leg = null;
                    route = null;
                    drive.stop();
                }
                boolean lft = aimL.pressed(gamepad1.dpad_left), rgt = aimR.pressed(gamepad1.dpad_right);
                if (spot != null && (lft || rgt)) {
                    spot.aimTrimDeg = Angle.clamp(spot.aimTrimDeg + (lft ? AIM_STEP_DEG : -AIM_STEP_DEG), -MAX_AIM_TRIM_DEG, MAX_AIM_TRIM_DEG);    // left is counterclockwise
                    reAimFrom = p.h;
                    reAimTarget = Aim.forCell(p.x, p.y, cell, spot.aimTrimDeg, 0);
                    logTrim(spot, p);
                    reAim = new DriveToPose(drive, "re-aim at grid spot " + spot.number, new Pose(p.x, p.y, reAimTarget))
                            .maxPower(REAIM_POWER).tolerance(REAIM_HOLD_IN, REAIM_TOL_DEG).stiction(true).timeout(REAIM_CAP_S);
                }
                if (reAim != null) {
                    if (!reAim.run(null)) {
                        double asked = Math.toDegrees(Angle.diff(reAimTarget, reAimFrom)), turned = Math.toDegrees(Angle.diff(p.h, reAimFrom));
                        double leftDeg = Math.toDegrees(Angle.diff(reAimTarget, p.h));
                        last = String.format(Locale.US, "RE-AIM %s: asked %+.2f deg, turned %+.2f deg, %.2f deg left", Math.abs(leftDeg) <= REAIM_STUCK_DEG ? "OK" : "STUCK", asked, turned, leftDeg);
                        RunLog.event(last + (spot == null ? "" : String.format(Locale.US, " (spot %d, aim trim %+.1f deg)", spot.number, spot.aimTrimDeg)));
                        reAim = null;
                        drive.stop();
                    }
                } else {
                    driveRobotOriented(gamepad1.right_bumper ? PRECISION : 1.0);
                }
            }
            wasHeld = held;

            // ---- The spot's buttons ----
            if (spot != null) {
                boolean faster = up.pressed(gamepad1.dpad_up), slower = down.pressed(gamepad1.dpad_down);
                if (faster || slower) {
                    spot.speedTrim += faster ? SPEED_STEP : -SPEED_STEP;
                    logTrim(spot, p);
                }
                if (fire.pressed(gamepad1.y) && mech.requestLaunch("grid tuner")) {
                    spot.salvos++;
                    RunLog.eventf("FIRE spot %d salvo %d: pose (%.1f, %.1f, %.1f deg), aim heading %.1f deg (trim %+.1f), %.1f in from %s, setpoint %.0f (map %.0f, trim %+.0f)",
                            spot.number, spot.salvos, p.x, p.y, p.headingDeg(), spot.aimDeg(), spot.aimTrimDeg, Aim.distance(p.x, p.y, cell), cell,
                            spot.mapSpeed() + spot.speedTrim, spot.mapSpeed(), spot.speedTrim);
                }
                boolean ok = accept.pressed(gamepad1.a), no = noShot.pressed(gamepad1.b);
                if (ok && !atSpot) {
                    // Accept is ignored, with a notice, unless the robot is at the spot; "no shot" works from anywhere.
                    ok = false;
                    last = "A ignored: the robot is not at spot " + spot.number + " (hold the right trigger to drive there; B = no shot from here)";
                    RunLog.event("GRID " + last);
                }
                if (ok || no) {
                    spot.status = ok ? 'G' : 'N';
                    spot.doneSpeed = ok ? spot.mapSpeed() + spot.speedTrim : Double.NaN;
                    last = pointLine(spot, sweep.size(), p);
                    RunLog.event(last);
                    index++;
                    if (CARRY_TRIMS && index < sweep.size() && sweep.get(index).status == ' ') {
                        sweep.get(index).speedTrim = spot.speedTrim;
                        sweep.get(index).aimTrimDeg = spot.aimTrimDeg;
                    }
                    route = null;
                    leg = null;
                    reAim = null;
                }
            }
            if (backOne.pressed(gamepad1.x) && index > 0) {
                index--;
                route = null;
                leg = null;
                reAim = null;
                RunLog.eventf("GRID back one spot: to spot %d", sweep.get(index).number);
            }

            mech.update(p, true);
            if (wasSalvo && !mech.salvoRunning()) RunLog.eventf("GRID salvo done at spot %s", spot == null ? "-" : String.valueOf(spot.number));
            wasSalvo = mech.salvoRunning();
            RunLog.tick();
            telemetry.addLine(spot == null ? "SWEEP DONE: " + sweep.size() + " spots. X goes back one."
                    : String.format(Locale.US, "Spot %d (%d of %d) at (%.1f, %.1f)%s", spot.number, index + 1, sweep.size(), spot.x, spot.y,
                    atSpot ? ": THERE" : String.format(Locale.US, ", %.0f in away", p.distTo(spot.x, spot.y))));
            if (spot != null) {
                telemetry.addLine(String.format(Locale.US, "speed %.0f (map %.0f, trim %+.0f) %s, aim trim %+.1f deg, off aim %+.1f deg, salvos %d", spot.mapSpeed() + spot.speedTrim,
                        spot.mapSpeed(), spot.speedTrim, mech.flywheelText(), spot.aimTrimDeg,
                        Math.toDegrees(Angle.diff(Aim.forCell(p.x, p.y, cell, spot.aimTrimDeg, 0), p.h)), spot.salvos));
            }
            telemetry.addLine("Hold RT: drive to the spot. Y fire. A accept, B no shot, X back one. Held " + mech.heldText());
            telemetry.addLine(status);
            telemetry.addLine(last);
            String notice = mech.notice();
            if (notice != null) telemetry.addLine(notice);
            telemetry.update();
        }
        // The whole table at the end, ready to paste.
        StringBuilder sb = new StringBuilder("GRID TABLE (x, y, distance in, speed ticks/s, angle trim deg, status G good / N no shot / - not done, salvos fired):");
        for (Spot s : sweep) {
            sb.append(String.format(Locale.US, " {%.1f, %.1f, %.1f, %.0f, %.1f, '%c', %d},", s.x, s.y, s.distance, s.doneSpeed, s.aimTrimDeg, s.status == ' ' ? '-' : s.status, s.salvos));
        }
        RunLog.event(sb.toString());
    }
}
