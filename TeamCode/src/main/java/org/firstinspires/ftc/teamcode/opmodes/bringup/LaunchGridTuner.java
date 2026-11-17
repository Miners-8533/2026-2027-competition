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
 * stand, a short straight hop is used if it is clear, otherwise the operator drives by hand. Small re-aims use drive
 * to pose with its static friction term, and each reports whether it turned or stuck.
 *
 * Controls (gamepad 1). At INIT, D-pad left / right chooses NEAR (the rear cell) / FAR (the audience cell); the start
 * is against the rear wall, at (-63.4, 16.5, 0 deg). Right trigger (hold): drive to the next spot; letting go stops
 * at once. Sticks: drive by hand, robot-oriented, while the trigger is not held; right bumper precision (30 percent).
 * D-pad left / right: aim left / right by 0.5 deg. D-pad up / down: speed, 20 ticks/s a step. Y: fire.
 * A / B / X: accept the spot / no shot is possible here / back one spot. Left trigger (hold): intake OUT.
 * Back: clear a pusher fault; held 1 s, reseed the pose to the start.
 * Log lines: GRID POINT, FIRE, GRID TABLE.
 *
 * A sweep can be continued from a later spot, restricted to a list of spot numbers, or extended with extra positions,
 * by the settings below, read at INIT.
 *
 * SPEC GAP, each a choice of this rebuild:
 *  - the wording of the GRID POINT, FIRE and GRID TABLE lines (a PC tool turns the log into the grid table): they
 *    hold what the page lists, and the table rows have the layout of the generated table (specs/aiming.md 4.1);
 *  - the names of the three settings; that the trims start at 0 on every spot; the powers of the drive to a spot;
 *  - a "short straight hop": at most 24 in, with the robot's outline clear of every support and flower stand along it;
 *  - a re-aim has "turned" when the heading moved at least half of what was asked.
 */
@Config
@TeleOp(name = "Launch Grid Tuner", group = "2 Bring-up")
public final class LaunchGridTuner extends TeamOpMode {
    /** Continue a sweep from this spot number (1 = the first). */
    public static int START_AT_SPOT = 1;
    /** Restrict the sweep to these spot numbers, for example "3,7,12". Empty = all. */
    public static String ONLY_SPOTS = "";
    /** Extra positions after the generated spots: x1, y1, x2, y2, ... (blue frame). */
    public static double[] EXTRA_SPOTS = {};
    public static double START_Y = 16.5;
    public static double DRIVE_POWER = 0.6, HOP_MAX_IN = 24.0, AT_SPOT_IN = 3.0;
    private static final double SPEED_STEP = 20, AIM_STEP_DEG = 0.5, PRECISION = 0.30, TRIGGER = 0.3, RESEED_HOLD_S = 1.0;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Launch Grid Tuner"; }

    /** One spot of the sweep and what was found there. */
    private static final class Spot {
        int number;
        Vec at;
        String result;          // null = not done; "OK" or "NO SHOT"
        int salvos;
        double speedTrim, aimTrimDeg;
        // As accepted:
        Pose pose;
        double mapSpeed, speed, aimDeg, formulaDeg, residualDeg, distance;
    }

    private Footprint footprint;
    private Cell cell;

    private List<Spot> buildSweep() {
        List<Vec> all = new ArrayList<>(LaunchGridSpots.spots(cell, new LaunchGridSpots.Fit() {
            @Override public boolean clearOfObstacles(double x, double y, double heading) {
                return RoutePlanner.bodyClear(new Pose(x, y, heading), footprint, RoutePlanner.MARGIN_IN);
            }

            @Override public boolean insideWalls(double x, double y, double heading, double inset) {
                for (double[] c : footprint.corners(x, y, heading)) {
                    if (Math.abs(c[0]) > Field.HALF_IN - inset || Math.abs(c[1]) > Field.HALF_IN - inset) return false;
                }
                return true;
            }
        }));
        for (int k = 0; k + 1 < EXTRA_SPOTS.length; k += 2) all.add(new Vec(EXTRA_SPOTS[k], EXTRA_SPOTS[k + 1]));
        List<Integer> only = new ArrayList<>();
        for (String s : ONLY_SPOTS.split("[,; ]+")) {
            try {
                if (!s.isEmpty()) only.add(Integer.parseInt(s.trim()));
            } catch (NumberFormatException ignored) { }
        }
        List<Spot> sweep = new ArrayList<>();
        for (int k = 0; k < all.size(); k++) {
            int number = k + 1;
            if (number < START_AT_SPOT || (!only.isEmpty() && !only.contains(number))) continue;
            Spot s = new Spot();
            s.number = number;
            s.at = all.get(k);
            sweep.add(s);
        }
        return sweep;
    }

    /** A short straight hop: the robot's outline, turned to the heading, is clear of every support and flower stand along it. */
    private boolean hopClear(Vec from, Vec to, double heading) {
        if (from.dist(to) > HOP_MAX_IN) return false;
        for (int k = 0; k <= 6; k++) {
            double f = k / 6.0;
            if (!RoutePlanner.bodyClear(new Pose(from.x + (to.x - from.x) * f, from.y + (to.y - from.y) * f, heading), footprint, 0)) return false;
        }
        return true;
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
        Edge l = new Edge(), r = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            mech.initLoop();
            if (l.pressed(gamepad1.dpad_left)) cell = Cell.BLUE_REAR;
            if (r.pressed(gamepad1.dpad_right)) cell = Cell.BLUE_AUDIENCE;
            telemetry.addLine("Launch Grid Tuner, BLUE. D-pad left = NEAR (the rear cell), right = FAR (the audience cell): " + (cell.rear ? "NEAR" : "FAR"));
            telemetry.addLine("Start against the rear wall at " + start + ", still. Pose " + drive.pose() + (drive.localizer.seedConfirmed() ? "" : " SEED NOT CONFIRMED"));
            telemetry.addLine("Settings: start at spot " + START_AT_SPOT + ", only \"" + ONLY_SPOTS + "\", " + EXTRA_SPOTS.length / 2 + " extra. Found: " + mech.found());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        List<Spot> sweep = buildSweep();
        RunLog.eventf("SETUP profile %s, alliance %s, start pose default %s, driver forward %.0f deg; Launch Grid Tuner, cell %s (%s), %d spots (start at %d, only \"%s\", %d extra)",
                profile.name, alliance, drive.pose(), Math.toDegrees(alliance.driverForwardRad()), cell.label(), cell.rear ? "NEAR" : "FAR", sweep.size(),
                START_AT_SPOT, ONLY_SPOTS, EXTRA_SPOTS.length / 2);
        mech.setTargetCell(cell);
        mech.setSpin(true);
        Edge aimL = new Edge(), aimR = new Edge(), up = new Edge(), down = new Edge(), fire = new Edge(), accept = new Edge(), noShot = new Edge(), backOne = new Edge(), back = new Edge();
        Hold reseed = new Hold();
        int index = 0, fires = 0;
        List<Pose> route = null;        // the poses still to pass on the way to the spot
        DriveToPose leg = null, reAim = null;
        double reAimFrom = 0, reAimAsked = 0;
        boolean wasSalvo = false, wasHeld = false;
        String status = "", last = "";
        while (opModeIsActive()) {
            double t = Clock.now();
            drive.updatePose();
            Pose p = drive.pose();
            Spot spot = index < sweep.size() ? sweep.get(index) : null;
            boolean atSpot = spot != null && p.distTo(spot.at.x, spot.at.y) <= AT_SPOT_IN;
            double aimHere = Aim.forCell(p.x, p.y, cell, spot == null ? 0 : spot.aimTrimDeg, 0);
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
                    // Plan once, when the trigger is pulled, from where the robot is. The spot's heading is the aim from the spot itself.
                    double h = Aim.forCell(spot.at.x, spot.at.y, cell, spot.aimTrimDeg, 0);
                    Plan plan = RoutePlanner.plan(p.vec(), spot.at, footprint);
                    route = new ArrayList<>();
                    if (plan.ok()) {
                        for (int k = 1; k < plan.points.length; k++) route.add(new Pose(plan.points[k].x, plan.points[k].y, h));
                        status = "route: " + plan.describe();
                    } else if (hopClear(p.vec(), spot.at, h)) {
                        route.add(new Pose(spot.at.x, spot.at.y, h));
                        status = "the planner refused (" + plan.refusal + "): a short straight hop";
                    } else {
                        route = null;
                        status = "NO ROUTE (" + plan.refusal + ") and no clear hop: drive by hand";
                    }
                    RunLog.eventf("GRID drive to spot %d at %s from %s: %s", spot.number, spot.at, p, status);
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
                    spot.aimTrimDeg += lft ? AIM_STEP_DEG : -AIM_STEP_DEG;      // left is counterclockwise
                    double target = Aim.forCell(p.x, p.y, cell, spot.aimTrimDeg, 0);
                    reAimFrom = p.h;
                    reAimAsked = Math.toDegrees(Angle.diff(target, p.h));
                    reAim = new DriveToPose(drive, "re-aim at grid spot " + spot.number, new Pose(p.x, p.y, target)).stiction(true).timeout(1.5);
                }
                if (reAim != null) {
                    if (!reAim.run(null)) {
                        double turned = Math.toDegrees(Angle.diff(p.h, reAimFrom));
                        boolean ok = Math.abs(turned) >= 0.5 * Math.abs(reAimAsked) && turned * reAimAsked >= 0;
                        last = String.format(Locale.US, "re-aim asked %+.1f deg, turned %+.1f deg: %s", reAimAsked, turned, ok ? "turned" : "STUCK");
                        RunLog.eventf("GRID re-aim at spot %d: asked %+.1f deg, turned %+.1f deg: %s (aim trim %+.1f deg)", spot.number, reAimAsked, turned,
                                ok ? "turned" : "STUCK", spot.aimTrimDeg);
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
                if (up.pressed(gamepad1.dpad_up)) spot.speedTrim += SPEED_STEP;
                if (down.pressed(gamepad1.dpad_down)) spot.speedTrim -= SPEED_STEP;
                if (fire.pressed(gamepad1.y) && mech.requestLaunch("grid tuner")) {
                    fires++;
                    spot.salvos++;
                    RunLog.eventf("FIRE %d: spot %d, salvo %d there, speed %.0f (map %.0f, trim %+.0f), aim %.2f deg (trim %+.1f), heading %.2f deg, distance %.1f in, pose %s, battery %.2f V",
                            fires, spot.number, spot.salvos, mech.setpoint(), mech.mapSpeed(), spot.speedTrim, Math.toDegrees(aimHere), spot.aimTrimDeg,
                            p.headingDeg(), mech.distance(), p, battery());
                }
                boolean ok = accept.pressed(gamepad1.a), no = noShot.pressed(gamepad1.b);
                if (ok || no) {
                    spot.result = ok ? "OK" : "NO SHOT";
                    spot.pose = p;
                    spot.mapSpeed = mech.mapSpeed();
                    spot.speed = mech.setpoint();
                    spot.formulaDeg = Math.toDegrees(Aim.forCell(p.x, p.y, cell, 0, 0));
                    spot.aimDeg = p.headingDeg();
                    spot.residualDeg = Math.toDegrees(Angle.diff(p.h, Aim.bearing(p.x, p.y, cell)));
                    spot.distance = Aim.distance(p.x, p.y, cell);
                    last = String.format(Locale.US, "GRID POINT %d: %s%s, speed %.0f (map %.0f, trim %+.0f), aim heading %.2f deg (formula %.2f, trim %+.1f), residual %+.2f deg, distance %.1f in, %d salvo(s), pose %s, spot %s",
                            spot.number, spot.result, ok && spot.salvos == 0 ? " (unverified: no salvo was fired here)" : "", spot.speed, spot.mapSpeed, spot.speedTrim,
                            spot.aimDeg, spot.formulaDeg, spot.aimTrimDeg, spot.residualDeg, spot.distance, spot.salvos, p, spot.at);
                    RunLog.event(last);
                    index++;
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
                    : String.format(Locale.US, "Spot %d (%d of %d) at %s%s", spot.number, index + 1, sweep.size(), spot.at, atSpot ? ": THERE" : String.format(Locale.US, ", %.0f in away", p.distTo(spot.at.x, spot.at.y))));
            if (spot != null) {
                telemetry.addLine(String.format(Locale.US, "speed %.0f (map %.0f, trim %+.0f) %s, aim trim %+.1f deg, off aim %+.1f deg, salvos %d", mech.setpoint(), mech.mapSpeed(),
                        spot.speedTrim, mech.flywheelText(), spot.aimTrimDeg, Math.toDegrees(Angle.diff(aimHere, p.h)), spot.salvos));
            }
            telemetry.addLine("Hold RT: drive to the spot. Y fire. A accept, B no shot, X back one. Held " + mech.heldText());
            telemetry.addLine(status);
            telemetry.addLine(last);
            String notice = mech.notice();
            if (notice != null) telemetry.addLine(notice);
            telemetry.update();
        }
        // The table at the end: one row per finished spot, in the layout of the generated grid table.
        int done = 0;
        for (Spot s : sweep) if (s.result != null) done++;
        RunLog.eventf("GRID TABLE cell %s, %d of %d spots; rows are {x, y, speed ticks/s, aim residual deg, shot possible, estimated}", cell.label(), done, sweep.size());
        for (Spot s : sweep) {
            if (s.result == null) continue;
            RunLog.eventf("GRID TABLE {%.1f, %.1f, %.0f, %.2f, %d, 0},  // spot %d%s", s.pose.x, s.pose.y, s.speed, s.residualDeg, s.result.equals("OK") ? 1 : 0, s.number,
                    s.result.equals("OK") && s.salvos == 0 ? ", unverified" : "");
        }
    }
}
