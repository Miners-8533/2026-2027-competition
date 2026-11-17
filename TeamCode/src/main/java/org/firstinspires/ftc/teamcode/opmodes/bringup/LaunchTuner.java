package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aiming.TargetLock;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.LaunchMap;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Launch Tuner (specs/support-opmodes.md section 2.1): re-measure the launch map (speed from distance) after a
 * flywheel change or a mechanical adjustment.
 *
 * Drive as in the TeleOp, with the intake always in. The cell follows the pose. The speed is the map's speed for the
 * distance plus a speed trim; the aim is the target lock plus an aim trim. Fire, adjust the trims until the salvo
 * scores, then record the point. Recording resets both trims, and each point keeps its own; undo removes the last
 * point and restores the trims.
 *
 * Controls.
 *   Driver:   sticks, left trigger = precision (30 percent); right bumper = target lock on / off (a toggle here);
 *             D-pad up / down = speed trim, 20 ticks/s a step; D-pad right / left = aim trim, 0.5 deg a step (right
 *             aims left, counterclockwise); A = fire (with the lock on it waits for aligned);
 *             Back = clear a pusher fault, and held 1 s reseed the pose to the start.
 *   Operator: left trigger = intake OUT; D-pad = the TeleOp's landed low / high / left / right reports, with these
 *             steps; A / B = record (it replaces a point within 1 in) / undo; Y = fire, without waiting for the lock;
 *             left bumper = speed base: the saved map / the points recorded this session.
 * Log lines: TUNER SALVO, TRIM, RECORD, UNDO, TUNER TABLE.
 *
 * SPEC GAP: how the alliance is chosen is not on the page; D-pad right / left at INIT chooses BLUE / RED, as in the
 * TeleOp, and the start is the alliance's default start.
 */
@TeleOp(name = "Launch Tuner", group = "2 Bring-up")
public final class LaunchTuner extends TeamOpMode {
    private static final double SPEED_STEP = 20, AIM_STEP_DEG = 0.5, PRECISION = 0.30, TRIGGER = 0.3, RESEED_HOLD_S = 1.0, SAME_POINT_IN = 1.0;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Launch Tuner"; }

    /** One recorded point. */
    private static final class Point {
        double distance, speed, speedTrim, aimTrim;
    }

    private final List<Point> points = new ArrayList<>();
    private double speedTrim, aimTrim;

    private void trim(String who, double speedStep, double aimStep) {
        speedTrim += speedStep;
        aimTrim += aimStep;
        RunLog.eventf("TRIM %s: speed trim %+.0f ticks/s, aim trim %+.1f deg", who, speedTrim, aimTrim);
    }

    private static double[][] sorted(List<Point> pts) {
        List<Point> s = new ArrayList<>(pts);
        Collections.sort(s, new Comparator<Point>() {
            @Override public int compare(Point o1, Point o2) { return Double.compare(o1.distance, o2.distance); }
        });
        double[][] table = new double[s.size()][];
        for (int i = 0; i < table.length; i++) table[i] = new double[] {s.get(i).distance, s.get(i).speed};
        return table;
    }

    /** The recorded points as lines ready to paste into the launch map. */
    private void logTable() {
        StringBuilder sb = new StringBuilder("TUNER TABLE");
        for (double[] row : sorted(points)) sb.append(String.format(Locale.US, " {%.1f, %.0f},", row[0], row[1]));
        RunLog.event(sb.toString());
    }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        Alliance alliance = Alliance.BLUE;
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, alliance.pose(Field.startRear(profile.originToBack)));
        Mechanisms.Settings s = new Mechanisms.Settings();
        s.intakeAlwaysOn = true;
        s.flywheelStopsAfterSalvo = false;
        s.targetFollowsPose = true;
        s.flyPosAtInit = true;
        mech = new Mechanisms(hardwareMap, profile, alliance, s);
        Edge right = new Edge(), left = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            mech.initLoop();
            boolean r = right.pressed(gamepad1.dpad_right), l = left.pressed(gamepad1.dpad_left);
            if (r || l) {
                alliance = r ? Alliance.BLUE : Alliance.RED;
                mech.setAlliance(alliance);
                drive.localizer.seed(alliance.pose(Field.startRear(profile.originToBack)));
            }
            telemetry.addLine("Launch Tuner, " + alliance + " (D-pad right/left). Back against the wall at the default start.");
            telemetry.addLine("Pose " + drive.pose() + ", found: " + mech.found());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        Pose start = alliance.pose(Field.startRear(profile.originToBack));
        RunLog.eventf("SETUP profile %s, alliance %s, start pose default %s, driver forward %.0f deg; Launch Tuner, mechanisms: %s",
                profile.name, alliance, drive.pose(), Math.toDegrees(alliance.driverForwardRad()), mech.found());
        mech.setSpin(true);
        TargetLock lock = new TargetLock();
        Edge lockBtn = new Edge(), fire = new Edge(), fireNow = new Edge(), up = new Edge(), down = new Edge(), aimL = new Edge(), aimR = new Edge(),
                rec = new Edge(), undo = new Edge(), base = new Edge(), back = new Edge(),
                repL = new Edge(), repR = new Edge(), repD = new Edge(), repU = new Edge();
        Hold reseed = new Hold();
        double lastT = Clock.now(), launchSpeed = 0, lowest = 0;
        boolean lockOn = false, waitingForAim = false, sessionBase = false, wasSalvo = false;
        int salvos = 0;
        String lastSalvo = "";
        while (opModeIsActive()) {
            double t = Clock.now(), dt = Math.max(1e-3, t - lastT);
            lastT = t;
            drive.updatePose();
            Pose p = drive.pose();
            Cell cell = mech.targetCell() != null ? mech.targetCell() : Cell.of(alliance, true);

            // ---- The driver ----
            if (up.pressed(gamepad1.dpad_up)) trim("driver", SPEED_STEP, 0);
            if (down.pressed(gamepad1.dpad_down)) trim("driver", -SPEED_STEP, 0);
            if (aimR.pressed(gamepad1.dpad_right)) trim("driver", 0, AIM_STEP_DEG);       // right aims left, counterclockwise
            if (aimL.pressed(gamepad1.dpad_left)) trim("driver", 0, -AIM_STEP_DEG);
            if (lockBtn.pressed(gamepad1.right_bumper)) {
                lockOn = !lockOn;
                waitingForAim &= lockOn;
                RunLog.event("TARGET LOCK " + (lockOn ? "on" : "off") + " (tuner toggle), cell " + cell.label());
            }
            if (back.pressed(gamepad1.back)) mech.clearPusherStall();
            if (reseed.held(gamepad1.back, t, RESEED_HOLD_S)) {
                drive.stop();
                drive.localizer.setPose(start);
                RunLog.eventf("PINPOINT reseeded by the driver to %s", start);
            }
            driveByHand(alliance, gamepad1.left_trigger > TRIGGER ? PRECISION : 1.0);
            if (lockOn) {
                double turn = lock.update(p, drive.localizer.omega(), cell, aimTrim, 0, dt, Double.NaN);
                drive.drive(drive.cmdForward(), drive.cmdLeft(), turn);
            } else {
                lock.release();
            }
            // A: fire. With the lock on it waits for aligned.
            if (fire.pressed(gamepad1.a)) {
                if (lockOn && !lock.aligned()) waitingForAim = true;
                else mech.requestLaunch("tuner, driver");
            }
            if (waitingForAim && lock.aligned()) {
                waitingForAim = false;
                mech.requestLaunch("tuner, driver, once aligned");
            }

            // ---- The operator ----
            mech.setOuttake(gamepad2.left_trigger > TRIGGER);
            if (repL.pressed(gamepad2.dpad_left)) trim("operator, landed LEFT", 0, -AIM_STEP_DEG);     // landed left: aim clockwise
            if (repR.pressed(gamepad2.dpad_right)) trim("operator, landed RIGHT", 0, AIM_STEP_DEG);
            if (repD.pressed(gamepad2.dpad_down)) trim("operator, landed LOW", SPEED_STEP, 0);
            if (repU.pressed(gamepad2.dpad_up)) trim("operator, landed HIGH", -SPEED_STEP, 0);
            if (fireNow.pressed(gamepad2.y)) {
                waitingForAim = false;
                mech.requestLaunch("tuner, operator, not waiting for the lock");
            }
            mech.setSpeedTrim(speedTrim);
            if (rec.pressed(gamepad2.a)) {
                Point pt = new Point();
                pt.distance = mech.distance();
                pt.speed = mech.setpoint();
                pt.speedTrim = speedTrim;
                pt.aimTrim = aimTrim;
                boolean replaced = false;
                for (int i = 0; i < points.size() && !replaced; i++) {
                    if (Math.abs(points.get(i).distance - pt.distance) <= SAME_POINT_IN) {
                        points.set(i, pt);
                        replaced = true;
                    }
                }
                if (!replaced) points.add(pt);
                RunLog.eventf("RECORD point %d%s: distance %.1f in, speed %.0f (speed trim %+.0f), aim trim %+.1f deg, pose %s, cell %s, battery %.2f V",
                        points.size(), replaced ? " (replaces a point within 1 in)" : "", pt.distance, pt.speed, speedTrim, aimTrim, p, cell.label(), battery());
                logTable();
                speedTrim = 0;      // each point keeps its own trims
                aimTrim = 0;
                if (sessionBase) LaunchMap.use(sorted(points));
            }
            if (undo.pressed(gamepad2.b) && !points.isEmpty()) {
                Point last = points.remove(points.size() - 1);
                speedTrim = last.speedTrim;
                aimTrim = last.aimTrim;
                RunLog.eventf("UNDO: the last point (%.1f in, %.0f) is removed; trims restored to %+.0f ticks/s, %+.1f deg", last.distance, last.speed, speedTrim, aimTrim);
                if (sessionBase && points.isEmpty()) sessionBase = false;
                LaunchMap.use(sessionBase ? sorted(points) : null);
            }
            if (base.pressed(gamepad2.left_bumper)) {
                sessionBase = !sessionBase && !points.isEmpty();
                LaunchMap.use(sessionBase ? sorted(points) : null);
                RunLog.event("TUNER speed base: " + (sessionBase ? "the points recorded this session" : "the saved map"));
            }

            mech.update(p, true);

            // One log line per salvo.
            double speedNow = mech.flywheel == null ? 0 : mech.flywheel.filtered();
            if (mech.salvoRunning() && !wasSalvo) launchSpeed = lowest = speedNow;
            if (mech.salvoRunning()) lowest = Math.min(lowest, speedNow);
            if (!mech.salvoRunning() && wasSalvo) {
                salvos++;
                lastSalvo = String.format(Locale.US, "launch speed %.0f, lowest %.0f (dip %.0f), battery %.2f V, flywheel %.1f A, distance %.1f in, pose %s, lock %s, trims %+.0f ticks/s / %+.1f deg",
                        launchSpeed, lowest, launchSpeed - lowest, battery(), mech.flywheel == null ? 0 : mech.flywheel.amps(), mech.distance(), p,
                        lockOn ? String.format(Locale.US, "error %.1f deg", lock.errorDeg()) : "off", speedTrim, aimTrim);
                RunLog.event("TUNER SALVO " + salvos + ": " + lastSalvo);
            }
            wasSalvo = mech.salvoRunning();
            RunLog.tick();
            telemetry.addLine(String.format(Locale.US, "%s %s %.1f in, setpoint %.0f, %s", alliance, cell.label(), mech.distance(), mech.setpoint(), mech.flywheelText()));
            telemetry.addLine(String.format(Locale.US, "speed trim %+.0f, aim trim %+.1f deg, lock %s%s", speedTrim, aimTrim,
                    lockOn ? String.format(Locale.US, "ON (%.1f deg)", lock.errorDeg()) : "off", waitingForAim ? ", WAITING FOR AIM" : ""));
            telemetry.addLine("points " + points.size() + ", base: " + (sessionBase ? "session" : "saved map") + ", held " + mech.heldText());
            telemetry.addLine("last salvo: " + lastSalvo);
            String notice = mech.notice();
            if (notice != null) telemetry.addLine(notice);
            telemetry.update();
        }
        LaunchMap.use(null);
        logTable();
    }
}
