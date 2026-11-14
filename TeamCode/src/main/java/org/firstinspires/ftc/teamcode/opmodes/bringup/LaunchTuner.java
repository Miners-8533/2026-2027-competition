package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpCompetition;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.LaunchMap;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

/**
 * Launch Tuner (2026-11-14): for tuning the ONE-DIMENSIONAL launch map after a flywheel change or a mechanical adjustment. The driver drives the robot (field-oriented, as in the
 * TeleOp), the flywheel speed is the saved {@link LaunchMap} speed for the distance to the cell the pose picks, PLUS a speed trim; the aim is the target lock's aim PLUS an aim trim.
 * The intake is ALWAYS IN (as in the TeleOp), the operator's left trigger runs it OUT. Nothing is assumed about the number of elements: load it as you like.
 *
 * The cell is the closer of the BLUE alliance's two cells, from the pose (blue is assumed, the usual start against the rear wall, as in the Launch Grid Tuner); the screen shows which one and
 * the distance. The speed is the saved map's speed at that distance + the speed trim (the trim is the thing tuned), or, with the operator's left bumper, the map of the points recorded THIS
 * session (interpolated) + the trim. The aim trim applies while the target lock is on.
 *
 * RECORD (operator A): stores the point (distance, the speed then = map speed + speed trim, both trims, the pose, the cell, the battery), replacing a point within 1 in, then BOTH TRIMS RESET to 0
 * (the trim of each point is kept in its record, so you can see how far each point is from the saved map). UNDO (operator B) removes the last record and puts the trims back as they were
 * before it. The recorded points are listed on the screen and logged as lines to paste into LaunchMap.TABLE.
 *
 * Gamepad 1 (driver):
 *   Left stick / right stick X   drive / turn, FIELD-oriented (the TeleOp's)         Left trigger (hold): precision (30%)
 *   Right bumper                 target lock ON / OFF (a toggle here; held in the TeleOp). With it on the robot turns to the cell by itself
 *   A                            fire a salvo (with the lock on it waits for ALIGNED, at most LAUNCH_ALIGN_WAIT_SEC)
 *   D-pad up / down              speed trim +/- SPEED_STEP (10) ticks/s         D-pad right / left   aim trim +/- AIM_STEP_DEG (0.5) deg (right = aim LEFT, counterclockwise)
 *   Back                         clear a pusher fault; HOLD it 1 s to reseed the pose to the start pose (square the robot on the rear wall first)
 * Gamepad 2 (operator):
 *   A                            RECORD this point                B   UNDO the last record               Y   fire (same as the driver's A, without waiting for the lock)
 *   Left trigger (hold)          OUTTAKE                          Left bumper   speed source: saved map / points recorded this session
 *   D-pad (REPORTS, as in the TeleOp, seen from behind the robot):  down = landed LOW (speed +10)   up = landed HIGH (speed -10)   left = landed LEFT (aim right 0.5 deg)   right = landed RIGHT (aim left 0.5 deg)
 *
 * The log: a SALVO line per salvo (launch speed, lowest speed and the dip, battery, flywheel current, distance, pose, lock error, both trims), TRIM, RECORD and UNDO lines, and at the end a
 * TUNER TABLE line to paste. The pusher is assumed to be where the last opmode left it (the saved zero).
 */
@Config
@TeleOp(name = "Launch Tuner", group = "Bring-up")
public class LaunchTuner extends LinearOpMode {

    /** The usual blue start: against the rear wall facing the audience wall (the Launch Grid Tuner's, the TeleOp's blue start). */
    public static double START_X_IN = -63.4;
    public static double START_Y_IN = 16.5;
    public static double START_HEADING_DEG = 0.0;
    public static double SPEED_STEP = 10.0;
    public static double AIM_STEP_DEG = 0.5;
    public static double MAX_AIM_TRIM_DEG = 15.0;
    /** With the lock on and not ALIGNED, a launch waits this long for ALIGNED (then it is dropped). */
    public static double LAUNCH_ALIGN_WAIT_SEC = 1.5;
    /** A recorded point within this many inches of the new one is replaced. */
    public static double REPLACE_WITHIN_IN = 1.0;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double RESEED_HOLD_SEC = 1.0;
    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;

    /** One recorded point: distance, speed, speed trim, aim trim, x, y, heading, battery, cell. */
    private static final class Point {
        final double distance, speed, speedTrim, aimTrim, x, y, headingDeg, battery;
        final HiveTarget cell;

        Point(double distance, double speed, double speedTrim, double aimTrim, double x, double y, double headingDeg, double battery, HiveTarget cell) {
            this.distance = distance;
            this.speed = speed;
            this.speedTrim = speedTrim;
            this.aimTrim = aimTrim;
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
            this.battery = battery;
            this.cell = cell;
        }
    }

    /** What an UNDO restores: the points as they were and the trims as they were before the record. */
    private static final class Undo {
        final List<Point> points;
        final double speedTrim, aimTrim;

        Undo(List<Point> points, double speedTrim, double aimTrim) {
            this.points = points;
            this.speedTrim = speedTrim;
            this.aimTrim = aimTrim;
        }
    }

    // what the log's sample rows show
    private double logX, logY, logHeadingDeg, logDistance, logBase, logSpeedTrim, logAimTrim, logLockErr = Double.NaN;
    private String logCell = "", logSource = "";

    private static Pose2d startPose() {
        return new Pose2d(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
    }

    private static double battery(VoltageSensor sensor) {
        return sensor == null ? Double.NaN : sensor.getVoltage();
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Launch Tuner", hardwareMap);
        try {
            runTuner();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runTuner() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose());
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);
        mechanisms.setUseLaunchMap(false);          // this opmode works out the speed itself: the map (or the session's points) + the trim
        mechanisms.setAutoStopAfterSalvo(false);    // the flywheel runs the whole time
        mechanisms.setBlueAlliance(true);           // assumed: blue
        mechanisms.setAutoTarget(true);             // the cell follows the pose (the closer of blue's two), as in the TeleOp
        mechanisms.setIntakeAlwaysOn(true);         // IN whatever the count; only the operator's left trigger runs it OUT
        mechanisms.setOpenCell(HiveTarget.BLUE_REAR);
        mechanisms.setSpinRequested(true);

        VoltageSensor batterySensor = null;
        Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        if (sensors.hasNext()) batterySensor = sensors.next();

        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();   // reads only; nothing moves in INIT
            Pose2d p = drive.getPose();
            Pose2d want = startPose();
            boolean poseOk = Math.hypot(p.position.x - want.position.x, p.position.y - want.position.y) < 1.5;
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Alliance", "BLUE (assumed)");
            telemetry.addData("Start pose to use", "X %.1f, Y %.2f in, heading %.0f deg (START_X_IN, START_Y_IN, START_HEADING_DEG in the Dashboard)",
                    want.position.x, want.position.y, Math.toDegrees(want.heading.toDouble()));
            telemetry.addData("Pinpoint pose", "%.1f, %.1f, %.0f deg   %s", p.position.x, p.position.y, p.heading.toDegrees(), poseOk ? "OK" : "WARNING: not at the start pose");
            telemetry.addData("Saved LaunchMap.TABLE", LaunchMap.isEmpty() ? "EMPTY: the speed will be the 1350 ticks/s fallback" : LaunchMap.TABLE.length + " points");
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addLine("Place the robot at the start pose and keep still while the Pinpoint calibrates. Then Start.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.event("SETUP launch tuner, BLUE assumed, start pose (" + START_X_IN + ", " + START_Y_IN + ", " + START_HEADING_DEG + " deg), saved map "
                + (LaunchMap.isEmpty() ? "EMPTY" : LaunchMap.TABLE.length + " points") + ", " + mechanisms.describe());
        RunLog.setSampler("x,y,hdg_deg,cell,distance,source,base_speed,speed_trim,aim_trim_deg,lock_err_deg," + Mechanisms.SAMPLE_COLUMNS, 200,
                () -> String.format(Locale.US, "%.1f,%.1f,%.1f,%s,%.1f,%s,%.0f,%.0f,%.1f,%.1f,%s", logX, logY, logHeadingDeg, logCell, logDistance, logSource, logBase, logSpeedTrim,
                        logAimTrim, logLockErr, mechanisms.sampleRow()));

        TargetLock targetLock = new TargetLock();
        double driverForwardRad = Math.toRadians(TeleOpCompetition.BLUE_DRIVER_FORWARD_DEG);
        List<Point> points = new ArrayList<>();
        List<Undo> undos = new ArrayList<>();
        double speedTrim = 0.0, aimTrim = 0.0;
        boolean useSession = false;
        boolean lockOn = false, launchWaiting = false;
        ElapsedTime launchAlignClock = new ElapsedTime();
        ElapsedTime loopClock = new ElapsedTime();
        boolean backDown = false, reseeded = false;
        ElapsedTime backHold = new ElapsedTime();

        // the per-salvo log
        boolean salvoWas = false;
        int salvoNo = 0;
        double salvoStartSpeed = Double.NaN, salvoStartVolts = Double.NaN, salvoStartAmps = Double.NaN, salvoStartDist = Double.NaN, salvoStartLockErr = Double.NaN;
        double salvoStartSetpoint = Double.NaN, salvoStartX = 0, salvoStartY = 0, salvoStartHdg = 0, salvoStartSpeedTrim = 0, salvoStartAimTrim = 0;
        String salvoStartCell = "";
        String lastSalvo = "none yet";
        String notice = "";
        ElapsedTime noticeClock = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = loopClock.seconds();
            loopClock.reset();
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            double heading = pose.heading.toDouble();

            // ---- driver: field-oriented drive, as in the TeleOp ----
            double scale = gamepad1.left_trigger > TRIGGER_THRESHOLD ? PRECISION_SCALE : 1.0;
            double up = -gamepad1.left_stick_y * scale;
            double right = gamepad1.left_stick_x * scale;
            double fieldX = up * Math.cos(driverForwardRad) + right * Math.cos(driverForwardRad - Math.PI / 2.0);
            double fieldY = up * Math.sin(driverForwardRad) + right * Math.sin(driverForwardRad - Math.PI / 2.0);
            double robotForward = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
            double robotLeft = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);

            // target lock: a toggle on the driver's right bumper
            if (gamepad1.rightBumperWasPressed()) {
                lockOn = !lockOn;
                if (!lockOn) {
                    targetLock.release();
                    launchWaiting = false;
                }
                RunLog.event("COMMAND target lock " + (lockOn ? "ON" : "off") + " (driver right bumper)");
            }
            HiveTarget cell = mechanisms.getOpenCell();
            double distance = cell.distanceFrom(pose);
            double turn;
            if (lockOn) {
                turn = targetLock.turn(pose, 0.0, cell, TargetLock.farCellExtraLeftIn(cell, true), aimTrim, 0.0, dt);
                logLockErr = targetLock.errorDeg();
            } else {
                turn = -gamepad1.right_stick_x * TURN_SCALE * scale;
                logLockErr = Double.NaN;
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotForward, robotLeft), turn));

            if (gamepad1.backWasPressed()) mechanisms.clearPusherStall();
            if (gamepad1.back) {
                if (!backDown) {
                    backDown = true;
                    reseeded = false;
                    backHold.reset();
                }
                if (!reseeded && backHold.seconds() >= RESEED_HOLD_SEC) {
                    drive.setPose(startPose());
                    reseeded = true;
                    RunLog.event("POSE reseeded to the start pose");
                }
            } else {
                backDown = false;
            }

            // ---- trims: the driver's D-pad ADJUSTS, the operator's D-pad REPORTS where the salvo landed ----
            double dSpeed = 0.0, dAim = 0.0;
            if (gamepad1.dpadUpWasPressed()) dSpeed += SPEED_STEP;
            if (gamepad1.dpadDownWasPressed()) dSpeed -= SPEED_STEP;
            if (gamepad1.dpadRightWasPressed()) dAim += AIM_STEP_DEG;   // counterclockwise = aim left
            if (gamepad1.dpadLeftWasPressed()) dAim -= AIM_STEP_DEG;
            String how = "driver";
            if (gamepad2.dpadDownWasPressed()) {
                dSpeed += SPEED_STEP;   // landed LOW: more speed
                how = "operator reports LOW";
            }
            if (gamepad2.dpadUpWasPressed()) {
                dSpeed -= SPEED_STEP;   // landed HIGH: less speed
                how = "operator reports HIGH";
            }
            if (gamepad2.dpadLeftWasPressed()) {
                dAim -= AIM_STEP_DEG;   // landed LEFT: aim right
                how = "operator reports LEFT";
            }
            if (gamepad2.dpadRightWasPressed()) {
                dAim += AIM_STEP_DEG;   // landed RIGHT: aim left
                how = "operator reports RIGHT";
            }
            if (dSpeed != 0.0 || dAim != 0.0) {
                speedTrim += dSpeed;
                aimTrim = Math.max(-MAX_AIM_TRIM_DEG, Math.min(MAX_AIM_TRIM_DEG, aimTrim + dAim));
                RunLog.event(String.format(Locale.US, "TRIM (%s): speed trim %+.0f ticks/s, aim trim %+.1f deg | %s at %.1f in, pose (%.1f, %.1f, %.0f deg)", how, speedTrim, aimTrim, cell, distance,
                        pose.position.x, pose.position.y, pose.heading.toDegrees()));
            }

            // ---- the speed source and the speed ----
            if (gamepad2.leftBumperWasPressed()) {
                useSession = !useSession;
                RunLog.event("COMMAND speed source -> " + (useSession ? "the points recorded this session" : "the saved launch map") + (useSession && points.isEmpty() ? " (none yet: the saved map is used)" : ""));
            }
            double saved = LaunchMap.velocityFor(distance);
            if (Double.isNaN(saved)) saved = 1350.0;
            double sessionSpeed = Double.NaN;
            if (!points.isEmpty()) {
                List<double[]> table = new ArrayList<>();
                for (Point p : points) table.add(new double[]{p.distance, p.speed});
                table.sort((a, b) -> Double.compare(a[0], b[0]));
                sessionSpeed = LaunchMap.interpolate(table, distance);
            }
            boolean fromSession = useSession && !Double.isNaN(sessionSpeed);
            double base = fromSession ? sessionSpeed : saved;
            double setpoint = Math.max(0.0, base + speedTrim);
            mechanisms.setManualSetpoint(setpoint);

            // ---- operator: intake out, fire, record, undo ----
            mechanisms.setManualOuttake(gamepad2.left_trigger > TRIGGER_THRESHOLD);
            boolean fire = false;
            if (gamepad1.aWasPressed()) {
                if (lockOn && !targetLock.isAligned()) {
                    launchWaiting = true;
                    launchAlignClock.reset();
                } else {
                    fire = true;
                }
            }
            if (launchWaiting) {
                if (!lockOn) {
                    launchWaiting = false;
                } else if (targetLock.isAligned()) {
                    launchWaiting = false;
                    fire = true;
                } else if (launchAlignClock.seconds() > LAUNCH_ALIGN_WAIT_SEC) {
                    launchWaiting = false;
                    notice = String.format(Locale.US, "launch dropped: not aligned after %.1f s", LAUNCH_ALIGN_WAIT_SEC);
                    noticeClock.reset();
                    RunLog.event("LAUNCH queue dropped: " + notice);
                }
            }
            if (gamepad2.yWasPressed()) fire = true;
            if (fire) {
                RunLog.event(String.format(Locale.US, "LAUNCH requested: %s at %.1f in, pose (%.1f, %.1f, %.0f deg), speed %.0f = %s %.0f %+.0f trim, aim trim %+.1f deg, lock %s",
                        cell, distance, pose.position.x, pose.position.y, pose.heading.toDegrees(), setpoint, fromSession ? "session points" : "saved map", base, speedTrim, aimTrim,
                        lockOn ? String.format(Locale.US, "on err %+.1f deg", targetLock.errorDeg()) : "off"));
                mechanisms.requestLaunch();
            }

            if (gamepad2.aWasPressed()) {
                undos.add(new Undo(new ArrayList<>(points), speedTrim, aimTrim));
                final double d = distance;
                points.removeIf(p -> Math.abs(p.distance - d) < REPLACE_WITHIN_IN);
                double volts = battery(batterySensor);
                points.add(new Point(distance, setpoint, speedTrim, aimTrim, pose.position.x, pose.position.y, pose.heading.toDegrees(), volts, cell));
                points.sort((a, b) -> Double.compare(a.distance, b.distance));
                RunLog.event(String.format(Locale.US, "RECORD %.1f in to %s at %.0f ticks/s (%s %.0f, speed trim %+.0f, aim trim %+.1f deg; pose %.1f, %.1f, %.0f deg; battery %.2f V); both trims reset to 0",
                        distance, cell, setpoint, fromSession ? "session points" : "saved map", base, speedTrim, aimTrim, pose.position.x, pose.position.y, pose.heading.toDegrees(), volts));
                speedTrim = 0.0;
                aimTrim = 0.0;
            }
            if (gamepad2.bWasPressed()) {
                if (undos.isEmpty()) {
                    notice = "nothing to undo";
                    noticeClock.reset();
                } else {
                    Undo u = undos.remove(undos.size() - 1);
                    points.clear();
                    points.addAll(u.points);
                    speedTrim = u.speedTrim;
                    aimTrim = u.aimTrim;
                    RunLog.event(String.format(Locale.US, "UNDO the last record: %d points left, the trims are back to speed %+.0f, aim %+.1f deg", points.size(), speedTrim, aimTrim));
                }
            }

            // ---- per-salvo log ----
            boolean salvoNow = mechanisms.isSalvoActive();
            if (salvoNow && !salvoWas) {
                salvoStartSpeed = mechanisms.flywheel == null ? Double.NaN : mechanisms.flywheel.getVelocity();
                salvoStartAmps = mechanisms.flywheel == null ? Double.NaN : mechanisms.flywheel.getCurrentAmps();
                salvoStartVolts = battery(batterySensor);
                salvoStartDist = distance;
                salvoStartLockErr = lockOn ? targetLock.errorDeg() : Double.NaN;
                salvoStartSetpoint = setpoint;
                salvoStartX = pose.position.x;
                salvoStartY = pose.position.y;
                salvoStartHdg = pose.heading.toDegrees();
                salvoStartSpeedTrim = speedTrim;
                salvoStartAimTrim = aimTrim;
                salvoStartCell = cell.name();
            }
            if (!salvoNow && salvoWas) {
                salvoNo++;
                double low = mechanisms.getSalvoMinSpeed();
                lastSalvo = String.format(Locale.US, "#%d: %.0f at the start (set %.0f), lowest %.0f (dip %.0f)", salvoNo, salvoStartSpeed, salvoStartSetpoint, low, salvoStartSpeed - low);
                RunLog.event(String.format(Locale.US, "TUNER SALVO %d: %s at %.1f in, pose (%.1f, %.1f, %.0f deg) | flywheel at the start %.0f of %.0f ticks/s, lowest %.0f, dip %.0f | battery %.2f V, flywheel %.1f A | lock error %s | speed trim %+.0f, aim trim %+.1f deg",
                        salvoNo, salvoStartCell, salvoStartDist, salvoStartX, salvoStartY, salvoStartHdg, salvoStartSpeed, salvoStartSetpoint, low, salvoStartSpeed - low, salvoStartVolts,
                        salvoStartAmps, Double.isNaN(salvoStartLockErr) ? "lock off" : String.format(Locale.US, "%+.1f deg", salvoStartLockErr), salvoStartSpeedTrim, salvoStartAimTrim));
            }
            salvoWas = salvoNow;

            logX = pose.position.x;
            logY = pose.position.y;
            logHeadingDeg = pose.heading.toDegrees();
            logDistance = distance;
            logBase = base;
            logSpeedTrim = speedTrim;
            logAimTrim = aimTrim;
            logCell = cell.name();
            logSource = fromSession ? "session" : "saved";

            mechanisms.setPusherLockWanted(lockOn);   // as in the TeleOp: the pusher parks during a target lock
            mechanisms.update(pose);
            RunLog.tick();

            telemetry.addData("Cell (from the pose)", "%s, %.1f in", cell, distance);
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f", pose.position.x, pose.position.y, pose.heading.toDegrees());
            telemetry.addData("SPEED", "%.0f ticks/s = %s %.0f  %+.0f trim%s", setpoint, fromSession ? "session points" : "saved map", base, speedTrim,
                    useSession && !fromSession ? "   (no points yet: the saved map)" : "");
            telemetry.addData("AIM trim", "%+.1f deg   target lock %s", aimTrim, lockOn
                    ? String.format(Locale.US, "ON, error %+.1f deg%s", targetLock.errorDeg(), targetLock.isAligned() ? "  ALIGNED" : "") : "off (the trim applies when it is on)");
            telemetry.addData("Flywheel now", "%.0f ticks/s   last salvo %s", mechanisms.flywheel == null ? 0.0 : mechanisms.flywheel.getVelocity(), lastSalvo);
            if (launchWaiting) telemetry.addData("Launch", "QUEUED until ALIGNED (%.1f s left)", Math.max(0.0, LAUNCH_ALIGN_WAIT_SEC - launchAlignClock.seconds()));
            if (!notice.isEmpty() && noticeClock.seconds() < 3.0) telemetry.addLine("NOTICE: " + notice);
            mechanisms.log(telemetry);
            telemetry.addLine("--- recorded: distance / speed (speed trim, aim trim, battery) | paste into LaunchMap.TABLE ---");
            for (Point p : points) {
                telemetry.addLine(String.format(Locale.US, "        {%.1f, %.0f},   // %s: trim %+.0f, aim %+.1f deg, %.1f V", p.distance, p.speed, p.cell, p.speedTrim, p.aimTrim, p.battery));
            }
            telemetry.addLine("GP1: stick drive, LT precision, RB lock on/off, A fire, D-pad up/down speed trim, right/left aim trim, Back clear fault (hold = reseed)   GP2: A record, B undo, Y fire, LT outtake, LB speed source, D-pad report landed low/high/left/right");
            telemetry.update();
        }
        StringBuilder table = new StringBuilder("TUNER TABLE at the end of the run, paste into LaunchMap.TABLE:");
        for (Point p : points) {
            table.append(String.format(Locale.US, " {%.1f, %.0f},", p.distance, p.speed));
        }
        RunLog.event(table.toString());
        for (Point p : points) {
            RunLog.event(String.format(Locale.US, "TUNER POINT %.1f in, %.0f ticks/s, speed trim %+.0f, aim trim %+.1f deg, %s, pose (%.1f, %.1f, %.0f deg), battery %.2f V", p.distance, p.speed,
                    p.speedTrim, p.aimTrim, p.cell, p.x, p.y, p.headingDeg, p.battery));
        }
        drive.stop();
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }
}
