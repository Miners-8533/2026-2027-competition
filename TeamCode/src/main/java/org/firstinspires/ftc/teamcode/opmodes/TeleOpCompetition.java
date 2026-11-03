package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The competition TeleOp: a driver (gamepad 1) drives the chassis FIELD-oriented and an operator (gamepad 2) runs
 * the mechanisms through the {@link Mechanisms} coordinator.
 *
 * Start pose: if an auto ran recently (see AUTO_POSE_MAX_AGE_SEC) the TeleOp starts from the pose and alliance the
 * auto saved in {@link PoseStorage}; otherwise from the alliance's default start pose: blue (-64.4, +16) facing +X against the
 * rear wall, red (+64.4, -16) facing -X against the audience wall (the same pose turned 180 deg about the center, so neither
 * starts on a flower). The INIT screen says which.
 * It also reports whether both gamepads are assigned (Start + A on the driver's controller, Start + B on the
 * operator's), because a gamepad that is not assigned silently does nothing.
 *
 * The drive, the Pinpoint and the mechanisms are built in INIT (the Pinpoint resets and recalibrates there, so do not move the
 * robot after INIT). The INIT screen checks the Pinpoint pose against the start pose, which mechanisms were found, and
 * whether the element counter is reading.
 *
 * INIT (gamepad 1):
 *   D-pad right / left   BLUE / RED alliance (only when there is no auto pose; an auto pose fixes the alliance)
 *   D-pad up             discard the auto pose and use the default start pose (then the D-pad picks the alliance)
 *
 * Gamepad 1 (driver):
 *   D-pad (in the match)  MARK a shot for the log: up = it landed good, left = landed LEFT, right = landed RIGHT, down = off (some other way).
 *                        Each mark records the pose, heading, target, bearing and the predicted miss, and repeats the aim of the last fire (with the
 *                        lock state and nudge then, since the lock releases after the salvo), so the shots can be inspected afterwards.
 *   left stick           drive, field-oriented: up = away from the driver, right = the driver's right
 *   right stick X        turn
 *   right trigger (hold) precision mode (30%)
 *   A                    fire a salvo (spins up and fires when ready); also cancels a running reject, the way out of a jam the other way.
 *                        Works with any number of elements held, locked or not.
 *   (target lock engages by itself as soon as the robot holds 4 elements and stays on through the salvo, which releases it (a finished salvo also clears the manual toggle; the auto lock then waits until the robot is no longer full before it can engage again); while it is on the robot
 *    turns itself to point its front (intake and shooter end) at the nearest of your alliance's two CELLs while you keep driving;
 *    with the left bumper held the right stick nudges the aim up to 15 deg (without it the stick does nothing while locked); ALIGNED on the screen says it is on target. The operator's right
 *    bumper (hold) engages it by hand, full or not.)
 *   left + right bumper  (hold 1 s) reseed the pose to the default start pose (held, so it cannot happen by accident)
 *
 * Gamepad 2 (operator):
 *   (the intake has no toggle: it runs IN until the robot holds 4 elements, then stops; Mechanisms.FULL_RUNS_OUT makes it run OUT instead)
 *   right trigger (hold) force the intake IN, even when full
 *   left trigger (hold)  force the intake OUT
 *   A                    TARGET LOCK by hand, on / off (a toggle), whether the robot is full or not (the lock also engages by itself when it is full);
 *                        A also cancels an automatic lock (it stays off until the robot is next full), the way out of a bad element sensor
 *   hold left bumper + B reject (pusher reverse + outtake); also the way out of a jam: it cancels a running salvo or a queued fire
 *   X / Y                deploy / stow the ramp (manual, no limits: the stow does not wait for the pose or a flower)
 *   (the flywheel has no control: it runs from Start, keeps running after salvos, and stops only when the opmode ends)
 *   hold left bumper + D-pad up / down   speed trim +/- 25 ticks/s added to the shot map's speed (the manual setpoint while the map is empty)
 *   (the target cell is not picked: it is always the closer of your alliance's own two cells, chosen from the pose)
 *   hold left bumper + Y pusher calibration sweep now (the track must be empty; the intake is held off by itself until it ends; again to stop it); Y alone stows
 *   (no control clears a pusher fault: the pusher does not fault while Pusher.FAULTS_ENABLED is false, the current setting)
 *   (neither controller uses Back or Start)
 *
 * BACKUP MODE (no trusted pose or heading), one way: it is entered by the driver holding the left bumper and X for BACKUP_HOLD_SEC (1 s), or automatically
 * when the Pinpoint stays faulted or silent for BACKUP_PINPOINT_FAULT_SEC (a blip does not count). Both controllers give three rumble taps.
 * From then on, until the opmode ends: robot-oriented drive (stick up = robot forward), no target lock (auto or manual), no
 * reseed, and the flywheel speed comes from the fixed distance BACKUP_SHOT_DISTANCE_IN (the start position, about 42 in) with the speed trim
 * still working; the driver drives to that spot to shoot. There is no way out: service the robot.
 *
 * There are no sensor overrides to toggle: a failed Pinpoint is handled by the backup mode above, a counter that stops reading shows a
 * warning (count by eye; hold the operator's left trigger to push extras out), and the flywheel always runs closed loop.
 *
 * Log: the run is written to /sdcard/FIRST/runlogs/teleop.log (the one before it to teleop_prev.log; see
 * {@link RunLog}), so it can be read after the match.
 *
 * "Away from the driver" is a field direction fixed by the alliance: drivers stand behind their alliance wall, so
 * blue faces -Y (-90 deg) and red faces +Y (+90 deg). If a stick direction is wrong at the field, correct
 * BLUE_DRIVER_FORWARD_DEG / RED_DRIVER_FORWARD_DEG in the Dashboard.
 */
@Config
@TeleOp(name = "TeleOp - Competition", group = "Competition")
public class TeleOpCompetition extends LinearOpMode {

    /** The blue default start (rear wall); red is it rotated 180 deg about the field center. X comes from the profile's depth. */
    public static double START_Y_IN = 16.0;
    public static double START_HEADING_DEG = 0.0;
    /** Field heading (deg) of the direction each driver faces, i.e. "stick up". */
    public static double BLUE_DRIVER_FORWARD_DEG = -90.0;
    public static double RED_DRIVER_FORWARD_DEG = 90.0;
    /** An auto pose older than this is ignored (a leftover from an earlier run). */
    public static double AUTO_POSE_MAX_AGE_SEC = 300.0;
    public static double RESEED_HOLD_SEC = 1.0;
    /** Target lock engages by itself when the robot holds a full load (it stays on until the salvo is done). The operator's right bumper engages it by hand regardless. */
    public static boolean AUTO_LOCK_WHEN_FULL = true;
    /** One rumble pulse on both controllers each time the robot becomes full (4 elements, confirmed). Needs a gamepad with rumble. */
    public static int FULL_RUMBLE_MS = 250;
    /** BACKUP MODE: the fixed shot distance (the start position's distance to the cell), and how long the Pinpoint must stay faulted before it is not just a blip. */
    public static double BACKUP_SHOT_DISTANCE_IN = 42.0;
    public static double BACKUP_PINPOINT_FAULT_SEC = 0.5;
    /** How long the driver holds left bumper + X to enter the backup mode by hand. */
    public static double BACKUP_HOLD_SEC = 1.0;
    public static double TURN_SCALE = 0.5;
    public static double PRECISION_SCALE = 0.30;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double SETPOINT_STEP = 25.0;
    /** How often the log writes a sample row (events are written when they happen). */
    public static long SAMPLE_PERIOD_MS = 100;

    private static final double FIELD_HALF_IN = 72.0;

    /** What the driver is commanding and where the robot is, for the log's sample rows. */
    private double logX, logY, logHeadingDeg, cmdForward, cmdLeft, cmdTurn, lockErrDeg = Double.NaN, logUp, logRight;
    private final TargetLock targetLock = new TargetLock();

    /** Driver override: false drives robot-oriented (stick up = robot forward), for when the heading is unreliable. */
    private boolean fieldOriented = true;
    /** BACKUP MODE: no trusted pose or heading. Entered by the driver (left bumper + X) or a Pinpoint fault; never left within a run. */
    private boolean backupMode = false;

    /**
     * One rumble pulse. The Logitech F310 has no rumble motors, so it is skipped; any other gamepad type (Xbox 360, PS4, or one the SDK
     * calls UNKNOWN) is asked to rumble, and the SDK ignores the request if it cannot. The SDK has no way to ask a gamepad whether it can.
     */
    private static void buzz(Gamepad gamepad, int ms) {
        if (gamepad.type() != Gamepad.Type.LOGITECH_F310) gamepad.rumble(ms);
    }

    /** Short rumble taps (three for the backup mode, two for a lost flywheel encoder); skipped on the F310 like {@link #buzz}. */
    private static void buzzBlips(Gamepad gamepad, int n) {
        if (gamepad.type() != Gamepad.Type.LOGITECH_F310) gamepad.rumbleBlips(n);
    }

    /** Switch to the backup mode for the rest of the run: robot-oriented drive, no lock, a fixed shot speed, three taps on both gamepads. */
    private void enterBackupMode(String why, Mechanisms mechanisms) {
        if (backupMode) return;
        backupMode = true;
        fieldOriented = false;
        mechanisms.enterBackupMode(BACKUP_SHOT_DISTANCE_IN);
        buzzBlips(gamepad1, 3);
        buzzBlips(gamepad2, 3);
        RunLog.event("BACKUP MODE ENTERED (" + why + "): robot-oriented drive, fixed shot speed, no target lock, no way back this run");
    }

    /**
     * One line describing the aim at this moment, for the FIRE and MARK log events: the pose, the target cell and its distance, the bearing to it,
     * where the target sits relative to the robot's front line, and the miss predicted from the shooter's sideways offset (+ = the shot lands to the
     * LEFT of the target, - = to the RIGHT, seen from behind the robot). Compare the predicted miss with the shots marked left or right.
     */
    private String aimSummary(Pose2d pose, Mechanisms mechanisms, boolean blue, boolean lockOn) {
        HiveTarget t = mechanisms.getOpenCell();
        double dx = t.x - pose.position.x, dy = t.y - pose.position.y;
        double dist = Math.hypot(dx, dy);
        double bearing = Math.atan2(dy, dx);
        double err = Rotation2d.normalize(bearing - pose.heading.toDouble());   // + = the target is counterclockwise (left) of the front
        double targetLeftIn = dist * Math.sin(err);
        double miss = TargetLock.SHOOTER_OFFSET_LEFT_IN - targetLeftIn;
        return String.format(Locale.US, "%s, pose (%.1f, %.1f, %.1f deg), target %s (%.1f, %.1f) %.1f in, bearing %.1f deg, target %+.1f deg / %+.1f in left of the front line, "
                        + "shooter offset %+.1f in + far-cell extra %+.1f in, predicted miss %+.1f in (against the shooter offset alone), setpoint %.0f (trim %+.0f), lock %s",
                blue ? "BLUE" : "RED", pose.position.x, pose.position.y, pose.heading.toDegrees(), t, t.x, t.y, dist, Math.toDegrees(bearing),
                Math.toDegrees(err), targetLeftIn, TargetLock.SHOOTER_OFFSET_LEFT_IN, TargetLock.farCellExtraLeftIn(t, blue), miss, mechanisms.getLastSetpoint(), mechanisms.getSpeedTrim(),
                lockOn ? (targetLock.isAligned() ? "ALIGNED" : "on") + String.format(Locale.US, " err %+.1f deg nudge %+.1f deg", targetLock.errorDeg(), targetLock.trimDeg()) : "off");
    }

    private static boolean assigned(Gamepad gamepad) {
        return gamepad.getGamepadId() != Gamepad.ID_UNASSOCIATED;
    }

    /**
     * Where the robot starts: the pose the auto saved, or the alliance's default pose. Blue starts against the rear wall; red is the
     * same pose turned 180 deg about the field center (against the audience wall, facing -X), so neither starts on a flower.
     */
    private static Pose2d expectedStart(boolean fromAuto, boolean blue, double startX) {
        if (fromAuto) return PoseStorage.pose();
        return blue ? new Pose2d(startX, START_Y_IN, Math.toRadians(START_HEADING_DEG))
                : new Pose2d(-startX, -START_Y_IN, Math.toRadians(START_HEADING_DEG + 180.0));
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.TELEOP, "TeleOp - Competition", hardwareMap);
        try {
            runCompetition();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runCompetition() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        double startX = -(FIELD_HALF_IN - profile.originFromBackIn);   // back against the wall

        boolean useAutoPose = PoseStorage.isFresh(AUTO_POSE_MAX_AGE_SEC);
        boolean blue = useAutoPose ? PoseStorage.isBlue() : true;

        // Everything is built here, in INIT, like the autos: the Pinpoint resets and recalibrates its IMU while the robot is
        // still, and the INIT screen can check the pose, the mechanisms and the element counter before Start. The robot
        // must not be moved from here until Start. The motors are not driven in INIT (the counter only reads).
        boolean seededAuto = useAutoPose, seededBlue = blue;
        Pose2d expected = expectedStart(useAutoPose, blue, startX);
        MecanumDrive drive = new MecanumDrive(hardwareMap, expected);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpadUpWasPressed() && useAutoPose) {
                useAutoPose = false;
                PoseStorage.clear();
                blue = true;
            }
            if (!useAutoPose) {
                if (gamepad1.dpad_right) blue = true;
                if (gamepad1.dpad_left) blue = false;
            }
            if (useAutoPose != seededAuto || blue != seededBlue) {
                seededAuto = useAutoPose;
                seededBlue = blue;
                expected = expectedStart(useAutoPose, blue, startX);
                drive.setPose(expected);
            }
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();
            Pose2d p = drive.getPose();
            boolean poseOk = Math.hypot(p.position.x - expected.position.x, p.position.y - expected.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - expected.heading.toDouble()))) < 3.0;
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Start pose", useAutoPose
                    ? String.format("AUTO pose, saved %.0f s ago (D-pad up on gamepad 1 discards it)", PoseStorage.ageSec())
                    : blue ? "default start pose (blue: rear wall, facing +X)" : "default start pose (red: audience wall, facing -X)");
            telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   %s", loc.getDeviceStatus(),
                    p.position.x, p.position.y, p.heading.toDegrees(),
                    poseOk && loc.seedConfirmed ? "OK" : "WARNING: does not match the start pose (do not move the robot after INIT; restart the opmode)");
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addData("Element counter", mechanisms.counter == null ? "not configured"
                    : mechanisms.counter.isValid() ? "reading   " + mechanisms.counter.stackString()
                    : "NOT READING (all four sensors must answer; count by eye in the match if it stays this way)");
            telemetry.addData("Gamepad 1 (driver)", assigned(gamepad1) ? "assigned" : "NOT ASSIGNED: press Start + A on it");
            telemetry.addData("Gamepad 2 (operator)", assigned(gamepad2) ? "assigned" : "NOT ASSIGNED: press Start + B on it");
            telemetry.addLine(useAutoPose ? "Press Start on the Driver Station when ready."
                    : "Gamepad 1: D-pad right = BLUE, D-pad left = RED, then press Start on the Driver Station.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();

        Pose2d defaultStart = expectedStart(false, blue, startX);
        Pose2d startPose = expected;
        double driverForwardRad = Math.toRadians(blue ? BLUE_DRIVER_FORWARD_DEG : RED_DRIVER_FORWARD_DEG);

        mechanisms.setOpenCell(blue ? HiveTarget.BLUE_REAR : HiveTarget.RED_AUDIENCE);
        mechanisms.setBlueAlliance(blue);
        mechanisms.setAutoTarget(true);            // the target is the closer of the alliance's own two cells: nothing to pick
        mechanisms.setIntakeAlwaysOn(true);        // the intake runs IN until the robot holds 4, then stops; the operator's triggers override
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel runs the whole time ...
        mechanisms.setSpinRequested(true);         // ... from the start, until the operator stops it (left bumper)

        RunLog.event(String.format("SETUP profile %s, alliance %s, start pose %s (%.1f, %.1f, %.1f deg), driver forward %.0f deg; %s",
                profile.name, blue ? "BLUE" : "RED", useAutoPose ? "from the AUTO" : "default",
                startPose.position.x, startPose.position.y, startPose.heading.toDegrees(), Math.toDegrees(driverForwardRad),
                mechanisms.describe()));
        RunLog.event("SETUP gamepad 1 " + (assigned(gamepad1) ? "assigned" : "NOT ASSIGNED") + " (" + gamepad1.type() + "), gamepad 2 "
                + (assigned(gamepad2) ? "assigned" : "NOT ASSIGNED") + " (" + gamepad2.type() + ")");
        RunLog.setSampler("x,y,hdg_deg,stick_up,stick_right,cmd_fwd,cmd_left,cmd_turn,drive_mode,lock_err_deg," + Mechanisms.SAMPLE_COLUMNS, SAMPLE_PERIOD_MS,
                () -> String.format(Locale.US, "%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.1f,%s", logX, logY, logHeadingDeg, logUp, logRight, cmdForward, cmdLeft,
                        cmdTurn, backupMode ? "backup" : fieldOriented ? "field" : "robot", lockErrDeg, mechanisms.sampleRow()));
        boolean gamepad1Was = assigned(gamepad1), gamepad2Was = assigned(gamepad2);

        boolean reseedDown = false, reseeded = false, backupHeld = false, manualLockOn = false, autoLockSuppressed = false;
        int salvosSeen = 0;
        ElapsedTime backupHold = new ElapsedTime();
        ElapsedTime reseedHold = new ElapsedTime();
        boolean lockOn = false, lockWasAligned = false, autoLockLatched = false, wasFull = false, pinpointBad = false;
        String lastFireAim = "none yet";   // the aim line of the last fire (with the lock state and nudge then), repeated in the marks that follow it
        ElapsedTime pinpointBadFor = new ElapsedTime();
        ElapsedTime loopClock = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = loopClock.seconds();
            loopClock.reset();
            PoseVelocity2d measured = null;
            if (!backupMode) {
                // a Pinpoint that stops answering (a read error, a fault status, or an exception) for BACKUP_PINPOINT_FAULT_SEC is not a blip
                boolean bad;
                String badWhy;
                try {
                    measured = drive.updatePoseEstimate();
                    GoBildaPinpointDriver.DeviceStatus status = loc.getDeviceStatus();
                    bad = status != GoBildaPinpointDriver.DeviceStatus.READY;
                    badWhy = "status " + status;
                } catch (RuntimeException e) {
                    bad = true;
                    badWhy = e.toString();
                }
                if (bad && !pinpointBad) {
                    pinpointBad = true;
                    pinpointBadFor.reset();
                    RunLog.event("PINPOINT fault: " + badWhy + " (backup mode if it lasts " + BACKUP_PINPOINT_FAULT_SEC + " s)");
                } else if (bad && pinpointBadFor.seconds() >= BACKUP_PINPOINT_FAULT_SEC) {
                    enterBackupMode("Pinpoint " + badWhy + " for " + String.format(Locale.US, "%.1f", pinpointBadFor.seconds()) + " s", mechanisms);
                } else if (!bad && pinpointBad) {
                    pinpointBad = false;
                    RunLog.event(String.format(Locale.US, "PINPOINT recovered after %.2f s (a blip, no backup mode)", pinpointBadFor.seconds()));
                }
            }
            Pose2d pose = drive.getPose();
            if (!backupMode) PoseStorage.save(pose, blue);
            double heading = pose.heading.toDouble();

            // driver: stick up = away from the driver, stick right = the driver's right, both in field terms
            double scale = gamepad1.right_trigger > TRIGGER_THRESHOLD ? PRECISION_SCALE : 1.0;
            double up = -gamepad1.left_stick_y * scale;
            double right = gamepad1.left_stick_x * scale;
            logUp = up;
            logRight = right;
            double robotForward, robotLeft;
            if (fieldOriented) {
                double fieldX = up * Math.cos(driverForwardRad) + right * Math.cos(driverForwardRad - Math.PI / 2.0);
                double fieldY = up * Math.sin(driverForwardRad) + right * Math.sin(driverForwardRad - Math.PI / 2.0);
                robotForward = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
                robotLeft = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);
            } else {
                robotForward = up;
                robotLeft = -right;
            }
            cmdForward = robotForward;
            cmdLeft = robotLeft;
            double manualTurn = -gamepad1.right_stick_x * TURN_SCALE * scale;
            // auto lock: from the moment the robot is full until its salvo is done; manual lock: operator A toggles it, full or not.
            // A finished salvo releases BOTH; the auto lock then waits for the robot to stop being full before it can engage again.
            if (gamepad2.aWasPressed()) {
                if (manualLockOn || (AUTO_LOCK_WHEN_FULL && autoLockLatched)) {
                    // any lock that is on, manual or automatic, goes off; the automatic one stays off until the robot is next full
                    RunLog.event("COMMAND target lock off (" + (manualLockOn ? "manual" : "") + (manualLockOn && autoLockLatched ? " and " : "")
                            + (autoLockLatched ? "automatic" : "") + ")");
                    manualLockOn = false;
                    if (autoLockLatched) {
                        autoLockLatched = false;
                        autoLockSuppressed = true;
                    }
                } else {
                    manualLockOn = true;
                    RunLog.event("COMMAND manual target lock -> ON" + (backupMode ? " (no effect: backup mode)" : ""));
                }
            }
            if (mechanisms.getSalvosDone() != salvosSeen) {
                salvosSeen = mechanisms.getSalvosDone();
                if (manualLockOn || autoLockLatched) RunLog.event("TARGET LOCK released: the salvo is done" + (manualLockOn ? " (manual lock cleared)" : ""));
                manualLockOn = false;
                autoLockLatched = false;
                autoLockSuppressed = true;
            }
            boolean manualLock = manualLockOn;
            boolean fullNow = mechanisms.isFull();
            if (fullNow && !wasFull && FULL_RUMBLE_MS > 0) {
                buzz(gamepad1, FULL_RUMBLE_MS);
                buzz(gamepad2, FULL_RUMBLE_MS);
            }
            wasFull = fullNow;
            if (fullNow) {
                if (!autoLockSuppressed) autoLockLatched = true;
            } else {
                autoLockSuppressed = false;
                if (!mechanisms.isFireBusy()) autoLockLatched = false;
            }
            boolean lockWanted = !backupMode && (manualLock || (AUTO_LOCK_WHEN_FULL && autoLockLatched));
            if (lockWanted) {
                HiveTarget lockTarget = mechanisms.getOpenCell();
                if (!lockOn) {
                    lockOn = true;
                    lockWasAligned = false;
                    RunLog.event(String.format(Locale.US, "TARGET LOCK on (%s): %s at (%.2f, %.2f), %.1f in away, robot pose (%.1f, %.1f, %.1f deg), alliance %s",
                            manualLock ? "manual" : "full", lockTarget, lockTarget.x, lockTarget.y, lockTarget.distanceFrom(pose), pose.position.x, pose.position.y,
                            pose.heading.toDegrees(), blue ? "BLUE" : "RED"));
                }
                cmdTurn = targetLock.turn(pose, measured == null ? 0.0 : measured.angVel, lockTarget,
                        TargetLock.farCellExtraLeftIn(lockTarget, blue), gamepad1.left_bumper ? gamepad1.right_stick_x : 0.0, dt);   // the nudge needs the left bumper held
                lockErrDeg = targetLock.errorDeg();
                boolean aligned = targetLock.isAligned();
                if (aligned && !lockWasAligned) {
                    RunLog.event(String.format(Locale.US, "TARGET LOCK aligned: error %+.1f deg, nudge %+.1f deg, %.1f in from %s, bearing %.1f deg, robot pose (%.1f, %.1f, %.1f deg)",
                            lockErrDeg, targetLock.trimDeg(), lockTarget.distanceFrom(pose), lockTarget, targetLock.bearingDeg(),
                            pose.position.x, pose.position.y, pose.heading.toDegrees()));
                }
                lockWasAligned = aligned;
            } else {
                if (lockOn) {
                    lockOn = false;
                    targetLock.release();
                    RunLog.event("TARGET LOCK off");
                }
                lockErrDeg = Double.NaN;
                cmdTurn = manualTurn;
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotForward, robotLeft), cmdTurn));
            logX = pose.position.x;
            logY = pose.position.y;
            logHeadingDeg = pose.heading.toDegrees();

            if (assigned(gamepad1) != gamepad1Was || assigned(gamepad2) != gamepad2Was) {
                gamepad1Was = assigned(gamepad1);
                gamepad2Was = assigned(gamepad2);
                RunLog.event("GAMEPAD assignment changed: gamepad 1 " + (gamepad1Was ? "assigned" : "NOT ASSIGNED")
                        + ", gamepad 2 " + (gamepad2Was ? "assigned" : "NOT ASSIGNED"));
            }

            // read both edges every loop so a press made without the guard cannot trigger an override later
            if (gamepad1.left_bumper && gamepad1.x && !backupMode) {
                if (!backupHeld) {
                    backupHeld = true;
                    backupHold.reset();
                } else if (backupHold.seconds() >= BACKUP_HOLD_SEC) {
                    enterBackupMode("driver held left bumper + X", mechanisms);
                }
            } else {
                backupHeld = false;
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper && !backupMode) {
                if (!reseedDown) {
                    reseedDown = true;
                    reseeded = false;
                    reseedHold.reset();
                }
                if (!reseeded && reseedHold.seconds() >= RESEED_HOLD_SEC) {
                    drive.setPose(defaultStart);
                    reseeded = true;
                    RunLog.event(String.format("DRIVER pose reseeded to the default start (was %.1f, %.1f, %.1f deg)",
                            pose.position.x, pose.position.y, pose.heading.toDegrees()));
                }
            } else {
                reseedDown = false;
            }

            // operator
            mechanisms.setManualIntake(gamepad2.right_trigger > TRIGGER_THRESHOLD);
            mechanisms.setManualOuttake(gamepad2.left_trigger > TRIGGER_THRESHOLD);
            if (gamepad1.aWasPressed()) {
                lastFireAim = aimSummary(pose, mechanisms, blue, lockOn);
                RunLog.event("FIRE requested: " + lastFireAim);
                mechanisms.requestFire();
            }
            boolean markGood = gamepad1.dpadUpWasPressed(), markOff = gamepad1.dpadDownWasPressed();
            boolean markLeft = gamepad1.dpadLeftWasPressed(), markRight = gamepad1.dpadRightWasPressed();
            if (markGood || markOff || markLeft || markRight) {
                RunLog.event("MARK " + (markGood ? "good" : markLeft ? "landed LEFT" : markRight ? "landed RIGHT" : "off") + ": "
                        + aimSummary(pose, mechanisms, blue, lockOn) + (backupMode ? " (backup mode: the pose is not trusted)" : "")
                        + " | AT THE LAST FIRE: " + lastFireAim);
                buzz(gamepad1, 80);
            }
            boolean operatorB = gamepad2.bWasPressed();   // read every loop; the reject needs the left bumper held, so it cannot be set off by accident
            if (operatorB && gamepad2.left_bumper) mechanisms.requestReject();
            if (gamepad2.xWasPressed()) mechanisms.requestDeploy();
            boolean operatorY = gamepad2.yWasPressed();   // read once: with the left bumper held, Y is the pusher calibration instead of the stow
            if (operatorY && gamepad2.left_bumper) mechanisms.requestPusherCalibration();
            else if (operatorY) mechanisms.requestStowNow();   // stow at once: no flower check
            boolean trimUp = gamepad2.dpadUpWasPressed(), trimDown = gamepad2.dpadDownWasPressed();   // read every loop; the left bumper must be held
            if (trimUp && gamepad2.left_bumper) mechanisms.adjustManualSetpoint(SETPOINT_STEP);
            if (trimDown && gamepad2.left_bumper) mechanisms.adjustManualSetpoint(-SETPOINT_STEP);

            mechanisms.update(pose);
            if (mechanisms.takeFlywheelLostNotice()) {   // the flywheel's speed sensor died: open loop from now on; two taps on both pads
                buzzBlips(gamepad1, 2);
                buzzBlips(gamepad2, 2);
            }
            RunLog.tick();

            if (backupHeld && !backupMode) telemetry.addData("Backup mode", "keep holding LB + X... %.1f s", BACKUP_HOLD_SEC - backupHold.seconds());
            if (backupMode) telemetry.addLine(String.format(Locale.US, "*** BACKUP MODE: no pose or heading. Robot-oriented drive, shot speed fixed for %.0f in (drive to that spot), no target lock. Service the robot. ***", BACKUP_SHOT_DISTANCE_IN));
            else if (pinpointBad) telemetry.addLine("*** PINPOINT FAULT (backup mode in a moment if it stays) ***");
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, pose.heading.toDegrees());
            if (reseedDown && !reseeded) {
                telemetry.addData("Reseed pose", "keep holding LB + RB... %.1f s", RESEED_HOLD_SEC - reseedHold.seconds());
            }
            telemetry.addData("Drive", backupMode ? "ROBOT-ORIENTED (backup mode)" : fieldOriented ? "field-oriented" : "robot-oriented");
            telemetry.addData("Target lock (operator A toggles; on by itself when full)", lockOn
                    ? String.format(Locale.US, "ON -> %s, bearing %.1f deg, error %+.1f deg%s", mechanisms.getOpenCell(), targetLock.bearingDeg(),
                            targetLock.errorDeg(), targetLock.isAligned() ? "   ALIGNED" : "")
                    : "off");
            mechanisms.log(telemetry);
            telemetry.addLine("GP1: stick drive, hold RT precision, A fire, hold LB + RB = reseed pose, hold LB + X 1 s = BACKUP MODE (no way back)");
            telemetry.addLine("GP2: intake automatic (IN, stops when full), hold RT = force in, hold LT = force out, A target lock on/off (it also engages by itself when full), hold LB + B reject, X deploy, Y stow, hold LB + D-pad up/down speed trim, hold LB + Y pusher calibration sweep");
            telemetry.addLine("BACKUP MODE (no way back): GP1 hold LB + X for 1 s, or a Pinpoint fault");
            telemetry.update();
        }
        drive.stop();
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.setManualIntake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
        RunLog.event("STOPPED by the driver station");
    }
}
