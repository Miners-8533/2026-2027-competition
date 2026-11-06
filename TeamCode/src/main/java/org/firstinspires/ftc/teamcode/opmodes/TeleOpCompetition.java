package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AutoLaunch;
import org.firstinspires.ftc.teamcode.drive.HiveDash;
import org.firstinspires.ftc.teamcode.drive.ParkDash;
import org.firstinspires.ftc.teamcode.drive.HoldPosition;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
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
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Arrays;
import java.util.Locale;

/**
 * The competition TeleOp: a driver (gamepad 1) drives the chassis FIELD-oriented and an operator (gamepad 2) runs
 * the mechanisms through the {@link Mechanisms} coordinator.
 *
 * Start pose: if an auto ran recently (see AUTO_POSE_MAX_AGE_SEC) the TeleOp starts from the alliance the auto saved in
 * {@link PoseStorage} and ADOPTS the Pinpoint's own pose: nothing is written to the device and its IMU is not recalibrated, so the pose it
 * kept tracking since the auto (a bump in the transition included) is used as it is (a device that is not READY, reads (0, 0, 0) while the saved
 * pose is elsewhere, or reads a pose outside the field is reset and seeded with the saved pose instead). Otherwise (no fresh auto pose: a
 * practice start, or the app restarted) the Pinpoint is reset, recalibrated and seeded with the alliance's default start pose: blue (-63.4, +16) facing +X against the
 * rear wall, red (+63.4, -16) facing -X against the audience wall (the same pose turned 180 deg about the center, so neither
 * starts on a flower). The INIT screen says which.
 * It also reports whether both gamepads are assigned (Start + A on the driver's controller, Start + B on the
 * operator's), because a gamepad that is not assigned silently does nothing.
 *
 * The drive, the Pinpoint and the mechanisms are built in INIT. When the Pinpoint is reset and seeded (no fresh auto pose) the robot must be
 * still and in the start pose from INIT on; when it is adopted the robot may be anywhere. The INIT screen shows which, checks the Pinpoint
 * pose (against the start pose, or against the pose the auto saved), which mechanisms were found, and whether the element counter is reading.
 * The run log's PINPOINT lines (INIT decision, START with the device numbers, reseeds, at Start) say what was done and why.
 *
 * INIT (gamepad 1):
 *   D-pad right / left   BLUE / RED alliance (only when there is no auto pose; an auto pose fixes the alliance)
 *   D-pad up             discard the auto pose and use the default start pose (then the D-pad picks the alliance)
 *
 * Gamepad 1 (driver):
 *   left stick           drive, field-oriented: up = away from the driver, right = the driver's right
 *   right stick X        turn
 *   left trigger (hold)  precision mode (30%)
 *   A                    launch a salvo (spins up and launches when ready); also cancels a running reject, the way out of a jam the other way.
 *                        Works with any number of elements held, locked or not. While the target lock (right trigger) is held and not yet ALIGNED the launch is QUEUED until it is
 *                        (LAUNCH_WAITS_FOR_ALIGN): it goes the moment the lock aligns, and is dropped if the trigger is released or LAUNCH_ALIGN_WAIT_SEC passes (10 of 19 salvos on
 *                        2026-11-03 went before the lock was aligned, two pointing 70 to 110 deg off the cell).
 *   right trigger (hold) TARGET LOCK by hand: while it is held the robot turns itself to point its front (intake and launcher end) at the nearest of your alliance's two
 *                        CELLs while you keep driving; with the left bumper also held the right stick nudges the aim up to 15 deg (without it the stick does nothing while
 *                        locked); ALIGNED on the screen says it is on target. The automatic lock when the robot is full is OFF (AUTO_LOCK_WHEN_FULL = false: a sliding
 *                        pack of 3 pollen lit all four sensors and kept engaging it). Release the trigger to drive normally.
 *   D-pad left / right   (HOLD) HIVE DASH: the robot drives to the launch spot of a cell, target locks and launches when ALIGNED. Left = the cell on the driver's LEFT, right = the
 *                        cell on the driver's RIGHT as seen from behind the alliance wall (blue: left = audience cell, right = rear cell; red the other way round). Releasing
 *                        the D-pad cancels at once (the sticks drive again); a salvo already launched always finishes. Refused, with a short rumble and no movement, in backup mode, with a
 *                        Pinpoint fault, when the counter reads and holds nothing, or when the robot is touching a hive support. See {@link HiveDash} and design/hive-dash.md.
 *   D-pad up / down      (HOLD) PARK DASH: the robot drives to a park corner of its own loading zone and stays there while the D-pad is held. Up = the corner on the driver's LEFT
 *                        (the audience-side corner for blue), down = the corner on the driver's RIGHT (the hive-side corner). The ramp is stowed when it starts. Releasing the D-pad
 *                        cancels at once. Refused, with a short rumble and no movement, in backup mode, with a Pinpoint fault, with an unseeded pose, while the Hive Dash runs, or when the
 *                        robot is touching a hive support. See {@link ParkDash} and design/park-dash.md.
 *   (HOLD POSITION has no control: from a launch request to the end of the salvo, once the robot has settled, it holds that pose with a PID, resisting a push, and the heading too when the
 *                        target lock is not turning it; moving either stick cancels it at once. See {@link HoldPosition} and HoldPosition.ENABLED.)
 *   left + right bumper  (hold 1 s) reseed the pose to the default start pose (held, so it cannot happen by accident)
 *
 * Gamepad 2 (operator):
 *   (the intake has no toggle: it runs IN until the robot holds 4 elements, then stops; Mechanisms.FULL_RUNS_OUT makes it run OUT instead)
 *   right trigger (hold) force the intake IN, even when full
 *   left trigger (hold)  force the intake OUT
 *   hold left bumper + B reject (pusher reverse + outtake); also the way out of a jam: it cancels a running salvo or a queued launch
 *   X / Y                deploy / stow the ramp (manual, no limits: the stow does not wait for the pose or a flower)
 *   (the pusher LOCK has no control: while a target lock (driver right trigger, the operator's A, auto-launch) or a Hive Dash is on and something is held, the arm moves 90 deg forward of HOLD across the intake
 *                        entrance by itself (Mechanisms.PUSHER_LOCK_ENABLED); the element count is frozen and the intake is OFF while locked (it is NOT locked just because the robot is full: the count of 4 is not
 *                        trusted); a launch goes from the lock (310 deg), a reject unlocks first, holding the right trigger (force IN) unlocks it, and the arm goes back to HOLD 0.5 s after the target lock or dash ends without a launch)
 *   (the flywheel has no control: it runs from Start, keeps running after salvos, and stops only when the opmode ends)
 *   D-pad (unguarded)    REPORT where the LAST salvo landed, seen from behind the robot: left = it landed LEFT (the remembered aim for that cell turns 1.5 deg right),
 *                        right = landed RIGHT (1.5 deg left), down = landed LOW / UNDER (speed trim +10 ticks/s), up = landed HIGH / OVER (speed trim -10). The remembered aim is kept
 *                        per CELL (the cell of the last launch), is used by the next lock on that cell, and is cleared by a driver reseed; the speed trim is one shared offset
 *                        added to the launch map's speed (the manual setpoint while the map is empty). Each report is logged with the aim of the last launch.
 *   hold left bumper + D-pad left / right   clear the remembered aim of the last launched cell;  hold left bumper + D-pad up / down   clear the speed trim
 *   (the target cell is not picked: it is always the closer of your alliance's own two cells, chosen from the pose)
 *   right bumper         AUTO-LAUNCH on / off (a toggle, off at Start; a prototype). With it on, while the target lock is on (automatic or manual) and the robot is inside the
 *                        launch zone of the cell it is locked on (see {@link AutoLaunch}), the chassis stops by itself and, once the lock is ALIGNED and the robot has stopped,
 *                        one salvo is launched. It does nothing when the counter reads 0 elements, and the driver gets the robot back if the launch has not gone within
 *                        AutoLaunch.FREEZE_MAX_SEC. The salvo releases the lock as always.
 *   hold left bumper + Y pusher calibration sweep now (the track must be empty; the intake is held off by itself until it ends; again to stop it); Y alone stows
 *   (no control clears a pusher fault: the pusher does not fault while Pusher.FAULTS_ENABLED is false, the current setting)
 *   (neither controller uses Back or Start)
 *
 * BACKUP MODE (no trusted pose or heading), one way: it is entered by the driver holding the left bumper and X for BACKUP_HOLD_SEC (1 s), or automatically
 * when the Pinpoint stays faulted or silent for BACKUP_PINPOINT_FAULT_SEC (a blip does not count). Both controllers give three rumble taps.
 * From then on, until the opmode ends: robot-oriented drive (stick up = robot forward), no target lock (auto or manual), no
 * reseed, and the flywheel speed comes from the fixed distance BACKUP_LAUNCH_DISTANCE_IN (the start position, about 42 in) with the speed trim
 * still working; the driver drives to that spot to launch. There is no way out: service the robot.
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
    /** True: target lock also engages by itself when the robot holds a full load (until the salvo is done). OFF since 2026-11-03: the lock is held by the driver's right trigger. */
    public static boolean AUTO_LOCK_WHEN_FULL = false;
    /** With the lock held and not ALIGNED, the driver's launch waits for ALIGNED (at most this long, then it is dropped) instead of firing at once. */
    public static boolean LAUNCH_WAITS_FOR_ALIGN = true;
    public static double LAUNCH_ALIGN_WAIT_SEC = 1.5;
    /** One rumble pulse on both controllers each time the robot becomes full (4 elements, confirmed). Needs a gamepad with rumble. */
    public static int FULL_RUMBLE_MS = 250;
    /** BACKUP MODE: the fixed launch distance (the start position's distance to the cell), and how long the Pinpoint must stay faulted before it is not just a blip. */
    public static double BACKUP_LAUNCH_DISTANCE_IN = 42.0;
    public static double BACKUP_PINPOINT_FAULT_SEC = 0.5;
    /** How long the driver holds left bumper + X to enter the backup mode by hand. */
    public static double BACKUP_HOLD_SEC = 1.0;
    public static double TURN_SCALE = 0.5;
    public static double PRECISION_SCALE = 0.30;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double SETPOINT_STEP = 10.0;
    /** One D-pad report of a salvo that landed left or right turns the remembered aim for that cell by this much, up to NUDGE_MAX_DEG either way. */
    public static double NUDGE_STEP_DEG = 1.5;
    public static double NUDGE_MAX_DEG = 10.0;
    /** How often the log writes a sample row (events are written when they happen). */
    public static long SAMPLE_PERIOD_MS = 100;


    /** What the driver is commanding and where the robot is, for the log's sample rows. */
    private double logX, logY, logHeadingDeg, cmdForward, cmdLeft, cmdTurn, lockErrDeg = Double.NaN, logUp, logRight;
    private final TargetLock targetLock = new TargetLock();
    /** The operator's standing aim correction for each cell (index = HiveTarget ordinal), degrees, positive = counterclockwise. */
    private final double[] cellNudgeDeg = new double[HiveTarget.values().length];

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

    /** Switch to the backup mode for the rest of the run: robot-oriented drive, no lock, a fixed launch speed, three taps on both gamepads. */
    private void enterBackupMode(String why, Mechanisms mechanisms) {
        if (backupMode) return;
        backupMode = true;
        fieldOriented = false;
        mechanisms.enterBackupMode(BACKUP_LAUNCH_DISTANCE_IN);
        buzzBlips(gamepad1, 3);
        buzzBlips(gamepad2, 3);
        RunLog.event("BACKUP MODE ENTERED (" + why + "): robot-oriented drive, fixed launch speed, no target lock, no way back this run");
    }

    /**
     * One line describing the aim at this moment, for the LAUNCH log events and the operator's reports: the pose, the target cell and its distance, the bearing to it,
     * where the target sits relative to the robot's front line, and the miss predicted from the launcher's sideways offset (+ = the launch lands to the
     * LEFT of the target, - = to the RIGHT, seen from behind the robot). Compare the predicted miss with the reports of left or right.
     */
    private String aimSummary(Pose2d pose, Mechanisms mechanisms, boolean blue, boolean lockOn) {
        HiveTarget t = mechanisms.getOpenCell();
        double dx = t.x - pose.position.x, dy = t.y - pose.position.y;
        double dist = Math.hypot(dx, dy);
        double bearing = Math.atan2(dy, dx);
        double err = Rotation2d.normalize(bearing - pose.heading.toDouble());   // + = the target is counterclockwise (left) of the front
        double targetLeftIn = dist * Math.sin(err);
        double miss = TargetLock.LAUNCHER_OFFSET_LEFT_IN - targetLeftIn;
        return String.format(Locale.US, "%s, pose (%.1f, %.1f, %.1f deg), target %s (%.1f, %.1f) %.1f in, bearing %.1f deg, target %+.1f deg / %+.1f in left of the front line, "
                        + "launcher offset %+.1f in + far-cell extra %+.1f in, predicted miss %+.1f in (against the launcher offset alone), setpoint %.0f (trim %+.0f), lock %s, remembered nudge %+.1f deg",
                blue ? "BLUE" : "RED", pose.position.x, pose.position.y, pose.heading.toDegrees(), t, t.x, t.y, dist, Math.toDegrees(bearing),
                Math.toDegrees(err), targetLeftIn, TargetLock.LAUNCHER_OFFSET_LEFT_IN, TargetLock.farCellExtraLeftIn(t, blue), miss, mechanisms.getLastSetpoint(), mechanisms.getSpeedTrim(),
                lockOn ? (targetLock.isAligned() ? "ALIGNED" : "on") + String.format(Locale.US, " err %+.1f deg nudge %+.1f deg", targetLock.errorDeg(), targetLock.trimDeg()) : "off", cellNudgeDeg[t.ordinal()]);
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
        double startX = -(FieldGeometry.HALF_IN - profile.originFromBackIn);   // back against the wall

        boolean useAutoPose = PoseStorage.isFresh(AUTO_POSE_MAX_AGE_SEC);
        boolean blue = useAutoPose ? PoseStorage.isBlue() : true;

        // Everything is built here, in INIT, like the autos, and the INIT screen can check the pose, the mechanisms and the element counter
        // before Start. The motors are not driven in INIT (the counter only reads). The Pinpoint: with a fresh auto pose the device is ADOPTED
        // (nothing written to it, IMU not recalibrated: the auto left it in a good state and it kept tracking through the transition); without
        // one it is reset, recalibrated and seeded with the default start, and then the robot must be still and not moved until Start.
        boolean seededAuto = useAutoPose, seededBlue = blue;
        Pose2d expected = expectedStart(useAutoPose, blue, startX);
        double savedAgeSec = PoseStorage.ageSec();
        Pose2d savedPose = PoseStorage.pose();
        RunLog.event(savedPose == null
                ? "PINPOINT INIT decision: no saved pose (no auto or TeleOp has run since the app started, or it was discarded): reset, recalibrate and seed the default start"
                : String.format(Locale.US, "PINPOINT INIT decision: saved pose (%.2f, %.2f, %.1f deg, alliance %s) is %.1f s old, limit %.0f s: %s",
                        savedPose.position.x, savedPose.position.y, savedPose.heading.toDegrees(), PoseStorage.isBlue() ? "BLUE" : "RED", savedAgeSec,
                        AUTO_POSE_MAX_AGE_SEC, useAutoPose ? "adopt the device's own pose (no reset, no seed, no IMU recalibration)"
                                : "too old, reset, recalibrate and seed the default start"));
        MecanumDrive drive = new MecanumDrive(hardwareMap, expected, useAutoPose);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        RunLog.event(String.format(Locale.US, "PINPOINT START mode %s, seedConfirmed %s, bring-up %.0f ms: %s", loc.startMode, loc.seedConfirmed, loc.bringUpMs, loc.startNote));
        Pose2d initPose = drive.getPose();   // where the device was after bring-up (and after each reseed in INIT), to see whether the robot moved before Start
        // The ramp is NOT commanded in INIT: the servos sag when the auto ends, the auto may leave the ramp stuck or deployed, and G403 allows no
        // powered movement in the transition. It is stowed once, at Start. (The intake, flywheel and pusher constructors only set power 0.)
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE, false);

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
                Pose2d was = drive.getPose();
                boolean discarded = useAutoPose != seededAuto;
                seededAuto = useAutoPose;
                seededBlue = blue;
                expected = expectedStart(useAutoPose, blue, startX);
                drive.setPose(expected);
                initPose = expected;
                RunLog.event(String.format(Locale.US, "PINPOINT INIT manual reset: %s; the device was reseeded to (%.2f, %.2f, %.1f deg) (it read %.2f, %.2f, %.1f deg; position written only, the IMU is not recalibrated)",
                        discarded ? "the driver pressed D-pad up and the auto pose is discarded" : "the alliance was changed to " + (blue ? "BLUE" : "RED"),
                        expected.position.x, expected.position.y, expected.heading.toDegrees(), was.position.x, was.position.y, was.heading.toDegrees()));
            }
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();
            Pose2d p = drive.getPose();
            // continuing: the device's own pose was adopted (not reset), so the robot may be anywhere and the pose is only checked for a healthy device
            boolean continuing = useAutoPose && loc.startMode == PinpointLocalizer.StartMode.ADOPTED;
            boolean poseOk = continuing ? loc.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                    : Math.hypot(p.position.x - expected.position.x, p.position.y - expected.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - expected.heading.toDouble()))) < 3.0;
            double savedDiffIn = Math.hypot(p.position.x - expected.position.x, p.position.y - expected.position.y);
            double savedDiffDeg = Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - expected.heading.toDouble())));
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Start pose", useAutoPose
                    ? String.format("AUTO pose, saved %.0f s ago (D-pad up on gamepad 1 discards it and reseeds the default start)", PoseStorage.ageSec())
                    : blue ? "default start pose (blue: rear wall, facing +X)" : "default start pose (red: audience wall, facing -X)");
            if (continuing) {
                telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   CONTINUING from the auto: the device was not reset or reseeded   %s", loc.getDeviceStatus(),
                        p.position.x, p.position.y, p.heading.toDegrees(), poseOk ? "OK" : "WARNING: the device is not READY");
                telemetry.addData("Auto saved", "%.1f, %.1f, %.0f deg; the device differs by %.1f in / %.0f deg%s", expected.position.x, expected.position.y,
                        expected.heading.toDegrees(), savedDiffIn, savedDiffDeg,
                        savedDiffIn > 3.0 || savedDiffDeg > 5.0 ? "   CHECK where the robot really is (it moved after the auto's last save, or the save was stale); D-pad up reseeds" : "");
            } else {
                telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   %s   (%s)", loc.getDeviceStatus(),
                        p.position.x, p.position.y, p.heading.toDegrees(),
                        poseOk && loc.seedConfirmed ? "OK" : "WARNING: does not match the start pose (do not move the robot after INIT; restart the opmode)",
                        loc.startMode == PinpointLocalizer.StartMode.RESEEDED_AFTER_FAILED_ADOPT ? "the device was not usable as it was: reset and reseeded with the auto's pose, see the log"
                                : loc.startMode == PinpointLocalizer.StartMode.ADOPTED ? "reseeded to the default start (position written, the IMU was not recalibrated)"
                                : "reset, recalibrated and seeded with the default start");
            }
            telemetry.addData("Mechanisms present", mechanisms.describe());
            if (mechanisms.rampPositionUnknown()) telemetry.addData("Ramp", "position unknown: not commanded in INIT, it stows when you press Start");
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
        boolean continuingAtStart = useAutoPose && loc.startMode == PinpointLocalizer.StartMode.ADOPTED;
        Pose2d atStart = drive.getPose();
        Pose2d startPose = continuingAtStart ? atStart : expected;
        RunLog.event(String.format(Locale.US, "PINPOINT at Start: %s, pose (%.2f, %.2f, %.1f deg), moved %.2f in / %.1f deg since INIT, device %s, seedConfirmed %s%s",
                continuingAtStart ? "CONTINUING from the auto" : loc.startMode == PinpointLocalizer.StartMode.RESEEDED_AFTER_FAILED_ADOPT && useAutoPose ? "reseeded with the auto's pose" : "seeded with the default start",
                atStart.position.x, atStart.position.y, atStart.heading.toDegrees(),
                Math.hypot(atStart.position.x - initPose.position.x, atStart.position.y - initPose.position.y),
                Math.abs(Math.toDegrees(Rotation2d.normalize(atStart.heading.toDouble() - initPose.heading.toDouble()))),
                loc.getDeviceStatus(), loc.seedConfirmed,
                continuingAtStart ? "" : " (the robot must not have moved since INIT: a move shows as an error in the pose)"));
        double driverForwardRad = Math.toRadians(blue ? BLUE_DRIVER_FORWARD_DEG : RED_DRIVER_FORWARD_DEG);

        mechanisms.stowRampAtStart();              // the first and only ramp command of the start (it was left alone in INIT)
        mechanisms.setOpenCell(blue ? HiveTarget.BLUE_REAR : HiveTarget.RED_AUDIENCE);
        mechanisms.setBlueAlliance(blue);
        mechanisms.setAutoTarget(true);            // the target is the closer of the alliance's own two cells: nothing to pick
        mechanisms.setIntakeAlwaysOn(true);        // the intake runs IN until the robot holds 4, then stops; the operator's triggers override
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel runs the whole time ...
        mechanisms.setSpinRequested(true);         // ... from the start, until the operator stops it (left bumper)

        RunLog.event(String.format("SETUP profile %s, alliance %s, start pose %s (%.1f, %.1f, %.1f deg), driver forward %.0f deg; %s",
                profile.name, blue ? "BLUE" : "RED", useAutoPose ? "from the AUTO" : "default",   // (pull_logs.py parses this; the PINPOINT lines say whether the device was adopted)
                startPose.position.x, startPose.position.y, startPose.heading.toDegrees(), Math.toDegrees(driverForwardRad),
                mechanisms.describe()));
        RunLog.event("SETUP gamepad 1 " + (assigned(gamepad1) ? "assigned" : "NOT ASSIGNED") + " (" + gamepad1.type() + "), gamepad 2 "
                + (assigned(gamepad2) ? "assigned" : "NOT ASSIGNED") + " (" + gamepad2.type() + ")");
        RunLog.setSampler("x,y,hdg_deg,stick_up,stick_right,cmd_fwd,cmd_left,cmd_turn,drive_mode,lock_err_deg," + Mechanisms.SAMPLE_COLUMNS, SAMPLE_PERIOD_MS,
                () -> String.format(Locale.US, "%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.1f,%s", logX, logY, logHeadingDeg, logUp, logRight, cmdForward, cmdLeft,
                        cmdTurn, backupMode ? "backup" : fieldOriented ? "field" : "robot", lockErrDeg, mechanisms.sampleRow()));
        boolean gamepad1Was = assigned(gamepad1), gamepad2Was = assigned(gamepad2);

        boolean reseedDown = false, reseeded = false, backupHeld = false, manualLockOn = false, autoLockSuppressed = false, prevDriverLock = false;
        int salvosSeen = 0;
        ElapsedTime backupHold = new ElapsedTime();
        ElapsedTime reseedHold = new ElapsedTime();
        boolean lockOn = false, lockWasAligned = false, autoLockLatched = false, wasFull = false, pinpointBad = false;
        boolean launchWaiting = false;   // a driver launch queued until the lock is ALIGNED
        ElapsedTime launchAlignClock = new ElapsedTime();
        boolean autoLaunchOn = false, autoLaunchDone = false, autoLaunchInZone = false;   // auto-launch: the operator's toggle, taken or given up for this lock, inside a zone now
        String autoLaunchState = "";
        ElapsedTime autoLaunchZoneClock = new ElapsedTime();
        ElapsedTime autoLaunchStillClock = new ElapsedTime();
        String lastLaunchAim = "none yet";   // the aim line of the last launch (with the lock state and nudge then), repeated in the operator's reports that follow it
        HiveTarget lastLaunchCell = null;    // the cell of the last launch: the operator's report of where it landed applies to this cell
        HiveDash hiveDash = new HiveDash(profile.robotDepthIn, profile.robotWidthIn, profile.originFromBackIn);
        HiveDash.Input dashIn = new HiveDash.Input();
        ParkDash parkDash = new ParkDash(profile.robotDepthIn, profile.robotWidthIn, profile.originFromBackIn);
        ParkDash.Input parkIn = new ParkDash.Input();
        HoldPosition hold = new HoldPosition();
        boolean dashOverride = false;   // Hive Dash has taken over the target cell (the nearest-cell logic is off until it lets go)
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
            double scale = gamepad1.left_trigger > TRIGGER_THRESHOLD ? PRECISION_SCALE : 1.0;
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
            // Hive Dash (driver D-pad, held): drive to the launch spot of a cell, lock on it, launch when aligned. It uses this loop's pose and flags; nothing is read again.
            boolean counterReads = mechanisms.counter != null && mechanisms.counter.isValid();
            dashIn.pose = pose;
            dashIn.measured = measured;
            // Park Dash (driver D-pad up / down, held) shares the loop's pose and flags; the two dashes never run together
            parkIn.pose = pose;
            parkIn.measured = measured;
            parkIn.requested = hiveDash.state() != HiveDash.State.IDLE ? ParkDash.Corner.NONE : ParkDash.cornerForDpad(gamepad1.dpad_up, gamepad1.dpad_down);
            parkIn.blue = blue;
            parkIn.driverForwardRad = driverForwardRad;
            parkIn.blockedWhy = backupMode ? "backup mode" : pinpointBad ? "Pinpoint fault" : !loc.seedConfirmed ? "pose not seeded" : null;
            parkIn.otherDashBusy = hiveDash.state() != HiveDash.State.IDLE;
            long profiledPark = LoopProfiler.start();
            ParkDash.Output park = parkDash.update(parkIn);
            LoopProfiler.stop("park_dash", profiledPark);
            if (park.refused) buzz(gamepad1, 150);
            if (park.stowRamp) mechanisms.requestStowNow();   // the ramp must not be dragged into the zone; the dash does not wait for it
            dashIn.requestedCell = parkDash.state() != ParkDash.State.IDLE ? null : HiveDash.cellForDpad(gamepad1.dpad_left, gamepad1.dpad_right, blue, driverForwardRad);
            dashIn.blue = blue;
            dashIn.blockedWhy = backupMode ? "backup mode" : pinpointBad ? "Pinpoint fault" : !loc.seedConfirmed ? "pose not seeded" : null;
            dashIn.counterReadsEmpty = counterReads && mechanisms.counter.count() == 0;
            dashIn.hasElements = !counterReads || mechanisms.counter.count() > 0;
            dashIn.lockAligned = targetLock.isAligned();
            dashIn.launchBusy = mechanisms.isLaunchBusy();
            long profiledHive = LoopProfiler.start();
            HiveDash.Output dash = hiveDash.update(dashIn);
            LoopProfiler.stop("hive_dash", profiledHive);   // route planning on the first loop of a dash shows as the maximum
            if (dash.refused) buzz(gamepad1, 150);
            if (dash.overrideCell != null) {
                if (!dashOverride) {
                    dashOverride = true;
                    mechanisms.setAutoTarget(false);
                }
                mechanisms.setOpenCell(dash.overrideCell);
            } else if (dashOverride) {
                dashOverride = false;
                mechanisms.setAutoTarget(true);
            }
            if (dash.translation) {
                robotForward = dash.forward;
                robotLeft = dash.left;
                cmdForward = dash.forward;
                cmdLeft = dash.left;
            }
            if (park.translation) {
                robotForward = park.forward;
                robotLeft = park.left;
                cmdForward = park.forward;
                cmdLeft = park.left;
            }
            // target lock: HELD on the driver's right trigger (the automatic lock when full is off unless AUTO_LOCK_WHEN_FULL is set; the operator has no lock control).
            boolean driverLockHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            if (driverLockHeld != prevDriverLock) {
                prevDriverLock = driverLockHeld;
                RunLog.event("COMMAND target lock " + (driverLockHeld ? "held ON (driver right trigger)" : "released (driver right trigger)") + (backupMode ? " (no effect: backup mode)" : ""));
            }
            manualLockOn = driverLockHeld;
            if (mechanisms.getSalvosDone() != salvosSeen) {
                salvosSeen = mechanisms.getSalvosDone();
                if (autoLockLatched) RunLog.event("TARGET LOCK released: the salvo is done");
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
                if (!mechanisms.isLaunchBusy()) autoLockLatched = false;
            }
            boolean lockWanted = !backupMode && (manualLock || (AUTO_LOCK_WHEN_FULL && autoLockLatched) || dash.lockWanted);
            if (lockWanted) {
                HiveTarget lockTarget = mechanisms.getOpenCell();
                if (!lockOn) {
                    lockOn = true;
                    lockWasAligned = false;
                    RunLog.event(String.format(Locale.US, "TARGET LOCK on (%s): %s at (%.2f, %.2f), %.1f in away, robot pose (%.1f, %.1f, %.1f deg), alliance %s",
                            manualLock ? "manual" : dash.lockWanted ? "Hive Dash" : "full", lockTarget, lockTarget.x, lockTarget.y, lockTarget.distanceFrom(pose), pose.position.x, pose.position.y,
                            pose.heading.toDegrees(), blue ? "BLUE" : "RED"));
                }
                long profiledLock = LoopProfiler.start();
                cmdTurn = targetLock.turn(pose, measured == null ? 0.0 : measured.angVel, lockTarget,
                        TargetLock.farCellExtraLeftIn(lockTarget, blue), cellNudgeDeg[lockTarget.ordinal()], gamepad1.left_bumper ? gamepad1.right_stick_x : 0.0, dt);   // the nudge needs the left bumper held
                LoopProfiler.stop("target_lock", profiledLock);
                lockErrDeg = targetLock.errorDeg();
                boolean aligned = targetLock.isAligned();
                if (aligned && !lockWasAligned) {
                    RunLog.event(String.format(Locale.US, "TARGET LOCK aligned: error %+.1f deg, nudge %+.1f deg, %.1f in from %s, bearing %.1f deg, robot pose (%.1f, %.1f, %.1f deg)",
                            lockErrDeg, targetLock.trimDeg(), lockTarget.distanceFrom(pose), lockTarget, targetLock.bearingDeg(),
                            pose.position.x, pose.position.y, pose.heading.toDegrees()));
                }
                lockWasAligned = aligned;

                // auto-launch (prototype, the operator's right bumper): inside the launch zone of the locked cell the chassis stops, then one salvo goes
                // when the lock is ALIGNED and the robot has stopped. Not with an empty robot (the counter reads 0), and at most FREEZE_MAX_SEC of holding.
                boolean hasElements = mechanisms.counter == null || !mechanisms.counter.isValid() || mechanisms.counter.count() > 0;
                boolean inZone = AutoLaunch.inZone(lockTarget, pose.position.x, pose.position.y);
                if (!autoLaunchOn) {
                    autoLaunchState = "off";
                } else if (autoLaunchDone) {
                    autoLaunchState = "done for this lock";
                } else if (!hasElements) {
                    autoLaunchState = "waiting: no elements";
                } else if (!inZone) {
                    autoLaunchInZone = false;
                    autoLaunchState = String.format(Locale.US, "waiting: %.0f in from the %s launch zone", AutoLaunch.distanceFromZone(lockTarget, pose.position.x, pose.position.y) - AutoLaunch.ZONE_RADIUS_IN, lockTarget);
                } else {
                    if (!autoLaunchInZone) {
                        autoLaunchInZone = true;
                        autoLaunchZoneClock.reset();
                        autoLaunchStillClock.reset();
                        RunLog.event(String.format(Locale.US, "AUTO LAUNCH: entered the %s zone at (%.1f, %.1f), speed %.1f in/s: stopping the chassis", lockTarget,
                                pose.position.x, pose.position.y, measured == null ? 0.0 : Math.hypot(measured.linearVel.x, measured.linearVel.y)));
                    }
                    double speed = measured == null ? 0.0 : Math.hypot(measured.linearVel.x, measured.linearVel.y);
                    boolean still = aligned && speed <= AutoLaunch.MAX_SPEED_IN_PER_S;
                    if (!still) autoLaunchStillClock.reset();
                    if (autoLaunchZoneClock.seconds() > AutoLaunch.FREEZE_MAX_SEC) {
                        autoLaunchDone = true;
                        RunLog.event(String.format(Locale.US, "AUTO LAUNCH gave up after %.1f s in the zone (%s, speed %.1f in/s): the driver has the robot", autoLaunchZoneClock.seconds(),
                                aligned ? "aligned" : "not aligned", speed));
                    } else {
                        robotForward = 0.0;   // stop the chassis: the driver's translation is ignored
                        robotLeft = 0.0;
                        cmdForward = 0.0;
                        cmdLeft = 0.0;
                        autoLaunchState = String.format(Locale.US, "IN ZONE: stopping (%s, %.1f in/s)", aligned ? "aligned" : "aligning", speed);
                        if (autoLaunchStillClock.seconds() >= AutoLaunch.ALIGNED_HOLD_SEC && !mechanisms.isLaunchBusy()) {
                            autoLaunchDone = true;
                            lastLaunchAim = aimSummary(pose, mechanisms, blue, lockOn);
                            lastLaunchCell = lockTarget;
                            RunLog.event(String.format(Locale.US, "AUTO LAUNCH: %.2f s after entering the zone, speed %.1f in/s | %s", autoLaunchZoneClock.seconds(), speed, lastLaunchAim));
                            mechanisms.requestLaunch();
                        }
                    }
                }
            } else {
                if (lockOn) {
                    lockOn = false;
                    targetLock.release();
                    RunLog.event("TARGET LOCK off");
                }
                lockErrDeg = Double.NaN;
                cmdTurn = dash.turnActive ? dash.turn : park.turnActive ? park.turn : manualTurn;
                autoLaunchDone = false;
                autoLaunchInZone = false;
                autoLaunchState = autoLaunchOn ? "waiting: no target lock" : "off";
            }
            // hold position while launching: from the launch request to the end of the salvo the robot holds the pose it settled in (the sticks cancel it at once)
            boolean dashDriving = (dash.translation && (Math.abs(dash.forward) + Math.abs(dash.left) > 1e-3)) || (park.translation && (Math.abs(park.forward) + Math.abs(park.left) > 1e-3));
            boolean driverMoving = Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || (!lockOn && Math.abs(gamepad1.right_stick_x) > 0.1);
            long profiledHold = LoopProfiler.start();
            HoldPosition.Output holding = hold.update(pose, measured, !backupMode && !pinpointBad && (mechanisms.isLaunchBusy() || parkDash.holdWanted()), driverMoving || dashDriving || backupMode || pinpointBad, !lockOn);
            LoopProfiler.stop("hold", profiledHold);
            if (holding.active) {
                robotForward = holding.forward;
                robotLeft = holding.left;
                cmdForward = holding.forward;
                cmdLeft = holding.left;
                if (holding.turnActive) cmdTurn = holding.turn;
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
                    Arrays.fill(cellNudgeDeg, 0.0);   // the remembered aim corrected the drift of the old pose
                    RunLog.event("DRIVER reseed cleared the remembered aim of every cell");
                    RunLog.event(String.format("DRIVER pose reseeded to the default start (was %.1f, %.1f, %.1f deg)",
                            pose.position.x, pose.position.y, pose.heading.toDegrees()));
                }
            } else {
                reseedDown = false;
            }

            // operator
            mechanisms.setManualIntake(gamepad2.right_trigger > TRIGGER_THRESHOLD);
            mechanisms.setManualOuttake(gamepad2.left_trigger > TRIGGER_THRESHOLD);
            boolean launchNow = false;
            if (gamepad1.aWasPressed()) {
                if (LAUNCH_WAITS_FOR_ALIGN && lockOn && !targetLock.isAligned()) {
                    launchWaiting = true;
                    launchAlignClock.reset();
                    RunLog.event(String.format(Locale.US, "LAUNCH queued until the lock is ALIGNED (error %+.1f deg, at most %.1f s)", targetLock.errorDeg(), LAUNCH_ALIGN_WAIT_SEC));
                } else {
                    launchNow = true;
                }
            }
            if (launchWaiting) {
                if (!lockOn) {
                    launchWaiting = false;
                    RunLog.event("LAUNCH queue dropped: the lock was released before it aligned");
                } else if (targetLock.isAligned()) {
                    launchWaiting = false;
                    launchNow = true;
                    RunLog.event(String.format(Locale.US, "LAUNCH released: aligned after %.2f s", launchAlignClock.seconds()));
                } else if (launchAlignClock.seconds() > LAUNCH_ALIGN_WAIT_SEC) {
                    launchWaiting = false;
                    RunLog.event(String.format(Locale.US, "LAUNCH queue dropped: still not aligned after %.1f s (error %+.1f deg)", LAUNCH_ALIGN_WAIT_SEC, targetLock.errorDeg()));
                }
            }
            if (dash.launchNow) launchNow = true;   // Hive Dash: aligned and stopped at the launch spot
            if (launchNow) {
                lastLaunchAim = aimSummary(pose, mechanisms, blue, lockOn);
                lastLaunchCell = mechanisms.getOpenCell();
                RunLog.event("LAUNCH requested: " + lastLaunchAim);
                mechanisms.requestLaunch();
            }
            if (gamepad2.rightBumperWasPressed()) {
                autoLaunchOn = !autoLaunchOn;
                autoLaunchDone = false;
                autoLaunchInZone = false;
                RunLog.event("COMMAND auto-launch -> " + (autoLaunchOn ? "ON" : "off") + (backupMode ? " (no effect: backup mode)" : ""));
            }
            boolean operatorB = gamepad2.bWasPressed();   // read every loop; the reject needs the left bumper held, so it cannot be set off by accident
            if (operatorB && gamepad2.left_bumper) mechanisms.requestReject();
            if (gamepad2.xWasPressed()) mechanisms.requestDeploy();
            boolean operatorY = gamepad2.yWasPressed();   // read once: with the left bumper held, Y is the pusher calibration instead of the stow
            if (operatorY && gamepad2.left_bumper) mechanisms.requestPusherCalibration();
            else if (operatorY) mechanisms.requestStowNow();   // stow at once: no flower check
            // the operator's reports of the last salvo (unguarded D-pad, seen from behind the robot); with the left bumper held they are resets instead
            boolean reportLeft = gamepad2.dpadLeftWasPressed(), reportRight = gamepad2.dpadRightWasPressed();
            boolean reportLow = gamepad2.dpadDownWasPressed(), reportHot = gamepad2.dpadUpWasPressed();   // down = landed LOW (more speed), up = landed HIGH (less speed)
            HiveTarget reportCell = lastLaunchCell != null ? lastLaunchCell : mechanisms.getOpenCell();
            if (gamepad2.left_bumper) {
                if (reportLeft || reportRight) {
                    cellNudgeDeg[reportCell.ordinal()] = 0.0;
                    RunLog.event("COMMAND remembered aim of " + reportCell + " cleared");
                }
                if (reportLow || reportHot) mechanisms.resetSpeedTrim();
            } else {
                if (reportLeft || reportRight) {
                    double before = cellNudgeDeg[reportCell.ordinal()];
                    double after = Math.max(-NUDGE_MAX_DEG, Math.min(NUDGE_MAX_DEG, before + (reportRight ? NUDGE_STEP_DEG : -NUDGE_STEP_DEG)));   // landed LEFT: aim right (clockwise, negative)
                    cellNudgeDeg[reportCell.ordinal()] = after;
                    RunLog.event(String.format(Locale.US, "REPORT landed %s (seen from behind the robot): remembered aim for %s %+.1f -> %+.1f deg | AT THE LAST LAUNCH: %s",
                            reportLeft ? "LEFT" : "RIGHT", reportCell, before, after, lastLaunchAim));
                }
                if (reportLow || reportHot) {
                    RunLog.event(String.format(Locale.US, "REPORT landed %s: speed trim %+.0f ticks/s | AT THE LAST LAUNCH: %s", reportLow ? "LOW / UNDER" : "HIGH / OVER",
                            reportLow ? SETPOINT_STEP : -SETPOINT_STEP, lastLaunchAim));
                    mechanisms.adjustManualSetpoint(reportLow ? SETPOINT_STEP : -SETPOINT_STEP);
                }
            }

            mechanisms.setPusherLockWanted(!backupMode && (lockOn || hiveDash.state() != HiveDash.State.IDLE));   // lock the pusher during a target lock or a Hive Dash
            mechanisms.update(pose);
            if (mechanisms.takeFlywheelLostNotice()) {   // the flywheel's speed sensor died: open loop from now on; two taps on both pads
                buzzBlips(gamepad1, 2);
                buzzBlips(gamepad2, 2);
            }
            RunLog.tick();

            if (backupHeld && !backupMode) telemetry.addData("Backup mode", "keep holding LB + X... %.1f s", BACKUP_HOLD_SEC - backupHold.seconds());
            if (backupMode) telemetry.addLine(String.format(Locale.US, "*** BACKUP MODE: no pose or heading. Robot-oriented drive, launch speed fixed for %.0f in (drive to that spot), no target lock. Service the robot. ***", BACKUP_LAUNCH_DISTANCE_IN));
            else if (pinpointBad) telemetry.addLine("*** PINPOINT FAULT (backup mode in a moment if it stays) ***");
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, pose.heading.toDegrees());
            if (reseedDown && !reseeded) {
                telemetry.addData("Reseed pose", "keep holding LB + RB... %.1f s", RESEED_HOLD_SEC - reseedHold.seconds());
            }
            telemetry.addData("Drive", backupMode ? "ROBOT-ORIENTED (backup mode)" : fieldOriented ? "field-oriented" : "robot-oriented");
            if (launchWaiting) telemetry.addData("Launch", "QUEUED: waiting for the lock to be ALIGNED (%.1f s left)", Math.max(0.0, LAUNCH_ALIGN_WAIT_SEC - launchAlignClock.seconds()));
            telemetry.addData("Target lock (driver right trigger, held)", lockOn
                    ? String.format(Locale.US, "ON -> %s, bearing %.1f deg, error %+.1f deg%s", mechanisms.getOpenCell(), targetLock.bearingDeg(),
                            targetLock.errorDeg(), targetLock.isAligned() ? "   ALIGNED" : "")
                    : "off");
            telemetry.addData("Remembered aim (operator D-pad left / right)", "blue rear %+.1f, blue audience %+.1f, red rear %+.1f, red audience %+.1f deg   | speed trim %+.0f ticks/s (D-pad up / down)",
                    cellNudgeDeg[HiveTarget.BLUE_REAR.ordinal()], cellNudgeDeg[HiveTarget.BLUE_AUDIENCE.ordinal()], cellNudgeDeg[HiveTarget.RED_REAR.ordinal()],
                    cellNudgeDeg[HiveTarget.RED_AUDIENCE.ordinal()], mechanisms.getSpeedTrim());
            telemetry.addData("Auto-launch (operator RB toggles)", backupMode ? "off (backup mode)" : autoLaunchOn ? "ON: " + autoLaunchState : "off");
            telemetry.addData("Hive Dash (driver D-pad left / right, held)", hiveDash.status());
            telemetry.addData("Park Dash (driver D-pad up / down, held)", parkDash.status());
            telemetry.addData("Hold position (from the launch request to the end of the salvo)", hold.status());
            mechanisms.log(telemetry);
            telemetry.addLine("GP1: stick drive, hold LT precision, hold RT target lock, A launch, hold D-pad left / right = HIVE DASH (drive to a cell's launch spot and launch), hold D-pad up / down = PARK DASH (drive to the left / right park corner), hold LB + RB = reseed pose, hold LB + X 1 s = BACKUP MODE (no way back)");
            telemetry.addLine("GP2: intake automatic (IN, stops when full), hold RT = force in, hold LT = force out, RB auto-launch on/off, hold LB + B reject, X deploy, Y stow, D-pad = report the last salvo (left / right = aim, down = low, up = high = speed trim; hold LB to clear), hold LB + Y pusher calibration sweep");
            telemetry.addLine("BACKUP MODE (no way back): GP1 hold LB + X for 1 s, or a Pinpoint fault");
            long profiledTelemetry = LoopProfiler.start();
            telemetry.update();
            LoopProfiler.stop("telemetry", profiledTelemetry);   // the Driver Station and the Dashboard
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
