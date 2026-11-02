package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * auto saved in {@link PoseStorage}; otherwise from the alliance's default start pose: blue (-63.5, +16) facing +X against the
 * rear wall, red (+63.5, -16) facing -X against the audience wall (the same pose turned 180 deg about the center, so neither
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
 *   Back                 discard the auto pose and use the default start pose (then the D-pad picks the alliance)
 *
 * Gamepad 1 (driver):
 *   left stick           drive, field-oriented: up = away from the driver, right = the driver's right
 *   right stick X        turn
 *   right bumper (hold)  precision mode (30%)
 *   Y                    reset the heading: point the robot AWAY from you first, then press
 *   right trigger (hold) TARGET LOCK: the robot turns itself to point its front (intake and shooter end) at the nearest of your
 *                        alliance's two CELLs while
 *                        you keep driving; the turn stick nudges the aim up to 15 deg; a short buzz and ALIGNED on the screen
 *                        say it is on target. Released = back to your turn stick. Ignored while the pose override is on.
 *   Back (hold 1 s)      reseed the pose to the default start pose (held, so it cannot happen by accident)
 *   D-pad up / down      only writes MARK good / MARK off to the log (pose, heading, target), to flag what you see on the field
 *
 * Gamepad 2 (operator):
 *   right trigger        intake on / off (a toggle)
 *   left trigger (hold)  outtake
 *   A                    fire a salvo (spins up and fires when ready)
 *   B                    reject (pusher reverse + outtake); also the way out of a jam: it cancels a running salvo or a queued fire
 *   X / Y                deploy / stow the ramp (stow waits until clear of a flower)
 *   (the flywheel has no control: it runs from Start, keeps running after salvos, and stops only when the opmode ends)
 *   D-pad up / down      manual flywheel setpoint +/- 25 ticks/s (used while the shot map is empty)
 *   (the target cell is not picked: it is always the closer of your alliance's own two cells, chosen from the pose)
 *   Back                 clear a pusher fault (the pusher does not fault while Pusher.FAULTS_ENABLED is false, the current setting)
 *   hold Back + Y        pusher calibration sweep now (the track must be empty and the intake off; again to stop it)
 *
 * OVERRIDES, for when a sensor fails in a match (each press toggles; the screen and the log say what is on):
 *   gamepad 1: hold left bumper + X   drive robot-oriented (stick up = robot forward), ignoring the heading
 *   gamepad 1: hold left bumper + B   ignore the pose: a ramp stow request is carried out without the flower check
 *   (the driver has no stick-button or Start controls, so nothing can be set off by accident; the left bumper is a guard)
 *   gamepad 2 left stick button    ignore the element counter: no 4-element stop, no element warnings
 *   gamepad 2 right stick button   flywheel open loop: power from the target, ready after a timer, speed sensor ignored
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
        double startX = -(FIELD_HALF_IN - profile.robotDepthIn / 2.0);

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
            if (gamepad1.backWasPressed() && useAutoPose) {
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
                    ? String.format("AUTO pose, saved %.0f s ago (Back on gamepad 1 discards it)", PoseStorage.ageSec())
                    : blue ? "default start pose (blue: rear wall, facing +X)" : "default start pose (red: audience wall, facing -X)");
            telemetry.addData("Pinpoint", "%s   pose %.1f, %.1f, %.0f deg   %s", loc.getDeviceStatus(),
                    p.position.x, p.position.y, p.heading.toDegrees(),
                    poseOk && loc.seedConfirmed ? "OK" : "WARNING: does not match the start pose (do not move the robot after INIT; restart the opmode)");
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addData("Element counter", mechanisms.counter == null ? "not configured"
                    : mechanisms.counter.isValid() ? "reading   " + mechanisms.counter.stackString()
                    : "NOT READING (all four sensors must answer; use the operator's L3 override in the match if it stays this way)");
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
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel runs the whole time ...
        mechanisms.setSpinRequested(true);         // ... from the start, until the operator stops it (left bumper)

        RunLog.event(String.format("SETUP profile %s, alliance %s, start pose %s (%.1f, %.1f, %.1f deg), driver forward %.0f deg; %s",
                profile.name, blue ? "BLUE" : "RED", useAutoPose ? "from the AUTO" : "default",
                startPose.position.x, startPose.position.y, startPose.heading.toDegrees(), Math.toDegrees(driverForwardRad),
                mechanisms.describe()));
        RunLog.event("SETUP gamepad 1 " + (assigned(gamepad1) ? "assigned" : "NOT ASSIGNED") + ", gamepad 2 "
                + (assigned(gamepad2) ? "assigned" : "NOT ASSIGNED"));
        RunLog.setSampler("x,y,hdg_deg,stick_up,stick_right,cmd_fwd,cmd_left,cmd_turn,drive_mode,lock_err_deg," + Mechanisms.SAMPLE_COLUMNS, SAMPLE_PERIOD_MS,
                () -> String.format(Locale.US, "%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.1f,%s", logX, logY, logHeadingDeg, logUp, logRight, cmdForward, cmdLeft,
                        cmdTurn, fieldOriented ? "field" : "robot", lockErrDeg, mechanisms.sampleRow()));
        boolean gamepad1Was = assigned(gamepad1), gamepad2Was = assigned(gamepad2);

        boolean prevIntakeTrigger = false;
        boolean backDown = false, reseeded = false;
        ElapsedTime backHold = new ElapsedTime();
        boolean lockOn = false, lockWasAligned = false, lockRefusalLogged = false;
        ElapsedTime loopClock = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = loopClock.seconds();
            loopClock.reset();
            PoseVelocity2d measured = drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            PoseStorage.save(pose, blue);
            double heading = pose.heading.toDouble();

            // driver: stick up = away from the driver, stick right = the driver's right, both in field terms
            double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
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
            boolean lockWanted = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            boolean lockUsable = lockWanted && !mechanisms.isPoseOverridden();
            if (lockUsable) {
                HiveTarget lockTarget = mechanisms.getOpenCell();
                if (!lockOn) {
                    lockOn = true;
                    lockWasAligned = false;
                    RunLog.event(String.format(Locale.US, "TARGET LOCK on: %s at (%.2f, %.2f), %.1f in away, robot pose (%.1f, %.1f, %.1f deg), alliance %s",
                            lockTarget, lockTarget.x, lockTarget.y, lockTarget.distanceFrom(pose), pose.position.x, pose.position.y,
                            pose.heading.toDegrees(), blue ? "BLUE" : "RED"));
                }
                cmdTurn = targetLock.turn(pose, measured == null ? 0.0 : measured.angVel, lockTarget, gamepad1.right_stick_x, dt);
                lockErrDeg = targetLock.errorDeg();
                boolean aligned = targetLock.isAligned();
                if (aligned && !lockWasAligned) {
                    gamepad1.rumble(120);
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
                if (lockWanted && !lockRefusalLogged) RunLog.event("TARGET LOCK refused: the pose override is on");
                lockRefusalLogged = lockWanted;
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
            boolean overrideDrive = gamepad1.xWasPressed(), overridePose = gamepad1.bWasPressed();
            if (gamepad1.left_bumper && overrideDrive) {
                fieldOriented = !fieldOriented;
                RunLog.event("OVERRIDE drive " + (fieldOriented ? "FIELD-oriented" : "ROBOT-oriented (heading ignored)"));
            }
            if (gamepad1.left_bumper && overridePose) mechanisms.togglePoseOverride();
            if (gamepad2.leftStickButtonWasPressed()) mechanisms.toggleCounterOverride();
            if (gamepad2.rightStickButtonWasPressed()) mechanisms.toggleFlywheelOpenLoop();

            boolean markGood = gamepad1.dpadUpWasPressed(), markOff = gamepad1.dpadDownWasPressed();
            if (markGood || markOff) {
                RunLog.event(String.format(Locale.US, "MARK %s: robot pose (%.1f, %.1f, %.1f deg), alliance %s, target %s, lock %s",
                        markGood ? "good" : "off", pose.position.x, pose.position.y, pose.heading.toDegrees(), blue ? "BLUE" : "RED",
                        mechanisms.getOpenCell(), lockOn ? "on" : "off"));
            }
            if (gamepad1.yWasPressed()) {
                drive.setPose(new Pose2d(pose.position, driverForwardRad));
                RunLog.event(String.format("DRIVER heading reset: was %.1f deg, now %.1f deg", pose.heading.toDegrees(), Math.toDegrees(driverForwardRad)));
            }
            if (gamepad1.back) {
                if (!backDown) {
                    backDown = true;
                    reseeded = false;
                    backHold.reset();
                }
                if (!reseeded && backHold.seconds() >= RESEED_HOLD_SEC) {
                    drive.setPose(defaultStart);
                    reseeded = true;
                    RunLog.event(String.format("DRIVER pose reseeded to the default start (was %.1f, %.1f, %.1f deg)",
                            pose.position.x, pose.position.y, pose.heading.toDegrees()));
                }
            } else {
                backDown = false;
            }

            // operator
            boolean intakeTrigger = gamepad2.right_trigger > TRIGGER_THRESHOLD;
            if (intakeTrigger && !prevIntakeTrigger) mechanisms.toggleIntake();
            prevIntakeTrigger = intakeTrigger;
            mechanisms.setManualOuttake(gamepad2.left_trigger > TRIGGER_THRESHOLD);
            if (gamepad2.aWasPressed()) {
                if (lockOn) {
                    RunLog.event(String.format(Locale.US, "FIRE requested with the lock %s (error %+.1f deg)",
                            targetLock.isAligned() ? "ALIGNED" : "not aligned", targetLock.errorDeg()));
                }
                mechanisms.requestFire();
            }
            if (gamepad2.bWasPressed()) mechanisms.requestReject();
            if (gamepad2.xWasPressed()) mechanisms.requestDeploy();
            boolean operatorY = gamepad2.yWasPressed();   // read once: with Back held, Y is the pusher calibration instead of the stow
            if (operatorY && gamepad2.back) mechanisms.requestPusherCalibration();
            else if (operatorY) mechanisms.requestStow();
            if (gamepad2.dpadUpWasPressed()) mechanisms.adjustManualSetpoint(SETPOINT_STEP);
            if (gamepad2.dpadDownWasPressed()) mechanisms.adjustManualSetpoint(-SETPOINT_STEP);
            if (gamepad2.backWasPressed()) mechanisms.clearPusherStall();

            mechanisms.update(pose);
            RunLog.tick();

            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f",
                    pose.position.x, pose.position.y, pose.heading.toDegrees());
            if (gamepad1.back && !reseeded) {
                telemetry.addData("Reseed pose", "keep holding Back... %.1f s", RESEED_HOLD_SEC - backHold.seconds());
            }
            telemetry.addData("Drive", fieldOriented ? "field-oriented" : "ROBOT-ORIENTED (override: heading ignored)");
            telemetry.addData("Target lock (hold driver RT)", lockOn
                    ? String.format(Locale.US, "ON -> %s, bearing %.1f deg, error %+.1f deg%s", mechanisms.getOpenCell(), targetLock.bearingDeg(),
                            targetLock.errorDeg(), targetLock.isAligned() ? "   ALIGNED" : "")
                    : "off");
            mechanisms.log(telemetry);
            telemetry.addLine("GP1: stick drive, RB precision, hold RT target lock, Y reset heading (face away first), Back held = reseed");
            telemetry.addLine("GP2: RT intake on/off, LT outtake (hold), A fire, B reject, X deploy, Y stow, RB/LB spin/stop, D-pad up/down speed trim, Back clear pusher fault, hold Back + Y pusher calibration sweep");
            telemetry.addLine("OVERRIDES (press again to undo): GP1 hold LB + X robot-oriented drive, GP1 hold LB + B ignore pose (ramp stow), GP2 L3 ignore element counter, GP2 R3 flywheel open loop");
            telemetry.update();
        }
        drive.stop();
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
        RunLog.event("STOPPED by the driver station");
    }
}
