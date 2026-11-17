package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.aiming.AutoLaunch;
import org.firstinspires.ftc.teamcode.aiming.LaunchGrid;
import org.firstinspires.ftc.teamcode.aiming.TargetLock;
import org.firstinspires.ftc.teamcode.drive.Handover;
import org.firstinspires.ftc.teamcode.drive.Localizer;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
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
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.robot.Footprint;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * TeleOp - Competition (specs/teleop.md): the driver-controlled period. A driver (gamepad 1) moves the chassis; an
 * operator (gamepad 2) looks after the mechanisms. Neither controller uses Start or Back, and the driver's stick
 * buttons are unused. Nothing is driven in INIT (rule G403).
 */
@Config
@TeleOp(name = "TeleOp - Competition", group = "1 Competition")
public final class CompetitionTeleOp extends TeamOpMode {
    public static double POSE_FRESH_S = 300.0;
    public static double TURN_SCALE = 0.5;
    public static double PRECISION_SCALE = 0.30;
    public static double TRIGGER = 0.3;
    public static double STICK_MOVING = 0.05;
    public static double LAUNCH_WAITS_FOR_ALIGNED_S = 1.5;
    public static double AIM_STEP_DEG = 3.0, AIM_LIMIT_DEG = 10.0, SPEED_STEP = 10.0;
    public static double RESEED_HOLD_S = 1.0, BACKUP_HOLD_S = 1.0;
    public static double PINPOINT_FAULT_S = 0.5, BACKUP_DISTANCE_IN = 42.0;
    public static double DRIVE_HOLD_FAILSAFE_S = 2.0;
    public static boolean AUTO_LAUNCH_ARMED_BY_FULL = true;
    public static double SAMPLE_PERIOD_S = 0.100;
    /** The verbose screen, for the pit: every value and the full mechanism readout. */
    public static boolean VERBOSE_SCREEN = false;
    /** Build-time switches: a live change would be unsafe. */
    public static final boolean DRIVE_HOLD_DURING_SWEEP = true;
    public static final boolean AUTO_LOCK_WHEN_FULL_BY_SLOTS = false;

    // Kept until the app restarts.
    private static boolean fullAuto = true;
    private static Alliance chosen = Alliance.BLUE;

    private Alliance alliance;
    private Footprint footprint;
    private final TargetLock lock = new TargetLock();
    private final AutoLaunch autoLaunch = new AutoLaunch();
    private final HoldPosition hold = new HoldPosition();
    private HiveDash hiveDash;
    private ParkDash parkDash;
    private final HiveDash.In hiveIn = new HiveDash.In();
    private final ParkDash.In parkIn = new ParkDash.In();

    /** The operator's remembered aim per cell, degrees, in the Cell order. */
    private final double[] aims = new double[Cell.values().length];
    private Cell lastLaunchCell;
    private String lastAimLine = "";

    private boolean backup, seededOk, usingSaved, autoLaunchOn, lockOn, waitingForAim, driveHeld;
    private double waitingSince, faultSince = -1, driveHoldSince = -1, lastOverlay;
    private Pose pose = new Pose(0, 0, 0), driveHoldStart;
    private double stickUp, stickRight, lockErrorDeg = Double.NaN;
    private String driveMode = "field";

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.TELEOP; }
    @Override protected String title() { return "TeleOp - Competition"; }

    private Pose defaultStart(Alliance a) { return a.pose(Field.startRear(profile.originToBack)); }

    @Override
    protected void body() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        profile = ChassisProfile.get();
        footprint = profile.footprint();

        // ---- INIT: everything is built; nothing is driven ----
        Pose saved = Handover.freshPose(POSE_FRESH_S);
        double savedAge = Handover.poseAgeS();
        usingSaved = saved != null;
        if (usingSaved) {
            alliance = Alliance.of(Handover.red());
            drive = MecanumDrive.adopt(hardwareMap, profile, saved, Field.HALF_IN);
        } else {
            alliance = chosen;
            drive = MecanumDrive.resetAndSeed(hardwareMap, profile, defaultStart(alliance));
        }
        boolean adopted = drive.localizer.mode() == Localizer.Mode.ADOPTED;
        int savedCount = Handover.freshCount(POSE_FRESH_S);
        double countAge = Handover.countAgeS();
        mech = new Mechanisms(hardwareMap, profile, alliance, Mechanisms.Settings.teleop(Math.max(0, savedCount)));
        // The remembered aims continue only when the device's pose is adopted: they correct that pose's drift.
        double[] savedAims = Handover.aims();
        boolean aimsContinued = adopted && savedAims != null && savedAims.length == aims.length;
        if (aimsContinued) {
            System.arraycopy(savedAims, 0, aims, 0, aims.length);
            RunLog.eventf("REMEMBERED AIM continued from the last TeleOp: %s", aimsText());
        } else {
            RunLog.event("REMEMBERED AIM cleared at INIT (" + (adopted ? "none saved" : "the pose was not adopted") + ")");
        }
        hiveDash = new HiveDash(footprint, new HiveDash.AimSource() {
            @Override public Vec launchSpot(Cell cell) { return Aim.launchSpot(cell); }
            @Override public double aimHeading(Vec from, Cell cell) { return Aim.forCell(from.x, from.y, cell, aims[cell.ordinal()], 0); }
        });
        parkDash = new ParkDash(footprint);
        // Warm up the planner, so the first press does not pay for it.
        for (Cell c : Cell.values()) RoutePlanner.plan(new Vec(0, 50), Aim.launchSpot(c), true, footprint);
        RunLog.eventf("PINPOINT at INIT: %s; start pose from %s", drive.localizer.mode(), usingSaved
                ? String.format(Locale.US, "a pose saved %.0f s ago", savedAge) : "the default start");

        Edge dRight = new Edge(), dLeft = new Edge(), dUp = new Edge(), xBtn = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            pose = drive.pose();
            mech.initLoop();
            boolean changed = false;
            if (dUp.pressed(gamepad1.dpad_up) && usingSaved) {
                // Discard what the last opmode saved: the pose, the remembered aims that went with it, and the CS5 held count.
                Handover.discard();
                usingSaved = false;
                aimsContinued = false;
                java.util.Arrays.fill(aims, 0);
                savedCount = -1;
                mech.setStartHeld(0, "the saved count was discarded at INIT");
                alliance = chosen;
                changed = true;
                RunLog.event("DRIVER discarded the saved pose, the remembered aims and the held count at INIT");
            }
            boolean right = dRight.pressed(gamepad1.dpad_right), left = dLeft.pressed(gamepad1.dpad_left);
            if (!usingSaved && (right || left)) {
                Alliance a = right ? Alliance.BLUE : Alliance.RED;
                changed |= a != alliance;
                alliance = chosen = a;
            }
            if (changed) {
                // Re-seed the position on the device at once, without recalibrating its IMU.
                mech.setAlliance(alliance);
                drive.localizer.seed(defaultStart(alliance));
                RunLog.eventf("PINPOINT re-seeded at INIT for %s: %s, %s", alliance, defaultStart(alliance),
                        drive.localizer.seedConfirmed() ? "confirmed" : "NOT CONFIRMED");
            }
            if (xBtn.pressed(gamepad1.x)) fullAuto = !fullAuto;
            initScreen(saved, savedAge, adopted && usingSaved, aimsContinued, savedCount, countAge);
            RunLog.tick();
            idle();
        }
        if (isStopRequested()) return;

        // ---- At Start ----
        markStarted();
        seededOk = poseAcceptable(adopted && usingSaved);
        mech.stowRampNow();
        mech.commandFlyPosPollen();
        mech.setTargetCell(nearestCell(pose));
        mech.settings.targetFollowsPose = true;
        mech.settings.intakeAlwaysOn = true;
        mech.settings.flywheelStopsAfterSalvo = false;
        mech.setSpin(true);
        mech.settings.fullActions = fullAuto;
        mech.settings.launchGridAllowed = true;
        RunLog.eventf("SETUP profile %s%s, alliance %s, start pose %s %s (%s), mechanisms: %s, %s, gamepads: driver %s, operator %s",
                profile.name, ChassisProfile.noHubRegistered() ? " (NO HUB REGISTERED)" : "", alliance, usingSaved ? "from the AUTO" : "default", pose, drive.localizer.mode(),
                mech.found(), fullAuto ? "FULL AUTO" : "COUNT ONLY", assigned(gamepad1) ? "assigned" : "NOT ASSIGNED",
                assigned(gamepad2) ? "assigned" : "NOT ASSIGNED");
        RunLog.eventf("PINPOINT at Start: %s, pose %s, %s", drive.localizer.statusText(), pose, seededOk ? "acceptable" : "NOT ACCEPTABLE");
        RunLog.sampler("x,y,hdg_deg,stick_up,stick_right," + MecanumDrive.LOG_COLUMNS.replace(",a_lf,a_lb,a_rb,a_rf", "")
                + ",drive_mode,lock_err," + Mechanisms.LOG_COLUMNS + ",a_lf,a_lb,a_rb,a_rf,grid_zone,grid_speed,grid_resid_deg",
                SAMPLE_PERIOD_S, new RunLog.Sampler() {
                    @Override public String row() { return sampleRow(); }
                });

        Edge launchBtn = new Edge(), rejectBtn = new Edge(), deployBtn = new Edge(), stowBtn = new Edge(), sweepBtn = new Edge(),
                autoBtn = new Edge(), repLeft = new Edge(), repRight = new Edge(), repDown = new Edge(), repUp = new Edge(), gridBtn = new Edge();
        Hold reseedHold = new Hold(), backupHold = new Hold();
        MatchScreen.View view = new MatchScreen.View();
        double lastT = Clock.now();

        while (opModeIsActive()) {
            double t = Clock.now(), dt = Math.max(1e-3, t - lastT);
            lastT = t;

            // ---- 1. Read the pose (unless in backup mode) and watch for a Pinpoint fault; save the pose ----
            boolean faultNow = false;
            if (!backup) {
                drive.updatePose();
                pose = drive.pose();
                faultNow = drive.localizer.faultNow();
                if (faultNow) {
                    if (faultSince < 0) faultSince = t;
                    if (t - faultSince >= PINPOINT_FAULT_S) enterBackup("the Pinpoint has faulted for " + PINPOINT_FAULT_S + " s: " + drive.localizer.statusText());
                } else {
                    if (faultSince >= 0) RunLog.eventf("PINPOINT blip: a fault for %.2f s, nothing changed", t - faultSince);
                    faultSince = -1;
                }
                if (!backup && !faultNow) {
                    Handover.savePose(pose, alliance.isRed());
                    Handover.saveAims(aims);
                }
            }
            Handover.saveCount(mech.heldCountForSave());     // the TeleOp keeps saving its count in backup mode
            double vx = drive.localizer.vx(), vy = drive.localizer.vy(), omega = drive.localizer.omega(), speed = Math.hypot(vx, vy);

            // ---- 2. Read the driver's sticks ----
            stickUp = -gamepad1.left_stick_y;
            stickRight = gamepad1.left_stick_x;
            double turnStick = gamepad1.right_stick_x;
            boolean precision = gamepad1.left_trigger > TRIGGER;
            boolean lb = gamepad1.left_bumper;
            boolean translating = Math.hypot(stickUp, stickRight) > STICK_MOVING, turning = Math.abs(turnStick) > STICK_MOVING;

            // ---- 3. Park Dash, then Hive Dash; apply the Hive Dash's cell override and launch distance ----
            String unavailable = backup ? "backup mode" : faultNow ? "a Pinpoint fault" : !seededOk ? "the pose was not seeded" : null;
            LoopProfiler.begin("park_dash");
            parkIn.pose = pose;
            parkIn.omega = omega;
            parkIn.t = t;
            parkIn.up = gamepad1.dpad_up;
            parkIn.down = gamepad1.dpad_down;
            parkIn.alliance = alliance;
            parkIn.unavailable = unavailable;
            parkIn.hiveDashRunning = hiveDash.running();
            ParkDash.Out park = parkDash.update(parkIn);
            LoopProfiler.end("park_dash");
            if (park.stowRamp) mech.stowRampNow();

            LoopProfiler.begin("hive_dash");
            hiveIn.pose = pose;
            hiveIn.vx = vx;
            hiveIn.vy = vy;
            hiveIn.omega = omega;
            hiveIn.t = t;
            hiveIn.left = gamepad1.dpad_left;
            hiveIn.right = gamepad1.dpad_right;
            hiveIn.alliance = alliance;
            hiveIn.unavailable = unavailable;
            hiveIn.parkDashRunning = parkDash.running();
            hiveIn.readsEmpty = mech.readsEmpty();
            hiveIn.lockAligned = lock.aligned();
            hiveIn.salvoRunning = mech.salvoRunning();
            hiveIn.salvoCount = mech.salvoCount();
            HiveDash.Out dash = hiveDash.update(hiveIn);
            LoopProfiler.end("hive_dash");
            if (dash.cell != null) mech.setDash(dash.cell, Aim.launchSpot(dash.cell), dash.launchDistance);
            else mech.setDash(null, null, 0);

            // ---- 4. Decide the target lock and compute its turn; run auto-launch ----
            LoopProfiler.begin("target_lock");
            Cell cell = dash.cell != null ? dash.cell : (mech.targetCell() != null ? mech.targetCell() : nearestCell(pose));
            boolean lockWanted = !backup && (gamepad1.right_bumper || dash.wantsLock
                    || (AUTO_LOCK_WHEN_FULL_BY_SLOTS && mech.slots != null && mech.slots.valid() && mech.slots.full()));
            if (lockWanted && !lockOn) {
                RunLog.eventf("TARGET LOCK on (%s) at %s, cell %s", dash.wantsLock ? "Hive Dash" : "driver", pose, cell.label());
            } else if (!lockWanted && lockOn) {
                RunLog.eventf("TARGET LOCK off at %s, error %.1f deg", pose, lock.errorDeg());
                lock.release();
            }
            boolean wasAligned = lock.aligned();
            lockOn = lockWanted;
            double lockTurn = 0;
            LaunchGrid.Result grid = mech.grid();
            boolean gridOn = LaunchGrid.ENABLED && grid != null && grid.hasValues() && !Double.isNaN(grid.residualRad);
            if (lockOn) {
                lockTurn = lock.update(pose, omega, cell, aims[cell.ordinal()], lb ? turnStick : 0, dt, gridOn ? grid.residualRad : Double.NaN);
                if (lock.aligned() && !wasAligned) RunLog.eventf("TARGET LOCK aligned on %s: error %.1f deg, nudge %.1f deg", cell.label(), lock.errorDeg(), lock.nudgeDeg());
            }
            lockErrorDeg = lockOn ? lock.errorDeg() : Double.NaN;
            mech.setAimLockWanted(lockOn || hiveDash.running());

            boolean armed = !backup && (autoLaunchOn || (AUTO_LAUNCH_ARMED_BY_FULL && fullAuto && mech.full()));
            autoLaunch.update(armed, lockOn && !hiveDash.running(), mech.bothReadEmpty(), cell, pose.x, pose.y, lock.aligned(), speed, t);
            if (autoLaunch.event != null) RunLog.event(autoLaunch.event);
            if (autoLaunch.requestLaunch) launch("auto-launch", cell);
            LoopProfiler.end("target_lock");

            // ---- 5. Decide the drive hold; update hold position ----
            LoopProfiler.begin("hold");
            boolean sweep = DRIVE_HOLD_DURING_SWEEP && !backup && mech.salvoRunning() && mech.inLaunchSweep();
            if (sweep && driveHoldSince < 0) {
                driveHoldSince = t;
                driveHoldStart = pose;
            }
            boolean held = sweep && t - driveHoldSince <= DRIVE_HOLD_FAILSAFE_S;
            if (!sweep && driveHoldSince >= 0) {
                RunLog.eventf("DRIVE HELD for %.2f s during the sweep: moved %.1f in, driver stick %.2f", t - driveHoldSince,
                        pose.distTo(driveHoldStart), Math.hypot(stickUp, stickRight));
                driveHoldSince = -1;
            }
            driveHeld = held;
            boolean holdWanted = !backup && (mech.launchQueued() || mech.salvoRunning() || park.parked);
            boolean holdCancel = backup || faultNow || dash.active && !dash.wantsLock || park.driving
                    || (!held && (translating || (turning && !lockOn)));
            if (dash.active && dash.wantsLock) holdCancel = backup || faultNow;      // the dash has arrived and is launching
            hold.update(pose, vx, vy, omega, holdWanted, holdCancel, !lockOn, held, t, dt);
            LoopProfiler.end("hold");

            // ---- 6. Send the drive command: translation and turn are decided separately, later rows overriding earlier ones ----
            double forward, left, turn;
            double scale = precision ? PRECISION_SCALE : 1.0;
            if (backup) {
                // Robot-oriented: stick up is the robot's front.
                forward = stickUp * scale;
                left = -stickRight * scale;
                driveMode = "backup";
            } else {
                double th = alliance.driverForwardRad();
                double fx = stickUp * Math.cos(th) + stickRight * Math.sin(th), fy = stickUp * Math.sin(th) - stickRight * Math.cos(th);
                forward = (fx * Math.cos(pose.h) + fy * Math.sin(pose.h)) * scale;
                left = (-fx * Math.sin(pose.h) + fy * Math.cos(pose.h)) * scale;
                driveMode = "field";
            }
            turn = -turnStick * TURN_SCALE * scale;
            if (dash.active) {
                forward = dash.forward;
                left = dash.left;
                turn = dash.hasTurn ? dash.turn : 0;
                driveMode = "hive_dash";
            }
            if (park.active) {
                forward = park.forward;
                left = park.left;
                turn = park.turn;
                driveMode = "park_dash";
            }
            if (autoLaunch.holdChassis) {
                forward = 0;
                left = 0;
            }
            if (lockOn) turn = lockTurn;
            if (held) {
                forward = 0;
                left = 0;
            }
            if (hold.active) {
                forward = hold.forward;
                left = hold.left;
                if (hold.hasTurn && !lockOn) turn = hold.turn;
            }
            drive.drive(forward, left, turn);

            // ---- 7. The hold chords ----
            boolean reseedChord = lb && gamepad1.right_trigger > TRIGGER && !backup, backupChord = lb && gamepad1.x && !backup;
            if (reseedHold.held(reseedChord, t, RESEED_HOLD_S)) {
                // The fix for a drifted pose or a wrong heading. The robot should be on the default start spot.
                Pose start = defaultStart(alliance);
                RunLog.eventf("DRIVER reseed: pose %s -> %s; remembered aims cleared (%s)", pose, start, aimsText());
                drive.localizer.setPose(start);
                java.util.Arrays.fill(aims, 0);
                hold.reset();
                seededOk = true;
            }
            if (backupHold.held(backupChord, t, BACKUP_HOLD_S)) enterBackup("the driver held left bumper + X");

            // ---- 8. The operator's controls and the driver's launch ----
            boolean olb = gamepad2.left_bumper;
            mech.setOuttake(gamepad2.left_trigger > TRIGGER);
            mech.setOperatorForceIn(gamepad2.right_trigger > TRIGGER);
            if (rejectBtn.pressed(olb && gamepad2.b)) {
                waitingForAim = false;
                mech.requestReject("operator");
            }
            if (deployBtn.pressed(gamepad2.x)) mech.deployRamp();
            boolean y = gamepad2.y;
            if (sweepBtn.pressed(olb && y)) mech.toggleCalibrationSweep();
            if (stowBtn.pressed(!olb && y)) mech.stowRampNow();
            if (autoBtn.pressed(gamepad2.right_bumper)) {
                autoLaunchOn = !autoLaunchOn;
                RunLog.event("AUTO LAUNCH switched " + (autoLaunchOn ? "on" : "off") + " by the operator");
            }
            boolean rl = repLeft.pressed(gamepad2.dpad_left), rr = repRight.pressed(gamepad2.dpad_right),
                    rd = repDown.pressed(gamepad2.dpad_down), ru = repUp.pressed(gamepad2.dpad_up);
            if (olb) {
                if ((rl || rr) && lastLaunchCell != null) {
                    aims[lastLaunchCell.ordinal()] = 0;
                    RunLog.eventf("REPORT cleared the remembered aim of %s", lastLaunchCell.label());
                }
                if (rd || ru) {
                    mech.setSpeedTrim(0);
                    RunLog.event("REPORT cleared the speed trim");
                }
            } else {
                if (rl) report("LEFT", -AIM_STEP_DEG, 0);       // landed left: aim clockwise
                if (rr) report("RIGHT", AIM_STEP_DEG, 0);
                if (rd) report("LOW", 0, SPEED_STEP);
                if (ru) report("HIGH", 0, -SPEED_STEP);
            }
            if (gridBtn.pressed(olb && gamepad2.a)) {
                LaunchGrid.ENABLED = !LaunchGrid.ENABLED;
                RunLog.event("COMMAND launch grid " + (LaunchGrid.ENABLED ? "on" : "off"));
            }
            // Driver A: launches at once, locked or not. If the lock is on but not yet aligned, the launch waits for aligned.
            if (launchBtn.pressed(gamepad1.a)) {
                if (lockOn && !lock.aligned()) {
                    waitingForAim = true;
                    waitingSince = t;
                    RunLog.eventf("LAUNCH held: waiting for the lock to align (error %.1f deg)", lock.errorDeg());
                } else {
                    launch("driver", cell);
                }
            }
            if (waitingForAim) {
                if (!lockOn) {
                    waitingForAim = false;
                    RunLog.event("LAUNCH dropped: the lock was released while waiting for aligned");
                } else if (lock.aligned()) {
                    waitingForAim = false;
                    RunLog.eventf("LAUNCH released: aligned after %.2f s", t - waitingSince);
                    launch("driver", cell);
                } else if (t - waitingSince > LAUNCH_WAITS_FOR_ALIGNED_S) {
                    waitingForAim = false;
                    RunLog.eventf("LAUNCH dropped: not aligned within %.1f s", LAUNCH_WAITS_FOR_ALIGNED_S);
                }
            }
            if (dash.requestLaunch) launch("Hive Dash", cell);

            // ---- 9. Update the mechanisms ----
            mech.update(backup ? null : pose, !backup);

            // ---- 10. The run log's tick; the screen ----
            RunLog.tick();
            LoopProfiler.begin("telemetry");
            view.alliance = alliance;
            view.pose = pose;
            view.backup = backup;
            view.pinpointFault = faultNow;
            view.driveHeld = driveHeld;
            view.autoLaunchArmed = armed;
            view.lockOn = lockOn;
            view.lockAligned = lock.aligned();
            view.waitingForAim = waitingForAim;
            view.countOnly = !fullAuto;
            view.wheelAlert = drive.wheelCheck.flaggedText();
            view.dashRefusal = dash.refusal;
            view.parkRefusal = park.refusal;
            view.dashActivity = dash.activity;
            view.parkActivity = park.activity;
            view.backupIn = backupHold.remaining(t, BACKUP_HOLD_S);
            view.reseedIn = reseedHold.remaining(t, RESEED_HOLD_S);
            view.aimRear = aims[Cell.of(alliance, true).ordinal()];
            view.aimAudience = aims[Cell.of(alliance, false).ordinal()];
            view.noShotZone = LaunchGrid.ENABLED && grid != null && grid.zone == LaunchGrid.Zone.NO_SHOT && (lockOn || mech.launchQueued());
            if (VERBOSE_SCREEN) verboseScreen();
            else telemetry.addLine(MatchScreen.build(view, mech));
            telemetry.update();
            overlay(t);
            LoopProfiler.end("telemetry");
        }
    }

    // ---- Helpers ----

    private Cell nearestCell(Pose p) {
        Cell rear = Cell.of(alliance, true), aud = Cell.of(alliance, false);
        return Aim.distance(p.x, p.y, rear) <= Aim.distance(p.x, p.y, aud) ? rear : aud;
    }

    private static boolean assigned(Gamepad g) {
        return g.getGamepadId() != Gamepad.ID_UNASSOCIATED;
    }

    private String aimsText() {
        return String.format(Locale.US, "REAR %+.0f, AUDIENCE %+.0f deg", aims[Cell.of(alliance, true).ordinal()], aims[Cell.of(alliance, false).ordinal()]);
    }

    /** One way: backup mode lasts until the opmode ends. */
    private void enterBackup(String why) {
        if (backup) return;
        backup = true;
        RunLog.event("BACKUP MODE ENTERED: " + why);
        mech.enterBackup(BACKUP_DISTANCE_IN);
        mech.setDash(null, null, 0);
        lock.release();
        lockOn = false;
        waitingForAim = false;
        hold.reset();
    }

    /** Requests a salvo and logs the full aim line. */
    private void launch(String who, Cell cell) {
        LaunchGrid.Result g = mech.grid();
        lastAimLine = String.format(Locale.US,
                "pose %s, cell %s, distance %.1f in, bearing %.1f deg, predicted miss %.1f in, setpoint %.0f (map %.0f, trim %+.0f), lock %s (error %.1f deg), "
                        + "remembered aim %+.1f deg, grid %s",
                backup ? "none" : pose.toString(), cell.label(), mech.distance(), Math.toDegrees(Aim.bearing(pose.x, pose.y, cell)),
                Aim.missIn(pose.x, pose.y, pose.h, cell), mech.setpoint(), mech.mapSpeed(), mech.speedTrim(),
                lockOn ? (lock.aligned() ? "aligned" : "on") : "off", lockErrorDeg, aims[cell.ordinal()],
                g == null ? "none" : String.format(Locale.US, "%s %.0f ticks/s %.1f deg%s", g.zone, g.speed, Math.toDegrees(g.residualRad), mech.gridUsed() ? " (in use)" : ""));
        RunLog.event("LAUNCH requested (" + who + "): " + lastAimLine);
        if (mech.requestLaunch(who)) lastLaunchCell = cell;
    }

    /** An operator report of where the last salvo landed, as seen from behind the robot. It applies to the cell of the last launch. */
    private void report(String what, double aimStep, double speedStep) {
        if (aimStep != 0 && lastLaunchCell != null) {
            int i = lastLaunchCell.ordinal();
            aims[i] = Angle.clamp(aims[i] + aimStep, -AIM_LIMIT_DEG, AIM_LIMIT_DEG);
        }
        if (speedStep != 0) mech.setSpeedTrim(mech.speedTrim() + speedStep);
        RunLog.eventf("REPORT landed %s: remembered aim of %s now %+.1f deg, speed trim %+.0f; the launch was: %s", what,
                lastLaunchCell == null ? "no cell yet" : lastLaunchCell.label(), lastLaunchCell == null ? 0 : aims[lastLaunchCell.ordinal()],
                mech.speedTrim(), lastAimLine);
    }

    private boolean poseAcceptable(boolean adopted) {
        if (adopted) return drive.localizer.ready();
        Pose s = defaultStart(alliance);
        return drive.localizer.seedConfirmed() && pose.distTo(s) <= 1.5 && Math.abs(Math.toDegrees(Angle.diff(pose.h, s.h))) <= 3.0;
    }

    private void initScreen(Pose saved, double savedAge, boolean adopted, boolean aimsContinued, int savedCount, double countAge) {
        boolean ok = poseAcceptable(adopted);
        StringBuilder sb = new StringBuilder();
        sb.append("<big>").append(alliance).append("</big>  ").append(fullAuto ? "FULL AUTO" : "COUNT ONLY").append(" (X)<br>");
        if (ChassisProfile.noHubRegistered()) sb.append("<font color='#ffd000'>no hub registered yet</font><br>");
        if (usingSaved) {
            sb.append(String.format(Locale.US, "start: saved %.0f s ago, %s<br>", savedAge, drive.localizer.mode()));
            if (saved != null && (pose.distTo(saved) > 3.0 || Math.abs(Math.toDegrees(Angle.diff(pose.h, saved.h))) > 5.0)) {
                sb.append("<font color='#ffd000'>differs from the saved pose: CHECK</font><br>");
            }
            sb.append("D-pad up: discard the saved pose<br>");
        } else {
            sb.append("start: default, back on the wall. D-pad R/L: BLUE/RED<br>");
        }
        sb.append(String.format(Locale.US, "pose %.1f, %.1f, %.0f deg: ", pose.x, pose.y, pose.headingDeg()))
                .append(ok ? "<font color='#40ff40'>OK</font>" : "<font color='#ff4040'>NOT OK</font>").append("<br>");
        sb.append("found: ").append(mech.found()).append("<br>");
        if (aimsContinued) sb.append("aims continued: ").append(aimsText()).append("<br>");
        sb.append(mech.entry == null ? "CS5 missing: no held count" : savedCount >= 0
                ? String.format(Locale.US, "held count starts at %d (saved %.0f s ago)", savedCount, countAge) : "held count starts at 0").append("<br>");
        sb.append("ramp: position unknown until Start<br>");
        sb.append(mech.flyPos == null ? "fly_pos missing: not commanded" : "fly_pos: POLLEN at Start").append("<br>");
        sb.append("slot count: ").append(mech.slots == null ? "no sensors" : mech.slots.valid() ? mech.slots.count() + " (" + mech.slots.slots() + ")" : "NOT READING").append("<br>");
        sb.append("gamepads: ").append(assigned(gamepad1) ? "driver ok" : "<font color='#ff4040'>NO DRIVER</font>").append(", ")
                .append(assigned(gamepad2) ? "operator ok" : "<font color='#ff4040'>NO OPERATOR</font>");
        telemetry.addLine(sb.toString());
        telemetry.update();
    }

    private void verboseScreen() {
        StringBuilder sb = new StringBuilder();
        sb.append(alliance).append(backup ? " BACKUP" : "").append(fullAuto ? " FULL AUTO" : " COUNT ONLY").append("<br>");
        sb.append(String.format(Locale.US, "pose %s, %s, drive %s<br>", pose, drive.localizer.statusText(), driveMode));
        sb.append(String.format(Locale.US, "cell %s %.1f in, setpoint %.0f, trim %+.0f<br>", mech.targetCell() == null ? "-" : mech.targetCell().label(),
                mech.distance(), mech.setpoint(), mech.speedTrim()));
        if (mech.flywheel != null) sb.append(String.format(Locale.US, "flywheel %.0f / %.0f %s %.1f A<br>", mech.flywheel.filtered(), mech.flywheel.target(),
                mech.flywheelText(), mech.flywheel.amps()));
        if (mech.pusher != null) sb.append(String.format(Locale.US, "pusher %s %.0f deg %.2f A lock %s check %s<br>", mech.pusher.state(), mech.pusher.angle(),
                mech.pusher.amps(), mech.lockState(), mech.check == null ? "-" : mech.check.result()));
        sb.append(String.format(Locale.US, "held %s, slots %s, tally %d, %s<br>", mech.heldText(), mech.slots == null ? "-" : mech.slots.slots(),
                mech.launchedTally(), mech.fullStepText() == null ? "" : mech.fullStepText()));
        sb.append(String.format(Locale.US, "lock %s err %.1f, aims %s<br>", lockOn ? "on" : "off", lockErrorDeg, aimsText()));
        for (java.util.Map.Entry<org.firstinspires.ftc.teamcode.subsystems.Warnings.Code, String> e : mech.warnings().all().entrySet()) {
            sb.append("<font color='#ffd000'>").append(e.getValue()).append("</font><br>");
        }
        String n = mech.notice();
        if (n != null) sb.append(n).append("<br>");
        telemetry.addLine(sb.toString());
    }

    /** The supports, route, goal and robot on the Dashboard's field view, at 5 per second while a plan is shown. */
    private void overlay(double t) {
        Plan plan = hiveDash.plan() != null ? hiveDash.plan() : parkDash.plan();
        if (!HiveDash.OVERLAY || plan == null || !plan.ok() || t - lastOverlay < 0.2) return;
        lastOverlay = t;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();
        Canvas c = packet.fieldOverlay();
        c.setStroke("#808080");
        for (RoutePlanner.Obstacle o : RoutePlanner.obstacles()) {
            double[] xs = new double[o.poly.length], ys = new double[o.poly.length];
            for (int i = 0; i < o.poly.length; i++) {
                xs[i] = o.poly[i][0];
                ys[i] = o.poly[i][1];
            }
            c.strokePolygon(xs, ys);
        }
        c.setStroke("#00a0ff");
        double[] xs = new double[plan.points.length], ys = new double[plan.points.length];
        for (int i = 0; i < plan.points.length; i++) {
            xs[i] = plan.points[i].x;
            ys[i] = plan.points[i].y;
        }
        c.strokePolyline(xs, ys);
        c.strokeCircle(plan.goal().x, plan.goal().y, 3);
        c.setStroke("#00c000");
        double[][] body = footprint.corners(pose.x, pose.y, pose.h);
        c.strokePolygon(new double[] {body[0][0], body[1][0], body[2][0], body[3][0]}, new double[] {body[0][1], body[1][1], body[2][1], body[3][1]});
        dashboard.sendTelemetryPacket(packet);
    }

    private String sampleRow() {
        LaunchGrid.Result g = mech.grid();
        return String.format(Locale.US, "%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s,%s,%.1f,%.1f,%.1f,%.1f,%s,%s,%s",
                pose.x, pose.y, pose.headingDeg(), stickUp, stickRight, drive.cmdForward(), drive.cmdLeft(), drive.cmdTurn(),
                driveMode, RunLog.fmt(lockErrorDeg, 1), mech.logRow(),
                drive.wheelCheck.lastAmps(0), drive.wheelCheck.lastAmps(1), drive.wheelCheck.lastAmps(2), drive.wheelCheck.lastAmps(3),
                g == null ? "" : g.zone, g == null ? "" : RunLog.fmt(g.speed, 0), g == null ? "" : RunLog.fmt(Math.toDegrees(g.residualRad), 1));
    }
}
