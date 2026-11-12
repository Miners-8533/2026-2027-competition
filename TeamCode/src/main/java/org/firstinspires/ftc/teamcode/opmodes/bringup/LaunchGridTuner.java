package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
import org.firstinspires.ftc.teamcode.drive.HiveDash;
import org.firstinspires.ftc.teamcode.drive.HiveRoute;
import org.firstinspires.ftc.teamcode.drive.LaunchGrid;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.LaunchMap;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Launch Grid Tuner (2026-11-10, replaces the one-dimensional Launch Map Tuner): builds a TWO-dimensional grid of launch aim and speed. The robot drives to
 * each point of a grid of launching spots for ONE hive cell (the NEAR cell, the blue REAR cell on the start side, or the FAR cell, the blue AUDIENCE cell, picked at INIT; the
 * assumed alliance is BLUE for now, the usual start against the rear wall), already pointed at the cell with today's aim (TargetLock's aim heading with the launcher offset and the far-cell
 * extra) and the flywheel at today's launch map speed. At the spot the operator loads elements, fires, and trims the ANGLE and the SPEED until the salvo scores;
 * A moves to the next spot (the trims it was left with are taken as good), B says no shot is possible from here and moves on, X goes back one spot.
 *
 * The spots ({@link LaunchGrid}, 12 in pitch): only on the cell's OPEN side, at least TargetLock.MIN_DISTANCE_IN from the cell, within ANGLE_LIMIT_DEG of the
 * opening's normal, with the robot at its aim heading clear of the hive supports, the flower stands and the walls (the CAD keep-outs of HiveRoute). There is no
 * maximum distance: the field bounds it. They are visited in a serpentine sweep, the column nearest the start first.
 *
 * Driving: HOLD THE RIGHT TRIGGER to drive to the spot (letting go stops at once; hold it again to continue, the route is planned again from where the robot
 * is). The route is HiveRoute's (round the hive supports and the flower stands); when the router refuses a spot beside a flower stand it drives a short
 * straight hop when that is clear, and when it is not it says BLOCKED and you drive by hand. A trim of the angle turns the robot in place by itself (slowly);
 * touching the sticks cancels that. The sticks always drive the robot by hand when the trigger is not held.
 *
 * Each accepted spot is logged as one GRID POINT line (the speed, the angle trim, the heading flown, the distance, how many salvos were fired, the pose) and the
 * whole table is logged again when the opmode ends (GRID TABLE), ready to paste. A spot where no salvo was fired is marked unverified.
 *
 * Gamepad 1:
 *   Right trigger (HOLD)   drive to the spot          Left stick / right stick X   drive / turn by hand (robot oriented)    Right bumper (hold): precision (30%)
 *   Y                      fire a salvo (the flywheel is already spinning)
 *   D-pad up / down        speed +/- SPEED_STEP ticks/s            D-pad left / right   aim left / right by ANGLE_STEP_DEG (turns the robot)
 *   A                      this spot is done: take its trims as good and go to the next      B   no shot is possible from here: next      X   back one spot
 *   (the intake is ALWAYS IN, as in the TeleOp)                    Left trigger (hold)   outtake
 *   Back                   clear a pusher fault; HOLD it 1 s to reseed the pose to the start pose (square the robot on the wall first)
 *
 * INIT (gamepad 1): D-pad left = NEAR cell, D-pad right = FAR cell. Start pose: START_X_IN, START_Y_IN, START_HEADING_DEG (the usual blue start: against the rear
 * wall facing the audience wall, the pose the TeleOp starts blue from); keep still while the Pinpoint calibrates. START_INDEX starts the sweep at a later spot (to continue an earlier session).
 * The pusher is assumed to be where the last opmode left it (the saved zero).
 */
@Config
@TeleOp(name = "Launch Grid Tuner", group = "Bring-up")
public class LaunchGridTuner extends LinearOpMode {

    public static double START_X_IN = -63.4;
    public static double START_Y_IN = 16.5;
    public static double START_HEADING_DEG = 0.0;
    /** Grid spacing, inches (12: 30 spots for the near cell, 29 for the far). */
    public static double PITCH_IN = 12.0;
    /** A shot more than this many degrees off the opening's normal is not tried (a shot nearly parallel to the face cannot enter the cell). */
    public static double ANGLE_LIMIT_DEG = 75.0;
    /** Start the sweep at this spot (0 = the first). */
    public static int START_INDEX = 0;
    public static double ANGLE_STEP_DEG = 0.5;
    public static double SPEED_STEP = 20.0;
    public static double MAX_ANGLE_TRIM_DEG = 20.0;
    /** The power cap of the drive to a spot. */
    public static double HOP_POWER = 0.5;
    /** The power cap of the turn in place after an angle trim. */
    public static double REAIM_POWER = 0.3;
    /** Start a spot with the previous spot's trims instead of none. */
    public static boolean CARRY_TRIMS = false;
    /** Speed used when the saved launch map is empty. */
    public static double FALLBACK_SETPOINT = 1350.0;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double STICK_CANCEL = 0.15;
    /** Within this many inches of a spot the robot counts as at it (A, Y and the angle trim work), whether it drove there itself or was driven by hand. */
    public static double HERE_IN = 4.0;
    public static double RESEED_HOLD_SEC = 1.0;
    private static final double TURN_SCALE = 0.5;
    private static final double PRECISION_SCALE = 0.30;

    private enum State { DRIVING, AT_POINT }

    // what the log's sample rows show
    private double logX, logY, logHeadingDeg, logTargetHeadingDeg, logAngleTrim, logSpeedTrim, logSetpoint;
    private int logPoint;
    private String logState = "";

    private static HiveRoute.Params routeParams() {
        HiveRoute.Params p = new HiveRoute.Params();
        p.robotHeightIn = HiveDash.ROBOT_HEIGHT_IN;
        p.marginIn = HiveDash.MARGIN_IN;
        p.alignMinLegIn = HiveDash.ALIGN_MIN_LEG_IN;
        p.turnPenaltyInPerDeg = HiveDash.TURN_PENALTY_IN_PER_DEG;
        p.unalignedSpeedRatio = HiveDash.UNALIGNED_SPEED_RATIO;
        p.underHive = HiveDash.UNDER_HIVE_ROUTES;
        p.laneHalfWidthIn = HiveDash.LANE_HALF_WIDTH_IN;
        p.flowerClearanceIn = HiveDash.FLOWER_CLEARANCE_IN;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) p.points.add(new double[]{sx * HiveDash.HELPER_X, sy * HiveDash.HELPER_Y});
            p.points.add(new double[]{sx * HiveDash.GATE_X, 0.0});
        }
        return p;
    }

    private static Pose2d startPose() {
        return new Pose2d(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
    }

    /** The blue hive's cell: NEAR = the rear cell (the start is against the rear wall), FAR = the audience cell. */
    private static HiveTarget cellFor(boolean far) {
        return far ? HiveTarget.BLUE_AUDIENCE : HiveTarget.BLUE_REAR;
    }

    private static LaunchGrid.Aim aimFor(HiveTarget cell) {
        double extra = TargetLock.farCellExtraLeftIn(cell, true);   // blue alliance
        return (x, y) -> TargetLock.aimHeadingRad(x, y, cell, extra);
    }

    private static List<LaunchGrid.Point> buildGrid(HiveTarget cell, HiveRoute.Params p) {
        List<LaunchGrid.Point> pts = LaunchGrid.generate(cell, PITCH_IN, ANGLE_LIMIT_DEG, TargetLock.MIN_DISTANCE_IN, FieldGeometry.HALF_IN, aimFor(cell), p);
        LaunchGrid.order(pts, START_X_IN, START_Y_IN);
        return pts;
    }

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Launch Grid Tuner", hardwareMap);
        try {
            runTuner();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    // ---- per-spot results ----
    private List<LaunchGrid.Point> pts = new ArrayList<>();
    private char[] status = new char[0];          // ' ' not done, 'G' good, 'N' no shot
    private double[] angleTrim = new double[0];   // degrees, positive = aim left (counterclockwise)
    private double[] speedTrim = new double[0];   // ticks/s added to the launch map's speed
    private int[] salvos = new int[0];
    private double[] flownHeadingDeg = new double[0];
    private double[] donePoseX = new double[0], donePoseY = new double[0];
    private double[] doneSpeed = new double[0];

    private void resetResults() {
        int n = pts.size();
        status = new char[n];
        java.util.Arrays.fill(status, ' ');
        angleTrim = new double[n];
        speedTrim = new double[n];
        salvos = new int[n];
        flownHeadingDeg = new double[n];
        donePoseX = new double[n];
        donePoseY = new double[n];
        doneSpeed = new double[n];
    }

    private static double mapSpeed(double distanceIn) {
        double v = LaunchMap.velocityFor(distanceIn);
        return Double.isNaN(v) ? FALLBACK_SETPOINT : v;
    }

    private void runTuner() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        HiveRoute.Params params = routeParams();

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose());
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.CONTINUE);
        mechanisms.setUseLaunchMap(false);          // this opmode sets the speed itself
        mechanisms.setAutoStopAfterSalvo(false);    // the flywheel stays on between launches
        mechanisms.setBlueAlliance(true);           // the assumed alliance (2026-11-12): blue; its usual start is against the rear wall
        mechanisms.setIntakeAlwaysOn(true);         // IN whatever the count, as in the TeleOp (the 2026-11-12 run had it OFF: nothing turned it on)

        boolean far = false;
        HiveTarget cell = cellFor(far);
        pts = buildGrid(cell, params);
        resetResults();

        while (!isStarted() && !isStopRequested()) {
            boolean changed = false;
            if (gamepad1.dpad_left && far) {
                far = false;
                changed = true;
            }
            if (gamepad1.dpad_right && !far) {
                far = true;
                changed = true;
            }
            if (changed) {
                cell = cellFor(far);
                pts = buildGrid(cell, params);
                resetResults();
            }
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();   // reads only; nothing moves in INIT
            Pose2d p = drive.getPose();
            Pose2d want = startPose();
            boolean poseOk = Math.hypot(p.position.x - want.position.x, p.position.y - want.position.y) < 1.5;
            telemetry.addData("Cell (D-pad left = NEAR, right = FAR)", "%s: %s, %d spots (pitch %.0f in, angle limit %.0f deg)", far ? "FAR" : "NEAR", cell, pts.size(), PITCH_IN, ANGLE_LIMIT_DEG);
            telemetry.addData("Start pose to use", "X %.1f, Y %.2f in, heading %.0f deg", want.position.x, want.position.y, Math.toDegrees(want.heading.toDouble()));
            telemetry.addData("Pinpoint pose", "%.1f, %.1f, %.0f deg   %s", p.position.x, p.position.y, p.heading.toDegrees(), poseOk ? "OK" : "WARNING: not at the start pose");
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addData("Starts at spot", "%d of %d (START_INDEX)", Math.min(START_INDEX, Math.max(0, pts.size() - 1)) + 1, pts.size());
            telemetry.addLine("Place the robot at the start pose and keep still while the Pinpoint calibrates. Then Start.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        mechanisms.setOpenCell(cell);
        RunLog.started();
        RunLog.event(String.format(Locale.US, "SETUP launch grid for %s (%s), %d spots, pitch %.0f in, angle limit %.0f deg, min distance %.0f in, start pose (%.1f, %.1f, %.0f deg), %s",
                cell, far ? "FAR" : "NEAR", pts.size(), PITCH_IN, ANGLE_LIMIT_DEG, TargetLock.MIN_DISTANCE_IN, START_X_IN, START_Y_IN, START_HEADING_DEG, mechanisms.describe()));
        StringBuilder list = new StringBuilder("GRID SPOTS in visiting order:");
        for (int i = 0; i < pts.size(); i++) {
            LaunchGrid.Point q = pts.get(i);
            list.append(String.format(Locale.US, " %d(%.1f,%.1f,%.1fdeg,%.1fin%s)", i + 1, q.x, q.y, Math.toDegrees(q.headingRad), q.distanceIn, q.slid ? ",slid" : ""));
        }
        RunLog.event(list.toString());
        RunLog.setSampler("x,y,hdg_deg,target_hdg_deg,spot,state,angle_trim_deg,speed_trim,grid_setpoint," + Mechanisms.SAMPLE_COLUMNS, 200,
                () -> String.format(Locale.US, "%.1f,%.1f,%.1f,%.1f,%d,%s,%.1f,%.0f,%.0f,%s", logX, logY, logHeadingDeg, logTargetHeadingDeg, logPoint, logState, logAngleTrim,
                        logSpeedTrim, logSetpoint, mechanisms.sampleRow()));

        int idx = Math.max(0, Math.min(START_INDEX, pts.size() - 1));
        State state = State.DRIVING;
        boolean finished = pts.isEmpty();
        boolean spinning = true;
        List<Pose2d> legs = new ArrayList<>();
        List<Boolean> legIsVia = new ArrayList<>();
        int legIdx = 0;
        String routeNote = "no route yet";
        boolean blocked = false;
        DriveToPose current = null;
        DriveToPose reaim = null;
        boolean planned = false;
        String notice = "";
        long noticeUntilNs = 0L;
        boolean backDown = false, reseeded = false;
        ElapsedTime backHold = new ElapsedTime();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.getPose();
            TelemetryPacket packet = new TelemetryPacket();
            LaunchGrid.Point pt = pts.isEmpty() ? null : pts.get(idx);
            boolean deadMan = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > STICK_CANCEL || Math.abs(gamepad1.left_stick_y) > STICK_CANCEL || Math.abs(gamepad1.right_stick_x) > STICK_CANCEL;

            // at the spot: arrived by the auto drive, or by hand to within HERE_IN of it
            boolean here = pt != null && !finished && (state == State.AT_POINT || Math.hypot(pose.position.x - pt.x, pose.position.y - pt.y) < HERE_IN);

            // ---- controls that change the spot ----
            if (pt != null && !finished) {
                boolean aPressed = gamepad1.aWasPressed(), bPressed = gamepad1.bWasPressed();
                if (aPressed && !here) {
                    aPressed = false;
                    notice = "A ignored: the robot is not at spot " + (idx + 1) + " yet: HOLD the RIGHT TRIGGER to drive there (B = no shot from here, X = back)";
                    noticeUntilNs = System.nanoTime() + 6_000_000_000L;
                    RunLog.event("A ignored: the robot is not at spot " + (idx + 1) + " (B says no shot from here, X goes back)");
                }
                if (aPressed || bPressed) {
                    boolean noShot = bPressed && !aPressed;
                    status[idx] = noShot ? 'N' : 'G';
                    flownHeadingDeg[idx] = Math.toDegrees(pt.headingRad) + angleTrim[idx];
                    donePoseX[idx] = pose.position.x;
                    donePoseY[idx] = pose.position.y;
                    doneSpeed[idx] = mapSpeed(pt.distanceIn) + speedTrim[idx];
                    RunLog.event(resultLine(idx, pose));
                    int next = idx + 1;
                    if (next >= pts.size()) {
                        finished = true;
                        current = null;
                        reaim = null;
                        drive.stop();
                        RunLog.event("GRID COMPLETE: every spot visited");
                    } else {
                        if (CARRY_TRIMS && status[next] == ' ') {
                            angleTrim[next] = angleTrim[idx];
                            speedTrim[next] = speedTrim[idx];
                        }
                        idx = next;
                        state = State.DRIVING;
                        planned = false;
                        current = null;
                        reaim = null;
                        pt = pts.get(idx);
                    }
                } else if (gamepad1.xWasPressed() && idx > 0) {
                    idx--;
                    state = State.DRIVING;
                    planned = false;
                    current = null;
                    reaim = null;
                    pt = pts.get(idx);
                    RunLog.event("BACK to spot " + (idx + 1));
                }
            }
            if (pt != null) {
                // speed trim (always) and angle trim (turns the robot when it is at the spot)
                if (gamepad1.dpadUpWasPressed()) speedTrim[idx] += SPEED_STEP;
                if (gamepad1.dpadDownWasPressed()) speedTrim[idx] -= SPEED_STEP;
                boolean trimmed = false;
                if (gamepad1.dpadLeftWasPressed()) {
                    angleTrim[idx] = Math.min(MAX_ANGLE_TRIM_DEG, angleTrim[idx] + ANGLE_STEP_DEG);
                    trimmed = true;
                }
                if (gamepad1.dpadRightWasPressed()) {
                    angleTrim[idx] = Math.max(-MAX_ANGLE_TRIM_DEG, angleTrim[idx] - ANGLE_STEP_DEG);
                    trimmed = true;
                }
                if (trimmed && here && !deadMan) {
                    double target = pt.headingRad + Math.toRadians(angleTrim[idx]);
                    reaim = new DriveToPose(drive, new Pose2d(pose.position.x, pose.position.y, target)).withTolerance(1.0, 0.7).withMaxOutput(REAIM_POWER).withTimeout(2.5);
                    RunLog.event(String.format(Locale.US, "TRIM spot %d: angle %+.1f deg, speed %+.0f ticks/s (aim heading %.1f deg)", idx + 1, angleTrim[idx], speedTrim[idx], Math.toDegrees(target)));
                }
            }

            // ---- the mechanisms ----
            mechanisms.setManualOuttake(gamepad1.left_trigger > TRIGGER_THRESHOLD);
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
            if (pt != null && gamepad1.yWasPressed() && here) {
                salvos[idx]++;
                RunLog.event(String.format(Locale.US, "FIRE spot %d salvo %d: pose (%.1f, %.1f, %.1f deg), aim heading %.1f deg (trim %+.1f), %.1f in from %s, setpoint %.0f (map %.0f, trim %+.0f)",
                        idx + 1, salvos[idx], pose.position.x, pose.position.y, pose.heading.toDegrees(), Math.toDegrees(pt.headingRad) + angleTrim[idx], angleTrim[idx],
                        cell.distanceFrom(pose), cell, mapSpeed(pt.distanceIn) + speedTrim[idx], mapSpeed(pt.distanceIn), speedTrim[idx]));
                mechanisms.requestLaunch();
            }

            // ---- the drive ----
            String driveMode = "idle";
            boolean auto = deadMan && !finished && pt != null;
            if (auto) {
                reaim = null;
                double targetH = pt.headingRad + Math.toRadians(angleTrim[idx]);
                double headingErrDeg = Math.abs(Math.toDegrees(Rotation2d.normalize(targetH - pose.heading.toDouble())));
                boolean atSpot = Math.hypot(pose.position.x - pt.x, pose.position.y - pt.y) < 1.5 && headingErrDeg < 3.0;
                if (state == State.AT_POINT && atSpot && !planned) {
                    drive.stop();
                    driveMode = "at the spot";
                } else {
                    if (!planned) {
                        // plan from where the robot is
                        legs.clear();
                        legIsVia.clear();
                        legIdx = 0;
                        blocked = false;
                        current = null;
                        double h0 = pose.heading.toDouble();
                        if (Math.hypot(pose.position.x - pt.x, pose.position.y - pt.y) < 1.0) {
                            legs.add(new Pose2d(pt.x, pt.y, targetH));
                            legIsVia.add(false);
                            routeNote = "at the spot: settling the heading";
                        } else {
                            HiveRoute.Plan plan = HiveRoute.plan(pose.position.x, pose.position.y, h0, pt.x, pt.y, params);
                            if (plan.ok) {
                                double[][] w = plan.waypoints;
                                for (int k = 1; k < w.length; k++) {
                                    boolean last = k == w.length - 1;
                                    double h = last ? targetH : Math.atan2(w[k + 1][1] - w[k][1], w[k + 1][0] - w[k][0]);
                                    legs.add(new Pose2d(w[k][0], w[k][1], h));
                                    legIsVia.add(!last);
                                }
                                routeNote = "route: " + plan.description;
                            } else if (LaunchGrid.straightHopClear(pose.position.x, pose.position.y, h0, pt.x, pt.y, targetH, FieldGeometry.HALF_IN, params)) {
                                legs.add(new Pose2d(pt.x, pt.y, targetH));
                                legIsVia.add(false);
                                routeNote = "DIRECT hop (the router refused: " + plan.refusal + ")";
                            } else {
                                blocked = true;
                                routeNote = "BLOCKED: " + plan.refusal + " and a straight hop is not clear: drive by hand";
                            }
                        }
                        planned = true;
                        state = State.DRIVING;
                        RunLog.event(String.format(Locale.US, "DRIVE to spot %d (%.1f, %.1f) from (%.1f, %.1f, %.0f deg): %s", idx + 1, pt.x, pt.y, pose.position.x, pose.position.y,
                                pose.heading.toDegrees(), routeNote));
                    }
                    if (blocked) {
                        drive.stop();
                        driveMode = "blocked";
                    } else if (legIdx < legs.size()) {
                        if (current == null) {
                            current = new DriveToPose(drive, legs.get(legIdx)).withMaxOutput(HOP_POWER).withDistanceTimeout(1.0, 22.0, 1.5, 0.5, 8.0);
                            if (legIsVia.get(legIdx)) current.withTolerance(3.0, 12.0).withSettle(0.0);
                        }
                        if (!current.run(packet)) {
                            current = null;
                            legIdx++;
                        }
                        driveMode = "auto leg " + Math.min(legIdx + 1, legs.size()) + "/" + legs.size();
                    }
                    if (!blocked && legIdx >= legs.size()) {
                        state = State.AT_POINT;
                        drive.stop();
                        RunLog.event(String.format(Locale.US, "ARRIVED at spot %d: pose (%.1f, %.1f, %.1f deg) for (%.1f, %.1f, %.1f deg)", idx + 1, pose.position.x, pose.position.y,
                                pose.heading.toDegrees(), pt.x, pt.y, Math.toDegrees(pt.headingRad) + angleTrim[idx]));
                        legs.clear();
                        legIsVia.clear();
                        legIdx = 0;
                        planned = false;
                    }
                }
            } else {
                // the trigger is up (or there is nothing to drive to): an auto leg stops and is planned again from wherever the robot is on the next press
                if (planned) RunLog.event("DRIVE paused: the trigger was released");
                current = null;
                legs.clear();
                legIsVia.clear();
                legIdx = 0;
                planned = false;
                blocked = false;
                if (reaim != null && sticksMoved) reaim = null;
                if (reaim != null) {
                    if (!reaim.run(packet)) reaim = null;
                    driveMode = "turning to the trimmed aim";
                } else {
                    double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale),
                            -gamepad1.right_stick_x * TURN_SCALE * scale));
                    driveMode = sticksMoved ? "by hand" : "idle";
                }
            }

            // ---- the speed and the mechanisms update ----
            double setpoint = 0.0;
            double targetHeadingDeg = Double.NaN;
            if (pt != null) {
                setpoint = Math.max(0.0, mapSpeed(pt.distanceIn) + speedTrim[idx]);
                targetHeadingDeg = Math.toDegrees(pt.headingRad) + angleTrim[idx];
                mechanisms.setManualSetpoint(setpoint);
            }
            mechanisms.setSpinRequested(spinning);
            mechanisms.update(pose);
            RunLog.tick();

            logX = pose.position.x;
            logY = pose.position.y;
            logHeadingDeg = pose.heading.toDegrees();
            logTargetHeadingDeg = targetHeadingDeg;
            logPoint = pt == null ? 0 : idx + 1;
            logState = finished ? "finished" : state.name();
            logAngleTrim = pt == null ? 0.0 : angleTrim[idx];
            logSpeedTrim = pt == null ? 0.0 : speedTrim[idx];
            logSetpoint = setpoint;

            // ---- the screen ----
            if (pt != null && !finished && state != State.AT_POINT && !here) telemetry.addLine("*** HOLD THE RIGHT TRIGGER to drive to spot " + (idx + 1) + " (nothing moves by itself) ***");
            if (System.nanoTime() < noticeUntilNs) telemetry.addLine("*** " + notice + " ***");
            if (pt != null) {
                telemetry.addData("Spot", "%d of %d at (%.1f, %.1f)%s   %.1f in from %s   %s", idx + 1, pts.size(), pt.x, pt.y, pt.slid ? " (slid off a wall)" : "", pt.distanceIn, cell,
                        finished ? "GRID COMPLETE" : state == State.AT_POINT ? "AT THE SPOT" : "hold RIGHT TRIGGER to drive here");
                telemetry.addData("Route", routeNote);
                telemetry.addData("Aim", "heading %.1f deg = geometric aim %.1f %+.1f trim; robot is at %.1f (error %.1f)", targetHeadingDeg, Math.toDegrees(pt.headingRad), angleTrim[idx],
                        pose.heading.toDegrees(), Math.toDegrees(Rotation2d.normalize(Math.toRadians(targetHeadingDeg) - pose.heading.toDouble())));
                telemetry.addData("Speed", "%.0f ticks/s = map %.0f %+.0f trim   (fly_vel %.0f)", setpoint, mapSpeed(pt.distanceIn), speedTrim[idx], mechanisms.flywheel == null ? 0.0 : mechanisms.flywheel.getVelocity());
                telemetry.addData("Salvos here", "%d%s   status %s", salvos[idx], salvos[idx] == 0 ? " (none yet)" : "", status[idx] == 'G' ? "GOOD" : status[idx] == 'N' ? "NO SHOT" : "not done");
            }
            StringBuilder row = new StringBuilder();
            for (int i = 0; i < pts.size(); i++) {
                char c = status[i] == ' ' ? '.' : status[i];
                row.append(i == idx ? "[" + c + "]" : String.valueOf(c));
            }
            telemetry.addData("Progress (G good, N no shot)", row.toString());
            telemetry.addData("Drive", driveMode);
            telemetry.addData("Pose x / y (in), heading (deg)", "%.1f / %.1f / %.1f", pose.position.x, pose.position.y, pose.heading.toDegrees());
            mechanisms.log(telemetry);
            telemetry.addLine("RT hold drive, Y fire, A next, B no shot, X back, D-pad up/down speed, left/right aim, LT outtake, Back clear fault (hold = reseed pose)");
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
        RunLog.event(tableText());
        drive.stop();
        mechanisms.setIntakeAlwaysOn(false);
        mechanisms.setIntakeRequested(false);
        mechanisms.setManualOuttake(false);
        mechanisms.stopFlywheel();
        mechanisms.update(drive.getPose());
    }

    private String resultLine(int i, Pose2d pose) {
        LaunchGrid.Point q = pts.get(i);
        return String.format(Locale.US, "GRID POINT %d of %d: spot (%.1f, %.1f)%s, %.1f in, %s%s", i + 1, pts.size(), q.x, q.y, q.slid ? " slid" : "", q.distanceIn,
                status[i] == 'N' ? "NO SHOT possible" : String.format(Locale.US, "GOOD: speed %.0f ticks/s (map %.0f %+.0f trim), aim heading %.1f deg (geometric %.1f %+.1f trim)",
                        mapSpeed(q.distanceIn) + speedTrim[i], mapSpeed(q.distanceIn), speedTrim[i], Math.toDegrees(q.headingRad) + angleTrim[i], Math.toDegrees(q.headingRad), angleTrim[i]),
                String.format(Locale.US, ", %d salvo(s) fired%s, pose (%.1f, %.1f, %.1f deg)", salvos[i], salvos[i] == 0 && status[i] == 'G' ? " (UNVERIFIED)" : "", pose.position.x, pose.position.y, pose.heading.toDegrees()));
    }

    /** The whole table, one spot after another, for pasting: {x, y, distance, speed, angle trim, status}. */
    private String tableText() {
        StringBuilder sb = new StringBuilder("GRID TABLE (x, y, distance in, speed ticks/s, angle trim deg, status G good / N no shot / - not done, salvos fired):");
        for (int i = 0; i < pts.size(); i++) {
            LaunchGrid.Point q = pts.get(i);
            char s = status[i] == ' ' ? '-' : status[i];
            sb.append(String.format(Locale.US, " {%.1f, %.1f, %.1f, %.0f, %.1f, '%c', %d},", q.x, q.y, q.distanceIn, status[i] == 'G' ? doneSpeed[i] : Double.NaN, angleTrim[i], s, salvos[i]));
        }
        return sb.toString();
    }
}
