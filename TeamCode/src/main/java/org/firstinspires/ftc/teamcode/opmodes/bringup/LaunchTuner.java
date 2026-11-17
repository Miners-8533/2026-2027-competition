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
 * scores, then record the point. Recording resets both trims; undo removes the last point and restores the trims.
 *
 * SPEC GAP: the specs give the procedure, not the controls. This rebuild's choice (gamepad 1): right bumper = lock,
 * A = fire, D-pad up/down = speed trim, D-pad left/right = aim trim, X = record, Y = undo, B = switch the speed base
 * between the saved map and the points recorded this session. D-pad right/left at INIT chooses BLUE/RED.
 */
@TeleOp(name = "Launch Tuner", group = "2 Bring-up")
public final class LaunchTuner extends TeamOpMode {
    private static final double SPEED_STEP = 10, AIM_STEP_DEG = 1.0;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Launch Tuner"; }

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
        mech.setSpin(true);
        TargetLock lock = new TargetLock();
        Edge fire = new Edge(), up = new Edge(), down = new Edge(), aimL = new Edge(), aimR = new Edge(), rec = new Edge(), undo = new Edge(), base = new Edge();
        List<double[]> points = new ArrayList<>();      // {distance, speed, speed trim, aim trim}
        double speedTrim = 0, aimTrim = 0, lastT = Clock.now(), launchSpeed = 0;
        boolean sessionBase = false, wasSalvo = false;
        int salvos = 0;
        String lastSalvo = "";
        while (opModeIsActive()) {
            double t = Clock.now(), dt = Math.max(1e-3, t - lastT);
            lastT = t;
            drive.updatePose();
            Pose p = drive.pose();
            Cell cell = mech.targetCell() != null ? mech.targetCell() : Cell.of(alliance, true);
            if (up.pressed(gamepad1.dpad_up)) speedTrim += SPEED_STEP;
            if (down.pressed(gamepad1.dpad_down)) speedTrim -= SPEED_STEP;
            if (aimL.pressed(gamepad1.dpad_left)) aimTrim += AIM_STEP_DEG;
            if (aimR.pressed(gamepad1.dpad_right)) aimTrim -= AIM_STEP_DEG;
            mech.setSpeedTrim(speedTrim);
            boolean locked = gamepad1.right_bumper;
            double fwdScale = gamepad1.left_trigger > 0.3 ? 0.3 : 1.0;
            driveByHand(alliance, fwdScale);
            if (locked) {
                double turn = lock.update(p, drive.localizer.omega(), cell, aimTrim, 0, dt, Double.NaN);
                drive.drive(drive.cmdForward(), drive.cmdLeft(), turn);
            } else {
                lock.release();
            }
            if (fire.pressed(gamepad1.a)) mech.requestLaunch("tuner");
            if (rec.pressed(gamepad1.x)) {
                points.add(new double[] {mech.distance(), mech.setpoint(), speedTrim, aimTrim});
                RunLog.eventf("TUNER recorded point %d: distance %.1f in, speed %.0f (trim %+.0f), aim trim %+.1f deg, pose %s, cell %s, battery %.2f V",
                        points.size(), mech.distance(), mech.setpoint(), speedTrim, aimTrim, p, cell.label(), battery());
                RunLog.eventf("TUNER table line: {%.1f, %.0f},", mech.distance(), mech.setpoint());
                speedTrim = 0;      // each point keeps its own trims
                aimTrim = 0;
            }
            if (undo.pressed(gamepad1.y) && !points.isEmpty()) {
                double[] last = points.remove(points.size() - 1);
                speedTrim = last[2];
                aimTrim = last[3];
                RunLog.event("TUNER undo: the last point is removed and its trims restored");
            }
            if (base.pressed(gamepad1.b)) {
                sessionBase = !sessionBase && !points.isEmpty();
                if (sessionBase) {
                    List<double[]> sorted = new ArrayList<>(points);
                    Collections.sort(sorted, new Comparator<double[]>() {
                        @Override public int compare(double[] o1, double[] o2) { return Double.compare(o1[0], o2[0]); }
                    });
                    double[][] table = new double[sorted.size()][];
                    for (int i = 0; i < table.length; i++) table[i] = new double[] {sorted.get(i)[0], sorted.get(i)[1]};
                    LaunchMap.use(table);
                } else {
                    LaunchMap.use(null);
                }
                RunLog.event("TUNER speed base: " + (sessionBase ? "the points recorded this session" : "the saved map"));
            }
            mech.update(p, true);
            // One log line per salvo.
            if (mech.salvoRunning() && !wasSalvo) launchSpeed = mech.flywheel == null ? 0 : mech.flywheel.filtered();
            if (!mech.salvoRunning() && wasSalvo) {
                salvos++;
                lastSalvo = String.format(Locale.US, "launch speed %.0f, distance %.1f in, pose %s, lock error %.1f deg, trims %+.0f / %+.1f deg, battery %.2f V, flywheel %.1f A",
                        launchSpeed, mech.distance(), p, lock.errorDeg(), speedTrim, aimTrim, battery(), mech.flywheel == null ? 0 : mech.flywheel.amps());
                RunLog.event("TUNER salvo " + salvos + ": " + lastSalvo);
            }
            wasSalvo = mech.salvoRunning();
            RunLog.tick();
            telemetry.addLine(String.format(Locale.US, "%s %s %.1f in, setpoint %.0f, %s", alliance, cell.label(), mech.distance(), mech.setpoint(), mech.flywheelText()));
            telemetry.addLine(String.format(Locale.US, "speed trim %+.0f, aim trim %+.1f deg, lock %s (%.1f deg)", speedTrim, aimTrim, locked ? "on" : "off", lock.errorDeg()));
            telemetry.addLine("points " + points.size() + ", base: " + (sessionBase ? "session" : "saved map") + ", held " + mech.heldText());
            telemetry.addLine("last salvo: " + lastSalvo);
            telemetry.update();
        }
        LaunchMap.use(null);
        StringBuilder table = new StringBuilder("TUNER table:");
        for (double[] pt : points) table.append(String.format(Locale.US, " {%.1f, %.0f},", pt[0], pt[1]));
        RunLog.event(table.toString());
    }
}
