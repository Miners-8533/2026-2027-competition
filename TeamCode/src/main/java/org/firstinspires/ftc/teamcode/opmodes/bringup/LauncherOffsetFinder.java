package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Launcher Offset Finder (specs/support-opmodes.md sections 2.3 and 3.2): measure the launcher's sideways offset, and
 * separate it from any error in where the hive's cells are.
 *
 * No target lock. Drive by hand to a wall, line the launcher up with a cell's centerline by eye, launch, and mark the
 * salvo as good, or as landed left or right. Repeat for all four cells, ideally at two distances each, reseeding the
 * pose after long drives. A PC tool works the offset out from the marks.
 *
 * Controls (gamepad 1). At INIT, D-pad right / left chooses the BLUE / RED default start. Sticks drive as in the
 * TeleOp; left bumper is precision. Right trigger: intake IN, even when full; left trigger: OUT. A launches.
 * Y marks a good salvo; D-pad left / right marks one that landed left / right. X held 1 s reseeds.
 * The speed is for the nearest of all four cells; the flywheel runs from Start; there is no speed trim.
 *
 * SPEC GAP: the wording of the launch and mark lines is not on the page (a PC tool reads them). Here:
 * "OFFSET LAUNCH n: ..." with the pose at the launch, and "OFFSET MARK n: GOOD | LEFT | RIGHT, launch n ...".
 * Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@TeleOp(name = "Launcher Offset Finder", group = "2 Bring-up")
public final class LauncherOffsetFinder extends TeamOpMode {
    private static final double PRECISION = 0.30, TRIGGER = 0.3, RESEED_HOLD_S = 1.0;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Launcher Offset Finder"; }

    private static Cell nearestOfAll(Pose p) {
        Cell best = null;
        for (Cell c : Cell.values()) {
            if (best == null || Aim.distance(p.x, p.y, c) < Aim.distance(p.x, p.y, best)) best = c;
        }
        return best;
    }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        Alliance alliance = Alliance.BLUE;
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, alliance.pose(Field.startRear(profile.originToBack)));
        Mechanisms.Settings s = new Mechanisms.Settings();
        s.flywheelStopsAfterSalvo = false;
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
            telemetry.addLine("Launcher Offset Finder, " + alliance + " default start (D-pad right/left). Back on the wall, keep still.");
            telemetry.addLine("Pose " + drive.pose() + (drive.localizer.seedConfirmed() ? "" : " SEED NOT CONFIRMED") + ", found: " + mech.found());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        Pose start = alliance.pose(Field.startRear(profile.originToBack));
        RunLog.eventf("SETUP profile %s, alliance %s, start pose default %s, driver forward %.0f deg; Launcher Offset Finder, launcher offset %.2f in, mechanisms: %s",
                profile.name, alliance, drive.pose(), Math.toDegrees(alliance.driverForwardRad()), Aim.LAUNCHER_OFFSET_IN, mech.found());
        mech.setSpin(true);     // the flywheel runs from Start
        Edge launch = new Edge(), good = new Edge(), markL = new Edge(), markR = new Edge();
        Hold reseed = new Hold();
        int launches = 0, marks = 0;
        Pose launchPose = null;
        Cell launchCell = null;
        String last = "";
        while (opModeIsActive()) {
            double t = Clock.now();
            drive.updatePose();
            Pose p = drive.pose();
            Cell cell = nearestOfAll(p);        // the speed is for the nearest of all four cells
            mech.setTargetCell(cell);
            driveByHand(alliance, gamepad1.left_bumper ? PRECISION : 1.0);
            mech.setAutoForceIn(gamepad1.right_trigger > TRIGGER);      // IN, even when full
            mech.setOuttake(gamepad1.left_trigger > TRIGGER);
            if (launch.pressed(gamepad1.a) && mech.requestLaunch("offset finder")) {
                launches++;
                launchPose = p;
                launchCell = cell;
                RunLog.eventf("OFFSET LAUNCH %d: pose %s, nearest cell %s %s at (%.1f, %.1f), distance %.1f in, bearing %.1f deg, setpoint %.0f",
                        launches, p, cell.alliance, cell.label(), cell.x(), cell.y(), Aim.distance(p.x, p.y, cell),
                        Math.toDegrees(Aim.bearing(p.x, p.y, cell)), mech.setpoint());
            }
            boolean g = good.pressed(gamepad1.y), l = markL.pressed(gamepad1.dpad_left), r = markR.pressed(gamepad1.dpad_right);
            if (g || l || r) {
                if (launchPose == null) {
                    last = "no launch to mark yet";
                } else {
                    marks++;
                    last = String.format(Locale.US, "OFFSET MARK %d: %s, launch %d from pose %s at cell %s %s (%.1f, %.1f), distance %.1f in", marks,
                            g ? "GOOD" : l ? "LEFT" : "RIGHT", launches, launchPose, launchCell.alliance, launchCell.label(), launchCell.x(), launchCell.y(),
                            Aim.distance(launchPose.x, launchPose.y, launchCell));
                    RunLog.event(last);
                }
            }
            if (reseed.held(gamepad1.x, t, RESEED_HOLD_S)) {
                drive.stop();
                drive.localizer.setPose(start);
                RunLog.eventf("PINPOINT reseeded by the driver to %s", start);
            }
            mech.update(p, true);
            RunLog.tick();
            telemetry.addLine(String.format(Locale.US, "pose %s", p));
            telemetry.addLine(String.format(Locale.US, "nearest cell %s %s, %.1f in, setpoint %.0f, %s", cell.alliance, cell.label(), mech.distance(), mech.setpoint(), mech.flywheelText()));
            telemetry.addLine("launches " + launches + ", marks " + marks + ", held " + mech.heldText());
            telemetry.addLine("A launch. Y good, D-pad left/right landed left/right. X 1 s reseeds.");
            double left2 = reseed.remaining(t, RESEED_HOLD_S);
            if (left2 >= 0) telemetry.addLine(String.format(Locale.US, "RESEED in %.1f s", left2));
            telemetry.addLine(last);
            telemetry.update();
        }
    }
}
