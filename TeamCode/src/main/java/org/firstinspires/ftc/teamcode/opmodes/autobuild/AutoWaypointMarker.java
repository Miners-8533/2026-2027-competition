package org.firstinspires.ftc.teamcode.opmodes.autobuild;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Auto Waypoint Marker (specs/support-opmodes.md section 5): collect poses for writing autos. Choose a start at INIT,
 * drive by hand, and mark the current pose. Each mark is logged and shown as a ready-to-paste pose with the ramp state.
 * There is no undo.
 *
 * Controls (gamepad 1). At INIT, D-pad right / left chooses the blue start (rear wall, y +16) / the red start
 * (audience wall, y -16); nothing drives until one is chosen. Right trigger is precision. A marks; B logs a numbered
 * check at a spot whose true position is known; X / Y deploy / stow the ramp; left bumper + right bumper held 1 s
 * reseeds to the start.
 *
 * The pose to paste is in the blue frame, which is how the autos are written; x, y and heading are the field pose.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@TeleOp(name = "Auto Waypoint Marker", group = "5 Auto build")
public final class AutoWaypointMarker extends TeamOpMode {
    private static final double PRECISION = 0.30, RESEED_HOLD_S = 1.0;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.WAYPOINTS; }
    @Override protected String title() { return "Auto Waypoint Marker"; }

    /** Blue: the rear wall, y +16. Red: the same start rotated, which is the audience wall, y -16. */
    private Pose start(Alliance a) { return a.pose(Field.startRear(profile.originToBack)); }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        Alliance alliance = null;       // nothing drives until a start is chosen
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, start(Alliance.BLUE));
        Ramp ramp = Ramp.create(hardwareMap, profile, true);
        Edge r = new Edge(), l = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            boolean right = r.pressed(gamepad1.dpad_right), left = l.pressed(gamepad1.dpad_left);
            if (right || left) {
                alliance = right ? Alliance.BLUE : Alliance.RED;
                drive.localizer.seed(start(alliance));
            }
            telemetry.addLine("Waypoint Marker. D-pad right: BLUE start (rear wall). D-pad left: RED start (audience wall).");
            telemetry.addLine(alliance == null ? "NO START CHOSEN: nothing drives until one is"
                    : alliance + ": place the robot at " + start(alliance) + ", back on the wall, and keep it still.");
            telemetry.addLine("Pose " + drive.pose() + (drive.localizer.seedConfirmed() ? "" : " SEED NOT CONFIRMED"));
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        if (alliance != null) RunLog.eventf("SETUP waypoint marker: alliance %s, start pose %s", alliance, start(alliance));
        Edge mark = new Edge(), check = new Edge(), deploy = new Edge(), stow = new Edge();
        Hold reseed = new Hold();
        int marks = 0, checks = 0;
        String last = "";
        while (opModeIsActive()) {
            double t = Clock.now();
            drive.updatePose();
            Pose p = drive.pose();
            if (alliance == null) {
                // The start can still be chosen after Start, with the robot on its spot and still.
                boolean right = r.pressed(gamepad1.dpad_right), left = l.pressed(gamepad1.dpad_left);
                if (right || left) {
                    alliance = right ? Alliance.BLUE : Alliance.RED;
                    drive.localizer.seed(start(alliance));
                    RunLog.eventf("SETUP waypoint marker: alliance %s, start pose %s", alliance, start(alliance));
                }
                drive.stop();
                RunLog.tick();
                telemetry.addLine("NO START CHOSEN: D-pad right = BLUE (rear wall), left = RED (audience wall). Nothing drives until then.");
                telemetry.update();
                continue;
            }
            driveByHand(alliance, gamepad1.right_trigger > 0.3 ? PRECISION : 1.0);
            Pose blue = alliance == Alliance.BLUE ? p : p.rotated180();
            String rampState = ramp == null ? "none" : ramp.target().toString();
            boolean m = mark.pressed(gamepad1.a), c = check.pressed(gamepad1.b);
            if (m || c) {
                int n = m ? ++marks : ++checks;
                String paste = String.format(Locale.US, "Pose.deg(%.1f, %.1f, %.0f),  // %s %d, ramp %s", blue.x, blue.y, blue.headingDeg(), m ? "mark" : "check", n, rampState);
                last = String.format(Locale.US, "%s %d: x %.1f, y %.1f, heading %.1f deg, ramp %s | %s", m ? "MARK" : "CHECK", n, p.x, p.y, p.headingDeg(), rampState, paste);
                RunLog.event(last);
            }
            if (ramp != null && deploy.pressed(gamepad1.x)) {
                ramp.deploy();
                RunLog.event("COMMAND ramp deploy");
            }
            if (ramp != null && stow.pressed(gamepad1.y)) {
                ramp.stow();
                RunLog.event("COMMAND ramp stow now");
            }
            boolean chord = gamepad1.left_bumper && gamepad1.right_bumper;
            if (reseed.held(chord, t, RESEED_HOLD_S)) {
                drive.stop();
                drive.localizer.setPose(start(alliance));
                RunLog.eventf("PINPOINT reseeded by the driver to %s", start(alliance));
            }
            RunLog.tick();
            telemetry.addLine(String.format(Locale.US, "%s field %s", alliance, p));
            telemetry.addLine(String.format(Locale.US, "blue frame (%.1f, %.1f, %.0f deg), ramp %s", blue.x, blue.y, blue.headingDeg(), rampState));
            telemetry.addLine("A mark (" + marks + "), B check (" + checks + "), X deploy, Y stow, RT precision");
            double left = reseed.remaining(t, RESEED_HOLD_S);
            telemetry.addLine(left >= 0 ? String.format(Locale.US, "RESEED in %.1f s", left) : "LB + RB held 1 s: reseed to the start");
            telemetry.addLine(last);
            telemetry.update();
        }
    }
}
