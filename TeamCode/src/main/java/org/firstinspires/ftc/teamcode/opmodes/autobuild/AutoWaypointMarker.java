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
 * SPEC GAP: the controls are not in the specs. This rebuild's choice (gamepad 1): A = mark, B = a numbered check at a
 * spot whose true position is known, X = deploy the ramp, Y = stow it, left bumper + right trigger held 1 s = reseed
 * to the start. At INIT: D-pad right/left = BLUE/RED, D-pad up/down = rear-wall / audience-wall start.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@TeleOp(name = "Auto Waypoint Marker", group = "5 Auto build")
public final class AutoWaypointMarker extends TeamOpMode {
    @Override protected RunLog.Kind logKind() { return RunLog.Kind.WAYPOINTS; }
    @Override protected String title() { return "Auto Waypoint Marker"; }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        Alliance alliance = Alliance.BLUE;
        boolean rear = true;
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, Field.startRear(profile.originToBack));
        Ramp ramp = Ramp.create(hardwareMap, profile, true);
        Edge r = new Edge(), l = new Edge(), u = new Edge(), d = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            boolean changed = false;
            if (r.pressed(gamepad1.dpad_right)) { alliance = Alliance.BLUE; changed = true; }
            if (l.pressed(gamepad1.dpad_left)) { alliance = Alliance.RED; changed = true; }
            if (u.pressed(gamepad1.dpad_up)) { rear = true; changed = true; }
            if (d.pressed(gamepad1.dpad_down)) { rear = false; changed = true; }
            if (changed) drive.localizer.seed(start(alliance, rear));
            telemetry.addLine("Waypoint Marker: " + alliance + ", " + (rear ? "rear-wall" : "audience-wall") + " start");
            telemetry.addLine("Place the robot at " + start(alliance, rear) + " and keep it still.");
            telemetry.addLine("Pose " + drive.pose() + (drive.localizer.seedConfirmed() ? "" : " SEED NOT CONFIRMED"));
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;
        markStarted();
        RunLog.eventf("SETUP waypoint marker: %s, start %s", alliance, start(alliance, rear));
        Edge mark = new Edge(), check = new Edge(), deploy = new Edge(), stow = new Edge();
        Hold reseed = new Hold();
        int marks = 0, checks = 0;
        String last = "";
        while (opModeIsActive()) {
            double t = Clock.now();
            drive.updatePose();
            Pose p = drive.pose();
            driveByHand(alliance, gamepad1.left_trigger > 0.3 ? 0.3 : 0.7);
            // A pose is shown in the blue frame, which is how the autos are written.
            Pose blue = alliance == Alliance.BLUE ? p : p.rotated180();
            String rampState = ramp == null ? "no ramp" : ramp.target().toString();
            if (mark.pressed(gamepad1.a)) {
                marks++;
                last = String.format(Locale.US, "Pose.deg(%.1f, %.1f, %.0f)  // mark %d, ramp %s", blue.x, blue.y, blue.headingDeg(), marks, rampState);
                RunLog.eventf("WAYPOINT mark %d: field %s, blue frame %s", marks, p, last);
            }
            if (check.pressed(gamepad1.b)) {
                checks++;
                RunLog.eventf("WAYPOINT check %d: the Pinpoint reads %s", checks, p);
                last = "check " + checks + " at " + p;
            }
            if (ramp != null && deploy.pressed(gamepad1.x)) ramp.deploy();
            if (ramp != null && stow.pressed(gamepad1.y)) ramp.stow();
            if (reseed.held(gamepad1.left_bumper && gamepad1.right_trigger > 0.3, t, 1.0)) {
                drive.localizer.setPose(start(alliance, rear));
                RunLog.eventf("WAYPOINT reseed to %s", start(alliance, rear));
            }
            RunLog.tick();
            telemetry.addLine(String.format(Locale.US, "field %s", p));
            telemetry.addLine(String.format(Locale.US, "blue frame (%.1f, %.1f, %.0f deg), ramp %s", blue.x, blue.y, blue.headingDeg(), rampState));
            telemetry.addLine("A mark (" + marks + "), B check (" + checks + "), X deploy, Y stow");
            telemetry.addLine(last);
            telemetry.update();
        }
    }

    private Pose start(Alliance a, boolean rear) {
        return a.pose(rear ? Field.startRear(profile.originToBack) : Field.startAudience(profile.originToBack));
    }
}
