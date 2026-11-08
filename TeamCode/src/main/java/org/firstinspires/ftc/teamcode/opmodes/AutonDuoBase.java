package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.Actions;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * The shared skeleton of the two duo autos: the INIT loop (alliance with the D-pad, the pose check), the log set-up, the run and the hand-over of the final pose. The two subclasses give the
 * start pose and the action sequence ({@link AutonDuoFlowerSide}, {@link AutonDuoGardenSide}). See {@link DuoAutos} for the values and design/duo-autos.md for the plan.
 */
abstract class AutonDuoBase extends LinearOpMode {

    /** What a subclass needs to build its sequence. */
    static final class Context {
        MecanumDrive drive;
        Mechanisms mechanisms;
        DuoKit kit;
        ChassisProfile profile;
        boolean blue;
    }

    /** The name written in the log and the INIT text. */
    protected abstract String autoName();

    /** The blue-frame start pose for this chassis. */
    protected abstract Pose2d blueStart(ChassisProfile profile);

    /** One line of INIT text: where to put the robot. */
    protected abstract String placementText(boolean blue);

    /** The whole sequence for this alliance, ending with the park. */
    protected abstract Action buildAuto(Context c);

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.AUTO, autoName(), hardwareMap);
        try {
            runAuto();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runAuto() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        boolean blue = true, seededBlue = true;
        Pose2d start = DuoKit.forAlliance(blueStart(profile), blue);
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);
        PinpointLocalizer loc = (PinpointLocalizer) drive.localizer;
        Mechanisms mechanisms = Mechanisms.fromHardware(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right) blue = true;
            if (gamepad1.dpad_left) blue = false;
            if (blue != seededBlue) {
                seededBlue = blue;
                start = DuoKit.forAlliance(blueStart(profile), blue);
                drive.setPose(start);
            }
            drive.updatePoseEstimate();
            if (mechanisms.counter != null) mechanisms.counter.update();   // reads only; nothing moves in INIT
            Pose2d p = drive.getPose();
            boolean poseOk = loc.seedConfirmed
                    && Math.hypot(p.position.x - start.position.x, p.position.y - start.position.y) < 1.5
                    && Math.abs(Math.toDegrees(Rotation2d.normalize(p.heading.toDouble() - start.heading.toDouble()))) < 3.0;
            telemetry.addLine(autoName() + ". " + placementText(blue) + " Keep still.");
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Pinpoint status", loc.getDeviceStatus());
            telemetry.addData("Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f", p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.addData("Expected start pose", "%.2f, %.2f, %.1f", start.position.x, start.position.y, start.heading.toDegrees());
            telemetry.addData("Mechanisms present", mechanisms.describe());
            telemetry.addData("Preload", mechanisms.counter == null ? "no element counter"
                    : mechanisms.counter.isValid() ? mechanisms.counter.count() + " elements seen (expect 4)   " + mechanisms.counter.stackString()
                    : "counter NOT READING");
            telemetry.addLine(poseOk ? "Pose OK." : "WARNING: Pinpoint pose does not match the start pose. Do NOT start; restart the opmode.");
            telemetry.addLine("Gamepad 1: D-pad right = BLUE, D-pad left = RED, then press Start on the Driver Station.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();

        final boolean isBlue = blue;
        Context c = new Context();
        c.drive = drive;
        c.mechanisms = mechanisms;
        c.kit = new DuoKit(telemetry);
        c.profile = profile;
        c.blue = isBlue;
        c.kit.autoClock.reset();

        RunLog.event(String.format(Locale.US, "SETUP %s, profile %s, alliance %s, start pose (%.1f, %.1f, %.1f deg), %s", autoName(), profile.name, isBlue ? "BLUE" : "RED",
                drive.getPose().position.x, drive.getPose().position.y, drive.getPose().heading.toDegrees(), mechanisms.describe()));
        RunLog.setSampler("x,y,hdg_deg,cmd_fwd,cmd_left,cmd_turn," + Mechanisms.SAMPLE_COLUMNS + ",a_lf,a_lb,a_rb,a_rf", 100, () -> {
            Pose2d p = drive.getPose();
            PoseVelocity2d dc = drive.getLastCommand();   // the drive command in the sample rows (from 2026-11-07), as in the TeleOp: what the legs and the jostle sent
            return String.format(Locale.US, "%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%s", p.position.x, p.position.y, p.heading.toDegrees(), dc.linearVel.x, dc.linearVel.y, dc.angVel, mechanisms.sampleRow() + "," + drive.wheelAmpsRow());
        });

        mechanisms.setBlueAlliance(isBlue);
        mechanisms.setIntakeAlwaysOn(true);   // the intake runs IN for the whole auto (user, 2026-11-07), whatever the count says; only a reject or an outtake runs it OUT
        mechanisms.setJamRetries(Pusher.JAM_RETRIES_AUTO);   // one jam back-off per pusher move: the autos are tight on time
        mechanisms.setAutoStopAfterSalvo(false);   // the flywheel stays up between launches ...
        mechanisms.setSpinRequested(true);         // ... and spins from Start, so it is at speed on arrival at each launch spot

        Action auto = buildAuto(c);

        Actions.runBlocking(this, auto, () -> mechanisms.update(drive.getPose()));
        RunLog.event(isStopRequested() ? "AUTO stopped before it finished" : "AUTO actions finished");
        RunLog.event(String.format(Locale.US, "AUTO TIMING total %.2f s into the auto; steps (s): %s", c.kit.autoClock.seconds(), String.join("; ", c.kit.timings)));

        // hand the final pose (and the alliance) to the TeleOp, whether the auto finished or was stopped
        drive.updatePoseEstimate();
        PoseStorage.save(drive.getPose(), isBlue);

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d p = drive.getPose();
            PoseStorage.save(p, isBlue);
            mechanisms.update(p);
            RunLog.tick();
            telemetry.addLine("Auto complete.");
            telemetry.addData("Final Pinpoint pose (in, in, deg)", "%.2f, %.2f, %.1f", p.position.x, p.position.y, p.heading.toDegrees());
            telemetry.update();
        }
    }

    /** The wall-flush start x for this chassis, back against the audience (+1) or rear (-1) wall in the blue frame. */
    protected static double wallStartX(ChassisProfile profile, double sign) {
        return sign * (FieldGeometry.HALF_IN - profile.originFromBackIn);
    }
}
