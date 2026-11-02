package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.PusherCheck;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Tests the pusher check ({@link PusherCheck}) on its own, with no coordinator: park the arm in front of CS4, find a slipped zero, send
 * the arm back to HOLD. The track must be EMPTY and the arm must be at HOLD when the opmode is initialized (that is where the zero is
 * assumed). The arm has no stops: keep clear.
 *
 *   A                 run the check now (the arm must be at rest, the track empty)
 *   B                 abort a running check (the arm goes forward to HOLD)
 *   X / Y             FAKE a belt slip of +1 / -1 tooth (18 deg): the pusher's zero is shifted, the arm does not move
 *   Right / left bumper  FAKE a slip of +5 / -5 teeth (90 deg): more than the check tries before it sweeps
 *   Left stick        jog the arm (up = forward) when no check is running
 *   Back              mark HOLD here (jog the arm to HOLD first), as in Shot Test
 *
 * Try, in order: A with the zero right (expect OK, the arm goes back 33 deg, looks, and returns to HOLD); X, then A (expect one
 * continuous sweep that finds the arm and corrects the zero by about 18 deg, then A again: OK); Y then A (the arm is then ahead of
 * the zero, so the sweep goes nearly a full turn); RB then A (a correction of about 90 deg; check afterwards that the arm returns
 * to the real HOLD by eye). Every result, with its reason, is on screen and in the robot
 * log (adb logcat -s PusherCheckTest) and runlogs/bringup.log.
 */
@Config
@TeleOp(name = "Pusher Check Test (CS4)", group = "Bring-up")
public class PusherCheckTest extends LinearOpMode {

    public static double JOG_MAX_POWER = 0.25;
    public static double SLIP_TEETH_SMALL = 1.0;
    public static double SLIP_TEETH_LARGE = 5.0;

    private static final String TAG = "PusherCheckTest";

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Pusher Check Test", hardwareMap);
        try {
            runTest();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runTest() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Pusher pusher = new Pusher(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);
        ElementCounter counter = ElementCounter.tryCreate(hardwareMap);
        if (counter == null) {
            telemetry.addLine("The four color sensors CS1-CS4 are not all configured: nothing to test.");
            telemetry.update();
            waitForStart();
            return;
        }
        PusherCheck check = new PusherCheck(pusher, counter);

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Pusher Check Test. The arm must be at HOLD now, and the track EMPTY. Press Start.");
        telemetry.update();
        waitForStart();
        RunLog.started();

        String lastShown = "";
        String lastEvent = "-";
        while (opModeIsActive()) {
            counter.update();

            double stick = -gamepad1.left_stick_y;
            boolean free = !check.isActive() && (pusher.getState() == Pusher.State.JOG || pusher.getState() == Pusher.State.HOLD);
            if (free && Math.abs(stick) > 0.05) pusher.jog(stick * JOG_MAX_POWER);
            else if (pusher.getState() == Pusher.State.JOG) pusher.stopInPlace();

            if (gamepad1.backWasPressed() && free) {
                pusher.markHoldHere();
                RunLog.event("HOLD marked here");
            }
            if (gamepad1.aWasPressed()) {
                if (pusher.getState() == Pusher.State.JOG) pusher.stopInPlace();
                String refusal = check.startRefusal();
                lastEvent = refusal == null ? "check started" : "check NOT started: " + refusal;
                if (refusal == null) check.start();
                RunLog.event(lastEvent);
            }
            if (gamepad1.bWasPressed()) {
                check.abort("aborted by the operator");
                lastEvent = "abort pressed";
            }
            if (!check.isActive()) {
                if (gamepad1.xWasPressed()) fakeSlip(pusher, SLIP_TEETH_SMALL);
                if (gamepad1.yWasPressed()) fakeSlip(pusher, -SLIP_TEETH_SMALL);
                if (gamepad1.rightBumperWasPressed()) fakeSlip(pusher, SLIP_TEETH_LARGE);
                if (gamepad1.leftBumperWasPressed()) fakeSlip(pusher, -SLIP_TEETH_LARGE);
            }

            check.update();
            pusher.update();

            if (check.takeFinished()) {
                String line = check.lastResult() + ": " + check.lastDetail();
                RobotLog.ii(TAG, line);
                lastShown = line;
            }

            telemetry.addData("Check", "%s   last: %s", check.statusCode(), lastShown.isEmpty() ? "-" : lastShown);
            telemetry.addData("Last button", lastEvent);
            telemetry.addData("Pusher", "%s   angle %.1f deg ahead of HOLD   error to nearest HOLD %+.1f deg",
                    pusher.getState(), pusher.getAngleDeg(), pusher.getHoldErrorDeg());
            telemetry.addData("CS4", "%s   reads %d   arm seen %s", counter.isValid() ? "reading" : "NOT READING",
                    counter.readCount(ElementCounter.ARM_SENSOR), counter.pusherArmSeen());
            telemetry.addData("Elements", "%d   %s", counter.count(), counter.stackString());
            pusher.log(telemetry);
            telemetry.addLine("A check, B abort, X/Y fake +/-1 tooth, RB/LB fake +/-5 teeth, stick jog, Back mark HOLD");
            telemetry.update();
        }
        pusher.stopInPlace();
        pusher.update();
    }

    /** Pretends the belt slipped: the pusher's zero moves by the given teeth, the arm does not. */
    private void fakeSlip(Pusher pusher, double teeth) {
        double deg = teeth * 360.0 / PusherCheck.PULLEY_TEETH;
        pusher.resyncAngle(pusher.getUnwrappedTicks(), pusher.getAngleDeg() + deg);
        Pusher.setZeroVerified(false);
        RunLog.event(String.format(Locale.US, "FAKE slip of %+.0f tooth/teeth (%+.0f deg): the zero was shifted, the arm did not move", teeth, deg));
    }
}
