package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * Bring-up test for the flywheel: set a speed, watch it hold, and measure what a ball does to it.
 *
 *   A                  spin on / off
 *   D-pad up / down    setpoint up / down by STEP ticks/s
 *   Y                  arm a dip measurement, then feed a ball (by hand or with the gate)
 *   Options             clear the last measurement and any fault
 *
 * Both motors ("fly_motor_enc", plus "fly_follow" if configured) get the same power, with no hub encoder loop.
 *
 * Checks, in order:
 *   1. Spin it up at a low setpoint: the flywheel turns the shooting way and the speed reads POSITIVE.
 *      If it turns the wrong way, flip flywheelDirection (or flywheelFollowerDirection for the second
 *      motor) for this chassis in ChassisProfile. If the speed reads negative, the flywheel stops with a
 *      fault: set Flywheel.ENCODER_SIGN = -1.
 *   2. Raise the setpoint and watch the velocity settle near it, with Flywheel.FEED_FORWARD (start 0.2)
 *      holding it. Tune FEED_FORWARD live in the FTC Dashboard so the hold power is just enough.
 *   3. Press Y, then feed a ball. It records how far the speed dips and how long it takes to recover.
 *      Do a 4-ball burst a ball at a time. This is the data for deciding between one motor and two.
 *
 * Needs the encoder motor "fly_motor_enc". Keep clear of the wheel.
 */
@Config
@TeleOp(name = "Flywheel Test", group = "Bring-up")
public class FlywheelTest extends LinearOpMode {

    public static double STEP = 25.0;
    public static double DIP_START_FRACTION = 0.95;   // a dip starts when raw speed falls below this fraction of the setpoint
    public static double RECOVERED_FRACTION = 0.98;   // recovered when filtered speed is back above this fraction

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        boolean spinning = false;
        double setpoint = 400.0;

        boolean measuring = false, dipped = false;
        double minVelocity = 0;
        ElapsedTime sinceDip = new ElapsedTime();
        ElapsedTime spinTimer = new ElapsedTime();
        double spinUpSec = Double.NaN;
        String lastMeasurement = "-";

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("Flywheel Test ready. A = spin, D-pad = setpoint, Y = arm a dip measurement.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                spinning = !spinning;
                spinUpSec = Double.NaN;
                spinTimer.reset();
            }
            if (gamepad1.dpadUpWasPressed()) setpoint += STEP;
            if (gamepad1.dpadDownWasPressed()) setpoint = Math.max(0.0, setpoint - STEP);
            if (gamepad1.optionsWasPressed()) {
                lastMeasurement = "-";
                flywheel.clearFault();
            }
            if (gamepad1.yWasPressed()) {
                measuring = true;
                dipped = false;
                minVelocity = Double.MAX_VALUE;
            }

            flywheel.setTargetVelocity(spinning ? setpoint : 0.0);
            flywheel.update();

            double velocity = flywheel.getVelocity();
            double target = flywheel.getTargetVelocity();
            if (spinning && Double.isNaN(spinUpSec) && flywheel.isReady()) spinUpSec = spinTimer.seconds();
            if (measuring && target > 0) {
                minVelocity = Math.min(minVelocity, velocity);
                if (!dipped && velocity < DIP_START_FRACTION * target) {
                    dipped = true;
                    sinceDip.reset();
                }
                if (dipped && flywheel.getFilteredVelocity() >= RECOVERED_FRACTION * target) {
                    lastMeasurement = String.format("dipped to %.0f ticks/s (%.0f%% of setpoint), recovered in %.0f ms",
                            minVelocity, 100.0 * minVelocity / target, sinceDip.milliseconds());
                    measuring = false;
                }
            }

            telemetry.addData("Chassis profile", profile.label());
            telemetry.addData("Spinning / setpoint", "%s   %.0f ticks/s", spinning, setpoint);
            telemetry.addData("Spin-up to ready", Double.isNaN(spinUpSec) ? (spinning ? "spinning up..." : "-") : String.format("%.2f s", spinUpSec));
            flywheel.log(telemetry);
            telemetry.addData("Dip measurement", measuring ? (dipped ? "dipped, waiting to recover..." : "armed, feed a ball") : "idle");
            telemetry.addData("Last measurement", lastMeasurement);
            telemetry.addLine("A = spin, D-pad up/down = setpoint, Y = arm dip, Options = clear");
            telemetry.update();
        }
        flywheel.stop();
    }
}
