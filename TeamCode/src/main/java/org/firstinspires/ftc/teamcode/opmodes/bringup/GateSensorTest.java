package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.TransportGate;

import java.util.Locale;

/**
 * Can the REV Color Sensor V3 "CS4" see the (grey) gate arm? Move the gate by hand-control and watch what the sensor
 * reads, then let the test find the reading that best tells "arm in front" from "arm away" and show when it sees it.
 * The result is a candidate for a gate zero reference (an arc of gate angles where the arm is seen), see
 * design/mechanisms.md.
 *
 * It drives the gate motor DIRECTLY, not through TransportGate, so it never touches the TeleOp's saved HOLD zero.
 * The gate angle shown here is relative to a local mark (A), nothing else. The gate has no hard stops: jog slowly.
 *
 *   Left stick   jog the gate (up = forward), JOG_MAX_POWER at full deflection
 *   A            mark the gate angle as 0 here (local to this test)
 *   X            capture BACKGROUND: jog the arm AWAY from the sensor first, hold still while it averages
 *   Y            capture GATE: jog the arm IN FRONT of the sensor first, hold still while it averages
 *   B            sensor LED on / off
 *   D-pad up/down  sensor gain +/- 0.5 (affects the color readings; distance is unaffected)
 *   Back         clear the captures and the seen-arc
 *
 * Do X and Y once each. The screen then ranks the five readings (distance, red, green, blue, alpha) by how well they
 * separate the two captures (the gap divided by the noise), picks the best, and reports SEEN / not seen live. Jog the
 * gate slowly through a full turn and read the "last seen arc": the gate angles where the sensor sees the arm.
 * The readings are also written to the robot log (tag GateSensor, about 7 per second) so they can be read over USB.
 *
 * Needs a REV Color Sensor V3 named "CS4" and the motor "gate".
 */
@Config
@TeleOp(name = "Gate Sensor Test (CS4)", group = "Bring-up")
public class GateSensorTest extends LinearOpMode {

    public static double JOG_MAX_POWER = 0.25;
    public static double CAPTURE_SEC = 1.0;
    /** Below this separation (gap / noise) a reading is not trustworthy for detection. */
    public static double MIN_SEPARATION = 3.0;
    public static double GAIN = 2.0;

    private static final String SENSOR_NAME = "CS4";
    private static final String[] METRICS = {"distance cm", "red", "green", "blue", "alpha"};
    private static final int N = METRICS.length;
    private static final String LOG_TAG = "GateSensor";

    private static final class Stats {
        int n = 0;
        final double[] sum = new double[N];
        final double[] sumSq = new double[N];

        void add(double[] v) {
            n++;
            for (int i = 0; i < N; i++) {
                sum[i] += v[i];
                sumSq[i] += v[i] * v[i];
            }
        }

        double mean(int i) {
            return n == 0 ? Double.NaN : sum[i] / n;
        }

        double std(int i) {
            if (n < 2) return 0.0;
            double m = sum[i] / n;
            return Math.sqrt(Math.max(0.0, sumSq[i] / n - m * m));
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.detect(hardwareMap);
        DcMotorEx gate = hardwareMap.get(DcMotorEx.class, "gate");
        gate.setDirection(profile.gateDirection);
        gate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gate.setPower(0.0);

        RevColorSensorV3 sensor = hardwareMap.tryGet(RevColorSensorV3.class, SENSOR_NAME);
        if (sensor == null) {
            telemetry.addLine("No REV Color Sensor V3 named \"" + SENSOR_NAME + "\" in the hub configuration.");
            telemetry.addData("Configured devices", hardwareMap.getAllNames(HardwareDevice.class));
            telemetry.update();
            waitForStart();
            return;
        }
        double gain = GAIN;
        boolean led = true;
        sensor.setGain((float) gain);
        sensor.enableLed(led);

        int markTicks = gate.getCurrentPosition();
        Stats background = new Stats(), gateSeen = new Stats(), capturing = null;
        int captureTarget = 0;   // 1 = background, 2 = gate
        ElapsedTime captureTimer = new ElapsedTime();
        ElapsedTime logTimer = new ElapsedTime();

        int best = -1;
        double threshold = Double.NaN, band = 0.0;
        boolean gateHigher = true, seen = false;
        double runMin = Double.NaN, runMax = Double.NaN;
        String lastArc = "-";

        telemetry.addLine("Gate Sensor Test. Jog the gate slowly, then X (background) and Y (gate in front). Nothing moves until you push the stick.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double stick = -gamepad1.left_stick_y;
            gate.setPower(Math.abs(stick) > 0.05 ? stick * JOG_MAX_POWER : 0.0);

            int ticks = gate.getCurrentPosition();
            double angle = TransportGate.ENCODER_SIGN * (ticks - markTicks) * 360.0 / TransportGate.TICKS_PER_REV;

            if (gamepad1.aWasPressed()) markTicks = ticks;
            if (gamepad1.bWasPressed()) {
                led = !led;
                sensor.enableLed(led);
            }
            if (gamepad1.dpadUpWasPressed()) gain += 0.5;
            if (gamepad1.dpadDownWasPressed()) gain = Math.max(0.5, gain - 0.5);
            sensor.setGain((float) gain);
            if (gamepad1.backWasPressed()) {
                background = new Stats();
                gateSeen = new Stats();
                best = -1;
                seen = false;
                lastArc = "-";
                runMin = runMax = Double.NaN;
            }
            boolean captureBackground = gamepad1.xWasPressed();
            boolean captureGate = gamepad1.yWasPressed();
            if (captureTarget == 0 && (captureBackground || captureGate)) {
                captureTarget = captureBackground ? 1 : 2;
                capturing = captureTarget == 1 ? (background = new Stats()) : (gateSeen = new Stats());
                captureTimer.reset();
            }

            double[] v = new double[N];
            NormalizedRGBA c = sensor.getNormalizedColors();
            v[0] = sensor.getDistance(DistanceUnit.CM);
            v[1] = c.red;
            v[2] = c.green;
            v[3] = c.blue;
            v[4] = c.alpha;

            if (captureTarget != 0) {
                capturing.add(v);
                if (captureTimer.seconds() >= CAPTURE_SEC) {
                    captureTarget = 0;
                    capturing = null;
                    if (background.n > 0 && gateSeen.n > 0) {
                        double bestSep = -1;
                        for (int i = 0; i < N; i++) {
                            double sep = Math.abs(gateSeen.mean(i) - background.mean(i)) / (background.std(i) + gateSeen.std(i) + 1e-6);
                            if (sep > bestSep) {
                                bestSep = sep;
                                best = i;
                            }
                        }
                        threshold = (gateSeen.mean(best) + background.mean(best)) / 2.0;
                        band = 0.15 * Math.abs(gateSeen.mean(best) - background.mean(best));
                        gateHigher = gateSeen.mean(best) > background.mean(best);
                    }
                }
            }

            if (best >= 0) {
                // hysteresis: switch to SEEN past the midpoint plus a band, back to not-seen past the midpoint minus it
                double signed = gateHigher ? v[best] - threshold : threshold - v[best];
                boolean wasSeen = seen;
                if (!seen && signed > band) seen = true;
                if (seen && signed < -band) seen = false;
                if (seen) {
                    runMin = Double.isNaN(runMin) ? angle : Math.min(runMin, angle);
                    runMax = Double.isNaN(runMax) ? angle : Math.max(runMax, angle);
                }
                if (wasSeen && !seen && !Double.isNaN(runMin)) {
                    lastArc = String.format(Locale.US, "from %.1f to %.1f deg (width %.1f, center %.1f), relative to the mark",
                            runMin, runMax, runMax - runMin, (runMin + runMax) / 2.0);
                    runMin = runMax = Double.NaN;
                }
            }

            if (logTimer.milliseconds() > 150) {
                logTimer.reset();
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "angle=%.1f ticks=%d dist=%.2f r=%.4f g=%.4f b=%.4f a=%.4f seen=%b best=%s",
                        angle, ticks, v[0], v[1], v[2], v[3], v[4], seen, best >= 0 ? METRICS[best] : "-"));
            }

            telemetry.addData("Gate angle (deg, from your mark)", "%.1f   ticks %d   stick %.2f", angle, ticks, stick);
            telemetry.addData("Sensor (distance cm / r / g / b / alpha)", "%.2f / %.4f / %.4f / %.4f / %.4f", v[0], v[1], v[2], v[3], v[4]);
            telemetry.addData("LED / gain", "%s / %.1f", led ? "on" : "off", gain);
            telemetry.addData("Capture", captureTarget == 0 ? "idle"
                    : String.format(Locale.US, "%s: hold still %.1f s", captureTarget == 1 ? "BACKGROUND" : "GATE", CAPTURE_SEC - captureTimer.seconds()));
            telemetry.addData("Captures taken", "background %d samples, gate %d samples", background.n, gateSeen.n);
            if (background.n > 0 && gateSeen.n > 0) {
                for (int i = 0; i < N; i++) {
                    double sep = Math.abs(gateSeen.mean(i) - background.mean(i)) / (background.std(i) + gateSeen.std(i) + 1e-6);
                    telemetry.addData((i == best ? "BEST  " : "      ") + METRICS[i],
                            "background %.4f (+/-%.4f)  gate %.4f (+/-%.4f)  separation %.1f%s",
                            background.mean(i), background.std(i), gateSeen.mean(i), gateSeen.std(i), sep,
                            sep < MIN_SEPARATION ? "  (weak)" : "");
                }
                telemetry.addData("DETECTION", best < 0 ? "-" : String.format(Locale.US, "%s using %s (threshold %.4f, gate reads %s)",
                        seen ? "SEEN" : "not seen", METRICS[best], threshold, gateHigher ? "HIGHER" : "LOWER"));
                telemetry.addData("Last seen arc", lastArc);
            } else {
                telemetry.addLine("Jog the arm away from the sensor, press X; then jog it in front of the sensor, press Y.");
            }
            telemetry.addLine("Stick jog, A mark angle 0, X background, Y gate, B LED, D-pad gain, Back clear");
            telemetry.update();
        }
        gate.setPower(0.0);
    }
}
