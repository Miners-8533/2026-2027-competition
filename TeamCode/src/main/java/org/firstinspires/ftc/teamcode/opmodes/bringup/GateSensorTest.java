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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Can the REV Color Sensor V3 "CS4" see the (grey) gate arm, and how repeatably? The eventual use is to confirm,
 * during a salvo, that the gate has not skipped belt: the arm passes the sensor once per turn, so the encoder angle
 * at which the sensor sees it should be the same every turn. If the belt skips, the encoder and the arm disagree and
 * that angle shifts. So the number that matters is how much that crossing angle varies from turn to turn, at the speed
 * a salvo runs.
 *
 * It drives the gate motor DIRECTLY, not through TransportGate, so it never touches the TeleOp's saved HOLD zero. The
 * gate angle shown is relative to a local mark (A) and only matters within this test. The gate has no hard stops.
 *
 *   Left stick       jog the gate (up = forward), JOG_MAX_POWER at full deflection
 *   A                mark the gate angle as 0 here (local to this test)
 *   X                capture BACKGROUND: jog the arm AWAY from the sensor first, hold still while it averages
 *   Y                capture GATE: jog the arm IN FRONT of the sensor first, hold still while it averages
 *   B                sensor LED on / off
 *   D-pad up/down    sensor gain +/- 0.5 (affects the color readings; distance is unaffected)
 *   D-pad left/right condition label: empty / pollen / nectar / pollen+nectar (for comparing runs; goes in the log)
 *   Left bumper      next sweep power (0.15, 0.2, 0.3, 0.45, 0.6: include 0.3, the salvo's SWEEP_POWER)
 *   Right bumper     start / stop the automatic SWEEP (needs X and Y done first)
 *   Back             clear the captures and the sweep results
 *
 * SWEEP: the gate turns forward at the chosen power for SWEEP_PASSES turns. Each time the best reading crosses the
 * midpoint between the two captures it records the encoder angle, interpolated between the two samples around the
 * crossing. It reports, for the rising edge, the falling edge and the arc's center: the spread of the angle from turn
 * to turn (std and min..max), plus the arc width, the gate speed, and the angle the gate moves between two sensor
 * readings (the resolution). A stick push, a timeout, or 1.5 turns without a crossing ends the sweep.
 *
 * Everything is also written to the robot log (tag GateSensor) so it can be read over USB. Run it empty, then with
 * yellow pollen in the robot, and compare the summaries: the label is in every line.
 *
 * Needs a REV Color Sensor V3 named "CS4" and the motor "gate".
 */
@Config
@TeleOp(name = "Gate Sensor Test (CS4)", group = "Bring-up")
public class GateSensorTest extends LinearOpMode {

    public static double JOG_MAX_POWER = 0.25;
    public static double CAPTURE_SEC = 1.0;
    public static double MIN_SEPARATION = 3.0;
    public static double GAIN = 2.0;
    public static int SWEEP_PASSES = 12;
    public static double SWEEP_MAX_SEC = 90.0;
    public static double SWEEP_ABORT_STICK = 0.3;

    private static final double[] SWEEP_POWERS = {0.15, 0.2, 0.3, 0.45, 0.6};
    private static final String[] CONDITIONS = {"empty", "pollen", "nectar", "pollen+nectar"};
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

    private static double wrap180(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    /** mean, population std, min, max of a list. */
    private static double[] summarize(List<Double> values) {
        if (values.isEmpty()) return new double[]{Double.NaN, Double.NaN, Double.NaN, Double.NaN};
        double sum = 0, min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (double x : values) {
            sum += x;
            min = Math.min(min, x);
            max = Math.max(max, x);
        }
        double mean = sum / values.size(), var = 0;
        for (double x : values) var += (x - mean) * (x - mean);
        return new double[]{mean, Math.sqrt(var / values.size()), min, max};
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
        ElapsedTime loopTimer = new ElapsedTime();

        int best = -1;
        double threshold = Double.NaN, band = 0.0;
        boolean gateHigher = true, seen = false;

        int condition = 0, powerIndex = 2;
        boolean sweeping = false;
        ElapsedTime sweepTimer = new ElapsedTime();
        List<Double> rises = new ArrayList<>(), falls = new ArrayList<>();
        boolean haveRise = false, havePrev = false;
        double prevSigned = 0, prevAngle = 0, lastEdgeAngle = 0;
        double stepSum = 0, loopMsSum = 0, speedSum = 0;
        int sweepLoops = 0;
        String sweepStatus = "idle";
        String sweepSummary = "-";

        telemetry.addLine("Gate Sensor Test. Jog the gate slowly, then X (background) and Y (gate in front), then RB to sweep. Nothing moves until you push the stick or start a sweep.");
        telemetry.update();
        waitForStart();
        loopTimer.reset();
        double lastAngleForSpeed = 0;

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            double stick = -gamepad1.left_stick_y;
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
            if (gamepad1.dpadRightWasPressed()) condition = (condition + 1) % CONDITIONS.length;
            if (gamepad1.dpadLeftWasPressed()) condition = (condition + CONDITIONS.length - 1) % CONDITIONS.length;
            if (gamepad1.leftBumperWasPressed() && !sweeping) powerIndex = (powerIndex + 1) % SWEEP_POWERS.length;
            if (gamepad1.backWasPressed()) {
                background = new Stats();
                gateSeen = new Stats();
                best = -1;
                seen = false;
                sweeping = false;
                rises.clear();
                falls.clear();
                sweepSummary = "-";
                sweepStatus = "idle";
            }

            boolean captureBackground = gamepad1.xWasPressed();
            boolean captureGate = gamepad1.yWasPressed();
            if (captureTarget == 0 && !sweeping && (captureBackground || captureGate)) {
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
                        havePrev = false;
                    }
                }
            }

            double signed = 0;
            if (best >= 0) {
                signed = gateHigher ? v[best] - threshold : threshold - v[best];
                if (!seen && signed > band) seen = true;
                if (seen && signed < -band) seen = false;
            }

            // ---- the sweep ----
            if (gamepad1.rightBumperWasPressed()) {
                if (sweeping) {
                    sweeping = false;
                    sweepStatus = "stopped by you";
                } else if (best >= 0 && captureTarget == 0) {
                    sweeping = true;
                    sweepTimer.reset();
                    rises.clear();
                    falls.clear();
                    haveRise = false;
                    havePrev = false;
                    lastEdgeAngle = angle;
                    stepSum = loopMsSum = speedSum = 0;
                    sweepLoops = 0;
                    sweepStatus = "running";
                    sweepSummary = "-";
                    lastAngleForSpeed = angle;
                }
            }

            double power;
            if (sweeping) {
                power = SWEEP_POWERS[powerIndex];
                double stepDeg = Math.abs(angle - lastAngleForSpeed);
                stepSum += stepDeg;
                loopMsSum += loopMs;
                speedSum += loopMs > 0 ? stepDeg / (loopMs / 1000.0) : 0.0;
                sweepLoops++;
                lastAngleForSpeed = angle;

                if (havePrev) {
                    boolean wasOn = prevSigned >= 0, isOn = signed >= 0;
                    if (wasOn != isOn) {
                        double frac = (0.0 - prevSigned) / (signed - prevSigned);   // where the reading crosses the midpoint between two samples
                        double edge = prevAngle + frac * (angle - prevAngle);
                        lastEdgeAngle = edge;
                        if (isOn) {
                            rises.add(edge);
                            haveRise = true;
                        } else if (haveRise && rises.size() > falls.size()) {
                            falls.add(edge);
                            int k = falls.size() - 1;
                            RobotLog.ii(LOG_TAG, String.format(Locale.US,
                                    "SWEEP label=%s power=%.2f pass=%d rise=%.2f fall=%.2f center=%.2f width=%.2f",
                                    CONDITIONS[condition], power, k, rises.get(k), edge, (rises.get(k) + edge) / 2.0, edge - rises.get(k)));
                        }
                    }
                }
                prevSigned = signed;
                prevAngle = angle;
                havePrev = true;

                if (Math.abs(stick) > SWEEP_ABORT_STICK) {
                    sweeping = false;
                    sweepStatus = "aborted by the stick";
                } else if (sweepTimer.seconds() > SWEEP_MAX_SEC) {
                    sweeping = false;
                    sweepStatus = "timed out";
                } else if (angle - lastEdgeAngle > 540.0) {
                    sweeping = false;
                    sweepStatus = "NO CROSSING in 1.5 turns: the sensor is not seeing the arm";
                } else if (falls.size() >= SWEEP_PASSES) {
                    sweeping = false;
                    sweepStatus = "done";
                }

                if (!sweeping) {
                    int passes = falls.size();
                    List<Double> risePhase = new ArrayList<>(), fallPhase = new ArrayList<>(), centerPhase = new ArrayList<>(), widths = new ArrayList<>();
                    for (int k = 0; k < passes; k++) {
                        // phase of each crossing relative to the first rise, wrapped into [-180, 180): a missed pass cannot
                        // show up as a 360 degree jump
                        double r = wrap180(rises.get(k) - rises.get(0));
                        double f = wrap180(falls.get(k) - rises.get(0));
                        risePhase.add(r);
                        fallPhase.add(f);
                        centerPhase.add((r + f) / 2.0);
                        widths.add(falls.get(k) - rises.get(k));
                    }
                    double[] rs = summarize(risePhase), fs = summarize(fallPhase), cs = summarize(centerPhase), ws = summarize(widths);
                    double stepAvg = sweepLoops > 0 ? stepSum / sweepLoops : Double.NaN;
                    double loopAvg = sweepLoops > 0 ? loopMsSum / sweepLoops : Double.NaN;
                    double speedAvg = sweepLoops > 0 ? speedSum / sweepLoops : Double.NaN;
                    sweepSummary = passes == 0 ? "no complete passes" : String.format(Locale.US,
                            "%d passes at power %.2f (%s): RISE spread std %.2f deg (min..max %.2f..%.2f); FALL std %.2f (%.2f..%.2f); CENTER std %.2f (%.2f..%.2f); WIDTH %.1f +/- %.2f; speed %.0f deg/s; %.1f ms per loop; %.2f deg between readings",
                            passes, SWEEP_POWERS[powerIndex], CONDITIONS[condition],
                            rs[1], rs[2], rs[3], fs[1], fs[2], fs[3], cs[1], cs[2], cs[3], ws[0], ws[1], speedAvg, loopAvg, stepAvg);
                    RobotLog.ii(LOG_TAG, "SWEEP SUMMARY label=" + CONDITIONS[condition] + " " + sweepStatus + ": " + sweepSummary);
                }
            } else {
                power = Math.abs(stick) > 0.05 ? stick * JOG_MAX_POWER : 0.0;
            }
            gate.setPower(power);

            if (logTimer.milliseconds() > 150) {
                logTimer.reset();
                double sum = v[1] + v[2] + v[3] + 1e-9;
                RobotLog.ii(LOG_TAG, String.format(Locale.US,
                        "angle=%.1f ticks=%d dist=%.2f r=%.4f g=%.4f b=%.4f a=%.4f chroma=%.2f/%.2f/%.2f seen=%b best=%s sweeping=%b label=%s",
                        angle, ticks, v[0], v[1], v[2], v[3], v[4], v[1] / sum, v[2] / sum, v[3] / sum, seen,
                        best >= 0 ? METRICS[best] : "-", sweeping, CONDITIONS[condition]));
            }

            double chromaSum = v[1] + v[2] + v[3] + 1e-9;
            telemetry.addData("Gate angle (deg, from your mark)", "%.1f   ticks %d   stick %.2f", angle, ticks, stick);
            telemetry.addData("Sensor (distance cm / r / g / b / alpha)", "%.2f / %.4f / %.4f / %.4f / %.4f", v[0], v[1], v[2], v[3], v[4]);
            telemetry.addData("Chromaticity r / g / b (share of the color)", "%.2f / %.2f / %.2f   (grey is near equal; yellow pollen is red and green high, blue low)",
                    v[1] / chromaSum, v[2] / chromaSum, v[3] / chromaSum);
            telemetry.addData("LED / gain / condition", "%s / %.1f / %s", led ? "on" : "off", gain, CONDITIONS[condition]);
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
            } else {
                telemetry.addLine("Jog the arm away from the sensor, press X; then jog it in front of the sensor, press Y.");
            }
            telemetry.addData("SWEEP", sweeping
                    ? String.format(Locale.US, "running at power %.2f: pass %d of %d (%.0f s)", SWEEP_POWERS[powerIndex], falls.size(), SWEEP_PASSES, sweepTimer.seconds())
                    : String.format(Locale.US, "%s   (next power %.2f, LB changes, RB starts)", sweepStatus, SWEEP_POWERS[powerIndex]));
            telemetry.addData("Sweep result", sweepSummary);
            telemetry.addLine("Stick jog, A mark, X background, Y gate, B LED, D-pad up/down gain, D-pad left/right condition, LB power, RB sweep, Back clear");
            telemetry.update();
        }
        gate.setPower(0.0);
    }
}
