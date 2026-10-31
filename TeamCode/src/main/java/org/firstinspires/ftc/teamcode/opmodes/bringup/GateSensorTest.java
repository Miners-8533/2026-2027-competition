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

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.TransportGate;

import java.lang.reflect.Method;
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
 *   Left stick click capture POLLEN: put a yellow pollen in front of the sensor (the arm away), hold still. With it,
 *                    detection picks a reading where the ARM is an extreme among background, arm and pollen (pollen
 *                    reads much brighter than the arm, so alpha alone mistakes it; color shares tell them apart)
 *   B                sensor LED on / off
 *   D-pad up/down    sensor gain +/- 0.5 (affects the color readings; distance is unaffected)
 *   D-pad left/right condition label: empty / pollen / nectar / pollen+nectar (for comparing runs; goes in the log)
 *   Left bumper      next sweep power (0.15, 0.2, 0.3, 0.45, 0.6: include 0.3, the salvo's SWEEP_POWER)
 *   Right bumper     start / stop the automatic SWEEP. With X and Y captured it uses them; without, it learns the alpha
 *                    range itself during the first turn (AUTO_CAL_DEG), then counts passes
 *   Right stick click  next sensor rate preset: SDK default (color and proximity refresh every 100 ms), medium, fast
 *                    (color 25 ms, proximity 6.25 ms). The SDK fixes the default and offers no setter, so this uses
 *                    reflection. The readings rescale, so the captures are cleared (RB self-calibrates again).
 *   Back             clear the captures and the sweep results
 *
 * Why the rate matters: at the default 100 ms the sensor reports a new value only ten times a second, so at 340 deg/s
 * the arm moves about 34 deg between fresh values. The sweep therefore interpolates only between FRESH readings (the
 * value changed), not between loops, so the unknown refresh phase does not show up as a crossing-angle error; what is
 * left is a constant lag that grows with speed. Compare presets and powers to see how much.
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
    /**
     * SOFTWARE gain: the driver multiplies each normalized reading by this and clips at 1.0. It changes nothing in the
     * sensor and adds no signal, so keep it at 1.0 (anything brighter than 1/GAIN of full scale clips). The sensor's own
     * hardware gain is 3x, set at start-up, and the SDK offers no public setter for it.
     */
    public static double GAIN = 1.0;
    public static int SWEEP_PASSES = 12;
    public static double SWEEP_MAX_SEC = 90.0;
    public static double SWEEP_ABORT_STICK = 0.3;
    /** Without X / Y captures the sweep self-calibrates on alpha over this much travel, and needs this much alpha range to trust it. */
    public static double AUTO_CAL_DEG = 400.0;
    public static double AUTO_CAL_MIN_RANGE = 0.03;

    private static final double[] SWEEP_POWERS = {0.15, 0.2, 0.3, 0.45, 0.6};
    private static final String[] CONDITIONS = {"empty", "pollen", "nectar", "pollen+nectar"};
    private static final String SENSOR_NAME = "CS4";
    private static final String[] METRICS = {"distance cm", "red", "green", "blue", "alpha", "red share", "green share", "blue share"};
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

    private static final String[] RATE_NAMES = {
            "SDK default: color 100 ms (18-bit), proximity 100 ms",
            "fast: color 25 ms (16-bit), proximity 6.25 ms",
            "medium: color 50 ms (17-bit), proximity 25 ms"};

    /**
     * The SDK fixes the sensor's measurement rates at start-up (color and proximity both 100 ms) and offers no public
     * way to change them, so this calls its protected setters by reflection. Test-only: if it ever fails, the sensor
     * simply keeps the rates it has. The color readings rescale with the resolution, so captures must be redone.
     */
    private static String applyRates(RevColorSensorV3 sensor, int preset) {
        try {
            BroadcomColorSensor.PSMeasurementRate psRate;
            BroadcomColorSensor.LSResolution lsRes;
            BroadcomColorSensor.LSMeasurementRate lsRate;
            switch (preset) {
                case 1:
                    psRate = BroadcomColorSensor.PSMeasurementRate.R6_25ms;
                    lsRes = BroadcomColorSensor.LSResolution.R16BIT;
                    lsRate = BroadcomColorSensor.LSMeasurementRate.R25ms;
                    break;
                case 2:
                    psRate = BroadcomColorSensor.PSMeasurementRate.R25ms;
                    lsRes = BroadcomColorSensor.LSResolution.R17BIT;
                    lsRate = BroadcomColorSensor.LSMeasurementRate.R50ms;
                    break;
                default:
                    psRate = BroadcomColorSensor.PSMeasurementRate.R100ms;
                    lsRes = BroadcomColorSensor.LSResolution.R18BIT;
                    lsRate = BroadcomColorSensor.LSMeasurementRate.R100ms;
                    break;
            }
            Method ps = BroadcomColorSensorImpl.class.getDeclaredMethod("setPSRateAndRes",
                    BroadcomColorSensor.PSResolution.class, BroadcomColorSensor.PSMeasurementRate.class);
            ps.setAccessible(true);
            ps.invoke(sensor, BroadcomColorSensor.PSResolution.R8BIT, psRate);
            Method ls = BroadcomColorSensorImpl.class.getDeclaredMethod("setLSRateAndRes",
                    BroadcomColorSensor.LSResolution.class, BroadcomColorSensor.LSMeasurementRate.class);
            ls.setAccessible(true);
            ls.invoke(sensor, lsRes, lsRate);
            return "applied: " + RATE_NAMES[preset];
        } catch (Exception e) {
            return "FAILED to apply (" + e + "); the sensor keeps its current rates";
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
        Stats background = new Stats(), gateSeen = new Stats(), pollenSeen = new Stats(), capturing = null;
        int captureTarget = 0;   // 1 = background, 2 = arm in front, 3 = pollen in front
        ElapsedTime moveTimer = new ElapsedTime();
        double moveRefAngle = 0;
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
        boolean autoCal = false;
        double calMin = 0, calMax = 0, calStartAngle = 0, lastLoggedAngle = 0;
        String note = "-";
        int ratePreset = 0;
        String rateStatus = "SDK default (not changed): " + RATE_NAMES[0];
        double lastMetricValue = Double.NaN;
        int sweepFresh = 0;

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
            if (gamepad1.rightStickButtonWasPressed() && !sweeping && captureTarget == 0) {
                ratePreset = (ratePreset + 1) % RATE_NAMES.length;
                rateStatus = applyRates(sensor, ratePreset);
                background = new Stats();
                gateSeen = new Stats();
                pollenSeen = new Stats();
                best = -1;
                seen = false;
                note = "sensor rate changed: the readings rescale, so redo X / Y or let RB self-calibrate";
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "RATE %s (software gain %.1f, LED %s)", rateStatus, gain, led ? "on" : "off"));
            }
            if (gamepad1.backWasPressed()) {
                background = new Stats();
                gateSeen = new Stats();
                pollenSeen = new Stats();
                best = -1;
                seen = false;
                sweeping = false;
                autoCal = false;
                note = "cleared";
                rises.clear();
                falls.clear();
                sweepSummary = "-";
                sweepStatus = "idle";
            }

            boolean captureBackground = gamepad1.xWasPressed();
            boolean captureGate = gamepad1.yWasPressed();
            boolean capturePollen = gamepad1.leftStickButtonWasPressed();
            if (captureBackground || captureGate || capturePollen) {
                if (captureTarget == 0 && !sweeping) {
                    captureTarget = captureBackground ? 1 : captureGate ? 2 : 3;
                    capturing = captureTarget == 1 ? (background = new Stats())
                            : captureTarget == 2 ? (gateSeen = new Stats()) : (pollenSeen = new Stats());
                    captureTimer.reset();
                    note = (captureTarget == 1 ? "BACKGROUND" : captureTarget == 2 ? "ARM" : "POLLEN") + " capture started: hold still";
                } else {
                    note = "capture ignored: wait until the current capture or the sweep has finished";
                }
            }

            double[] v = new double[N];
            NormalizedRGBA c = sensor.getNormalizedColors();
            v[0] = sensor.getDistance(DistanceUnit.CM);
            v[1] = c.red;
            v[2] = c.green;
            v[3] = c.blue;
            v[4] = c.alpha;
            double colorSum = c.red + c.green + c.blue + 1e-9;
            v[5] = c.red / colorSum;
            v[6] = c.green / colorSum;
            v[7] = c.blue / colorSum;

            if (captureTarget != 0) {
                capturing.add(v);
                if (captureTimer.seconds() >= CAPTURE_SEC) {
                    String captureName = captureTarget == 1 ? "BACKGROUND" : captureTarget == 2 ? "ARM" : "POLLEN";
                    RobotLog.ii(LOG_TAG, String.format(Locale.US, "CAPTURE %s done: %d samples, distance %.2f, alpha %.4f (+/-%.4f), r %.4f g %.4f b %.4f, shares %.3f/%.3f/%.3f (software gain %.1f, LED %s, %s)",
                            captureName, capturing.n, capturing.mean(0), capturing.mean(4), capturing.std(4),
                            capturing.mean(1), capturing.mean(2), capturing.mean(3), capturing.mean(5), capturing.mean(6), capturing.mean(7),
                            gain, led ? "on" : "off", RATE_NAMES[ratePreset]));
                    captureTarget = 0;
                    capturing = null;
                    if (background.n > 0 && gateSeen.n > 0) {
                        // pick the reading that separates the ARM best. With a pollen capture the arm must be an EXTREME among
                        // background, arm and pollen (one threshold cannot isolate it otherwise), and the score is the weaker of
                        // its two separations (from the background, from the pollen).
                        double bestScore = -1;
                        best = -1;
                        for (int i = 0; i < N; i++) {
                            double arm = gateSeen.mean(i), bg = background.mean(i);
                            double score = Math.abs(arm - bg) / (background.std(i) + gateSeen.std(i) + 1e-6);
                            if (pollenSeen.n > 0) {
                                double pol = pollenSeen.mean(i);
                                if (!((arm > bg && arm > pol) || (arm < bg && arm < pol))) continue;
                                score = Math.min(score, Math.abs(arm - pol) / (pollenSeen.std(i) + gateSeen.std(i) + 1e-6));
                            }
                            if (score > bestScore) {
                                bestScore = score;
                                best = i;
                            }
                        }
                        if (best >= 0) {
                            double arm = gateSeen.mean(best), bg = background.mean(best);
                            double other = bg;
                            if (pollenSeen.n > 0) other = arm > bg ? Math.max(bg, pollenSeen.mean(best)) : Math.min(bg, pollenSeen.mean(best));
                            threshold = (arm + other) / 2.0;
                            band = 0.15 * Math.abs(arm - other);
                            gateHigher = arm > other;
                            havePrev = false;
                            note = captureName + " captured. Detecting the arm with " + METRICS[best] + ". RB starts the sweep.";
                        } else {
                            note = captureName + " captured, but no single reading separates the arm from both the background and the pollen.";
                        }
                    } else {
                        note = captureName + " captured. Still needed: X (background) and Y (arm in front)."
                                + " L3 adds a pollen capture so pollen is not mistaken for the arm.";
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
                } else if (captureTarget != 0) {
                    note = "sweep not started: a capture is still running";
                } else {
                    boolean haveCaptures = best >= 0 && background.n > 0 && gateSeen.n > 0;
                    autoCal = !haveCaptures;
                    if (autoCal) {
                        best = -1;
                        calMin = Double.POSITIVE_INFINITY;
                        calMax = Double.NEGATIVE_INFINITY;
                        calStartAngle = angle;
                        note = "no X / Y captures: the sweep self-calibrates on alpha during its first turn (" + (int) AUTO_CAL_DEG
                                + " deg). Pollen in view can fool this (it is brighter than the arm): then use X, Y and L3 instead.";
                    } else {
                        note = "sweep using your X / Y captures";
                    }
                    sweeping = true;
                    sweepTimer.reset();
                    rises.clear();
                    falls.clear();
                    haveRise = false;
                    havePrev = false;
                    lastEdgeAngle = angle;
                    stepSum = loopMsSum = speedSum = 0;
                    sweepLoops = 0;
                    sweepFresh = 0;
                    lastMetricValue = Double.NaN;
                    sweepStatus = autoCal ? "calibrating" : "running";
                    sweepSummary = "-";
                    lastAngleForSpeed = angle;
                    moveRefAngle = angle;
                    moveTimer.reset();
                    RobotLog.ii(LOG_TAG, String.format(Locale.US, "SWEEP START label=%s power=%.2f autoCal=%b software gain %.1f, LED %s, %s", CONDITIONS[condition], SWEEP_POWERS[powerIndex], autoCal, gain, led ? "on" : "off", RATE_NAMES[ratePreset]));
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

                // Work only on FRESH sensor readings. The sensor refreshes every 100 ms by default, so most loops see the
                // same value again; interpolating between loops would put a crossing at the wrong angle by up to a whole
                // refresh (the repeating three-turn sawtooth in the first sweep). Between fresh readings the lag is constant.
                double metricNow = v[autoCal || best < 0 ? 4 : best];
                boolean fresh = metricNow != lastMetricValue;
                if (fresh) {
                    lastMetricValue = metricNow;
                    sweepFresh++;
                }
                if (autoCal) {
                    // no X / Y captures: learn the alpha range (arm away .. arm in front) over the first turn
                    if (fresh) {
                        calMin = Math.min(calMin, v[4]);
                        calMax = Math.max(calMax, v[4]);
                    }
                    if (angle - calStartAngle >= AUTO_CAL_DEG) {
                        autoCal = false;
                        if (calMax - calMin < AUTO_CAL_MIN_RANGE) {
                            sweeping = false;
                            sweepStatus = String.format(Locale.US, "NO ARM SEEN in a full turn: alpha only varied %.3f (needs %.2f)", calMax - calMin, AUTO_CAL_MIN_RANGE);
                        } else {
                            best = 4;                                   // alpha
                            threshold = (calMin + calMax) / 2.0;
                            band = 0.15 * (calMax - calMin);
                            gateHigher = true;
                            seen = false;
                            signed = v[4] - threshold;
                            havePrev = false;
                            lastEdgeAngle = angle;
                            sweepStatus = String.format(Locale.US, "running (self-calibrated: alpha %.3f .. %.3f, threshold %.3f)", calMin, calMax, threshold);
                            RobotLog.ii(LOG_TAG, "SWEEP " + sweepStatus);
                        }
                    }
                } else if (fresh) {
                    if (havePrev) {
                        boolean wasOn = prevSigned >= 0, isOn = signed >= 0;
                        if (wasOn != isOn) {
                            double frac = (0.0 - prevSigned) / (signed - prevSigned);   // where the reading crosses the midpoint between two fresh readings
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
                }

                if (Math.abs(angle - moveRefAngle) > 2.0) {
                    moveRefAngle = angle;
                    moveTimer.reset();
                }
                if (Math.abs(stick) > SWEEP_ABORT_STICK) {
                    sweeping = false;
                    sweepStatus = "aborted by the stick";
                } else if (moveTimer.seconds() > 1.0) {
                    sweeping = false;
                    sweepStatus = "GATE NOT MOVING for 1 s (blocked, for example by a pollen?): stopped";
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
                    double freshMs = sweepTimer.seconds() * 1000.0 / Math.max(1, sweepFresh);
                    sweepSummary = passes == 0 ? "no complete passes" : String.format(Locale.US,
                            "%d passes at power %.2f (%s): RISE spread std %.2f deg (min..max %.2f..%.2f); FALL std %.2f (%.2f..%.2f); CENTER std %.2f (%.2f..%.2f); WIDTH %.1f +/- %.2f; speed %.0f deg/s; %.1f ms per loop; %.2f deg between loops; sensor value changed every %.0f ms (= %.1f deg of travel) [%s]",
                            passes, SWEEP_POWERS[powerIndex], CONDITIONS[condition],
                            rs[1], rs[2], rs[3], fs[1], fs[2], fs[3], cs[1], cs[2], cs[3], ws[0], ws[1], speedAvg, loopAvg, stepAvg,
                            freshMs, speedAvg * freshMs / 1000.0, RATE_NAMES[ratePreset]);
                    RobotLog.ii(LOG_TAG, String.format(Locale.US, "SWEEP SUMMARY label=%s (software gain %.1f, LED %s) %s: %s",
                            CONDITIONS[condition], gain, led ? "on" : "off", sweepStatus, sweepSummary));
                }
            } else {
                power = Math.abs(stick) > 0.05 ? stick * JOG_MAX_POWER : 0.0;
            }
            gate.setPower(power);

            // the phone keeps only 256 KB of log: write often only while something is changing, so the sweep lines survive
            boolean moving = sweeping || Math.abs(angle - lastLoggedAngle) > 0.5 || Math.abs(stick) > 0.05;
            double logEveryMs = sweeping ? 500.0 : moving ? 150.0 : 1000.0;
            if (logTimer.milliseconds() > logEveryMs) {
                logTimer.reset();
                lastLoggedAngle = angle;
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
            telemetry.addData("Captures taken", "background %d, arm %d, pollen %d samples", background.n, gateSeen.n, pollenSeen.n);
            telemetry.addData("Note", note);
            telemetry.addData("Sensor rate (right stick click changes)", rateStatus);
            if (background.n > 0 && gateSeen.n > 0) {
                for (int i = 0; i < N; i++) {
                    double sep = Math.abs(gateSeen.mean(i) - background.mean(i)) / (background.std(i) + gateSeen.std(i) + 1e-6);
                    String pollenPart = "";
                    if (pollenSeen.n > 0) {
                        double sepPollen = Math.abs(gateSeen.mean(i) - pollenSeen.mean(i)) / (pollenSeen.std(i) + gateSeen.std(i) + 1e-6);
                        pollenPart = String.format(Locale.US, "  pollen %.4f  arm-vs-pollen separation %.1f", pollenSeen.mean(i), sepPollen);
                    }
                    telemetry.addData((i == best ? "BEST  " : "      ") + METRICS[i],
                            "background %.4f (+/-%.4f)  arm %.4f (+/-%.4f)  separation %.1f%s%s",
                            background.mean(i), background.std(i), gateSeen.mean(i), gateSeen.std(i), sep,
                            sep < MIN_SEPARATION ? "  (weak)" : "", pollenPart);
                }
                telemetry.addData("DETECTION", best < 0 ? "-" : String.format(Locale.US, "%s using %s (threshold %.4f, gate reads %s)",
                        seen ? "SEEN" : "not seen", METRICS[best], threshold, gateHigher ? "HIGHER" : "LOWER"));
            } else {
                telemetry.addLine("Optional: jog the arm away from the sensor and press X, then in front of it and press Y. Or just press RB: the sweep then self-calibrates on its first turn.");
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
