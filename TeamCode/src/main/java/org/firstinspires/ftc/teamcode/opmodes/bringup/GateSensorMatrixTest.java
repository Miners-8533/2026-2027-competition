package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.TransportGate;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * One automatic run that steps CS4 (REV Color Sensor V3) through every setting we want to compare, with the robot
 * EMPTY of game elements, and reports which settings find the gate arm best.
 *
 * Why: the plan is to check the gate after a salvo by turning it to the CS4 position and looking for the arm, and
 * to re-home with a slow sweep if it is not seen. With no pollen in play the only questions are which sensor setting
 * (hardware gain, LED, sweep speed) gives a clean, repeatable arm signal that does not saturate.
 *
 * For every combination of HARDWARE GAIN (1x, 3x default, 6x, 9x, 18x), LED (on; also off if TEST_LED_OFF) and
 * SWEEP POWER (0.15, 0.3, 0.6), it: sets the sensor, waits SETTLE_SEC, turns the gate forward through one calibration
 * turn (CAL_DEG) to learn the empty and arm-in-front alpha levels, then PASSES_PER_SETTING passes recording the encoder
 * angle where the arm enters and leaves the sensor's view (interpolated between FRESH readings only). Per setting it
 * logs: empty level and noise, arm level and noise, the separation (gap / noise), whether alpha saturated, and the
 * spread of the arm's rising edge and center from turn to turn, the arc width, the speed and the loop time. At the end
 * it ranks the settings. Sensor rate is the fast preset throughout (the default 100 ms refresh cannot locate the arm).
 *
 * The hardware gain and the rate are set by reflection on protected SDK methods: test-only code.
 *
 *   Left stick       jog the gate (up = forward) before starting, JOG_MAX_POWER at full deflection
 *   A                mark the gate angle as 0 here. Put the arm at HOLD first: the results then say where the arc is,
 *                    in degrees AHEAD of HOLD, at each sweep speed (the team expects about 90 deg)
 *   Right bumper     start the whole run (any stick push, Back or a gate that stops moving ends it)
 *   Back             stop the run / clear the results
 *
 * Needs a REV Color Sensor V3 named "CS4" and the motor "gate". The gate has no stops: keep clear. Everything is also
 * written to the robot log (tag GateSensorMatrix); read it over USB.
 */
@Config
@TeleOp(name = "Gate Sensor Matrix (CS4)", group = "Bring-up")
public class GateSensorMatrixTest extends LinearOpMode {

    public static double JOG_MAX_POWER = 0.25;
    public static int PASSES_PER_SETTING = 6;
    public static double SETTLE_SEC = 0.6;
    public static double CAL_DEG = 400.0;
    public static double ABORT_STICK = 0.3;
    public static double MAX_SEC_PER_SETTING = 60.0;
    public static boolean TEST_LED_OFF = false;
    /** Only the SDK default hardware gain (3x), which the first matrix run recommended; false steps through 1x to 18x as before. */
    public static boolean ONLY_DEFAULT_GAIN = true;
    /** Alpha (software gain 1.0) at or above this counts as saturated: the sensor ran out of range. */
    public static double SATURATION_ALPHA = 0.98;
    /** The arm must raise alpha by at least this much over the empty level, or the setting is reported as "arm not seen". */
    public static double MIN_RANGE = 0.02;
    /** A usable setting keeps the arm's alpha below this, leaving headroom for brighter lighting than the test's. */
    public static double HEADROOM_ALPHA = 0.6;

    private static final String SENSOR_NAME = "CS4";
    private static final String LOG_TAG = "GateSensorMatrix";
    private static final double[] POWERS = {0.15, 0.3, 0.6};
    private static final BroadcomColorSensor.Gain[] GAINS = {
            BroadcomColorSensor.Gain.GAIN_1, BroadcomColorSensor.Gain.GAIN_3, BroadcomColorSensor.Gain.GAIN_6,
            BroadcomColorSensor.Gain.GAIN_9, BroadcomColorSensor.Gain.GAIN_18};
    private static final String[] GAIN_NAMES = {"1x", "3x (SDK default)", "6x", "9x", "18x"};

    private enum Phase { IDLE, SETTLE, CALIBRATE, MEASURE, DONE }

    private static final class Combo {
        final int gainIndex;
        final boolean led;
        final double power;

        Combo(int gainIndex, boolean led, double power) {
            this.gainIndex = gainIndex;
            this.led = led;
            this.power = power;
        }

        String label() {
            return String.format(Locale.US, "gain %s, LED %s, power %.2f", GAIN_NAMES[gainIndex], led ? "on" : "off", power);
        }
    }

    private static final class Sums {
        int n = 0;
        double sum = 0, sumSq = 0;

        void add(double x) {
            n++;
            sum += x;
            sumSq += x * x;
        }

        double mean() {
            return n == 0 ? Double.NaN : sum / n;
        }

        double std() {
            if (n < 2) return 0.0;
            double m = sum / n;
            return Math.sqrt(Math.max(0.0, sumSq / n - m * m));
        }
    }

    private static final class Result {
        double power;
        String label;
        boolean ok;
        String status;
        boolean saturated;
        int passes;
        double bgMean, bgStd, armMean, armStd, separation;
        double riseStd = Double.NaN, centerStd = Double.NaN, widthMean = Double.NaN, widthStd = Double.NaN;
        /** Where the arc is, in degrees AHEAD of the mark (HOLD), 0 to 360: where it starts, where it ends, and its center. */
        double riseAbs = Double.NaN, fallAbs = Double.NaN, centerAbs = Double.NaN;
        double speed = Double.NaN, loopMs = Double.NaN, freshMs = Double.NaN;
        double armMax = Double.NaN;
    }

    private static double norm360(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    private static double wrap180(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static double std(List<Double> values) {
        if (values.size() < 2) return 0.0;
        double sum = 0;
        for (double x : values) sum += x;
        double mean = sum / values.size(), var = 0;
        for (double x : values) var += (x - mean) * (x - mean);
        return Math.sqrt(var / values.size());
    }

    private static double mean(List<Double> values) {
        if (values.isEmpty()) return Double.NaN;
        double sum = 0;
        for (double x : values) sum += x;
        return sum / values.size();
    }

    /** Fast preset: color 25 ms (16-bit), proximity 6.25 ms, set by reflection (the SDK has no public setter). */
    private static String applyFastRate(RevColorSensorV3 sensor) {
        try {
            Method ps = BroadcomColorSensorImpl.class.getDeclaredMethod("setPSRateAndRes",
                    BroadcomColorSensor.PSResolution.class, BroadcomColorSensor.PSMeasurementRate.class);
            ps.setAccessible(true);
            ps.invoke(sensor, BroadcomColorSensor.PSResolution.R8BIT, BroadcomColorSensor.PSMeasurementRate.R6_25ms);
            Method ls = BroadcomColorSensorImpl.class.getDeclaredMethod("setLSRateAndRes",
                    BroadcomColorSensor.LSResolution.class, BroadcomColorSensor.LSMeasurementRate.class);
            ls.setAccessible(true);
            ls.invoke(sensor, BroadcomColorSensor.LSResolution.R16BIT, BroadcomColorSensor.LSMeasurementRate.R25ms);
            return "fast rate applied (color 25 ms 16-bit, proximity 6.25 ms)";
        } catch (Exception e) {
            return "FAST RATE FAILED (" + e + ")";
        }
    }

    /** The sensor's HARDWARE gain (1, 3, 6, 9, 18x), by reflection on the protected setter. */
    private static String applyHardwareGain(RevColorSensorV3 sensor, BroadcomColorSensor.Gain gain) {
        try {
            Method m = BroadcomColorSensorImpl.class.getDeclaredMethod("setHardwareGain", BroadcomColorSensor.Gain.class);
            m.setAccessible(true);
            m.invoke(sensor, gain);
            return "ok";
        } catch (Exception e) {
            return "FAILED (" + e + ")";
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
        sensor.setGain(1.0f);   // software gain 1.0: nothing is rescaled or clipped by the driver
        String rateStatus = applyFastRate(sensor);
        RobotLog.ii(LOG_TAG, rateStatus);

        int markTicks = gate.getCurrentPosition();
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime phaseTimer = new ElapsedTime();
        ElapsedTime comboTimer = new ElapsedTime();
        ElapsedTime moveTimer = new ElapsedTime();
        ElapsedTime logTimer = new ElapsedTime();

        Phase phase = Phase.IDLE;
        List<Combo> combos = new ArrayList<>();
        List<Result> results = new ArrayList<>();
        int comboIndex = 0;
        Combo combo = null;

        // per-setting working state
        double calStartAngle = 0, threshold = 0, band = 0, calBg = 0, calMax = 0;
        List<Double> calVals = new ArrayList<>();
        Sums bgSums = new Sums(), armSums = new Sums();
        List<Double> rises = new ArrayList<>(), falls = new ArrayList<>();
        boolean haveRise = false, havePrev = false;
        double prevSigned = 0, prevAngle = 0, lastEdgeAngle = 0, lastMetric = Double.NaN;
        double measureStartAngle = 0, loopMsSum = 0;
        int loops = 0, fresh = 0;
        double moveRefAngle = 0;
        String runStatus = "idle: jog the gate if you like, then press the right bumper to start the run";
        String ranking = "-";

        telemetry.addLine("Gate Sensor Matrix (CS4). Empty robot, nothing in the transport. Right bumper starts the automatic run.");
        telemetry.update();
        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            double stick = -gamepad1.left_stick_y;
            int ticks = gate.getCurrentPosition();
            double angle = TransportGate.ENCODER_SIGN * (ticks - markTicks) * 360.0 / TransportGate.TICKS_PER_REV;
            NormalizedRGBA c = sensor.getNormalizedColors();
            double alpha = c.alpha;
            boolean isFresh = alpha != lastMetric;
            if (isFresh) lastMetric = alpha;

            if (gamepad1.aWasPressed() && (phase == Phase.IDLE || phase == Phase.DONE)) {
                markTicks = ticks;
                angle = 0.0;
                RobotLog.ii(LOG_TAG, "MARK: the gate angle is now 0 here (HOLD)");
                runStatus = "HOLD marked here: the arc positions in the results are measured from this mark";
            }
            boolean abort = false;
            String abortWhy = "";
            if (gamepad1.backWasPressed()) {
                if (phase == Phase.IDLE || phase == Phase.DONE) {
                    results.clear();
                    ranking = "-";
                    runStatus = "cleared";
                    phase = Phase.IDLE;
                } else {
                    abort = true;
                    abortWhy = "stopped with Back";
                }
            }

            double power = 0.0;
            switch (phase) {
                case IDLE:
                case DONE:
                    power = Math.abs(stick) > 0.05 ? stick * JOG_MAX_POWER : 0.0;
                    if (gamepad1.rightBumperWasPressed()) {
                        combos.clear();
                        results.clear();
                        for (int g = 0; g < GAINS.length; g++) {
                            if (ONLY_DEFAULT_GAIN && g != 1) continue;
                            for (int led = 0; led < (TEST_LED_OFF ? 2 : 1); led++) {
                                for (double p : POWERS) combos.add(new Combo(g, led == 0, p));
                            }
                        }
                        comboIndex = 0;
                        ranking = "-";
                        RobotLog.ii(LOG_TAG, String.format(Locale.US, "RUN START: %d settings, %d passes each, software gain 1.0, %s", combos.size(), PASSES_PER_SETTING, rateStatus));
                        combo = combos.get(0);
                        applyHardwareGain(sensor, GAINS[combo.gainIndex]);
                        sensor.enableLed(combo.led);
                        phase = Phase.SETTLE;
                        phaseTimer.reset();
                        comboTimer.reset();
                        runStatus = "setting 1 of " + combos.size() + ": " + combo.label();
                    }
                    break;

                case SETTLE:
                    power = 0.0;
                    if (Math.abs(stick) > ABORT_STICK) {
                        abort = true;
                        abortWhy = "aborted by the stick";
                    } else if (phaseTimer.seconds() >= SETTLE_SEC) {
                        phase = Phase.CALIBRATE;
                        calVals.clear();
                        calStartAngle = angle;
                        moveRefAngle = angle;
                        moveTimer.reset();
                        lastMetric = Double.NaN;
                    }
                    break;

                case CALIBRATE:
                case MEASURE: {
                    power = combo.power;
                    if (Math.abs(angle - moveRefAngle) > 2.0) {
                        moveRefAngle = angle;
                        moveTimer.reset();
                    }
                    if (Math.abs(stick) > ABORT_STICK) {
                        abort = true;
                        abortWhy = "aborted by the stick";
                    } else if (moveTimer.seconds() > 1.0) {
                        abort = true;
                        abortWhy = "gate not moving for 1 s (blocked?)";
                    } else if (comboTimer.seconds() > MAX_SEC_PER_SETTING) {
                        abort = true;
                        abortWhy = "timed out";
                    }
                    if (abort) break;

                    if (phase == Phase.CALIBRATE) {
                        if (isFresh) calVals.add(alpha);
                        if (angle - calStartAngle >= CAL_DEG && calVals.size() >= 4) {
                            List<Double> sorted = new ArrayList<>(calVals);
                            Collections.sort(sorted);
                            calBg = sorted.get(sorted.size() / 2);     // the arm is in view a small part of a turn, so the median is the empty level
                            calMax = sorted.get(sorted.size() - 1);
                            if (calMax - calBg < MIN_RANGE) {
                                Result r = new Result();
                                r.label = combo.label();
                                r.power = combo.power;
                                r.ok = false;
                                r.status = String.format(Locale.US, "ARM NOT SEEN (alpha range only %.3f)", calMax - calBg);
                                r.bgMean = calBg;
                                r.armMean = calMax;
                                results.add(r);
                                RobotLog.ii(LOG_TAG, "MATRIX " + r.label + ": " + r.status);
                                combo = null;
                                phase = Phase.SETTLE;     // advance below
                                comboIndex++;
                                break;
                            }
                            threshold = (calBg + calMax) / 2.0;
                            band = 0.15 * (calMax - calBg);
                            bgSums = new Sums();
                            armSums = new Sums();
                            rises.clear();
                            falls.clear();
                            haveRise = false;
                            havePrev = false;
                            lastEdgeAngle = angle;
                            measureStartAngle = angle;
                            loopMsSum = 0;
                            loops = 0;
                            fresh = 0;
                            phaseTimer.reset();
                            phase = Phase.MEASURE;
                        }
                    } else {
                        loops++;
                        loopMsSum += loopMs;
                        double signed = alpha - threshold;
                        if (isFresh) {
                            fresh++;
                            if (signed > band) armSums.add(alpha);
                            else if (signed < -band) bgSums.add(alpha);
                            if (havePrev) {
                                boolean wasOn = prevSigned >= 0, isOn = signed >= 0;
                                if (wasOn != isOn) {
                                    double frac = (0.0 - prevSigned) / (signed - prevSigned);
                                    double edge = prevAngle + frac * (angle - prevAngle);
                                    lastEdgeAngle = edge;
                                    if (isOn) {
                                        rises.add(edge);
                                        haveRise = true;
                                    } else if (haveRise && rises.size() > falls.size()) {
                                        falls.add(edge);
                                    }
                                }
                            }
                            prevSigned = signed;
                            prevAngle = angle;
                            havePrev = true;
                        }

                        boolean enough = falls.size() >= PASSES_PER_SETTING;
                        boolean lost = angle - lastEdgeAngle > 540.0;
                        if (enough || lost) {
                            Result r = new Result();
                            r.label = combo.label();
                            r.power = combo.power;
                            r.passes = falls.size();
                            r.bgMean = bgSums.mean();
                            r.bgStd = bgSums.std();
                            r.armMean = armSums.mean();
                            r.armStd = armSums.std();
                            r.armMax = calMax;
                            r.saturated = calMax >= SATURATION_ALPHA;
                            r.separation = (r.armMean - r.bgMean) / (r.bgStd + r.armStd + 1e-9);
                            List<Double> risePhase = new ArrayList<>(), centerPhase = new ArrayList<>(), widths = new ArrayList<>();
                            for (int k = 0; k < r.passes; k++) {
                                double rp = wrap180(rises.get(k) - rises.get(0));
                                double fp = wrap180(falls.get(k) - rises.get(0));
                                risePhase.add(rp);
                                centerPhase.add((rp + fp) / 2.0);
                                widths.add(falls.get(k) - rises.get(k));
                            }
                            if (r.passes >= 2) {
                                r.riseStd = std(risePhase);
                                r.centerStd = std(centerPhase);
                                r.widthMean = mean(widths);
                                r.widthStd = std(widths);
                                // absolute position of the arc relative to the mark (A): the first rise plus the mean phases
                                double riseMeanPhase = mean(risePhase);
                                r.riseAbs = norm360(rises.get(0) + riseMeanPhase);
                                r.fallAbs = norm360(rises.get(0) + riseMeanPhase + r.widthMean);
                                r.centerAbs = norm360(rises.get(0) + mean(centerPhase));
                            }
                            double seconds = Math.max(1e-3, phaseTimer.seconds());
                            r.speed = Math.abs(angle - measureStartAngle) / seconds;
                            r.loopMs = loops > 0 ? loopMsSum / loops : Double.NaN;
                            r.freshMs = fresh > 0 ? seconds * 1000.0 / fresh : Double.NaN;
                            r.ok = enough && !r.saturated;
                            r.status = enough ? (r.saturated ? "SATURATED" : "ok") : "LOST THE ARM after " + r.passes + " passes";
                            results.add(r);
                            RobotLog.ii(LOG_TAG, String.format(Locale.US,
                                    "MATRIX %s: %s; passes %d; empty alpha %.4f (+/-%.4f); arm alpha %.4f (+/-%.4f), max %.3f; separation %.1f; RISE std %.2f deg; CENTER std %.2f deg; width %.1f +/- %.2f; ARC %.1f to %.1f deg ahead of the mark (center %.1f); speed %.0f deg/s; %.1f ms per loop; value changed every %.0f ms",
                                    r.label, r.status, r.passes, r.bgMean, r.bgStd, r.armMean, r.armStd, r.armMax, r.separation,
                                    r.riseStd, r.centerStd, r.widthMean, r.widthStd, r.riseAbs, r.fallAbs, r.centerAbs, r.speed, r.loopMs, r.freshMs));
                            combo = null;
                            phase = Phase.SETTLE;     // advance below
                            comboIndex++;
                        }
                    }
                    break;
                }
            }

            if (abort) {
                gate.setPower(0.0);
                RobotLog.ii(LOG_TAG, "RUN ENDED early: " + abortWhy + " after " + results.size() + " of " + combos.size() + " settings");
                runStatus = "ENDED early: " + abortWhy;
                phase = Phase.DONE;
                combo = null;
                power = 0.0;
            } else if ((phase == Phase.SETTLE) && combo == null) {
                // a setting just finished: start the next one, or finish the run
                power = 0.0;
                if (comboIndex >= combos.size()) {
                    phase = Phase.DONE;
                    // rank WITHIN each sweep power (the spread grows with speed, so a slow run must not beat a fast one):
                    // complete, not saturated, arm alpha with headroom, lowest center spread first
                    StringBuilder sb = new StringBuilder();
                    for (double p : POWERS) {
                        List<Result> usable = new ArrayList<>();
                        for (Result r : results) {
                            if (r.power == p && r.ok && !Double.isNaN(r.centerStd) && r.armMax <= HEADROOM_ALPHA) usable.add(r);
                        }
                        Collections.sort(usable, (a, b) -> Double.compare(a.centerStd, b.centerStd));
                        sb.append(String.format(Locale.US, "At power %.2f: ", p));
                        if (usable.isEmpty()) {
                            sb.append("no setting was complete, unsaturated and under the headroom limit\n");
                            continue;
                        }
                        sb.append("\n");
                        for (int i = 0; i < Math.min(3, usable.size()); i++) {
                            Result r = usable.get(i);
                            sb.append(String.format(Locale.US, "   %d. %s: center std %.2f deg, rise std %.2f, separation %.0f, arm alpha %.3f\n",
                                    i + 1, r.label, r.centerStd, r.riseStd, r.separation, r.armMax));
                        }
                    }
                    ranking = sb.toString();
                    RobotLog.ii(LOG_TAG, "RANKING (per sweep power; complete, not saturated, arm alpha <= " + HEADROOM_ALPHA + ", lowest center spread first):\n" + ranking);
                    runStatus = "DONE: " + results.size() + " settings measured";
                    gate.setPower(0.0);
                } else {
                    combo = combos.get(comboIndex);
                    applyHardwareGain(sensor, GAINS[combo.gainIndex]);
                    sensor.enableLed(combo.led);
                    phaseTimer.reset();
                    comboTimer.reset();
                    runStatus = "setting " + (comboIndex + 1) + " of " + combos.size() + ": " + combo.label();
                }
            }
            gate.setPower(power);

            if (logTimer.milliseconds() > (phase == Phase.IDLE || phase == Phase.DONE ? 1000.0 : 2000.0)) {
                logTimer.reset();
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "angle=%.1f alpha=%.4f phase=%s %s", angle, alpha, phase, combo == null ? "-" : combo.label()));
            }

            telemetry.addData("Run", runStatus);
            telemetry.addData("Phase", combo == null ? phase.toString() : String.format(Locale.US, "%s (%d passes of %d)", phase, falls.size(), PASSES_PER_SETTING));
            telemetry.addData("Gate angle / alpha", "%.1f deg / %.4f   (stick %.2f)", angle, alpha, stick);
            telemetry.addData("Sensor", rateStatus);
            int from = Math.max(0, results.size() - 8);
            for (int i = from; i < results.size(); i++) {
                Result r = results.get(i);
                telemetry.addData((i + 1) + ". " + r.label,
                        "%s  empty %.3f / arm %.3f  sep %.0f  rise std %.2f  center std %.2f  width %.1f  arc center %.1f deg ahead of the mark",
                        r.status, r.bgMean, r.armMean, r.separation, r.riseStd, r.centerStd, r.widthMean, r.centerAbs);
            }
            if (phase == Phase.DONE) telemetry.addData("Ranking", ranking);
            telemetry.addLine("Right bumper: start the run   Left stick: jog (before) / abort (during)   Back: stop or clear");
            telemetry.update();
        }
        gate.setPower(0.0);
    }
}
