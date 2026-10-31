package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Locale;

/**
 * Shows the live readings of the four color sensors CS1 to CS4 (REV Color Sensor V3), so elements can be moved in
 * front of them to see how pollen reads. The eventual use is to tell when the robot holds 4 elements.
 *
 * For each sensor: distance (cm), alpha (total light), red / green / blue, the color shares (each channel's part of the
 * color), how long its reads took, and, once an EMPTY baseline is captured, whether something is PRESENT and whether it
 * looks like yellow POLLEN. Presence = alpha well above the empty level, or the object clearly nearer than the empty
 * distance. Yellow = high green share and low blue share (pollen measured about 0.29 / 0.52 / 0.20 against 0.31 / 0.38
 * / 0.31 for an empty view, and about 2 cm away against 7.8 cm). The thresholds are in the FTC Dashboard.
 *
 *   A         capture the EMPTY baseline for all four (nothing in front of any sensor; holds still for CAPTURE_SEC)
 *   B         sensor LEDs on / off (all four)
 *   Back      clear the baseline
 *
 * Needs four REV Color Sensor V3 named CS1, CS2, CS3, CS4. They all have the same I2C address, so each must be on its
 * own I2C bus. Uses the SDK default sensor rate (100 ms refresh), where the distance reading is calibrated. Everything
 * is also written to the robot log (tag ColorSensors): once a second, and at once whenever a sensor's PRESENT or POLLEN
 * state changes, so it can be read over USB.
 */
@Config
@TeleOp(name = "Color Sensors Test (CS1-CS4)", group = "Bring-up")
public class ColorSensorsTest extends LinearOpMode {

    /** Software gain: a multiplier clipped at 1.0, so keep it at 1.0. */
    public static double GAIN = 1.0;
    public static double CAPTURE_SEC = 1.0;
    /** PRESENT when alpha is at least this many times the empty level AND at least ALPHA_MIN_RISE above it ... */
    public static double ALPHA_RATIO = 2.0;
    public static double ALPHA_MIN_RISE = 0.02;
    /** ... or the object is at least this much nearer than the empty distance. */
    public static double DIST_DROP_CM = 1.5;
    /** Yellow pollen: green share at least this, blue share at most this. */
    public static double YELLOW_MIN_GREEN_SHARE = 0.45;
    public static double YELLOW_MAX_BLUE_SHARE = 0.27;

    private static final String[] NAMES = {"CS1", "CS2", "CS3", "CS4"};
    private static final String LOG_TAG = "ColorSensors";
    private static final int N = NAMES.length;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        NormalizedColorSensor[] color = new NormalizedColorSensor[N];
        DistanceSensor[] distance = new DistanceSensor[N];
        for (int i = 0; i < N; i++) {
            color[i] = hardwareMap.tryGet(NormalizedColorSensor.class, NAMES[i]);
            distance[i] = hardwareMap.tryGet(DistanceSensor.class, NAMES[i]);
            if (color[i] != null) color[i].setGain((float) GAIN);
        }
        boolean led = true;
        setLeds(color, led);

        double[] baseAlpha = new double[N], baseDist = new double[N];
        boolean[] haveBase = new boolean[N];
        double[] sumAlpha = new double[N], sumDist = new double[N];
        int[] sumCount = new int[N];
        boolean capturing = false;
        ElapsedTime captureTimer = new ElapsedTime();
        ElapsedTime logTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        boolean[] wasPresent = new boolean[N], wasPollen = new boolean[N];
        String note = "Press A with nothing in front of any sensor to capture the empty baseline.";

        int missing = 0;
        for (int i = 0; i < N; i++) if (color[i] == null) missing++;
        telemetry.addLine("Color Sensors Test. " + (missing == 0 ? "All four sensors found." : missing + " sensor(s) NOT FOUND (see the list once started)."));
        telemetry.update();
        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            if (gamepad1.bWasPressed()) {
                led = !led;
                setLeds(color, led);
            }
            if (gamepad1.backWasPressed()) {
                Arrays.fill(haveBase, false);
                note = "baseline cleared";
            }
            if (gamepad1.aWasPressed() && !capturing) {
                capturing = true;
                captureTimer.reset();
                Arrays.fill(sumAlpha, 0.0);
                Arrays.fill(sumDist, 0.0);
                Arrays.fill(sumCount, 0);
                note = "capturing the empty baseline: keep everything away from the sensors";
            }

            int present = 0, pollen = 0;
            boolean logNow = logTimer.milliseconds() > 1000.0;
            StringBuilder logLine = new StringBuilder();

            for (int i = 0; i < N; i++) {
                if (color[i] == null) {
                    telemetry.addData(NAMES[i], "NOT FOUND in the hub configuration (or not a color sensor)");
                    continue;
                }
                long t0 = System.nanoTime();
                NormalizedRGBA c = color[i].getNormalizedColors();
                double d = distance[i] != null ? distance[i].getDistance(DistanceUnit.CM) : Double.NaN;
                double readMs = (System.nanoTime() - t0) / 1e6;

                double sum = c.red + c.green + c.blue + 1e-9;
                double rs = c.red / sum, gs = c.green / sum, bs = c.blue / sum;

                if (capturing) {
                    sumAlpha[i] += c.alpha;
                    sumDist[i] += d;
                    sumCount[i]++;
                }

                String state = "no baseline";
                boolean isPresent = false, isPollen = false;
                if (haveBase[i]) {
                    boolean brighter = c.alpha >= baseAlpha[i] * ALPHA_RATIO && c.alpha - baseAlpha[i] >= ALPHA_MIN_RISE;
                    boolean nearer = !Double.isNaN(d) && d <= baseDist[i] - DIST_DROP_CM;
                    isPresent = brighter || nearer;
                    isPollen = isPresent && gs >= YELLOW_MIN_GREEN_SHARE && bs <= YELLOW_MAX_BLUE_SHARE;
                    state = !isPresent ? "empty" : isPollen ? "PRESENT: yellow POLLEN" : "PRESENT: other color";
                }
                if (isPresent) present++;
                if (isPollen) pollen++;

                if (isPresent != wasPresent[i] || isPollen != wasPollen[i]) {
                    RobotLog.ii(LOG_TAG, String.format(Locale.US, "%s %s: dist %.2f cm, alpha %.3f, rgb %.4f/%.4f/%.4f, shares %.2f/%.2f/%.2f",
                            NAMES[i], state, d, c.alpha, c.red, c.green, c.blue, rs, gs, bs));
                    wasPresent[i] = isPresent;
                    wasPollen[i] = isPollen;
                }
                if (logNow) {
                    logLine.append(String.format(Locale.US, "%s[%s d%.1f a%.3f s%.2f/%.2f/%.2f] ", NAMES[i],
                            !haveBase[i] ? "?" : !isPresent ? "-" : isPollen ? "POLLEN" : "OBJ", d, c.alpha, rs, gs, bs));
                }

                telemetry.addData(NAMES[i], "%s", state);
                telemetry.addData("   " + NAMES[i] + " readings",
                        "dist %.2f cm%s | alpha %.3f%s | rgb %.4f / %.4f / %.4f | shares %.2f / %.2f / %.2f | read %.1f ms",
                        d, haveBase[i] ? String.format(Locale.US, " (empty %.2f)", baseDist[i]) : "",
                        c.alpha, haveBase[i] ? String.format(Locale.US, " (empty %.3f)", baseAlpha[i]) : "",
                        c.red, c.green, c.blue, rs, gs, bs, readMs);
            }

            if (capturing && captureTimer.seconds() >= CAPTURE_SEC) {
                capturing = false;
                for (int i = 0; i < N; i++) {
                    if (color[i] != null && sumCount[i] > 0) {
                        baseAlpha[i] = sumAlpha[i] / sumCount[i];
                        baseDist[i] = sumDist[i] / sumCount[i];
                        haveBase[i] = true;
                    }
                }
                note = "baseline captured";
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "BASELINE alpha %.3f / %.3f / %.3f / %.3f, dist %.2f / %.2f / %.2f / %.2f",
                        baseAlpha[0], baseAlpha[1], baseAlpha[2], baseAlpha[3], baseDist[0], baseDist[1], baseDist[2], baseDist[3]));
            }

            telemetry.addData("ELEMENTS SEEN", "%d of %d sensors present, %d look like yellow pollen", present, N, pollen);
            telemetry.addData("Note", note);
            telemetry.addData("LEDs / loop", "%s / %.1f ms (four sensors read every loop; a real robot would read one per loop)", led ? "on" : "off", loopMs);
            telemetry.addLine("A baseline (all empty), B LEDs, Back clear. Thresholds are in the Dashboard (ColorSensorsTest).");
            telemetry.update();

            if (logNow) {
                logTimer.reset();
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "present %d pollen %d loop %.1f ms | %s", present, pollen, loopMs, logLine));
            }
        }
        setLeds(color, false);
    }

    private static void setLeds(NormalizedColorSensor[] sensors, boolean on) {
        for (NormalizedColorSensor s : sensors) {
            if (s instanceof ColorSensor) ((ColorSensor) s).enableLed(on);
        }
    }
}
