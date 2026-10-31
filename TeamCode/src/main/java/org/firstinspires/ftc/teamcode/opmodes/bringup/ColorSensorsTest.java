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
 * Live readings of the four color sensors CS1 to CS4 (REV Color Sensor V3), so elements can be moved in front of them
 * to see how each reads. The eventual use is to tell when the robot holds 4 elements, and what they are.
 *
 * For each sensor: distance (cm), alpha (total light), red / green / blue, the color shares (each channel's part of the
 * color), the age of the reading, and, once an EMPTY baseline is captured, whether something is PRESENT and what it
 * looks like:
 *   POLLEN       green clearly above blue (green/blue ratio), modest red   (measured about 0.28-0.31 / 0.44-0.51 / 0.18-0.27)
 *   RED NECTAR   red share high and above green                             (measured about 0.40-0.47 / 0.32-0.36 / 0.21-0.24)
 *   BLUE NECTAR  blue share high and above green                            (measured about 0.15 / 0.29 / 0.56)
 *   GATE ARM     CS4 only: bluish, far (5.5 cm or more)                     (measured about 0.23 / 0.38 / 0.39, 6.5 cm)
 *   OTHER        anything else
 * The nectar identities are an inference (the two non-yellow colors seen); confirm them. Presence uses the shift in the
 * color shares from the empty baseline first (brightness and distance contrast against a near wall is small), then the
 * alpha ratio and the nearer distance, with hysteresis (separate on / off thresholds) and a debounce, so a pollen near a
 * threshold no longer flickers. All thresholds are in the FTC Dashboard.
 *
 * Layout: the sensors are numbered along the track, intake -> CS1 -> CS2 -> CS3 -> CS4 -> flywheel (CS4 is deepest, nearest
 * the flywheel and the gate arm; CS1 is nearest the intake). The test shows the pattern as a STACK line, how many are in a
 * row from the flywheel end, and the element at the deepest slot (it fires first). The fill order (from the flywheel end)
 * is an assumption: a GAP warning means an element is seen without a neighbor toward the flywheel.
 *
 * Reading: each sensor read is two blocking I2C transactions (about 13 ms), and the sensors only refresh every 100 ms,
 * so by default ONE sensor is read per loop, cycling: the loop stays short and each sensor is still read every 4 loops.
 *
 *   A         capture the EMPTY baseline for all four (nothing in front of any sensor; holds still for CAPTURE_SEC)
 *   B         sensor LEDs on / off (all four)
 *   X         read mode: one sensor per loop (default) / all four every loop (to compare the loop time)
 *   Back      clear the baseline
 *
 * Needs four REV Color Sensor V3 named CS1, CS2, CS3, CS4, each on its own I2C bus (they share an address). Uses the SDK
 * default sensor rate (100 ms refresh), where the distance reading is calibrated. Everything is also written to the robot
 * log (tag ColorSensors): once a second, and at once whenever a sensor's PRESENT state or class changes.
 */
@Config
@TeleOp(name = "Color Sensors Test (CS1-CS4)", group = "Bring-up")
public class ColorSensorsTest extends LinearOpMode {

    /** Software gain: a multiplier clipped at 1.0, so keep it at 1.0. */
    public static double GAIN = 1.0;
    public static double CAPTURE_SEC = 1.0;
    public static boolean READ_ONE_PER_LOOP = true;

    /** Presence turns ON when the color shares move this far from the empty baseline (distance in share space) ... */
    public static double COLOR_SHIFT_ON = 0.05;
    /** ... or alpha is this many times the empty level (and this much above it) ... */
    public static double ALPHA_RATIO_ON = 2.0;
    public static double ALPHA_MIN_RISE = 0.02;
    /** ... or the object is this much nearer than the empty distance. It turns OFF only when all three are below the OFF values. */
    public static double DIST_DROP_ON_CM = 1.5;
    public static double COLOR_SHIFT_OFF = 0.03;
    public static double ALPHA_RATIO_OFF = 1.6;
    public static double DIST_DROP_OFF_CM = 0.8;
    /** A new state must persist this long before it is reported. */
    public static double DEBOUNCE_MS = 150.0;

    public static double POLLEN_MIN_GREEN_OVER_BLUE = 1.65;
    public static double POLLEN_MAX_RED = 0.35;
    public static double RED_NECTAR_MIN_RED = 0.38;
    public static double BLUE_NECTAR_MIN_BLUE = 0.48;
    public static double ARM_MIN_DIST_CM = 5.5;
    public static double ARM_MIN_BLUE = 0.36;
    public static double ARM_MAX_RED = 0.28;

    private static final String[] NAMES = {"CS1", "CS2", "CS3", "CS4"};
    private static final String LOG_TAG = "ColorSensors";
    private static final int N = NAMES.length;
    private static final int ARM_SENSOR = 3;   // CS4 is the one that also looks at the gate arm

    /** The class of a present object, from its color shares and distance. */
    static String classify(int sensor, double red, double green, double blue, double distCm) {
        if (sensor == ARM_SENSOR && distCm >= ARM_MIN_DIST_CM && blue >= ARM_MIN_BLUE && red <= ARM_MAX_RED) return "GATE ARM";
        if (red >= RED_NECTAR_MIN_RED && red > green) return "RED NECTAR";
        if (blue >= BLUE_NECTAR_MIN_BLUE && blue > green) return "BLUE NECTAR";
        if (green >= POLLEN_MIN_GREEN_OVER_BLUE * blue && red <= POLLEN_MAX_RED) return "POLLEN";
        return "OTHER";
    }

    private static String symbolFor(String cls) {
        switch (cls) {
            case "POLLEN": return "P";
            case "RED NECTAR": return "R";
            case "BLUE NECTAR": return "B";
            case "GATE ARM": return "A";
            default: return "O";
        }
    }

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

        // latest reading per sensor
        double[] alpha = new double[N], dist = new double[N], sr = new double[N], sg = new double[N], sb = new double[N];
        double[] red = new double[N], green = new double[N], blue = new double[N], readMs = new double[N];
        long[] readAt = new long[N];
        boolean[] haveReading = new boolean[N];
        // empty baseline
        double[] bAlpha = new double[N], bDist = new double[N], bR = new double[N], bG = new double[N], bB = new double[N];
        boolean[] haveBase = new boolean[N];
        double[] sumAlpha = new double[N], sumDist = new double[N], sumR = new double[N], sumG = new double[N], sumB = new double[N];
        int[] sumCount = new int[N];
        // state with hysteresis and debounce
        boolean[] present = new boolean[N], candidate = new boolean[N];
        long[] candidateSince = new long[N];
        String[] cls = new String[N];
        String[] lastLoggedState = new String[N];
        Arrays.fill(cls, "-");
        Arrays.fill(lastLoggedState, "");

        boolean capturing = false;
        ElapsedTime captureTimer = new ElapsedTime();
        ElapsedTime logTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        int nextSensor = 0;
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
            long now = System.nanoTime();

            if (gamepad1.bWasPressed()) {
                led = !led;
                setLeds(color, led);
            }
            if (gamepad1.xWasPressed()) READ_ONE_PER_LOOP = !READ_ONE_PER_LOOP;
            if (gamepad1.backWasPressed()) {
                Arrays.fill(haveBase, false);
                Arrays.fill(present, false);
                Arrays.fill(candidate, false);
                Arrays.fill(cls, "-");
                note = "baseline cleared";
            }
            if (gamepad1.aWasPressed() && !capturing) {
                capturing = true;
                captureTimer.reset();
                Arrays.fill(sumAlpha, 0.0);
                Arrays.fill(sumDist, 0.0);
                Arrays.fill(sumR, 0.0);
                Arrays.fill(sumG, 0.0);
                Arrays.fill(sumB, 0.0);
                Arrays.fill(sumCount, 0);
                note = "capturing the empty baseline: keep everything away from the sensors";
            }

            // which sensors to read this loop
            boolean[] readNow = new boolean[N];
            if (READ_ONE_PER_LOOP) {
                for (int k = 0; k < N; k++) {
                    int i = (nextSensor + k) % N;
                    if (color[i] != null) {
                        readNow[i] = true;
                        nextSensor = (i + 1) % N;
                        break;
                    }
                }
            } else {
                for (int i = 0; i < N; i++) readNow[i] = color[i] != null;
            }

            for (int i = 0; i < N; i++) {
                if (!readNow[i]) continue;
                long t0 = System.nanoTime();
                NormalizedRGBA c = color[i].getNormalizedColors();
                double d = distance[i] != null ? distance[i].getDistance(DistanceUnit.CM) : Double.NaN;
                readMs[i] = (System.nanoTime() - t0) / 1e6;
                red[i] = c.red;
                green[i] = c.green;
                blue[i] = c.blue;
                alpha[i] = c.alpha;
                dist[i] = d;
                double sum = c.red + c.green + c.blue + 1e-9;
                sr[i] = c.red / sum;
                sg[i] = c.green / sum;
                sb[i] = c.blue / sum;
                readAt[i] = System.nanoTime();
                haveReading[i] = true;
                if (capturing) {
                    sumAlpha[i] += alpha[i];
                    sumDist[i] += d;
                    sumR[i] += sr[i];
                    sumG[i] += sg[i];
                    sumB[i] += sb[i];
                    sumCount[i]++;
                }
            }

            if (capturing && captureTimer.seconds() >= CAPTURE_SEC) {
                capturing = false;
                for (int i = 0; i < N; i++) {
                    if (color[i] != null && sumCount[i] > 0) {
                        bAlpha[i] = sumAlpha[i] / sumCount[i];
                        bDist[i] = sumDist[i] / sumCount[i];
                        bR[i] = sumR[i] / sumCount[i];
                        bG[i] = sumG[i] / sumCount[i];
                        bB[i] = sumB[i] / sumCount[i];
                        haveBase[i] = true;
                    }
                }
                note = "baseline captured";
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "BASELINE alpha %.3f / %.3f / %.3f / %.3f, dist %.2f / %.2f / %.2f / %.2f, shares CS1 %.2f/%.2f/%.2f CS2 %.2f/%.2f/%.2f CS3 %.2f/%.2f/%.2f CS4 %.2f/%.2f/%.2f",
                        bAlpha[0], bAlpha[1], bAlpha[2], bAlpha[3], bDist[0], bDist[1], bDist[2], bDist[3],
                        bR[0], bG[0], bB[0], bR[1], bG[1], bB[1], bR[2], bG[2], bB[2], bR[3], bG[3], bB[3]));
            }

            // state, class and the live display
            int presentCount = 0, pollenCount = 0, redCount = 0, blueCount = 0, otherCount = 0, armCount = 0;
            boolean logNow = logTimer.milliseconds() > 1000.0;
            StringBuilder logLine = new StringBuilder();
            for (int i = 0; i < N; i++) {
                if (color[i] == null) {
                    telemetry.addData(NAMES[i], "NOT FOUND in the hub configuration (or not a color sensor)");
                    continue;
                }
                if (!haveReading[i]) {
                    telemetry.addData(NAMES[i], "waiting for the first reading");
                    continue;
                }
                String state = "no baseline";
                if (haveBase[i]) {
                    double shift = Math.sqrt(Math.pow(sr[i] - bR[i], 2) + Math.pow(sg[i] - bG[i], 2) + Math.pow(sb[i] - bB[i], 2));
                    double ratio = alpha[i] / Math.max(bAlpha[i], 1e-6);
                    double rise = alpha[i] - bAlpha[i];
                    double drop = Double.isNaN(dist[i]) ? 0.0 : bDist[i] - dist[i];
                    boolean on = shift >= COLOR_SHIFT_ON || (ratio >= ALPHA_RATIO_ON && rise >= ALPHA_MIN_RISE) || drop >= DIST_DROP_ON_CM;
                    boolean off = shift < COLOR_SHIFT_OFF && ratio < ALPHA_RATIO_OFF && drop < DIST_DROP_OFF_CM;
                    boolean raw = present[i] ? !off : on;
                    if (raw != candidate[i]) {
                        candidate[i] = raw;
                        candidateSince[i] = now;
                    }
                    if (candidate[i] != present[i] && (now - candidateSince[i]) / 1e6 >= DEBOUNCE_MS) present[i] = candidate[i];
                    cls[i] = present[i] ? classify(i, sr[i], sg[i], sb[i], dist[i]) : "-";
                    state = present[i] ? "PRESENT: " + cls[i] : "empty";
                    if (present[i]) {
                        presentCount++;
                        switch (cls[i]) {
                            case "POLLEN": pollenCount++; break;
                            case "RED NECTAR": redCount++; break;
                            case "BLUE NECTAR": blueCount++; break;
                            case "GATE ARM": armCount++; break;
                            default: otherCount++; break;
                        }
                    }
                    if (!state.equals(lastLoggedState[i])) {
                        RobotLog.ii(LOG_TAG, String.format(Locale.US, "%s %s: dist %.2f cm, alpha %.3f, rgb %.4f/%.4f/%.4f, shares %.2f/%.2f/%.2f (shift %.3f, alpha x%.2f, nearer %.2f cm)",
                                NAMES[i], state, dist[i], alpha[i], red[i], green[i], blue[i], sr[i], sg[i], sb[i], shift, ratio, drop));
                        lastLoggedState[i] = state;
                    }
                }
                if (logNow) {
                    logLine.append(String.format(Locale.US, "%s[%s d%.1f a%.3f s%.2f/%.2f/%.2f] ", NAMES[i],
                            !haveBase[i] ? "?" : present[i] ? cls[i] : "-", dist[i], alpha[i], sr[i], sg[i], sb[i]));
                }

                telemetry.addData(NAMES[i], "%s", state);
                telemetry.addData("   " + NAMES[i] + " readings",
                        "dist %.2f cm%s | alpha %.3f%s | rgb %.4f / %.4f / %.4f | shares %.2f / %.2f / %.2f | read %.1f ms, %.0f ms old",
                        dist[i], haveBase[i] ? String.format(Locale.US, " (empty %.2f)", bDist[i]) : "",
                        alpha[i], haveBase[i] ? String.format(Locale.US, " (empty %.3f)", bAlpha[i]) : "",
                        red[i], green[i], blue[i], sr[i], sg[i], sb[i], readMs[i], (System.nanoTime() - readAt[i]) / 1e6);
            }

            int elements = presentCount - armCount;
            telemetry.addData("ELEMENTS SEEN", "%d  (pollen %d, red nectar %d, blue nectar %d, other %d)%s", elements, pollenCount, redCount, blueCount, otherCount,
                    armCount > 0 ? "   [CS4 sees the gate arm: not counted]" : "");

            // The track runs intake -> CS1 -> CS2 -> CS3 -> CS4 -> flywheel, and the stack is expected to fill from the
            // flywheel end. Show the pattern, the contiguous run from CS4, and the element at the deepest slot (it fires first).
            StringBuilder stack = new StringBuilder("intake > ");
            int contiguous = 0;
            boolean unbroken = true;
            String head = "-";
            for (int i = 0; i < N; i++) {
                String symbol = color[i] == null ? "?" : !haveBase[i] ? "_" : !present[i] ? "." : symbolFor(cls[i]);
                stack.append("[").append(symbol).append("] ");
            }
            stack.append("> flywheel");
            for (int i = N - 1; i >= 0; i--) {
                boolean element = haveBase[i] && present[i] && !"GATE ARM".equals(cls[i]);
                if (unbroken && element) {
                    contiguous++;
                    if (head.equals("-")) head = cls[i];
                } else {
                    unbroken = false;
                }
            }
            telemetry.addData("STACK (. empty, P pollen, R red nectar, B blue nectar, O other, A arm)", stack.toString());
            telemetry.addData("From the flywheel end", "%d in a row (CS4 back toward CS1); next out: %s%s", contiguous, head,
                    elements != contiguous ? String.format(Locale.US, "   GAP: %d element(s) seen but only %d in a row (still moving, or a misread)", elements, contiguous) : "");
            telemetry.addData("Note", note);
            telemetry.addData("Read mode / LEDs / loop", "%s / %s / %.1f ms", READ_ONE_PER_LOOP ? "one sensor per loop" : "all four every loop", led ? "on" : "off", loopMs);
            telemetry.addLine("A baseline (all empty), B LEDs, X read mode, Back clear. Thresholds are in the Dashboard (ColorSensorsTest).");
            telemetry.update();

            if (logNow) {
                logTimer.reset();
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "elements %d (pollen %d red %d blue %d other %d arm %d) loop %.1f ms %s | %s", elements, pollenCount,
                        redCount, blueCount, otherCount, armCount, loopMs, READ_ONE_PER_LOOP ? "one-per-loop" : "all-per-loop", logLine));
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
