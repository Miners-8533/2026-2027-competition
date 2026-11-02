package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;

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
 *   RED NECTAR   red share high and not below green, little blue            (measured about 0.36-0.47 / 0.32-0.36 / 0.21-0.27)
 *   BLUE NECTAR  blue share high and well above green, little red           (measured about 0.11-0.17 / 0.24-0.37 / 0.45-0.64)
 *   PUSHER ARM     CS4 only: far (5.5 cm or more), bluish with green close to blue (measured about 0.23 / 0.38 / 0.39, 6.5 cm)
 *   OTHER        anything else (partial views at an edge, shadows)
 * The nectar identities (red and blue alliance nectar) were confirmed by the team. Presence uses the shift in the
 * color shares from the empty baseline first (brightness and distance contrast against a near wall is small), then the
 * alpha ratio and the nearer distance, with hysteresis (separate on / off thresholds) and a debounce, so a pollen near a
 * threshold no longer flickers. All thresholds are in the FTC Dashboard.
 *
 * Layout: the sensors are numbered along the track, intake -> CS1 -> CS2 -> CS3 -> CS4 -> flywheel (CS4 is deepest, nearest
 * the flywheel and the pusher arm; CS1 is nearest the intake). The test shows the pattern as a STACK line, how many are in a
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
 * LABELED TEST (to score the classifier): say what is really in front of each sensor, and it compares every reading
 * with that label. The labels are NOTHING, POLLEN, RED NECTAR, BLUE NECTAR, PUSHER ARM, OTHER; every sensor starts at NOTHING,
 * so only the sensor you are testing needs a label.
 *   D-pad up / down      select a sensor (shown with a > on the LABELS line)
 *   D-pad left / right   set the selected sensor's label
 *   Left bumper          set every label to NOTHING
 *   Y                    start / stop RECORDING: while recording, each reading is scored against its sensor's label, the
 *                        log is written twice a second with the labels, and the confusion table is logged every
 *                        SNAPSHOT_SEC and when recording stops
 *   Right bumper         clear the confusion counts
 * Set the label AFTER the item is in place and back to NOTHING before it leaves: the debounce makes the classifier lag a
 * moment, and a wrong label scores a correct answer as an error.
 *
 * Needs four REV Color Sensor V3 named CS1, CS2, CS3, CS4, each on its own I2C bus (they share an address). Uses the SDK
 * default sensor rate (100 ms refresh), where the distance reading is calibrated. Everything is also written to the robot
 * log (tag ColorSensors): once a second, and at once whenever a sensor's PRESENT state or class changes.
 */
@Config
@Disabled   // hidden from the Driver Station; remove this line and the import to use it again
@TeleOp(name = "Color Sensors Test (CS1-CS4)", group = "Bring-up")
public class ColorSensorsTest extends LinearOpMode {

    public static double CAPTURE_SEC = 1.0;
    public static boolean READ_ONE_PER_LOOP = true;
    // The classification thresholds, the gain, the debounce and the presence rules live in ElementCounter (also in the
    // Dashboard), so this test scores exactly what the robot uses. For the labeled test the pusher arm may be held in
    // front of any sensor, so this test sets ElementCounter.ARM_ANY_SENSOR on (and ARM_REQUIRE_DISTANCE off) while it
    // runs and puts them back when it ends.
    /** While recording, the confusion table is also written to the log this often (seconds). */
    public static double SNAPSHOT_SEC = 20.0;
    /** Readings are not scored for this long after a sensor's label changes (the classifier lags by its debounce). */
    public static double LABEL_SETTLE_MS = 700.0;

    /** What the team says is in front of a sensor (the ground truth), and what the classifier can answer (index 0 = empty / nothing). */
    private static final String[] CLASSES = {"NOTHING", "POLLEN", "RED NECTAR", "BLUE NECTAR", "PUSHER ARM", "OTHER"};
    private static final String[] PREDICTIONS = {"EMPTY", "POLLEN", "RED NECTAR", "BLUE NECTAR", "PUSHER ARM", "OTHER"};

    private static final String[] NAMES = {"CS1", "CS2", "CS3", "CS4"};
    private static final String LOG_TAG = "ColorSensors";
    private static final int N = NAMES.length;

    /** The class name the test shows and scores, from ElementCounter's classifier. */
    static String classify(int sensor, double red, double green, double blue, double distCm) {
        switch (ElementCounter.classify(sensor, red, green, blue, distCm)) {
            case POLLEN: return "POLLEN";
            case RED_NECTAR: return "RED NECTAR";
            case BLUE_NECTAR: return "BLUE NECTAR";
            case PUSHER_ARM: return "PUSHER ARM";
            default: return "OTHER";
        }
    }

    /** Index into PREDICTIONS (0 = nothing seen). */
    private static int predictionIndex(boolean present, String cls) {
        if (!present) return 0;
        switch (cls) {
            case "POLLEN": return 1;
            case "RED NECTAR": return 2;
            case "BLUE NECTAR": return 3;
            case "PUSHER ARM": return 4;
            default: return 5;
        }
    }

    /** One line per truth class with samples: how the classifier answered, as percentages, over the sensors in {@code sensors}. */
    private static String confusionLine(int[][][] counts, int truthClass, int[] sensors) {
        int total = 0;
        int[] byPrediction = new int[PREDICTIONS.length];
        for (int s : sensors) {
            for (int p = 0; p < PREDICTIONS.length; p++) {
                byPrediction[p] += counts[s][truthClass][p];
                total += counts[s][truthClass][p];
            }
        }
        if (total == 0) return null;
        StringBuilder sb = new StringBuilder(String.format(Locale.US, "%d samples -> ", total));
        for (int p = 0; p < PREDICTIONS.length; p++) {
            if (byPrediction[p] > 0) sb.append(String.format(Locale.US, "%s %.0f%%  ", PREDICTIONS[p], 100.0 * byPrediction[p] / total));
        }
        return sb.toString();
    }

    private static void logConfusion(int[][][] counts, String reason) {
        RobotLog.ii(LOG_TAG, "CONFUSION (" + reason + "): truth -> what the classifier said");
        int[] all = {0, 1, 2, 3};
        for (int t = 0; t < CLASSES.length; t++) {
            String line = confusionLine(counts, t, all);
            if (line != null) RobotLog.ii(LOG_TAG, "CONFUSION   " + CLASSES[t] + " (all sensors): " + line);
            for (int s = 0; s < N; s++) {
                String per = confusionLine(counts, t, new int[]{s});
                if (per != null) RobotLog.ii(LOG_TAG, "CONFUSION     " + CLASSES[t] + " on " + NAMES[s] + ": " + per);
            }
        }
    }

    private static String symbolFor(String cls) {
        switch (cls) {
            case "POLLEN": return "P";
            case "RED NECTAR": return "R";
            case "BLUE NECTAR": return "B";
            case "PUSHER ARM": return "A";
            default: return "O";
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElementCounter.ARM_ANY_SENSOR = true;
        ElementCounter.ARM_REQUIRE_DISTANCE = false;
        NormalizedColorSensor[] color = new NormalizedColorSensor[N];
        DistanceSensor[] distance = new DistanceSensor[N];
        for (int i = 0; i < N; i++) {
            color[i] = hardwareMap.tryGet(NormalizedColorSensor.class, NAMES[i]);
            distance[i] = hardwareMap.tryGet(DistanceSensor.class, NAMES[i]);
            if (color[i] != null) color[i].setGain((float) ElementCounter.GAIN);
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

        // labeled test: what the team says is in front of each sensor, and how the classifier did against it
        int[] truth = new int[N];                      // index into CLASSES, per sensor; default NOTHING
        long[] labelSince = new long[N];               // when each label last changed
        int selected = 0;
        boolean recording = false;
        int[][][] counts = new int[N][CLASSES.length][PREDICTIONS.length];
        ElapsedTime snapshotTimer = new ElapsedTime();

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
            if (gamepad1.dpadUpWasPressed()) selected = (selected + N - 1) % N;
            if (gamepad1.dpadDownWasPressed()) selected = (selected + 1) % N;
            if (gamepad1.dpadRightWasPressed()) {
                truth[selected] = (truth[selected] + 1) % CLASSES.length;
                labelSince[selected] = now;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                truth[selected] = (truth[selected] + CLASSES.length - 1) % CLASSES.length;
                labelSince[selected] = now;
            }
            if (gamepad1.leftBumperWasPressed()) {
                Arrays.fill(truth, 0);
                Arrays.fill(labelSince, now);
            }
            if (gamepad1.rightBumperWasPressed()) {
                for (int[][] perSensor : counts) for (int[] perTruth : perSensor) Arrays.fill(perTruth, 0);
                note = "confusion counts cleared";
            }
            if (gamepad1.yWasPressed()) {
                recording = !recording;
                snapshotTimer.reset();
                if (recording) {
                    note = "RECORDING: samples are scored against the labels and the log is written twice a second";
                    RobotLog.ii(LOG_TAG, "RECORDING STARTED");
                } else {
                    note = "recording stopped";
                    logConfusion(counts, "recording stopped");
                }
            }
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
            boolean logNow = logTimer.milliseconds() > (recording ? 500.0 : 1000.0);
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
                    double drop = Double.isNaN(dist[i]) ? 0.0 : bDist[i] - dist[i];
                    boolean raw = ElementCounter.presenceNext(present[i], sr[i], sg[i], sb[i], alpha[i], dist[i],
                            bR[i], bG[i], bB[i], bAlpha[i], bDist[i]);
                    if (raw != candidate[i]) {
                        candidate[i] = raw;
                        candidateSince[i] = now;
                    }
                    if (candidate[i] != present[i] && (now - candidateSince[i]) / 1e6 >= ElementCounter.DEBOUNCE_MS) present[i] = candidate[i];
                    cls[i] = present[i] ? classify(i, sr[i], sg[i], sb[i], dist[i]) : "-";
                    state = present[i] ? "PRESENT: " + cls[i] : "empty";
                    if (recording && readNow[i] && (now - labelSince[i]) / 1e6 >= LABEL_SETTLE_MS) {
                        counts[i][truth[i]][predictionIndex(present[i], cls[i])]++;
                    }
                    if (present[i]) {
                        presentCount++;
                        switch (cls[i]) {
                            case "POLLEN": pollenCount++; break;
                            case "RED NECTAR": redCount++; break;
                            case "BLUE NECTAR": blueCount++; break;
                            case "PUSHER ARM": armCount++; break;
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
                    logLine.append(String.format(Locale.US, "%s[truth %s, said %s, d%.1f a%.3f s%.2f/%.2f/%.2f] ", NAMES[i], CLASSES[truth[i]],
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
                    armCount > 0 ? "   [CS4 sees the pusher arm: not counted]" : "");

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
                boolean element = haveBase[i] && present[i] && !"PUSHER ARM".equals(cls[i]);
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
            // the labels, and how the classifier is scoring against them
            StringBuilder labels = new StringBuilder();
            for (int i = 0; i < N; i++) {
                labels.append(i == selected ? ">" : " ").append(NAMES[i]).append("=").append(CLASSES[truth[i]]).append("   ");
            }
            telemetry.addData("LABELS (what is really in front; > = selected)", labels.toString());
            telemetry.addData("Scoring", recording ? "RECORDING (Y stops and logs the table)" : "not recording (Y starts)");
            int[] allSensors = {0, 1, 2, 3};
            for (int t = 0; t < CLASSES.length; t++) {
                String line = confusionLine(counts, t, allSensors);
                if (line != null) telemetry.addData("   truth " + CLASSES[t], line);
            }
            telemetry.addData("Note", note);
            telemetry.addData("Read mode / LEDs / loop", "%s / %s / %.1f ms", READ_ONE_PER_LOOP ? "one sensor per loop" : "all four every loop", led ? "on" : "off", loopMs);
            telemetry.addLine("A baseline (all empty), B LEDs, X read mode, Back clear baseline | D-pad up/down pick a sensor, left/right set its label, LB all NOTHING, Y record, RB clear counts");
            telemetry.update();

            if (recording && snapshotTimer.seconds() >= SNAPSHOT_SEC) {
                snapshotTimer.reset();
                logConfusion(counts, "snapshot");
            }

            if (logNow) {
                logTimer.reset();
                RobotLog.ii(LOG_TAG, String.format(Locale.US, "elements %d (pollen %d red %d blue %d other %d arm %d) loop %.1f ms %s | %s", elements, pollenCount,
                        redCount, blueCount, otherCount, armCount, loopMs, READ_ONE_PER_LOOP ? "one-per-loop" : "all-per-loop", logLine));
            }
        }
        setLeds(color, false);
        ElementCounter.ARM_ANY_SENSOR = false;
        ElementCounter.ARM_REQUIRE_DISTANCE = true;
    }

    private static void setLeds(NormalizedColorSensor[] sensors, boolean on) {
        for (NormalizedColorSensor s : sensors) {
            if (s instanceof ColorSensor) ((ColorSensor) s).enableLed(on);
        }
    }
}
