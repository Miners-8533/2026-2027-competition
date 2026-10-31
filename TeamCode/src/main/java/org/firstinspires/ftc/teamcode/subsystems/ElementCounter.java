package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * Counts and identifies the elements in the transport with four REV Color Sensor V3 named CS1 to CS4, numbered along the
 * track: intake -> CS1 -> CS2 -> CS3 -> CS4 -> flywheel (CS4 is deepest, nearest the flywheel and the gate arm). The count
 * is the number of sensors that see an element. The stack fills from the CS4 end, so the run of elements in a row from
 * CS4 is a check: an element seen without a neighbor toward the flywheel is still moving (or a misread) and sets
 * {@link #hasGap()}, but it still counts as held.
 *
 * Each sensor's slot is EMPTY, POLLEN, RED_NECTAR, BLUE_NECTAR, OTHER, or GATE_ARM (CS4 sees the arm; it is not an
 * element). Presence is decided by how far the color shares move from the sensor's own EMPTY readings (the empty views
 * differ from sensor to sensor and brightness contrast against a near wall is small), then the alpha ratio and a nearer
 * distance, with separate on and off thresholds and a debounce so a pollen near a threshold does not flicker. The kind
 * comes from the color shares. The thresholds were measured on the robot (see ColorSensorsTest, which scores them against
 * labels and shares these same settings) and are in the FTC Dashboard.
 *
 * Reading: each sensor read is two blocking I2C transactions (about 13 ms) and the sensors only refresh every 100 ms, so
 * {@link #update()} reads ONE sensor every READ_PERIOD_MS, cycling (about 4 x READ_PERIOD_MS for a full refresh), and does
 * nothing in between. The reported count changes only after the raw count has been steady (COUNT_RISE_STABLE_MS when it
 * goes up, so a full robot is noticed quickly; COUNT_FALL_STABLE_MS when it goes down), because the end sensor flickers
 * for about a second while an element moves. A fourth element is reported about 0.4 s after it is first seen. If a sensor has not been read for STALE_MS the
 * counter is not {@link #isValid() valid}, and callers should not act on it.
 *
 * The EMPTY baseline is the table below (measured 2026-10-31 with the track empty). Lighting changes the empty readings,
 * so re-measure it in the pit with ColorSensorsTest (A, with the track empty) and copy the BASELINE line into the table.
 * It cannot be captured automatically at the start of a match, because the robot may already hold elements.
 */
@Config
public class ElementCounter {

    public enum Slot { EMPTY, POLLEN, RED_NECTAR, BLUE_NECTAR, OTHER, GATE_ARM }

    /** Software gain: a multiplier clipped at 1.0 (no signal is added), so keep it 1.0. */
    public static double GAIN = 1.0;
    public static double READ_PERIOD_MS = 40.0;
    public static double STALE_MS = 1000.0;
    public static double COUNT_RISE_STABLE_MS = 150.0;
    public static double COUNT_FALL_STABLE_MS = 300.0;
    /** The most elements the robot may hold (rule G407). */
    public static int CAPACITY = 4;

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
    /** A new presence state must persist this long before it is reported. */
    public static double DEBOUNCE_MS = 150.0;

    /** Pollen: green at least this many times blue (a dim or far pollen reads about 1.5; the empty views read 1.25-1.45), modest red. */
    public static double POLLEN_MIN_GREEN_OVER_BLUE = 1.5;
    public static double POLLEN_MAX_RED = 0.35;
    /** Red nectar: red share at least this (0.36-0.38 when it sits further from the sensor, 0.40-0.47 close), red not below green, little blue. */
    public static double RED_NECTAR_MIN_RED = 0.35;
    public static double RED_NECTAR_MAX_BLUE = 0.30;
    /** Blue nectar: blue share at least this, and at least BLUE_NECTAR_MIN_BLUE_OVER_GREEN above the green share, with little red. */
    public static double BLUE_NECTAR_MIN_BLUE = 0.44;
    public static double BLUE_NECTAR_MIN_BLUE_OVER_GREEN = 0.08;
    public static double BLUE_NECTAR_MAX_RED = 0.25;
    /** Gate arm: bluish with GREEN CLOSE TO BLUE (about 0.23 / 0.38 / 0.39); a dim blue nectar has blue well above green. */
    public static double ARM_MIN_BLUE = 0.36;
    public static double ARM_MAX_RED = 0.28;
    public static double ARM_MAX_GREEN_BLUE_GAP = 0.06;
    /** The arm is only expected at CS4; ColorSensorsTest turns this on to hold it in front of any sensor. */
    public static boolean ARM_ANY_SENSOR = false;
    public static boolean ARM_REQUIRE_DISTANCE = true;
    public static double ARM_MIN_DIST_CM = 5.5;

    public static final String[] NAMES = {"CS1", "CS2", "CS3", "CS4"};
    public static final int N = NAMES.length;
    public static final int ARM_SENSOR = 3;   // CS4

    /** Empty-track readings, measured 2026-10-31: alpha, distance (cm), and the red / green / blue shares, per sensor. */
    private static final double[] BASE_ALPHA = {0.077, 0.080, 0.173, 0.038};
    private static final double[] BASE_DIST = {3.68, 5.23, 4.11, 7.87};
    private static final double[][] BASE_SHARES = {{0.32, 0.38, 0.30}, {0.29, 0.42, 0.29}, {0.28, 0.40, 0.32}, {0.30, 0.39, 0.31}};

    private final NormalizedColorSensor[] color = new NormalizedColorSensor[N];
    private final DistanceSensor[] distance = new DistanceSensor[N];
    private final boolean[] present = new boolean[N];
    private final boolean[] candidate = new boolean[N];
    private final long[] candidateSinceNs = new long[N];
    private final Slot[] slot = new Slot[N];
    private final long[] readAtNs = new long[N];
    private final double[] lastAlpha = new double[N];
    private final double[] lastDist = new double[N];
    private final double[][] lastShares = new double[N][3];
    private int nextSensor = 0;
    private long lastReadNs = 0L;
    private int stableCount = 0;
    private int rawCount = 0;
    private long rawSinceNs = 0L;

    private ElementCounter(HardwareMap hardwareMap) {
        for (int i = 0; i < N; i++) {
            color[i] = hardwareMap.get(NormalizedColorSensor.class, NAMES[i]);
            distance[i] = hardwareMap.tryGet(DistanceSensor.class, NAMES[i]);
            color[i].setGain((float) GAIN);
            if (color[i] instanceof ColorSensor) ((ColorSensor) color[i]).enableLed(true);
        }
        Arrays.fill(slot, Slot.EMPTY);
    }

    /** The counter if all four sensors CS1 to CS4 are configured, else null (a robot without them still runs). */
    public static ElementCounter tryCreate(HardwareMap hardwareMap) {
        for (String name : NAMES) {
            if (hardwareMap.tryGet(NormalizedColorSensor.class, name) == null) return null;
        }
        return new ElementCounter(hardwareMap);
    }

    // ---- classification, shared with ColorSensorsTest so the two cannot drift apart ----

    /** Whether something is present at a sensor, given its readings, its empty baseline, and whether it was present. Thresholds with hysteresis, no debounce. */
    public static boolean presenceNext(boolean wasPresent, double shareR, double shareG, double shareB, double alpha, double dist,
                                       double baseR, double baseG, double baseB, double baseAlpha, double baseDist) {
        double shift = Math.sqrt(Math.pow(shareR - baseR, 2) + Math.pow(shareG - baseG, 2) + Math.pow(shareB - baseB, 2));
        double ratio = alpha / Math.max(baseAlpha, 1e-6);
        double rise = alpha - baseAlpha;
        double drop = Double.isNaN(dist) ? 0.0 : baseDist - dist;
        boolean on = shift >= COLOR_SHIFT_ON || (ratio >= ALPHA_RATIO_ON && rise >= ALPHA_MIN_RISE) || drop >= DIST_DROP_ON_CM;
        boolean off = shift < COLOR_SHIFT_OFF && ratio < ALPHA_RATIO_OFF && drop < DIST_DROP_OFF_CM;
        return wasPresent ? !off : on;
    }

    /** What a PRESENT object is, from its color shares (and distance, for the arm). */
    public static Slot classify(int sensor, double red, double green, double blue, double distCm) {
        if ((ARM_ANY_SENSOR || sensor == ARM_SENSOR) && (!ARM_REQUIRE_DISTANCE || distCm >= ARM_MIN_DIST_CM)
                && blue >= ARM_MIN_BLUE && red <= ARM_MAX_RED && Math.abs(green - blue) <= ARM_MAX_GREEN_BLUE_GAP) return Slot.GATE_ARM;
        if (red >= RED_NECTAR_MIN_RED && red >= green - 0.01 && blue < RED_NECTAR_MAX_BLUE) return Slot.RED_NECTAR;
        if (blue >= BLUE_NECTAR_MIN_BLUE && blue - green >= BLUE_NECTAR_MIN_BLUE_OVER_GREEN && red <= BLUE_NECTAR_MAX_RED) return Slot.BLUE_NECTAR;
        if (green >= POLLEN_MIN_GREEN_OVER_BLUE * blue && red <= POLLEN_MAX_RED) return Slot.POLLEN;
        return Slot.OTHER;
    }

    // ---- running ----

    /** Call once per loop. Reads at most one sensor, and only every READ_PERIOD_MS. */
    public void update() {
        long now = System.nanoTime();
        if ((now - lastReadNs) / 1e6 >= READ_PERIOD_MS) {
            lastReadNs = now;
            readSensor(nextSensor, now);
            nextSensor = (nextSensor + 1) % N;
        }
        // the count changes only after the raw count has been steady for COUNT_STABLE_MS
        int raw = elementsSeen();
        if (raw != rawCount) {
            rawCount = raw;
            rawSinceNs = now;
        }
        double steadyMs = rawCount > stableCount ? COUNT_RISE_STABLE_MS : COUNT_FALL_STABLE_MS;
        if (rawCount != stableCount && (now - rawSinceNs) / 1e6 >= steadyMs) stableCount = rawCount;
    }

    private void readSensor(int i, long now) {
        NormalizedRGBA c = color[i].getNormalizedColors();
        double d = distance[i] != null ? distance[i].getDistance(DistanceUnit.CM) : Double.NaN;
        double sum = c.red + c.green + c.blue + 1e-9;
        double shareR = c.red / sum, shareG = c.green / sum, shareB = c.blue / sum;
        lastAlpha[i] = c.alpha;
        lastDist[i] = d;
        lastShares[i][0] = shareR;
        lastShares[i][1] = shareG;
        lastShares[i][2] = shareB;
        readAtNs[i] = now;

        boolean raw = presenceNext(present[i], shareR, shareG, shareB, c.alpha, d,
                BASE_SHARES[i][0], BASE_SHARES[i][1], BASE_SHARES[i][2], BASE_ALPHA[i], BASE_DIST[i]);
        if (raw != candidate[i]) {
            candidate[i] = raw;
            candidateSinceNs[i] = now;
        }
        if (candidate[i] != present[i] && (now - candidateSinceNs[i]) / 1e6 >= DEBOUNCE_MS) present[i] = candidate[i];
        slot[i] = present[i] ? classify(i, shareR, shareG, shareB, d) : Slot.EMPTY;
    }

    private boolean isElement(int i) {
        return slot[i] != Slot.EMPTY && slot[i] != Slot.GATE_ARM;
    }

    /** Sensors that currently see an element (not the gate arm). */
    private int elementsSeen() {
        int n = 0;
        for (int i = 0; i < N; i++) if (isElement(i)) n++;
        return n;
    }

    /** Elements in a row from CS4 (the flywheel end) back toward CS1: the expected fill order. */
    private int contiguousFromFlywheelEnd() {
        int n = 0;
        for (int i = N - 1; i >= 0; i--) {
            if (isElement(i)) n++;
            else break;
        }
        return n;
    }

    // ---- results ----

    /** True once every sensor has been read recently. Do not act on the counter while this is false. */
    public boolean isValid() {
        long now = System.nanoTime();
        for (int i = 0; i < N; i++) {
            if (readAtNs[i] == 0L || (now - readAtNs[i]) / 1e6 > STALE_MS) return false;
        }
        return true;
    }

    /** Elements held (sensors that see an element), steady for COUNT_RISE_STABLE_MS / COUNT_FALL_STABLE_MS. */
    public int count() {
        return stableCount;
    }

    public boolean isFull() {
        return stableCount >= CAPACITY;
    }

    /** True if an element is seen that is not part of the run from CS4 (still moving, or a misread). It still counts as held. */
    public boolean hasGap() {
        return elementsSeen() != contiguousFromFlywheelEnd();
    }

    /** What sensor i (0 = CS1) sees right now. */
    public Slot slot(int i) {
        return slot[i];
    }

    /** The element at the deepest slot (it fires first), or EMPTY if CS4 holds none. */
    public Slot next() {
        return isElement(N - 1) ? slot[N - 1] : Slot.EMPTY;
    }

    public int countOf(Slot kind) {
        int n = 0;
        for (int i = 0; i < N; i++) if (slot[i] == kind) n++;
        return n;
    }

    /** True while CS4 sees the gate arm (for the gate position check). */
    public boolean gateArmSeen() {
        return slot[ARM_SENSOR] == Slot.GATE_ARM;
    }

    /** The first sensor (1 to 4, from the intake) holding the OPPONENT's nectar, or 0 if none. */
    public int opponentNectarAt(boolean blueAlliance) {
        Slot opponent = blueAlliance ? Slot.RED_NECTAR : Slot.BLUE_NECTAR;
        for (int i = 0; i < N; i++) if (slot[i] == opponent) return i + 1;
        return 0;
    }

    private static String symbol(Slot s) {
        switch (s) {
            case POLLEN: return "P";
            case RED_NECTAR: return "R";
            case BLUE_NECTAR: return "B";
            case GATE_ARM: return "A";
            case OTHER: return "O";
            default: return ".";
        }
    }

    /** For example "intake > [.] [P] [R] [B] > flywheel". */
    public String stackString() {
        StringBuilder sb = new StringBuilder("intake > ");
        for (int i = 0; i < N; i++) sb.append("[").append(symbol(slot[i])).append("] ");
        return sb.append("> flywheel").toString();
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Elements held", "%d%s   next out: %s   %s", stableCount, isFull() ? " (FULL)" : "",
                next(), isValid() ? "" : "   SENSORS NOT READ RECENTLY");
        telemetry.addData("Stack (P pollen, R red nectar, B blue nectar, O other, A arm)", "%s%s%s", stackString(),
                hasGap() ? "   (gap: an element is moving)" : "", gateArmSeen() ? "   (CS4 sees the gate arm)" : "");
    }
}
