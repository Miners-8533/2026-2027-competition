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
 * Counts and identifies the elements in the transport with THREE REV Color Sensor V3 along the track: intake -> CS1 -> CS3 ->
 * CS4 -> flywheel (CS4 is deepest, nearest the flywheel and the pusher arm). Until 2026-11-18 there were four: CS2 sat between
 * CS1 and CS3. The team moved it to the top of the feed ramp, where {@link LaunchSensor} reads it; it is NOT read here, never
 * counts, and its place in the slots text is a dash (IN_TRACK). The arrays keep four places so that the sensor numbers and the
 * log's columns stay as they were. The count is therefore 3 at most and {@link #isFull()} can no longer be true: the robot's
 * count is the CS5 entry count (Mechanisms.COUNT_BY_ENTRIES). Not run on the robot in this form. The count
 * is the number of sensors that see an element. The stack fills from the CS4 end, so the run of elements in a row from
 * CS4 is a check: an element seen without a neighbor toward the flywheel is still moving (or a misread) and sets
 * {@link #hasGap()}, but it still counts as held.
 *
 * Each sensor's slot is EMPTY, POLLEN, RED_NECTAR, BLUE_NECTAR, OTHER, or PUSHER_ARM (CS4 sees the arm; it is not an
 * element). Presence is decided by how far the color shares move from the sensor's own EMPTY readings (the empty views
 * differ from sensor to sensor and brightness contrast against a near wall is small), then the alpha ratio and a nearer
 * distance, with separate on and off thresholds and a debounce so a pollen near a threshold does not flicker. The kind
 * comes from the color shares. The thresholds were measured on the robot (see ColorSensorsTest, which scores them against
 * labels and shares these same settings) and are in the FTC Dashboard.
 *
 * Reading: each sensor read is two blocking I2C transactions (about 13 ms) and the sensors only refresh every 100 ms, so
 * {@link #update()} reads ONE sensor every READ_PERIOD_MS, cycling (about 4 x READ_PERIOD_MS for a full refresh), and does
 * nothing in between. The reported count changes only after the raw count has been steady (COUNT_RISE_STABLE_MS when it
 * goes up; COUNT_FULL_STABLE_MS when it reaches CAPACITY, so four are certain before the intake stops; COUNT_FALL_STABLE_MS when it goes down), because the end sensor flickers
 * for about a second while an element moves. A fourth element is reported about 1.3 s after it is first seen (COUNT_FULL_STABLE_MS is 1000 since 2026-11-04, was 500: of the 111 stretches with all four sensors lit on 2026-11-03, 79 lasted 0.5 s or more and 59 lasted 1.0 s or more). If a sensor has not been read for STALE_MS the
 * counter is not {@link #isValid() valid}, and callers should not act on it.
 *
 * The EMPTY baseline is the table below (measured 2026-10-31 with the track empty). Lighting changes the empty readings,
 * so re-measure it in the pit with ColorSensorsTest (A, with the track empty) and copy the BASELINE line into the table.
 * It cannot be captured automatically at the start of a match, because the robot may already hold elements.
 */
@Config
public class ElementCounter {

    public enum Slot { EMPTY, POLLEN, RED_NECTAR, BLUE_NECTAR, OTHER, PUSHER_ARM }

    /** Software gain: a multiplier clipped at 1.0 (no signal is added), so keep it 1.0. */
    public static double GAIN = 1.0;
    public static double READ_PERIOD_MS = 40.0;
    public static double STALE_MS = 1000.0;
    public static double COUNT_RISE_STABLE_MS = 150.0;
    /** Reaching CAPACITY (FULL, which stops the intake) must be steady this long, so a flicker at CS4 does not read as four. */
    public static double COUNT_FULL_STABLE_MS = 1000.0;   // 500 before 2026-11-04 (user): a sliding pack lit all four sensors for 0.5 to 1 s at a time
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
    /** Pusher arm: bluish with GREEN CLOSE TO BLUE (about 0.23 / 0.38 / 0.39); a dim blue nectar has blue well above green. */
    // 2026-11-19 (user, the new normal at CS4): the arm now reads rgb 0.21 to 0.22 / 0.41 to 0.42 / 0.36 to 0.37 at 2.5 to 3.0 cm (Pusher Calibration Test), so the blue floor is 0.34, the green to blue gap 0.10, there is
    // a blue CEILING (blue nectar is 0.44 and up), and the distance is no longer asked for (below).
    public static double ARM_MIN_BLUE = 0.34;
    public static double ARM_MAX_BLUE = 0.42;
    public static double ARM_MAX_RED = 0.28;
    public static double ARM_MAX_GREEN_BLUE_GAP = 0.10;
    /** The arm is only expected at CS4; ColorSensorsTest turns this on to hold it in front of any sensor. */
    public static boolean ARM_ANY_SENSOR = false;
    public static boolean ARM_REQUIRE_DISTANCE = false;   // FALSE 2026-11-19: the arm now reads 2.5 to 3.0 cm, as near as pollen (3.0), so the color alone says arm: pollen's blue is 0.18 to 0.27, nectar's 0.44 and up
    /**
     * The arm reads farther than pollen at CS4 (pollen about 3.0 cm, an empty track about 7.7). 4.0 since 2026-11-05 (was 5.5): with the recolored transport the arm's CENTER reads 5.1 to 5.2 cm and only
     * its shoulders read 5.8 cm or more, so 5.5 accepted only the shoulders: the pusher check saw a crossing 8 to 18 deg wide instead of about 46 and corrected the zero by 14 to 29 deg (and by 94 and 155 deg
     * in two runs) from the wrong center. Checked on the Color Sensor Study of 2026-11-05: no pollen read at CS4 is accepted at 4.0 (0 of 130 still reads).
     */
    public static double ARM_MIN_DIST_CM = 4.0;

    public static final String[] NAMES = {"CS1", "CS2", "CS3", "CS4"};
    public static final int N = NAMES.length;
    public static final int ARM_SENSOR = 3;   // CS4
    /** CS2: at the top of the feed ramp since 2026-11-18, read by {@link LaunchSensor} and not by this class. */
    public static final int FEED_RAMP_SENSOR = 1;
    /** Which of the four names is a place along the track. A sensor that is not is never read here, is never an element, and is not needed for the counter to exist or to be valid. */
    private static final boolean[] IN_TRACK = {true, false, true, true};

    /** True if sensor i (0 = CS1) is a place along the track; false for CS2, which is on the feed ramp. */
    public static boolean inTrack(int i) {
        return IN_TRACK[i];
    }

    /**
     * Empty-track readings: alpha, distance (cm), and the red / green / blue shares, per sensor. RE-MEASURED 2026-11-05 after the rotary transport was
     * recolored (Color Sensor Study step 1, 8 s, 57 reads per sensor, the alpha spread within 0.003): the old values read CS1 to CS3 as lit on an empty track
     * (shifts 0.10, 0.14 and 0.06 against the 0.05 ON threshold); the values before this were alpha 0.062 0.023 0.173 0.039, distance 3.69 5.39 4.11 7.75, shares
     * 0.318/0.389/0.293 0.328/0.429/0.244 0.274/0.402/0.325 0.266/0.403/0.331. Re-measured 2026-11-03 from the empty STILL takes of the
     * Color Sensor Study (874 rows, two runs): CS2's empty view had changed (alpha 0.080 -> 0.023, shares 0.29/0.42/0.29 -> 0.33/0.43/0.24), which put it 0.06 from
     * the old baseline, over the 0.05 ON threshold, so the empty track read one POLLEN at CS2 for good and the count was always one too high. (Measured
     * 2026-10-31: alpha 0.077 0.080 0.173 0.038, distance 3.68 5.23 4.11 7.87, shares 0.32/0.38/0.30 0.29/0.42/0.29 0.28/0.40/0.32 0.30/0.39/0.31.)
     * Lighting and the sensor views change: re-measure in the pit (Color Sensor Study: an empty STILL take) and copy the medians here.
     */
    // CS4 re-measured 2026-11-19 (user: the new normal for this sensor): its empty view now reads about 3 cm, not 7.7 (the 19:53 TeleOp read 6.1 to 6.9, every run from 21:45 on 2.7 to 2.9). The Pusher
    // Calibration Test's two empty baselines, the arm at HOLD (alpha 0.078, 2.74 cm, 0.237/0.429/0.335) and the arm 40 deg past it (0.055, 3.13 cm, 0.263/0.430/0.306), averaged, so both are within about 0.02 of it.
    // CS4 REPLACED (a new sensor, 2026-11-21, user) and re-measured the same day with the Pusher Calibration Test (robot-logs/2026-11-21/093545_bringup.log), the arm at HOLD as always: alpha 0.128, 4.53 cm,
    // 0.248/0.415/0.337 (the old sensor's last values were 0.067, 2.94 cm, 0.250/0.430/0.320; against those the new sensor's empty view read 2.6 times the alpha, a phantom element). With the arm far away the new
    // sensor reads alpha 0.094 and 0.277/0.412/0.311 (a shift of 0.039, under the 0.05 ON threshold); the arm at its center reads alpha 0.28 to 0.38 (2.2 to 3.0 times the baseline), 3.5 to 3.9 cm, 0.23/0.41/0.36.
    // CS3 re-measured 2026-11-21 (user, after the Novi Qualifier), from the same Pusher Calibration Test (093545_bringup.log, the robot empty): alpha 0.575, 3.51 cm, 0.230/0.428/0.343. The stored values were
    // 0.353, 3.92 cm, 0.226/0.414/0.360: against them the empty CS3 read 1.63 to 1.66 times the alpha, over ALPHA_RATIO_OFF (1.6), so once the first pollen of a run had passed it CS3 never read empty again. It
    // showed OTHER whenever nothing was in front of it and counted as one element all day: every pusher check was skipped ("the robot did not read empty") and the held count read up to one too high.
    private static final double[] BASE_ALPHA = {0.209, 0.109, 0.575, 0.128};
    private static final double[] BASE_DIST = {3.60, 5.20, 3.51, 4.53};
    private static final double[][] BASE_SHARES = {{0.238, 0.410, 0.352}, {0.220, 0.448, 0.332}, {0.230, 0.428, 0.343}, {0.248, 0.415, 0.337}};

    private final NormalizedColorSensor[] color = new NormalizedColorSensor[N];
    private final DistanceSensor[] distance = new DistanceSensor[N];
    private final boolean[] present = new boolean[N];
    private final boolean[] candidate = new boolean[N];
    private final long[] candidateSinceNs = new long[N];
    private final Slot[] slot = new Slot[N];
    private final long[] readAtNs = new long[N];
    private final int[] readCount = new int[N];
    private final double[] lastAlpha = new double[N];
    private final double[] lastDist = new double[N];
    private final double[][] lastShares = new double[N][3];
    private boolean armCountIgnored = false;
    private boolean allCountIgnored = false;
    private boolean armNear = false;
    private long countIgnoredUntilNs = 0L;
    private int nextSensor = 0;
    private long lastReadNs = 0L;
    private int stableCount = 0;
    private int rawCount = 0;
    private long rawSinceNs = 0L;

    private ElementCounter(HardwareMap hardwareMap) {
        for (int i = 0; i < N; i++) {
            if (!IN_TRACK[i]) continue;   // CS2: left to LaunchSensor (its fast proximity setting leaves no usable distance)
            color[i] = hardwareMap.get(NormalizedColorSensor.class, NAMES[i]);
            distance[i] = hardwareMap.tryGet(DistanceSensor.class, NAMES[i]);
            color[i].setGain((float) GAIN);
            if (color[i] instanceof ColorSensor) ((ColorSensor) color[i]).enableLed(true);
        }
        Arrays.fill(slot, Slot.EMPTY);
    }

    /** The counter if the three track sensors CS1, CS3 and CS4 are configured, else null (a robot without them still runs). CS2 is not needed. */
    public static ElementCounter tryCreate(HardwareMap hardwareMap) {
        for (int i = 0; i < N; i++) {
            if (IN_TRACK[i] && hardwareMap.tryGet(NormalizedColorSensor.class, NAMES[i]) == null) return null;
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

    /** True if a reading looks like sensor i's EMPTY baseline: nothing in the track in front of it, and the lighting as measured (for the pit check). */
    public static boolean looksEmpty(int sensor, double shareR, double shareG, double shareB, double alpha, double dist) {
        return !presenceNext(false, shareR, shareG, shareB, alpha, dist,
                BASE_SHARES[sensor][0], BASE_SHARES[sensor][1], BASE_SHARES[sensor][2], BASE_ALPHA[sensor], BASE_DIST[sensor]);
    }

    /** What a PRESENT object is, from its color shares (and distance, for the arm). */
    public static Slot classify(int sensor, double red, double green, double blue, double distCm) {
        if ((ARM_ANY_SENSOR || sensor == ARM_SENSOR) && (!ARM_REQUIRE_DISTANCE || distCm >= ARM_MIN_DIST_CM)
                && blue >= ARM_MIN_BLUE && blue <= ARM_MAX_BLUE && red <= ARM_MAX_RED && Math.abs(green - blue) <= ARM_MAX_GREEN_BLUE_GAP) return Slot.PUSHER_ARM;
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
            do {
                nextSensor = (nextSensor + 1) % N;
            } while (!IN_TRACK[nextSensor]);
        }
        if (frozen) return;   // the pusher lock: the sensors are still read (and logged) but the count stays where it was
        // the count changes only after the raw count has been steady for COUNT_STABLE_MS
        int raw = elementsSeen();
        if (raw != rawCount) {
            rawCount = raw;
            rawSinceNs = now;
        }
        double steadyMs = rawCount < stableCount ? COUNT_FALL_STABLE_MS
                : rawCount >= CAPACITY ? COUNT_FULL_STABLE_MS : COUNT_RISE_STABLE_MS;
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
        readCount[i]++;

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
        if (!IN_TRACK[i]) return false;
        if (allCountIgnored || System.nanoTime() < countIgnoredUntilNs) return false;
        if (i == ARM_SENSOR && armCountIgnored) return false;
        if (i == ARM_SENSOR && armNear && slot[i] == Slot.OTHER) return false;   // the edge of the arm in CS4's view, not an element
        return slot[i] != Slot.EMPTY && slot[i] != Slot.PUSHER_ARM;
    }

    /**
     * While the pusher check moves the arm across CS4, the edge of the arm can read as "other" and would count as an element.
     * True keeps CS4 out of the count (CS4 is still read, and {@link #pusherArmSeen} still works) until set false.
     */
    public void ignoreArmSensorCount(boolean on) {
        armCountIgnored = on;
    }

    /**
     * Tells the counter the pusher arm is where CS4 can see part of it. CS4 then reads the arm's edge as "other" (and flickers between that
     * and the arm itself) for as long as the arm rests there, which counted as an element: 2026-11-01 logs show the arm resting 5-15 deg
     * past HOLD, its usual rest, read "other" about a quarter of the time. An "other" at CS4 is then not counted; pollen and nectar still are.
     */
    public void setArmNear(boolean near) {
        armNear = near;
    }

    /**
     * True keeps ALL four sensors out of the count until set false. A pusher sweep turns the arm through the whole track, past CS1 to
     * CS3 as well, and the arm reads as "other" there (the 2026-11-01 manual sweep was aborted at CS1 for "an element in the robot").
     */
    public void ignoreAllCount(boolean on) {
        allCountIgnored = on;
    }

    /** Keeps all sensors out of the count for this many seconds from now (the arm's last readings as it leaves their view). */
    public void ignoreCountFor(double seconds) {
        countIgnoredUntilNs = System.nanoTime() + (long) (seconds * 1e9);
    }

    private boolean frozen = false;

    /**
     * True holds the reported count where it is (the sensors are still read, so the slots can be logged). The pusher lock uses it: with the arm parked across the entrance the
     * arm may read as an element at one of the sensors, and nothing can enter or leave while it is locked. Set false and the count follows the sensors again after the usual steady time.
     */
    public void setFrozen(boolean on) {
        setFrozen(on, false);
    }

    /**
     * requalifyFull (only when releasing): a reported FULL drops to CAPACITY - 1 if the sensors still read CAPACITY, so FULL has to be confirmed again by the normal COUNT_FULL_STABLE_MS
     * (used when the operator forces the intake IN through the pusher lock: the robot is locked again as soon as the sensors have shown full for that long, with no further wait).
     */
    public void setFrozen(boolean on, boolean requalifyFull) {
        if (on == frozen) return;
        frozen = on;
        if (!on) {
            rawCount = elementsSeen();
            rawSinceNs = System.nanoTime();
            if (requalifyFull && stableCount >= CAPACITY && rawCount >= CAPACITY) stableCount = CAPACITY - 1;
        }
    }

    public boolean isFrozen() {
        return frozen;
    }

    /** Sensors that currently see an element (not the pusher arm). */
    private int elementsSeen() {
        int n = 0;
        for (int i = 0; i < N; i++) if (isElement(i)) n++;
        return n;
    }

    /** Elements in a row from CS4 (the flywheel end) back toward CS1: the expected fill order. */
    private int contiguousFromFlywheelEnd() {
        int n = 0;
        for (int i = N - 1; i >= 0; i--) {
            if (!IN_TRACK[i]) continue;   // CS2 is not between CS1 and CS3 any more
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
            if (!IN_TRACK[i]) continue;
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

    /** The element at the deepest slot (it launches first), or EMPTY if CS4 holds none. */
    public Slot next() {
        return isElement(N - 1) ? slot[N - 1] : Slot.EMPTY;
    }

    public int countOf(Slot kind) {
        int n = 0;
        for (int i = 0; i < N; i++) if (slot[i] == kind) n++;
        return n;
    }

    // ---- raw readings, for the Color Sensor Study log ----

    /** Alpha (total light) of sensor i's latest read. */
    public double lastAlpha(int i) {
        return lastAlpha[i];
    }

    /** Distance (cm) of sensor i's latest read; NaN if it has no distance sensor. */
    public double lastDistCm(int i) {
        return lastDist[i];
    }

    /** Color share of sensor i's latest read: channel 0 = red, 1 = green, 2 = blue. */
    public double lastShare(int i, int channel) {
        return lastShares[i][channel];
    }

    /** How far sensor i's latest color shares are from its own empty baseline (the presence rule's first test). 0 for CS2, which is not read here. */
    public double colorShift(int i) {
        if (!IN_TRACK[i]) return 0.0;
        return Math.sqrt(Math.pow(lastShares[i][0] - BASE_SHARES[i][0], 2) + Math.pow(lastShares[i][1] - BASE_SHARES[i][1], 2)
                + Math.pow(lastShares[i][2] - BASE_SHARES[i][2], 2));
    }

    /** The stored empty baselines, for the log (tools/logs/baseline_health.py compares the live empty readings with them). */
    public static String describeBaselines() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < N; i++) {
            if (!IN_TRACK[i]) {
                sb.append(String.format(java.util.Locale.US, "%sCS%d not in the slot count (on the feed ramp)", i == 0 ? "" : "; ", i + 1));
                continue;
            }
            sb.append(String.format(java.util.Locale.US, "%sCS%d alpha %.3f dist %.2f cm shares %.3f/%.3f/%.3f", i == 0 ? "" : "; ", i + 1, BASE_ALPHA[i], BASE_DIST[i],
                    BASE_SHARES[i][0], BASE_SHARES[i][1], BASE_SHARES[i][2]));
        }
        return sb.toString();
    }

    /** Sensor i's presence result on its latest read, before the debounce. */
    public boolean rawPresent(int i) {
        return candidate[i];
    }

    /** Sensors that see an element right now (before the steady-count windows that make {@link #count}). */
    public int elementsNow() {
        return elementsSeen();
    }

    /** How many times sensor i has been read so far (the pusher check waits for fresh reads after the arm stops). */
    public int readCount(int i) {
        return readCount[i];
    }

    /**
     * Reads CS4 right now (about 13 ms) and says whether it sees the pusher arm, without touching the counter's own state. For the
     * pusher check's sweep, which cannot wait for the counter's one-sensor-per-40-ms schedule. The sensor still only refreshes every 100 ms.
     */
    public boolean armSeenNow() {
        NormalizedRGBA c = color[ARM_SENSOR].getNormalizedColors();
        double d = distance[ARM_SENSOR] != null ? distance[ARM_SENSOR].getDistance(DistanceUnit.CM) : Double.NaN;
        double sum = c.red + c.green + c.blue + 1e-9;
        double r = c.red / sum, g = c.green / sum, b = c.blue / sum;
        return !looksEmpty(ARM_SENSOR, r, g, b, c.alpha, d) && classify(ARM_SENSOR, r, g, b, d) == Slot.PUSHER_ARM;
    }

    /** True while CS4 sees the pusher arm (for the pusher position check). */
    public boolean pusherArmSeen() {
        return slot[ARM_SENSOR] == Slot.PUSHER_ARM;
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
            case PUSHER_ARM: return "A";
            case OTHER: return "O";
            default: return ".";
        }
    }

    /** For example ".-RB" (CS1 to CS4; P pollen, R red nectar, B blue nectar, O other, A arm, . empty; the dash is CS2, which is not a place along the track). */
    public String slotsCompact() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < N; i++) sb.append(IN_TRACK[i] ? symbol(slot[i]) : "-");
        return sb.toString();
    }

    /** For example "intake > [.] [R] [B] > flywheel" (CS1, CS3, CS4). */
    public String stackString() {
        StringBuilder sb = new StringBuilder("intake > ");
        for (int i = 0; i < N; i++) if (IN_TRACK[i]) sb.append("[").append(symbol(slot[i])).append("] ");
        return sb.append("> flywheel").toString();
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Elements held", "%d%s   next out: %s   %s", stableCount, isFull() ? " (FULL)" : "",
                next(), isValid() ? "" : "   SENSORS NOT READ RECENTLY");
        telemetry.addData("Stack (P pollen, R red nectar, B blue nectar, O other, A arm)", "%s%s%s", stackString(),
                hasGap() ? "   (gap: an element is moving)" : "", pusherArmSeen() ? "   (CS4 sees the pusher arm)" : "");
    }
}
