package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;

import java.util.Locale;

/**
 * The slot count (specs/element-counting.md section 1): CS1 to CS4 along the track say which kind of element sits at
 * each of four places. It no longer says how many are held (the CS5 entry count does); it serves the pusher check
 * (the track reads empty), the opponent-nectar warning, seeing the pusher arm, and the log.
 *
 * One sensor is read every 40 ms, in rotation, never more than one per loop: each read blocks about 13 ms.
 */
@Config
public final class SlotCounter {
    public static double READ_PERIOD_MS = 40, STALE_MS = 1000, PRESENCE_DEBOUNCE_MS = 150;
    public static double RISE_MS = 150, FULL_MS = 1000, FALL_MS = 300;
    public static int CAPACITY = 4;
    public static double SOFTWARE_GAIN = 1.0;

    // Presence: separate on and off thresholds so a reading near a threshold does not flicker.
    public static double SHIFT_ON = 0.05, SHIFT_OFF = 0.03;
    public static double BRIGHT_ON = 2.0, BRIGHT_OFF = 1.6, BRIGHT_MIN_ABOVE = 0.02;
    public static double NEAR_ON_CM = 1.5, NEAR_OFF_CM = 0.8;

    // Kind, on the color shares.
    public static double ARM_BLUE_MIN = 0.36, ARM_RED_MAX = 0.28, ARM_GREEN_BLUE_WITHIN = 0.06, ARM_MIN_CM = 4.0;
    public static double RED_RED_MIN = 0.35, RED_BLUE_MAX = 0.30;
    public static double BLUE_BLUE_MIN = 0.44, BLUE_OVER_GREEN = 0.08, BLUE_RED_MAX = 0.25;
    public static double POLLEN_GREEN_OVER_BLUE = 1.5, POLLEN_RED_MAX = 0.35;

    /** The stored empty readings (robot_design/sensors.md, measured 2026-11-05). Re-measured in the pit when the lighting or the track changes. */
    public static double[] EMPTY_ALPHA = {0.209, 0.109, 0.353, 0.061};
    public static double[] EMPTY_CM = {3.60, 5.20, 3.92, 7.67};
    public static double[] EMPTY_RED = {0.238, 0.220, 0.226, 0.240};
    public static double[] EMPTY_GREEN = {0.410, 0.448, 0.414, 0.408};
    public static double[] EMPTY_BLUE = {0.352, 0.332, 0.360, 0.352};

    public enum Kind {
        EMPTY('.'), POLLEN('P'), RED_NECTAR('R'), BLUE_NECTAR('B'), OTHER('O'), PUSHER_ARM('A');

        public final char letter;

        Kind(char letter) { this.letter = letter; }

        public boolean isElement() { return this == POLLEN || this == RED_NECTAR || this == BLUE_NECTAR || this == OTHER; }
    }

    private final RevColorSensorV3[] sensors = new RevColorSensorV3[4];
    private final double[] alpha = new double[4], cm = new double[4], red = new double[4], green = new double[4], blue = new double[4], shift = new double[4];
    private final double[] readAt = {-1e9, -1e9, -1e9, -1e9};
    private final boolean[] rawPresent = new boolean[4], present = new boolean[4];
    private final double[] rawSince = new double[4];
    private final Kind[] kind = {Kind.EMPTY, Kind.EMPTY, Kind.EMPTY, Kind.EMPTY};
    private int next, cs4Reads;
    private double lastReadAt = -1e9;

    private int rawCount, count;
    private double rawCountSince;

    // Things that are kept out of the count: set by the coordinator each loop.
    private boolean armNearCs4, checkRunning, noneCounted, frozen, cs4Only;

    private SlotCounter() { }

    /** Null unless all four sensors are configured. */
    public static SlotCounter create(HardwareMap hw) {
        SlotCounter c = new SlotCounter();
        for (int i = 0; i < 4; i++) {
            c.sensors[i] = hw.tryGet(RevColorSensorV3.class, ChassisProfile.SLOT_SENSORS[i]);
            if (c.sensors[i] == null) return null;
            try {
                c.sensors[i].setGain((float) SOFTWARE_GAIN);
            } catch (Throwable ignored) { }
        }
        return c;
    }

    /**
     * @param armNearCs4   the arm is near CS4: an OTHER reading there is the arm's edge
     * @param checkRunning a pusher check is running: CS4 is not counted
     * @param noneCounted  a check is sweeping, or just ended: no sensor is counted
     * @param frozen       the pusher is locked: the count is frozen at its value; the sensors are still read
     * @param cs4Only      a check is sweeping: read CS4 on every loop
     */
    public void setContext(boolean armNearCs4, boolean checkRunning, boolean noneCounted, boolean frozen, boolean cs4Only) {
        this.armNearCs4 = armNearCs4;
        this.checkRunning = checkRunning;
        this.noneCounted = noneCounted;
        this.frozen = frozen;
        this.cs4Only = cs4Only;
    }

    public void update(double t) {
        if (cs4Only) {
            read(3, t);
        } else if ((t - lastReadAt) * 1000 >= READ_PERIOD_MS) {
            read(next, t);
            next = (next + 1) % 4;
        }
        // The count: sensors that see an element. The pusher arm is not an element.
        int raw = 0;
        if (!noneCounted) {
            for (int i = 0; i < 4; i++) {
                if (!present[i] || !kind[i].isElement()) continue;
                if (i == 3 && (checkRunning || (armNearCs4 && kind[i] == Kind.OTHER))) continue;
                raw++;
            }
        }
        if (raw != rawCount) {
            rawCount = raw;
            rawCountSince = t;
        }
        if (!frozen && rawCount != count) {
            double need = rawCount < count ? FALL_MS : (rawCount >= CAPACITY ? FULL_MS : RISE_MS);
            if ((t - rawCountSince) * 1000 >= need) count = rawCount;
        }
        now = t;
    }

    private double now;

    private void read(int i, double t) {
        lastReadAt = t;
        NormalizedRGBA c;
        double d;
        try {
            c = sensors[i].getNormalizedColors();
            d = sensors[i].getDistance(DistanceUnit.CM);
        } catch (Throwable e) {
            return;     // the sensor goes stale and the counter stops being valid
        }
        double sum = c.red + c.green + c.blue;
        if (sum <= 0 || Double.isNaN(d)) return;
        readAt[i] = t;
        if (i == 3) cs4Reads++;
        alpha[i] = c.alpha;
        cm[i] = d;
        red[i] = c.red / sum;
        green[i] = c.green / sum;
        blue[i] = c.blue / sum;
        shift[i] = Math.sqrt(sq(red[i] - EMPTY_RED[i]) + sq(green[i] - EMPTY_GREEN[i]) + sq(blue[i] - EMPTY_BLUE[i]));
        double ratio = alpha[i] / Math.max(1e-6, EMPTY_ALPHA[i]), nearer = EMPTY_CM[i] - d;
        boolean on = shift[i] >= SHIFT_ON || (ratio >= BRIGHT_ON && alpha[i] - EMPTY_ALPHA[i] >= BRIGHT_MIN_ABOVE) || nearer >= NEAR_ON_CM;
        boolean off = shift[i] < SHIFT_OFF && ratio < BRIGHT_OFF && nearer < NEAR_OFF_CM;
        boolean raw = rawPresent[i] ? !off : on;
        if (raw != rawPresent[i]) {
            rawPresent[i] = raw;
            rawSince[i] = t;
        }
        // A new presence state must persist before it is reported.
        if (rawPresent[i] != present[i] && (t - rawSince[i]) * 1000 >= PRESENCE_DEBOUNCE_MS) present[i] = rawPresent[i];
        kind[i] = present[i] ? classify(i) : Kind.EMPTY;
    }

    private Kind classify(int i) {
        double r = red[i], g = green[i], b = blue[i];
        if (i == 3 && b >= ARM_BLUE_MIN && r <= ARM_RED_MAX && Math.abs(g - b) <= ARM_GREEN_BLUE_WITHIN && cm[i] >= ARM_MIN_CM) return Kind.PUSHER_ARM;
        if (r >= RED_RED_MIN && r >= g && b < RED_BLUE_MAX) return Kind.RED_NECTAR;
        if (b >= BLUE_BLUE_MIN && b >= g + BLUE_OVER_GREEN && r <= BLUE_RED_MAX) return Kind.BLUE_NECTAR;
        if (g >= POLLEN_GREEN_OVER_BLUE * b && r <= POLLEN_RED_MAX) return Kind.POLLEN;
        return Kind.OTHER;
    }

    private static double sq(double v) { return v * v; }

    /** Valid only while every sensor has been read within the last second. Callers must not act on it otherwise. */
    public boolean valid() {
        for (int i = 0; i < 4; i++) {
            if ((now - readAt[i]) * 1000 > STALE_MS && !(cs4Only && i < 3)) return false;
        }
        return true;
    }

    /** The first sensor that has stopped answering, 1 to 4, or 0. */
    public int staleSensor() {
        for (int i = 0; i < 4; i++) if ((now - readAt[i]) * 1000 > STALE_MS) return i + 1;
        return 0;
    }

    public int count() { return count; }
    public boolean full() { return count >= CAPACITY; }
    public Kind kind(int i) { return kind[i]; }
    /** The element at CS4 launches first. */
    public Kind kindAtCs4() { return kind[3]; }
    public int cs4Reads() { return cs4Reads; }

    /** Whether CS4 classifies what it sees right now as the pusher arm (raw presence, no debounce: the check reads it every loop). */
    public boolean cs4SeesArm() {
        return rawPresent[3] && classify(3) == Kind.PUSHER_ARM;
    }

    /** An element seen without a neighbor on its flywheel side: still moving, or a misread. It still counts. */
    public boolean gap() {
        for (int i = 0; i < 3; i++) {
            if (present[i] && kind[i].isElement() && !(present[i + 1] && kind[i + 1].isElement())) return true;
        }
        return false;
    }

    /** The first sensor from the intake end, 1 to 4, that holds the opponent's nectar; 0 for none. */
    public int opponentNectarAt(Alliance us) {
        Kind theirs = us == Alliance.BLUE ? Kind.RED_NECTAR : Kind.BLUE_NECTAR;
        for (int i = 0; i < 4; i++) if (present[i] && kind[i] == theirs) return i + 1;
        return 0;
    }

    /** The four slots from the intake end, for the log: P pollen, R red nectar, B blue nectar, O other, A arm, a dot for empty. */
    public String slots() {
        return "" + kind[0].letter + kind[1].letter + kind[2].letter + kind[3].letter;
    }

    public double shift(int i) { return shift[i]; }
    public double alpha(int i) { return alpha[i]; }
    public double cm(int i) { return cm[i]; }

    /** The four live color shifts, for the empty-track health line. */
    public String shiftsText() {
        return String.format(Locale.US, "%.3f %.3f %.3f %.3f", shift[0], shift[1], shift[2], shift[3]);
    }

    public static String baselinesText() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            sb.append(String.format(Locale.US, "%sCS%d alpha %.3f, %.2f cm, shares %.3f/%.3f/%.3f", i == 0 ? "" : "; ", i + 1,
                    EMPTY_ALPHA[i], EMPTY_CM[i], EMPTY_RED[i], EMPTY_GREEN[i], EMPTY_BLUE[i]));
        }
        return sb.toString();
    }

    /** One sensor's latest reading, for studies and the verbose screen. */
    public String reading(int i) {
        return String.format(Locale.US, "CS%d %s alpha %.3f %.2f cm %.3f/%.3f/%.3f shift %.3f", i + 1, kind[i], alpha[i], cm[i], red[i], green[i], blue[i], shift[i]);
    }
}
