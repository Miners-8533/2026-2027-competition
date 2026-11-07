package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;
import java.util.Locale;

/**
 * Drive wheel check from the motor currents (2026-11-07, after a belt broke on a drive wheel in Q10 at the Hartland Qualifier and nothing on the robot noticed).
 *
 * A wheel that is driven but loaded draws about 1.5 to 1.7 A at 0.3 to 1 power (2.4 to 2.9 A per unit of power in the 2026-11-07 Drive Characterization logs, the four
 * wheels within about 20 percent of each other); a wheel whose belt has gone spins freely and draws a fraction of that. {@link MecanumDrive} reads ONE wheel's current every
 * PARAMS.currentCheckMs (round robin, only a wheel that is being driven at MIN_POWER or more, so a robot standing still reads nothing) and calls {@link #add}. For each wheel
 * the amps per unit of commanded power are averaged (EMA); a wheel whose average stays under LOW_FRACTION of the median of the other wheels for CONFIRM_SAMPLES of its
 * own samples is flagged, and the flag clears when it is back over RECOVER_FRACTION. This class touches no hardware.
 */
@Config
public final class DriveHealth {

    /** A sample counts only when the wheel's commanded power is at least this (the power of the wheel, not of the stick). */
    public static double MIN_POWER = 0.3;
    /** Flagged when the wheel's amps per unit power is under this fraction of the median of the others'. A freewheeling wheel draws about 0.3 to 0.5 A against 1.6 A loaded (about 0.2 to 0.3). */
    public static double LOW_FRACTION = 0.45;
    /** The flag clears when the wheel is back over this fraction. */
    public static double RECOVER_FRACTION = 0.7;
    /** Weight of a new sample in the wheel's average. */
    public static double EMA_ALPHA = 0.3;
    /** A wheel needs this many samples before it is judged or used as a reference. */
    public static int MIN_SAMPLES = 6;
    /** The wheel must stay low for this many of its own samples in a row (a wheel is read a few times a second while it is driven). */
    public static int CONFIRM_SAMPLES = 4;

    public static final String[] NAMES = {"LF", "LB", "RB", "RF"};

    private final double[] ema = new double[4];
    private final int[] samples = new int[4];
    private final int[] lowRun = new int[4];
    private final boolean[] flagged = new boolean[4];

    /**
     * One current reading of a wheel.
     *
     * @param wheel 0 LF, 1 LB, 2 RB, 3 RF
     * @param amps  the motor current, amps
     * @param power the power commanded to that wheel when it was read, -1 to 1
     * @return a log message when the wheel's state changed (flagged or back to normal), otherwise null
     */
    public String add(int wheel, double amps, double power) {
        double p = Math.abs(power);
        if (Double.isNaN(amps) || p < MIN_POWER) return null;
        double r = amps / p;
        ema[wheel] = samples[wheel] == 0 ? r : ema[wheel] + EMA_ALPHA * (r - ema[wheel]);
        samples[wheel]++;
        if (samples[wheel] < MIN_SAMPLES) return null;
        double ref = referenceExcluding(wheel);
        if (Double.isNaN(ref) || ref <= 0.0) return null;
        if (ema[wheel] < LOW_FRACTION * ref) {
            lowRun[wheel]++;
            if (!flagged[wheel] && lowRun[wheel] >= CONFIRM_SAMPLES) {
                flagged[wheel] = true;
                return String.format(Locale.US, "DRIVE WHEEL %s LOW CURRENT: %.2f A per unit of power against %.2f A for the other wheels (%.0f percent): a broken belt or a motor that is not loaded?",
                        NAMES[wheel], ema[wheel], ref, 100.0 * ema[wheel] / ref);
            }
        } else if (ema[wheel] > RECOVER_FRACTION * ref) {
            lowRun[wheel] = 0;
            if (flagged[wheel]) {
                flagged[wheel] = false;
                return String.format(Locale.US, "DRIVE WHEEL %s current back to normal: %.2f A per unit of power against %.2f A for the others", NAMES[wheel], ema[wheel], ref);
            }
        }
        return null;
    }

    /** The median of the other wheels' averages, over those with enough samples; NaN when fewer than two have. */
    private double referenceExcluding(int wheel) {
        double[] v = new double[3];
        int n = 0;
        for (int i = 0; i < 4; i++) {
            if (i != wheel && samples[i] >= MIN_SAMPLES) v[n++] = ema[i];
        }
        if (n < 2) return Double.NaN;
        double[] s = Arrays.copyOf(v, n);
        Arrays.sort(s);
        return n == 2 ? 0.5 * (s[0] + s[1]) : s[1];
    }

    public boolean isFlagged(int wheel) {
        return flagged[wheel];
    }

    /** "" when every wheel is fine, else a short text for the screen, such as "DRIVE WHEEL LB?". */
    public String warning() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            if (flagged[i]) sb.append(sb.length() == 0 ? "DRIVE WHEEL " : "+").append(NAMES[i]);
        }
        return sb.length() == 0 ? "" : sb.append("?").toString();
    }

    public double average(int wheel) {
        return ema[wheel];
    }
}
