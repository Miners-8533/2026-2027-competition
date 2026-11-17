package org.firstinspires.ftc.teamcode.drive;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Notices a drive wheel that is not loaded: a broken or slipping belt (specs/drive.md section 3).
 * Pure logic: the drive feeds it one wheel current at a time.
 */
public final class WheelCheck {
    public static final String[] NAMES = {"LF", "LB", "RB", "RF"};

    private final double[] avg = new double[4];
    private final int[] samples = new int[4];
    private final int[] lowRun = new int[4];
    private final boolean[] flagged = new boolean[4];
    private final double[] lastAmps = new double[4];
    private int next;

    /** The wheel to read now, taking the wheels in turn and skipping any commanded below the minimum; -1 if none. */
    public int pick(double[] powers) {
        for (int k = 0; k < 4; k++) {
            int i = (next + k) % 4;
            if (Math.abs(powers[i]) >= DriveParams.CHECK_MIN_POWER) {
                next = (i + 1) % 4;
                return i;
            }
        }
        return -1;
    }

    /** Judges one sample. Returns a log line when the wheel is flagged or recovers, else null. */
    public String sample(int i, double power, double amps) {
        lastAmps[i] = amps;
        double perPower = amps / Math.max(1e-6, Math.abs(power));
        double w = DriveParams.CHECK_NEW_WEIGHT;
        avg[i] = samples[i] == 0 ? perPower : avg[i] * (1 - w) + perPower * w;
        samples[i]++;
        if (samples[i] < DriveParams.CHECK_SAMPLES_TO_JUDGE) return null;
        List<Double> others = new ArrayList<>();
        for (int j = 0; j < 4; j++) {
            if (j != i && samples[j] >= DriveParams.CHECK_SAMPLES_TO_JUDGE) others.add(avg[j]);
        }
        if (others.isEmpty()) return null;
        Collections.sort(others);
        int n = others.size();
        double ref = n % 2 == 1 ? others.get(n / 2) : (others.get(n / 2 - 1) + others.get(n / 2)) / 2;
        if (!flagged[i]) {
            if (avg[i] < DriveParams.CHECK_LOW_FRACTION * ref) {
                if (++lowRun[i] >= DriveParams.CHECK_SAMPLES_TO_CONFIRM) {
                    flagged[i] = true;
                    return String.format(Locale.US, "DRIVE WHEEL %s draws too little current: %.2f A per unit power against %.2f for the others (belt?)",
                            NAMES[i], avg[i], ref);
                }
            } else {
                lowRun[i] = 0;
            }
        } else if (avg[i] > DriveParams.CHECK_RECOVER_FRACTION * ref) {
            flagged[i] = false;
            lowRun[i] = 0;
            return String.format(Locale.US, "DRIVE WHEEL %s recovered: %.2f A per unit power against %.2f", NAMES[i], avg[i], ref);
        }
        return null;
    }

    /** Short screen text naming the flagged wheels, for example "DRIVE WHEEL LB?", or null. */
    public String flaggedText() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            if (flagged[i]) sb.append(sb.length() == 0 ? "" : " ").append(NAMES[i]);
        }
        return sb.length() == 0 ? null : "DRIVE WHEEL " + sb + "?";
    }

    public double lastAmps(int i) { return lastAmps[i]; }
}
