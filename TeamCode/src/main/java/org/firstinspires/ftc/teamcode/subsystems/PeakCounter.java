package org.firstinspires.ftc.teamcode.subsystems;

/**
 * The streaming PEAK detector that counts elements at the intake from CS5's proximity (2026-11-14). Pure Java (no FTC classes), so tools/entry/PeakCounterCheck.java replays recorded runs through it.
 *
 * One element passing the sensor is a bump in the proximity count (+8 to +44 above the empty level of about 65; the noise is about 0.5 counts). A bump is counted when its PROMINENCE is at least
 * `prominence` counts: the signal rose that far above the lowest point since the previous peak, and then fell that far from the top. That is relative to the surroundings, so a drifting empty
 * level does not matter, and the dips between touching elements in a clump (to only 2 to 4 counts above the empty level) still separate them, which a fixed threshold with hysteresis merged. A
 * peak is only reported once the signal has fallen `prominence` from it (a delay of one to three samples). Peaks closer than `minSepSec` are not counted twice.
 *
 * The 2026-11-14 run with 31 elements fed (ground truth: the B presses): 30 of 31 matched at 97 percent precision at 20 ms sampling; resampled to a 30 ms grid, prominence 6 without smoothing:
 * recall 96, precision 97. A median over 3 samples (smooth 3) removes single-sample spikes but at 30 ms it blurs two elements 0.1 s apart (recall 88 to 94), so the default is no smoothing.
 */
public final class PeakCounter {
    private final double prominence, minSepSec;
    private final int smooth;
    private final double[] win = new double[3];
    private int winN = 0;
    private boolean haveValley = false, haveCandidate = false;
    private double valley, candidate, candidateT, lastPeakT = -1e9;
    private double lastValley = Double.NaN;

    /** @param smooth 1 (none) or 3 (a median of the last three samples) */
    public PeakCounter(double prominence, int smooth, double minSepSec) {
        this.prominence = prominence;
        this.smooth = smooth >= 3 ? 3 : 1;
        this.minSepSec = minSepSec;
    }

    /**
     * One sample (time in seconds, any origin; proximity count).
     *
     * @return the height of a peak that has just been confirmed (its prominence above the lowest point before it, in counts), or NaN. The peak itself was at {@link #lastPeakTime()}.
     */
    public double push(double t, double x) {
        double v = x;
        if (smooth == 3) {
            win[winN % 3] = x;
            winN++;
            if (winN >= 3) {
                double a = win[0], b = win[1], c = win[2];
                v = Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));   // the median of three
            }
        }
        if (!haveValley) {
            valley = v;
            haveValley = true;
        }
        if (!haveCandidate) {
            valley = Math.min(valley, v);
            if (v - valley >= prominence) {
                candidate = v;
                candidateT = t;
                haveCandidate = true;
            }
            return Double.NaN;
        }
        if (v > candidate) {
            candidate = v;
            candidateT = t;
            return Double.NaN;
        }
        if (candidate - v >= prominence) {
            double height = candidate - valley;
            boolean counted = candidateT - lastPeakT >= minSepSec;
            if (counted) lastPeakT = candidateT;
            lastValley = valley;
            valley = v;
            haveCandidate = false;
            return counted ? height : Double.NaN;
        }
        return Double.NaN;
    }

    /** The lowest value before the last peak that was reported (the valley its rise started from), in counts. */
    public double lastValley() {
        return lastValley;
    }

    /** The time (as given to push) of the last peak that was reported. */
    public double lastPeakTime() {
        return lastPeakT;
    }
}
