package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Arrays;
import java.util.Locale;

/**
 * Says WHAT each element counted by CS5 is (pollen, red nectar, blue nectar), from CS5's color channels while the element passes the sensor (2026-11-17, the user: "start implementing the
 * classification with CS5 so it makes a load order"; not run on the robot). A pure class (no FTC classes): EntryCounter's thread feeds it every sample, and tools/entry/EntryClassifierCheck.java
 * replays a recorded run through THE SAME code.
 *
 * WHAT IT MEASURES. The sensor's green, blue and red counts rise while an element is in front of it. The rise over the level just before the element came (the REFERENCE: the median of the last
 * few color samples taken with nothing in front) is summed over the pass, and the three sums' SHARES of their total say the kind. The 2026-11-14 13:27 CS Pass Study (31 pollen fed, the only run
 * with CS5's color recorded while elements passed) gives for pollen: green 0.52 to 0.65, blue 0.00 to 0.13, red 0.31 to 0.46, a rise of 60 to 140 green counts per sample against an empty level
 * that wanders by about 15. NO NECTAR HAS BEEN RECORDED PASSING CS5 WITH ITS COLOR, so the two nectar limits are a first guess from the colors themselves (a red object returns mostly red, a blue
 * one mostly blue) and wait for a run with nectar: every result carries its shares so the limits can be read off the log.
 *
 * A PASS begins when the proximity is `near` or more above the empty level (below the count's own rise threshold, so the color read is already on when the entry is counted) and ends when it has
 * been below that for more than `tailSamples` samples (the color lags the proximity: it is a 25 ms average). A pass in which no entry was counted gives nothing. A pass that lasts `maxSec` after
 * its entry (an element parked under the sensor) is classified with what it has, and no more color is asked for until it has gone. A second entry counted before the first one's pass has closed
 * (touching elements whose signal dipped to the re-arm level and no further) closes the first with what it has.
 */
public final class EntryClassifier {

    public enum Kind {
        POLLEN('P'), RED_NECTAR('R'), BLUE_NECTAR('B'), UNKNOWN('?');

        public final char letter;

        Kind(char letter) {
            this.letter = letter;
        }

        public static Kind ofLetter(char c) {
            for (Kind k : values()) if (k.letter == c) return k;
            return UNKNOWN;
        }
    }

    /** The limits of the kind rule, on the shares of the summed rise. */
    public static final class Limits {
        public final double minSignal, redMin, blueMin, pollenGreenMin, pollenBlueMax;

        public Limits(double minSignal, double redMin, double blueMin, double pollenGreenMin, double pollenBlueMax) {
            this.minSignal = minSignal;
            this.redMin = redMin;
            this.blueMin = blueMin;
            this.pollenGreenMin = pollenGreenMin;
            this.pollenBlueMax = pollenBlueMax;
        }
    }

    /** One classified entry. */
    public static final class Result {
        public final long entry;          // the entry's number (EntryCounter.Entry.number)
        public final Kind kind;
        public final double green, blue, red;   // the shares of the summed rise (0 when there was no signal)
        public final double signal;       // the summed rise of the three channels, counts
        public final int samples;         // color samples in the sum
        public final double seconds;      // from the start of the pass to the result
        public final String why;          // "" or why it is UNKNOWN

        Result(long entry, Kind kind, double green, double blue, double red, double signal, int samples, double seconds, String why) {
            this.entry = entry;
            this.kind = kind;
            this.green = green;
            this.blue = blue;
            this.red = red;
            this.signal = signal;
            this.samples = samples;
            this.seconds = seconds;
            this.why = why;
        }

        public String text() {
            return String.format(Locale.US, "%s (shares green %.2f, blue %.2f, red %.2f; rise %.0f counts over %d color samples, %.2f s%s)", kind, green, blue, red, signal, samples, seconds,
                    why.isEmpty() ? "" : "; " + why);
        }
    }

    private final double near, maxSec;
    private final int tailSamples;
    private final Limits limits;
    private final double[][] ref;     // the latest color samples with nothing in front: {green, blue, red}
    private int refCount = 0, refNext = 0;

    private boolean active = false, hold = false;
    private long entry = 0L;
    private double tStart = 0.0, tEntry = 0.0;
    private double refG = Double.NaN, refB = Double.NaN, refR = Double.NaN;
    private double sumG = 0.0, sumB = 0.0, sumR = 0.0;
    private int n = 0, tail = 0;

    /**
     * @param near        a pass begins at this many proximity counts above the empty level
     * @param tailSamples a pass ends when the proximity has been below `near` for more than this many samples
     * @param maxSec      a pass is closed this long after its entry (or after its start, when no entry came)
     * @param refSamples  how many of the latest empty color samples make the reference (their median)
     */
    public EntryClassifier(double near, int tailSamples, double maxSec, int refSamples, Limits limits) {
        this.near = near;
        this.tailSamples = Math.max(0, tailSamples);
        this.maxSec = maxSec;
        this.ref = new double[Math.max(1, refSamples)][3];
        this.limits = limits;
    }

    /** True when the next sample should carry the color: an element is in front of the sensor and has not been classified yet. */
    public boolean wantsColor() {
        return active;
    }

    /**
     * One sample of the counter's thread.
     *
     * @param t      seconds
     * @param above  the proximity above the empty level, or NaN while the empty level is not known
     * @param entry  the number of the entry counted ON THIS SAMPLE, or 0
     * @param color  true when green, blue and red hold a NEW color reading
     * @return the result of an entry whose pass has just closed, or null
     */
    public Result push(double t, double above, long entry, boolean color, int green, int blue, int red) {
        boolean isNear = !Double.isNaN(above) && above >= near;
        Result out = null;
        if (hold) {   // classified already (or given up on): wait for it to leave
            if (entry == 0L) {
                if (isNear) tail = 0;
                else if (++tail > tailSamples) hold = false;
                return null;
            }
            hold = false;   // a new entry: the signal had come back down to the re-arm level
        }
        if (!active) {
            if (!isNear && entry == 0L) {
                if (color) addRef(green, blue, red);
                return null;
            }
            begin(t);
        }
        if (entry != 0L) {
            if (this.entry != 0L) {   // the first one's pass never closed
                out = finish(t);
                begin(t);
            }
            this.entry = entry;
            tEntry = t;
        }
        if (color && !Double.isNaN(refG)) {
            sumG += green - refG;
            sumB += blue - refB;
            sumR += red - refR;
            n++;
        }
        tail = isNear ? 0 : tail + 1;
        if (tail > tailSamples) {
            Result r = this.entry != 0L ? finish(t) : null;
            active = false;
            return out != null ? out : r;
        }
        if (t - (this.entry != 0L ? tEntry : tStart) >= maxSec) {
            Result r = this.entry != 0L ? finish(t) : null;
            active = false;
            hold = true;
            tail = 0;
            return out != null ? out : r;
        }
        return out;
    }

    private void begin(double t) {
        active = true;
        entry = 0L;
        tStart = t;
        tail = 0;
        n = 0;
        sumG = sumB = sumR = 0.0;
        refG = median(0);
        refB = median(1);
        refR = median(2);
    }

    private Result finish(double t) {
        Result r = classify(entry, sumG, sumB, sumR, n, t - tStart, limits, Double.isNaN(refG) ? "no empty color level yet" : "");
        entry = 0L;
        return r;
    }

    private void addRef(int g, int b, int r) {
        ref[refNext][0] = g;
        ref[refNext][1] = b;
        ref[refNext][2] = r;
        refNext = (refNext + 1) % ref.length;
        if (refCount < ref.length) refCount++;
    }

    private double median(int channel) {
        if (refCount == 0) return Double.NaN;
        double[] v = new double[refCount];
        for (int i = 0; i < refCount; i++) v[i] = ref[i][channel];
        Arrays.sort(v);
        return refCount % 2 == 1 ? v[refCount / 2] : 0.5 * (v[refCount / 2 - 1] + v[refCount / 2]);
    }

    /** The empty color level in use now, "green/blue/red", or "" when there is none yet (for the log). */
    public String referenceText() {
        if (refCount == 0) return "";
        return String.format(Locale.US, "%.0f/%.0f/%.0f", median(0), median(1), median(2));
    }

    /**
     * THE KIND RULE, on the summed rise of each channel over the pass (a channel that fell counts as 0). In this order: too little signal (or no color sample at all) is UNKNOWN; red's share at
     * `redMin` or more is RED NECTAR; blue's share at `blueMin` or more is BLUE NECTAR; green's share at `pollenGreenMin` or more with blue's at `pollenBlueMax` or less is POLLEN; anything else is
     * UNKNOWN.
     */
    public static Result classify(long entry, double sumG, double sumB, double sumR, int samples, double seconds, Limits lim, String note) {
        double g = Math.max(0.0, sumG), b = Math.max(0.0, sumB), r = Math.max(0.0, sumR);
        double total = g + b + r;
        if (samples < 1) return new Result(entry, Kind.UNKNOWN, 0.0, 0.0, 0.0, total, samples, seconds, note.isEmpty() ? "no color sample in the pass" : note);
        if (total < lim.minSignal || total <= 0.0) {
            return new Result(entry, Kind.UNKNOWN, total > 0.0 ? g / total : 0.0, total > 0.0 ? b / total : 0.0, total > 0.0 ? r / total : 0.0, total, samples, seconds,
                    String.format(Locale.US, "the rise is under %.0f counts", lim.minSignal));
        }
        double gs = g / total, bs = b / total, rs = r / total;
        Kind kind;
        String why = "";
        if (rs >= lim.redMin) kind = Kind.RED_NECTAR;
        else if (bs >= lim.blueMin) kind = Kind.BLUE_NECTAR;
        else if (gs >= lim.pollenGreenMin && bs <= lim.pollenBlueMax) kind = Kind.POLLEN;
        else {
            kind = Kind.UNKNOWN;
            why = "the shares fit no kind";
        }
        return new Result(entry, kind, gs, bs, rs, total, samples, seconds, why);
    }
}
