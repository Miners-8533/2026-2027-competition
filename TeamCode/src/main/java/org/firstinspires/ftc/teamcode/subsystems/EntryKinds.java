package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;
import java.util.Locale;

/**
 * The kind of each CS5 entry (specs/element-counting.md section 2.8): pollen, red nectar, blue nectar or unknown, from
 * the color CS5 sees while the element passes. One pure piece of logic with no robot dependencies, like the counting
 * rule, so a recorded run can be replayed through exactly the code the robot runs.
 *
 * The sampling thread asks {@link #wantColor} before each read (the longer read that also returns the color, or the
 * small proximity read), tells {@link #readDone} which read it made, and feeds every sample to {@link #sample}. A pass
 * is the time one element is in front of the sensor; the rise of each color channel over the empty color level is
 * summed over the pass, and the three sums' shares of their total name the kind.
 *
 * Nothing acts on the kinds: they make the load order, which is logged and shown.
 */
@Config
public final class EntryKinds {
    /** Off: the thread reads the proximity alone and there are no kinds. */
    public static boolean ENABLED = true;
    /** A pass begins at this many counts above the empty level, and ends after more than END_SAMPLES samples below it. */
    public static int PASS_COUNTS = 2, PASS_END_SAMPLES = 2;
    /** A pass is closed this long after its entry if the element is still in front; with no entry, this long after it began. */
    public static double CLOSE_AFTER_ENTRY_S = 0.5;
    /** Outside a pass and the first second, the color is read on one read in this many. */
    public static int EMPTY_READ_EVERY = 8;
    /** The least rise (green + blue + red, summed over the pass) that can be named. */
    public static double MIN_RISE = 60;
    /** Red nectar and blue nectar: FIRST GUESSES, no nectar has been recorded passing CS5 with its color. */
    public static double RED_SHARE = 0.55, BLUE_SHARE = 0.40;
    /** Pollen, measured: green 0.52 to 0.65, blue 0.00 to 0.13. */
    public static double POLLEN_GREEN_SHARE = 0.45, POLLEN_BLUE_SHARE_MAX = 0.20;

    /** The empty color level is the median of this many empty color samples. */
    private static final int EMPTY_SAMPLES = 3;

    public enum Kind {
        POLLEN('P'), RED_NECTAR('R'), BLUE_NECTAR('B'), UNKNOWN('?');

        /** The kind's letter in the load order. */
        public final char letter;

        Kind(char letter) {
            this.letter = letter;
        }
    }

    /** The kind of one entry, with what it was decided on. */
    public static final class Result {
        /** The entry's number, as the counting rule gave it. */
        public final int entry;
        /** The moment of the entry this kind belongs to, as the counting rule gave it. */
        public final double entryT;
        public final Kind kind;
        /** Each channel's share of the total rise (0 when there was no rise). */
        public final double green, blue, red;
        /** Green + blue + red, summed over the pass. */
        public final double rise;
        /** How many color values the pass held. */
        public final int values;
        /** From the beginning of the pass to this result. */
        public final double seconds;
        /** Why the kind is unknown, or empty. */
        public final String why;

        Result(int entry, double entryT, double g, double b, double r, int values, double seconds, boolean hadEmptyLevel) {
            this.entry = entry;
            this.entryT = entryT;
            g = Math.max(0, g);         // a channel whose sum is negative counts as 0
            b = Math.max(0, b);
            r = Math.max(0, r);
            rise = g + b + r;
            green = rise > 0 ? g / rise : 0;
            blue = rise > 0 ? b / rise : 0;
            red = rise > 0 ? r / rise : 0;
            this.values = values;
            this.seconds = seconds;
            kind = classify(g, b, r, values);
            if (kind != Kind.UNKNOWN) why = "";
            else if (values <= 0) why = hadEmptyLevel ? "no color sample in the pass" : "no empty color level yet";
            else if (rise < MIN_RISE) why = String.format(Locale.US, "the rise is under %.0f counts", MIN_RISE);
            else why = "the shares fit no kind";
        }

        /** The result as the CS5 KIND line of the log gives it, after the entry's number. */
        public String text() {
            return String.format(Locale.US, "%s (shares green %.2f, blue %.2f, red %.2f; rise %.0f counts over %d color samples, %.2f s%s)",
                    kind, green, blue, red, rise, values, seconds, why.isEmpty() ? "" : "; " + why);
        }
    }

    /**
     * The kind rule. The three sums' shares of their total are tested in this order: no color value or too little rise
     * is unknown; then red nectar, blue nectar, pollen; anything else is unknown.
     */
    public static Kind classify(double greenSum, double blueSum, double redSum, int values) {
        double g = Math.max(0, greenSum), b = Math.max(0, blueSum), r = Math.max(0, redSum);
        double total = g + b + r;
        if (values <= 0 || total < MIN_RISE) return Kind.UNKNOWN;
        if (r / total >= RED_SHARE) return Kind.RED_NECTAR;
        if (b / total >= BLUE_SHARE) return Kind.BLUE_NECTAR;
        if (g / total >= POLLEN_GREEN_SHARE && b / total <= POLLEN_BLUE_SHARE_MAX) return Kind.POLLEN;
        return Kind.UNKNOWN;
    }

    /** PARKED: no pass is open and no empty color sample is taken, until the element has gone or an entry is counted. */
    private enum State { IDLE, PASS, PARKED }

    private State state = State.IDLE;
    private int smallReads;

    // The last empty color samples: {green, blue, red} each.
    private final double[][] empties = new double[EMPTY_SAMPLES][];
    private int emptyN, emptyAt;

    // The pass in hand.
    private double[] emptyLevel;        // fixed at the start of the pass; null when there is no empty color sample yet
    private double passT;
    private int entryN;                 // the number of the pass's entry, 0 while it has none
    private double entryT = Double.NaN;
    private double sumG, sumB, sumR;
    private int values, below;

    /**
     * Whether the next read is to be the longer one that also returns the color: through the first second of a run,
     * while the empty proximity level is being measured; on every read of a pass; and otherwise on one read in 8: seven
     * small reads follow each longer read. The kinds need edge counting.
     *
     * @param level the counting rule's empty proximity level (NaN during the first second)
     */
    public boolean wantColor(double level) {
        if (!ENABLED || !EntryRule.EDGE_COUNTING) return false;
        if (Double.isNaN(level)) return true;
        if (state == State.PASS) return true;
        return smallReads >= Math.max(1, EMPTY_READ_EVERY) - 1;
    }

    /**
     * The thread tells which read it made. A longer read that failed is not told: the next read is a longer read again.
     */
    public void readDone(boolean longer) {
        smallReads = longer ? 0 : smallReads + 1;
    }

    /**
     * Feeds one sample.
     *
     * @param level   the counting rule's empty proximity level (NaN during the first second)
     * @param entry   the number of the entry the counting rule counted on this sample, or 0
     * @param color   {green, blue, red} when this read returned a color value the sensor reports as new, else null
     * @return the kind of an entry whose pass this sample closed, or null
     */
    public Result sample(double t, int prox, double level, int entry, int[] color) {
        boolean front = !Double.isNaN(level) && prox >= level + PASS_COUNTS;
        boolean counted = entry > 0;
        if (state == State.IDLE) {
            if (!front && !counted) {
                // An empty color sample: every new color value of the first second (the intake must be clear for it),
                // and afterwards every one read with the proximity under the pass level.
                if (color != null) addEmpty(color);
                return null;
            }
            begin(t);
        } else if (state == State.PARKED) {
            if (!counted) {
                // The one read in 8 goes on while it is parked; its color is not used.
                if (front) below = 0;
                else if (++below > PASS_END_SAMPLES) state = State.IDLE;
                return null;
            }
            begin(t);       // an entry is counted: a new pass begins on this sample
        }

        Result closed = null;
        if (counted) {
            if (entryN > 0) {
                // A second entry before the first one's pass has ended: the first is closed at this moment and a new
                // pass begins for the second, from the same empty color level. A color value read on this very sample
                // belongs to the second.
                closed = close(t);
                begin(t);
            }
            entryN = entry;
            entryT = t;
        }
        // A value read on the sample that ends or closes the pass is added before it ends.
        if (color != null && emptyLevel != null) {
            sumG += color[0] - emptyLevel[0];
            sumB += color[1] - emptyLevel[1];
            sumR += color[2] - emptyLevel[2];
            values++;
        }
        if (front) {
            below = 0;
        } else if (++below > PASS_END_SAMPLES) {
            // The element has gone. The color lags the proximity, so the samples just after it still carried it.
            Result r = entryN > 0 ? close(t) : null;        // a pass in which no entry was counted gives nothing
            state = State.IDLE;
            return closed != null ? closed : r;
        }
        if (t - (entryN > 0 ? entryT : passT) >= CLOSE_AFTER_ENTRY_S) {
            // Still in front half a second after its entry: closed with what it has. With no entry half a second after
            // the pass began: dropped. Either way what is in front is parked.
            Result r = entryN > 0 ? close(t) : null;
            state = State.PARKED;
            below = 0;
            return closed != null ? closed : r;
        }
        return closed;
    }

    private void begin(double t) {
        state = State.PASS;
        emptyLevel = emptyLevel();
        passT = t;
        entryN = 0;
        entryT = Double.NaN;
        sumG = sumB = sumR = 0;
        values = 0;
        below = 0;
    }

    private Result close(double t) {
        Result r = new Result(entryN, entryT, sumG, sumB, sumR, values, t - passT, emptyLevel != null);
        entryN = 0;
        entryT = Double.NaN;
        return r;
    }

    private void addEmpty(int[] color) {
        empties[emptyAt] = new double[] {color[0], color[1], color[2]};
        emptyAt = (emptyAt + 1) % EMPTY_SAMPLES;
        emptyN = Math.min(EMPTY_SAMPLES, emptyN + 1);
    }

    /**
     * The median of the last 3 empty color samples, each channel on its own; of two, their mean; of one, that sample;
     * null before the first.
     */
    private double[] emptyLevel() {
        if (emptyN == 0) return null;
        double[] out = new double[3];
        double[] v = new double[emptyN];
        for (int c = 0; c < 3; c++) {
            for (int i = 0; i < emptyN; i++) v[i] = empties[i][c];
            Arrays.sort(v);
            out[c] = emptyN % 2 == 1 ? v[emptyN / 2] : (v[emptyN / 2 - 1] + v[emptyN / 2]) / 2;
        }
        return out;
    }

    /** The empty color level as it stands now, "green/blue/red" in whole counts, or "-" before the first empty color sample. */
    public String emptyLevelText() {
        double[] e = emptyLevel();
        return e == null ? "-" : String.format(Locale.US, "%.0f/%.0f/%.0f", e[0], e[1], e[2]);
    }
}
