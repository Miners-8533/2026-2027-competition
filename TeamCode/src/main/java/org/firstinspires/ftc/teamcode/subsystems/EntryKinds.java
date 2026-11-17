package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;

/**
 * The kind of each CS5 entry (specs/element-counting.md section 2.8): pollen, red nectar, blue nectar or unknown, from
 * the color CS5 sees while the element passes. One pure piece of logic with no robot dependencies, like the counting
 * rule, so a recorded run can be replayed through exactly the code the robot runs.
 *
 * The sampling thread asks {@link #wantColor} before each read (the longer read that also returns the color, or the
 * small proximity read) and feeds every sample to {@link #sample}. A pass is the time one element is in front of the
 * sensor; the rise of each color channel over the empty color level is summed over the pass, and the three sums'
 * shares of their total name the kind.
 *
 * Nothing acts on the kinds: they make the load order, which is logged and shown.
 */
@Config
public final class EntryKinds {
    /** Off: the thread reads the proximity alone and there are no kinds. */
    public static boolean ENABLED = true;
    /** A pass begins at this many counts above the empty level, and ends after more than END_SAMPLES samples below it. */
    public static int PASS_COUNTS = 2, PASS_END_SAMPLES = 2;
    /** A pass is closed this long after its entry if the element is still in front. */
    public static double CLOSE_AFTER_ENTRY_S = 0.5;
    /** The color is read on every sample through this much of the start of a run. */
    public static double FIRST_S = 1.0;
    /** With nothing in front, the color is read on one read in this many. */
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
        POLLEN('P', "pollen"), RED_NECTAR('R', "red nectar"), BLUE_NECTAR('B', "blue nectar"), UNKNOWN('?', "unknown");

        /** The kind's letter in the load order. */
        public final char letter;
        public final String text;

        Kind(char letter, String text) {
            this.letter = letter;
            this.text = text;
        }
    }

    /** The kind of one entry, with what it was decided on. */
    public static final class Result {
        /** The moment of the entry this kind belongs to, as the counting rule gave it. */
        public final double entryT;
        public final Kind kind;
        /** Each channel's share of the total rise (0 when there was no rise). */
        public final double green, blue, red;
        /** Green + blue + red, summed over the pass. */
        public final double rise;
        /** How many color values the pass held. */
        public final int values;

        Result(double entryT, double g, double b, double r, int values) {
            this.entryT = entryT;
            g = Math.max(0, g);         // a channel whose sum is negative counts as 0
            b = Math.max(0, b);
            r = Math.max(0, r);
            rise = g + b + r;
            green = rise > 0 ? g / rise : 0;
            blue = rise > 0 ? b / rise : 0;
            red = rise > 0 ? r / rise : 0;
            this.values = values;
            kind = classify(g, b, r, values);
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

    private enum State { IDLE, PASS, PARKED }

    private State state = State.IDLE;
    private double startT = Double.NaN;
    private int emptyReads;

    // The last color values read with nothing in front: {green, blue, red} each.
    private final double[][] empties = new double[EMPTY_SAMPLES][];
    private int emptyN, emptyAt;

    // The pass in hand.
    private double[] emptyLevel;        // fixed at the start of the pass; null when no empty color has been read yet
    private double entryT = Double.NaN;
    private double sumG, sumB, sumR;
    private int values, below;

    /**
     * Whether the next read is to be the longer one that also returns the color: through the first second of a run;
     * from the moment the proximity is above the pass level until the pass is closed; and, with nothing in front, on
     * one read in 8. The kinds need edge counting.
     */
    public boolean wantColor(double t) {
        if (!ENABLED || !EntryRule.EDGE_COUNTING) return false;
        if (Double.isNaN(startT)) startT = t;
        if (t - startT < FIRST_S) return true;
        if (state == State.PASS) return true;
        if (state == State.PARKED) return false;      // no more color is asked for until the element has gone
        if (++emptyReads < EMPTY_READ_EVERY) return false;
        emptyReads = 0;
        return true;
    }

    /**
     * Feeds one sample.
     *
     * @param level   the counting rule's empty proximity level (NaN during the first second)
     * @param entry   the counting rule counted an entry on this sample
     * @param color   {green, blue, red} when this read returned a color value the sensor reports as new, else null
     * @return the kind of an entry whose pass this sample closed, or null
     */
    public Result sample(double t, int prox, double level, boolean entry, int[] color) {
        boolean front = !Double.isNaN(level) && prox >= level + PASS_COUNTS;
        if (state == State.IDLE) {
            if (!front && !entry) {
                // SPEC GAP: the page does not say what the color values of the first second are for. They are taken as
                // empty color samples here (the intake must be clear for that second, as for the proximity level).
                if (color != null) addEmpty(color);
                return null;
            }
            begin();
        } else if (state == State.PARKED) {
            if (!entry) {
                if (front) below = 0;
                else if (++below > PASS_END_SAMPLES) state = State.IDLE;
                return null;
            }
            begin();        // another element has been counted behind the one that stayed
        }

        Result closed = null;
        if (entry) {
            if (!Double.isNaN(entryT)) {
                // A second entry before the first one's pass has ended: the first is closed at this moment and a new
                // pass begins for the second.
                // SPEC GAP: the page does not say which of the two the color value of this very sample goes to. It
                // goes to the second.
                closed = close();
                begin();
            }
            entryT = t;
        }
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
            Result r = Double.isNaN(entryT) ? null : close();    // a pass in which no entry was counted gives nothing
            state = State.IDLE;
            return closed != null ? closed : r;
        }
        if (closed == null && !Double.isNaN(entryT) && t - entryT >= CLOSE_AFTER_ENTRY_S) {
            // Still in front half a second after its entry: closed with what it has.
            Result r = close();
            state = State.PARKED;
            below = 0;
            return r;
        }
        return closed;
    }

    private void begin() {
        state = State.PASS;
        emptyLevel = emptyLevel();
        entryT = Double.NaN;
        sumG = sumB = sumR = 0;
        values = 0;
        below = 0;
    }

    private Result close() {
        Result r = new Result(entryT, sumG, sumB, sumR, values);
        entryT = Double.NaN;
        return r;
    }

    private void addEmpty(int[] color) {
        empties[emptyAt] = new double[] {color[0], color[1], color[2]};
        emptyAt = (emptyAt + 1) % EMPTY_SAMPLES;
        emptyN = Math.min(EMPTY_SAMPLES, emptyN + 1);
    }

    /** The median, channel by channel, of the last empty color samples; null before the first. */
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
}
