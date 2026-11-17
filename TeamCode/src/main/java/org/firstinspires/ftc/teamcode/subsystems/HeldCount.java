package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;

/**
 * Part of the coordinator (specs/mechanisms.md section 16, specs/element-counting.md sections 2.5, 2.8 and 5): how many
 * elements are held, by the CS5 entry count, and what each one is, in the order they came in. Pure logic.
 *
 * The count restarts at 0 when a salvo starts or a reject finishes. Entries while the intake ran OUT are elements
 * leaving and are not counted. A salvo that did not complete puts the count back.
 *
 * The load order is a text of one letter per held element: P pollen, R red nectar, B blue nectar, ? unknown. The first
 * letter is the element nearest the flywheel, the one a salvo launches first. Nothing acts on it.
 */
public final class HeldCount {
    private int held, totalEntries, tally;
    private int heldAtSalvo, takenAtSalvo;
    private final StringBuilder load = new StringBuilder();
    /** The moment of the entry behind each letter of the load order (NaN when the robot did not see it come in). */
    private final ArrayList<Double> loadT = new ArrayList<>();
    private String loadAtSalvo = "";
    // The stretches during which the intake ran OUT: {start, end}.
    private final double[][] out = new double[8][2];
    private int outN;
    private boolean outNow;

    /**
     * The starting load: 4 in an auto, the last opmode's saved count in the TeleOp, else 0.
     *
     * @param startLoad the load order saved with the count, or null when the kinds are not known (a preload): all "?"
     */
    public void start(int startHeld, String startLoad) {
        held = Math.max(0, startHeld);
        load.setLength(0);
        loadT.clear();
        loadAtSalvo = "";
        // SPEC GAP: the page does not say what happens when the saved load order and the saved count disagree in
        // length. The count rules: a longer text is cut, a shorter one is filled with "?".
        for (int i = 0; i < held; i++) {
            load.append(startLoad != null && i < startLoad.length() ? startLoad.charAt(i) : '?');
            loadT.add(Double.NaN);
        }
    }

    /** Call every update with whether the intake is running OUT. */
    public void intakeOut(boolean isOut, double t) {
        if (isOut) {
            if (!outNow) {
                outN = Math.min(out.length, outN + 1);
                System.arraycopy(out, 0, out, 1, out.length - 1);
                out[0] = new double[] {t, t};
            }
            out[0][1] = t;
        }
        outNow = isOut;
    }

    /** One entry seen by CS5 at a time. Returns whether it was added to the held count. */
    public boolean onEntry(double entryT) {
        totalEntries++;
        for (int i = 0; i < outN; i++) {
            if (entryT >= out[i][0] && entryT <= out[i][1] + MechParams.OUT_EXCLUDE_S) return false;
        }
        held++;
        load.append('?');       // its kind arrives when its pass closes
        loadT.add(entryT);
        return true;
    }

    /**
     * The kind of the entry counted at a moment has arrived. Returns whether it replaced that entry's "?" in the load
     * order; false when the entry is no longer in the load (it was not counted as held, or the track has been emptied since).
     */
    public boolean onKind(double entryT, char letter) {
        for (int i = 0; i < loadT.size(); i++) {
            if (loadT.get(i) == entryT) {
                load.setCharAt(i, letter);
                return true;
            }
        }
        return false;
    }

    /** The load order: one letter per held element, the first one in first. Empty when nothing is held. */
    public String load() { return load.toString(); }

    public int held() { return held; }
    public int totalEntries() { return totalEntries; }
    /** The launched tally: how many elements the robot takes itself to have launched. */
    public int tally() { return tally; }
    public boolean full() { return held >= MechParams.ENTRY_CAPACITY; }

    /**
     * A salvo starts: the tally grows by the number taken to be in the robot and the count restarts, which ends FULL.
     *
     * @param slotCount the slot count, used only with the "count by entries" switch off (the higher of the two)
     */
    public int salvoStarted(int slotCount) {
        int n = MechParams.COUNT_BY_ENTRIES ? held : Math.max(held, slotCount);
        n = Math.min(n, MechParams.ENTRY_CAPACITY);
        heldAtSalvo = held;
        takenAtSalvo = n;
        loadAtSalvo = load.toString();
        tally += n;
        held = 0;
        clearLoad();
        return n;
    }

    /** The salvo did not complete: put the count and the tally back as if nothing left the robot, plus whatever entered since. */
    public void salvoIncomplete() {
        held += heldAtSalvo;
        tally -= takenAtSalvo;
        // Its elements go back in front of whatever has entered since.
        load.insert(0, loadAtSalvo);
        for (int i = 0; i < loadAtSalvo.length(); i++) loadT.add(0, Double.NaN);
        heldAtSalvo = 0;
        takenAtSalvo = 0;
        loadAtSalvo = "";
    }

    public void reset() {
        held = 0;
        clearLoad();
    }

    private void clearLoad() {
        load.setLength(0);
        loadT.clear();
    }
}
