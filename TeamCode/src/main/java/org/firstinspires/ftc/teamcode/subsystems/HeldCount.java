package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Part of the coordinator (specs/mechanisms.md section 16, specs/element-counting.md sections 2.5 and 5): how many
 * elements are held, by the CS5 entry count. Pure logic.
 *
 * The count restarts at 0 when a salvo starts or a reject finishes. Entries while the intake ran OUT are elements
 * leaving and are not counted. A salvo that did not complete puts the count back.
 */
public final class HeldCount {
    private int held, totalEntries, tally;
    private int heldAtSalvo, takenAtSalvo;
    // The stretches during which the intake ran OUT: {start, end}.
    private final double[][] out = new double[8][2];
    private int outN;
    private boolean outNow;

    /** The starting load: 4 in an auto, the last opmode's saved count in the TeleOp, else 0. */
    public void start(int startHeld) {
        held = Math.max(0, startHeld);
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
        return true;
    }

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
        tally += n;
        held = 0;
        return n;
    }

    /** The salvo did not complete: put the count and the tally back as if nothing left the robot, plus whatever entered since. */
    public void salvoIncomplete() {
        held += heldAtSalvo;
        tally -= takenAtSalvo;
        heldAtSalvo = 0;
        takenAtSalvo = 0;
    }

    public void reset() { held = 0; }
}
