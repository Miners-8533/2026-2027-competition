package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** The four building blocks: sequential, parallel, sleep, instant. */
public final class Actions {
    private Actions() { }

    /** A condition evaluated each loop. */
    public interface Condition { boolean get(); }

    /** Runs its children one after another. When a child finishes the next starts in the same loop, so a run of instant steps takes one loop. */
    public static Action sequence(Action... children) {
        final List<Action> list = new ArrayList<>(Arrays.asList(children));
        return new Action() {
            int i = 0;

            @Override public boolean run(TelemetryPacket p) {
                while (i < list.size()) {
                    if (list.get(i).run(p)) return true;
                    i++;
                }
                return false;
            }
        };
    }

    /** Runs every unfinished child each loop; finished when all are. */
    public static Action parallel(Action... children) {
        final Action[] list = children.clone();
        final boolean[] done = new boolean[list.length];
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                boolean more = false;
                for (int i = 0; i < list.length; i++) {
                    if (done[i]) continue;
                    if (list[i].run(p)) more = true;
                    else done[i] = true;
                }
                return more;
            }
        };
    }

    /** Finishes after a fixed time, counted from its first loop. */
    public static Action sleep(final double seconds) {
        return new Action() {
            long start = -1;

            @Override public boolean run(TelemetryPacket p) {
                if (start < 0) start = System.nanoTime();
                return (System.nanoTime() - start) / 1e9 < seconds;
            }
        };
    }

    /** Runs a piece of code once and finishes in the same loop. */
    public static Action instant(final Runnable code) {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                code.run();
                return false;
            }
        };
    }

    /** Finishes when the condition is true, or after the time limit. */
    public static Action waitUntil(final Condition done, final double timeoutS) {
        return new Action() {
            long start = -1;

            @Override public boolean run(TelemetryPacket p) {
                if (start < 0) start = System.nanoTime();
                return !done.get() && (System.nanoTime() - start) / 1e9 < timeoutS;
            }
        };
    }
}
