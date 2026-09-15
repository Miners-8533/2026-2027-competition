package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs its child actions one after another. Finished when the last child finishes.
 *
 * If a child finishes on a given loop, the next child starts on the SAME loop, so a
 * run of {@link InstantAction}s (or other instantly-done actions) all fire in one tick.
 */
public final class SequentialAction implements Action {

    private final List<Action> actions;
    private int index = 0;

    public SequentialAction(Action... actions) {
        this.actions = new ArrayList<>(Arrays.asList(actions));
    }

    public SequentialAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        while (index < actions.size()) {
            if (actions.get(index).run(packet)) {
                return true; // current child still working
            }
            index++; // child done - fall through to start the next one this same loop
        }
        return false;
    }
}
