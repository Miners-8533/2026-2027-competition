package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs all of its child actions at once, each loop. Finished only when EVERY child
 * has finished. A child that finishes early simply stops being ticked.
 *
 * Typical use: drive a path while spinning up the flywheel -
 * {@code new ParallelAction(driveToPose, spinUp)}.
 */
public final class ParallelAction implements Action {

    private List<Action> running;

    public ParallelAction(Action... actions) {
        this.running = new ArrayList<>(Arrays.asList(actions));
    }

    public ParallelAction(List<Action> actions) {
        this.running = new ArrayList<>(actions);
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        List<Action> stillRunning = new ArrayList<>(running.size());
        for (Action action : running) {
            if (action.run(packet)) {
                stillRunning.add(action);
            }
        }
        running = stillRunning;
        return !running.isEmpty();
    }
}
