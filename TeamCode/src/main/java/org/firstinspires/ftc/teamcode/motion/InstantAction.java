package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Runs a piece of code once and finishes immediately (same loop). Use for one-launch
 * state changes inside a sequence, e.g.
 * {@code new InstantAction(() -> launcher.setTargetVelocity(700))}.
 */
public final class InstantAction implements Action {

    private final Runnable action;

    public InstantAction(Runnable action) {
        this.action = action;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        action.run();
        return false;
    }
}
