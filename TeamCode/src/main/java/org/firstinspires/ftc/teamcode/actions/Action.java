package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * A unit of work that advances a little on every control loop (specs/motion.md section 1).
 * The team's own small replacement for the Road Runner action classes.
 */
public interface Action {
    /**
     * Runs one loop.
     *
     * @param packet the loop's Dashboard telemetry packet, for debug values
     * @return true if it needs another loop
     */
    boolean run(TelemetryPacket packet);
}
