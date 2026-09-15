package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * A unit of autonomous work that runs a little bit each control loop.
 *
 * This is the team's own lightweight replacement for the Road Runner {@code Action}
 * (last season's autons were built on RR Actions). Same convention: {@link #run} is
 * called once per loop and returns whether the action still needs more time.
 *
 * Compose with {@link SequentialAction}, {@link ParallelAction}, {@link SleepAction},
 * {@link InstantAction}. Drive it with {@link Actions#runBlocking}.
 */
public interface Action {

    /**
     * Do one loop's worth of work.
     *
     * @param packet dashboard telemetry packet for this loop - put debug values here;
     *               they show up in the FTC Dashboard graph/telemetry.
     * @return {@code true} if this action needs to run again, {@code false} when finished.
     */
    boolean run(TelemetryPacket packet);
}
