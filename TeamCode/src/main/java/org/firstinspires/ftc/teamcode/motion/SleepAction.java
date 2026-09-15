package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Does nothing for a fixed number of seconds, then finishes. */
public final class SleepAction implements Action {

    private final double seconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;

    public SleepAction(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            started = true;
            timer.reset();
        }
        return timer.seconds() < seconds;
    }
}
