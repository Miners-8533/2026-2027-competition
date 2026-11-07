package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/**
 * PC stub of the robot's RunLog for tools/dash/DashSim: {@link #event} collects the events (with the simulated time) instead of writing a file, so the simulation can read the
 * dash's own "HIVE DASH arrived ..." lines back. Only the call the dash classes make is here.
 */
public final class RunLog {

    public static final class Event {
        public final double t;
        public final String text;

        Event(double t, String text) {
            this.t = t;
            this.text = text;
        }
    }

    public static final List<Event> EVENTS = new ArrayList<>();
    /** True: also print each event as it happens. */
    public static boolean echo = false;

    private RunLog() {}

    public static void event(String text) {
        double t = com.qualcomm.robotcore.util.ElapsedTime.now;
        EVENTS.add(new Event(t, text));
        if (echo) System.out.printf("E,%.3f,%s%n", t, text);
    }

    public static void clear() {
        EVENTS.clear();
    }
}
