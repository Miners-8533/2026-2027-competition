package org.firstinspires.ftc.teamcode.util;

/** One monotonic clock for the mechanisms and the CS5 thread, so their times can be compared. Seconds. */
public final class Clock {
    private Clock() { }

    public static double now() {
        return System.nanoTime() / 1e9;
    }
}
