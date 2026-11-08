package com.qualcomm.robotcore.util;

/**
 * PC stub of the SDK's ElapsedTime for tools/dash/DashSim (only the calls the dash classes make). It reads the SIMULATED clock {@link #now}, which the simulation
 * advances each loop, so every timer in HiveDash, ParkDash, HoldPosition and PidController runs on simulated time.
 */
public class ElapsedTime {

    /** The simulated time, seconds. Set by the simulation; never read the wall clock. */
    public static double now = 0.0;

    private double start;

    public ElapsedTime() {
        start = now;
    }

    public void reset() {
        start = now;
    }

    public double seconds() {
        return now - start;
    }

    public double milliseconds() {
        return (now - start) * 1000.0;
    }
}
