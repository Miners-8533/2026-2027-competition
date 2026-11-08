package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/** PC stub of FTC Dashboard for tools/dash/DashSim: packets go nowhere. */
public class FtcDashboard {
    private static final FtcDashboard INSTANCE = new FtcDashboard();

    public static FtcDashboard getInstance() {
        return INSTANCE;
    }

    public void sendTelemetryPacket(TelemetryPacket packet) {
    }
}
