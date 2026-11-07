package com.acmerobotics.dashboard.telemetry;

import com.acmerobotics.dashboard.canvas.Canvas;

/** PC stub of FTC Dashboard's TelemetryPacket for tools/dash/DashSim: holds nothing. */
public class TelemetryPacket {
    private final Canvas canvas = new Canvas();

    public Canvas fieldOverlay() {
        return canvas;
    }

    public void put(String key, Object value) {
    }
}
