package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.SlotCounter;

import java.util.Locale;

/**
 * Raw readings of CS1 to CS4 for the two color sensor opmodes: brightness (alpha), distance and the three color
 * shares, the same quantities the element counter stores as its empty readings (specs/element-counting.md).
 */
final class ColorReader {
    final RevColorSensorV3[] sensors = new RevColorSensorV3[4];

    /** Null unless all four sensors are configured. */
    static ColorReader create(HardwareMap hw) {
        ColorReader c = new ColorReader();
        for (int i = 0; i < 4; i++) {
            c.sensors[i] = hw.tryGet(RevColorSensorV3.class, ChassisProfile.SLOT_SENSORS[i]);
            if (c.sensors[i] == null) return null;
            try {
                c.sensors[i].setGain((float) SlotCounter.SOFTWARE_GAIN);
            } catch (Throwable ignored) { }
        }
        return c;
    }

    /** One reading of sensor i (0 to 3): {alpha, cm, red share, green share, blue share}, or null when it did not answer. */
    double[] read(int i) {
        try {
            NormalizedRGBA c = sensors[i].getNormalizedColors();
            double d = sensors[i].getDistance(DistanceUnit.CM);
            double sum = c.red + c.green + c.blue;
            if (sum <= 0 || Double.isNaN(d)) return null;
            return new double[] {c.alpha, d, c.red / sum, c.green / sum, c.blue / sum};
        } catch (Throwable e) {
            return null;
        }
    }

    void led(boolean on) {
        for (RevColorSensorV3 s : sensors) {
            try {
                s.enableLed(on);
            } catch (Throwable ignored) { }
        }
    }

    static String text(double[] r) {
        return r == null ? "no answer" : String.format(Locale.US, "alpha %.3f, %.2f cm, shares %.3f/%.3f/%.3f", r[0], r[1], r[2], r[3], r[4]);
    }

    /** A running mean of readings. */
    static final class Mean {
        final double[] sum = new double[5];
        int n;

        void add(double[] r) {
            if (r == null) return;
            for (int k = 0; k < 5; k++) sum[k] += r[k];
            n++;
        }

        double[] get() {
            if (n == 0) return null;
            double[] m = new double[5];
            for (int k = 0; k < 5; k++) m[k] = sum[k] / n;
            return m;
        }
    }
}
