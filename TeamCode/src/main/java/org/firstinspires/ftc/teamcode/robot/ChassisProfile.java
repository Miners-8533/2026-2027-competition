package org.firstinspires.ftc.teamcode.robot;

import android.os.Build;
import android.provider.Settings;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Everything that differs between chassis, chosen from the identity of the Control Hub (specs/drive.md section 1,
 * robot_design/). One build runs on every chassis the team has.
 *
 * Three profiles: ROBOT_1, the competition robot, and two earlier drive bases with no mechanisms tuned and no hub
 * registered, PROTOTYPE and DRAFT_1.
 *
 * SPEC GAP: the page says a profile still carries a direction for the second flywheel motor, which was removed from
 * the robot, and gives no value for it. Nothing here drives that motor, so the profiles do not carry it.
 */
public final class ChassisProfile {
    // Hub configuration names (robot_design/hub-configuration.md). The pusher and ramp keep their older names.
    public static final String LEFT_FRONT = "leftFront", LEFT_BACK = "leftBack", RIGHT_FRONT = "rightFront", RIGHT_BACK = "rightBack";
    public static final String PINPOINT = "pinpoint";
    public static final String PUSHER = "gate", FLYWHEEL = "fly_motor_enc", INTAKE = "intake";
    public static final String RAMP_LEFT = "depolLeft", RAMP_RIGHT = "depolRight", FLY_POS = "fly_pos";
    public static final String[] SLOT_SENSORS = {"CS1", "CS2", "CS3", "CS4"};
    public static final String ENTRY_SENSOR = "CS5";
    public static final String STALE_FLYWHEEL = "fly_follow";

    public final String name;
    public final String[] hubIds;
    /** true = the motor is configured REVERSE. */
    public final boolean leftReversed, rightReversed, intakeReversed, flywheelReversed, pusherReversed;
    /** Pinpoint pods: offsets in millimetres, and whether each counts reversed. */
    public final double xPodOffsetMm, yPodOffsetMm;
    public final boolean xPodReversed, yPodReversed;
    /** Footprint, inches. */
    public final double depth, width, originToBack;
    /** Ramp servo positions; NaN = not set, and the ramp then never moves. */
    public final double rampStowLeft, rampStowRight, rampDeployLeft, rampDeployRight;

    private ChassisProfile(String name, String[] hubIds, boolean leftReversed, boolean rightReversed,
                           double xPodOffsetMm, boolean xPodReversed, double yPodOffsetMm, boolean yPodReversed,
                           double depth, double width, double originToBack,
                           boolean intakeReversed, boolean flywheelReversed, boolean pusherReversed,
                           double rampStowLeft, double rampStowRight, double rampDeployLeft, double rampDeployRight) {
        this.name = name;
        this.hubIds = hubIds;
        this.leftReversed = leftReversed;
        this.rightReversed = rightReversed;
        this.xPodOffsetMm = xPodOffsetMm;
        this.xPodReversed = xPodReversed;
        this.yPodOffsetMm = yPodOffsetMm;
        this.yPodReversed = yPodReversed;
        this.depth = depth;
        this.width = width;
        this.originToBack = originToBack;
        this.intakeReversed = intakeReversed;
        this.flywheelReversed = flywheelReversed;
        this.pusherReversed = pusherReversed;
        this.rampStowLeft = rampStowLeft;
        this.rampStowRight = rampStowRight;
        this.rampDeployLeft = rampDeployLeft;
        this.rampDeployRight = rampDeployRight;
    }

    /** The competition robot. Serial and Android id of its Control Hub. */
    public static final ChassisProfile ROBOT_1 = new ChassisProfile("ROBOT_1",
            new String[] {"5462f57f0f18d316", "7c789032eb35f27b"},
            true, false,
            -133.7, false, 9.5, true,
            17.48, 13.70, 7.598,
            false, true, false,
            0.75, 0.25, 0.0, 1.0);

    /** The first drive base. No hub registered, no mechanisms tuned, ramp servo positions not set. */
    public static final ChassisProfile PROTOTYPE = new ChassisProfile("PROTOTYPE",
            new String[0],
            false, true,
            -55.0, true, -75.6, true,
            18.0, 14.0, 9.0,
            true, true, false,
            Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    /** The second drive base. Its pod offsets are the prototype's (never tuned here) and its footprint is assumed. */
    public static final ChassisProfile DRAFT_1 = new ChassisProfile("DRAFT_1",
            new String[0],
            true, false,
            -55.0, false, -75.6, false,
            18.0, 14.0, 9.0,
            true, true, false,
            Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    private static final ChassisProfile[] ALL = {PROTOTYPE, DRAFT_1, ROBOT_1};
    /** Used only when no profile has any hub registered at all (first-time setup). */
    private static final ChassisProfile DEFAULT = DRAFT_1;

    public Footprint footprint() { return new Footprint(originToFront(), originToBack, halfWidth(), 18.0); }
    public double originToFront() { return depth - originToBack; }
    public double halfWidth() { return width / 2; }
    /** Distance from the origin to the farthest corner. */
    public double farCorner() { return Math.hypot(Math.max(originToFront(), originToBack), halfWidth()); }
    public boolean rampSet() {
        return !(Double.isNaN(rampStowLeft) || Double.isNaN(rampStowRight) || Double.isNaN(rampDeployLeft) || Double.isNaN(rampDeployRight));
    }

    // ---- Selection ----
    private static ChassisProfile selected;
    private static boolean noHubRegistered;

    /** True when no profile has any hub registered (first-time setup); INIT screens report it. */
    public static boolean noHubRegistered() { return noHubRegistered; }

    /** The profile of the hub this app runs on. Throws on a hub that is not registered: a known robot must never run with another chassis's directions. */
    public static synchronized ChassisProfile get() {
        if (selected != null) return selected;
        String serial = hardwareSerial(), androidId = androidId();
        boolean anyRegistered = false;
        for (ChassisProfile p : ALL) {
            for (String id : p.hubIds) {
                anyRegistered = true;
                if ((serial != null && id.equalsIgnoreCase(serial)) || (androidId != null && id.equalsIgnoreCase(androidId))) {
                    selected = p;
                    return p;
                }
            }
        }
        if (!anyRegistered) {
            noHubRegistered = true;
            selected = DEFAULT;
            return selected;
        }
        throw new IllegalStateException("Unrecognized Control Hub: " + hubIdsText() + ". Register it in ChassisProfile before running.");
    }

    public static String hubIdsText() {
        String serial = hardwareSerial(), androidId = androidId();
        return "serial " + (serial == null ? "(none)" : serial) + ", Android id " + (androidId == null ? "(none)" : androidId);
    }

    /** The hub's hardware serial: the Android build serial, ignored (null) when it is empty or reads "unknown". */
    @SuppressWarnings("deprecation")
    private static String hardwareSerial() {
        try {
            String s = Build.SERIAL;
            if (s != null && !s.isEmpty() && !"unknown".equalsIgnoreCase(s)) return s;
        } catch (Throwable ignored) { }
        return null;
    }

    /** The system's secure Android id, or null when it cannot be read. */
    private static String androidId() {
        try {
            String s = Settings.Secure.getString(AppUtil.getDefContext().getContentResolver(), Settings.Secure.ANDROID_ID);
            return s == null || s.isEmpty() ? null : s;
        } catch (Throwable e) {
            return null;
        }
    }

}
