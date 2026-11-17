package org.firstinspires.ftc.teamcode.robot;

import android.os.Build;
import android.provider.Settings;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Everything that differs between chassis, chosen from the identity of the Control Hub (specs/drive.md section 1,
 * robot_design/). One build runs on every chassis the team has.
 *
 * SPEC GAP: the specs name three profiles (ROBOT_1, PROTOTYPE, DRAFT_1) but document only ROBOT_1, so only ROBOT_1 is here.
 * SPEC GAP: how the hub's hardware serial is read is not stated; Build.SERIAL and the ro.serialno property are tried.
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

    private static final ChassisProfile[] ALL = {ROBOT_1};
    private static final ChassisProfile DEFAULT = ROBOT_1;

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
                if (id.equalsIgnoreCase(serial) || id.equalsIgnoreCase(androidId)) {
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
        throw new IllegalStateException("Unrecognized Control Hub: serial " + serial + ", Android id " + androidId
                + ". Register it in ChassisProfile before running.");
    }

    public static String hubIdsText() {
        return "serial " + hardwareSerial() + ", Android id " + androidId();
    }

    @SuppressWarnings("deprecation")
    private static String hardwareSerial() {
        try {
            String s = Build.SERIAL;
            if (s != null && !s.isEmpty() && !"unknown".equalsIgnoreCase(s)) return s;
        } catch (Throwable ignored) { }
        try {
            Class<?> sp = Class.forName("android.os.SystemProperties");
            Object s = sp.getMethod("get", String.class).invoke(null, "ro.serialno");
            if (s != null && !s.toString().isEmpty()) return s.toString();
        } catch (Throwable ignored) { }
        return "unknown";
    }

    private static String androidId() {
        try {
            String s = Settings.Secure.getString(AppUtil.getDefContext().getContentResolver(), Settings.Secure.ANDROID_ID);
            return s == null ? "unknown" : s;
        } catch (Throwable e) {
            return "unknown";
        }
    }
}
