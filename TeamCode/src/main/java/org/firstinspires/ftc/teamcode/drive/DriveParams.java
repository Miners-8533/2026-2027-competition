package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

/** Tunable values of the drive base and the localizer (specs/drive.md section 6). */
@Config
public final class DriveParams {
    private DriveParams() { }

    /** Global multiplier on wheel powers. */
    public static double POWER_SCALE = 1.0;
    /** Smallest change of a wheel power that is sent. */
    public static double WRITE_DEADBAND = 0.01;
    /** Longest time a wheel goes without a write. */
    public static double REFRESH_MS = 200;

    /** One wheel current per period; 0 = the wheel check is off. */
    public static double CHECK_PERIOD_MS = 100;
    public static double CHECK_MIN_POWER = 0.3;
    public static double CHECK_LOW_FRACTION = 0.45;
    public static double CHECK_RECOVER_FRACTION = 0.70;
    public static int CHECK_SAMPLES_TO_JUDGE = 6;
    public static int CHECK_SAMPLES_TO_CONFIRM = 4;
    /** Weight of a new sample in the running average. */
    public static double CHECK_NEW_WEIGHT = 0.3;

    public static double SEED_TOL_IN = 0.5;
    public static double SEED_TOL_DEG = 2.0;
    public static int SEED_ATTEMPTS = 5;
    public static double RESET_WAIT_MIN_S = 0.3;
    public static double RESET_WAIT_MAX_S = 2.0;
    public static double ADOPT_READY_S = 0.5;
}
