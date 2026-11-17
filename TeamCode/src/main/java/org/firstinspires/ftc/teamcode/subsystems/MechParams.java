package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/** Tunable values of the mechanisms coordinator (specs/mechanisms.md section 13). */
@Config
public final class MechParams {
    private MechParams() { }

    public static double LAUNCH_TIMEOUT_S = 3.0;
    public static double FLYWHEEL_STOP_DELAY_S = 0.5;
    public static double NOTICE_S = 4.0;
    public static double CELL_HYSTERESIS_IN = 6.0;
    public static double BURST_AFTER_SALVO_S = 0.6;
    public static double HEALTH_LINE_S = 1.0;

    public static double CHECK_WAIT_S = 2.0, CHECK_SETTLE_S = 0.4, CHECK_COUNT_HOLDOFF_S = 0.8;
    public static double ARM_NEAR_CS4_BEHIND_DEG = 70.0, ARM_NEAR_CS4_AHEAD_DEG = 70.0;

    public static int ENTRY_CAPACITY = 4;
    /** The preload the rules require; 0 for practice without a preload. */
    public static int AUTO_START_HELD = 4;
    /** Full, the Full warning, the intake stop and the launched tally go by the CS5 entry count. Off returns them to the slot count. */
    public static boolean COUNT_BY_ENTRIES = true;
    /** An entry while the intake ran OUT, or this long after, is an element leaving. */
    public static double OUT_EXCLUDE_S = 0.4;

    public static double FULL_RUN_ON_S = 0.1, FULL_REVERSE_S = 0.1;
    public static double LOCK_RELEASE_DELAY_S = 0.5;
    public static int LOCK_MAX_TRIES = 2;
    /** Lock whenever the TeleOp has a target lock or a Hive Dash active and something is held. Off: the TeleOp intake is always in. */
    public static boolean LOCK_FOR_AIM = false;
    /** FULL step 3. Off = the intake stays stopped while locked and the fifth-element watch runs instead. */
    public static boolean INTAKE_OUT_WHILE_LOCKED = true;
    public static double FIFTH_ARM_AFTER_S = 0.5, FIFTH_DWELL_S = 0.3, FIFTH_CLEAR_S = 0.3;
    public static int FIFTH_DWELL_COUNTS = 8, FIFTH_CLEAR_COUNTS = 3;
    public static double FIFTH_MIN_OUT_S = 0.6, FIFTH_GIVE_UP_S = 3.0, FIFTH_RETRY_S = 2.0;

    /** Used only if the launch map were empty. */
    public static double MANUAL_SETPOINT = 1350.0;

    // Two guards that sit switched off beside the count; they go when the entry count has run in a match.
    public static boolean FULL_NEEDS_SLOT_AGREEMENT = false;
    /** Reset the held count when the slot count has read 0 for this long. 0 = off. */
    public static double RESET_WHEN_SLOTS_EMPTY_S = 0.0;
}
