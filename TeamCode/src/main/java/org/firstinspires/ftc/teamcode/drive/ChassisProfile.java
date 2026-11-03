package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;
import android.provider.Settings;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Everything that differs from one physical chassis to the next, chosen automatically by which
 * Control Hub the app is running on (each chassis has its own hub). The same build works on
 * every chassis; nobody picks a profile.
 *
 * To register a hub: run any opmode on it (the Pit Check will do): the error it throws lists the hub ids; add one (the serial
 * is preferred) to that profile's list below.
 *
 * Detection: a hub that matches a profile uses it. A hub that matches none throws with its ids
 * in the message, so a wrong chassis can never silently run with the wrong directions - EXCEPT
 * while no profile has any id registered yet (the first-time setup): then {@link #DEFAULT} is
 * used and {@link #wasMatched()} is false, so init screens can say so.
 */
public final class ChassisProfile {

    public final String name;
    private final List<String> hubIds;
    /** Direction that makes positive power drive each side's wheels robot-forward. */
    public final DcMotorSimple.Direction leftDirection, rightDirection;
    /** Pinpoint pod encoder directions (X is the forward pod, Y the strafe pod). */
    public final GoBildaPinpointDriver.EncoderDirection podXDirection, podYDirection;
    /** Pinpoint pod offsets from the tracking center, mm (see PinpointLocalizer.Params). */
    public final double xOffsetMM, yOffsetMM;
    /** Robot footprint, inches: depth is front-to-back, width side-to-side. */
    public final double robotDepthIn, robotWidthIn;
    /**
     * Distance from the robot's back edge to its origin (the Pinpoint's tracking point, near the center of rotation), inches. The origin is centered
     * side to side but NOT front to back, so a robot with its back against a wall has its origin this far from the wall (not depth / 2). Defaults to
     * depth / 2 when a profile does not give it.
     */
    public final double originFromBackIn;
    /** Distance from the origin to the front edge, inches: the depth less {@link #originFromBackIn}. */
    public double originFromFrontIn() {
        return robotDepthIn - originFromBackIn;
    }

    /** Distance from the origin to each side edge, inches: the origin is centered side to side. */
    public double originFromSideIn() {
        return robotWidthIn / 2.0;
    }

    /** Direction that makes positive intake power pull elements in. Not yet verified on any chassis. */
    public final DcMotorSimple.Direction intakeDirection;
    /** Flywheel motor direction (positive power spins it the shooting way) and pusher direction (positive = the forward, pushing way). Both unverified. */
    public final DcMotorSimple.Direction flywheelDirection, pusherDirection;
    /** Direction of the optional second flywheel motor ("fly_follow", no encoder). It usually faces the first, so the opposite of flywheelDirection. Verified on the flywheel bench: flywheel REVERSE, follower FORWARD (both spun the wrong way with last year's FORWARD / REVERSE). */
    public final DcMotorSimple.Direction flywheelFollowerDirection;
    /**
     * Ramp servo positions, 0 to 1, for the left and right servo. NaN means not measured yet:
     * the ramp will not move until they are filled in (the positions were found with the Ramp Test, since deleted; see git history).
     */
    public final double rampStowedLeft, rampStowedRight, rampDeployedLeft, rampDeployedRight;

    private ChassisProfile(String name, List<String> hubIds,
                           DcMotorSimple.Direction leftDirection, DcMotorSimple.Direction rightDirection,
                           GoBildaPinpointDriver.EncoderDirection podXDirection,
                           GoBildaPinpointDriver.EncoderDirection podYDirection,
                           double xOffsetMM, double yOffsetMM, double robotDepthIn, double robotWidthIn, double originFromBackIn,
                           DcMotorSimple.Direction intakeDirection,
                           DcMotorSimple.Direction flywheelDirection, DcMotorSimple.Direction pusherDirection,
                           DcMotorSimple.Direction flywheelFollowerDirection,
                           double rampStowedLeft, double rampStowedRight,
                           double rampDeployedLeft, double rampDeployedRight) {
        this.name = name;
        this.hubIds = hubIds;
        this.leftDirection = leftDirection;
        this.rightDirection = rightDirection;
        this.podXDirection = podXDirection;
        this.podYDirection = podYDirection;
        this.xOffsetMM = xOffsetMM;
        this.yOffsetMM = yOffsetMM;
        this.robotDepthIn = robotDepthIn;
        this.robotWidthIn = robotWidthIn;
        this.originFromBackIn = Double.isNaN(originFromBackIn) ? robotDepthIn / 2.0 : originFromBackIn;
        this.intakeDirection = intakeDirection;
        this.flywheelDirection = flywheelDirection;
        this.pusherDirection = pusherDirection;
        this.flywheelFollowerDirection = flywheelFollowerDirection;
        this.rampStowedLeft = rampStowedLeft;
        this.rampStowedRight = rampStowedRight;
        this.rampDeployedLeft = rampDeployedLeft;
        this.rampDeployedRight = rampDeployedRight;
    }

    /** The earlier prototype chassis. Offsets were tuned with Pinpoint Offset Check. */
    public static final ChassisProfile PROTOTYPE = new ChassisProfile("PROTOTYPE",
            Arrays.asList(/* hub ids: add from the "Unrecognized Control Hub" error */),
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
            GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED,
            -55.0, -75.6, 18.0, 14.0, Double.NaN, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
            Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    /**
     * First draft of the new chassis. Motor and pod directions come from the Motor Map Test and Pinpoint
     * Push Test. The offsets are the prototype's, NOT yet tuned here (rerun Pinpoint Offset Check),
     * and the 18 x 14 in footprint is assumed.
     */
    public static final ChassisProfile DRAFT_1 = new ChassisProfile("DRAFT_1",
            Arrays.asList(/* hub ids: add from the "Unrecognized Control Hub" error */),
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD,
            -55.0, -75.6, 18.0, 14.0, Double.NaN, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
            Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    /**
     * The assembled robot with all four mechanisms (intake, flywheel, pusher, ramp). Starts as a copy of
     * DRAFT_1: drive and pod directions are verified on that chassis. The pod offsets were measured with a tape on 2026-11-01 (the X pod 6.5 in
     * RIGHT of the tracking center, so xOffset -165.1 mm; the Y pod level with the center front to back, so yOffset 0, its sideways position
     * does not matter); then corrected by the first Pinpoint Offset Check (2026-11-01, 3 turns: wobble 2.58 in, forward-axis error -31.4 mm,
     * strafe-axis error 9.5 mm) to x -133.7 mm, y +9.5 mm. The pods' own ticks over those turns give the same: the X pod 5.28 in (134 mm) right of the
     * rotation center and the Y pod 8.5 mm ahead of it. (The tape said 6.5 in and 0: the robot's rotation center is not its geometric center.) The Y pod direction is REVERSED (the Push Test of 2026-11-01 read a left
     * push as -Y, with the pod ticks falling about 5,600 for 11.5 in); the X pod and the heading passed. The ramp pair is mirrored: stowed left 1.0 / right 0.0, deployed left 0.0 / right 1.0 (set
     * 2026-10-31 from the Ramp Test, ramp seen stowed up at left 1.0 / right 0.0; the servos have no feedback, so
     * these are commanded values; TRAVEL_SEC = 0.8 s was measured by the team). The intake direction was flipped to FORWARD on
     * 2026-10-31 (it ran backwards with REVERSE; not yet re-checked after the flip). The footprint
     * was 17 x 13.75 in (measured 2026-10-31); from CAD (2026-11-02) it is 444 x 348 mm = 17.48 x 13.70 in, with the origin 193 mm (7.598 in) from the back edge
     * and 174 mm (6.85 in) from each side, so 9.88 in from the front edge. The robot-frame check poses of 2026-11-02 agree with that to about 0.4 in. See design/integration-plan.md,
     * Phases 1 and 2. Hub: serial and Android id read over adb.
     */
    public static final ChassisProfile ROBOT_1 = new ChassisProfile("ROBOT_1",
            Arrays.asList("5462f57f0f18d316", "7c789032eb35f27b"),
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED,   // Y pod reversed: Pinpoint Push Test 2026-11-01 read a left push as -Y
            -139.0, 1.0, 17.48, 13.70, 7.598, DcMotorSimple.Direction.FORWARD,   // TEMPORARY 2026-11-03 for the spin test: CAD pod positions (X pod 313 mm from the left edge, Y pod 194 mm from the back edge) with the CAD origin (174 / 193) as the tracking point. The measured offsets, kept until the test says otherwise, are -133.7, 9.5
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
            1.0, 0.0, 0.0, 1.0);   // ramp: stowed left / right, deployed left / right (the tied servos are mirrored)

    private static final ChassisProfile[] ALL = {PROTOTYPE, DRAFT_1, ROBOT_1};
    private static final ChassisProfile DEFAULT = DRAFT_1;
    private static boolean lastMatched = true;

    /** Hub ids of the hub this app is running on: hardware serial (when readable) and Android id. */
    @SuppressWarnings("deprecation")
    public static List<String> currentHubIds(HardwareMap hardwareMap) {
        List<String> ids = new ArrayList<>();
        String serial = Build.SERIAL;
        if (serial != null && !serial.isEmpty() && !serial.equalsIgnoreCase("unknown")) {
            ids.add(serial.toLowerCase(Locale.US));
        }
        String androidId = Settings.Secure.getString(
                hardwareMap.appContext.getContentResolver(), Settings.Secure.ANDROID_ID);
        if (androidId != null && !androidId.isEmpty()) {
            ids.add(androidId.toLowerCase(Locale.US));
        }
        return ids;
    }

    /** The profile for this hub, or null if no profile lists it. */
    public static ChassisProfile match(HardwareMap hardwareMap) {
        List<String> ids = currentHubIds(hardwareMap);
        for (ChassisProfile p : ALL) {
            for (String id : p.hubIds) {
                if (ids.contains(id.toLowerCase(Locale.US))) return p;
            }
        }
        return null;
    }

    public static ChassisProfile detect(HardwareMap hardwareMap) {
        ChassisProfile matched = match(hardwareMap);
        if (matched != null) {
            lastMatched = true;
            return matched;
        }
        boolean anyRegistered = false;
        for (ChassisProfile p : ALL) {
            if (!p.hubIds.isEmpty()) anyRegistered = true;
        }
        if (!anyRegistered) {
            lastMatched = false;
            return DEFAULT;
        }
        throw new IllegalStateException("Unrecognized Control Hub " + currentHubIds(hardwareMap)
                + ". Add one of these ids (the serial is preferred) to a ChassisProfile in ChassisProfile.java.");
    }

    /** False if the last {@link #detect} fell back to the default because no hub is registered yet. */
    public static boolean wasMatched() {
        return lastMatched;
    }

    public String label() {
        return name + (lastMatched ? "" : "   <-- no hub registered yet, using the default profile");
    }
}
