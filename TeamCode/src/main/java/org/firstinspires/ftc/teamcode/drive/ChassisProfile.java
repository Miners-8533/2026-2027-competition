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
 * To register a hub: run the "Chassis Info" opmode on it, read the hub ids it shows, and add
 * them to that profile's list below.
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
    /** Direction that makes positive intake power pull elements in. Not yet verified on any chassis. */
    public final DcMotorSimple.Direction intakeDirection;

    private ChassisProfile(String name, List<String> hubIds,
                           DcMotorSimple.Direction leftDirection, DcMotorSimple.Direction rightDirection,
                           GoBildaPinpointDriver.EncoderDirection podXDirection,
                           GoBildaPinpointDriver.EncoderDirection podYDirection,
                           double xOffsetMM, double yOffsetMM, double robotDepthIn, double robotWidthIn,
                           DcMotorSimple.Direction intakeDirection) {
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
        this.intakeDirection = intakeDirection;
    }

    /** The earlier prototype chassis. Offsets were tuned with Pinpoint Offset Check. */
    public static final ChassisProfile PROTOTYPE = new ChassisProfile("PROTOTYPE",
            Arrays.asList(/* hub ids: add from Chassis Info */),
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
            GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED,
            -55.0, -75.6, 18.0, 14.0, DcMotorSimple.Direction.REVERSE);

    /**
     * First draft of the new chassis. Motor and pod directions come from Motor Map Test and Pinpoint
     * Push Test. The offsets are the prototype's, NOT yet tuned here (rerun Pinpoint Offset Check),
     * and the 18 x 14 in footprint is assumed.
     */
    public static final ChassisProfile DRAFT_1 = new ChassisProfile("DRAFT_1",
            Arrays.asList(/* hub ids: add from Chassis Info */),
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD,
            -55.0, -75.6, 18.0, 14.0, DcMotorSimple.Direction.REVERSE);

    private static final ChassisProfile[] ALL = {PROTOTYPE, DRAFT_1};
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
                + ". Run the 'Chassis Info' opmode and add this hub's id to a ChassisProfile.");
    }

    /** False if the last {@link #detect} fell back to the default because no hub is registered yet. */
    public static boolean wasMatched() {
        return lastMatched;
    }

    public String label() {
        return name + (lastMatched ? "" : "   <-- no hub registered yet, using the default profile");
    }
}
