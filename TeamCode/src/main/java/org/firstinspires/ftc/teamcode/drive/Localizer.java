package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;

import java.util.Locale;

/**
 * One field pose and one field-frame velocity per loop, from the Pinpoint (specs/drive.md section 4).
 * update() is called exactly once per loop, before anything reads the pose.
 */
public final class Localizer {
    public enum Mode { RESET_AND_SEED, ADOPTED, RESEEDED_AFTER_FAILED_ADOPT }

    private final GoBildaPinpointDriver pp;
    private final ChassisProfile profile;

    private Pose pose = new Pose(0, 0, 0);
    private double vx, vy, omega;
    private Mode mode = Mode.RESET_AND_SEED;
    private String note = "";
    private boolean seedConfirmed;
    private boolean readError;
    private GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.NOT_READY;

    private Localizer(HardwareMap hw, ChassisProfile profile) {
        this.pp = hw.get(GoBildaPinpointDriver.class, ChassisProfile.PINPOINT);
        this.profile = profile;
    }

    /** Autos, test opmodes, and a TeleOp with no fresh pose. The robot must be still. */
    public static Localizer resetAndSeed(HardwareMap hw, ChassisProfile profile, Pose start) {
        Localizer l = new Localizer(hw, profile);
        l.mode = Mode.RESET_AND_SEED;
        double readyS = l.reset(start);
        l.note = l.describe(readyS, null);
        return l;
    }

    /**
     * The TeleOp after an auto: nothing is written to the device, so the pose it kept through the transition is used as it is.
     * Falls back to a reset and seed with the saved pose when the device cannot be trusted.
     */
    public static Localizer adopt(HardwareMap hw, ChassisProfile profile, Pose saved, double fieldHalf) {
        Localizer l = new Localizer(hw, profile);
        long t0 = System.nanoTime();
        boolean ready = false;
        while ((System.nanoTime() - t0) / 1e9 < DriveParams.ADOPT_READY_S) {
            l.update();
            if (!l.readError && l.status == GoBildaPinpointDriver.DeviceStatus.READY) {
                ready = true;
                break;
            }
            pause(20);
        }
        double readyS = (System.nanoTime() - t0) / 1e9;
        Pose dev = l.pose;
        String why = null;
        if (!ready) why = "not READY within " + DriveParams.ADOPT_READY_S + " s";
        else if (!dev.isFinite()) why = "a reading is not a number";
        else if (Math.hypot(dev.x, dev.y) < 1.0 && Math.abs(dev.headingDeg()) < 2.0
                && (Math.hypot(saved.x, saved.y) > 3.0 || Math.abs(saved.headingDeg()) > 5.0)) why = "the device reads zero: it restarted";
        else if (Math.abs(dev.x) > fieldHalf + 6 || Math.abs(dev.y) > fieldHalf + 6) why = "the device reads outside the field";
        if (why == null) {
            l.mode = Mode.ADOPTED;
            l.seedConfirmed = true;
            l.note = l.describe(readyS, saved);
        } else {
            String before = l.describe(readyS, saved);
            l.mode = Mode.RESEEDED_AFTER_FAILED_ADOPT;
            l.reset(saved);
            l.note = "adopt failed (" + why + "); was: " + before + "; now: " + l.describe(readyS, saved);
        }
        return l;
    }

    /** Reads the device once. Stores the pose and the field-frame velocity (in/s, in/s, rad/s). */
    public void update() {
        try {
            pp.update();
            status = pp.getDeviceStatus();
            double x = pp.getPosX(DistanceUnit.INCH), y = pp.getPosY(DistanceUnit.INCH);
            double h = Angle.wrap(pp.getHeading(AngleUnit.RADIANS));
            pose = new Pose(x, y, h);
            vx = pp.getVelX(DistanceUnit.INCH);
            vy = pp.getVelY(DistanceUnit.INCH);
            omega = pp.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            readError = !pose.isFinite();
        } catch (Throwable e) {
            readError = true;
        }
    }

    public Pose pose() { return pose; }
    public double vx() { return vx; }
    public double vy() { return vy; }
    public double omega() { return omega; }
    public double speed() { return Math.hypot(vx, vy); }
    public Mode mode() { return mode; }
    /** One line for the INIT screen and the log. */
    public String note() { return note; }
    public boolean seedConfirmed() { return seedConfirmed; }
    public boolean ready() { return !readError && status == GoBildaPinpointDriver.DeviceStatus.READY; }
    public String statusText() { return readError ? "READ ERROR" : String.valueOf(status); }

    /** A fault status or a read error on the last update. */
    public boolean faultNow() {
        if (readError) return true;
        switch (status) {
            case FAULT_X_POD_NOT_DETECTED:
            case FAULT_Y_POD_NOT_DETECTED:
            case FAULT_NO_PODS_DETECTED:
            case FAULT_IMU_RUNAWAY:
            case FAULT_BAD_READ:
                return true;
            default:
                return false;
        }
    }

    /** Forces the estimate and writes it to the device. The IMU is not recalibrated. */
    public void setPose(Pose p) {
        try {
            pp.setPosition(new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.RADIANS, p.h));
        } catch (Throwable e) {
            readError = true;
        }
        pose = p;
    }

    /** Writes a pose and reads it back until it agrees, for INIT. Returns whether it was confirmed. */
    public boolean seed(Pose p) {
        seedConfirmed = false;
        for (int i = 0; i < DriveParams.SEED_ATTEMPTS && !seedConfirmed; i++) {
            setPose(p);
            pause(25);
            update();
            seedConfirmed = !readError && pose.distTo(p) <= DriveParams.SEED_TOL_IN
                    && Math.abs(Math.toDegrees(Angle.diff(p.h, pose.h))) <= DriveParams.SEED_TOL_DEG;
        }
        if (!seedConfirmed) pose = p;
        return seedConfirmed;
    }

    /** Zero the device, recalibrate its IMU, wait for READY, then seed. Returns the seconds READY took. */
    private double reset(Pose start) {
        long t0 = System.nanoTime();
        try {
            pp.setOffsets(profile.xPodOffsetMm, profile.yPodOffsetMm, DistanceUnit.MM);
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setEncoderDirections(
                    profile.xPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    profile.yPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pp.resetPosAndIMU();
        } catch (Throwable e) {
            readError = true;
        }
        // A pose written before the reset finishes is lost.
        pause((long) (DriveParams.RESET_WAIT_MIN_S * 1000));
        while ((System.nanoTime() - t0) / 1e9 < DriveParams.RESET_WAIT_MAX_S) {
            update();
            if (!readError && status == GoBildaPinpointDriver.DeviceStatus.READY) break;
            pause(20);
        }
        double readyS = (System.nanoTime() - t0) / 1e9;
        seed(start);
        return readyS;
    }

    private String describe(double readyS, Pose saved) {
        String s;
        try {
            s = String.format(Locale.US, "%s: status %s, READY after %.2f s, device pose %s, %.0f Hz, firmware %d, encoders %d / %d",
                    mode, statusText(), readyS, pose, pp.getFrequency(), pp.getDeviceVersion(), pp.getEncoderX(), pp.getEncoderY());
        } catch (Throwable e) {
            s = mode + ": status " + statusText() + ", device pose " + pose;
        }
        if (mode != Mode.ADOPTED) s += seedConfirmed ? ", seed confirmed" : ", SEED NOT CONFIRMED";
        if (saved != null) {
            s += String.format(Locale.US, ", saved pose %s differs by %.1f in, %.1f deg", saved, pose.distTo(saved),
                    Math.toDegrees(Angle.diff(pose.h, saved.h)));
        }
        return s;
    }

    private static void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
