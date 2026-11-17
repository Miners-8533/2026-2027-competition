package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Turns one robot-frame command into four wheel powers, and owns the localizer (specs/drive.md section 2).
 * The command is open-loop power: forward, left, counterclockwise turn, each from -1 to 1.
 */
public final class MecanumDrive {
    /** Wheel order everywhere: left front, left back, right back, right front. */
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final double[] sent = new double[4];
    private final long[] sentAt = new long[4];
    private final double[] wanted = new double[4];
    private double cmdF, cmdL, cmdT;
    private long lastCheck;

    public final Localizer localizer;
    public final WheelCheck wheelCheck = new WheelCheck();
    public final ChassisProfile profile;

    private MecanumDrive(HardwareMap hw, ChassisProfile profile, Localizer localizer) {
        this.profile = profile;
        this.localizer = localizer;
        String[] names = {ChassisProfile.LEFT_FRONT, ChassisProfile.LEFT_BACK, ChassisProfile.RIGHT_BACK, ChassisProfile.RIGHT_FRONT};
        for (int i = 0; i < 4; i++) {
            motors[i] = hw.get(DcMotorEx.class, names[i]);
            boolean left = i < 2;
            boolean reversed = left ? profile.leftReversed : profile.rightReversed;
            motors[i].setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** All hubs use automatic bulk reads. */
    public static void bulkAuto(HardwareMap hw) {
        for (LynxModule hub : hw.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    /** Built for an auto or a test opmode: the Pinpoint is reset and seeded. The robot must be still. */
    public static MecanumDrive resetAndSeed(HardwareMap hw, ChassisProfile profile, Pose start) {
        bulkAuto(hw);
        MecanumDrive d = new MecanumDrive(hw, profile, Localizer.resetAndSeed(hw, profile, start));
        RunLog.event("PINPOINT " + d.localizer.note());
        return d;
    }

    /** Built for the TeleOp after an auto: the Pinpoint's own pose is adopted when it can be. */
    public static MecanumDrive adopt(HardwareMap hw, ChassisProfile profile, Pose saved, double fieldHalf) {
        bulkAuto(hw);
        MecanumDrive d = new MecanumDrive(hw, profile, Localizer.adopt(hw, profile, saved, fieldHalf));
        RunLog.event("PINPOINT " + d.localizer.note());
        return d;
    }

    /** Reads the pose: exactly once per loop, before anything uses it. */
    public void updatePose() {
        LoopProfiler.begin("pose");
        localizer.update();
        LoopProfiler.end("pose");
    }

    public Pose pose() { return localizer.pose(); }

    /** One drive command per loop. */
    public void drive(double forward, double left, double turn) {
        cmdF = forward;
        cmdL = left;
        cmdT = turn;
        wanted[0] = forward - left - turn;
        wanted[1] = forward + left - turn;
        wanted[2] = forward - left + turn;
        wanted[3] = forward + left + turn;
        double max = 1.0;
        for (double w : wanted) max = Math.max(max, Math.abs(w));
        LoopProfiler.begin("drive_write");
        long now = System.nanoTime();
        for (int i = 0; i < 4; i++) {
            wanted[i] = wanted[i] / max * DriveParams.POWER_SCALE;
            write(i, wanted[i], now);
        }
        LoopProfiler.end("drive_write");
        checkWheels(now);
    }

    public void stop() { drive(0, 0, 0); }

    /** A single wheel at a power, for Pit Check. The others stop. */
    public void driveWheel(int wheel, double power) {
        long now = System.nanoTime();
        for (int i = 0; i < 4; i++) write(i, i == wheel ? power : 0, now);
        cmdF = cmdL = cmdT = 0;
    }

    private void write(int i, double p, long now) {
        boolean stop = p == 0 && sent[i] != 0;
        boolean changed = Math.abs(p - sent[i]) >= DriveParams.WRITE_DEADBAND;
        boolean stale = (now - sentAt[i]) / 1e6 >= DriveParams.REFRESH_MS;
        if (stop || changed || stale) {
            motors[i].setPower(p);
            sent[i] = p;
            sentAt[i] = now;
            LoopProfiler.count("drive_writes", 1);
        } else {
            LoopProfiler.count("drive_skips", 1);
        }
    }

    /** One wheel current per period, taking the wheels in turn. A robot standing still is never read. */
    private void checkWheels(long now) {
        if (DriveParams.CHECK_PERIOD_MS <= 0 || (now - lastCheck) / 1e6 < DriveParams.CHECK_PERIOD_MS) return;
        int i = wheelCheck.pick(sent);
        if (i < 0) return;
        lastCheck = now;
        LoopProfiler.begin("drive_amps");
        double amps;
        try {
            amps = motors[i].getCurrent(CurrentUnit.AMPS);
        } catch (Throwable e) {
            amps = Double.NaN;
        }
        LoopProfiler.end("drive_amps");
        if (Double.isNaN(amps)) return;
        String event = wheelCheck.sample(i, sent[i], amps);
        if (event != null) RunLog.event(event);
    }

    public double cmdForward() { return cmdF; }
    public double cmdLeft() { return cmdL; }
    public double cmdTurn() { return cmdT; }
    public double wheelPower(int i) { return sent[i]; }

    /** Reads one wheel's current now (2 to 3 ms). For Drive Characterization and Pit Check. */
    public double readAmps(int i) {
        try {
            return motors[i].getCurrent(CurrentUnit.AMPS);
        } catch (Throwable e) {
            return Double.NaN;
        }
    }

    public static final String LOG_COLUMNS = "cmd_fwd,cmd_left,cmd_turn,a_lf,a_lb,a_rb,a_rf";

    /** The last drive command and the four wheel currents, as log columns. */
    public String logRow() {
        return String.format(Locale.US, "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f", cmdF, cmdL, cmdT,
                wheelCheck.lastAmps(0), wheelCheck.lastAmps(1), wheelCheck.lastAmps(2), wheelCheck.lastAmps(3));
    }
}
