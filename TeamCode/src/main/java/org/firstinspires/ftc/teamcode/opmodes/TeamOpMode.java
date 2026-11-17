package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.util.ConfigDump;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * The frame every team opmode runs in. It gives every opmode the same way in and the same way out
 * (specs/architecture.md section 7.1, target T10; specs/logging.md section 1.3):
 *
 * in:  open the log first, before any hardware is touched, so a crash during initialization is captured;
 * out: stop the drive; stop all mechanisms, with no update after it; save the pose if the opmode saves one; close the log.
 * The way out runs on every way out, a crash included.
 */
public abstract class TeamOpMode extends LinearOpMode {
    /** Set by the opmode when it builds them; the shutdown stops whichever exist. */
    protected MecanumDrive drive;
    protected Mechanisms mech;
    protected ChassisProfile profile;

    /** The kind of run log this opmode writes, or null for none. */
    protected abstract RunLog.Kind logKind();

    /** The opmode's name in the log. */
    protected abstract String title();

    /** The opmode itself: INIT, the wait for Start, and the run. */
    protected abstract void body() throws InterruptedException;

    /** Called in the shutdown, after everything is stopped: save the pose if this opmode saves one. */
    protected void saveAtEnd() { }

    @Override
    public final void runOpMode() throws InterruptedException {
        if (logKind() != null) RunLog.open(logKind(), title());
        try {
            RunLog.volts(new RunLog.Volts() {
                @Override public double read() { return battery(); }
            });
            body();
        } catch (InterruptedException e) {
            throw e;                    // the opmode was stopped
        } catch (RuntimeException e) {
            RunLog.crash(e);
            throw e;
        } catch (Error e) {
            RunLog.crash(e);
            throw e;
        } finally {
            shutdown();
        }
    }

    private void shutdown() {
        try {
            if (drive != null) drive.stop();
        } catch (Throwable ignored) { }
        try {
            if (mech != null) mech.stopAll();
        } catch (Throwable ignored) { }
        try {
            saveAtEnd();
        } catch (Throwable ignored) { }
        RunLog.close();
    }

    /** Start was pressed: mark it in the log and dump the settings in force. */
    protected void markStarted() {
        RunLog.started();
        ConfigDump.dump();
    }

    /** The lowest voltage any hub reports. */
    protected double battery() {
        double v = Double.NaN;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double x = s.getVoltage();
            if (x > 0 && (Double.isNaN(v) || x < v)) v = x;
        }
        return v;
    }

    /** Plain field-oriented driving for the support opmodes: stick up is away from the alliance's drivers; turn stick right is clockwise. */
    protected void driveByHand(org.firstinspires.ftc.teamcode.field.Alliance alliance, double scale) {
        double up = -gamepad1.left_stick_y, right = gamepad1.left_stick_x, th = alliance.driverForwardRad(), h = drive.pose().h;
        double fx = up * Math.cos(th) + right * Math.sin(th), fy = up * Math.sin(th) - right * Math.cos(th);
        drive.drive((fx * Math.cos(h) + fy * Math.sin(h)) * scale, (-fx * Math.sin(h) + fy * Math.cos(h)) * scale, -gamepad1.right_stick_x * 0.5 * scale);
    }

    /** Robot-oriented driving for the studies: stick up is the robot's front, stick right its right; turn stick right is clockwise. */
    protected void driveRobotOriented(double scale) {
        drive.drive(-gamepad1.left_stick_y * scale, -gamepad1.left_stick_x * scale, -gamepad1.right_stick_x * 0.5 * scale);
    }

    /** Pads a loop to a fixed length, so a study's loop is as long as the TeleOp's (28 ms in the studies that ask for it). */
    public static final class LoopPad {
        private long last = System.nanoTime();

        /** Call once per loop, at its end. */
        public void pad(double ms) throws InterruptedException {
            long left = (long) (ms * 1e6) - (System.nanoTime() - last);
            if (left > 0) Thread.sleep(left / 1_000_000L, (int) (left % 1_000_000L));
            last = System.nanoTime();
        }
    }

    /** A button that reports only the moment it is pressed. */
    public static final class Edge {
        private boolean last;

        public boolean pressed(boolean now) {
            boolean p = now && !last;
            last = now;
            return p;
        }
    }

    /** A chord that must be held for a time; fires once per hold. */
    public static final class Hold {
        private double since = -1;
        private boolean fired;

        /** @return true once, when the chord has been held for the time */
        public boolean held(boolean down, double t, double seconds) {
            if (!down) {
                since = -1;
                fired = false;
                return false;
            }
            if (since < 0) since = t;
            if (!fired && t - since >= seconds) {
                fired = true;
                return true;
            }
            return false;
        }

        /** Seconds left while the chord is down and has not fired, else -1. */
        public double remaining(double t, double seconds) {
            return since < 0 || fired ? -1 : Math.max(0, seconds - (t - since));
        }
    }
}
