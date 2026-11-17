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

    protected abstract RunLog.Kind logKind();

    /** The opmode's name in the log. */
    protected abstract String title();

    /** The opmode itself: INIT, the wait for Start, and the run. */
    protected abstract void body() throws InterruptedException;

    /** Called in the shutdown, after everything is stopped: save the pose if this opmode saves one. */
    protected void saveAtEnd() { }

    @Override
    public final void runOpMode() throws InterruptedException {
        RunLog.open(logKind(), title());
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

    /** A button that reports only the moment it is pressed. */
    protected static final class Edge {
        private boolean last;

        public boolean pressed(boolean now) {
            boolean p = now && !last;
            last = now;
            return p;
        }
    }

    /** A chord that must be held for a time; fires once per hold. */
    protected static final class Hold {
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
