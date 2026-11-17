package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPosition;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Flywheel Step Study (specs/support-opmodes.md sections 3 and 3.2): how fast does the flywheel follow a setpoint
 * change? The flywheel alone steps through a list of setpoints; each change is timed until the speed is in the launch
 * band and has stayed there. Not yet run on the robot.
 *
 * The list is run 2 times. The loop is padded to 28 ms. A step has settled when the speed has stayed in the launch
 * band 0.3 s; it ends 1.5 s after that, or after 8 s with NEVER SETTLED.
 *
 * Controls (gamepad 1): A runs the sequence (again: restarts it); B stops it and the wheel. With it stopped, D-pad
 * up / down steps the setpoint by 100 and right / left by 25; from off, any D-pad press spins up to 1350.
 * One STEP line per change and a STEP TABLE at the end.
 */
@Config
@TeleOp(name = "Flywheel Step Study", group = "2 Bring-up")
public final class FlywheelStepStudy extends TeamOpMode {
    public static double[] SETPOINTS = {1350, 1650, 1350, 1450, 1350, 1375, 1325, 1650, 1450, 1650, 1325};
    public static int RUNS = 2;
    public static double LOOP_MS = 28.0;
    public static double SETTLED_S = 0.3, AFTER_SETTLED_S = 1.5, NEVER_SETTLED_S = 8.0;
    public static double BIG_STEP = 100, SMALL_STEP = 25, FROM_OFF = 1350;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Flywheel Step Study"; }

    private Flywheel fly;
    private final StringBuilder table = new StringBuilder();
    private int changes, changesInTable;
    private String lastLine = "";
    // The change being timed.
    private boolean timing, settled;
    private double changedAt, from, to, inBandSince, settledAt, lowest, highest;

    private void change(double target, double t) {
        if (timing) finish(t, !settled);     // the change before this one has not had its line yet
        from = fly.filtered();
        to = Math.max(0, target);
        fly.setTarget(to);
        changedAt = t;
        inBandSince = -1;
        settled = false;
        timing = to > 0;
        lowest = highest = from;
        if (!timing) RunLog.eventf("STEP wheel off from %.0f", from);
    }

    /** Writes the STEP line of the change being timed. */
    private void finish(double t, boolean interrupted) {
        changes++;
        String how = settled ? String.format(Locale.US, "settled in %.2f s", settledAt - changedAt)
                : interrupted ? String.format(Locale.US, "changed again after %.2f s, before it settled", t - changedAt) : "NEVER SETTLED";
        lastLine = String.format(Locale.US, "%.0f -> %.0f: %s, lowest %.0f, highest %.0f, battery %.2f V", from, to, how, lowest, highest, battery());
        RunLog.event("STEP " + changes + ": " + lastLine);
        table.append(String.format(Locale.US, " | %.0f>%.0f %s", from, to, settled ? String.format(Locale.US, "%.2f", settledAt - changedAt) : interrupted ? "cut" : "never"));
    }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        fly = Flywheel.create(hardwareMap, profile);
        FlywheelPosition pos = FlywheelPosition.create(hardwareMap);
        telemetry.addLine(fly == null ? "NO FLYWHEEL in the configuration" : "Flywheel Step Study: Start, then A runs the sequence. B stops it and the wheel.");
        telemetry.addLine("Stopped: D-pad up/down 100, right/left 25; from off any D-pad press spins up to " + (int) FROM_OFF + ".");
        telemetry.update();
        waitForStart();
        if (isStopRequested() || fly == null) return;
        markStarted();
        if (pos != null) pos.commandPollen();
        RunLog.sampler("target,speed,filtered,command,amps", 0, new RunLog.Sampler() {
            @Override public String row() {
                return String.format(Locale.US, "%.0f,%.0f,%.0f,%.2f,%.1f", fly.target(), fly.speed(), fly.filtered(), fly.command(), fly.amps());
            }
        });
        RunLog.burst(true);
        Edge a = new Edge(), b = new Edge(), up = new Edge(), down = new Edge(), right = new Edge(), left = new Edge();
        LoopPad pad = new LoopPad();
        int total = SETPOINTS.length * Math.max(1, RUNS), step = -1;       // step < 0: the sequence is stopped
        try {
            while (opModeIsActive()) {
                double t = Clock.now();
                if (b.pressed(gamepad1.b)) {
                    if (step >= 0) RunLog.event("STEP sequence stopped by B at step " + (step + 1) + " of " + total);
                    step = -1;
                    change(0, t);
                } else if (a.pressed(gamepad1.a)) {
                    RunLog.event("STEP sequence " + (step >= 0 ? "restarted" : "started") + ": " + SETPOINTS.length + " setpoints, " + Math.max(1, RUNS) + " times");
                    step = 0;
                    change(SETPOINTS[0], t);
                }
                boolean u = up.pressed(gamepad1.dpad_up), d = down.pressed(gamepad1.dpad_down), r = right.pressed(gamepad1.dpad_right), l = left.pressed(gamepad1.dpad_left);
                if (step < 0 && (u || d || r || l)) {
                    if (fly.target() <= 0) change(FROM_OFF, t);
                    else change(fly.target() + (u ? BIG_STEP : d ? -BIG_STEP : r ? SMALL_STEP : -SMALL_STEP), t);
                }
                fly.update(t);
                if (timing) {
                    lowest = Math.min(lowest, fly.filtered());
                    highest = Math.max(highest, fly.filtered());
                    if (fly.inBand()) {
                        if (inBandSince < 0) inBandSince = t;
                    } else if (!settled) {
                        inBandSince = -1;
                    }
                    if (!settled && inBandSince >= 0 && t - inBandSince >= SETTLED_S) {
                        settled = true;
                        settledAt = inBandSince;     // the time to settle is the time to enter the band for good
                    }
                    boolean over = settled ? t - (settledAt + SETTLED_S) >= AFTER_SETTLED_S : t - changedAt >= NEVER_SETTLED_S;
                    if (over) {
                        finish(t, false);
                        timing = false;
                        if (step >= 0) {
                            step++;
                            if (step < total) {
                                change(SETPOINTS[step % SETPOINTS.length], t);
                            } else {
                                step = -1;
                                RunLog.event("STEP TABLE" + table);
                                changesInTable = changes;
                                change(0, t);
                            }
                        }
                    }
                }
                RunLog.tick();
                telemetry.addLine(step < 0 ? "sequence stopped (A runs it)" : "step " + (step + 1) + " of " + total + " (B stops)");
                telemetry.addLine(String.format(Locale.US, "target %.0f, speed %.0f %s", fly.target(), fly.filtered(), fly.inBand() ? "IN BAND" : ""));
                telemetry.addLine("last: " + lastLine);
                telemetry.update();
                pad.pad(LOOP_MS);
            }
        } finally {
            fly.stopNow();
            if (changes > changesInTable) RunLog.event("STEP TABLE" + table);
        }
    }
}
