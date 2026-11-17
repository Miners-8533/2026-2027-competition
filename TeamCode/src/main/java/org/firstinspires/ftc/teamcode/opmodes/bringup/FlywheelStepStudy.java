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
 * Flywheel Step Study (specs/support-opmodes.md section 3): how fast does the flywheel follow a setpoint change?
 * The flywheel alone steps through a list of setpoints; each change is timed until the speed is in the launch band
 * and has stayed there. Not yet run on the robot.
 *
 * SPEC GAP: the list of setpoints and how long "stayed there" is are not in the specs; the values below are this
 * rebuild's choice. Press A to start; B stops the wheel.
 */
@Config
@TeleOp(name = "Flywheel Step Study", group = "2 Bring-up")
public final class FlywheelStepStudy extends TeamOpMode {
    public static double[] SETPOINTS = {1350, 1450, 1350, 1650, 1325, 1450, 0, 1350};
    public static double STAYED_S = 0.5;
    public static double STEP_LIMIT_S = 6.0;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.BRINGUP; }
    @Override protected String title() { return "Flywheel Step Study"; }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        final Flywheel fly = Flywheel.create(hardwareMap, profile);
        FlywheelPosition pos = FlywheelPosition.create(hardwareMap);
        telemetry.addLine(fly == null ? "NO FLYWHEEL in the configuration" : "Flywheel Step Study: Start, then A to begin. B stops the wheel.");
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
        int step = -1;
        double stepAt = 0, inBandSince = -1, from = 0;
        String results = "";
        try {
            while (opModeIsActive()) {
                double t = Clock.now();
                if (gamepad1.b) {
                    step = SETPOINTS.length;
                    fly.setTarget(0);
                }
                if (step < 0 && gamepad1.a) {
                    step = 0;
                    stepAt = t;
                    inBandSince = -1;
                    from = fly.filtered();
                    fly.setTarget(SETPOINTS[0]);
                }
                fly.update(t);
                if (step >= 0 && step < SETPOINTS.length) {
                    double sp = SETPOINTS[step];
                    boolean in = sp <= 0 ? fly.filtered() < 100 : fly.inBand();
                    if (in) {
                        if (inBandSince < 0) inBandSince = t;
                    } else {
                        inBandSince = -1;
                    }
                    boolean stayed = inBandSince >= 0 && t - inBandSince >= STAYED_S, limit = t - stepAt >= STEP_LIMIT_S;
                    if (stayed || limit) {
                        String line = String.format(Locale.US, "%.0f -> %.0f: %s", from, sp,
                                stayed ? String.format(Locale.US, "in the band after %.2f s", inBandSince - stepAt) : "NOT in the band within " + STEP_LIMIT_S + " s");
                        RunLog.event("FLYWHEEL STEP " + line + String.format(Locale.US, ", battery %.2f V", battery()));
                        results += line + "\n";
                        step++;
                        if (step < SETPOINTS.length) {
                            stepAt = t;
                            inBandSince = -1;
                            from = fly.filtered();
                            fly.setTarget(SETPOINTS[step]);
                        } else {
                            fly.setTarget(0);
                        }
                    }
                }
                RunLog.tick();
                telemetry.addLine(step < 0 ? "A to begin" : step >= SETPOINTS.length ? "done" : "step " + (step + 1) + " of " + SETPOINTS.length);
                telemetry.addLine(String.format(Locale.US, "target %.0f, speed %.0f", fly.target(), fly.filtered()));
                telemetry.addLine(results);
                telemetry.update();
            }
        } finally {
            fly.stopNow();
        }
    }
}
