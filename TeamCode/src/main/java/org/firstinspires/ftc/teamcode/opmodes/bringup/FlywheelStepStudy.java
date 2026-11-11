package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * How fast does the flywheel track a change of its setpoint? A step-response study: the flywheel ALONE (the real {@link Flywheel} class, its
 * bang-bang plus FEED_FORWARD, nothing else on the robot is driven) is sent through a list of setpoints, and every change of the setpoint is
 * timed until the wheel is in the LAUNCH BAND (where a salvo may start) and has settled there. The loop is padded to LOOP_TARGET_MS, the
 * TeleOp's loop time, because the bang-bang only switches once per loop.
 *
 * The robot can sit anywhere (on blocks is fine); take the pollen OUT first: the pusher is not driven, but the wheel spins up to 1650 ticks/s.
 *
 * What each change of setpoint measures, from the loop where the setpoint changed (t = 0), using the hub's speed reading:
 *   band     the first time the FILTERED speed (5 samples, as the launcher uses) is inside the launch band (Flywheel.LAUNCH_BAND_LOW_FRACTION to
 *            _HIGH_FRACTION of the new setpoint): the earliest a salvo could start; 0 when the old speed was already inside it
 *   settled  the start of the first stretch of SETTLE_SEC with the filtered speed in the band the whole time
 *   10-90    the time the raw speed takes from 10 to 90 percent of the way from the old speed to the new setpoint, and the RATE that gives
 *            (ticks/s per second): full power on the way up, the feedforward (a coast) on the way down. Meaningless for steps of a few ripples
 *   cross    the first time the raw speed reaches the new setpoint
 *   over     how far the raw speed went PAST the new setpoint after crossing it (ticks/s; up: above it, down: below it)
 *   steady   after settling: the raw speed's mean error, its lowest and highest, and the fraction of loops at full power (the duty cycle)
 * A step ends STEADY_SEC after it settles, or MAX_STEP_SEC after it began if it never settles ("NEVER SETTLED": e.g. a feedforward too high
 * to let the wheel come down to a low setpoint).
 *
 * Gamepad 1 (Start is not used):
 *   A              run the sequence (STEPS, REPEATS times; from rest, the first entry is a spin-up); A again restarts it
 *   B              stop: flywheel off, sequence stopped
 *   D-pad up/down  (sequence not running) setpoint +100 / -100; right/left +25 / -25; from off, any D-pad press spins up to MANUAL_START.
 *                  Every manual change is measured like a sequence step.
 *
 * Log kind bringup (bringup.log), laid out for offline analysis: a row EVERY loop with the time of the speed reading (t_step since the setpoint change,
 * age_ms before the row), the hub's speed, the filtered speed, the encoder POSITION from the same hub read (so the speed can be recomputed offline,
 * independent of the hub's own calculation), the motor command written after the reading, the current, and the in-band flag; a STEP n START line at
 * each change of setpoint and a STEP n line with its measurements when it ends; a STEP TABLE at the end with the mean of each from -> to pair.
 * Bulk caching is AUTO on every hub, as in the TeleOp (MecanumDrive sets it), so the position comes from the speed's bulk read at no extra cost.
 * design/opmodes.md, design/run-logs.md (columns).
 */
@Config
@TeleOp(name = "Flywheel Step Study", group = "Bring-up")
public class FlywheelStepStudy extends LinearOpMode {

    /** The setpoints visited in order (ticks/s). The default covers the launch map (1325 to 1650): steps up and down of 25 to 325. */
    public static String STEPS = "1350,1650,1350,1450,1350,1375,1325,1650,1450,1650,1325";
    /** How many times the sequence runs (each repeat continues from the last setpoint, so only the first starts from rest). */
    public static int REPEATS = 2;
    /** The filtered speed must stay in the launch band this long to count as settled. */
    public static double SETTLE_SEC = 0.3;
    /** How long a step goes on after it settles (the steady part is measured over this). */
    public static double STEADY_SEC = 1.5;
    /** A step that has not settled by this time is ended and marked NEVER SETTLED. */
    public static double MAX_STEP_SEC = 8.0;
    /** The rest before the first spin-up of a run (the wheel must be still for the spin-up to start from 0). */
    public static double REST_SEC = 1.0;
    /** Pad each loop to this many ms (the TeleOp's loop is about 25 to 28 ms); 0 runs as fast as the hub allows. */
    public static double LOOP_TARGET_MS = 28.0;
    /** The setpoint the D-pad starts from when the flywheel is off. */
    public static double MANUAL_START = 1350.0;
    public static double MAX_TARGET = 2000.0;

    /** One change of setpoint, measured from the loop it happened in. */
    private static final class Step {
        final int n;
        final String source;
        final double from, to, startSpeed, volt, feedForward;
        final long t0Ns;
        double tBand = Double.NaN, tSettled = Double.NaN, t10 = Double.NaN, t90 = Double.NaN, tCross = Double.NaN;
        double inBandSince = Double.NaN;
        double over = 0.0;
        double steadySum = 0.0, steadyMin = Double.MAX_VALUE, steadyMax = -Double.MAX_VALUE;
        int steadyLoops = 0, steadyFull = 0;
        double ended = Double.NaN;

        Step(int n, String source, double from, double to, double startSpeed, double volt, long t0Ns) {
            this.n = n;
            this.source = source;
            this.from = from;
            this.to = to;
            this.startSpeed = startSpeed;
            this.volt = volt;
            this.feedForward = Flywheel.FEED_FORWARD;
            this.t0Ns = t0Ns;
        }

        boolean up() {
            return to >= startSpeed;
        }

        /** One loop: t seconds since the change, the raw and filtered speed, whether the wheel was at full power. */
        void sample(double t, double raw, double filtered, boolean full) {
            double change = to - startSpeed;
            if (Math.abs(change) > 1e-6) {
                double frac = (raw - startSpeed) / change;
                if (Double.isNaN(t10) && frac >= 0.1) t10 = t;
                if (Double.isNaN(t90) && frac >= 0.9) t90 = t;
                if (Double.isNaN(tCross) && frac >= 1.0) tCross = t;
            } else if (Double.isNaN(tCross)) {
                tCross = t;
            }
            if (!Double.isNaN(tCross)) over = Math.max(over, up() ? raw - to : to - raw);

            boolean inBand = filtered >= to * Flywheel.LAUNCH_BAND_LOW_FRACTION && filtered <= to * Flywheel.LAUNCH_BAND_HIGH_FRACTION;
            if (inBand) {
                if (Double.isNaN(tBand)) tBand = t;
                if (Double.isNaN(inBandSince)) inBandSince = t;
                if (Double.isNaN(tSettled) && t - inBandSince >= SETTLE_SEC) tSettled = inBandSince;
            } else {
                inBandSince = Double.NaN;
            }
            if (!Double.isNaN(tSettled) && t - tSettled >= SETTLE_SEC) {
                steadySum += raw - to;
                steadyMin = Math.min(steadyMin, raw);
                steadyMax = Math.max(steadyMax, raw);
                steadyLoops++;
                if (full) steadyFull++;
            }
        }

        /** Done once it has been steady for STEADY_SEC, or at MAX_STEP_SEC. */
        boolean done(double t) {
            return (!Double.isNaN(tSettled) && t - tSettled >= SETTLE_SEC + STEADY_SEC) || t >= MAX_STEP_SEC;
        }

        double rate() {
            return Double.isNaN(t10) || Double.isNaN(t90) || t90 <= t10 ? Double.NaN : 0.8 * Math.abs(to - startSpeed) / (t90 - t10);
        }

        String line() {
            String steady = steadyLoops == 0 ? "steady -" : String.format(Locale.US, "steady error %+.0f mean, %.0f to %.0f, full power %.0f%% of %d loops",
                    steadySum / steadyLoops, steadyMin, steadyMax, 100.0 * steadyFull / steadyLoops, steadyLoops);
            return String.format(Locale.US, "STEP %d (%s): %.0f -> %.0f (%s %.0f, speed at the change %.0f), %.2f V, FF %.2f | band %s, settled %s, 10-90 %s (%s ticks/s per s), "
                            + "cross %s, over %.0f | %s | ended %.2f s",
                    n, source, from, to, up() ? "UP" : "DOWN", Math.abs(to - startSpeed), startSpeed, volt, feedForward,
                    sec(tBand), Double.isNaN(tSettled) ? "NEVER SETTLED" : sec(tSettled),
                    Double.isNaN(t10) || Double.isNaN(t90) ? "-" : sec(t90 - t10), Double.isNaN(rate()) ? "-" : String.format(Locale.US, "%.0f", rate()),
                    sec(tCross), over, steady, ended);
        }

        String shortLine() {
            return String.format(Locale.US, "#%d %.0f->%.0f  band %s  settled %s  rate %s  over %.0f", n, from, to, sec(tBand),
                    Double.isNaN(tSettled) ? "NEVER" : sec(tSettled), Double.isNaN(rate()) ? "-" : String.format(Locale.US, "%.0f", rate()), over);
        }
    }

    private static String sec(double s) {
        return Double.isNaN(s) ? "-" : String.format(Locale.US, "%.2f s", s);
    }

    private static List<Double> parseSteps() {
        List<Double> out = new ArrayList<>();
        for (String part : STEPS.split("[,; ]+")) {
            try {
                double v = Double.parseDouble(part.trim());
                if (v > 0.0) out.add(Math.min(v, MAX_TARGET));
            } catch (NumberFormatException ignored) {
                // skip it
            }
        }
        return out;
    }

    private final List<Step> steps = new ArrayList<>();
    private Step current;
    private double logTarget, logVel, logFilt, logPower, logAmps, logTStep = Double.NaN, logUpdMs;
    private int logBand, logPos;
    /** System.nanoTime() just before this loop's speed read was sent (the first thing Flywheel.update() does). */
    private long readNs;

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Flywheel Step Study", hardwareMap);
        try {
            runStudy();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private double volts() {
        try {
            return hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    /** Ends the step in progress (if any) and writes its STEP line. */
    private void finishStep(double t) {
        if (current == null) return;
        current.ended = t;
        RunLog.event(current.line());
        current = null;
    }

    /** A new setpoint: ends the step in progress and starts measuring the new one. */
    private void changeTarget(Flywheel flywheel, double target, String source) {
        double from = flywheel.getTargetVelocity();
        double volt = target > 0.0 ? volts() : Double.NaN;   // read before the clock starts: it is a hub round trip of its own
        long now = System.nanoTime();
        if (current != null) finishStep((now - current.t0Ns) / 1e9);
        flywheel.setTargetVelocity(target);
        if (target > 0.0) {
            current = new Step(steps.size() + 1, source, from, target, flywheel.getVelocity(), volt, now);
            steps.add(current);
            RunLog.event(String.format(Locale.US, "STEP %d START (%s): %.0f -> %.0f, speed at the change %.0f (the reading of the loop before), %.2f V, FF %.2f;"
                            + " the new power is written in this loop's update, after its speed reading",
                    current.n, source, from, target, current.startSpeed, volt, current.feedForward));
        } else if (from > 0.0) {
            RunLog.event(String.format(Locale.US, "FLYWHEEL OFF (%s): %.0f -> 0", source, from));
        }
    }

    private void runStudy() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // AUTO bulk caching on every hub, as in the TeleOp: the speed read is one bulk read, and the position read right after it comes from the same packet
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        Flywheel flywheel = new Flywheel(hardwareMap);
        DcMotorEx flyMotor = hardwareMap.get(DcMotorEx.class, "fly_motor_enc");   // the same motor, for its encoder position only

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Flywheel Step Study: times how fast the flywheel follows a change of its setpoint.");
            telemetry.addLine("Take the pollen out of the robot. Only the flywheel runs.");
            telemetry.addData("Sequence", "%s, %d times", STEPS, REPEATS);
            telemetry.addData("Feedforward / launch band", "%.2f / %.2f to %.2f", Flywheel.FEED_FORWARD, Flywheel.LAUNCH_BAND_LOW_FRACTION, Flywheel.LAUNCH_BAND_HIGH_FRACTION);
            telemetry.addLine("After Start: A runs the sequence, B stops, D-pad steps the setpoint by hand.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.event(String.format(Locale.US, "SETUP flywheel step study: STEPS %s x %d, FEED_FORWARD %.2f, launch band %.2f to %.2f, SETTLE_SEC %.2f, STEADY_SEC %.2f, MAX_STEP_SEC %.1f, "
                        + "loop target %.0f ms, bulk caching AUTO, ENCODER_SIGN %+.0f", STEPS, REPEATS, Flywheel.FEED_FORWARD, Flywheel.LAUNCH_BAND_LOW_FRACTION,
                Flywheel.LAUNCH_BAND_HIGH_FRACTION, SETTLE_SEC, STEADY_SEC, MAX_STEP_SEC, LOOP_TARGET_MS, Flywheel.ENCODER_SIGN));
        RunLog.setSampler("step,t_step,age_ms,upd_ms,target,vel,filt,pos,power,amps,in_band", 0,
                () -> String.format(Locale.US, "%d,%.4f,%.1f,%.1f,%.0f,%.1f,%.1f,%d,%.3f,%.2f,%d", current != null ? current.n : 0, logTStep,
                        (System.nanoTime() - readNs) / 1e6, logUpdMs, logTarget, logVel, logFilt, logPos, logPower, logAmps, logBand));
        RunLog.setBurst(true);   // a row every loop

        List<Double> sequence = new ArrayList<>();
        int seqIndex = -1;            // the entry being held; -1 = not running
        boolean resting = false;
        ElapsedTime restTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        double loopSum = 0.0, loopMax = 0.0;
        long loops = 0;

        while (opModeIsActive()) {
            loopTimer.reset();
            boolean running = seqIndex >= 0;

            // ---- commands ----
            if (gamepad1.aWasPressed()) {
                sequence.clear();
                List<Double> one = parseSteps();
                for (int r = 0; r < Math.max(1, REPEATS); r++) sequence.addAll(one);
                if (sequence.isEmpty()) {
                    RunLog.event("SEQUENCE not started: STEPS has no setpoints (" + STEPS + ")");
                } else {
                    RunLog.event(String.format(Locale.US, "SEQUENCE started: %d setpoints %s", sequence.size(), sequence));
                    changeTarget(flywheel, 0.0, "rest");
                    resting = true;
                    restTimer.reset();
                    seqIndex = 0;
                    running = true;
                }
            }
            if (gamepad1.bWasPressed()) {
                if (running) RunLog.event("SEQUENCE stopped by B at entry " + (seqIndex + 1) + " of " + sequence.size());
                changeTarget(flywheel, 0.0, "B");
                seqIndex = -1;
                resting = false;
                running = false;
            }
            if (!running) {
                double delta = gamepad1.dpadUpWasPressed() ? 100 : gamepad1.dpadDownWasPressed() ? -100
                        : gamepad1.dpadRightWasPressed() ? 25 : gamepad1.dpadLeftWasPressed() ? -25 : 0;
                if (delta != 0) {
                    double now = flywheel.getTargetVelocity();
                    double next = now <= 0.0 ? MANUAL_START : Math.max(100.0, Math.min(MAX_TARGET, now + delta));
                    if (next != now) changeTarget(flywheel, next, "manual");
                }
            } else {
                // the sequence: rest, then each entry until its step is done
                if (resting) {
                    if (restTimer.seconds() >= REST_SEC) {
                        resting = false;
                        changeTarget(flywheel, sequence.get(0), "seq 1/" + sequence.size());
                    }
                } else if (current == null || current.done((System.nanoTime() - current.t0Ns) / 1e9)) {
                    seqIndex++;
                    if (seqIndex >= sequence.size()) {
                        RunLog.event("SEQUENCE finished");
                        changeTarget(flywheel, 0.0, "end");
                        seqIndex = -1;
                    } else if (sequence.get(seqIndex).doubleValue() != flywheel.getTargetVelocity()) {
                        changeTarget(flywheel, sequence.get(seqIndex), "seq " + (seqIndex + 1) + "/" + sequence.size());
                    } else {
                        RunLog.event("SEQUENCE entry " + (seqIndex + 1) + " is the same setpoint as before: skipped");
                    }
                }
            }

            // ---- the flywheel and the measurement ----
            readNs = System.nanoTime();   // update() sends the speed read first; the hub answers in a few ms
            flywheel.update();
            logUpdMs = (System.nanoTime() - readNs) / 1e6;   // the read, plus the power write when it changed and the current read every 250 ms
            logPos = (int) Math.round(Flywheel.ENCODER_SIGN * flyMotor.getCurrentPosition());   // from the same bulk read
            double raw = flywheel.getVelocity(), filt = flywheel.getFilteredVelocity();
            logTStep = current != null ? (readNs - current.t0Ns) / 1e9 : Double.NaN;
            if (current != null) current.sample(logTStep, raw, filt, flywheel.getLastPower() >= 1.0);
            logAmps = flywheel.getCurrentAmps();
            logTarget = flywheel.getTargetVelocity();
            logVel = raw;
            logFilt = filt;
            logPower = flywheel.getLastPower();
            logBand = flywheel.isInLaunchBand() ? 1 : 0;

            // ---- screen ----
            telemetry.addData("State", seqIndex >= 0 ? (resting ? "SEQUENCE: resting before the spin-up" : "SEQUENCE entry " + (seqIndex + 1) + " of " + sequence.size())
                    : "manual (A runs the sequence)");
            telemetry.addData("Setpoint / speed / filtered", "%.0f / %.0f / %.0f ticks/s", logTarget, raw, filt);
            telemetry.addData("Power / in band", "%.2f / %s", logPower, flywheel.launchBandState());
            if (current != null) {
                double t = (System.nanoTime() - current.t0Ns) / 1e9;
                telemetry.addData("This step", "#%d %.0f -> %.0f, %.1f s, band %s, settled %s", current.n, current.from, current.to, t, sec(current.tBand), sec(current.tSettled));
            }
            for (int i = steps.size() - 1, shown = 0; i >= 0 && shown < 6; i--) {
                if (steps.get(i) == current) continue;
                telemetry.addLine(steps.get(i).shortLine());
                shown++;
            }
            telemetry.addData("Loop", "%.1f ms mean, %.0f ms max (target %.0f)", loops > 0 ? loopSum / loops : 0.0, loopMax, LOOP_TARGET_MS);
            telemetry.addLine("A run the sequence, B stop, D-pad up/down +-100, right/left +-25 (not during the sequence)");
            telemetry.update();

            // pad the loop to the TeleOp's loop time: the bang-bang only switches once per loop
            while (LOOP_TARGET_MS > 0 && loopTimer.milliseconds() < LOOP_TARGET_MS && opModeIsActive()) sleep(1);
            double ms = loopTimer.milliseconds();
            loopSum += ms;
            loopMax = Math.max(loopMax, ms);
            loops++;
            RunLog.tick();
        }

        if (current != null) finishStep((System.nanoTime() - current.t0Ns) / 1e9);
        flywheel.stop();
        logTable(loops > 0 ? loopSum / loops : 0.0, loopMax);
    }

    /** The mean of each from -> to pair over its repeats. */
    private void logTable(double loopMean, double loopMax) {
        Map<String, List<Step>> byPair = new LinkedHashMap<>();
        for (Step s : steps) {
            String key = String.format(Locale.US, "%.0f -> %.0f", s.from, s.to);
            List<Step> list = byPair.get(key);
            if (list == null) {
                list = new ArrayList<>();
                byPair.put(key, list);
            }
            list.add(s);
        }
        RunLog.event(String.format(Locale.US, "STEP TABLE: %d steps, loop %.1f ms mean, %.0f ms max. Per pair: runs, band (mean, worst), settled (mean, worst, never), rate, over (mean, worst)",
                steps.size(), loopMean, loopMax));
        for (Map.Entry<String, List<Step>> e : byPair.entrySet()) {
            double bandSum = 0, bandMax = 0, setSum = 0, setMax = 0, rateSum = 0, overSum = 0, overMax = 0;
            int bandN = 0, setN = 0, never = 0, rateN = 0;
            for (Step s : e.getValue()) {
                if (!Double.isNaN(s.tBand)) {
                    bandSum += s.tBand;
                    bandMax = Math.max(bandMax, s.tBand);
                    bandN++;
                }
                if (!Double.isNaN(s.tSettled)) {
                    setSum += s.tSettled;
                    setMax = Math.max(setMax, s.tSettled);
                    setN++;
                } else if (!Double.isNaN(s.ended) && s.ended >= MAX_STEP_SEC) {
                    never++;
                }
                if (!Double.isNaN(s.rate())) {
                    rateSum += s.rate();
                    rateN++;
                }
                overSum += s.over;
                overMax = Math.max(overMax, s.over);
            }
            int n = e.getValue().size();
            RunLog.event(String.format(Locale.US, "STEP TABLE %s: %d runs | band %s, worst %s | settled %s, worst %s, never %d | rate %s ticks/s per s | over %.0f, worst %.0f",
                    e.getKey(), n, bandN > 0 ? sec(bandSum / bandN) : "-", bandN > 0 ? sec(bandMax) : "-",
                    setN > 0 ? sec(setSum / setN) : "-", setN > 0 ? sec(setMax) : "-", never,
                    rateN > 0 ? String.format(Locale.US, "%.0f", rateSum / rateN) : "-", overSum / n, overMax));
        }
    }
}
