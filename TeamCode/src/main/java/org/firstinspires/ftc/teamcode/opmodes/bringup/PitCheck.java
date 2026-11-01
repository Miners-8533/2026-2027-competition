package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The pit check: run it with the robot ON BLOCKS (wheels off the floor) after any repair, rewiring or a hub swap, and
 * before a match day. It finds the wiring and sign problems that code no longer watches for while running: a missing or
 * misnamed device, a reversed or dead encoder, a wrong motor direction, a color sensor that is not answering, a weak
 * battery. Nothing moves until you press A for that step, every move is short and slow, and Back stops everything.
 *
 * Automatic, no motion (right after Start): configuration, battery, Pinpoint, color sensors (the track must be EMPTY).
 * Then, one step at a time, each asking first:
 *   flywheel   spins at low power: the encoder must count the right way (automatic), the wheel must turn the shooting way (you judge)
 *   pusher     turns the arm about PUSHER_TEST_DEG forward and back to where it started (the encoder sign and the return are
 *              automatic, the pushing direction is yours to judge)
 *   intake     pulses IN then OUT (you judge the direction; the motor current is shown)
 *   ramp       deploys and stows (you judge that the pair moves together, mirrored)
 *   drive      each wheel alone for a moment (you judge that its top moves toward the robot's FRONT)
 *
 * Gamepad 1:  A = run the step / what you saw was right      B = what you saw was WRONG
 *             X = skip the step / cannot tell                Back = stop everything and end
 *
 * Every result (PASS, FAIL or SKIP, with the reason and the fix) is on screen, written to the robot log
 * (adb logcat -s PitCheck) and to /sdcard/FIRST/runlogs/pitcheck.log (see RunLog). The robot is left as it was found:
 * the pusher returns to where it started. If it cannot, the report says by how much; put it back at HOLD before running
 * any other opmode.
 *
 * It uses the raw hardware, not the Pusher or Flywheel classes, so it never changes the pusher's saved zero.
 */
@Config
@TeleOp(name = "Pit Check (robot on blocks)", group = "Bring-up")
public class PitCheck extends LinearOpMode {

    public static double MIN_BATTERY_V = 12.0;
    public static double FLYWHEEL_POWER = 0.3;
    public static double FLYWHEEL_SPIN_SEC = 1.5;
    /** The measured speed must reach this (ticks/s, after ENCODER_SIGN) to count as counting the right way. */
    public static double FLYWHEEL_MIN_TICKS_PER_SEC = 100.0;
    public static double PUSHER_POWER = 0.25;
    public static double PUSHER_TEST_DEG = 60.0;
    public static double PUSHER_MIN_MOVE_DEG = 15.0;
    public static double PUSHER_TIMEOUT_SEC = 2.0;
    /** The arm must end this close (deg) to where it started. */
    public static double PUSHER_RETURN_TOLERANCE_DEG = 10.0;
    public static double INTAKE_POWER_SCALE = 0.5;
    public static double INTAKE_PULSE_SEC = 0.8;
    public static double DRIVE_POWER = 0.3;
    public static double DRIVE_PULSE_SEC = 0.6;
    public static double PINPOINT_MIN_LOOP_US = 300.0;
    public static double PINPOINT_MAX_LOOP_US = 1500.0;

    private static final String TAG = "PitCheck";
    private static final String[] DRIVE_MOTORS = {"leftFront", "leftBack", "rightFront", "rightBack"};
    private static final String[] OTHER_MOTORS = {"intake", "fly_motor_enc", "gate"};

    private enum Verdict { PASS, FAIL, SKIP }

    private enum Choice { YES, NO, SKIP, ABORT }

    private final List<String[]> results = new ArrayList<>();
    private boolean aborted = false;
    private ChassisProfile profile;

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.PITCHECK, "Pit Check", hardwareMap);
        try {
            runCheck();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            RunLog.close();
        }
    }

    private void runCheck() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        profile = ChassisProfile.detect(hardwareMap);

        telemetry.addData("Chassis profile", profile.label());
        telemetry.addLine("PIT CHECK. Put the robot ON BLOCKS, the wheels clear of the floor and the ramp free to swing.");
        telemetry.addLine("Keep the element track EMPTY. Nothing moves until you press A for that step.");
        telemetry.addLine("Gamepad 1: A = run / it was right, B = it was wrong, X = skip / cannot tell, Back = stop everything.");
        telemetry.addLine("Press Start on the Driver Station to begin.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.event("SETUP profile " + profile.name);

        checkConfiguration();
        checkBattery();
        checkPinpoint();
        checkColorSensors();

        if (!aborted) flywheelStep();
        if (!aborted) pusherStep();
        if (!aborted) intakeStep();
        if (!aborted) rampStep();
        for (String name : DRIVE_MOTORS) {
            if (!aborted) driveStep(name);
        }
        finish();
    }

    // ---- automatic checks (no motion) ----

    private void checkConfiguration() {
        List<String> missing = new ArrayList<>();
        for (String name : DRIVE_MOTORS) if (hardwareMap.tryGet(DcMotorEx.class, name) == null) missing.add(name + " (motor)");
        for (String name : OTHER_MOTORS) if (hardwareMap.tryGet(DcMotorEx.class, name) == null) missing.add(name + " (motor)");
        for (String name : new String[]{"depolLeft", "depolRight"}) if (hardwareMap.tryGet(Servo.class, name) == null) missing.add(name + " (servo)");
        for (String name : ElementCounter.NAMES) if (hardwareMap.tryGet(NormalizedColorSensor.class, name) == null) missing.add(name + " (REV color sensor)");
        if (hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint") == null) missing.add("pinpoint (goBILDA Pinpoint)");
        if (missing.isEmpty()) {
            record("Configuration", Verdict.PASS, "all 4 drive motors, intake, flywheel, pusher, 2 ramp servos, 4 color sensors and the Pinpoint are configured");
        } else {
            record("Configuration", Verdict.FAIL, "MISSING or the wrong type: " + missing
                    + ". Fix the names or types in Configure Robot (exact spelling, case matters), save, activate, and rerun.");
        }
        if (hardwareMap.tryGet(DcMotorEx.class, "fly_follow") != null) {
            record("Configuration (stale)", Verdict.FAIL, "'fly_follow' is still configured but the code no longer uses it: delete it");
        }
    }

    private void checkBattery() {
        double min = Double.POSITIVE_INFINITY;
        StringBuilder all = new StringBuilder();
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            min = Math.min(min, v);
            all.append(String.format(Locale.US, "%s %.2f V  ", sensor.getDeviceName(), v));
        }
        if (min == Double.POSITIVE_INFINITY) {
            record("Battery", Verdict.FAIL, "no voltage sensor found");
        } else if (min < MIN_BATTERY_V) {
            record("Battery", Verdict.FAIL, String.format(Locale.US, "%.2f V is below %.1f V: charge or swap the battery. %s", min, MIN_BATTERY_V, all));
        } else {
            record("Battery", Verdict.PASS, all.toString().trim());
        }
    }

    private void checkPinpoint() {
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");
        if (pinpoint == null) {
            record("Pinpoint", Verdict.FAIL, "not configured");
            return;
        }
        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus st = pinpoint.getDeviceStatus();
        int loop = pinpoint.getLoopTime();
        if (st != GoBildaPinpointDriver.DeviceStatus.READY) {
            record("Pinpoint", Verdict.FAIL, "status " + st + " (not READY): check the pod cables and the I2C cable, then restart the robot");
        } else if (loop < PINPOINT_MIN_LOOP_US || loop > PINPOINT_MAX_LOOP_US) {
            record("Pinpoint", Verdict.FAIL, "READY but the loop time is " + loop + " us (expected about 500-1100): check the I2C cable");
        } else {
            record("Pinpoint", Verdict.PASS, "READY, loop " + loop + " us");
        }
    }

    private void checkColorSensors() {
        for (int i = 0; i < ElementCounter.N; i++) {
            String name = ElementCounter.NAMES[i];
            NormalizedColorSensor sensor = hardwareMap.tryGet(NormalizedColorSensor.class, name);
            if (sensor == null) {
                record(name, Verdict.FAIL, "not configured");
                continue;
            }
            if (sensor instanceof ColorSensor) ((ColorSensor) sensor).enableLed(true);
            sleep(150);
            sensor.getNormalizedColors();   // the first read after the LED comes on can be old
            sleep(120);
            NormalizedRGBA c = sensor.getNormalizedColors();
            double dist = sensor instanceof DistanceSensor ? ((DistanceSensor) sensor).getDistance(DistanceUnit.CM) : Double.NaN;
            double sum = c.red + c.green + c.blue + 1e-9;
            double r = c.red / sum, g = c.green / sum, b = c.blue / sum;
            String reading = String.format(Locale.US, "alpha %.3f, distance %.2f cm, shares %.2f/%.2f/%.2f", c.alpha, dist, r, g, b);
            if (c.alpha <= 0.0 || Double.isNaN(dist) || dist > 100.0) {
                record(name, Verdict.FAIL, "no usable reading (" + reading + "): check the cable and that it is on its own I2C bus");
            } else if (!ElementCounter.looksEmpty(i, r, g, b, c.alpha, dist)) {
                record(name, Verdict.FAIL, "answers, but does not match its EMPTY baseline (" + reading + "): something is in the track, or the lighting changed (re-measure the baseline with Color Sensors Test)");
            } else {
                record(name, Verdict.PASS, reading);
            }
        }
    }

    // ---- steps that move something ----

    private void flywheelStep() {
        if (!askRun("Flywheel", String.format(Locale.US, "Spins the flywheel at %.2f power for %.1f s. Keep everything clear of the wheel.", FLYWHEEL_POWER, FLYWHEEL_SPIN_SEC))) return;
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, "fly_motor_enc");
        if (motor == null) {
            record("Flywheel", Verdict.FAIL, "fly_motor_enc is not configured");
            return;
        }
        motor.setDirection(profile.flywheelDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double peak = 0.0, lowest = 0.0, amps = 0.0;
        ElapsedTime timer = new ElapsedTime();
        motor.setPower(FLYWHEEL_POWER);
        while (motionTick() && timer.seconds() < FLYWHEEL_SPIN_SEC) {
            double v = Flywheel.ENCODER_SIGN * motor.getVelocity();
            peak = Math.max(peak, v);
            lowest = Math.min(lowest, v);
            show("Flywheel spinning", String.format(Locale.US, "speed %.0f ticks/s", v));
        }
        amps = motor.getCurrent(CurrentUnit.AMPS);
        motor.setPower(0.0);
        waitForSpinDown(motor);
        if (aborted) return;

        if (peak >= FLYWHEEL_MIN_TICKS_PER_SEC) {
            record("Flywheel encoder", Verdict.PASS, String.format(Locale.US, "counts the right way: %.0f ticks/s at %.2f power, %.1f A", peak, FLYWHEEL_POWER, amps));
        } else if (lowest <= -FLYWHEEL_MIN_TICKS_PER_SEC) {
            record("Flywheel encoder", Verdict.FAIL, String.format(Locale.US, "reads BACKWARD (%.0f ticks/s). Flip Flywheel.ENCODER_SIGN, unless the wheel also turned the wrong way (then flip flywheelDirection in ChassisProfile and recheck)", lowest));
        } else {
            record("Flywheel encoder", Verdict.FAIL, "reads about 0 while the motor runs: no encoder counts (check the encoder cable and port), or the motor did not turn");
        }
        judge("Flywheel direction", "Did the flywheel turn the SHOOTING way (the way that throws an element out)?",
                "flip flywheelDirection for this chassis in ChassisProfile (and then Flywheel.ENCODER_SIGN if the encoder check fails)");
    }

    private void waitForSpinDown(DcMotorEx motor) {
        ElapsedTime timer = new ElapsedTime();
        while (motionTick() && timer.seconds() < 6.0 && Math.abs(motor.getVelocity()) > 50.0) {
            show("Flywheel spinning down", String.format(Locale.US, "speed %.0f ticks/s", motor.getVelocity()));
        }
    }

    private void pusherStep() {
        if (!askRun("Pusher", String.format(Locale.US, "Turns the pusher arm about %.0f deg forward at %.2f power, then back to where it started. Keep clear of the arm and the track.", PUSHER_TEST_DEG, PUSHER_POWER))) return;
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, "gate");
        if (motor == null) {
            record("Pusher", Verdict.FAIL, "gate is not configured");
            return;
        }
        motor.setDirection(profile.pusherDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticksPerDeg = Pusher.TICKS_PER_REV / 360.0;
        int start = motor.getCurrentPosition();
        double targetTicks = PUSHER_TEST_DEG * ticksPerDeg;
        double stopEarlyTicks = Pusher.STOP_EARLY_DEG * ticksPerDeg;

        double peakVelocity = 0.0, amps = 0.0;
        ElapsedTime timer = new ElapsedTime();
        motor.setPower(PUSHER_POWER);
        while (motionTick() && timer.seconds() < PUSHER_TIMEOUT_SEC && Math.abs(motor.getCurrentPosition() - start) < targetTicks - stopEarlyTicks) {
            peakVelocity = Math.max(peakVelocity, Math.abs(motor.getVelocity()));
            amps = Math.max(amps, motor.getCurrent(CurrentUnit.AMPS));
            show("Pusher turning forward", String.format(Locale.US, "%.1f deg", Pusher.ENCODER_SIGN * (motor.getCurrentPosition() - start) / ticksPerDeg));
        }
        motor.setPower(0.0);
        sleep(400);
        double forwardDeg = Pusher.ENCODER_SIGN * (motor.getCurrentPosition() - start) / ticksPerDeg;

        // back to the start, whichever way the encoder counts
        timer.reset();
        motor.setPower(-PUSHER_POWER);
        while (motionTick() && timer.seconds() < PUSHER_TIMEOUT_SEC + 1.0
                && Math.abs(motor.getCurrentPosition() - start) > stopEarlyTicks) {
            show("Pusher returning", String.format(Locale.US, "%.1f deg from the start", Pusher.ENCODER_SIGN * (motor.getCurrentPosition() - start) / ticksPerDeg));
        }
        motor.setPower(0.0);
        sleep(400);
        double leftDeg = Pusher.ENCODER_SIGN * (motor.getCurrentPosition() - start) / ticksPerDeg;
        if (aborted) {
            record("Pusher", Verdict.FAIL, String.format(Locale.US, "stopped with the arm %.1f deg from where it started: put it back at HOLD by hand", leftDeg));
            return;
        }

        if (forwardDeg >= PUSHER_MIN_MOVE_DEG) {
            record("Pusher encoder", Verdict.PASS, String.format(Locale.US, "counted +%.1f deg for a forward move (sign correct), peak %.0f ticks/s, %.1f A", forwardDeg, peakVelocity, amps));
        } else if (forwardDeg <= -PUSHER_MIN_MOVE_DEG) {
            record("Pusher encoder", Verdict.FAIL, String.format(Locale.US, "counted %.1f deg for a forward move: it counts the OPPOSITE way. Flip Pusher.ENCODER_SIGN (unless the arm also moved the wrong way: then flip pusherDirection in ChassisProfile)", forwardDeg));
        } else {
            record("Pusher encoder", Verdict.FAIL, String.format(Locale.US, "only %.1f deg counted: no encoder counts (cable or port), or the arm did not move (jam, too little power, wrong name)", forwardDeg));
        }
        if (Math.abs(forwardDeg) < PUSHER_MIN_MOVE_DEG) {
            record("Pusher return", Verdict.FAIL, "no encoder counts, so the arm could not be brought back: if it moved, put it back at HOLD by hand");
        } else if (Math.abs(leftDeg) <= PUSHER_RETURN_TOLERANCE_DEG) {
            record("Pusher return", Verdict.PASS, String.format(Locale.US, "back within %.1f deg of where it started", Math.abs(leftDeg)));
        } else {
            record("Pusher return", Verdict.FAIL, String.format(Locale.US, "ended %.1f deg from where it started: put the arm back at HOLD by hand before running another opmode", leftDeg));
        }
        judge("Pusher direction", "Did the arm move in the PUSHING direction (the way that pushes elements toward the flywheel)?",
                "flip pusherDirection for this chassis in ChassisProfile");
    }

    private void intakeStep() {
        if (!askRun("Intake", String.format(Locale.US, "Pulses the intake IN for %.1f s, then OUT for %.1f s, at %.0f%% power. Keep clear of the roller.", INTAKE_PULSE_SEC, INTAKE_PULSE_SEC, 100 * INTAKE_POWER_SCALE))) return;
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, "intake");
        if (motor == null) {
            record("Intake", Verdict.FAIL, "intake is not configured");
            return;
        }
        motor.setDirection(profile.intakeDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double inAmps = pulse(motor, Intake.IN_POWER * INTAKE_POWER_SCALE, INTAKE_PULSE_SEC, "Intake IN");
        sleep(400);
        double outAmps = aborted ? 0.0 : pulse(motor, Intake.OUT_POWER * INTAKE_POWER_SCALE, INTAKE_PULSE_SEC, "Intake OUT");
        if (aborted) return;
        record("Intake current", Verdict.PASS, String.format(Locale.US, "IN %.1f A, OUT %.1f A (free running, for comparison with a jammed roller)", inAmps, outAmps));
        judge("Intake direction", "Did the roller pull IN first, then push OUT?", "flip intakeDirection for this chassis in ChassisProfile");
    }

    private void rampStep() {
        if (!askRun("Ramp", "Deploys the ramp, waits, then stows it. It may touch the blocks or the floor; keep clear of the ramp.")) return;
        Ramp ramp = new Ramp(hardwareMap);   // commands the stowed positions
        if (!ramp.isConfigured()) {
            record("Ramp", Verdict.SKIP, "the servo positions are not set in ChassisProfile (the ramp does not move)");
            return;
        }
        sleepWithAbort(Ramp.TRAVEL_SEC + 0.3, "Ramp going to STOWED (start)");
        ramp.deploy();
        sleepWithAbort(Ramp.TRAVEL_SEC + 0.3, "Ramp DEPLOYED");
        ramp.stow();
        sleepWithAbort(Ramp.TRAVEL_SEC + 0.3, "Ramp STOWED");
        if (aborted) return;
        judge("Ramp", "Did BOTH servos move together, mirrored, to deployed and back to stowed (and is it stowed now)?",
                "check the servo names depolLeft / depolRight, each servo's cable, and the positions in ChassisProfile");
    }

    private void driveStep(String name) {
        if (!askRun("Drive " + name, String.format(Locale.US, "Spins ONLY %s at %.2f power for %.1f s. The wheel must be off the floor.", name, DRIVE_POWER, DRIVE_PULSE_SEC))) return;
        DcMotor motor = hardwareMap.tryGet(DcMotor.class, name);
        if (motor == null) {
            record("Drive " + name, Verdict.FAIL, name + " is not configured");
            return;
        }
        motor.setDirection(name.startsWith("left") ? profile.leftDirection : profile.rightDirection);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulse(motor, DRIVE_POWER, DRIVE_PULSE_SEC, "Drive " + name);
        if (aborted) return;
        judge("Drive " + name, "Did ONLY " + name + " turn, with the top of the wheel moving toward the FRONT of the robot?",
                "check which motor port the cable is on and the config name; if only the direction is wrong, flip leftDirection / rightDirection in ChassisProfile (that flips both wheels of a side, so recheck the other one)");
    }

    // ---- helpers ----

    /** Runs a motor for a time and returns its current at the end (a hub read, so only once). */
    private double pulse(DcMotor motor, double power, double seconds, String label) {
        ElapsedTime timer = new ElapsedTime();
        motor.setPower(power);
        double amps = 0.0;
        while (motionTick() && timer.seconds() < seconds) {
            show(label, String.format(Locale.US, "%.1f s", timer.seconds()));
        }
        if (motor instanceof DcMotorEx) amps = ((DcMotorEx) motor).getCurrent(CurrentUnit.AMPS);
        motor.setPower(0.0);
        return amps;
    }

    private void sleepWithAbort(double seconds, String label) {
        ElapsedTime timer = new ElapsedTime();
        while (motionTick() && timer.seconds() < seconds) show(label, String.format(Locale.US, "%.1f s", timer.seconds()));
    }

    /** False when the opmode is stopping or Back was pressed (everything should stop at once). */
    private boolean motionTick() {
        if (gamepad1.backWasPressed()) aborted = true;
        return opModeIsActive() && !aborted;
    }

    private void show(String title, String detail) {
        showScreen(title, detail, "Back = stop everything");
    }

    private void showScreen(String title, String text, String keys) {
        telemetry.addLine("=== " + title + " ===");
        telemetry.addLine(text);
        telemetry.addLine(keys);
        telemetry.addLine("--- results so far ---");
        for (String[] r : results) telemetry.addLine(r[1] + "  " + r[0] + ": " + r[2]);
        telemetry.update();
    }

    private Choice ask(String title, String text, String keys) {
        gamepad1.aWasPressed();
        gamepad1.bWasPressed();
        gamepad1.xWasPressed();
        gamepad1.backWasPressed();
        while (opModeIsActive()) {
            showScreen(title, text, keys);
            if (gamepad1.backWasPressed()) {
                aborted = true;
                return Choice.ABORT;
            }
            if (gamepad1.aWasPressed()) return Choice.YES;
            if (gamepad1.bWasPressed()) return Choice.NO;
            if (gamepad1.xWasPressed()) return Choice.SKIP;
            sleep(20);
        }
        aborted = true;
        return Choice.ABORT;
    }

    /** Asks whether to run a moving step. A = run; B or X = skip it. */
    private boolean askRun(String name, String what) {
        Choice choice = ask(name, what + "  (robot on blocks?)", "A = run   X = skip this step   Back = stop everything");
        if (choice == Choice.YES) return true;
        if (choice != Choice.ABORT) record(name, Verdict.SKIP, "skipped");
        return false;
    }

    /** Asks the operator to judge what they saw. */
    private void judge(String name, String question, String fix) {
        Choice choice = ask(name, question, "A = yes, right   B = no, WRONG   X = could not tell   Back = stop everything");
        if (choice == Choice.YES) {
            record(name, Verdict.PASS, "seen to be right");
        } else if (choice == Choice.NO) {
            record(name, Verdict.FAIL, "seen to be WRONG: " + fix);
        } else if (choice == Choice.SKIP) {
            record(name, Verdict.SKIP, "could not tell");
        }
    }

    private void record(String name, Verdict verdict, String detail) {
        results.add(new String[]{name, verdict.name(), detail});
        RunLog.event(verdict + " " + name + ": " + detail);
        RobotLog.ii(TAG, verdict + "  " + name + ": " + detail);
    }

    private void finish() {
        int pass = 0, fail = 0, skip = 0;
        StringBuilder report = new StringBuilder();
        for (String[] r : results) {
            if (r[1].equals("PASS")) pass++;
            else if (r[1].equals("FAIL")) fail++;
            else skip++;
            report.append(r[1]).append("  ").append(r[0]).append(": ").append(r[2]).append('\n');
        }
        String summary = String.format(Locale.US, "PIT CHECK %s: %d passed, %d failed, %d skipped%s",
                fail == 0 && skip == 0 && !aborted ? "ALL PASS" : fail > 0 ? "FAILED" : "INCOMPLETE", pass, fail, skip,
                aborted ? " (stopped early)" : "");
        RunLog.event(summary);
        RobotLog.ii(TAG, "\n" + summary + "\n" + report);
        while (opModeIsActive()) {
            telemetry.addLine("=== " + summary + " ===");
            for (String[] r : results) telemetry.addLine(r[1] + "  " + r[0] + ": " + r[2]);
            telemetry.addLine("Stop the opmode to end. The report is also in the robot log (adb logcat -s PitCheck) and runlogs/pitcheck.log.");
            telemetry.update();
            sleep(250);
        }
    }
}
