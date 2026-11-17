package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.SlotCounter;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Pit Check (specs/support-opmodes.md section 1): the one check to run after a repair, a rewiring or a hub swap, and
 * before a match day. The running code does not watch for wiring and sign faults, so this is where they are found.
 *
 * Setup: robot on blocks, wheels clear, ramp free to move, track empty. Nothing moves until a button is pressed for
 * that step; every move is short and slow. It drives the raw hardware, not the subsystem classes, so it does not
 * disturb the pusher's saved zero. It does not check CS5 or the flywheel position servo.
 *
 * Controls (gamepad 1): A runs a step, or confirms "what I saw was right"; B says "wrong"; X skips; Back stops everything and ends
 * (one of the six support opmodes that use Back, which the match opmodes never do).
 * Hidden: remove the @Disabled line to bring it back.
 */
@Config
@Disabled
@TeleOp(name = "Pit Check (robot on blocks)", group = "2 Bring-up")
public final class PitCheck extends TeamOpMode {
    public static double MIN_BATTERY_V = 12.0;
    public static double FLYWHEEL_POWER = 0.3, FLYWHEEL_MIN_SPEED = 300;
    public static double PUSHER_POWER = 0.2, PUSHER_MOVE_DEG = 60, PUSHER_RETURN_TOL_DEG = 8;
    public static double WHEEL_POWER = 0.2, MOVE_S = 1.0;
    public static double CS_SHIFT_LIMIT = 0.03;

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.PITCHECK; }
    @Override protected String title() { return "Pit Check"; }

    private final List<String> results = new ArrayList<>();
    private int pass, fail, skipped;
    private final Edge a = new Edge(), b = new Edge(), x = new Edge();

    private void record(String step, String verdict, String detail) {
        if (verdict.equals("PASS")) pass++;
        else if (verdict.equals("FAIL")) fail++;
        else skipped++;
        String line = step + ": " + verdict + (detail == null || detail.isEmpty() ? "" : " - " + detail);
        results.add(line);
        RunLog.event("PITCHECK " + line);
    }

    private void show(String... now) {
        for (String s : now) telemetry.addLine(s);
        telemetry.addLine("----");
        for (int i = Math.max(0, results.size() - 8); i < results.size(); i++) telemetry.addLine(results.get(i));
        telemetry.update();
    }

    /** Waits for a button: 'A', 'B', 'X', or 'Y' for Back (stop everything and end). */
    private char waitButton(String... prompt) {
        while (opModeIsActive()) {
            if (gamepad1.back) return 'Y';
            if (a.pressed(gamepad1.a)) return 'A';
            if (b.pressed(gamepad1.b)) return 'B';
            if (x.pressed(gamepad1.x)) return 'X';
            show(prompt);
            idle();
        }
        return 'Y';
    }

    /** Runs a motor at a power for a time; Back stops at once. Returns false if stopped. */
    private boolean pulse(DcMotorEx m, double power, double seconds, String label) {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        m.setPower(power);
        boolean ok = true;
        while (opModeIsActive() && System.nanoTime() < end) {
            if (gamepad1.back) {
                ok = false;
                break;
            }
            show(label, String.format(Locale.US, "current %.2f A", m.getCurrent(CurrentUnit.AMPS)));
            idle();
        }
        m.setPower(0);
        return ok;
    }

    private void byEye(String step, String question, String fix) {
        char c = waitButton(step, question, "A = right, B = wrong, X = skip");
        record(step, c == 'A' ? "PASS" : c == 'B' ? "FAIL" : "SKIPPED", c == 'B' ? fix : "");
    }

    @Override
    protected void body() throws InterruptedException {
        profile = ChassisProfile.get();
        telemetry.addLine("Pit Check: robot ON BLOCKS, wheels clear, ramp free, track empty.");
        telemetry.addLine("A = run / right, B = wrong, X = skip, Back = stop everything and end.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        markStarted();

        // ---- Configuration: automatic ----
        String[] motors = {ChassisProfile.LEFT_FRONT, ChassisProfile.LEFT_BACK, ChassisProfile.RIGHT_FRONT, ChassisProfile.RIGHT_BACK,
                ChassisProfile.PUSHER, ChassisProfile.FLYWHEEL, ChassisProfile.INTAKE};
        List<String> missing = new ArrayList<>(), used = new ArrayList<>();
        StringBuilder ports = new StringBuilder();
        for (String n : motors) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, n);
            used.add(n);
            if (m == null) missing.add(n + " (motor)");
            else ports.append(String.format(Locale.US, " %s=%s motor %d;", n, m.getController().getConnectionInfo(), m.getPortNumber()));
        }
        for (String n : new String[] {ChassisProfile.RAMP_LEFT, ChassisProfile.RAMP_RIGHT}) {
            Servo s = hardwareMap.tryGet(Servo.class, n);
            used.add(n);
            if (s == null) missing.add(n + " (servo)");
            else ports.append(String.format(Locale.US, " %s=%s servo %d;", n, s.getController().getConnectionInfo(), s.getPortNumber()));
        }
        used.add(ChassisProfile.PINPOINT);
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, ChassisProfile.PINPOINT);
        if (pinpoint == null) missing.add(ChassisProfile.PINPOINT + " (goBILDA Pinpoint)");
        RevColorSensorV3[] cs = new RevColorSensorV3[4];
        for (int i = 0; i < 4; i++) {
            cs[i] = hardwareMap.tryGet(RevColorSensorV3.class, ChassisProfile.SLOT_SENSORS[i]);
            used.add(ChassisProfile.SLOT_SENSORS[i]);
            if (cs[i] == null) missing.add(ChassisProfile.SLOT_SENSORS[i] + " (REV Color Sensor V3)");
        }
        used.addAll(Arrays.asList(ChassisProfile.ENTRY_SENSOR, ChassisProfile.FLY_POS));
        boolean stale = hardwareMap.tryGet(DcMotor.class, ChassisProfile.STALE_FLYWHEEL) != null;
        List<String> unused = new ArrayList<>();
        for (HardwareDevice d : hardwareMap) {
            if (d instanceof LynxModule) continue;
            Set<String> names = hardwareMap.getNamesOf(d);
            boolean isUsed = false;
            for (String n : names) isUsed |= used.contains(n);
            if (!isUsed && !names.isEmpty()) unused.add(names.iterator().next());
        }
        RunLog.event("PITCHECK ports:" + ports);
        RunLog.event("PITCHECK configured but not used by the code: " + unused);
        String cfg = (missing.isEmpty() ? "" : "missing " + missing + ": fix the name or type in the hub configuration. ")
                + (stale ? "'" + ChassisProfile.STALE_FLYWHEEL + "' is configured but the motor was removed: delete it from the configuration." : "");
        record("Configuration", cfg.isEmpty() ? "PASS" : "FAIL", cfg);

        // ---- Battery: automatic ----
        StringBuilder volts = new StringBuilder();
        boolean low = false;
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            double v = hub.getInputVoltage(VoltageUnit.VOLTS);
            volts.append(String.format(Locale.US, " %.2f V", v));
            low |= v < MIN_BATTERY_V;
        }
        record("Battery", low ? "FAIL" : "PASS", volts + (low ? ": change the battery (at least " + MIN_BATTERY_V + " V at rest)" : ""));

        // ---- Pinpoint: automatic ----
        if (pinpoint == null) {
            record("Pinpoint", "FAIL", "not configured");
        } else {
            pinpoint.update();
            int loop = pinpoint.getLoopTime();
            boolean ok = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY && loop > 300 && loop < 1100;
            record("Pinpoint", ok ? "PASS" : "FAIL", "status " + pinpoint.getDeviceStatus() + ", device loop " + loop + " us"
                    + (ok ? "" : ": check the pod cables and the I2C cable (Control Hub bus 1)"));
        }

        // ---- CS1 to CS4: automatic ----
        for (int i = 0; i < 4; i++) {
            if (cs[i] == null) {
                record("CS" + (i + 1), "FAIL", "not configured");
                continue;
            }
            try {
                NormalizedRGBA c = cs[i].getNormalizedColors();
                double sum = c.red + c.green + c.blue, cm = cs[i].getDistance(DistanceUnit.CM);
                double shift = Math.sqrt(Math.pow(c.red / sum - SlotCounter.EMPTY_RED[i], 2) + Math.pow(c.green / sum - SlotCounter.EMPTY_GREEN[i], 2)
                        + Math.pow(c.blue / sum - SlotCounter.EMPTY_BLUE[i], 2));
                boolean ok = sum > 0 && shift <= CS_SHIFT_LIMIT;
                record("CS" + (i + 1), ok ? "PASS" : "FAIL", String.format(Locale.US, "alpha %.3f, %.2f cm, shares %.3f/%.3f/%.3f, shift %.3f%s", c.alpha, cm,
                        c.red / sum, c.green / sum, c.blue / sum, shift, ok ? "" : ": does not match its stored empty reading (is the track empty? re-measure the baselines)"));
            } catch (Throwable e) {
                record("CS" + (i + 1), "FAIL", "does not answer: check its cable and bus");
            }
        }

        // ---- Flywheel ----
        DcMotorEx fly = hardwareMap.tryGet(DcMotorEx.class, ChassisProfile.FLYWHEEL);
        if (fly != null && waitButton("Flywheel: spins at low power", "A = run, X = skip") == 'A') {
            fly.setDirection(profile.flywheelReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fly.setPower(FLYWHEEL_POWER);
            double best = 0;
            long end = System.nanoTime() + (long) 2e9;
            while (opModeIsActive() && System.nanoTime() < end && !gamepad1.back) {
                best = Math.max(best, Flywheel.ENCODER_SIGN * fly.getVelocity());
                show("Flywheel spinning", String.format(Locale.US, "%.0f ticks/s", best));
                idle();
            }
            fly.setPower(0);
            boolean ok = best >= FLYWHEEL_MIN_SPEED;
            record("Flywheel encoder", ok ? "PASS" : "FAIL", String.format(Locale.US, "%.0f ticks/s at power %.2f%s", best, FLYWHEEL_POWER,
                    ok ? "" : ": the encoder reads the wrong way or too slow; check the encoder cable (Expansion Hub motor 1) and the motor direction"));
            byEye("Flywheel direction", "Did it turn the launching way?", "reverse the flywheel direction in the chassis profile, or swap the motor leads");
        } else {
            record("Flywheel", "SKIPPED", fly == null ? "not configured" : "");
        }

        // ---- Pusher ----
        DcMotorEx gate = hardwareMap.tryGet(DcMotorEx.class, ChassisProfile.PUSHER);
        if (gate != null && waitButton("Pusher: turns about 60 deg forward and back", "A = run, X = skip") == 'A') {
            gate.setDirection(profile.pusherReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            gate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            gate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double start = gate.getCurrentPosition(), perDeg = Pusher.TICKS_PER_REV / 360.0;
            long end = System.nanoTime() + (long) 3e9;
            gate.setPower(PUSHER_POWER);
            while (opModeIsActive() && System.nanoTime() < end && !gamepad1.back
                    && Math.abs(gate.getCurrentPosition() - start) < PUSHER_MOVE_DEG * perDeg) idle();
            gate.setPower(0);
            sleep(300);
            double movedDeg = Pusher.ENCODER_SIGN * (gate.getCurrentPosition() - start) / perDeg;
            end = System.nanoTime() + (long) 3e9;
            gate.setPower(-PUSHER_POWER);
            while (opModeIsActive() && System.nanoTime() < end && !gamepad1.back
                    && Pusher.ENCODER_SIGN * (gate.getCurrentPosition() - start) / perDeg > 3) idle();
            gate.setPower(0);
            sleep(300);
            double backDeg = Pusher.ENCODER_SIGN * (gate.getCurrentPosition() - start) / perDeg;
            boolean signOk = movedDeg > PUSHER_MOVE_DEG / 2, returned = Math.abs(backDeg) <= PUSHER_RETURN_TOL_DEG;
            record("Pusher encoder sign", signOk ? "PASS" : "FAIL", String.format(Locale.US, "moved %.0f deg%s", movedDeg,
                    signOk ? "" : ": the encoder counts the wrong way or not at all; check the encoder cable (Expansion Hub motor 0) and the motor direction"));
            record("Pusher return", returned ? "PASS" : "FAIL", String.format(Locale.US, "%.0f deg from where it started%s", backDeg,
                    returned ? "" : ": it could not return; put the arm back at HOLD by hand"));
            byEye("Pusher direction", "Did it first turn the pushing way (toward the flywheel)?", "reverse the pusher direction in the chassis profile");
        } else {
            record("Pusher", "SKIPPED", gate == null ? "not configured" : "");
        }

        // ---- Intake ----
        DcMotorEx in = hardwareMap.tryGet(DcMotorEx.class, ChassisProfile.INTAKE);
        if (in != null && waitButton("Intake: pulses in, then out", "A = run, X = skip") == 'A') {
            in.setDirection(profile.intakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            pulse(in, 1.0, MOVE_S, "Intake IN");
            pulse(in, -0.5, MOVE_S, "Intake OUT");
            byEye("Intake", "Did it pull in first, then push out?", "reverse the intake direction in the chassis profile");
        } else {
            record("Intake", "SKIPPED", in == null ? "not configured" : "");
        }

        // ---- Ramp ----
        Servo left = hardwareMap.tryGet(Servo.class, ChassisProfile.RAMP_LEFT), right = hardwareMap.tryGet(Servo.class, ChassisProfile.RAMP_RIGHT);
        if (left != null && right != null && profile.rampSet() && waitButton("Ramp: deploys, then stows", "A = run, X = skip") == 'A') {
            left.setPosition(profile.rampDeployLeft);
            right.setPosition(profile.rampDeployRight);
            sleep(1200);
            left.setPosition(profile.rampStowLeft);
            right.setPosition(profile.rampStowRight);
            sleep(1200);
            byEye("Ramp", "Did it deploy and then stow, both sides together?", "check the servo ports (left: Control Hub servo 5, right: servo 4) and the positions in the chassis profile");
        } else {
            record("Ramp", "SKIPPED", left == null || right == null ? "not configured" : "");
        }

        // ---- Each drive wheel, one at a time ----
        String[] wheelNames = {ChassisProfile.LEFT_FRONT, ChassisProfile.LEFT_BACK, ChassisProfile.RIGHT_BACK, ChassisProfile.RIGHT_FRONT};
        String[] wheelLabels = {"left front", "left back", "right back", "right front"};
        for (int i = 0; i < 4; i++) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, wheelNames[i]);
            if (m == null) {
                record("Wheel " + wheelLabels[i], "FAIL", "not configured");
                continue;
            }
            if (waitButton("Drive wheel: " + wheelLabels[i], "A = run, X = skip") != 'A') {
                record("Wheel " + wheelLabels[i], "SKIPPED", "");
                continue;
            }
            boolean leftSide = i < 2;
            m.setDirection((leftSide ? profile.leftReversed : profile.rightReversed) ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            pulse(m, WHEEL_POWER, MOVE_S, "Wheel " + wheelLabels[i]);
            byEye("Wheel " + wheelLabels[i], "Did only the " + wheelLabels[i] + " wheel turn, its top toward the front?",
                    "wrong wheel: swap the motor ports; wrong way: fix the drive direction in the chassis profile");
        }

        // A skipped step is not a pass.
        String verdict = fail > 0 ? "FAILED" : skipped > 0 ? "INCOMPLETE" : "ALL PASS";
        RunLog.eventf("PITCHECK RESULT %s: %d passed, %d failed, %d skipped", verdict, pass, fail, skipped);
        while (opModeIsActive()) {
            telemetry.addLine(String.format(Locale.US, "%s: %d passed, %d failed, %d skipped", verdict, pass, fail, skipped));
            for (String r : results) if (!r.contains(": PASS")) telemetry.addLine(r);
            telemetry.update();
            idle();
        }
    }
}
