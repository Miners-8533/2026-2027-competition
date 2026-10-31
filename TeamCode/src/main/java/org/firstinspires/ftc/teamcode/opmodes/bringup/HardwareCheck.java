package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.drive.ChassisProfile;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

/**
 * Checks the hub configuration against the names the code looks up, and prints where each device is plugged in.
 * READ ONLY: it never commands a motor or servo, so it is safe with the robot on the floor. Runs in INIT and
 * after Start.
 *
 * It reports, in order: whether every required device is present (a MISSING or WRONG TYPE entry is the usual
 * reason a mechanism silently does nothing, because the coordinator skips mechanisms it cannot find); each
 * device's hub and port (copy these into the port map in design/hub-configuration.md); the encoder count of each
 * motor, so turning a part by hand shows which name it is wired to; the Pinpoint's status; the hubs and battery
 * voltage; and any configured device our code does not use (stale entries to delete).
 *
 * Required names: leftFront, leftBack, rightFront, rightBack, intake, fly_motor_enc, gate (motors); depolLeft,
 * depolRight (servos); CS1 to CS4 (REV Color Sensor V3, one per I2C bus); pinpoint (goBILDA Pinpoint).
 */
@TeleOp(name = "Hardware Check", group = "Bring-up")
public class HardwareCheck extends LinearOpMode {

    private static final List<String> DRIVE_MOTORS = Arrays.asList("leftFront", "leftBack", "rightFront", "rightBack");
    private static final List<String> MECHANISM_MOTORS = Arrays.asList("intake", "fly_motor_enc", "gate");
    private static final List<String> SERVOS = Arrays.asList("depolLeft", "depolRight");
    private static final List<String> COLOR_SENSORS = Arrays.asList("CS1", "CS2", "CS3", "CS4");
    private static final String PINPOINT = "pinpoint";

    private static final String LOG_TAG = "HardwareCheck";

    private final ElapsedTime logTimer = new ElapsedTime();
    private String lastLogged = "";
    private int missing;
    private Set<String> configuredNames;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ChassisProfile profile = ChassisProfile.match(hardwareMap);
        Set<String> expected = new HashSet<>();
        expected.addAll(DRIVE_MOTORS);
        expected.addAll(MECHANISM_MOTORS);
        expected.addAll(SERVOS);
        expected.addAll(COLOR_SENSORS);
        expected.add(PINPOINT);

        telemetry.addLine("Hardware Check ready. It only reads; nothing will move.");
        telemetry.update();

        while (!isStopRequested()) {
            missing = 0;
            configuredNames = hardwareMap.getAllNames(HardwareDevice.class);
            List<String[]> lines = new ArrayList<>();

            for (String name : DRIVE_MOTORS) lines.add(checkMotor("Drive", name));
            for (String name : MECHANISM_MOTORS) lines.add(checkMotor("Mechanism", name));
            for (String name : SERVOS) lines.add(checkServo(name));
            for (String name : COLOR_SENSORS) lines.add(checkColorSensor(name));
            lines.add(checkPinpoint());

            StringBuilder report = new StringBuilder();
            report(report, "Chassis profile", profile == null ? "NONE - this hub is not registered" : profile.name);
            report(report, "RESULT", missing == 0 ? "all required devices are present" : missing + " required device(s) MISSING or the wrong type");
            for (String[] line : lines) report(report, line[0], line[1]);

            telemetry.addLine("--- hubs ---");
            Set<String> hubNames = new HashSet<>();
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                hubNames.addAll(hardwareMap.getNamesOf(module));
                report(report, "Hub address " + module.getModuleAddress(), String.format("%s, battery %.2f V",
                        module.isParent() ? "Control Hub" : "Expansion Hub", module.getInputVoltage(VoltageUnit.VOLTS)));
            }

            Set<String> unused = new TreeSet<>();
            for (String name : configuredNames) {
                if (!expected.contains(name) && !hubNames.contains(name)) unused.add(name);
            }
            telemetry.addLine("--- configured but not used by our code (delete stale ones) ---");
            report(report, "Unused configured devices", unused.isEmpty() ? "(none)" : unused.toString());
            telemetry.addLine("Turn a part by hand and watch its encoder to confirm which name it is wired to.");
            telemetry.update();

            // Also written to the robot log (read it over USB with: adb logcat -s HardwareCheck), at most once a second
            // and only when something changed.
            String text = report.toString();
            if (!text.equals(lastLogged) && logTimer.seconds() > 1.0) {
                RobotLog.ii(LOG_TAG, "\n" + text);
                lastLogged = text;
                logTimer.reset();
            }
            sleep(250);
        }
    }

    /** Shows one line on the Driver Station and records it for the log. */
    private void report(StringBuilder report, String key, String value) {
        telemetry.addData(key, value);
        report.append(key).append(": ").append(value).append('\n');
    }

    private String[] checkMotor(String role, String name) {
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
        if (motor == null) return absent(name, DcMotorEx.class);
        return new String[]{name, String.format("OK   %s   [%s]   encoder %d", motor.getConnectionInfo(), role, motor.getCurrentPosition())};
    }

    private String[] checkServo(String name) {
        Servo servo = hardwareMap.tryGet(Servo.class, name);
        if (servo == null) return absent(name, Servo.class);
        return new String[]{name, String.format("OK   %s   [Depollinator]", servo.getConnectionInfo())};
    }

    private String[] checkColorSensor(String name) {
        NormalizedColorSensor sensor = hardwareMap.tryGet(NormalizedColorSensor.class, name);
        if (sensor == null) return absent(name, NormalizedColorSensor.class);
        return new String[]{name, String.format("OK   %s   [element counter]", sensor.getConnectionInfo())};
    }

    private String[] checkPinpoint() {
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT);
        if (pinpoint == null) return absent(PINPOINT, GoBildaPinpointDriver.class);
        pinpoint.update();
        return new String[]{PINPOINT, String.format("OK   %s   status %s, firmware %d, loop %d us (500-1100 is normal)",
                pinpoint.getConnectionInfo(), pinpoint.getDeviceStatus(), pinpoint.getDeviceVersion(), pinpoint.getLoopTime())};
    }

    private String[] absent(String name, Class<?> type) {
        missing++;
        return new String[]{name, configuredNames.contains(name)
                ? "WRONG TYPE: the name exists but is not configured as a " + type.getSimpleName()
                : "MISSING from the hub configuration"};
    }
}
