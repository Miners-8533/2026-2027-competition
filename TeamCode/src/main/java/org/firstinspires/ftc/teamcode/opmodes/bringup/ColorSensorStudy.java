package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.ElementCounter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Records what the four element sensors (CS1 to CS4) see, so the element count can be understood and improved. It does not decide anything: it
 * logs the raw readings of every sensor about 20 times a second (distance, alpha, the three color shares, the shift from the empty baseline, whether
 * the sensor is lit before the debounce, and a read counter that shows when a value is fresh), next to what the counter reported, the pusher angle,
 * the intake, the drive commands and the robot's motion, and a label for the take: its type and the number of pollen really in the robot (set by hand).
 * The log is analyzed afterwards (tools/logs/sensor_study.py); see design/color-sensor-study.md for the procedure.
 *
 * The sensors are read one every SENSOR_READ_PERIOD_MS (25 ms, so each is read about every 100 ms, the rate at which the sensors refresh; the robot's
 * own counter reads one every 40 ms). The sensors are left at the SDK's default rate and gain. The counter's own logic (debounce and steady-count
 * windows) runs as on the robot, so its reported count can be compared with the truth.
 *
 * Takes (D-pad up / down picks the type, A starts and stops):
 *   STILL    the robot stays put (the baseline: empty track, or 4 pollen with the pusher at HOLD).
 *   SWEEP    automatic, for loading 4 pollen first and checking by eye: about 2 s still, then the pusher turns BACKWARD one full turn slowly with the
 *            intake on OUT (the pollen move past each sensor in turn, and the arm passes each sensor too), then about 2 s still. The sticks are off.
 *            A during the sweep aborts it. The pollen end up on the floor in front of the intake: reload for the next take.
 *   JOSTLE   the robot is driven around by the sticks with a known number of pollen inside (and the intake OFF, IN or OUT as chosen).
 *
 *   D-pad left / right  the number of pollen really in the robot, 0 to 4 (set it BEFORE the take; changing it during one is logged)
 *   X / Y / B           intake IN / OUT / OFF
 *   Left stick / right stick x   drive (robot-oriented), not during a SWEEP
 *   Right bumper        log a NOTE (the time and the take), for something you saw
 *
 * The pusher must be at HOLD when the opmode is initialized (its zero is assumed there), as in the autos.
 */
@Config
@TeleOp(name = "Color Sensor Study", group = "Bring-up")
public class ColorSensorStudy extends LinearOpMode {

    /** One sensor is read this often (ms), cycling CS1 to CS4. */
    public static double SENSOR_READ_PERIOD_MS = 25.0;
    /** Rows in the log. */
    public static long SAMPLE_PERIOD_MS = 50;

    public static double SWEEP_POWER = 0.15;       // backward, open loop (about 120 deg/s empty)
    public static double SWEEP_DEG = 360.0;
    public static double SWEEP_MAX_SEC = 20.0;
    public static double SWEEP_NO_MOVE_SEC = 2.0;  // abort if the arm has not moved 5 deg by then (raise SWEEP_POWER)
    public static double LEAD_SEC = 2.0;
    public static double TAIL_SEC = 2.0;

    public static double DRIVE_SCALE = 0.6;
    public static double TURN_SCALE = 0.5;
    public static double STICK_DEADBAND = 0.05;

    private enum TakeType { STILL, SWEEP, JOSTLE }

    private enum Phase { IDLE, RUN, LEAD_IN, SWEEP, TAIL }

    // read by the log sampler
    private TakeType type = TakeType.STILL;
    private Phase phase = Phase.IDLE;
    private int takeId = 0;
    private int known = 0;
    private boolean recording = false;
    private final ElapsedTime takeClock = new ElapsedTime();
    private final ElapsedTime phaseClock = new ElapsedTime();
    private double cmdFwd, cmdLeft, cmdTurn;
    private Pose2d pose = new Pose2d(0.0, 0.0, 0.0);
    private PoseVelocity2d velocity = null;

    private ElementCounter counter;
    private Pusher pusher;
    private Intake intake;
    private double sweepStartTicks;

    @Override
    public void runOpMode() {
        RunLog.open(RunLog.Kind.BRINGUP, "Color Sensor Study", hardwareMap);
        double savedReadPeriod = ElementCounter.READ_PERIOD_MS;
        try {
            study();
        } catch (Throwable t) {
            RunLog.crash(t);
            throw t;
        } finally {
            ElementCounter.READ_PERIOD_MS = savedReadPeriod;
            RunLog.close();
        }
    }

    private static String columns() {
        StringBuilder sb = new StringBuilder("take,type,phase,known,take_s,intake");
        for (int i = 1; i <= ElementCounter.N; i++) {
            String p = ",cs" + i + "_";
            sb.append(p).append("dist").append(p).append("alpha").append(p).append("r").append(p).append("g").append(p).append("b")
                    .append(p).append("shift").append(p).append("on").append(p).append("n");
        }
        sb.append(",slots,raw_count,count,pusher_deg,pusher_unwrapped_deg,pusher_state,pusher_dps,pusher_amps,intake_amps,x,y,hdg_deg,vx,vy,w_dps,cmd_fwd,cmd_left,cmd_turn");
        return sb.toString();
    }

    private String row() {
        StringBuilder sb = new StringBuilder();
        sb.append(recording ? takeId : 0).append(',').append(type).append(',').append(phase).append(',').append(known).append(',');
        sb.append(String.format(Locale.US, "%.2f", recording ? takeClock.seconds() : 0.0)).append(',').append(intake.getMode());
        for (int i = 0; i < ElementCounter.N; i++) {
            sb.append(String.format(Locale.US, ",%.2f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,%d", counter.lastDistCm(i), counter.lastAlpha(i),
                    counter.lastShare(i, 0), counter.lastShare(i, 1), counter.lastShare(i, 2), counter.colorShift(i),
                    counter.rawPresent(i) ? 1 : 0, counter.readCount(i)));
        }
        double dps = pusher.getVelocityTicksPerSec() * 360.0 / Pusher.TICKS_PER_REV;
        double vx = velocity == null ? 0.0 : velocity.linearVel.x, vy = velocity == null ? 0.0 : velocity.linearVel.y;
        double w = velocity == null ? 0.0 : Math.toDegrees(velocity.angVel);
        sb.append(String.format(Locale.US, ",%s,%d,%d,%.1f,%.1f,%s,%.0f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.0f,%.2f,%.2f,%.2f",
                counter.slotsCompact(), counter.elementsNow(), counter.count(), pusher.getAngleDeg(),
                pusher.getUnwrappedTicks() * 360.0 / Pusher.TICKS_PER_REV, pusher.getState(), dps, pusher.getCurrentAmps(), intake.getCurrentAmps(),
                pose.position.x, pose.position.y, pose.heading.toDegrees(), vx, vy, w, cmdFwd, cmdLeft, cmdTurn));
        return sb.toString();
    }

    private void startTake() {
        takeId++;
        recording = true;
        takeClock.reset();
        phaseClock.reset();
        phase = type == TakeType.SWEEP ? Phase.LEAD_IN : Phase.RUN;
        RunLog.event(String.format(Locale.US, "TAKE %d START: type %s, known pollen %d, intake %s, pusher %.0f deg, sensor read period %.0f ms",
                takeId, type, known, intake.getMode(), pusher.getAngleDeg(), ElementCounter.READ_PERIOD_MS));
    }

    private void endTake(String how) {
        if (type == TakeType.SWEEP) {
            pusher.stopInPlace();
            intake.setMode(Intake.Mode.OFF);
        }
        RunLog.event(String.format(Locale.US, "TAKE %d END (%s): %.1f s, counter said %d, %d sensors lit, pusher %.0f deg", takeId, how, takeClock.seconds(),
                counter.count(), counter.elementsNow(), pusher.getAngleDeg()));
        recording = false;
        phase = Phase.IDLE;
    }

    private static double dead(double v) {
        return Math.abs(v) < STICK_DEADBAND ? 0.0 : v;
    }

    private void study() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElementCounter.READ_PERIOD_MS = SENSOR_READ_PERIOD_MS;

        counter = ElementCounter.tryCreate(hardwareMap);
        if (counter == null) {
            while (!isStopRequested()) {
                telemetry.addLine("The four color sensors CS1 to CS4 are not all configured. Fix the hub configuration.");
                telemetry.update();
            }
            return;
        }
        pusher = new Pusher(hardwareMap, Pusher.Reference.ASSUME_AT_HOLD);
        pusher.setAutoCorrect(false);   // nothing moves the arm except the sweep
        intake = new Intake(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        while (!isStarted() && !isStopRequested()) {
            drive.updatePoseEstimate();
            counter.update();
            pusher.update();
            telemetry.addLine("Color Sensor Study. Pusher at HOLD now (its zero is assumed here). Keep the robot still until Start.");
            addSensorLines();
            telemetry.addLine("A = start / stop a take, D-pad up / down = take type, D-pad left / right = pollen really inside, X / Y / B = intake IN / OUT / OFF.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        RunLog.started();
        RunLog.setSampler(columns(), SAMPLE_PERIOD_MS, this::row);
        RunLog.event(String.format(Locale.US, "SETUP sensor read period %.0f ms (the robot's counter uses 40), log rows every %d ms, sweep power %.2f for %.0f deg",
                ElementCounter.READ_PERIOD_MS, SAMPLE_PERIOD_MS, SWEEP_POWER, SWEEP_DEG));

        while (opModeIsActive()) {
            velocity = drive.updatePoseEstimate();
            pose = drive.getPose();
            counter.update();
            pusher.update();
            intake.update();

            if (gamepad1.dpadLeftWasPressed() && known > 0) {
                known--;
                RunLog.event("KNOWN pollen -> " + known + (recording ? " (during take " + takeId + ")" : ""));
            }
            if (gamepad1.dpadRightWasPressed() && known < ElementCounter.CAPACITY) {
                known++;
                RunLog.event("KNOWN pollen -> " + known + (recording ? " (during take " + takeId + ")" : ""));
            }
            if (!recording) {
                if (gamepad1.dpadUpWasPressed()) type = TakeType.values()[(type.ordinal() + 1) % TakeType.values().length];
                if (gamepad1.dpadDownWasPressed()) type = TakeType.values()[(type.ordinal() + TakeType.values().length - 1) % TakeType.values().length];
            }
            boolean sweeping = recording && type == TakeType.SWEEP && (phase == Phase.SWEEP);
            if (!sweeping) {
                if (gamepad1.xWasPressed()) { intake.setMode(Intake.Mode.IN); RunLog.event("INTAKE IN"); }
                if (gamepad1.yWasPressed()) { intake.setMode(Intake.Mode.OUT); RunLog.event("INTAKE OUT"); }
                if (gamepad1.bWasPressed()) { intake.setMode(Intake.Mode.OFF); RunLog.event("INTAKE OFF"); }
            }
            if (gamepad1.rightBumperWasPressed()) {
                RunLog.event(String.format(Locale.US, "NOTE (take %d, %.1f s, known %d, counter %d, %d lit)", recording ? takeId : 0,
                        recording ? takeClock.seconds() : 0.0, known, counter.count(), counter.elementsNow()));
            }
            if (gamepad1.aWasPressed()) {
                if (!recording) startTake();
                else if (type == TakeType.SWEEP && phase != Phase.TAIL) endTake("ABORTED by the driver");
                else endTake("stopped by the driver");
            }

            // the sweep
            if (recording && type == TakeType.SWEEP) {
                if (phase == Phase.LEAD_IN && phaseClock.seconds() >= LEAD_SEC) {
                    phase = Phase.SWEEP;
                    phaseClock.reset();
                    sweepStartTicks = pusher.getUnwrappedTicks();
                    intake.setMode(Intake.Mode.OUT);
                    pusher.jog(-SWEEP_POWER);
                    RunLog.event(String.format(Locale.US, "TAKE %d SWEEP start: backward at %.2f power, intake OUT", takeId, SWEEP_POWER));
                } else if (phase == Phase.SWEEP) {
                    double travelled = (sweepStartTicks - pusher.getUnwrappedTicks()) * 360.0 / Pusher.TICKS_PER_REV;
                    if (travelled >= SWEEP_DEG) {
                        pusher.stopInPlace();
                        intake.setMode(Intake.Mode.OFF);
                        RunLog.event(String.format(Locale.US, "TAKE %d SWEEP done: %.0f deg in %.1f s", takeId, travelled, phaseClock.seconds()));
                        phase = Phase.TAIL;
                        phaseClock.reset();
                    } else if (phaseClock.seconds() > SWEEP_NO_MOVE_SEC && travelled < 5.0) {
                        endTake("ABORTED: the arm did not move; raise SWEEP_POWER");
                    } else if (phaseClock.seconds() > SWEEP_MAX_SEC) {
                        endTake("ABORTED: the sweep took too long");
                    }
                } else if (phase == Phase.TAIL && phaseClock.seconds() >= TAIL_SEC) {
                    endTake("complete");
                }
            }

            // driving
            if (type == TakeType.SWEEP) {   // the robot stays still for a sweep
                cmdFwd = cmdLeft = cmdTurn = 0.0;
            } else {
                cmdFwd = -dead(gamepad1.left_stick_y) * DRIVE_SCALE;
                cmdLeft = -dead(gamepad1.left_stick_x) * DRIVE_SCALE;
                cmdTurn = -dead(gamepad1.right_stick_x) * TURN_SCALE;
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(cmdFwd, cmdLeft), cmdTurn));

            telemetry.addData("Take", recording ? "RECORDING take " + takeId + "  " + String.format(Locale.US, "%.1f s", takeClock.seconds()) + "  phase " + phase
                    : "not recording (next take " + (takeId + 1) + ")");
            telemetry.addData("Type (D-pad up / down)", type + (type == TakeType.SWEEP ? "   (drive is off; A aborts)" : type == TakeType.JOSTLE ? "   (drive with the sticks)" : ""));
            telemetry.addData("Pollen really inside (D-pad left / right)", known);
            telemetry.addData("Intake (X in / Y out / B off)", intake.getMode());
            telemetry.addData("Pusher", "%.0f deg, %s", pusher.getAngleDeg(), pusher.getState());
            addSensorLines();
            telemetry.addData("Pose", "%.1f, %.1f, %.0f deg", pose.position.x, pose.position.y, pose.heading.toDegrees());
            telemetry.update();
            RunLog.tick();
        }
        drive.stop();
        pusher.stopInPlace();
        intake.setMode(Intake.Mode.OFF);
    }

    private void addSensorLines() {
        for (int i = 0; i < ElementCounter.N; i++) {
            telemetry.addData(ElementCounter.NAMES[i], "%s  dist %.1f cm  alpha %.3f  rgb %.2f / %.2f / %.2f  shift %.3f  reads %d",
                    counter.rawPresent(i) ? "LIT" : "---", counter.lastDistCm(i), counter.lastAlpha(i), counter.lastShare(i, 0), counter.lastShare(i, 1),
                    counter.lastShare(i, 2), counter.colorShift(i), counter.readCount(i));
        }
        telemetry.addData("Counter", "%s   %d lit now, reported %d%s", counter.slotsCompact(), counter.elementsNow(), counter.count(),
                counter.isValid() ? "" : "   (sensors not read recently)");
    }
}
