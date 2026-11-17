package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.subsystems.SlotCounter;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * Color Sensors Test (CS1-CS4) (specs/support-opmodes.md sections 3.1 and 3.2): an older tool that measures the empty
 * baseline and scores the classification thresholds against hand-set labels. The Color Sensor Study replaced it for
 * baselines. It writes no run log; its lines go to the device log.
 *
 * Controls (gamepad 1): A captures the empty baseline; Back clears it (one of the six support opmodes that use Back).
 * B turns the sensor LEDs on / off. X switches between one sensor per loop and all four every loop. D-pad up / down
 * selects a sensor and left / right sets its label; left bumper sets every label to NOTHING. Y starts / stops
 * recording against the labels; right bumper clears the counts.
 *
 * A reading is classified with the element counter's own thresholds, against the captured baseline when there is one
 * and the stored empty readings otherwise.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@TeleOp(name = "Color Sensors Test (CS1-CS4)", group = "2 Bring-up")
public final class ColorSensorsTest extends TeamOpMode {
    private static final String TAG = "ColorSensorsTest";
    private static final int BASELINE_READS = 20;
    private static final SlotCounter.Kind[] LABELS = SlotCounter.Kind.values();

    @Override protected RunLog.Kind logKind() { return null; }
    @Override protected String title() { return "Color Sensors Test (CS1-CS4)"; }

    private static String name(SlotCounter.Kind k) { return k == SlotCounter.Kind.EMPTY ? "NOTHING" : k.toString(); }

    @Override
    protected void body() throws InterruptedException {
        ColorReader cs = ColorReader.create(hardwareMap);
        telemetry.addLine(cs == null ? "CS1 to CS4 are not all in the configuration" : "Color Sensors Test: press Start. Nothing moves.");
        telemetry.update();
        waitForStart();
        if (isStopRequested() || cs == null) return;
        Edge a = new Edge(), back = new Edge(), b = new Edge(), x = new Edge(), y = new Edge(), up = new Edge(), down = new Edge(),
                left = new Edge(), right = new Edge(), lb = new Edge(), rb = new Edge();
        double[][] baseline = new double[4][];          // captured; null = use the stored empty reading
        double[][] latest = new double[4][];
        SlotCounter.Kind[] seen = new SlotCounter.Kind[4];
        int[] label = new int[4];                       // index into LABELS; 0 = EMPTY = NOTHING
        int[] total = new int[4], correct = new int[4];
        boolean led = true, all = false, recording = false;
        int selected = 0, next = 0;
        long loops = 0, t0 = System.nanoTime();
        while (opModeIsActive()) {
            // Read: one sensor per loop, or all four every loop.
            for (int k = 0; k < (all ? 4 : 1); k++) {
                int i = all ? k : next;
                double[] r = cs.read(i);
                latest[i] = r;
                if (r != null) {
                    double[] empty = baseline[i] != null ? baseline[i] : SlotCounter.storedEmpty(i);
                    seen[i] = SlotCounter.presentAgainst(r[0], r[1], r[2], r[3], r[4], empty) ? SlotCounter.classify(i, r[2], r[3], r[4], r[1]) : SlotCounter.Kind.EMPTY;
                    if (recording) {
                        total[i]++;
                        if (seen[i] == LABELS[label[i]]) correct[i]++;
                    }
                }
            }
            next = (next + 1) % 4;
            loops++;

            if (a.pressed(gamepad1.a)) {
                // Capture the empty baseline: the mean of several readings of each sensor. The track must be empty.
                for (int i = 0; i < 4; i++) {
                    ColorReader.Mean m = new ColorReader.Mean();
                    for (int k = 0; k < BASELINE_READS && opModeIsActive(); k++) m.add(cs.read(i));
                    baseline[i] = m.get();
                    RobotLog.ii(TAG, "baseline CS%d: %s (%d readings)", i + 1, ColorReader.text(baseline[i]), m.n);
                }
            }
            if (back.pressed(gamepad1.back)) {
                baseline = new double[4][];
                RobotLog.ii(TAG, "baseline cleared: the stored empty readings are used");
            }
            if (b.pressed(gamepad1.b)) {
                led = !led;
                cs.led(led);
                RobotLog.ii(TAG, "sensor LEDs %s", led ? "on" : "off");
            }
            if (x.pressed(gamepad1.x)) all = !all;
            if (up.pressed(gamepad1.dpad_up)) selected = (selected + 3) % 4;
            if (down.pressed(gamepad1.dpad_down)) selected = (selected + 1) % 4;
            if (right.pressed(gamepad1.dpad_right)) label[selected] = (label[selected] + 1) % LABELS.length;
            if (left.pressed(gamepad1.dpad_left)) label[selected] = (label[selected] + LABELS.length - 1) % LABELS.length;
            if (lb.pressed(gamepad1.left_bumper)) label = new int[4];
            if (rb.pressed(gamepad1.right_bumper)) {
                total = new int[4];
                correct = new int[4];
            }
            if (y.pressed(gamepad1.y)) {
                recording = !recording;
                if (!recording) {
                    for (int i = 0; i < 4; i++) {
                        RobotLog.ii(TAG, "score CS%d labelled %s: %d of %d readings classified as the label (%.0f percent)", i + 1, name(LABELS[label[i]]),
                                correct[i], total[i], total[i] == 0 ? 0.0 : 100.0 * correct[i] / total[i]);
                    }
                }
            }

            double loopMs = (System.nanoTime() - t0) / 1e6 / Math.max(1, loops);
            telemetry.addLine(String.format(Locale.US, "%s, LEDs %s, loop %.0f ms%s", all ? "all four every loop" : "one sensor per loop", led ? "on" : "off", loopMs,
                    recording ? ", RECORDING" : ""));
            for (int i = 0; i < 4; i++) {
                telemetry.addLine(String.format(Locale.US, "%sCS%d sees %s, label %s, %d/%d", i == selected ? "> " : "  ", i + 1,
                        seen[i] == null ? "?" : name(seen[i]), name(LABELS[label[i]]), correct[i], total[i]));
                telemetry.addLine("     " + ColorReader.text(latest[i]) + (baseline[i] != null ? " (own baseline)" : ""));
            }
            telemetry.addLine("A baseline, Back clear it, B LEDs, X one/all, Y record, RB clear counts, LB labels to NOTHING");
            telemetry.update();
        }
    }
}
