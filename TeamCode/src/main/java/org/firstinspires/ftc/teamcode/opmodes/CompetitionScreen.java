package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The competition TeleOp's Driver Station screen (design/telemetry-fonts.md, 2026-11-06): a few big, colored lines for both drivers, written in the HTML display format (the only format that
 * formats, measured from the photos of the Telemetry Font Test). From the top: the alerts (only when something is wrong), the alliance as the largest text, the flywheel state and the
 * element count, what the robot is doing, the target cell and its distance, the pose, and two small conditional lines. Everything fits the roughly 15 lines of about 36 characters the screen
 * shows, and no line should wrap. The tags used are only the ones the photos showed working: big, small, bold, font color, br-free lines. No ramp and no voltage (both are visible elsewhere).
 * The old detailed screen stays behind TeleOpCompetition.TELEMETRY_VERBOSE.
 *
 * {@link State} holds plain facts so the Telemetry Font Test can draw sample screens without a robot.
 */
public final class CompetitionScreen {

    static final int GREEN = 0, YELLOW = 1, RED = 2, CYAN = 3;

    private CompetitionScreen() {}

    /** What the screen shows. Filled by TeleOpCompetition (and by {@link #fromMechanisms} for the robot's own facts). */
    public static final class State {
        boolean blue = true;
        int count = -1;               // -1: the counter is not reading
        boolean full = false;
        String flywheel = "READY";
        int flywheelColor = GREEN;
        String activity = "";         // "" when the driver drives by hand
        int activityColor = CYAN;
        String target = "";           // for example "REAR 36 in"
        double x, y, headingDeg;
        boolean pusherLocked = false;
        double speedTrim = 0.0;
        final List<int[]> alertColors = new ArrayList<>();   // one {color} per alert, in the same order as alertTexts
        final List<String> alertTexts = new ArrayList<>();

        void alert(int color, String text) {
            alertColors.add(new int[]{color});
            alertTexts.add(text);
        }
    }

    private static String hex(int color) {
        switch (color) {
            case GREEN: return "#00cc00";
            case YELLOW: return "#ffff00";
            case RED: return "#ff0000";
            default: return "#00ffff";
        }
    }

    private static String colored(int color, String text) {
        return "<font color=\"" + hex(color) + "\">" + text + "</font>";
    }

    /** Draws the screen: call telemetry.update() afterwards. The display format must be HTML. */
    public static void render(Telemetry t, State s) {
        // alerts first, red before yellow, at most three lines
        int shown = 0;
        for (int pass = 0; pass < 2 && shown < 3; pass++) {
            for (int i = 0; i < s.alertTexts.size() && shown < 3; i++) {
                int c = s.alertColors.get(i)[0];
                if ((pass == 0) != (c == RED)) continue;
                t.addLine("<big><b>" + colored(c, s.alertTexts.get(i)) + "</b></big>");
                shown++;
            }
        }
        // the alliance, the largest text on the screen
        t.addLine("<big><big><big><b><font color=\"" + (s.blue ? "#3399ff" : "#ff3333") + "\">" + (s.blue ? "BLUE" : "RED") + "</font></b></big></big></big>");
        // the flywheel and the count
        String count = s.count < 0 ? colored(YELLOW, "? HELD") : (s.full ? colored(GREEN, s.count + " HELD") : s.count + " HELD");
        t.addLine("<big><big><b>" + colored(s.flywheelColor, s.flywheel) + "</b></big></big>  <big><b>" + count + "</b></big>");
        // what the robot is doing
        if (!s.activity.isEmpty()) t.addLine("<big><b>" + colored(s.activityColor, s.activity) + "</b></big>");
        // the target cell and its distance
        if (!s.target.isEmpty()) t.addLine("<big><b>" + s.target + "</b></big>");
        // the pose
        t.addLine(String.format(Locale.US, "%.1f, %.1f, %.0f deg", s.x, s.y, s.headingDeg));
        // small and conditional
        if (s.pusherLocked) t.addLine("<small>" + colored(YELLOW, "PUSHER LOCKED") + "</small>");
        if (Math.abs(s.speedTrim) > 0.5) t.addLine(String.format(Locale.US, "<small>speed trim %+.0f</small>", s.speedTrim));
    }

    /** The robot's own facts: the flywheel, the count, the target cell and the warnings that matter. */
    static void fromMechanisms(State s, Mechanisms m, boolean backupMode, double backupFixedDistanceIn) {
        if (m.counter != null && m.counter.isValid()) {
            s.count = m.counter.count();
        } else {
            s.count = -1;
        }
        s.full = m.isFull();

        if (m.flywheel == null) {
            s.flywheel = "NO FLY";
            s.flywheelColor = RED;
        } else if (m.flywheel.isInLaunchBand()) {
            s.flywheel = "READY";
            s.flywheelColor = GREEN;
        } else if (m.flywheel.launchBandState().startsWith("BELOW")) {
            s.flywheel = "SPIN UP";
            s.flywheelColor = YELLOW;
        } else {
            s.flywheel = "SLOWING";
            s.flywheelColor = YELLOW;
        }

        HiveTarget cell = m.getOpenCell();
        String name = cell.name().endsWith("REAR") ? "REAR" : "AUDIENCE";
        s.target = backupMode ? String.format(Locale.US, "FIXED %.0f in", backupFixedDistanceIn) : String.format(Locale.US, "%s %.0f in", name, m.getLastDistance());
        s.pusherLocked = m.isPusherLocked();
        s.speedTrim = m.getSpeedTrim();

        for (String w : m.getWarnings()) {
            if (w.startsWith("FULL") || w.startsWith("Stow waiting") || w.startsWith("Pusher zero not verified") || w.startsWith("BACKUP MODE")) continue;   // shown elsewhere or not for a match
            if (w.startsWith("FLYWHEEL ENCODER LOST")) s.alert(RED, "FLYWHEEL LOST");
            else if (w.startsWith("FLYWHEEL OPEN LOOP")) s.alert(RED, "FLY OPEN LOOP");
            else if (w.startsWith("Pusher STALLED")) s.alert(RED, "PUSHER STALLED");
            else if (w.startsWith("Pusher not calibrated")) s.alert(RED, "PUSHER NOT CALIBRATED");
            else if (w.startsWith("Pusher JAMMED")) s.alert(RED, "PUSHER JAMMED");
            else if (w.startsWith("PUSHER ZERO UNCERTAIN")) s.alert(YELLOW, "PUSHER ZERO?");
            else if (w.startsWith("OPPONENT NECTAR")) s.alert(YELLOW, "OPPONENT NECTAR: REJECT");
            else if (w.startsWith("ELEMENT COUNTER NOT READING")) s.alert(YELLOW, "COUNTER NOT READING");
            else if (w.startsWith("COUNTER OVERRIDE")) s.alert(YELLOW, "COUNTER OVERRIDE");
            else if (w.startsWith("Launch map empty")) s.alert(YELLOW, "LAUNCH MAP EMPTY");
            else if (w.startsWith("Ramp positions not measured")) continue;   // the ramp is not on this screen
            else s.alert(YELLOW, "CHECK ROBOT (verbose)");
        }
        String notice = m.getNotice();
        if (!notice.isEmpty()) s.alert(YELLOW, notice.length() > 26 ? notice.substring(0, 26) : notice);
    }

    // ---- sample screens for the Telemetry Font Test ----

    /** 0: normal driving; 1: a dash in progress with a warning; 2: faults. */
    public static State sample(int variant) {
        State s = new State();
        s.blue = variant != 2;
        s.x = 12.3;
        s.y = -45.6;
        s.headingDeg = 90;
        s.target = "REAR 36 in";
        switch (variant) {
            case 0:
                s.count = 4;
                s.full = true;
                s.flywheel = "READY";
                s.flywheelColor = GREEN;
                s.activity = "LOCK ALIGNED";
                s.activityColor = GREEN;
                break;
            case 1:
                s.count = 2;
                s.flywheel = "SPIN UP";
                s.flywheelColor = YELLOW;
                s.activity = "DASH: AUDIENCE";
                s.target = "AUDIENCE 34 in";
                s.pusherLocked = true;
                s.speedTrim = 10;
                s.alert(YELLOW, "OPPONENT NECTAR: REJECT");
                break;
            default:
                s.count = -1;
                s.flywheel = "SLOWING";
                s.flywheelColor = YELLOW;
                s.target = "FIXED 42 in";
                s.alert(RED, "BACKUP MODE");
                s.alert(RED, "FLYWHEEL LOST");
                s.alert(YELLOW, "COUNTER NOT READING");
                s.alert(YELLOW, "PUSHER ZERO?");
                break;
        }
        return s;
    }
}
