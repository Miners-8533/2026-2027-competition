package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Warnings;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The match screen (specs/teleop.md section 11). It uses the Driver Station's HTML display mode, the only mode that
 * formats text: about 15 lines of about 36 characters, and nothing may wrap. Only match-critical information.
 * The controllers have no rumble, so every signal to the drivers is on this screen.
 */
final class MatchScreen {
    private MatchScreen() { }

    static final String RED = "#ff4040", YELLOW = "#ffd000", CYAN = "#40e0ff", GREEN = "#40ff40", BLUE = "#5090ff", WHITE = "#ffffff";

    /** What the TeleOp hands over each loop. */
    static final class View {
        Alliance alliance;
        Pose pose;
        boolean backup, pinpointFault, driveHeld, autoLaunchArmed, lockOn, lockAligned, waitingForAim, countOnly;
        String wheelAlert, dashRefusal, parkRefusal, dashActivity, parkActivity;
        double backupIn = -1, reseedIn = -1, aimRear, aimAudience;
        boolean noShotZone;
    }

    private static String color(String c, String text) { return "<font color='" + c + "'>" + text + "</font>"; }
    private static String big(String text) { return "<big><big>" + text + "</big></big>"; }
    private static String small(String text) { return "<small>" + text + "</small>"; }

    private static String cut(String s, int n) { return s.length() <= n ? s : s.substring(0, n); }

    static String build(View v, Mechanisms mech) {
        Warnings w = mech.warnings();
        List<String> red = new ArrayList<>(), yellow = new ArrayList<>(), cyan = new ArrayList<>();
        // The alerts are chosen from the coordinator's warning codes, not their wording.
        if (v.backup) red.add("BACKUP MODE");
        if (v.pinpointFault) red.add("PINPOINT FAULT");
        if (v.wheelAlert != null) red.add(v.wheelAlert);
        if (w.has(Warnings.Code.FLYWHEEL_LOST)) red.add("FLYWHEEL LOST");
        if (w.has(Warnings.Code.PUSHER_STALLED)) red.add("PUSHER STALLED");
        if (w.has(Warnings.Code.PUSHER_NOT_CALIBRATED)) red.add("PUSHER NOT CALIBRATED");
        if (w.has(Warnings.Code.PUSHER_JAMMED)) red.add("PUSHER JAMMED");
        if (v.backupIn >= 0) yellow.add(String.format(Locale.US, "BACKUP IN %.1f s", v.backupIn));
        if (v.reseedIn >= 0) yellow.add(String.format(Locale.US, "RESEED IN %.1f s", v.reseedIn));
        if (v.dashRefusal != null) yellow.add("DASH REFUSED");
        if (v.parkRefusal != null) yellow.add("PARK DASH REFUSED");
        if (w.has(Warnings.Code.ENTRY_NOT_ANSWERING)) yellow.add("CS5 NOT ANSWERING");
        if (w.has(Warnings.Code.ZERO_UNCERTAIN)) yellow.add("PUSHER ZERO?");
        if (w.has(Warnings.Code.OPPONENT_NECTAR)) yellow.add("OPPONENT NECTAR: REJECT");
        if (w.has(Warnings.Code.COUNTER_NOT_READING)) yellow.add("COUNTER NOT READING");
        if (w.has(Warnings.Code.LAUNCH_MAP_EMPTY)) yellow.add("LAUNCH MAP EMPTY");
        if (v.noShotZone) yellow.add("NO SHOT ZONE");
        String notice = mech.notice();
        if (notice != null) yellow.add(cut(notice, 26));
        if (w.has(Warnings.Code.RAMP_NOT_CONFIGURED)) yellow.add("CHECK ROBOT (verbose)");   // a warning with no alert of its own
        // Deliberately not shown: Full (the count line turns green), stow waiting, zero not verified yet, the coordinator's own backup warning.
        if (v.driveHeld) cyan.add("DRIVE HELD");
        if (mech.fullStepText() != null) cyan.add(mech.fullStepText());
        if (v.autoLaunchArmed) cyan.add("AUTO-LAUNCH ARMED");
        if (org.firstinspires.ftc.teamcode.aiming.LaunchGrid.ENABLED) cyan.add("LAUNCH GRID ON");

        StringBuilder sb = new StringBuilder();
        // Alerts: at most three, red first. Blank when all is well.
        int shown = 0;
        for (String s : red) if (shown < 3) { sb.append(color(RED, s)).append("<br>"); shown++; }
        for (String s : yellow) if (shown < 3) { sb.append(color(YELLOW, s)).append("<br>"); shown++; }
        for (String s : cyan) if (shown < 3) { sb.append(color(CYAN, s)).append("<br>"); shown++; }
        for (; shown < 3; shown++) sb.append("<br>");

        sb.append(big(color(v.alliance == Alliance.BLUE ? BLUE : RED, v.alliance.name()))).append("<br>");

        // Flywheel and count. READY means the flywheel is in the launch band. N HELD is the CS5 entry count, never the slot count.
        String fly = mech.flywheelText();
        String flyColor = fly.equals("READY") ? GREEN : fly.equals("NO FLY") ? RED : YELLOW;
        int held = mech.heldCount();
        String count = held < 0 ? color(YELLOW, "? HELD") : color(mech.full() ? GREEN : WHITE, held + " HELD");
        sb.append("<big>").append(color(flyColor, fly)).append("  ").append(count).append("</big><br>");

        // Activity: first match wins.
        String act = v.dashActivity != null ? v.dashActivity : v.parkActivity != null ? v.parkActivity
                : mech.salvoRunning() ? "LAUNCHING" : v.waitingForAim ? "WAITING FOR AIM" : mech.rejecting() ? "REJECTING"
                : v.lockOn ? (v.lockAligned ? "LOCK ALIGNED" : "LOCKING") : null;
        if (act != null) sb.append("<big>").append(color("LOCK ALIGNED".equals(act) ? GREEN : WHITE, act)).append("</big><br>");

        if (v.backup) sb.append(String.format(Locale.US, "FIXED %.0f in", mech.distance())).append("<br>");
        else if (mech.targetCell() != null) sb.append(String.format(Locale.US, "%s %.0f in", mech.targetCell().label(), mech.distance())).append("<br>");
        if (v.pose != null && !v.backup) sb.append(String.format(Locale.US, "%.1f, %.1f, %.0f deg", v.pose.x, v.pose.y, v.pose.headingDeg())).append("<br>");

        StringBuilder sm = new StringBuilder();
        sm.append(mech.launchedTally()).append(" SHOT");
        if (mech.lockState() == org.firstinspires.ftc.teamcode.subsystems.FullAndLock.Lock.LOCKED) sm.append("  PUSHER LOCKED");
        if (v.countOnly) sm.append("  COUNT ONLY");
        if (mech.speedTrim() != 0) sm.append(String.format(Locale.US, "  trim %+.0f", mech.speedTrim()));
        sb.append(small(sm.toString())).append("<br>");
        if (v.aimRear != 0 || v.aimAudience != 0) {
            sb.append(small(String.format(Locale.US, "aim REAR %+.0f  AUD %+.0f deg", v.aimRear, v.aimAudience))).append("<br>");
        }
        return sb.toString();
    }
}
