package org.firstinspires.ftc.teamcode.opmodes.bringup;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionScreen;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Finds out what the Driver Station can show: larger and smaller text, headings, colors, bold, monospace. It uses the Driver Station's HTML display format
 * (Telemetry.DisplayFormat.HTML, which takes a subset of HTML tags), and can switch to the MONOSPACE and CLASSIC formats to compare. It drives and moves
 * nothing and needs no hardware. Pages of test patterns to photograph; each page names itself and says which tag it is testing, so a picture shows what
 * worked and what was ignored (an ignored tag shows its text at the normal size, a tag the screen does not understand may show as plain text).
 *
 * Gamepad 1 (press Start + A on it first, as always): D-pad right / left = next / previous page, A = change the display format (HTML, MONOSPACE, CLASSIC).
 *
 * Pages: 1 sizes with big and small, 2 headings h1 to h6, 3 sizes with font size (attribute and style), 4 text styles and fonts, 5 colors, 6 a mock-up of a
 * status display, 7 how many characters fit across at each size, 8 how many lines fit (normal size), 9 how many lines fit (large).
 *
 * Reading the result: the tags that change the size of the text on the Driver Station are the ones to use in the real TeleOp's key lines (for example the
 * pusher lock state, backup mode, the Hive Dash result) so they can be read at a glance.
 */
@TeleOp(name = "Telemetry Font Test", group = "Bring-up")
public class TelemetryFontTest extends LinearOpMode {

    private static final int PAGES = 12;

    private Telemetry.DisplayFormat format = Telemetry.DisplayFormat.HTML;
    private int page = 0;

    private boolean html() {
        return format == Telemetry.DisplayFormat.HTML;
    }

    /** A line; in HTML the markup is used as written, in the other formats the tags are removed so only the text is left. */
    private void line(String htmlText) {
        telemetry.addLine(html() ? htmlText : htmlText.replaceAll("<[^>]*>", "").replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&"));
    }

    /** A line that shows raw markup text on purpose (the tag names), identical in every format. */
    private void raw(String text) {
        telemetry.addLine(text);
    }

    private static String sizes(String label, int n, boolean big) {
        StringBuilder open = new StringBuilder(), close = new StringBuilder();
        for (int i = 0; i < n; i++) {
            open.append(big ? "<big>" : "<small>");
            close.append(big ? "</big>" : "</small>");
        }
        return open + label + close;
    }

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(format);
        while (!isStarted() && !isStopRequested()) {
            telemetry.setDisplayFormat(format);
            line("<big><b>Telemetry Font Test</b></big>");
            telemetry.addLine("Press Start. Then D-pad right / left = page, A = display format.");
            line("This line is in " + format + " format.");
            telemetry.update();
        }
        while (opModeIsActive()) {
            Gamepad g = gamepad1;
            if (g.dpadRightWasPressed()) page = (page + 1) % PAGES;
            if (g.dpadLeftWasPressed()) page = (page + PAGES - 1) % PAGES;
            if (g.aWasPressed()) {
                Telemetry.DisplayFormat[] all = Telemetry.DisplayFormat.values();
                format = all[(format.ordinal() + 1) % all.length];
            }
            telemetry.setDisplayFormat(format);

            line("<b>Page " + (page + 1) + " of " + PAGES + "</b>  format <b>" + format + "</b>  (D-pad = page, A = format)");
            switch (page) {
                case 0: sizesPage(); break;
                case 1: headingsPage(); break;
                case 2: fontSizePage(); break;
                case 3: stylesPage(); break;
                case 4: colorsPage(); break;
                case 5: mockupPage(); break;
                case 6: widthPage(); break;
                case 7: linesPage(false); break;
                case 8: linesPage(true); break;
                case 9: competitionPage(0); break;
                case 10: competitionPage(1); break;
                default: competitionPage(2); break;
            }
            if (gamepad1.getGamepadId() == Gamepad.ID_UNASSOCIATED) telemetry.addLine("Gamepad 1 is not assigned: press Start + A on it.");
            telemetry.update();
            sleep(50);
        }
    }

    // 1: the big and small tags, nested
    private void sizesPage() {
        raw("PAGE 1: big and small tags (each nesting is a step)");
        line("normal text, no tag");
        line(sizes("big x1", 1, true));
        line(sizes("big x2", 2, true));
        line(sizes("big x3", 3, true));
        line(sizes("big x4", 4, true));
        line(sizes("big x5", 5, true));
        line(sizes("small x1", 1, false));
        line(sizes("small x2", 2, false));
        line("normal again");
    }

    // 2: headings
    private void headingsPage() {
        raw("PAGE 2: heading tags h1 (largest) to h6");
        line("<h1>h1 heading</h1>");
        line("<h2>h2 heading</h2>");
        line("<h3>h3 heading</h3>");
        line("<h4>h4 heading</h4>");
        line("<h5>h5 heading</h5>");
        line("<h6>h6 heading</h6>");
        line("normal text");
    }

    // 3: font size attribute and style
    private void fontSizePage() {
        raw("PAGE 3: font size (attribute and style); an ignored one stays normal");
        for (int s = 1; s <= 7; s++) line("<font size=\"" + s + "\">font size=" + s + "</font>");
        line("<span style=\"font-size:24px\">style font-size:24px</span>");
        line("<span style=\"font-size:36pt\">style font-size:36pt</span>");
        line("<span style=\"font-size:2em\">style font-size:2em</span>");
        line("<span style=\"font-size:200%\">style font-size:200%</span>");
        line("<span style=\"font-size:xx-large\">style font-size:xx-large</span>");
    }

    // 4: styles and fonts
    private void stylesPage() {
        raw("PAGE 4: styles and fonts");
        line("<b>bold</b>  <i>italic</i>  <u>underline</u>  <s>strike</s>  <b><i><u>all three</u></i></b>");
        line("H<sub>2</sub>O  and  x<sup>2</sup>  (sub and sup)");
        line("<tt>tt: 0123 OIl1 monospace</tt>");
        line("<font face=\"monospace\">face monospace: 0123 OIl1</font>");
        line("<font face=\"serif\">face serif: The quick brown fox</font>");
        line("<font face=\"sans-serif\">face sans-serif: The quick brown fox</font>");
        line("<font face=\"sans-serif-condensed\">face condensed: The quick brown fox</font>");
        line("<big><b>big and bold</b></big>   <small>small and italic</small>");
        line("line one<br>line two after a br<br>line three");
        line("a &amp; b  &lt;tag&gt;  &nbsp;&nbsp;&nbsp;3 nbsp");
    }

    // 5: colors
    private void colorsPage() {
        raw("PAGE 5: colors");
        line("<font color=\"#ff0000\">red</font> <font color=\"#00cc00\">green</font> <font color=\"#3399ff\">blue</font> <font color=\"#ffff00\">yellow</font> <font color=\"#00ffff\">cyan</font> <font color=\"#ff00ff\">magenta</font>");
        line("<font color=\"red\">name red</font> <font color=\"green\">name green</font> <font color=\"yellow\">name yellow</font> <font color=\"gray\">name gray</font>");
        line("<span style=\"color:#ff8800\">style color orange</span>");
        line("<span style=\"background-color:#ffff00;color:#000000\"> black on yellow </span>");
        line("<span style=\"background-color:#ff0000;color:#ffffff\"> white on red </span>");
        line("<span style=\"background-color:#00aa00;color:#ffffff\"> white on green </span>");
        line("<span style=\"background-color:#0000ff;color:#ffffff\"> white on blue </span>");
        line("<big><big><font color=\"#ff0000\"><b>BIG BOLD RED</b></font></big></big>");
        line("<big><big><font color=\"#00cc00\"><b>BIG BOLD GREEN</b></font></big></big>");
    }

    // 6: a mock-up of what a real status display could look like
    private void mockupPage() {
        raw("PAGE 6: a mock-up of a status display");
        line("<big><big><b><font color=\"#00cc00\">READY</font></b></big></big>  <big><b>1350</b></big> <small>ticks/s</small>");
        line("<big><b>COUNT 4</b></big>  <font color=\"#ffff00\">LOCKED 92 deg</font>");
        line("<span style=\"background-color:#ff0000;color:#ffffff\"><big><b> BACKUP MODE </b></big></span>");
        line("<big>HIVE DASH <font color=\"#00ffff\">arrived, aligned</font></big>");
        line("<font color=\"#ff8800\"><b>WARNING</b> pusher zero not verified</font>");
        line("<small>pose 12.3, -45.6, 90 deg   loop 21 ms   12.4 V</small>");
        line("<small><font color=\"gray\">normal readings stay small and gray</font></small>");
    }

    // 7: how many characters fit across, at each size
    private void widthPage() {
        raw("PAGE 7: characters across (count to where the line wraps or cuts)");
        String ruler = "0123456789";
        StringBuilder r = new StringBuilder();
        for (int i = 1; i <= 12; i++) r.append(ruler.substring(0, 9)).append(i % 10);
        line("normal: " + r);
        line("<big>big: " + r + "</big>");
        line("<h2>h2: " + r + "</h2>");
        line("<small>small: " + r + "</small>");
        line("<tt>tt: " + r + "</tt>");
    }

    // 10 to 12: the competition TeleOp's screen (CompetitionScreen) with sample facts; only the HTML format shows it as designed
    private void competitionPage(int variant) {
        raw("PAGE " + (10 + variant) + ": the competition screen, sample " + (variant == 0 ? "normal driving" : variant == 1 ? "a dash with a warning" : "faults and a red alliance"));
        CompetitionScreen.render(telemetry, CompetitionScreen.sample(variant));
    }

    // 8 and 9: how many lines fit
    private void linesPage(boolean large) {
        raw(large ? "PAGE 9: lines of large text (h3): how many are shown, does it scroll or cut?" : "PAGE 8: numbered lines at normal size: how many are shown, does it scroll or cut?");
        int n = large ? 14 : 40;
        for (int i = 1; i <= n; i++) {
            String text = String.format("Line %02d of %d", i, n);
            line(large ? "<h3>" + text + "</h3>" : text);
        }
    }
}
