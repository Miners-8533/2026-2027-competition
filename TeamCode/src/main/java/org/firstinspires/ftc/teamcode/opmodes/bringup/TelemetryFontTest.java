package org.firstinspires.ftc.teamcode.opmodes.bringup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Telemetry Font Test (specs/support-opmodes.md section 6): find out what the Driver Station screen can display.
 * Pages of test patterns (sizes, headings, styles, colors, widths, line counts, and three mock-ups of the match
 * screen) in each of the Driver Station's display formats (HTML, MONOSPACE, CLASSIC). No hardware is used and no log
 * is written.
 *
 * Controls (gamepad 1): D-pad right / left changes the page and A the format.
 *
 * The finding it was built for: only the HTML format formats text. Nested "big" and "small" tags change size;
 * font-size attributes and background colors are ignored; about 15 to 16 lines of about 36 characters fit; longer
 * lines wrap.
 * Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@TeleOp(name = "Telemetry Font Test", group = "2 Bring-up")
public final class TelemetryFontTest extends TeamOpMode {
    private static final Telemetry.DisplayFormat[] FORMATS = {Telemetry.DisplayFormat.HTML, Telemetry.DisplayFormat.MONOSPACE, Telemetry.DisplayFormat.CLASSIC};
    private static final String[] PAGES = {"sizes", "headings", "styles", "colors", "widths", "line counts", "match screen A", "match screen B", "match screen C"};

    @Override protected RunLog.Kind logKind() { return null; }
    @Override protected String title() { return "Telemetry Font Test"; }

    @Override
    protected void body() throws InterruptedException {
        Edge next = new Edge(), prev = new Edge(), fmt = new Edge();
        int page = 0, format = 0;
        telemetry.setDisplayFormat(FORMATS[format]);
        telemetry.setMsTransmissionInterval(100);
        while (!isStopRequested()) {
            if (next.pressed(gamepad1.dpad_right)) page = (page + 1) % PAGES.length;
            if (prev.pressed(gamepad1.dpad_left)) page = (page + PAGES.length - 1) % PAGES.length;
            if (fmt.pressed(gamepad1.a)) {
                format = (format + 1) % FORMATS.length;
                telemetry.setDisplayFormat(FORMATS[format]);
            }
            telemetry.addLine("Page " + (page + 1) + "/" + PAGES.length + ": " + PAGES[page] + " | " + FORMATS[format] + " | D-pad R/L page, A format");
            show(page);
            telemetry.update();
            sleep(50);
        }
    }

    private void show(int page) {
        switch (page) {
            case 0:     // sizes
                telemetry.addLine("plain text");
                telemetry.addLine("<big>big</big> <big><big>big big</big></big> <big><big><big>big x3</big></big></big>");
                telemetry.addLine("<small>small</small> <small><small>small small</small></small>");
                telemetry.addLine("<font size='1'>font size 1</font> <font size='5'>font size 5</font> <font size='7'>font size 7</font>");
                telemetry.addLine("<span style='font-size:30px'>style font-size 30px</span>");
                break;
            case 1:     // headings
                for (int h = 1; h <= 6; h++) telemetry.addLine("<h" + h + ">heading h" + h + "</h" + h + ">");
                break;
            case 2:     // styles
                telemetry.addLine("<b>bold</b> <i>italic</i> <u>underline</u> <b><i>bold italic</i></b>");
                telemetry.addLine("<tt>teletype 0123456789</tt>");
                telemetry.addLine("<s>strike</s> x<sup>sup</sup> x<sub>sub</sub>");
                telemetry.addLine("line one<br>line two after a br tag");
                telemetry.addLine("a &amp; b &lt; c &gt; d &nbsp;&nbsp;&nbsp; three nbsp before this");
                break;
            case 3:     // colors
                for (String c : new String[] {"#ff4040", "#ffd000", "#40c040", "#40a0ff", "#ffffff", "#808080"}) {
                    telemetry.addLine("<font color='" + c + "'>font color " + c + "</font>");
                }
                telemetry.addLine("<span style='background-color:#ff4040'>background color red</span>");
                telemetry.addLine("<font color='#000000' style='background:#ffd000'>black on yellow background</font>");
                break;
            case 4: {   // widths
                StringBuilder ruler = new StringBuilder();
                for (int i = 1; i <= 60; i++) ruler.append(i % 10 == 0 ? String.valueOf(i / 10) : String.valueOf(i % 10));
                telemetry.addLine(ruler.toString());
                telemetry.addLine("<tt>" + ruler + "</tt>");
                telemetry.addLine("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");
                telemetry.addLine("iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii");
                telemetry.addLine("<big><big>1234567890123456789012345</big></big>");
                break;
            }
            case 5:     // line counts
                for (int i = 1; i <= 24; i++) telemetry.addLine("line " + i + " of 24");
                break;
            case 6:     // match screen: big state lines
                telemetry.addLine("<big><big><b><font color='#40c040'>READY</font></b></big></big>");
                telemetry.addLine("<big><b>BLUE  held 3  AUDIENCE</b></big>");
                telemetry.addLine("<font color='#ffd000'>FULL: LOCKED, INTAKE OUT</font>");
                telemetry.addLine("aim REAR +0  AUDIENCE -3 deg");
                break;
            case 7:     // match screen: alerts
                telemetry.addLine("<big><big><b><font color='#ff4040'>BACKUP MODE</font></b></big></big>");
                telemetry.addLine("<big><font color='#ffd000'>WAITING FOR AIM</font></big>");
                telemetry.addLine("<big><b>RED  held ?  REAR</b></big>");
                telemetry.addLine("<small>pose (-12.3, 45.6, 90 deg)  1350 SPIN UP</small>");
                break;
            default:    // match screen: dense
                telemetry.addLine("<b>BLUE</b> | held 4 <font color='#ffd000'>FULL</font> | <font color='#40c040'>READY</font>");
                telemetry.addLine("cell AUDIENCE 34.2 in, setpoint 1350");
                telemetry.addLine("lock ON aligned, auto-launch armed");
                telemetry.addLine("<font color='#ff4040'>HIVE DASH refused: the robot reads empty</font>");
                telemetry.addLine("warnings: ZERO_NOT_VERIFIED");
                telemetry.addLine("123456789012345678901234567890123456");
                break;
        }
    }
}
