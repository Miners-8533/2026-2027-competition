# Driver Station screen: what telemetry formatting it can show

Measured 2026-11-06 from the user's nine photos of the `Telemetry Font Test` opmode (page 1 to 9, HTML format, D-pad = page). The Driver Station is the Control Hub's DS app on the team's phone (the pictures show the telemetry area at the right half of the screen, 13.49 V battery, "DS: 21%"). Source of the numbers: the photos only. The competition TeleOp's match screen is built on these findings (2026-11-06) and was used at the Hartland and Novi Qualifiers; what it shows is in [specs/teleop.md](../specs/teleop.md) section 11.

## Only the HTML display format formats anything
The photos are all of the HTML format (`Telemetry.DisplayFormat.HTML`, the test opmode's start format). The user (2026-11-06) saw that the other formats the test opmode cycles to with A (MONOSPACE and CLASSIC) showed **no formatting at all**, so everything below needs `telemetry.setDisplayFormat(HTML)`. The competition TeleOp sets it for its match screen (`CompetitionScreen`) and so does the `Telemetry Font Test` opmode; any other opmode runs in the default format and shows none of this. A screen must set HTML first and write its lines with the tags below.

## What works
- **Big and small tags:** `<big>` nests, each nesting one step bigger (x1 to x5 all rendered, x5 about three times the normal height); `<small>` x1 and x2 render smaller than normal. Use these for size.
- **Headings h1 to h6** render, bold and larger, but each one takes about 2.5 to 3.3 normal lines of height because of the spacing around it. Only five fit on the page (h6 is cut off), and only **four h3 lines** fit. Prefer `<big>` for large text.
- **Styles:** bold, italic, underline, strike, subscript and superscript all work, and so do `<br>` and the entities `&amp;`, `&lt;`, `&nbsp;`.
- **Fonts:** `<tt>` and `face="monospace"` give a monospace font (use it to line up columns of numbers); `face="serif"`, `"sans-serif"` and `"condensed"` also work, and condensed is narrower than normal (about a fifth more characters per line).
- **Colors:** named colors (red, green, blue, yellow, cyan, magenta, gray, orange via `style="color: ..."`) work on the text, plain and with `<big><b>`: "BIG BOLD RED" and "BIG BOLD GREEN" are readable at a glance.
- **Page 6, the mock status display, is the model to follow:** READY (big, green) with `1350 ticks/s` small beside it; `COUNT 4` big with `LOCKED 92 deg` smaller; BACKUP MODE big; `HIVE DASH arrived, aligned` with the state in a color; a yellow `WARNING` line; the pose, loop time and voltage small; and the normal readings small and gray. All of it fits and reads from arm's length.

## What does not work
- **`<font size=...>` and `style="font-size: ..."` are ignored** (1 to 7, px, pt, em, %, xx-large: every line the same normal size). Only `<big>`, `<small>` and the heading tags change the size.
- **Background colors are ignored** (black on yellow, white on red, white on green and white on blue showed no background): color the text, do not rely on a background.

## How much fits
- **Width:** about 36 normal characters per line, about 28 in `<big>`, about 25 in an h2. A longer line wraps onto the next line (it is not cut), which pushes everything below it down.
- **Height:** the page shows about 19 normal lines in all, of which the first three are the opmode's own header ("Page n of 9 ... format" and the page title) in the test; the numbered lines showed 14 full lines of the 40 and part of a 15th, and nothing scrolls: the rest is cut off. So a competition screen has about **15 to 16 normal lines**, fewer if some are big: one `<big>` line costs about 1.5 normal lines, an h3 about 3.3.
- Lines that matter must be near the top; anything past the bottom is lost.

## Rules for a match screen
0. Set `telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)` at the start of the opmode.
1. The first lines are the few things the driver acts on, in `<big><b>` and colored (green = ready, yellow = wait or warning, red = a fault): flywheel READY or the wait, the element count, LOCKED or the lock state, the dash state, the warnings.
2. Everything else is normal or `<small>` and gray: pose, loop time, voltage, the counter details.
3. Keep every line under about 30 characters (big lines under about 24) so nothing wraps, and keep the total under about 15 lines.
4. Use `<tt>` only where columns must line up.
5. Test any new layout on the Driver Station before a match. The `Telemetry Font Test` opmode (hidden) is the check: its pages 1 to 9 are the formatting samples measured here, and pages 10 to 12 draw the match screen with sample facts (normal driving, a dash with a warning, faults with a red alliance).
