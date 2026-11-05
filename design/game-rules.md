# BIOBUZZ rules that matter to the autos (read 2026-11-04)

Source: the Competition Manual on https://ftc-resources.firstinspires.org/ftc/game (Section 9 ARENA TU06, Section 10 Game Details TU05, Section 11 Game Rules TU07; the page lists the complete manual at TU07, updated 2026-10-29, so re-check for team updates). Quotes are short; the section numbers say where to look.

## Match and scoring (10.1, 10.4, 10.5, Table 10-2)
- 30 s AUTO, an 8 s transition, 2 min TELEOP. Humans may not touch the robot or console in AUTO (G401).
- LEAVE 3 (AUTO): the robot is no longer contacting the perimeter wall, assessed at the END OF AUTO.
- PARK 5 in AUTO and 5 in TELEOP: the robot is at least partially in its own LOADING ZONE. AUTO PARK is assessed at the end of AUTO, TELEOP PARK at the end of the match.
- HIVE TIP 20, in AUTO or TELEOP: a tip that is complete before TELEOP starts is assessed as AUTO (10.5 B). Only launching into the UPWARD-facing cell may tip the hive (G417); launching at the downward-facing cell while a hive is tipping may disrupt the tip, so pause.
- Pollen / nectar left in an upward-facing cell at the end: 2 each. Flowers (owned, bottom nectar bonus) and gardens (1 each) score in TELEOP; nectar into flowers only in the last 60 s (G410).
- Ranking points: SWARM = combined LEAVE + PARK points of the alliance at or above 16 (all other events); POLLINATOR 1 at 4 tips, POLLINATOR 2 at 7 tips; win 3, tie 1. Two robots with LEAVE in AUTO and PARK in TELEOP already make 16; LEAVE + PARK in AUTO for both also makes 16.
- Staging (10.3.1): 3 nectar in each upward-facing cell, 4 pollen in each flower, 4 in each garden, 4 preloaded in each robot.

## Starting, motion, and sides
- G304: fully contained on its own alliance's side (FIELD columns A, B, C for red, D, E, F for blue; Figure 9-4), touching the perimeter wall, NOT contacting or in the scoring volume of a flower, NOT in the loading zone, in its starting configuration, touching exactly 4 pre-load pollen, motionless after INIT.
- G402: in AUTO each alliance has priority over the field and elements on ITS side (columns again); controlling scoring elements on the opposing side or contacting an opposing robot on their side disrupts AUTO (major foul).
- G403: no powered movement of the robot or any mechanism in the transition. The TeleOp's INIT commands the ramp servos to STOWED (the `Ramp` constructor): an auto must end with the ramp stowed, or INIT pressed in the transition moves it (the manual says to wait for TELEOP to press INIT if INIT moves anything).
- G407 at most 4 elements controlled at once; G409 do not catch elements spilling from a tipped hive (driving under a hive is allowed, stopping under it repeatedly is the strategic example); G412 / G415 / G417 / G418 as written.

## Loading zone (Section 9, CAD)
- Blue: x 23.9 to 46.6, y 59.1 to 70.1 in our frame (against the Blue Wall, audience end). Red is its 180 degree rotation. One zone per alliance; both partners park in it.

## Not settled by the text (the figures decide)
- Which half of the field is blue by the columns (G304.A, G402): Figures 9-4, 9-5 and 11-1. The start positions of the duo autos (one at the audience wall, one at the rear wall) and the gardens / flowers they gather from have to be on our side.
