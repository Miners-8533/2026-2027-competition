# Robot design (ROBOT_1)

The physical robot as the software needs to know it: what is on it, how it is wired to the hubs, and the measured dimensions and constants the code depends on. Team 8533, BIOBUZZ season (2026-27).

This folder states **what is true of the robot now**. It is not a history. The one procedure kept here is the hub setup checklist, on the hub configuration page beside the device tables it uses.

| Page | Holds |
|---|---|
| [hub-configuration.md](hub-configuration.md) | every device: configuration name, type, hub, port; the hubs' identity; controllers; power; the checklist for setting a hub up and checking it |
| [chassis.md](chassis.md) | footprint, robot frame, drive train, odometry, measured drive performance |
| [mechanisms.md](mechanisms.md) | the element path and its four actuators: intake, pusher, flywheel (with its position servo), ramp |
| [sensors.md](sensors.md) | the five color sensors, the encoders, motor currents, battery voltage |
| [driver-station-screen.md](driver-station-screen.md) | what the Driver Station's telemetry area can show: the formatting that works, how much fits, the rules a match screen follows |

## What belongs here and what does not

| Content | Where |
|---|---|
| Hardware, wiring, dimensions, and constants that describe the robot itself (a servo position, an encoder's ticks per turn, where a sensor sits) | here |
| What the software does with them (control laws, state machines, timing, tuned powers and thresholds) | `specs/` |
| The field and the game rules the code relies on | `specs/` |
| Why something was decided, how it was measured, what was tried before | the commit messages and the git history. The `design/` folder that held dated notes, plans and checklists was removed on 2026-11-23 (last present at commit 433799d), so that `specs/` and `robot_design/` are the only description |
| Setup and test procedures | the hub checklist on [hub-configuration.md](hub-configuration.md); Pit Check and the other support opmodes in [specs/support-opmodes.md](../specs/support-opmodes.md); getting the logs in [specs/logging.md](../specs/logging.md) section 5 |
| Ideas agreed to keep but not built | [specs/idea-queue.md](../specs/idea-queue.md) |
| Rules, strategy, scouting, and why the robot concept is what it is | the separate BIOBUZZ knowledge repo |

**Rule for a change:** if a wire moves, a part is swapped, a servo position or an offset is re-measured, change the code and the page here in the same commit.

## Conventions

- **Front** is the end with the intake and the launcher. The robot launches out of the end it takes elements in. Left and right are judged facing the front.
- Lengths are inches unless marked, angles degrees. Pod offsets are millimetres because the Pinpoint takes them that way.
- **Names.** The team says *pusher* and *ramp*. The hub configuration still uses the older names and they must not be renamed: the pusher motor is `gate`, the ramp servos are `depolLeft` and `depolRight`. The *feed ramp* is a different part: the fixed end of the track that turns the elements up into the flywheel.
- A value marked **NOT RECORDED** is a gap nobody has written down, not a zero.
- Where a fact is an inference rather than a record, it says so.

## Chassis profiles

The code carries three chassis profiles and picks one by the Control Hub it runs on, so one build serves every chassis. Only `ROBOT_1` (the competition robot) is described here. `PROTOTYPE` and `DRAFT_1` are earlier drive bases with no hub registered; they are kept in the code and not documented.

## Open gaps (for the team to fill)

These are not recorded anywhere in the repository. Each is marked NOT RECORDED on its page.

1. Hub and port of the `CS5` color sensor and the `fly_pos` servo.
2. A saved copy of the active hub configuration file (`/sdcard/FIRST/<name>.xml`); none is in the repo.
3. Drive motor model and gear ratio, wheel diameter, and the belt and pulley sizes of the drive.
4. Flywheel motor model, wheel diameter, encoder ticks per wheel turn, and the fixed hood angle.
5. Intake motor model and the roller geometry.
6. Pusher: arm length, track radius, and the belt ratio (1:1 is inferred, see [mechanisms.md](mechanisms.md)).
7. Ramp servo part number and the ramp's reach from the robot origin.
8. Robot height and weight.
9. Where each color sensor sits along the track in inches or degrees (only their order is recorded).
10. CAD views, photos or drawings of any mechanism.
