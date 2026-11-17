# Specifications

What the team's robot software does and how it behaves, written so that it could be rebuilt from these pages without the source. Team 8533, BIOBUZZ season (2026-27).

These pages describe the **`integration` branch as of 2026-11-17**. They cover the team's own code only, not the FTC SDK.

## How to read them

Start with [architecture.md](architecture.md). It explains the layers, the control loop and the conventions every other page assumes, and it has a table of which features have run on the robot.

| Page | Covers |
|---|---|
| [architecture.md](architecture.md) | layers, the control loop, conventions, failure philosophy, feature status |
| [field.md](field.md) | field frame, every position the code uses, the game rules it is built around |
| [drive.md](drive.md) | chassis profile, mecanum drive, wheel check, localization, pose hand-over |
| [motion.md](motion.md) | action framework, drive to pose, aim, square on a wall, jostle, hold position |
| [intake-and-ramp.md](intake-and-ramp.md) | the intake and the ramp |
| [flywheel.md](flywheel.md) | flywheel speed control, sensor failure, position servo, launch map |
| [pusher.md](pusher.md) | the pusher's moves, jam handling, active hold, the pusher check |
| [element-counting.md](element-counting.md) | the slot count (CS1 to CS4) and the entry count (CS5) |
| [mechanisms.md](mechanisms.md) | the coordinator: launching, reject, intake rules, FULL, the pusher lock, warnings |
| [aiming.md](aiming.md) | aim heading, target lock, auto-launch zones, launch grid |
| [dashes.md](dashes.md) | route planner, Hive Dash, Park Dash, grid routes |
| [teleop.md](teleop.md) | the competition TeleOp: INIT, controls, automatic behavior, backup mode, the screen |
| [autos.md](autos.md) | the six autonomous programs and their shared pieces |
| [support-opmodes.md](support-opmodes.md) | Pit Check, tuners, studies, build aids |
| [logging.md](logging.md) | run log, settings dump, loop profiler, build stamp |
| [tools.md](tools.md) | PC tools the robot code depends on |

The physical robot (devices, wiring, dimensions, measured constants) is in [robot_design/](../robot_design/README.md). Rules, strategy and scouting are in the team's separate BIOBUZZ knowledge repo.

## What a page contains

- **Purpose** of each component and its **interface**: what it is told and what it reports.
- **Behavior** as rules, states and sequences.
- **Parameters** with their present values. A rebuild needs them; they are data, not code.
- **Status**: whether it has run on the robot.

What is deliberately left out: line-by-line logic, class and method names where a plain name will do, the history of how a value was reached, and procedures for testing. History and procedures are in `design/`.

## Conventions

- Lengths are inches, angles degrees, times seconds, unless marked. Flywheel speed is encoder ticks per second. Motor powers are fractions from -1 to 1.
- Field frame and alliance rotation: [field.md](field.md).
- **Front** of the robot is the intake and launcher end. Left and right are judged facing the front.
- The team's words: *pusher*, *ramp*, *launch*, *launcher*, *salvo* (one sweep of the pusher that launches everything held), *hive cell*, *flower*, *garden*. *Slot count* and *entry count* are these pages' names for the two element counts.

## Status words

| Words | Meaning |
|---|---|
| In the tested build, or used at Hartland | part of the build that ran at the Hartland Qualifier on 2026-11-07 (the `main` branch) |
| Ran in practice | exercised on the robot after Hartland, outside a match |
| Not run on the robot | builds and passes any PC checks, but has never run on the robot |
| Off, or switched off | present in the code but disabled by a setting |
| Hidden | an opmode that is complete but not shown on the Driver Station |

Being in the tested build does not prove every path of a feature was exercised. Where a specific part is known to be untried, the page says so.

## When the code changes

The specs are only useful while they are true.

- A change of behavior updates the page for that component in the same commit.
- A change of a tuned value updates its row in the page's parameter table.
- A change to a device, a wire or a measured dimension belongs in `robot_design/`.
- When a feature first runs on the robot, update its status here and in the table in [architecture.md](architecture.md).

## Known open points

Collected from the pages; each is explained where it arises.

1. The drive-to-pose gains have never been tuned on this robot ([motion.md](motion.md), [autos.md](autos.md)).
2. Odometry drift across the field is compensated in three separate places ([architecture.md](architecture.md)).
3. The entry count and its FULL sequence, in their present form, have not run on the robot ([element-counting.md](element-counting.md), [mechanisms.md](mechanisms.md)).
4. The present dashes cut clearances fine at the robot's present speed; the grid routes that address this have not run ([dashes.md](dashes.md)).
5. The launch grid has data for one cell from one run ([aiming.md](aiming.md)).
6. Several live settings are missing from the settings dump ([logging.md](logging.md)).
