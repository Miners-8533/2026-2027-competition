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

## As built and target

These pages hold two kinds of text and keep them apart.

- **As built** is everything not marked otherwise: what the `integration` branch does today.
- **Target** blocks, headed "Target T1" to "Target T10", say what the software should do instead where the code has a known weakness. **Nothing in a target block is in the code.**

A rebuild follows the targets and takes everything else from the as-built text. Each target gives the weakness, the requirement, the design, what would behave differently, and what must be measured or decided before it is real. The full list is in [architecture.md](architecture.md) section 9.

Some targets only move things around and change nothing the robot does in a match (T5, T6, T7, T8, T10), and T1 changes very little. The others change what the robot does (T2, T3, T9) or remove code (T4), and each names the run on the robot or the decision it waits on. T1, T3 and T9 record decisions the team made on 2026-11-17.

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
| Target | specified here, not in the code |

Being in the tested build does not prove every path of a feature was exercised. Where a specific part is known to be untried, the page says so.

## When the code changes

The specs are only useful while they are true.

- A change of behavior updates the page for that component in the same commit.
- A change of a tuned value updates its row in the page's parameter table.
- A change to a device, a wire or a measured dimension belongs in `robot_design/`.
- When a feature first runs on the robot, update its status here and in the table in [architecture.md](architecture.md).
- When a target is implemented, fold its text into the as-built description, delete the target block, and strike its row in [architecture.md](architecture.md) section 9.

## Known open points

Collected from the pages; each is explained where it arises.

| Open point | Answer |
|---|---|
| The drive-to-pose gains have never been tuned on this robot | Target T2 ([motion.md](motion.md), [support-opmodes.md](support-opmodes.md)); needs a tuning session |
| Odometry drift across the field was allowed for in four separate places | Target T1 ([aiming.md](aiming.md)): drift is accepted and trimmed by the operator. Mostly built 2026-11-17, not run |
| Two element counts with different rules for "full" | Target T3 ([element-counting.md](element-counting.md)): the CS5 entry count is the count. Built 2026-11-17, not run |
| The present dashes are too slow to use and cut clearances fine | Target T9 ([dashes.md](dashes.md)); needs the grid routes' first robot run |
| Several live settings are missing from the settings dump | Target T8 ([logging.md](logging.md)) |
| The entry count and its FULL sequence, in their present form, have not run on the robot | no design change; needs a run ([element-counting.md](element-counting.md), [mechanisms.md](mechanisms.md)) |
| The launch grid has data for one cell from one run | no design change; needs data ([aiming.md](aiming.md)) |

## Gaps found by a rebuild

On 2026-11-17 the team's code was rebuilt from these pages and `robot_design/` alone, without the source (branch `from-specs`). Where a page was not enough, the rebuild had to choose. Each such gap is listed here.

- **Filled** means the missing fact is now on the page. Every one was read from, or checked against, the code (`integration` at 995f197) on 2026-11-17.
- **Proposal** means the pages did not agree with each other and the rebuild chose; the team has not decided.

| Gap | Page | State |
|---|---|---|
| The layout of a log line: separator, the first two lines, the common column names, the start and end markers | [logging.md](logging.md) section 1.2 | filled |
| The names and the order of the sample columns | [logging.md](logging.md) section 1.5, [mechanisms.md](mechanisms.md) section 14 | filled |
| The exact wording of the events the pull tool matches | [logging.md](logging.md) section 1.6 | filled |
| The order in which the coordinator's rules run | [mechanisms.md](mechanisms.md) section 16.3 | filled |
| What an incomplete salvo does to the salvo counter | [mechanisms.md](mechanisms.md) section 3 | filled |
| How a leg is checked: the swept rectangle, the lane walls, the goal | [dashes.md](dashes.md) section 1.2 | filled; the section is rewritten from the code |
| The layout of the grid route tables and the numbering of their states | [dashes.md](dashes.md) section 5.1 | filled |
| The corner speed table: this page and `design/dash-planner-grid.md` gave different values | [dashes.md](dashes.md) section 5.1 | settled: this page's values are the code's |
| The layout of the launch grid data | [aiming.md](aiming.md) section 4.1 | filled |
| The `PROTOTYPE` and `DRAFT_1` chassis profiles; how the hub's ids are read | [drive.md](drive.md) section 1 | filled |
| The controls of the support opmodes | [support-opmodes.md](support-opmodes.md) | filled. Six of them use the Back button, which the page's own rule forbade |
| The 34 tests of Drive Characterization | [support-opmodes.md](support-opmodes.md) section 4.3 | filled |
| When the pusher counts as not calibrated | [pusher.md](pusher.md) section 1 | filled |
| The heading asked for at a pass-through point of an auto | [autos.md](autos.md) section 2.7 | filled |
| Where a flower stand stands: centered on its opening, not flush against the wall | [field.md](field.md) section 3 | filled |
| The names in the grid route file, its model text, and how near a goal must be to have a table | [dashes.md](dashes.md) section 5.1 | filled |
| Details of the support opmodes a second pass still had to choose: the tuners' log lines and settings, the drive tests' seeding and random draws, the offset check's method, automatic clear | [support-opmodes.md](support-opmodes.md) | filled |
| Three choices the code still had to make after that: what the corner speed field holds, the Launch Grid Tuner's start index, re-aim limit and table speed, and what happens between two drive tests | [dashes.md](dashes.md) section 5.1, [support-opmodes.md](support-opmodes.md) sections 2.2 and 4.3 | filled |
| Where the worked-out launch spot of target T1 lives, given the layers of target T5 | [aiming.md](aiming.md) section 5.4 | proposal |

The generated tables themselves (the grid routes and the launch grid data) are data, not text. A rebuild regenerates them with the PC tools; it cannot take them from these pages.
