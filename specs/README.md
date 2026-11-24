# Specifications

What the team's robot software does and how it behaves, written so that it could be rebuilt from these pages without the source. Team 8533, BIOBUZZ season (2026-27).

These pages describe the **`integration` branch as of 2026-11-23** (robot code at commit 64087f6). They were first written on 2026-11-17 and are kept up with the code; the last full pass against the code and the run logs, those of the Novi Qualifier of 2026-11-21 included, was on 2026-11-23. They cover the team's own code only, not the FTC SDK.

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
| [idea-queue.md](idea-queue.md) | ideas agreed to keep but not built, and what was decided against. Not a description of the code |

The physical robot (devices, wiring, dimensions, measured constants) is in [robot_design/](../robot_design/README.md). Rules, strategy and scouting are in the team's separate BIOBUZZ knowledge repo.

**There is no `design/` folder any more** (removed 2026-11-23). `specs/` and `robot_design/` are the only description of the software and the robot. Five things were carried over from it: the hub setup checklist ([robot_design/hub-configuration.md](../robot_design/hub-configuration.md)), what the Driver Station screen can show ([robot_design/driver-station-screen.md](../robot_design/driver-station-screen.md)), Pit Check's controls and output ([support-opmodes.md](support-opmodes.md) section 1), getting the logs off the hub ([logging.md](logging.md) section 5) and the idea queue ([idea-queue.md](idea-queue.md)). The earlier design notes, plans and checklists are in the git history, last present at commit 433799d.

## What a page contains

- **Purpose** of each component and its **interface**: what it is told and what it reports.
- **Behavior** as rules, states and sequences.
- **Parameters** with their present values. A rebuild needs them; they are data, not code.
- **Status**: whether it has run on the robot.

What is deliberately left out: line-by-line logic, class and method names where a plain name will do, the history of how a value was reached, and procedures for testing. History is in the commit messages and the git history. The few procedures that are kept are where their subject is: the hub setup checklist in `robot_design/`, Pit Check in [support-opmodes.md](support-opmodes.md), getting the logs in [logging.md](logging.md).

## As built and target

These pages hold two kinds of text and keep them apart.

- **As built** is everything not marked otherwise: what the `integration` branch does today.
- **Target** blocks, headed "Target T1" to "Target T11", say what the software should do instead where the code has a known weakness. **Nothing in a target block is in the code.**

A rebuild follows the targets and takes everything else from the as-built text. Each target gives the weakness, the requirement, the design, what would behave differently, and what must be measured or decided before it is real. The full list is in [architecture.md](architecture.md) section 9.

Some targets only move things around and change nothing the robot does in a match (T5, T6, T7, T8, T10), and T1 changes very little. The others change what the robot does (T2, T3, T9, T11) or remove code (T4), and each names the run on the robot or the decision it waits on. T1, T3 and T9 record decisions the team made on 2026-11-17, and T11 one of 2026-11-18.

## Conventions

- Lengths are inches, angles degrees, times seconds, unless marked. Flywheel speed is encoder ticks per second. Motor powers are fractions from -1 to 1.
- Field frame and alliance rotation: [field.md](field.md).
- **Front** of the robot is the intake and launcher end. Left and right are judged facing the front.
- The team's words: *pusher*, *ramp*, *launch*, *launcher*, *salvo* (one sweep of the pusher that launches everything held), *hive cell*, *flower*, *garden*. *Slot count* and *entry count* are these pages' names for the two element counts.

## Status words

| Words | Meaning |
|---|---|
| In the tested build, or used at Hartland | part of the build that ran at the Hartland Qualifier on 2026-11-07 (merged into `main` that day) |
| Ran in practice | exercised on the robot after Hartland, outside a match |
| Ran at Novi, or used at the Novi Qualifier | ran in a match at the Novi Qualifier on 2026-11-21, on the `integration` branch (builds dfee25d, 1fa5b23, and 9599d8b with the uncommitted changes that became 9fcb12d: stamp `built 2026-11-21 16:24:44 (PC clock)`, fingerprint 82e29283). That last build is `main` since 2026-11-23 |
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
| Two element counts with different rules for "full" | Target T3 ([element-counting.md](element-counting.md)): the CS5 entry count is the count. Built and first run 2026-11-17; it undercounted, its levels changed 2026-11-18 and 2026-11-19. **CS5 is switched off in the code since 2026-11-20** (the user), so at present the robot has no held count and is never FULL; the Novi Qualifier was played that way |
| The present dashes are too slow to use and cut clearances fine | Target T9 ([dashes.md](dashes.md)); the grid routes as smooth paths are switched on since 2026-11-18 and need their first robot run |
| Several live settings are missing from the settings dump | Target T8 ([logging.md](logging.md)) |
| The entry count ran low in its first robot runs; at the levels of 2026-11-19 it is right at 204 of 208 loads by replay, and those levels ran only in four Solo autos on the night of 2026-11-19 (not examined) before CS5 was switched off | needs CS5 switched on again and a run; the true number of each load comes from CS2 in every log ([element-counting.md](element-counting.md) section 2.7) |
| CS2 has been moved to the top of the feed ramp (2026-11-18). Nothing counts what a salvo launches yet | Target T11 ([element-counting.md](element-counting.md) section 6): CS2 has left the slot count and is recorded through every salvo (built 2026-11-18; recorded in the runs of 2026-11-18 and 19; **switched off in the code since 2026-11-20**). The counting rule waits on those recordings, and the cost of the fast reads during a salvo is not known |
| The launch grid: both blue cells are measured (2026-11-18) and the grid was used in every match at the Novi Qualifier. Its no-shot zone begins about 34 in from the audience cell and the driver only learns of it on arrival: up to 17 blocked presses in a playoff match | not decided ([aiming.md](aiming.md) section 4.3) |
| A press of the driver's A just after a salvo has ended starts another salvo, whatever is held. At the Novi Qualifier that gave 5 to 15 second salvos a match, each holding the drive about 0.85 s; about half launched nothing | not decided ([teleop.md](teleop.md) section 5) |
| A stale empty baseline on one slot sensor (CS3) kept the track from reading empty all day at the Novi Qualifier, so the pusher check after a salvo was skipped every time | the baseline was re-measured on 2026-11-21 (not run since). After any change to a sensor or the track, compare every sensor's empty reading with its stored baseline ([element-counting.md](element-counting.md) section 1.6) |
| The autos' start pose at the audience wall (Flower Side and Blaser's Gambit, y = 7.5) is not on a tile seam, and the team lines the robot up on the tile marks | not decided ([autos.md](autos.md) section 9) |

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
| The corner speed table: this page and the earlier design notes (`design/dash-planner-grid.md`, in the git history) gave different values | [dashes.md](dashes.md) section 5.1 | settled: this page's values are the code's |
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
| How the route planner picks among routes: the straight line is one candidate and not preferred, the first long leg is charged the turn from the robot's own heading, and a short leg makes the next turn free. Found by running both planners on the same 3,000 trips (they agreed on every refusal and differed in the route on 4 to 8 %) | [dashes.md](dashes.md) section 1.3 | filled; the rebuilt planner on `from-specs` follows it since e6b048b |
| Four choices in the kinds of the CS5 entries: what the color values of the first second are for, which pass gets a color value read as a second entry is counted, a saved load order that does not fit the saved count, and the form of the log lines. Found with them: what happens while an element is parked, a pass that never gets an entry, a failed color read, the empty color level from fewer than 3 samples, and the entry's number | [element-counting.md](element-counting.md) sections 2.5 and 2.8, [logging.md](logging.md) section 1.6 | filled, from the code (`integration` at 2eee48e) |

The generated tables themselves (the grid routes and the launch grid data) are data, not text. A rebuild regenerates them with the PC tools; it cannot take them from these pages.
