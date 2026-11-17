# Dashes: Hive Dash, Park Dash and route planning

Two driver features of the competition TeleOp in which the robot drives itself across the field while a D-pad direction is held.

| Feature | Held control | Does |
|---|---|---|
| **Hive Dash** | D-pad left or right | drives to the launch spot of a hive cell, locks onto the cell, launches when aligned |
| **Park Dash** | D-pad up or down | drives to a park corner of the loading zone and holds there |

Both are pure logic with no hardware access. Each loop the TeleOp hands them the pose and some flags and applies what they return. That is what lets a PC simulation run the same code.

Field geometry: [field.md](field.md). Target lock: [aiming.md](aiming.md).

## 1. Route planner

**Purpose.** A list of waypoints from the robot's pose to a goal that keeps the robot clear of the hive supports and the flower stands.

### 1.1 Obstacles

- The two foot bars and four A-frame legs of the hive, for a robot of a given height.
- The four flower stands, each grown by a clearance.
- **Virtual lane walls.** Between the bars, walls keep the robot's center within about 4 in of the lane's middle line (y = 0), so a route under the hive is always centered. If routes under the hive are switched off, the whole lane is closed instead.

### 1.2 Checking a leg

A straight leg is acceptable when the area the robot sweeps along it, grown by a margin, touches no obstacle.

- A **long leg** (at least the align length) is driven with the robot aligned to it, front or rear leading, so the swept area is the robot's real rectangle moved along the leg.
- A **short leg** is not aligned, so the swept area uses the robot's far-corner radius.
- Where the heading changes, the robot is checked as a circle of the far-corner radius.

**Exemptions** that keep real situations plannable:

- Behind the start of a route the robot is only moving away, so a leg from the start is checked from the start point onward, with the robot's own circle at the start checked without margin.
- A robot that starts inside the lane may always leave: its first leg ignores the virtual walls, for the whole leg. (A short leg is checked with the far-corner radius plus the margin, 13.5 in, which is more than the lane's half width; if the walls counted anywhere on that leg, a robot in the lane could never reach a gate. Found by the rebuild of 2026-11-17.)
- On a leg that starts at the robot outside the lane, the walls are only checked from a short distance along the leg. **Not specified:** that distance.
- A robot that starts within reach of a flower and moves away from it ignores that flower on its first leg (it has just gathered there).
- A leg that ends at a launch spot is judged at the spot with the far-corner circle only, because the audience launch spot is only 6.8 in from a flower stand.

### 1.3 Choosing a route

1. If the straight line from the robot to the goal is acceptable, use it.
2. Otherwise search the chains of at most 3 route points (the four helper corners and the two lane gates) over acceptable legs, and take the cheapest.
3. Cost of a leg = its length, divided by the unaligned speed ratio if it is not aligned, plus a turn penalty for each degree turned between aligned legs. A leg can be driven front or rear first, so a turn is folded to between 0 and 90 deg.
4. If nothing is found, the plan is a refusal with a reason.

The route is planned once, when the D-pad is pressed, and kept for that dash.

A separate test answers whether the robot's own footprint already touches a support; a dash is refused in that case.

### 1.4 Parameters

| Name | Value |
|---|---|
| Robot height for the supports | 18 in |
| Margin | 1.5 in |
| Flower clearance | 3.0 in |
| Lane half width | 12.35 in |
| Align length | 40 in |
| Unaligned speed ratio | 0.75 |
| Turn penalty | 0.1 in per deg |
| Routes under the hive | allowed |
| Helper corners / lane gates | (±36, ±40) / (±34, 0) |
| Most via points | 3 |

3 in is the largest flower clearance that still leaves every launch spot reachable.

## 2. Leg follower (shared by both dashes)

Each loop it answers: which leg the robot is on, whether the goal is reached, and a translation command in the field frame.

- **Every leg but the last:** drive along the leg at the dash's power, with a sideways correction (proportional to the distance off the leg's line, limited). A via point counts as reached when the robot is within the pass tolerance of it along the leg, or past it. The robot does not stop there.
- **Corner slow-down (Hive Dash only):** before a via point the power ramps down to a corner power that depends on the turn there: no slowing below the free angle, the full corner power from the full angle on, in proportion between.
- **The last leg:** drive straight at the goal point with a power proportional to the distance, never below a minimum, so a robot that overshoots or passes to the side is driven back.
- **Approach limit (Hive Dash only):** within a radius of the goal the power is capped.
- **Arrival:** within the goal tolerance on the last leg.

The dash, not the follower, decides the heading:

- on an aligned leg the robot turns to the leg's direction, front or rear leading, whichever needs less rotation; its speed is scaled by the cosine of the heading error, never below a floor;
- on a short leg it keeps the heading it has;
- the field-frame command is rotated into the robot frame with the current heading, so the robot moves along the leg while it is still turning.

## 3. Hive Dash

### 3.1 Which cell

Left and right follow the driver's view of the field.

| Alliance | D-pad left | D-pad right |
|---|---|---|
| Blue | audience cell | rear cell |
| Red | rear cell | audience cell |

Holding both, or neither, means no dash. The goal is the cell's launch spot.

### 3.2 States

| State | Behavior | Leaves when |
|---|---|---|
| Idle | nothing | a direction is pressed: plan and start, or refuse |
| Driving | follow the route; the dash turns the chassis until the lock takes over | arrival |
| Arrived | no translation; the target lock finishes aligning | aligned and stopped: request the launch |
| Launching | wait for the salvo | the salvo is done, or a timeout |

### 3.3 Rules

**Refused, with no movement,** when: the TeleOp says a dash is not available (backup mode, a Pinpoint fault, the pose not seeded); the robot reads empty; the robot's footprint touches a support; or the planner finds no route. A refusal is logged and shown on the screen for 3 s.

**"Reads empty"** means a count is being read and neither the entry count nor the slot count holds anything. If no count is reading, the robot is taken to hold something.

**While it runs:**

- The driver's sticks are ignored.
- The dash's cell **overrides** the nearest-cell choice for the launch speed and the lock, and the launch speed is worked out for the launch spot from the start, so the flywheel is ready on arrival.
- The **target lock takes the heading** within the lock radius of the goal. On the last leg it takes over earlier, at the early lock radius, provided the robot is far enough from the field center not to be under or beside the hive. The aim turn is then finished on arrival.
- On the last leg the robot leads with whichever end leaves the smaller turn onto the cell.
- **Launch** is requested when the lock is aligned and the chassis is below the stopped speed. If the robot reads empty on arrival, no launch is requested.
- If the launch does not start within 1 s, or the salvo does not finish within 6 s, the dash ends.

**Cancel.** Releasing the D-pad cancels at once and gives the sticks back. A salvo that has already started always finishes. There is no stuck detection and no overall timeout: the driver sees a stuck robot and lets go.

**Dry run.** A switch makes the dash plan, log and draw its route without driving.

**Overlay.** The supports, route, goal and robot are drawn on the Dashboard's field view at 5 per second while a plan is shown. It can be switched off for matches.

### 3.4 Parameters

| Name | Value |
|---|---|
| Maximum power along a leg | 0.85 |
| Maximum turn power | 0.5 |
| Lock radius | 20 in |
| Early lock radius / only beyond this \|x\| | 32 in / 36 in |
| Goal tolerance | 2.5 in |
| Pass tolerance at a via point | 3.0 in |
| Last leg: power per inch / minimum power | 0.05 / 0.08 |
| Sideways correction: per inch / limit | 0.05 / 0.3 |
| Heading: kP / kD | 1.5 / 0.08 |
| Alignment speed floor | 0.4 |
| Approach: power limit / radius | 0.5 / 32 in |
| Corner: power / free angle / full angle / ramp | 0.5 / 20 deg / 90 deg / 0.03 power per inch |
| Hold before launching | 0 s |
| Launch start timeout / salvo timeout | 1.0 s / 6.0 s |
| Refusal shown for | 3.0 s |

## 4. Park Dash

### 4.1 Which corner

Up is the corner on the driver's left, down the corner on the driver's right.

| Alliance | Up (driver's left) | Down (driver's right) |
|---|---|---|
| Blue | (40, 60), audience side | (26, 60), hive side |
| Red | (-40, -60) | (-26, -60) |

### 4.2 States

| State | Behavior | Leaves when |
|---|---|---|
| Idle | nothing | a direction is pressed: plan and start, or refuse |
| Driving | follow the route | arrival within the goal tolerance |
| Parked | no drive power from the dash; it asks the TeleOp for *hold position*, which resists a push | the D-pad is released |

### 4.3 Rules

- Plans with the same planner and the same values as the Hive Dash.
- **Stows the ramp** once, when it starts, and does not wait for it.
- No cell, no lock, no launch. The robot ends with the heading of its last leg; any heading counts.
- **No corner slow-down and no approach limit**, unlike the Hive Dash.
- **Refused** when a dash is not available (backup mode, a Pinpoint fault, the pose not seeded), while the Hive Dash is running, when the robot touches a support, or when no route is found.
- The Hive Dash does not start while the Park Dash is running.
- Releasing the D-pad, or pressing the other direction, cancels at once.
- A dry-run switch plans and logs without driving.

| Name | Value |
|---|---|
| Maximum power / maximum turn power | 0.85 / 0.5 |
| Goal tolerance / pass tolerance | 2.0 in / 3.0 in |
| Other follower values | as the Hive Dash |

## 5. Grid routes (built, switched off, never run on the robot)

**Purpose.** Replace the hand-placed route points with precomputed, time-optimal routes that know about momentum, so the robot does not swing wide at corners or slide past its goal.

Each dash has its own switch. With a switch off, everything in sections 1 to 4 applies. With it on, only the route and how it is driven change; arrival, aiming, launching, parking, refusals and the overlay stay the dash's own.

### 5.1 Tables

- A lattice of nodes every 6 in from -66 to 66 on both axes, each with 16 travel directions, plus a final state (straight to the goal).
- For each of four blue goals (two launch spots, two park corners) a **tree**: for every state, the next state, the time to the goal, and the speed to carry at its end. Red uses the same tables with the pose rotated 180 degrees.
- A **corner speed table**: the fastest speed at which a turn of a given angle can be taken.
- The tables are generated offline by a PC tool that simulates the robot, and are stored as generated source. They record the lattice shape and the motion values they were built with.
- **Layout of the generated file** (from the generator in `tools/dash`; not checked against the code that reads it). It holds:
  - a text naming the lattice (the *geometry*) and a text naming the motion values (the *model*) the tables were built with;
  - the corner speed table;
  - for each of the four goals: its name, its x and y, and the heading at the goal (a cell's aim; none for a park corner);
  - for each goal three packed arrays with one entry per state, each stored as Base64 text: the next state (2 bytes; 0xFFFF = none), the time to the goal in milliseconds (2 bytes; 0xFFFF = unreachable), and the speed at the end of the leg in half inches per second (1 byte, rounded down).
  - A state is a node (23 by 23) with one of 16 travel directions, or a final state: about 9,000 states per goal. A final leg is at most 30 in.
  - **Not specified:** how a node, a direction and a final state are numbered, the byte order, and the wording of the geometry and model texts. A rebuild needs them to read the existing file; without them it regenerates the tables with its own numbering.
- If a table's lattice shape does not match the code, or a goal has moved (a launch spot or park corner changed on the Dashboard), the dash logs why and falls back to the planner of section 1.

| Corner angle (deg) | 0 | 10 | 20 | 30 | 45 | 60 and more |
|---|---|---|---|---|---|---|
| Speed limit (in/s) | 66 | 60 | 44 | 36 | 24 | 20 |

**To check:** `design/dash-planner-grid.md` records a later calibration (2026-11-07) with other values: 66 up to 10 deg, 64, 50 at 20 deg, 36 at 30 deg, 31 at 45 deg, 24 from 60 deg. Which table is in the code is not checked.

### 5.2 Picking a start

The cheapest start state among the lattice nodes within 9 in of the robot (18 in if none serves), counting the hop to the node, the body turn and the time from there to the goal. The hop and the turn onto the first leg are checked against the supports and flowers.

### 5.3 Following

- Each leg is driven aligned, front or rear first, at a target speed limited by the top speed and by what lets the robot brake in time for every corner limit ahead.
- A feedforward from speed to power, plus static friction terms; zero power (the motors brake) whenever the robot is more than 3 in/s too fast.
- A sideways correction with damping.
- **A cell's last leg is driven already turned to the aim**, sideways if need be; the target lock takes over only within 20 in.
- **The last leg stops on a speed profile** and arrival counts only below 10 in/s.

### 5.4 Off-route guard

If the robot is more than 6 in from the leg it is driving, it brakes. Once it is slower than 6 in/s, or after 1 s, a new route is picked from where it is, at most 2 times, after which the dash cancels.

### 5.5 Parameters

| Name | Value |
|---|---|
| Lattice pitch / half span / directions | 6 in / 66 in / 16 |
| Pick radius / fallback radius | 9 in / 18 in |
| Top speed / planning deceleration | 66 in/s / 120 in/s² |
| Tracking allowance: on the route / at the goal | 3.0 in / 0.5 in |
| Follower maximum power | 0.85 |
| Feedforward gain / dead zone | 92 in/s per unit / 0.11 |
| Brake when faster than the target by | 3 in/s |
| Static friction: forward / strafe / turn | 0.06 / 0.10 / 0.08 |
| Last leg: top speed / deceleration / arrival below | 40 in/s / 100 in/s² / 10 in/s |
| Off route / re-pick below / re-picks | 6 in / 6 in/s / 2 |

## 6. Status and known weaknesses

- Hive Dash and Park Dash with the planner of section 1 were in the build at Hartland, and the drivers did not press either in a match: no dash is recorded in any of the event's TeleOp logs (83 trips to a launch, all driven by hand). The dashes the logs do hold are from practice.
- **The team's judgement (2026-11-17): both dashes are too slow to be worth using in a match as they are, and need refinement before the drivers rely on them.**
- In simulation on a model of the robot as it now drives (faster than when the corner and approach values were set), the present dashes touch a flower stand or a support from a noticeable share of starting positions, and the Park Dash, having no corner slow-down, is the worse of the two. The grid routes touch nothing in the same simulation while the pose is true. This is the reason the grid routes exist, and they still need their first robot run.
- **Both gates of target T9 were run in simulation on 2026-11-17** (section 6.2). The grid routes pass the speed gate by a small margin and fail the safety gate: with the pose out by 4 in they touch a support or a flower stand from a few percent of starts. Today's planner fails both.

### 6.1 Target T9: a dash that is worth pressing

Not in the code. See "As built and target" in the [README](README.md).

**The weakness.**

- **Too slow.** The Hive Dash's corner and approach limits were set by hand for a slower robot and are cautious everywhere: the approach is capped within 32 in of the goal, and every corner is slowed by a fixed rule. A driver gets there sooner by hand.
- **And still not safe at speed.** In simulation the same dashes touch a support or a flower stand from a noticeable share of starts. The Park Dash has no corner slow-down at all. The planner checks the geometry of the legs, not where momentum carries the robot.
- So the hand-set values are wrong in both directions. Tuning them further by hand trades one fault for the other.

**Requirement.**

1. **Worth pressing.** A dash reaches its goal at least as fast as a practiced driver does by hand over the same trip. A dash that does not is not offered to the drivers.
2. **Safe at that speed.** No contact with a support, a flower stand or a wall from any start, with the pose out by the accepted drift of 4 in (target T1, [aiming.md](aiming.md) section 5).
3. **No hand-set speed values.** Speeds come from the measured drive.
4. Both dashes obey the same rules. They differ only in the goal and in what happens on arrival.

**Design.**

- **The grid routes are the route source** (section 5). They are built for exactly this: each route is the fastest one for the measured robot, and the follower brakes only as much as the next corner needs, at up to 66 in/s. The planner of section 1 stays only as the fallback for the cases section 5.1 names.
- **One follower contract** for both dashes: the speed for every corner from the corner speed table, a stop on a speed profile, arrival only below a speed. The Park Dash is no longer a special case.
- **The speed gate.** The baseline is the drivers' own transit time, from the cycle times tool ([tools.md](tools.md)) over the logged matches, per trip (which side of the field to which launch spot or park corner). The simulator's survey gives the dash's time for the same trips. The dash must not be slower.
- **The safety gate.** The same survey, run with the pose shifted by the accepted drift in each direction, must show no contact. The 3 in flower clearance of the planner is smaller than that drift, which is one reason the present dashes touch.
- **The model is kept true.** It is re-fitted from a Drive Characterization run after any change to the drive, the tables are regenerated, and both gates are run again.
- The simulator reproduces about 70 percent of logged dashes within its limits today. A clean survey therefore ranks options and does not prove a small clearance: the first robot runs of a new route source are a dry run, then low power, then full power, and the robot's logged times replace the simulated ones in the speed gate.

**What would behave differently.** Both dashes take momentum-aware routes at the speeds the robot can hold. The Park Dash brakes for corners. The dashes come back to the drivers only when they pass both gates.

**Waits on** route tables that keep the accepted drift clear (section 6.2), then the grid routes' first run on the robot. The drivers' baseline and both gates exist (2026-11-17).

### 6.2 The gates, run in simulation on 2026-11-17

Built in the tools ([tools.md](tools.md) section 3), not on the robot. Everything here is simulated on the wheel model fitted on 2026-11-07; nothing ran on the robot. The grid routes were switched on in the simulator only; on the robot they stay off.

**The baseline.** The Hartland logs hold 83 trips to a launch, none with a dash, and 71 of them of 12 in or more. The drivers took a median 2.43 s (mean 2.73 s, 90th percentile 4.92 s) over a median 60 in in a straight line, about 28 in/s. A trip runs from the last pickup that mattered to within 6 in of where the next salvo was fired.

**The speed gate.** The real Hive Dash driven from the start of each of the 71 trips to the launch spot of the same cell:

| Route source | Start | Dash, median | Dash minus drivers, trip by trip (25th / median / 75th) | Dash at least as fast | Touched on the way | Verdict |
|---|---|---|---|---|---|---|
| Today's planner | from rest | 1.89 s | -1.01 / -0.11 / +0.54 s | 40 of 71 | 9 trips | fail (contacts) |
| Today's planner | logged velocity | 1.91 s | -1.07 / -0.11 / +0.51 s | 40 of 70, 1 did not arrive | 11 trips | fail (contacts) |
| Grid routes | from rest | 1.92 s | -0.83 / -0.16 / +0.41 s | 40 of 71 | none | pass |
| Grid routes | logged velocity | 1.98 s | -0.86 / -0.14 / +0.36 s | 40 of 71 | none | pass |

How to read it:

- The dash's time is to its arrival at its own launch spot, a median 65 in away. The drivers' time is to within 6 in of wherever they chose to fire, a median 60 in away. The dash's measure is the stricter one.
- The margin is small. The dash is slower than the drivers in 31 of 71 trips. It wins clearly on long trips (100 in and more: 0.84 s sooner on the median, 11 of 13) and is about even below that. It loses where its launch spot is more than 10 in further than where the drivers fired (11 trips, 0.23 s later) and, of the four cells, on the blue rear cell (0.18 s later).
- Two trips begin with the logged pose already overlapping the flower stand the robot was gathering from. They are listed apart and are not contacts of the dash.

**The safety gate.** A start every 12 in at four headings (328 starts), blue alliance, to both launch spots and both park corners, with the believed pose true and then out by 4 in in eight directions:

| Route source | Pose true | Pose out by 4 in, worst of the eight directions | Verdict |
|---|---|---|---|
| Today's planner | Hive Dash: a support from up to 27 starts, a flower stand from up to 40. Park Dash: a flower stand from up to 83, no arrival from 60 | Hive Dash: a support from up to 36, a flower stand from up to 49. Park Dash: a flower stand from up to 97, no arrival from up to 171 | fail, even with the pose true |
| Grid routes | nothing touched, every dash arrives | Hive Dash: a support from up to 24 starts (7 percent), a flower stand from up to 16 (5 percent), every dash arrives. Park Dash: a support from up to 11, a flower stand from up to 11, and no arrival from up to 283 when the pose reads 4 in low in y | fail |

**What the gates say.**

- Today's planner is not a candidate: it touches things with a true pose.
- The grid routes are fast enough and clean with a true pose, and they are not built for the accepted drift. Their tables allow 3.0 in off the route, which is less than the 4 in the pose may be out. The next step of this target is to generate the tables with the obstacles grown by the accepted drift and to run both gates again. Wider clearances make longer routes, and the speed margin is 0.14 s, so the speed gate may then fail. If both cannot pass, the choice is the team's: a dash offered only from the parts of the field where both pass, or a smaller accepted drift for dashes with the reseed as its condition.
- The Park Dash needs an arrival rule that holds when the pose is out: with the pose reading 4 in low it pushes toward a corner it cannot reach and never reports arrival.
- The red alliance was not surveyed. The simulator has that option; the field is symmetric, the logged trips are not.
