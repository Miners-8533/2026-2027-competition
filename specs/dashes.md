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

A straight leg is acceptable when the area the robot sweeps along it touches no obstacle. The swept area is a rectangle laid along the leg.

| Leg | Half width of the rectangle | It reaches past each end by |
|---|---|---|
| **Long** (at least the align length): driven with the robot aligned to it, front or rear leading | half the robot's width + the margin | the longer of origin-to-front and origin-to-back + the margin |
| **Short**: not aligned | the far-corner radius + the margin | the far-corner radius + the margin |

The virtual lane walls are rectangles too: each fills the space from the lane half width out to the inner edge of a bar, along the bars' length.

**Exemptions** that keep real situations plannable:

- **A short leg from the robot** is boxed from the start point onward, with nothing behind it: behind the start the robot is only moving away. The robot's own circle at the start (far-corner radius, no margin) is then checked on its own, against the supports and the flower stands. A long leg from the robot keeps its reach behind the start.
- **A robot that starts inside the box between the two bars** may always leave: its first leg ignores the virtual walls, for the whole leg.
- **Any other leg from the robot** checks the virtual walls only from 10 in along the leg onward; a leg of 10 in or less ignores them. So a robot at the lane's mouth can leave or turn away, while one heading into the lane must be centered.
- **A flower just gathered from** is ignored on a leg from the robot, when the stand is within the far-corner radius plus the margin of the start and the leg ends farther from it than it starts.
- **A leg that ends at the goal** is judged against the flower stands differently: the rectangle ends at the goal and does not reach past it, and the robot at the goal is the far-corner circle with no margin. (The audience launch spot is only 16.8 in from a flower stand's footprint, 4.8 in more than the far-corner radius; a rectangle reaching past the spot would make it unreachable once the stand is grown by its clearance. The code's own note says 6.8 in, which the dimensions do not give.) Against the supports and the lane walls the ordinary rectangle applies.

### 1.3 Choosing a route

1. The candidates are the straight line from the robot to the goal, when it is acceptable, and every chain of at most 3 different route points (the four helper corners and the two lane gates) over acceptable legs.
2. The cheapest candidate is the route. **The straight line is not preferred**: it is one candidate among the others, and it wins a tie because it is tried first.
3. If there is no candidate, the plan is a refusal with a reason.

A route point that lies within the far-corner radius plus the margin of an obstacle is not used. None of the six does with the values of section 1.4.

**Cost of a route** = the sum over its legs, in order from the robot:

| Leg | Cost |
|---|---|
| Short (not aligned) | its length divided by the unaligned speed ratio. No turn is charged, and the direction of travel is forgotten |
| Long (aligned), the first leg of the route | its length + the turn penalty for each degree between **the robot's heading when the route is planned** and the leg |
| Long, right after another long leg | its length + the turn penalty for each degree between the two legs |
| Long, right after a short leg | its length. No turn is charged |

A leg can be driven front or rear first, so every turn is folded to between 0 and 90 deg.

**Known limit (found 2026-11-17 by comparing this planner with the one rebuilt from these pages).** The last row makes the turn onto a long leg free when a short leg comes before it, although the robot still has to make that turn. So a short hop to a nearby route point can cost less than turning onto the straight line, and the planner then takes the route point although the straight line is acceptable. On a 6 in grid of starts and the eight goals of the dashes this happens on 1.5 to 3.8 % of the routed trips, depending on the start heading. Those routes are about 1 in longer (4.4 in at most) and have one more point to pass. Not changed: the grid routes of section 6 are to replace this planner.

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
- **The drift margin** (built 2026-11-17, not run on the robot). The pose the robot believes may be out ([aiming.md](aiming.md) section 5), so the trees are built with every support and flower stand grown by 3.0 in and the walls moved in by 3.0 in. This is on top of the 1.5 in margin and the tracking allowance the routes already keep. The footprint at the goal itself is checked against the obstacles and walls as they are: the goal is where the team put it, and a park corner is against the wall. The margin is part of the model text, so tables built with another margin are reported.
- **Layout of the generated file.** It is a class named `GridTables` with the fields `GEOMETRY`, `MODEL`, `CORNER_SPEED`, `NAMES`, `GOAL_X`, `GOAL_Y`, `AIM`, `NEXT`, `COST` and `SPEED`. It holds:
  - a *geometry* text, `pitch 6.0 in, half span 66.0 in, 16 directions`, and a *model* text listing the motion values the tables were built with, `V_MAX 66.0, A_DEC 120.0, A_GRIP 150.0, E_MAX 3.00, E_GOAL 0.50, E_TAPER 24.0, CORNER_CLUSTER_IN 30.0, CORNER_SAFETY 0.75, DRIFT_IN 3.00` (top speed, planning deceleration, grip, the tracking allowance on the route and at the goal and the distance over which it tapers, the corner cluster length, the corner safety factor, the drift margin);
  - the corner speed table: `CORNER_SPEED` holds the eleven speed limits alone, in the order of the angles listed under the table below. The angles are in the code that reads the file, not in the file;
  - for each of the four goals, named `BLUE_REAR`, `BLUE_AUDIENCE`, `LEFT` and `RIGHT`: its x and y, and the heading at the goal in radians (a cell's aim; not a number for a park corner);
  - for each goal three packed arrays with one entry per state, each stored as Base64 text: the next state (2 bytes; 0xFFFF = none), the time to the goal in milliseconds (2 bytes; 0xFFFF = unreachable), and the speed at the end of the leg in half inches per second (1 byte, rounded down). Two-byte values have the high byte first.
- **Numbering of the states.** A node (i, j), with i and j from 0 to 22, is at x = -66 + 6 i, y = -66 + 6 j, and has the number 23 i + j. A state's number is 17 x node + direction: 8,993 states per goal. Directions 0 to 15 are the lattice steps (1, 0), (2, 1), (1, 1), (1, 2), (0, 1), (-1, 2), (-1, 1), (-2, 1), (-1, 0), (-2, -1), (-1, -1), (-1, -2), (0, -1), (1, -2), (1, -1), (2, -1), in that order. Direction 16 is the final state: straight from the node to the goal, at most 30 in.
- A table whose geometry text differs from the code's is not used. A model text that differs is reported in the log and the tables are still used.
- A goal has a table when a table's goal is within 1 in of it. Farther than that, the goal has moved.
- If a table's lattice shape does not match the code, or a goal has moved (a launch spot or park corner changed on the Dashboard), the dash logs why and falls back to the planner of section 1.

| Corner angle (deg) | 0 | 10 | 20 | 30 | 45 | 60 and more |
|---|---|---|---|---|---|---|
| Speed limit (in/s) | 66 | 60 | 44 | 36 | 24 | 20 |

The table has an entry for 0, 10, 20, 30, 45, 60, 75, 90, 120, 150 and 180 deg, and the limit between two entries is interpolated. These are the values in the generated tables in the code (calibrated 2026-11-07; the tables were generated again on 2026-11-17 with the drift margin, and the calibration gave the same speeds). `design/dash-planner-grid.md` gives other figures for that calibration; the code's are the ones in force.

### 5.2 Picking a start

The cheapest start state among the lattice nodes within 9 in of the robot (18 in if none serves), counting the hop to the node, the body turn and the time from there to the goal. The hop and the turn onto the first leg are checked against the supports and flowers, and both keep the drift margin: the hop stays 3.0 in clear, and the turn 1.5 in more than that. A robot that already stands nearer than the margin to something (it has just gathered at a flower stand) keeps the clearance it has, to the half inch: it may leave, it may not come nearer.

**Move clear first** (built 2026-11-17, not run on the robot). A grid dash is no longer refused because the robot stands too close to something, or because its pose reads as overlapping a hive support (which only a pose that is out can do). When no start serves from where the robot stands, or the pose overlaps a support, the route begins with a **move-clear leg**:

- It is a straight slide with the heading held. The robot does not turn, because a turn swings its corners.
- The slide is searched in 16 directions, in steps of 3 in, up to 30 in. In each direction the shortest slide that ends where a normal start serves is a candidate, and the cheapest in time (the slide at 20 in/s plus the route from its end) is taken.
- **It may not come nearer to anything than the robot already is.** A support or flower stand the robot is clear of keeps the clearance it has, as on a hop. One the pose overlaps must be left: the robot's center gets no nearer to it at any step, and the slide ends the drift margin clear of it. No corner of the robot goes further toward a wall than it is, and the slide ends the drift margin inside the walls.
- The leg is driven no faster than 20 in/s. From its end the route is a normal one: the hop, the checked turn, the tree.
- The dash's start line in the log says `MOVES CLEAR n in first (heading held), then` and the route.
- The dash is refused only when no direction serves within 30 in. The same pick serves a new route after the off-route guard.
- With grid routes off, the rules of sections 1 and 2 stand, the refusal for touching a support included.

### 5.3 Following

- Each leg is driven aligned, front or rear first, at a target speed limited by the top speed and by what lets the robot brake in time for every corner limit ahead.
- A feedforward from speed to power, plus static friction terms; zero power (the motors brake) whenever the robot is more than 3 in/s too fast.
- A sideways correction with damping.
- **A cell's last leg is driven already turned to the aim**, sideways if need be; the target lock takes over only within 20 in.
- **The last leg stops on a speed profile** and arrival counts only below 10 in/s.
- **Inside the lock's 20 in, the drive power is scaled by the cosine of the heading error to the aim**, not below 0.2 (built 2026-11-17). A robot still turned away from its aim goes slowly until the lock has turned it, so it does not spin at the launch spot.
- **After arrival a grid Hive Dash brakes against the speed it still has** (built 2026-11-17): drive power of 0.04 per in/s against the velocity, at most 0.35, until the robot is slower than 1.5 in/s. With no drive power and the lock turning it, the robot slid on about 2 in.
- **A park corner also counts as reached when the robot has stopped against the wall** (built 2026-11-17): within the goal tolerance plus the drift margin, slower than 1.0 in/s, after it has moved on this route. Without this a robot whose pose read short of the corner pushed at the wall and never arrived.

### 5.4 Off-route guard

If the robot is more than 6 in from the leg it is driving, it brakes. Once it is slower than 6 in/s, or after 1 s, a new route is picked from where it is, at most 2 times, after which the dash cancels.

### 5.5 Parameters

| Name | Value |
|---|---|
| Lattice pitch / half span / directions | 6 in / 66 in / 16 |
| Pick radius / fallback radius | 9 in / 18 in |
| Top speed / planning deceleration | 66 in/s / 120 in/s² |
| Tracking allowance: on the route / at the goal | 3.0 in / 0.5 in |
| Drift margin (obstacles grown, walls moved in, start hop) | 3.0 in |
| Park corner: stopped against the wall below | 1.0 in/s |
| Move clear: longest slide / search step / directions / most speed | 30 in / 3 in / 16 / 20 in/s |
| Aim scaling floor inside the lock radius | 0.2 |
| Arrival brake: power per in/s / limit / off below | 0.04 / 0.35 / 1.5 in/s |
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
- **Both gates of target T9 were run in simulation on 2026-11-17** (section 6.2). Today's planner fails both. The grid routes as first built passed the speed gate by a small margin and failed the safety gate. With the drift margin, a gentler last approach and a brake on arrival, all added the same day, **the grid routes pass the safety gate**: nothing is touched with the pose out by 4 in, and with the move-clear leg no dash is refused. They are then 0.26 s slower than the drivers on the median trip to a launch spot, so the speed gate fails.

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

**Waits on** the time (section 6.2: the goal the dash drives to, and the speed of the last approach), then the grid routes' first run on the robot. The drivers' baseline, both gates, tables that keep the drift clear, a stop that does not slide and the move-clear leg exist (2026-11-17), in simulation only.

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

**What the first run said.** Today's planner is not a candidate: it touches things with a true pose. The grid routes as first built were fast enough and clean with a true pose, and were not built for the accepted drift: they kept 4.5 in on a straight and less in a corner or on the hop from the robot to its first point, against a pose that may be out by 4 in. The Park Dash pushed toward a corner it could not reach and never reported arrival.

**The same gates with the drift margin** (sections 5.1 to 5.3; the tables generated again the same day):

| Gate | Result |
|---|---|
| Safety, pose true | nothing touched, every dash arrives |
| Safety, pose out by 4 in, eight directions | no support touched and no wall, by either dash. The Park Dash touches nothing and arrives from every start. The Hive Dash to the rear spot touches nothing. **The Hive Dash to the audience spot touches the audience flower stand from 16 of 328 starts in one direction** (the pose reading 2.8 in low in both x and y), by at most 0.7 in |
| Speed, logged velocity | dash median 2.40 s; dash minus drivers -0.48 / +0.14 / +0.70 s; at least as fast in 31 of 70; nothing touched; one start in a corner of the field refused |
| Speed, from rest | dash median 2.39 s; dash minus drivers -0.61 / +0.15 / +0.76 s; at least as fast in 32 of 70 |

So both gates still read "fail", for new and smaller reasons:

- **The one contact left is the stop, not the route.** The robot counts as arrived at up to 10 in/s within the goal tolerance and then coasts: in the traced case it came to rest 3.5 in past the launch spot, still turning to the aim. With the pose out by 4 in toward the flower stand, that slide reaches it. No route table can fix this.
- **The margin costs time.** The median trip went from 0.14 s sooner than the drivers to 0.14 s later. The dash still wins on trips of 100 in and more (0.33 s sooner). It loses most where its launch spot is more than 10 in further than where the drivers fired (10 trips, 0.60 s later). Counted from pickup to the salvo (the drivers took a median 0.31 s to aim after arriving; the dash is aligned about 0.2 s after arrival in the traced dashes), the dash is 0.13 s sooner on the median trip and sooner in 40 of 70.
- **Smaller margins were tried.** With 2.0 in a foot bar is still grazed from one start. Each of the three parts of the margin (supports, flower stands, walls) costs time on its own: the lattice is 6 in, so a route that no longer fits moves a whole column over.
- **Two quick changes to the stop were tried in the simulator and neither is kept.** Counting arrival only below 5 in/s cut the flower contacts from 16 starts to 1 and cost 0.27 s on the median trip. A faster last leg (55 in/s, braking planned at 130 in/s²) was slower still and touched the flower stand from 37 starts. The last leg does not hold its profile while the robot turns to the aim.

**The same gates with the stop and the move-clear leg** (sections 5.2 and 5.3; the tables are those of the drift margin):

| Gate | Result |
|---|---|
| Safety, pose true and out by 4 in in eight directions | **pass.** Nothing touched by either dash from any of the 328 starts in any of the nine surveys; every dash arrives |
| Refusals in those nine surveys (11,808 dashes) | **none** (176 before). All 176 were poses that read as overlapping a hive support. Each now moves clear first, touches nothing and arrives |
| Speed, logged velocity | **fail.** Dash median 2.54 s; dash minus drivers -0.39 / +0.26 / +0.72 s; at least as fast in 30 of 71; nothing touched; no trip refused (the corner start refused before now moves clear and arrives) |
| Speed, from rest | fail. Dash median 2.51 s; dash minus drivers -0.44 / +0.28 / +0.77 s; at least as fast in 30 of 71 |

- **The safety pass is narrow.** With half the arrival brake, or an aim scaling floor of 0.5, the audience flower stand is still grazed from 1 or 2 starts. The brake works on the Pinpoint's velocity at low speed, which the simulator models only roughly: watch the first robot runs for a buzz or a rocking stop.
- **What the stop cost.** The gentler last approach added about 0.1 s to the median trip. Counted from pickup to the salvo, as in the note above, the dash and the drivers are now even on the median trip (0.04 s to the dash) and the dash is sooner in 39 of 71.
- **Where the dash still wins and loses.** It is 0.33 s sooner on trips of 100 in and more. It is 0.41 s later where its launch spot is more than 10 in further than where the drivers fired (11 trips).
- The survey starts no nearer than 1 in to anything with a true pose, so the move-clear leg was exercised only by poses shifted into a support, and by the one logged trip that began in a corner of the field. Starts hard against a flower stand or a wall were not surveyed.

**What is next for this target.** The time. The dash averages 26 in/s over a trip on a robot the follower allows 66 in/s, and it drives to one fixed spot per cell while the drivers fire from nearer. The two levers are a goal chosen near where the drivers fire from, which needs the launch grid tuned on the robot, and a faster last approach that still stops, which the two quick trials above show is not a matter of raising two numbers. The dashes stay switched off until both gates pass and the grid routes have had their first robot runs.

- The red alliance was not surveyed. The simulator has that option; the field is symmetric, the logged trips are not.
