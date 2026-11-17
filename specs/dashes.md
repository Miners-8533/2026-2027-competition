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
- A robot that starts inside the lane may always leave: its first leg ignores the virtual walls. The walls are only checked from a short distance along a leg that starts at the robot.
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
- If a table's lattice shape does not match the code, or a goal has moved (a launch spot or park corner changed on the Dashboard), the dash logs why and falls back to the planner of section 1.

| Corner angle (deg) | 0 | 10 | 20 | 30 | 45 | 60 and more |
|---|---|---|---|---|---|---|
| Speed limit (in/s) | 66 | 60 | 44 | 36 | 24 | 20 |

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

- Hive Dash and Park Dash with the planner of section 1 were used at Hartland.
- In simulation on a model of the robot as it now drives (faster than when the corner and approach values were set), the present dashes touch a flower stand or a support from a noticeable share of starting positions, and the Park Dash, having no corner slow-down, is the worse of the two. The grid routes touch nothing in the same simulation. This is the reason the grid routes exist, and they still need their first robot run.
