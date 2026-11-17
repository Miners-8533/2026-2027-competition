# Aiming: target lock, auto-launch and the launch grid

How the robot points its launcher at a hive cell and decides when a launch may go. Launch speed is in [flywheel.md](flywheel.md); the field points are in [field.md](field.md).

## 1. Aim heading

The launcher's line is parallel to the robot's forward axis but offset sideways from the origin. To hit the cell, that line, not the robot's centerline, must pass through the cell point.

> aim heading = bearing to the cell - asin(offset / distance) + angle offset

- **bearing** is the direction from the robot's origin to the cell point;
- **offset** is the launcher's sideways position, positive to the robot's left, plus any extra for this cell. The ratio offset / distance is limited to ±0.9;
- **angle offset** corrects a launcher whose line is not exactly forward (0 at present).

**Far-cell extra.** An additional sideways offset is applied when aiming at the alliance's far cell (the audience cell for blue, the rear cell for red). It compensates for odometry drift that grows with distance from the start wall. It is a stopgap, not a property of the launcher. Target T1 (section 5) keeps it as the one starting allowance for the operator's trims.

| Name | Value |
|---|---|
| Launcher offset | -2.5 in (2.5 in to the right) |
| Far-cell extra | -3.0 in |
| Angle offset | 0 deg |

## 2. Target lock

**Purpose.** While the lock is on, the robot turns itself onto a cell and the driver keeps full control of translation. The lock's turn command replaces the driver's turn stick.

**Inputs each loop:** the pose; the measured turn rate; the cell; the far-cell extra for it; a remembered aim correction for that cell in degrees; the driver's turn stick; the loop time; optionally a launch grid residual.

**Law.**

1. If the robot is closer than 12 in to the cell, the bearing is meaningless: command no turn and report "not aligned".
2. Aim = the aim heading of section 1 + the driver's nudge + the remembered correction. (With a launch grid residual: aim = bearing + residual + nudge + remembered correction.)
3. Error = aim - heading, the short way round.
4. Inside the dead band, command 0.
5. Otherwise: turn power = kP x error - kD x measured turn rate, plus a small static power in the same direction, limited to a maximum.

**Aligned** is true when the error is within the aligned angle **and** the robot is turning slower than the aligned rate.

**Driver's nudge.** The turn stick moves a trim at a fixed rate, up to a limit either way (stick right is clockwise). The trim is forgotten when the lock is released.

**Remembered correction.** A standing correction per cell, supplied by the caller. In the TeleOp the operator builds it from reports of where the last salvo landed ([teleop.md](teleop.md)).

**Release** clears the nudge and the readouts.

| Name | Value |
|---|---|
| kP (per radian) | 1.2 |
| kD (per rad/s) | 0.08 |
| Static power | 0.05 |
| Maximum turn power | 0.6 |
| Dead band | 1.0 deg |
| Aligned: within / turning slower than | 2.0 deg / 25 deg/s |
| Nudge rate / limit | 40 deg/s / 15 deg |
| Minimum distance to the cell | 12 in |

The autos use the same law through *aim at a cell* ([motion.md](motion.md)), with a tighter aligned angle and a breakaway power.

## 3. Auto-launch zones

**Purpose.** A launch that takes itself: when the robot is in a good place, locked and still, a salvo goes without anyone pressing launch.

- Each cell has a **launch zone**: a circle of radius 10 in around its launch spot ([field.md](field.md)).
- The zone answers two questions: how far a point is from its center, and whether a point is inside.
- The spots keep the launch roughly straight on to the cell (within about 14 deg) and in the flat part of the launch map (about 34 in out).

The behavior built on the zones is the TeleOp's ([teleop.md](teleop.md)): inside the zone of the locked cell the chassis is held; once the lock is aligned and the robot has stopped, one salvo is launched; if that has not happened within a time limit, the driver gets the robot back.

| Name | Value |
|---|---|
| Zone radius | 10 in |
| Stopped below | 4 in/s |
| Aligned and stopped for | 0.15 s |
| Chassis held at most | 2.5 s |

## 4. Launch grid (two-dimensional map, off by default)

**Purpose.** Replace "speed from distance, aim from a formula" with measured values per field position, because the best aim and speed vary with where the robot stands, not only how far it is.

**Status:** built after Hartland from one tuning run for one cell; never used on the robot. It is off unless the operator switches it on in the TeleOp. The autos never use it.

### 4.1 Data

For each measured spot: where the robot actually stood, the flywheel speed that scored, the **aim residual** (the heading that scored minus the bearing to the cell's center), and whether a shot was possible at all.

- Only the blue rear cell was measured.
- The blue audience cell is a **seed**: the measured spots mirrored about x = 0, keeping the ball's path.
- The red cells are the blue data rotated 180 degrees.
- Spots the tuner would have visited but nobody measured (near the walls) are **estimated** from their nearest measured neighbours and flagged.
- The table is generated source, produced by a PC tool from the tuning run's log. It must not be edited by hand.

Because the residual is relative to the bearing, the grid's aim does not depend on the launcher offset or the far-cell extra of section 1; they are already in the data.

### 4.2 Lookup

Given a cell and a position, the answer is a speed, a residual and a **zone**:

| Zone | Meaning | Values |
|---|---|---|
| Interpolated | inside a triangle of measured spots | a linear blend of the three; a no-shot spot does not contribute |
| Extrapolated | outside all triangles | the value at the nearest point of the data's outline, at any distance |
| No shot | at a recorded no-shot spot, or nearest to one | still the nearby good spots' values; the zone itself is the warning |
| No data | the cell has no data at all | none |

There is **no fallback to the formula** while the grid is on: wherever the grid has values, they are used.

### 4.3 Use

- The grid supplies the flywheel speed (plus the operator's trim) and the aim residual to the target lock.
- During a Hive Dash it is looked up for the launch spot, not the current pose.
- In backup mode it is not used.
- While the lock is on or a launch is queued in a no-shot zone, the screen warns; the launch still goes.
- Whether the grid is on or off, every launch request logs the grid's answer beside the formula's and the map's, so practice sessions compare them.

### 4.4 Which spots a grid contains

The spot generator (used by the tuning opmode) keeps a lattice point for a cell only when all hold:

1. it is on the cell's open side;
2. it is at least 12 in from the cell;
3. the shot is within 75 deg of the opening's normal;
4. the robot, turned to its aim heading there, overlaps no hive support or flower stand (with the route planner's margin) and fits inside the walls less 3 in.

A point that fails only on a wall is slid toward the field center, up to 8 in. The lattice is centered on the field with a 12 in pitch, giving about 30 spots per cell, visited as a serpentine sweep.

## 5. Target T1: drift is accepted, and allowed for in one place

Not in the code. See "As built and target" in the [README](README.md).

### 5.1 The weakness

The pose drifts as the robot travels away from where it was seeded. Measured with the Launcher Offset Finder on 2026-11-03: about 3 percent of the distance, roughly 4 in sideways after crossing the field, following the start wall.

The code allows for it in four places that do not know of each other:

| Allowance | Where it lives | Value |
|---|---|---|
| Far-cell extra | the aim heading (section 1) | -3.0 in sideways when aiming at the far cell |
| The far cell's launch spot | a typed-in field position ([field.md](field.md)) | y = 7.5, where the launcher offset alone gives 10.25 |
| The autos' aim trims | Solo, Daryl, Cybugs, Primary ([autos.md](autos.md)) | -8.0 deg at the audience launch spot, **added to** the far-cell extra |
| The operator's remembered aim | the TeleOp ([teleop.md](teleop.md)) | 3 deg per report, up to 10 deg either way per cell |

Two of these work against each other. By the formula of section 1, at the autos' audience launch spot the far-cell extra turns the robot about 5.5 deg to the left and the trim turns it 8 deg to the right, a net of about 2.5 deg right. The TeleOp marks asked for an allowance in one direction and the auto launches for the other, which is itself evidence that the drift depends on the path driven.

### 5.2 Decision (team, 2026-11-17)

**Some drift is accepted.** What the drift will be in a match cannot be predicted: it depends on the start, the path, contact with other robots and wheel slip. The pose is not corrected by a model. **The operator's trims are the answer:** they keep the aim on the cell as the pose wanders.

A correction model in the localizer (a skew about the seed point) was considered and rejected for this reason.

Decided the same day:

| Point | Decision |
|---|---|
| The autos' two opposing allowances | the far-cell extra is not applied in the autos; each trim is set to the net value |
| The remembered aims in a TeleOp restart | saved, so a restart in a match keeps them |
| The match screen | shows the aim trim of each of the alliance's cells |

### 5.3 Requirement

1. The pose is used as it is. Nothing tries to predict its drift.
2. In the TeleOp the operator's remembered aim is the compensation. It is a first-class control: one press, logged, and never lost while the drift it corrects is still in the pose.
3. A fixed allowance is only a starting point for the trims. It exists once and other values are worked out from it, not typed again.
4. An auto drives the same path every time, so its drift repeats. Each auto launch carries **one** trim that holds all of its allowance.
5. Everything that uses a position without aiming must work with the accepted drift.

### 5.4 Design

**The aim for a cell** is the sum of four named parts, built in one place and used by the TeleOp's lock, the dashes and the tuners:

| Part | Source | Lives |
|---|---|---|
| Geometry | bearing and the launcher offset (section 1) | fixed |
| Starting allowance | one tunable value per cell: today's far-cell extra for the far cell, 0 for the near cell | until changed between events |
| Remembered aim | the operator's reports | until a reseed |
| Nudge | the driver's turn stick | until the lock is released |

- **The far cell's launch spot is worked out, not stored.** A launch spot is where the robot stands square to the wall with the launcher's line through the cell: the cell's y, shifted by the launcher offset and by the starting allowance. For the blue audience cell that is 12.75 - 2.5 - 3.0 = 7.25. In the code the spot is its own stored number, 7.5, set when the allowance was a little different, and the far-cell extra is another. Change the extra and the lock aims with the new value while the Hive Dash and the auto-launch zone still send the robot to the old spot, where it then has to stand slightly turned. With today's values the two differ by 0.25 in, which is under half a degree and harmless. The target removes the second number so they cannot drift apart: change the allowance and the spot and its zone move with it.
- **The remembered aim is saved with the pose hand-over** ([drive.md](drive.md) section 5). A TeleOp that is restarted in a match adopts the drifted pose; it must get the trims that go with that pose. Today they are lost. A reseed clears both, as now.
- **The match screen shows the aim trim of each of the alliance's two cells**, on one small line beside the speed trim, for example `AIM REAR +3  AUD -6`. The line is shown while either is not zero.
- **Autos.** Each launch has one aim trim, and the starting allowance is not applied in any auto. The trims that are -8.0 deg today become the net value, **-2.5 deg**, so every auto aims where it aims today:

  | Program | Launches | Today | Target |
  |---|---|---|---|
  | Solo | garden and flower pollen at the audience cell, from (50, 26.3) | -8.0 deg, plus the far-cell extra (about +5.5 deg) | -2.5 deg |
  | Daryl, Cybugs, Primary | at the audience cell, from (50, 26.3) | the same | -2.5 deg |
  | Every launch at the cell on the start side | | 0, no extra | 0 |
- **After each event** the log tools report, per match and per cell, the remembered aim the operator ended on. A value that repeats becomes the next starting allowance. This is how the fixed part follows what actually happened without anyone guessing.

**The accepted drift is a number: 4 in.** Features that use a position without aiming are checked against it:

| Feature | Margin today | Against 4 in |
|---|---|---|
| Auto-launch zone | 10 in radius | inside |
| Dash arrival | 2 to 2.5 in tolerance, then the lock aims | inside: the lock and the trims take over |
| Route planner clearance of a flower stand | 3 in | **smaller than the drift**; see target T9 ([dashes.md](dashes.md)) |
| Park corner | part of the robot in the zone; about 4 in inside | at the limit |
| Flower approach in an auto | the ram and the wall take up the error | inside |

The driver's reseed ([teleop.md](teleop.md) section 9) remains the remedy when the pose is further out than that.

### 5.5 What would behave differently

| Item | Change |
|---|---|
| Far cell's launch spot | y = 7.25 from today's values instead of the typed 7.5, and it follows the allowance from then on |
| Autos | none: the same aim (to within 0.01 deg) from one number per launch, -2.5 deg where it was -8.0 |
| A TeleOp restarted in a match | keeps the remembered aims |
| Match screen | one small line with both cells' aim trims while either is not zero |

### 5.6 Waits on

Nothing on the robot. The conversion of the autos' trims is checked with one practice launch per program.
