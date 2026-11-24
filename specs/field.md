# Field model and rules the code depends on

The software hard-codes a model of the BIOBUZZ field. This page is that model: the coordinate frame, every position the code uses, and the game rules that shape behavior. It is not a copy of the game manual; the rules themselves live in the team's BIOBUZZ knowledge repo.

## 1. Field frame

| Item | Convention |
|---|---|
| Origin | the center of the field |
| +X | toward the **audience wall** |
| -X | toward the **rear wall** |
| +Y | toward the **Blue Wall** |
| -Y | toward the **Red Wall** |
| Heading | 0 deg = facing +X; positive is counterclockwise seen from above |
| Units | inches and radians in the code; inches and degrees in logs and on screens |

**Red is blue rotated 180 degrees about the origin.** Every path, pose and zone is defined for blue and obtained for red by negating x and y and adding 180 deg to headings. Nothing is mirrored.

**Walls** are at ±71 in on both axes. The team measured the field at about 142 in between inside wall faces (the nominal 144 would give 72). This is one constant, read when an opmode starts, and is an assumption to re-check at each venue.

**Sides.** Blue's half is y > 0, red's is y < 0. The dividing line runs through the hive.

## 2. The hive

Two hive cells per alliance are aimed at. Each is a point the launcher's line is turned onto.

| Cell | Point (x, y) |
|---|---|
| Blue rear | (-21.5, 12.75) |
| Blue audience | (21.5, 12.75) |
| Red rear | (-21.5, -12.75) |
| Red audience | (21.5, -12.75) |

- "Rear" and "audience" name the side of the hive the cell opens toward. A cell can only be scored from its open side.
- Launch distance is always measured from the robot's origin to the cell point.
- The **far cell** for an alliance is the one on the opposite side of the field from its usual start wall: the audience cell for blue, the rear cell for red.

**Hive supports** (obstacles for driving), from the field CAD, symmetric in both signs:

| Obstacle | Geometry |
|---|---|
| Two foot bars | x from -19.47 to 19.47, \|y\| from 22.75 to 24.73, 2.15 in tall; the chassis cannot cross them |
| Four A-frame legs | each leans inward. Foot at (\|x\| 18.56, \|y\| 23.81) at height 0.53 in; at height H the leg is at \|x\| = 18.56 - 0.445 (H - 0.53), \|y\| = 23.81 - 0.273 (H - 0.53). For a robot 18 in tall each leg is a line 1.14 in thick from its foot to (10.78, 19.04) |
| The lane under the hive | the strip between the two bars, along y = 0. A robot driving along it clears the legs by 11.6 in |

## 3. Flowers, gardens and the loading zone

**Flower stands** (four, one on each wall). Opening center 68.45 in from the field center along the wall's normal and 23.4 in to one side of the wall's midpoint. Each is an obstacle 5.8 in along the wall by 4.8 in across, **centered on the opening center** and not flush against the wall: the audience-wall stand reaches from x = 66.05 to 70.85 and from y = 20.5 to 26.3. Nothing is passable underneath.

| Flower | Opening center |
|---|---|
| Audience wall | (68.45, 23.4) |
| Blue Wall | (-23.4, 68.45) |
| Rear wall | (-68.45, -23.4) |
| Red Wall | (23.4, -68.45) |

Blue's two flowers are the audience-wall and Blue Wall ones.

**Blue garden:** against the rear wall, y from about 47 to 70.

**Blue loading zone** (where a robot parks): x from 23.9 to 46.6, y from 59.1 to 70.1, against the Blue Wall. Only part of the robot has to be inside to score.

## 4. Positions the software drives to

All in the blue frame; red by the rotation.

**Start poses** (robot origin; the robot's back is flush against the wall and its origin is 7.6 in from it).

| Start | Pose (x, y, heading) | Used by |
|---|---|---|
| Rear wall | (-63.4, 16.0, 0 deg) | the TeleOp default, Solo, Daryl, Cybugs, Garden Side, Primary |
| Audience wall | (63.4, 7.5, 180 deg) | Flower Side |

The default TeleOp start for red is (63.4, -16.0, 180 deg).

**Launch spots** (also the centers of the auto-launch zones).

| Cell | Spot (x, y) |
|---|---|
| Blue rear | (-55.4, 15.25) |
| Blue audience | (55.4, 7.5) |
| Red audience | (55.4, -15.25) |
| Red rear | (-55.4, -7.5) |

Each is about 34 in from its cell, straight out along the cell's axis, and 8 in clear of the wall behind the robot. The y values differ from the cell's 12.75 because the launcher's line sits to the right of the robot's centerline.

**Park corners** (TeleOp Park Dash): (40, 60) on the audience side and (26, 60) on the hive side. The shown autos park at (22, 55.1) on the hive side, and Flower Side at (40, 55.1) (2026-11-20 and 21; before, (26, 53.1) and (40, 53.1)). Each program's park pose and heading is in [autos.md](autos.md).

**Route points** used by the route planner: helper corners at (±36, ±40) and lane gates at (±34, 0).

**Auto launch spots away from the walls:** (50, 26.3) for the audience cell and (-50, 26.3) for the rear cell, on the line of the flower pickups.

## 5. Driver orientation

Each alliance's drivers stand at their own wall, so "stick up" must mean a different field direction for each.

| Alliance | Driver looks toward | Stick up drives toward |
|---|---|---|
| Blue | -Y | -Y |
| Red | +Y | +Y |

The driver's left is therefore +X (the audience wall) for blue and -X (the rear wall) for red. Features described as "the cell on your left" use this.

## 6. Game rules the code is built around

Rule numbers are from the BIOBUZZ competition manual.

| Rule | What it requires | Where it shows in the software |
|---|---|---|
| Match timing | 30 s autonomous, 8 s transition, 2 min driver-controlled | auto time budgets and deadlines |
| G304 | start touching the wall, on your own side, not in the loading zone or a flower's volume, motionless, with exactly 4 pollen preloaded | start poses; the preload of 4 |
| G401 | nobody touches the robot or controls during autonomous | autos take all choices at INIT |
| G402 | in autonomous each alliance has priority on its own side | every auto path stays on its own half |
| G403 | no powered movement of any mechanism during the transition | the TeleOp commands nothing in INIT: the ramp and the flywheel position servo are first commanded at Start |
| G407 | at most 4 elements controlled at once | the capacity of 4; FULL; the pusher's HOLD position |
| G408 | do not control the opponent's nectar | the opponent-nectar warning; reject |
| G417 | only the upward-facing cell may be launched into; do not launch into a hive that is tipping | the duo autos' absolute-time waits |
| G418 | only pollen may be taken from a flower, from its bottom | the ramp |
| R102 | fit an 18 in cube at the start | the ramp starts stowed |

**Scoring facts the autos rely on.** Leaving the wall (LEAVE) scores 3 and parking (PARK) scores 5 in autonomous. A hive tip scores 20. The upward cell starts with 3 nectar, so 3 more elements tip it; the cell that then faces up is empty and needs 8. A tip is taken to last about 4 s.

## 7. Known inconsistency, and target T7: one field model

**As built.** The flower stands have one definition, the CAD values above: the ramp's "clear of a flower" check reads the route planner's (from 2026-11-17; before that it had its own copy from the manual's figures, (±68.6, ±24.0)). That check is not used by any competition opmode.

Field data is also spread out, and some of it is repeated:

| Data | Defined in |
|---|---|
| Hive cell points | with the mechanisms, though the aiming and dash code are their main users |
| Flower stands | the route planner; the ramp reads them from there |
| Helper point (36, 40) and its mirror | the Hive Dash, the Solo auto, and twice in the duo autos |
| Hive-side park pose (22, 55.1); (26, 53.1) until 2026-11-20 | the Solo family and the duo autos, each with its own copy |
| Launch spots | the auto-launch zones hold the TeleOp's; each auto holds its own |
| Auto poses | the autos, and a hand-kept copy in the duo path check on the PC ([tools.md](tools.md)) |
| Robot footprint | the chassis profile, and a hand-kept copy in the duo path check |

**Target T7** (not in the code; see "As built and target" in the [README](README.md)).

*Requirement.* Every field position and obstacle is defined once. The robot code and the PC checks read the same definition.

*Design.*

- One **field model**, pure data with no robot dependencies, holds sections 1 to 4 of this page: the frame and walls, the hive cells and supports, the flower stands, the garden and loading zone, the start poses, the launch spots, the park corners and the route points. It also holds the rotation for red.
- A position used by more than one feature has one name and one value. Where two features need different points on purpose, both are named and the reason is stated: the Park Dash corner and the autos' park pose differ by 6.9 in because the autos stop with the front edge 8 in from the wall.
- Each auto's poses are data in a form the PC checks compile directly. The duo path check keeps no copies, of poses or of the footprint.
- Positions stay tunable values where they are today, so a spot can still be moved from the Dashboard. The grid route tables already refuse to run when a goal they were built for has moved; that rule covers every generated table.

*What would behave differently.* Nothing. (The ramp's unused flower check moved by under an inch when it took the planner's values.)
