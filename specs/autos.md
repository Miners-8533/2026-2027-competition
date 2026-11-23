# Autonomous opmodes

Six 30-second programs. All are built from the same pieces: drive legs, aim, launch, ramp moves, the garden pickup, the flower gather, and a park. This page specifies the pieces once, then each program as a sequence of them.

Building blocks: [motion.md](motion.md), [mechanisms.md](mechanisms.md), [aiming.md](aiming.md). Positions and rules: [field.md](field.md).

| Program | Start wall | Shown on the Driver Station | Intended use |
|---|---|---|---|
| Auton - Solo | rear | yes | the partner only parks; since 2026-11-19 the preload takes only its flywheel speed from the 2-D launch grid (`PRELOAD_GRID_SPEED`: 1394 ticks/s at the start wall, extrapolated, against the map's 1350; the position and the aim are unchanged); every salvo runs from the LAUNCH-READY pusher position (`Mechanisms.requestLaunchReady`, requested on the last leg to each launch spot, forward only to the launch-ready angle, 141 deg, dropped when the salvo starts); the launch-spot legs end within 3 in and 12 deg (the grid's speed and aim are looked up at the pose the robot is really at; not run); since 2026-11-19 there is no squaring after the garden since 2026-11-19 (`GARDEN_SQUARING` false); the audience launch spot, one for all three salvos, is blue (56, 36) / red (-56, -36) (8 in toward the flower wall from (48, 36), 2026-11-19; a first move to (44, 36) went the wrong way for red) with the 2-D launch grid's speed and aim; also from 2026-11-19 (user) the garden in one leg from the start to the end of the angled pickup (last 12.5 in at 0.5 power) and a THIRD garden salvo of the wall flower's pollen into the same cell from that same spot (56, 36), then the park is tried even if time runs out (not run since) |
| Auton - Boomerang | rear | yes | (2026-11-21; ran in five playoff matches at the Novi Qualifier with the last salvo from blue (-55.6, 26.2): it finished and parked every time, in 26.9 to 28.5 s) Solo up to the Blue Wall flower's back-out; then the last salvo into the first (preload's) cell from blue (-57.6, 26.2) (its own spot from the evening of 2026-11-21, 2 in nearer the wall than the Triple Tip's (-55.6, 26.2); not run) instead of into the second cell from (56, 36), and the park through (-20, 44) (section 3) |
| Auton - Triple Tip | rear | yes | (2026-11-20, built, not run) Solo up to the first flower gather, the partner launches with our garden salvo; then around the outside of the hive to the first cell's side, two salvos into the first cell from blue (-55.6, 26.2) with the alliance-wall flower between them, and the park through (-20, 44) |
| Auton - Blaser's Gambit | audience | yes | (2026-11-21; run three times that day outside a match, none finished its actions) a triple tip from the audience wall: the preload after a 7 s wait and the audience flower into the audience cell, the Blue Wall flower and the garden into the rear cell from the Triple Tip's launch spot; the salvos come before the park, which is tried last |
| Auton - Duo Daryl | rear | yes | garden first; since 2026-11-20 the garden salvo launches from blue (56, 36) with the 2-D grid's speed and aim, every salvo from the launch-ready pusher position, the preload takes the grid's flywheel speed, and it parks at (22, 55.1); since 2026-11-21 the Blue Wall flower's pollen go into the original (preload's) cell from blue (-55.6, 26.2) and the partner wait, the second audience-cell salvo and the audience flower are gone (the version of 2026-11-20 ran once at the Novi Qualifier, as practice; the new finish ran three times that afternoon outside a match and finished once, in 27.0 s) |
| Auton - Cybugs | rear | hidden | Daryl without the last flower; ends about 5 s early |
| Auton - Duo Garden Side | rear | yes (from 2026-11-21) | one half of a matched pair; since 2026-11-21 with the changes of the other autos and its last two salvos as late as the park allows (section 6), built, never run |
| Auton - Duo Flower Side | audience | yes (from 2026-11-21) | the other half of the pair; since 2026-11-21 with the Solo and Daryl changes (section 6), built, never run |
| Auton - Duo Primary | rear | hidden | the longest program; does not fit 30 s with every step |

A hidden program is complete code that is disabled from the Driver Station list. Since 2026-11-19 (the user) only Solo and Daryl are shown; Triple Tip was added 2026-11-20, Flower Side, Blaser's Gambit, Garden Side and Boomerang 2026-11-21.

## 1. Common frame

### 1.1 Alliance

Every path is written for blue. Red is the same path rotated 180 degrees about the field center: (x, y, heading) becomes (-x, -y, heading + 180 deg). Cells are swapped the same way: blue rear corresponds to red audience, blue audience to red rear.

### 1.2 INIT

- Build the drive with *reset and seed* at the start pose (the robot must be in place and still) and the mechanisms with the pusher zero *assumed at HOLD*. The ramp is commanded stowed and the flywheel position to POLLEN.
- Gamepad 1 D-pad right / left chooses BLUE / RED; a change re-seeds the pose.
- Nothing moves. The slot count is read.
- The INIT screen shows where to place the robot, whether the pose is OK (seed confirmed, and within 1.5 in and 3 deg of the start), which mechanisms were found, and how many elements the slot count sees (4 are expected).

### 1.3 At Start

- The intake is set **always in** for the whole program.
- The flywheel runs from Start and stays up between launches. Its speed follows the distance to the current target cell, so it is at speed on arrival at each launch spot.
- Pusher jam retries are set to 1.
- The remembered aims an earlier TeleOp saved are dropped: the auto has reseeded the Pinpoint and they belonged to the old pose.
- The CS5 held count starts at 4, the preload ([element-counting.md](element-counting.md) section 2.5). The preload's kinds are not known to the load order (`????`, section 2.8 there).
- The pusher is moved to the lock angle while the flywheel spins up, so the preload salvo is shorter.
- The program is one sequential action; the mechanisms update is the every-loop work.

### 1.4 At the end

- Whether it finished or was stopped, the final pose and alliance are saved for the TeleOp.
- The CS5 held count is saved on every loop of the program and after it, with its load order, so the TeleOp starts with what the last gather left aboard.
- Until the opmode is stopped, the pose keeps being read and saved and the mechanisms keep updating.
- The flywheel is stopped as the last step of the sequence.
- When the opmode is stopped: the drive is stopped, then all mechanisms at once, with no update after it (built 2026-11-17; in every auto at the Novi Qualifier).

### 1.5 Logging

Kind AUTO. Each step writes an `AUTO TIMING` line with its duration and its start and end in seconds since Start; a final line lists every step. Each leg logs its start and whether it `settled` or `TIMED OUT`. Waits log `AUTO WAIT`, skipped steps `AUTO SKIP`. Sample rows carry the pose, the last drive command, the coordinator's columns and the wheel currents.

## 2. Pieces

### 2.1 Drive legs

All are *drive to pose* with different options.

| Leg | Power limit | Ends within | Time cap |
|---|---|---|---|
| Ordinary | the leg power | 1 in, 2 deg, after 0.2 s | from distance |
| Pass-through | the leg power | 3 in, 10 deg, at once | from distance |
| Launch spot | the leg power | 2.5 in, 6 deg | from distance |
| Cross-field | the cross power | as pass-through or launch spot | from distance |
| Slow (garden pickup) | 0.35 | 3 in, 10 deg | from distance at a slow speed |
| Back-out | the leg power | 1.5 in (2.5 in at a wall flower), 10 deg | from distance |
| Wall flower pre-position | the leg power | 2.5 in, 4 deg | from distance |
| Ram | 0.6 | never reached | 0.6 s fixed |
| Park | 1.0 | 2.5 in, 5 deg | 3.0 s fixed |

**Time cap from distance:** max(1.5 s, 1.0 s + distance / speed), plus 0.5 s for a turn over 120 deg, at most 8 s. The speed is 22 in/s at 0.6 power and scales with the leg's power limit.

**Leg and cross power.**

| Program | Leg power | Cross power | Pass-through settle |
|---|---|---|---|
| Solo, Daryl, Cybugs, Primary, Flower Side and Garden Side (both from 2026-11-21) | 0.85 | 1.0 | 0 s |

The launch spot tolerance is deliberately loose: the aim step that follows turns the robot onto the cell. The tight tolerance ran legs to their time caps because the drive stalls short of its target.

### 2.2 Launch

Aim at the cell (with the launch's aim trim), then launch one salvo and wait for it to complete or be dropped.

- **A salvo is always launched**, whatever any count says. A count that under-reads must never throw a salvo away.
- The leg before a launch ends at a heading that already includes the aim and the trim, so the aim step only fine-tunes.
- **Each launch has one aim trim, which holds all of its allowance.** The far-cell extra of [aiming.md](aiming.md) is not applied in any auto (built 2026-11-17; so in every auto at the Novi Qualifier; before that the far-cell launches got the extra and a -8.0 deg trim, which worked against each other).

### 2.3 Garden pickup

1. **Pre-position** at (-59.8, 46.0), heading 105 deg: the intake mouth angled toward the rear wall so the pollen lying along the wall are caught, not plowed ahead.
2. **Slow pickup** along the wall to (-59.8, 58.5) at the same heading, power limit 0.35.
3. **Square approach:** turn to heading 90 deg, 3 in off the rear wall.
4. **Square on the wall:** the side wall, then the front wall ([motion.md](motion.md)).

### 2.4 Flower gather

1. Deploy the ramp on the way.
2. Drive to the **pre-position**, facing the flower, and wait for the ramp to settle.
3. Force the intake in.
4. **Ram:** drive at a target 12 in beyond the flower at power 0.6 for 0.6 s.
5. **Jostle** ([motion.md](motion.md)) until the robot is full by the CS5 entry count (built 2026-11-17; before, by the slot count; with CS5 switched off, as it is since 2026-11-20, the robot is never full), or the jostle's deadline in seconds since Start passes, or its one cycle is done.
6. **Back out** straight, 9 in, before any turn. Turning at the flower catches the ramp on the stand.
7. Stow the ramp: either at once after the back-out, or during the following turn as soon as the robot has turned 45 deg from the flower heading.

| Flower | Pre-position | Flower pose | Facing |
|---|---|---|---|
| Audience wall | (38.0, 26.3) | (55.3, 26.3) | 0 deg |
| Blue Wall | the audience-wall poses rotated 90 deg counterclockwise: (-26.3, 38.0) | (-26.3, 55.3) | 90 deg |

The Blue Wall flower poses are a rotation of the tuned audience-wall numbers, not measured, and have a Dashboard nudge.

### 2.5 Waits and skips

- **Wait until** an absolute time since Start. A robot cannot see its partner, so coordination is by the clock. If the time has passed, the wait does nothing.
- **Unless late:** a block of steps runs only if it starts no later than a given time; otherwise it is skipped so that the park keeps its time.

### 2.6 Park

Full power to the park pose, facing the Blue Wall, then make sure the ramp is stowed.

| Corner | Pose |
|---|---|
| Hive side | (26, 53.1, 90 deg); Auton - Solo parks at (22, 55.1, 90 deg) since 2026-11-20 (4 in nearer the center in x, 2 in farther in y, the user) |
| Audience side | (40, 53.1, 90 deg) |

The robot's front edge ends 8 in from the wall and about 4 in inside the loading zone.

### 2.7 Route points

| Point | Use |
|---|---|
| East helper (36, 40) | around the outside of the hive on the audience side |
| West helper (-36, 40) | the same on the rear side |
| Audience launch spot (50, 26.3) | launches at the audience cell; on the line of the audience flower, 31.6 in from the cell |
| Rear launch spot (-50, 26.3) | its mirror, for the rear cell |
| Rear spot at the start wall (-61.5, 15.25) | launches at the rear cell from beside the start |
| Audience spot at the wall (61.5, 7.5) | launches at the audience cell from the audience-wall start |

**The heading asked for at a pass-through point** (blue frame).

| Point | Program | Heading |
|---|---|---|
| East helper | Solo, on the way to the audience launch spot | the heading of the launch spot that follows |
| East helper | Solo, on the way to the final gather | 180 deg |
| East helper | Daryl, Cybugs, Primary | 180 deg |
| West helper | Primary | 180 deg |
| West helper | Cybugs, Garden Side, on the way to the park | 90 deg, the park's |

## 3. Auton - Solo

Start: rear wall, (-63.4, 16.0, 0 deg). Cells: rear, then audience.

1. Launch the preload at the **rear** cell.
2. Garden pickup.
3. Set the target to the **audience** cell. Cross-field to the east helper, then to the audience launch spot.
4. Launch the garden pollen (trim -2.5 deg).
5. Audience-wall flower gather; jostle deadline 23 s.
6. Back out, then to the audience launch spot, the ramp stowing during the turn.
7. Launch the flower pollen (trim -2.5 deg).
8. **Final gather:** deploy the ramp, cross to the east helper, gather the Blue Wall flower with jostle deadline 26.5 s, back out, stow.
9. Park in the hive-side corner. The pollen from the final gather stay in the robot for the TeleOp.

Nominal duration about 29 s. A switch can replace steps 8 and 9 with a plain park at (51.1, 54.0, 180 deg).

**Auton - Boomerang** (2026-11-21, the user: Solo, but the last salvo into the other hive cell, then park; ran in five playoff matches at the Novi Qualifier) is the Solo program up to and including the Blue Wall flower's back-out at (-26.3, 46.3) (`AutonBoomerang extends AutonSolo`, the hook `lastSalvoIntoFirstCell()`; every number is the Solo's). Then, where the Solo drives 81 in back to (56, 36) for a third salvo into the audience cell:

**Boomerang's launch spot from 2026-11-21 (the user: closer to the wall):** blue (-57.6, 26.2) / red (57.6, -26.2) (`AutonSolo.BOOMERANG_LAUNCH_X / Y`), 2 in nearer the rear wall than the Triple Tip's (-55.6, 26.2), which is unchanged. Inside the grid's measured zone: speed 1402 ticks/s, aim residual +11.1 deg, heading -9.4 deg (blue), 38.5 in from the cell, the robot's back corner 4.8 in from the wall. The user first asked for 4 in; (-59.6, 26.2) is outside the measured zone and leaves 2.8 in to the wall, under the launch leg's 3 in tolerance, so 2 in was chosen. Built, not run on the robot.

1. Set the target to the **rear** cell (the one the preload went into). Straight to the rear launch spot, blue (-55.6, 26.2) (the Triple Tip's, 35 in away), the ramp stowing and the pusher going to launch-ready.
2. Launch the Blue Wall flower's pollen at the rear cell with the grid's speed and aim (trim `REAR_AIM_TRIM_DEG`, 0).
3. Park at (22, 55.1) through the via point (-20, 44) at heading 30 deg, as the Triple Tip and the Daryl do.

It applies only while `THIRD_SALVO_ENABLED` is true. Controller simulation (not a robot run): the last salvo starts at about 23.8 s and the park ends at about 27.3 s, against 25.1 s and 27.8 s for the Solo (the leg to the launch spot takes 1.4 s instead of 2.7 s, the park from the rear side 2.5 s instead of 1.8 s).

## 4. Auton - Duo Daryl

Start: rear wall. Garden first. The finish changed 2026-11-21 (the user; run three times that afternoon outside a match, finished once, in 27.0 s): steps 5 and 6 replace the partner's window, a second audience-cell salvo and the audience-wall flower.

1. Launch the preload at the **rear** cell.
2. Garden pickup.
3. East helper, then the audience launch spot, blue (56, 36); launch the garden pollen at the **audience** cell with the 2-D grid's speed and aim (trim 0).
4. East helper again, then the Blue Wall flower gather; jostle deadline 19 s; back out to (-26.3, 46.3), the ramp stowing.
5. Straight to the rear launch spot, blue (-55.6, 26.2) (the Triple Tip's, 35 in away), the pusher going to launch-ready; wait for the ramp to stow; aim at the **rear** cell, the one the preload went into, with the grid's aim (trim 0) and rest aimed; **wait until 23.8 s** (the user, 2026-11-21: the salvo as late as the park allows with a 2 s buffer), then launch the flower pollen at once.
6. Through the via point (-20, 44), park in the hive-side corner, (22, 55.1), at heading 30 deg.

The 23.8 s is 30 s less a 2.0 s buffer, 3.2 s for the park (the via point about 1.5 s, the park leg about 1.7 s) and 1.0 s for the salvo; the 3.2 s and the 1.0 s are estimates until these legs have run, and the value is then set to 28.0 s less their measured sum. Estimated times: the Blue Wall flower's back-out at about 15.7 s, at the rear launch spot and aimed at about 17.8 s, a wait of about 6 s, the salvo 23.8 to 24.8 s, parked at about 28.0 s. A robot that arrives after 23.8 s launches at once. Before 2026-11-21: not leaving the flower before 15.6 s, a second salvo at the audience cell not before 18 s (the partner's window), the audience-wall flower (skipped if it would start after 22.5 s) and the park at heading 90 deg, about 25 s.

## 5. Auton - Cybugs

Start: rear wall. Daryl's first four steps, then a finish that stops collecting.

1. to 4. As Daryl, without the partner wait.
5. Set the target to the **rear** cell; drive to the rear spot at the start wall; wait for the ramp to stow; **wait 1.0 s** with the flywheel spinning; launch the flower pollen (trim 0).
6. **Run the intake OUT for the rest of the program**, so nothing more is picked up on the way to the park.
7. West helper, then park in the hive-side corner.

Measured duration 24.9 to 25.9 s.

Because the outtake counts as busy, the automatic pusher check after the last salvo is skipped; the TeleOp's start-up check covers it.

## 6. Auton - Duo Garden Side and Duo Flower Side

A matched pair for an alliance whose partner runs the other. Each robot launches only at the cell on its own start side, so neither needs an allowance for the far cell. The two take turns tipping the hive.

**Garden Side** (rear wall):

1. Start at the rear wall, (-63.4, 16, 0 deg). From Start the flywheel takes the 2-D launch grid's speed and the pusher goes to the launch-ready position; launch the preload at the rear cell from the start pose with the formula aim.
2. Blue Wall flower gather (pre-position (-26.3, 38), ram, one jostle cycle, back out to (-26.3, 46.3)); jostle deadline 20 s.
3. To the rear launch spot (-55.6, 26.2), the Triple Tip's, the ramp stowing and the pusher going to launch-ready; aim at once with the grid's aim and rest aimed; **wait until 19.3 s**, then launch the flower pollen.
4. Garden as the Solo auto gathers it (one leg to (-59.8, 58.5, 105 deg), the last 12.5 in at 0.5 power, then the squaring for at most 0.8 s); back to the rear launch spot; aim at once; **not before 24.8 s**, launch the garden pollen.
5. Through the via point (-20, 44), park at (22, 55.1) at heading 30 deg.

Changed 2026-11-21 to follow the other autos (the user; built, not run; this program has never run on the robot). **The last two salvos are as late as the park allows with a 1 s buffer:** the last at 24.8 s = 30 - 1.0 buffer - 3.2 park - 1.0 salvo, the one before it at 19.3 s = 24.8 - 5.5 (its own salvo 1.0, the garden leg 1.6, the squaring 1.0, the leg back 1.5, the aim 0.4). Every piece is an estimate; after the first run the last is set to 29.0 s less the measured park and salvo, and the second from the measured time between them. A robot that reaches the spot later than its time launches at once. Estimated times: preload done 2.4 s, the flower backed out 6.9 s, at the launch spot and aimed about 8.8 s, a wait of about 10.5 s, the flower salvo 19.3 to 20.3 s, the garden salvo 24.8 to 25.8 s, parked about 29.0 s. Before: the 90 deg pusher lock for the preload only, both later salvos from (-61.5, 15.25) at the start wall with the formula aim, the three-step garden with the full squaring, legs at 0.6 power, one wait until 25 s before the flower salvo (which could not fit the 30 s), and the park at (26, 53.1) through the west helper.

**Flower Side** (audience wall, start (63.4, 7.5, 180 deg)):

1. From Start the flywheel takes the 2-D launch grid's speed and the pusher goes to the launch-ready position, where it stays through the wait.
2. Aim at the audience cell at once, from the start pose with the formula aim, and rest aimed. **Wait until 9.0 s** (the user, 2026-11-21; it was 7.2 s: the partner's preload salvo, the tip, a margin), then launch the preload at once. The aim is not repeated after the wait.
3. Audience-wall flower gather (pre-position (38, 26.3), ram, one jostle cycle, back out to (46.3, 26.3)); jostle deadline 18 s.
4. To the Solo launch spot, blue (56, 36), the ramp stowing during the turn and the pusher going to the launch-ready position; launch the flower pollen with the grid's speed and aim.
5. Park in the audience-side corner, (40, 55.1, 90 deg), straight from the launch spot.

Changed 2026-11-21 to follow Solo, Daryl and Triple Tip (built, not run; this program has never run on the robot). Before: the 90 deg pusher lock for the preload only, the flower salvo from (61.5, 7.5) at the start wall with the formula aim, legs at 0.6 power, the park at (40, 53.1) through the point (46, 32). The grid at the start pose is an estimate (4.0 in outside the measured spots: speed 1375); at (56, 36) it is measured (speed 1371, aim residual +3.9 deg).

**The first leg and the center line (accepted, the user, 2026-11-21: do not worry about clearance).** The start is at y 7.5 (the side of the robot 0.65 in from the field's center line) and the leg to the flower turns the robot 180 deg. With the turn spread evenly over the leg the footprint keeps 0.5 in on our side; if the robot turns before it has moved, a corner swings up to 2.7 in over the line.

All aim trims are 0. The Garden Side's two waits (19.3 s and 24.8 s) are estimates to be tuned from its first logged run.

## 6a. Auton - Blaser's Gambit

Start: audience wall. The user's design of 2026-11-21; run three times that day outside a match (stopped at 10.3 s, cut off at 30.0 s, stopped at 29.0 s), so it has not yet finished its actions. The partner starts at the rear wall and tips the hive with its preload (tip 1). Every piece is one another program already has: the Flower Side's start, the Solo's flower and garden, the Triple Tip's rear launch spot and park path.

1. Start with the back against the audience wall, (63.4, 7.5, 180 deg). From Start the flywheel takes the 2-D launch grid's speed, the pusher goes to the launch-ready position and the robot aims at the audience cell (the formula aim) and rests aimed. **Wait until 7.0 s** (the partner's preload salvo and the tip), then launch the preload at once.
2. Audience-wall flower gather (pre-position (38, 26.3), ram, one jostle cycle, back out to (46.3, 26.3)); to the Solo launch spot (56, 36), the ramp stowing during the turn; launch at the **audience** cell with the grid's speed and aim: 8 pollen, tip 2.
3. East helper (36, 40), then the Blue Wall flower gather (pre-position (-26.3, 38), ram, jostle, back out to (-26.3, 46.3)); straight to the rear launch spot (-55.6, 26.2), the Triple Tip's; launch at the **rear** cell, which tip 2 turned up.
4. Garden as the Solo auto gathers it: one leg from the launch spot to (-59.8, 58.5, 105 deg), the last 12.5 in at 0.5 power, then the squaring for at most 0.8 s; back to the rear launch spot; launch at the rear cell: 8 pollen, tip 3.
5. The park is tried: through the via point (-20, 44) to (22, 55.1) at heading 30 deg.

**The salvos come before the park** (the user): no step is skipped to save the park, and the park is only tried. The three long legs that would otherwise run at 0.85 power (east helper to the Blue Wall flower pre-position, and the two legs to the rear launch spot) run at 1.0 (`B_LONG_LEG_POWER`). All aim trims are 0. There is no guard that waits for a tip before the rear-cell salvos.

Estimated times, from the Solo run of 2026-11-20 00:23 where the legs match: preload done 8.0 s, audience flower salvo 15.4 s, Blue Wall flower backed out 20.8 s, its salvo 23.6 s, garden gathered and squared 26.2 s, garden salvo 28.9 s; the park would need until about 31.9 s, so the 30 s end on the way to it. The legs between the rear launch spot and the garden have not been driven by any program.

## 7. Auton - Duo Primary

Start: rear wall. Preload at the rear cell; Blue Wall flower gather (deadline 12 s); east helper to the audience launch spot, launch at the audience cell (trim -2.5 deg); audience-wall flower gather (deadline 21 s); east and west helpers to the rear launch spot, launch at the rear cell; garden pickup and a second launch from the rear spot, **skipped if it would start after 18 s**; west helper, park in the hive-side corner.

With the garden it ran past 30 s and lost its park, hence the skip.

## 8. Parameters

| Name | Value |
|---|---|
| Rear-wall start y / audience-wall start y | 16.0 / 7.5 in |
| Garden: pre-position / end / heading | (-59.8, 46.0) / (-59.8, 58.5) / 105 deg |
| Garden pickup power limit | 0.35 |
| Square gap from the wall | 3.0 in |
| Flower ram: power / beyond the flower / time | 0.6 / 12 in / 0.6 s |
| Flower back-out | 9 in |
| Ramp stows after turning | 45 deg |
| Jostle time limit: Solo / duo programs | 9 s / 12 s |
| Park: power / tolerance / time cap | 1.0 / 2.5 in, 5 deg / 3.0 s |
| Solo: jostle deadline / final jostle deadline | 23.0 s / 26.5 s |
| Solo: garden and flower aim trims | -2.5 deg / -2.5 deg |
| Daryl: wall jostle deadline | 19.0 s |
| Blaser's Gambit: first launch not before / audience jostle deadline / wall jostle deadline | 7.0 s / 16.0 s / 24.0 s |
| Blaser's Gambit: long-leg power / squaring limit | 1.0 / 0.8 s |
| Daryl: last salvo not before (from 2026-11-21) | 23.8 s |
| Daryl: rear launch spot / park via point / park heading (from 2026-11-21) | (-55.6, 26.2) / (-20, 44) / 30 deg |
| Daryl, Cybugs and Primary: audience-cell trims | -2.5 deg |
| Cybugs: wait before the last salvo / rear trim | 1.0 s / 0 deg |
| Garden Side: second launch not before / last launch not before / jostle deadline | 19.3 s / 24.8 s / 20.0 s |
| Flower Side: first launch not before / jostle deadline | 9.0 s / 18.0 s |
| Primary: wall jostle / audience jostle deadlines | 12.0 s / 21.0 s |
| Primary: garden latest start / audience flower latest start | 18.0 s / never skipped |

The -2.5 deg trims are the net of two older allowances at the far cell from (50, 26.3): a -8.0 deg trim, added because launches kept landing left, and the far-cell extra, which turned the robot about 5.5 deg the other way at that spot. Both are symptoms of the odometry drift described in [robot_design/chassis.md](../robot_design/chassis.md). The aim is the same as before at the spot itself (checked with the robot's own aiming code: 0.01 deg), and up to about half a degree different when a leg ends 2.5 in nearer to or further from the cell, because the extra's angle depended on the distance. Built 2026-11-17, not run on the robot.

## 9. Status

- Solo, Daryl and Cybugs ran at or just before Hartland and are in the tested build. In one Hartland match Daryl lost its park after colliding with the partner, and one Solo run started very late; neither is a reason to change the programs.
- **The Novi Qualifier (2026-11-21), on `integration`.** Solo ran in six matches (Q9, Q17, Q23, Q28, Q40 and playoff match 3): it finished in 27.6 to 29.1 s in three of them and was cut off by the 30 s in the other three (Q9, Q17, M3). Boomerang ran in five playoff matches and finished and parked every time (26.9 to 28.5 s); in its first (M7) the garden gather came up short (two held, the squaring ended 1.8 in from the wall) and the second cell was not tipped in the auto. In every run the preload salvo waited about 0.8 s for the flywheel to reach its band.
- Run outside a match that day: Daryl's new finish (three runs, one finished, in 27.0 s) and Blaser's Gambit (three runs, none finished). Not run: Triple Tip, Boomerang's moved launch spot.
- **Open: the start pose at the audience wall** (Flower Side and Blaser's Gambit, y = 7.5). The team lines the robot up on the tile marks, and y = 7.5 is on no seam (the user, 2026-11-21). Which seam and which part of the robot is not decided.
- Garden Side is shown on the Driver Station again from 2026-11-21 with the changes of section 6; it has never run on the robot.
- Primary is hidden. Flower Side is shown again from 2026-11-21 (the user) with the changes of section 6; it has never run on the robot.
- **The drive gains were never tuned on this robot**, so every program relies on loose tolerances, time caps and, at the wall, physical squaring. The autos are due a rework. Target T2 ([motion.md](motion.md) section 2.1) comes first: the rework is built on a drive that passes its acceptance test, and the leg table of section 2.1 is then revised so that each coarse tolerance states its purpose. "The drive stalls short" stops being one.
- Path checks on a PC sweep the robot's footprint along the duo legs against the hive supports and flower stands; the paths clear them.

## 10. Ideas recorded but not built

Using the planner's grid routes for auto legs. (Ending a gather when the fourth element enters by the CS5 count was on this list and is built.)
