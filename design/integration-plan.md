# Robot integration plan

How we take the assembled robot from "everything is bolted on" to "match ready". Every item is a checkbox with a pass criterion; record the measured numbers in the **Record sheet** at the end and commit them after each phase.

The day-of sequence, in order, is in [robot-day-runbook.md](robot-day-runbook.md).

Ground rules
- **One change at a time**, then re-test. Commit after each phase passes (`integration` branch, see Phase 0).
- **Wheels off the floor** (robot on blocks) for every first run of a mechanism or drive change. The **pusher has no hard stops** and the **flywheel is exposed**: hands and cables clear, eyes protected.
- **Fresh, charged battery** for calibration; write the battery voltage next to every tuned number. Pusher and flywheel behavior depends on voltage, so also re-check at a mid-drained battery (Phase 7).
- **A phase is done only when its exit criterion holds three times in a row.** One lucky pass does not count.
- A test that fails gets a one-line note (what, what was on screen) before anyone changes code.

---

## Status, 2026-10-31

**Reported by the team:** integration is finished for TeleOp. The `stop` servo entry is deleted from the hub configuration.

**Deferred on purpose (affects autos only, not TeleOp):** finish the Pinpoint localization (pod offsets for `ROBOT_1` are still the prototype's) and the DriveToPose PID tuning. The autos also still use simulated launches instead of the real mechanisms (Phase 6).

**Shelved (2026-11-02): hold position and heading while launching** in TeleOp - Competition, until the `DriveToPose` PIDs are tuned on ROBOT_1 (the Axial, Lateral and Heading tuners at 48 in). When built it must specifically fight being pushed (no early release when shoved), start at the operator's launch and end with the salvo, let the driver's sticks cancel it, and sit behind a Dashboard flag.

**Untested since the last robot session (2026-11-02):** the last build flashed and run on the robot was `4717258`. Everything committed after it (the control remap, always-on intake, automatic target lock, backup mode, the flywheel dead-encoder mode, pusher without faults, the 10-run log retention and the log tool against a real hub) builds but has never run. The rows T1-T3, C2b, P9, F1, F2, F4, F5, F6, F7 are the checks for it; do them in that order of risk: INIT and bindings first, then intake and full, launch and reject cancels, target lock, backup mode, and last the flywheel encoder tests.

**Pose check, 2026-11-02 (Auto Waypoint Marker, 12 CHECK marks at the start, the blue corners, the centerline and the start again, facing the audience and the rear):** CAD says the robot is 444 x 348 mm (17.48 x 13.70 in) with the origin (the Pinpoint point) 193 mm (7.60 in) from the back edge, 174 mm from each side and 251 mm (9.88 in) from the front. The rear-wall poses match that to about 0.4 in. Start x was being seeded with the origin at depth / 2 = 8.5 in from the wall; it is now 7.60 in. **Field size (2026-11-03):** the team measured the field at about 142 x 142 in inside wall face to inside wall face (same in x and y), so the walls are at +-71 in, not 72: `FieldGeometry.HALF_IN` = 71 is used everywhere (ASSUMPTION for now) and the start poses are blue (-63.4, +16), red (+63.4, -16). That also shrinks the pose under-read found above: the start-to-audience-wall travel read 125.8 in for a true 126.8 (0.8% short, equal to the hand-pushed tape test; it was 2.1 to 2.3% against 144), and the centerline-to-Blue-Wall travel 56.2 in for a true 57.3 (2.0% short; it was 3.6%). So the pose is closer than thought, and a 3 in pose shortfall no longer explains the extra nudge at the audience side.

**Autos need a major rework (2026-11-02):** `Auton - Blue Students` and `Auton - Hive Volley (Blue)` must start from the same poses as TeleOp - Competition (blue (-63.4, +16) at 0 deg, red (+63.4, -16) at 180 deg; Hive Volley still starts at y = 12.75) and need major rework before they are used; the Auto Waypoint Marker is how the new points will be collected. Do not rely on either as it stands. **`Auton - Solo` (2026-11-02, not yet run on the robot)** is the rework: one path written for blue from the marked poses snapped to 90 deg (marked in a pose frame seeded at -64.4, so every x is shifted +1.0 for the corrected seed -63.4: garden pre-position (-63.6, 37.1) and end (-63.7, 60.3) at 90 deg; one launch spot (63.6, 8.5) whose heading is the computed aim; flower pre-position (43.0, 21.3) and flower (55.3, 21.3) at 0 deg; park (51.1, 54.0) at 180 deg, via (51.1, 8.5)), red by the 180 deg rotation, alliance picked at INIT. Each launch turns onto the cell with the lock (`AimAtCell`, with the far-cell extra), the flywheel runs from Start and follows the distance to the open cell, the flower pause waits for 4 elements (at most 2 s), and every action finishes before the next starts. First run (2026-11-03, 29.8 s, stopped by the 30 s limit at the flower): both garden legs TIMED OUT at 8 s (the controller stalls 1.6 to 2 in short: no static friction term), and the garden-to-launch-spot leg stopped the robot with its back corner on the end of the hive's support rail at about (-23.7, 23.7) (the hive has two support rails, along the row borders 2|3 and 4|5 across columns C to D, y = +-23.7); the controller clamps each axis separately so that leg ran a 45 degree diagonal first, not a straight line. Fixed 2026-11-03 (not yet re-run): a helper point (36, 40) at the launch spot's heading outside the rails (footprint at least 8 in from the rail end in a controller simulation that matched the log within 2 in), hard caps from distance (max(1.5 s, 1 s + distance / 22 in/s) + 0.5 s for a turn over 120 deg, at most 8 s; they appear in the `DRIVE start` log line), the robot squared on the rear wall and then the Blue Wall after the garden (2026-11-03: the strafe squared it on the rear wall in 0.71 s and the pose read +2.2 deg of heading drift there, but it needed about 2 in forward to square on the Blue Wall: now it also drives forward at 0.4 power, 0.25 to 0.8 s; turn to heading 90, 3 in off the wall, strafe into it at 0.3 power for about 0.3 to 0.9 s; the pose is NOT reseeded, the `AUTO SQUARE` log line records the heading and x error of the pose against the wall-flush pose), the garden approached at an angle (heading 105 from the 2026-11-03 run, the mouth turned toward the rear wall, about 11 in off the wall, the pre-position nearer the Blue Wall at y 46, the slow pickup at 0.35 power: the 0.25 run moved only 5.6 in/s and the square, with no heading hold, rotated 11 deg and stopped short; now it holds heading and pushes at 0.5 for up to 1.4 s), (2026-11-03 21:18 run: all 4 garden pollen collected, 3 from the flower, the corner square worked, the auto was cut at 30.1 s during the park leg; then: the aim step accepts 2.5 deg and waits at most 0.6 s, the launch spots end within 2.5 in / 6 deg, the cross-field legs run at power 0.85, and the jostle stops with 3 or more held and nothing new for 0.8 s and pushes back to the contact point instead of for a fixed time; the 21:28 run took 26.3 s with 4 garden pollen but about 1 from the flower, so then: the jostle is anchored to the first contact point (no creep), presses 0.4 s at the contact before each back-off, and may run up to 5 s; then (2026-11-03, not yet run): ONE launch spot on the flower's line at (50.0, 26.3) for both the garden and the flower pollen (31.6 in from the audience cell, 3 in over the 28.6 in launch map minimum, about 25 deg off its axis; a spot nearer the centerline launch better but is off the flower's line), so the robot turns and backs to the flower pre-position (38, 26.3) in one move while the ramp deploys (after the garden salvo; RAMP_DEPLOY_DURING_LAUNCH only once the launcher is known to work with the ramp down), and after the jostle it goes straight back to the launch spot turning onto the cell (FLOWER_TURN_WHILE_BACKING; the 2026-11-03 21:59 run, 26.15 s, 4 pollen for the flower salvo, the jostle kept at 5 s: it frees them for the turn to sweep in; but turning at x 53.4 caught the ramp edge on the flower, so the default is now a straight back-out to x 46.3 first, and the ramp stows during the turn once it is 45 deg from the flower heading: the launch spot is inside the 24 in stow check, so the stow had waited for arrival plus 0.81 s), and the park via point is off; 2026-11-03 22:32 run, 24.2 s: the park at full power to a target 2 in past the pose settled in 1.26 s, but the flower gave nothing in that run and the empty salvo launched anyway; so then (2026-11-04): a salvo is skipped, aim and all, only when the element counter reads and holds nothing, and the jostle no longer stops early at 3 held: it gathers until full or 23 s into the auto, since all 4 matter more than the time) the flower jostled (ram 0.6 for 0.6 s, then back 1 in (halved on 2026-11-03: 2 in moved a little too far) and push 0.2 s repeated until full for up to 3 s, then back off 10 in before the turn; on 2026-11-03 a single hard ram gave nothing and the pollen came only after 2.3 s of sustained pushing while the robot was jammed), the flower poses moved 5 in toward +Y in total (y 26.3: 3 in, then 2 in more on 2026-11-03) with the pre-position 5 in farther back and the flower rammed (target 12 in past it, power 0.9, 0.8 s; the pollen did not come out of the flower on the straight approach), the garden pickup leg driven slowly (`GARDEN_PICKUP_MAX_POWER` 0.25, about 15 in/s; the fast first run picked up 3 pollen in 2 s and the 4th only because it stalled at the end), a 3 in / 10 deg tolerance on the pass-through legs (garden, helper, park via point), and the ramp wait no longer re-sends the deploy / stow command (that restarted its 0.8 s travel timer and cost 0.8 s).

**What was checked from the code side right after that report** (so the claim is not simply taken on trust):
- The repo has **no commits or edits beyond the integration branch's own**, and `LaunchMap.TABLE` is **still empty**. Nothing tuned during integration has been written into the code.
- The Dashboard on the robot hub showed every tunable at its **code default**: pusher `COAST_LOOKAHEAD_S` 0 and `SWEEP_POWER` 0.6, flywheel `FEED_FORWARD` 0.2, `DriveToPose` gains unchanged. Dashboard values reset on every app install or restart, so tuned values only survive if they are written into the code.
- The build installed on the hub was **older than the latest commits**: the Dashboard listed no `TeleOpCompetition`, and `SWEEP_POWER` still read 0.6 (the repo has 0.3).

So the numbers to commit (below) still need to be recorded, and the competition TeleOp has not been run yet.

## Queued configuration and tuning actions

Work that is deliberately not scheduled yet. Each one is a runnable checklist; run it when it comes up, then record the result and commit the numbers.

| # | Action | Affects | How |
|---|---|---|---|
| 1 | **Pinpoint localization for `ROBOT_1`**: **done 2026-11-01, all checks pass** (48 in tape push read 47.6 in; heading after 3 spins within 5 degrees, so no yaw correction). Push Test passes with the Y pod REVERSED; pod scale 505 ticks/in (4-bar) confirmed. Offset Check: with the tape offsets (x -165.1, y 0) the wobble was 2.58 in; with x -133.7, y +9.5 mm (from the estimate and the pod ticks) it is **0.16 in** (estimates 1.2 mm and -1.6 mm; real sliding 0.47 in per turn). (Tape offsets for reference: X pod 6.5 in right of center = -165.1 mm, Y pod level with the center = 0 mm; the rotation center is not the geometric center.) **Also affects the launch map's pose distances** (a wrong offset adds about 3 in of error per radian of turning), and the TeleOp pose | autos, launch map, TeleOp pose | Pinpoint Push Test, Pinpoint Offset Check, a tape push of 48 in, a full spin, then the two verify autos |
| 2 | **DriveToPose PID tuning** | autos only | the Axial, Lateral and Heading tuners at 48 in, then the verify autos |
| 3 | **Pusher HOLD to CS4 offset**: where the arm is seen, in degrees ahead of HOLD (the team expects about 90) | the future pusher check and re-home; not TeleOp | (opmode removed 2026-11-02, done) Pusher Sensor Matrix (CS4): jog the arm to HOLD, **A** to mark it, **right bumper** to run (about 1 minute, default 3x gain), then read the `PusherSensorMatrix` log for "ARC ... ahead of the mark" at powers 0.15, 0.3 and 0.6 |
| 4 | **Smallest belt slip worth catching**, in degrees of pusher rotation. (The tooth pitch is not needed: `TICKS_PER_REV`, 751.8, already converts encoder ticks to pusher degrees. A skipped tooth is 360 / the number of teeth on the pusher's pulley, for example 6 deg on a 60-tooth pulley.) It decides whether the parked check (about +/-15 deg) is enough or the slow pass is needed | the future pusher check | choose a target, or read the tooth count off the pulley; no test needed |
| 5 | **Element counter baseline** in the pit: the empty readings depend on lighting | TeleOp (the 4-element stop) | Color Sensors Test (now `@Disabled`: remove it to use): track empty, **A**, then copy the BASELINE log line into the table in `ElementCounter` |
| 6 | **Commit the tuned pusher, flywheel and ramp values** (see the list below) | TeleOp | read them from the Dashboard after a tuned run and write them into the code |

**Measured 2026-11-01 (item 3, Pusher Sensor Matrix on `ROBOT_1`, two runs):** the arm is seen at about **328 deg ahead of the mark at zero speed (about 32 deg BEHIND the mark)**, arc about 28 deg wide (about 314-342 deg). The center shifts about 15 ms of sensor delay with speed (329.7 / 333.3 / 339.5 deg at powers 0.15 / 0.3 / 0.6). Within a run the center spread is 0.14-0.9 deg, and two runs 21 turns apart agree within 0.6 deg, so `TICKS_PER_REV` 751.8 holds to about 0.03 deg per turn and there was no belt slip. **Not as expected** (about 90 deg ahead of HOLD): confirm the mark was at the true HOLD. HOLD placement by eye was not re-tested (one A mark only).

**Item 4 answered 2026-11-01:** the pusher pulley has 20 teeth, so one skipped tooth is **18 deg** of pusher rotation, wider than the arc's +/-14 deg half-width. A parked check at the arc center therefore catches a one-tooth slip as 'arm not seen' (parked, the default sensor rate is enough), and slips come in 18 deg steps, so a re-home can round the measured offset to a multiple of 18.

**Decided 2026-11-01 (salvo 400 deg):** a salvo now turns the arm `Pusher.SALVO_DEG` = 400 deg forward from the nearest HOLD (was 360), so the last element is surely out and the arm rests about 40 deg past a HOLD; the pusher check (park at CS4, then forward to HOLD) brings it back. The target is the nearest HOLD plus 400, not 400 from where the arm is, so when the check does not run the arm simply rests 40 deg past a HOLD again and nothing accumulates (the next salvo then travels about 355 deg). **Consequence:** `Pusher.hold()` (was Launch Test LB, `holdAction`; Launch Test removed 2026-11-02) from that rest position sees the arm more than 8 deg past HOLD and turns nearly a full turn forward; the open question above now matters after every salvo.

**Decided 2026-10-31 (code review):**
- **Flywheel reversed-encoder fault removed.** Wiring and sign problems belong to the **pit check opmode** (built 2026-11-01, see [pit-check.md](pit-check.md); not yet run on the robot). Hardware Check is read-only and covers presence only.
- **Pinpoint reset moved to INIT** in TeleOp - Competition, like the autos; the INIT screen shows the Pinpoint status and pose check, the mechanisms found and whether the element counter is reading.
- **Pusher zero:** TeleOp keeps trusting the saved zero (`CONTINUE`); the pusher check (built 2026-11-01) now catches a wrong zero after each salvo.
- **Open question:** `Pusher.hold()` (`holdAction`; was Launch Test LB, removed 2026-11-02) turns nearly a full turn forward when the arm rests more than 8 deg past HOLD, which launches the stack; the team is thinking about it.
- **Overrides for sensor failures** are built into TeleOp - Competition (see the F2 row); to be tested on the robot.

**Decided 2026-10-31 (no dedicated opmodes for these):**
- Items 4, 5 and 6 are small tasks, not opmodes: pick the slip target, copy the baseline in the pit, commit the values.
- **`COAST_LOOKAHEAD_S`:** settle it during the pusher sweep calibration, from the logs of those runs; either find the value there or remove the lookahead.
- **Flywheel:** only needs the launch map and a `FEED_FORWARD` value, which can be tied to the minimum launch speed.
- **Intake auto-stop at 4 and the ramp `CLEAR_DISTANCE_IN`:** checked while testing `TeleOp - Competition`. The ramp `TRAVEL_SEC` (0.8 s) was measured by the team.

**The pusher check is built (2026-11-01, not yet run on the robot)**, in `subsystems/PusherCheck.java`, run automatically by the coordinator after a salvo when the counter reads 0 and launch, reject and the outtake are idle (the intake may be running; the check aborts if an element appears) (Dashboard: `Mechanisms.AUTO_PUSHER_CHECK`). It reverses the arm to the measured arc center (`ARM_CENTER_DEG` 326.7, 33 deg behind HOLD), waits for fresh CS4 reads, and if the arm is seen the zero is right (OK, about 1 s). If not (any slip of a tooth, 18 deg, is enough: the pulley has 20 teeth), it does **one continuous slow forward sweep, reading CS4 on every loop, that does not give up before a full turn** (`SWEEP_DEG` 420), then reverses back across the sensor the other way: the sensor reports the arm late, which shifts the two crossings in opposite directions, so the average of their centers is the true position with no delay constant. The zero is corrected from it and verified with a parked look at the expected center (another sweep if that fails, up to 3, then LOST: a warning, the zero is not trusted). It then sends the arm forward to HOLD. The first version stepped to the positions 1, 2 and 3 teeth either side and returned to HOLD between tries; the first robot run (2026-11-01) showed that wasted time, so it was replaced. Any launch, reject, intake or ramp request, or an element in the robot, aborts it and sends the arm home. Tested on the PC against a simulated arm and a delayed, narrow sensor (sensor delay 0.06, 0.10 and 0.14 s; no slip, +/-1, +/-2, +/-3, +/-5 and +9 teeth, a dead sensor, an arm in view at the start of the sweep): all found and corrected to within a few degrees, a dead sensor gives LOST after 5.4 s. About 1 s when the zero is right, 2-3.7 s when the arm is behind where the zero says (the usual belt skip: the motor turns, the arm lags), up to about 6 s when it is ahead (the sweep has to go nearly a full turn). Bring-up opmode: **Pusher Check Test (CS4)** (A runs it, X/Y fake a slip of one tooth, RB/LB fake five teeth).

## Values still to write into the code (Phase 5, "final values committed")

Anything tuned live is lost on the next install. For each, record the final value and commit it: pusher `TICKS_PER_REV`, `ENCODER_SIGN`, `SWEEP_POWER`, `APPROACH_POWER`, `STOP_EARLY_DEG`, `COAST_LOOKAHEAD_S`, `REJECT_POWER`; flywheel `FEED_FORWARD` and `ENCODER_SIGN`; `Mechanisms` manual setpoint start and launch timeout; the launch map points; ramp `CLEAR_DISTANCE_IN`; any drive, intake, pusher or flywheel direction changes in `ROBOT_1`.

---

## Where we are starting from

| Area | Status | Source |
|---|---|---|
| Mecanum drive, Pinpoint (prototype chassis) | Verified: directions, pod directions, offsets tuned, 48 in verify autos acceptable | earlier sessions |
| Pinpoint on this robot | **Not verified.** DRAFT_1 pod directions were verified, but its **offsets are the prototype's**. Footprint measured 2026-10-31: **17 deep x 13.75 wide** | `ChassisProfile.ROBOT_1` |
| Pusher | Bench-tested: thresholding, salvo/reject, lands 5-12 deg past HOLD. `COAST_LOOKAHEAD_S` still 0 | bench |
| Flywheel | Bench-tested: direction, top speed about 2200 ticks/s. `FEED_FORWARD` not tuned; second motor removed | bench |
| Intake | **Never run on hardware.** `intakeDirection` is marked unverified | `ChassisProfile` |
| Ramp | **Never run.** All four servo positions are NaN, so it will not move | `ChassisProfile` |
| Launch map | **Empty** (coordinator falls back to the manual setpoint) | `LaunchMap.TABLE` |
| Coordinator + TeleOp | **Never run on hardware** | `Mechanisms`, `TeleOpCompetition` |
| Autos | Use **simulated launches** (delays), not the real mechanisms | `AutonHiveVolley`, `AutonBlueStudents` |
| TeleOp | **TeleOp - Competition** written 2026-10-31: field-oriented drive with a heading reset, two controllers, pose carried over from the auto through `PoseStorage`. **Not yet run on the robot** (the older robot-oriented `TeleOpDraft` was deleted 2026-10-31) | `TeleOpCompetition` header |
| Hub id | Unregistered (see the decision in Phase 0) | `ChassisProfile` |

---

## Risks to settle first

These come from reading the code; each one can make a later test fail for a reason that looks like something else.

1. **RESOLVED 2026-10-31: the coordinator no longer needs the pusher to land within 8 deg of HOLD.**
   `Mechanisms.update` used `pusher.atHold()` to finish a salvo, to start the next one, and to allow the intake. The pusher rests a few degrees past HOLD after a salvo (5-12 deg on the bench), and `HOLD_TOLERANCE_DEG` is 8, so a salvo that rested past 8 deg locked the coordinator out ("SALVO running" forever, no intake, no next launch), and nothing drives an overshoot back. **Fix (done):** the intake is no longer gated on the pusher position at all (rule 1 removed), and launch and "salvo finished" use `pusher.isStopped()` (idle and not moving, wherever it rests). A salvo is a full turn from the nearest HOLD, so it works from any resting position. Landing close to HOLD is still worth tuning (Phase 2.3) but nothing locks up if it does not.
2. **`COAST_LOOKAHEAD_S` includes loop delay, so it depends on the loop time.** The bench loop was 27 ms. The full robot adds the Pinpoint I2C read, four drive writes and the intake. Calibrate the pusher landing **in the final TeleOp loop** (Phase 4), not only in Launch Test.
3. **The pusher's zero is "assumed at HOLD" at init.** The encoder is relative. The pusher must be physically at HOLD when the robot is initialized (or after an auto, the saved reference is used). Write this into the pit power-on checklist (Phase 7), and test the failure: initialize with the pusher away from HOLD and see what the driver sees.
4. **Registering a hub changes what the bench hubs do.** Once any hub id is registered in `ChassisProfile`, every unregistered hub **throws** instead of falling back to the default. The temporary pusher and flywheel bench hubs must keep running until the benches are retired. See the Phase 0 decision.
5. **Battery sag and brown-out.** The flywheel runs at full power below its setpoint, plus four drive motors, intake and pusher. Log the voltage during a spin-up while driving (Phase 3.2).
6. **Pose accuracy near the flowers.** The "stow only when clear of a flower" rule trusts the Pinpoint pose and the flower coordinates read from the manual (about 1 in, wall offset estimated). Test it at all four flowers (Phase 4).

---

## Phase 0: Prepare

- [ ] **Branch.** Create `integration` from `flywheel-bench` (it already contains `main` plus the pusher and flywheel work). `main` stays as is until Phase 3 passes. *Pass: `git log` shows the pusher and flywheel commits on the branch.*
- [ ] **Hub configuration.** See [hub-configuration.md](hub-configuration.md) for the step-by-step checklist and the **Pit Check** opmode that verifies it. On the robot's Control Hub, create one configuration with exactly these names (the code looks them up by name):

  | Device | Name | Type |
  |---|---|---|
  | Drive | `leftFront`, `leftBack`, `rightFront`, `rightBack` | motors |
  | Odometry | `pinpoint` | goBILDA Pinpoint (I2C) |
  | Intake | `intake` | motor, no encoder needed |
  | Flywheel | `fly_motor_enc` | motor with encoder |
  | Pusher | `gate` | motor with encoder |
  | Ramp | `depolLeft`, `depolRight` | servos |

  `fly_follow` is no longer used. *Pass: the Driver Station shows the configuration active, no "device not found" on any opmode init.*
- [ ] **Port map.** Write down (and photograph) which port each device is on, and which hub. If an Expansion Hub is used, note it; its commands cost extra loop time.
- [x] **Decision: robot profile and hub id.** Done: a new `ROBOT_1` profile (copied from `DRAFT_1`, to be tuned in Phases 1-2) is registered to the robot's hub (serial `5462f57f0f18d316`, Android id `7c789032eb35f27b`). The pusher and flywheel benches are retired.
- [ ] **Register the other test chassis' hub.** Because a hub is now registered, the remaining test chassis' hub **throws** ("Unrecognized Control Hub") on its next opmode run until its ids are added to a profile (`PROTOTYPE` or `DRAFT_1`, whichever it is). Run any opmode on it: the "Unrecognized Control Hub" error lists its ids or connect it over USB and read the ids, then add them.
- [ ] **Safety setup.** Robot on blocks; pusher and flywheel guarded; stop procedure agreed (who holds the Driver Station Stop); battery strap and wiring strain relief checked.
- [ ] **Record sheet started** (end of this file).

Exit: configuration active, profile decision made, branch created.

---

## Phase 1: Walk-through (no code, then a smoke test)

- [ ] **Wiring.** Every connector seated and strain-relieved; no wire near the pusher arm, flywheel or intake; Pinpoint cable and pods secure; no wire that can be caught by the ramp.
- [ ] **Mechanical.** Pusher arm turns freely through a full turn by hand with no element in the track; flywheel spins freely by hand; intake rotates freely; ramp servos move without binding (by hand with power off, gently). Fasteners tight; thread locker where appropriate.
- [x] **Size.** (Confirmed 2026-10-31: the robot fits the 18 in cube with the ramp stowed.) With the ramp **stowed**, the robot fits the starting cube (rule R102, 18 x 18 x 18 in) in the starting configuration. Footprint **measured: 17 deep x 13.75 wide** and entered into `ROBOT_1` (the autos, the TeleOp and the launch-map tuner all derive their start X from the depth, so the start is now x = -63.5). The height was not recorded as a number, only confirmed to fit.
- [ ] **Power-up smoke test.** Battery in, hub boots, Driver Station connects, **Pit Check** shows the expected profile, nothing moves on its own. *Pass: no unexpected motion, no warnings.*

Exit: robot powers up safely and the footprint is recorded.

---

## Phase 2: Each subsystem on the real robot

Run the remaining bring-up opmodes (Motor Map Test, Open Loop Test, Drive Test, Intake Test, Flywheel Test, Pusher Test, Pusher Sensor Test and Ramp Test were deleted 2026-10-31 as done or covered; see git history). The bench numbers are a starting point, **not** a result: the robot's gearing, loading and wiring can differ.

### 2.1 Drive and localization
- [x] **Motor directions**: verified by the team (the Motor Map Test and Open Loop Test were deleted).
- [ ] **Pinpoint Push Test**: every step passes (pod directions and axis signs), following the opmode's own prompts.
- [ ] **Pinpoint Offset Check**: tune `xOffsetMM`, `yOffsetMM` **for this robot**. Pass: the wobble and the forward and strafe error estimates are as small as on the prototype (its tuned result was a wobble of 0.72 in with estimates near 0).
- [ ] **Verify autos** (Auton Verify - Forward 48 and back, and Lateral 48 and back): final error acceptable to the team. Record the numbers; the prototype's results were judged acceptable, so use those as the bar.
- [ ] **Tuners** (Axial, Lateral, Heading at 48 in): gains adjusted until the verify autos settle without oscillation.

### 2.2 Intake
- [ ] **Direction**: positive intake power pulls elements **in** (fix `intakeDirection` if not). Outtake (-0.5) pushes out.
- [ ] **With real elements**: both pollen and nectar feed in cleanly, no jam on the way to the pusher, none escape behind the robot.
- [ ] **Record intake current** while running free and while pulling an element, for a future jam detector (the team decided the driver handles jams for now but wants current-based detection later).

### 2.3 Pusher
- [ ] **Sign check** (Pit Check, or the Launch Test that was removed 2026-11-02): a positive command counts forward revolutions; velocity sign right. Set `ENCODER_SIGN` if not.
- [ ] **TICKS_PER_REV** (bench value 751.8; the robot may have different gearing): confirm it from the logs of the Pusher Sensor Matrix sweeps (that opmode was removed 2026-11-02). The arm is seen at the same angle every turn, so a wrong value shows up as the arc drifting from turn to turn (a large RISE or CENTER std).
- [ ] **Mark HOLD** (done; the Launch Test that did it was removed 2026-11-02) at the physical HOLD position (the arm at the back, limiting entry to 4).
- [ ] **Tune** `SWEEP_POWER`, `APPROACH_POWER`, `STOP_EARLY_DEG`, then `COAST_LOOKAHEAD_S` from the "Pusher last settle" suggestion. Do this **with the pusher loaded**: 0, 1, 2 and 4 elements in the track. Loaded salvos can need more power.
- [ ] **Reject** with elements in the track: reverses and finishes on the nearest HOLD behind.
- [ ] **Exit**: 10 salvos in a row (including with 4 elements) with no fault, each resting consistently near HOLD (read "Error to nearest HOLD" and "Pusher last settle"; the target is a few degrees, but a salvo that rests a little past HOLD no longer locks anything out, see Risk 1). Record the min and max landing error.

### 2.4 Flywheel
- [ ] **Direction and sign**: spins the launching way, speed reads positive (the bench fix flipped the flywheel direction; re-confirm on the robot).
- [ ] **FEED_FORWARD** tuned (in the Dashboard, with the Launch Map Tuner or the TeleOp) at about 600, 1000 and 1500 ticks/s so the hold power is just enough (single motor now).
- [ ] **Spin-up time** to ready at the speeds you expect to launch at; **dip and recovery** over a 4-ball burst (the `SALVO done` log line shows the lowest speed during each salvo). Record; this is the data that tells us whether one motor is enough.

### 2.5 Ramp
- [x] **Measured** 2026-10-31 (with the Ramp Test, since deleted): the four positions are in `ChassisProfile` and the mirrored pair is verified.
- [x] **TRAVEL_SEC** = 0.8 s, measured by the team.
- [ ] **Stowed** position keeps the robot inside the starting cube; **deployed** reaches the flower opening without touching the floor.
- [ ] **CLEAR_DISTANCE_IN**: measure the real ramp reach from the robot center; set it from that, not the 24 in placeholder.

Exit: each subsystem passes its own checklist, all numbers are in the Record sheet and committed.

---

## Phase 3: Mechanisms together (no coordinator yet)

- [ ] **Launches with real elements on the robot** (the Launch Test opmode was removed 2026-11-02; use the Launch Map Tuner or the TeleOp). Real elements (both kinds) through the pusher into the flywheel. Salvo (Y) and dry salvo (RB), reverse (X). Record per speed: speed at release, lowest speed during the salvo, whether all four elements leave and in what order. *Pass: repeatable, no pusher fault, no element stuck.*
- [ ] **Intake feeding the pusher**: run the intake with the pusher at HOLD; elements stop at the pusher arm, exactly 4 fit, a 5th is held out. (Rule G407: at most 4 controlled.)
- [ ] **Everything powered while driving.** Drive around while the flywheel spins up and the intake runs. Record the **battery voltage minimum** and the **loop time**. *Pass: no brown-out, no hub reset, no pose jumps from the Pinpoint while the flywheel spins.*
- [ ] **Loop time**: record it. If it is much above 27 ms, note which opmode and what was running (Risk 2).

Exit: the mechanisms work together without a coordinator, and loop time and voltage are recorded. Merge `integration` into `main` here if you want a stable base.

---

## Phase 4: Coordinator and TeleOp (the five rules)

Run **TeleOp - Competition**. Each row passes three times in a row.

| # | Test | Expected |
|---|---|---|
| R1 | Hold intake (operator RT) while the pusher is mid-salvo, resting past HOLD, or anywhere else | The intake runs regardless of the pusher's position (rule 1 removed) |
| R1b | Initialize with the pusher not at HOLD (Risk 3) | Behavior understood and written into the pit checklist |
| R1c | Launch several salvos in a row, the pusher resting a few degrees past HOLD each time | Each launches; nothing sticks on "SALVO running"; the flywheel auto-stops after each |
| R2 | Deploy (X), then stow (Y) | Deploy turns the intake on; stow turns it off unless RT is held |
| R2b | Deploy with the pusher away from HOLD | Ramp deploys and the intake turns on (the intake is not gated on the pusher) |
| R3 | Launch (A) with the flywheel cold | Spins up, then launches the moment it is ready |
| R3b | Launch with the flywheel already spinning | Launches at once |
| R3c | Launch when the flywheel cannot reach the setpoint (set it too high) | Drops after the timeout with a NOTICE, no stuck state |
| R3d | After a salvo | In TeleOp - Competition the flywheel keeps running (it runs from Start and has no stop control in the match). The 0.5 s auto-stop still applies where it is left on (the autos) |
| R4 | Reject (B) with elements | Pusher reverses with outtake for the whole move, then both stop |
| R4b | Press reject or launch while a salvo or reject is running | Ignored, no stuck state |
| R5 | Stow (Y) while under each of the **four flowers** | Stow waits ("Stow waiting"), then happens once the robot drives clear |
| C1 | Element counter: put 1, 2, 3, 4 elements in the transport one at a time | The screen shows the stack filling from CS4, the count rises 1 to 4, and "FULL" appears |
| C2 | With the intake toggled on, add the fourth element | The intake stops about half a second later, the toggle is cleared, a notice says "Intake stopped", and the outtake still works (the autos; in TeleOp - Competition see C2b) |
| C2b | TeleOp - Competition (also one 250 ms rumble on both pads when it becomes full, on a rumble-capable pad; the log's SETUP line names each pad's type): no toggle. Feed in elements, add the fourth; then hold the left trigger, then the right trigger | The intake runs IN from Start, stops about 0.8 s after the fourth element is first seen (500 ms confirmed), runs IN again when the count falls below 4; holding the left trigger forces OUT, holding the right trigger forces IN even when full. (There is no mechanical detent yet, so `Mechanisms.FULL_RUNS_OUT` is false: it stops instead of running OUT; with it true the intake runs OUT when full, which would need the detent to avoid hunting 4 -> 3 -> 4) |
| C3 | Launch a salvo from a full robot | The count falls to 0; the operator presses the intake trigger again to resume |
| C4 | Put the opponent's nectar in (red on blue alliance, or the reverse) | A warning names the sensor (CS1 to CS4) and no automatic action follows |
| C5 | Look at the loop time with the counter on | The added cost is one sensor read (about 13 ms) every 40 ms; note the loop ms and re-check the pusher landing |
| P1 | (Pusher Check Test, removed 2026-11-02; passed) Pusher Check Test (CS4): arm at HOLD at INIT, track empty, **A** | The arm goes back about 33 deg, the screen says CS4 sees the arm, it returns to HOLD, result OK in about 1-2 s |
| P2 | (removed 2026-11-02; passed) Pusher Check Test: **X** (fake +1 tooth) then **A**, then **A** again; same with **Y** (-1) | The first A sweeps, reports the arm found and the zero corrected by about -18 deg (+18 for Y), and returns to HOLD; the second A reports OK; the arm lands at the real HOLD. +1 takes about 2-4 s, -1 up to about 6 s |
| P3 | (removed 2026-11-02; passed) Pusher Check Test: **RB** (fake +5 teeth) then **A** | A slow sweep finds the arc, the zero is corrected by about -90 deg (result SWEEP), a second A reports OK; check by eye that the arm returns to the real HOLD |
| P2/P3 result 2026-11-01 | Pusher Check Test on the robot (continuous-sweep version) | OK in 0.8-1.4 s. A fake +1 tooth: one sweep, 2.8 s, corrected -16.8 deg (expected 18). A fake +90 deg: two sweeps, 4.8-5.3 s, net -103, -90, -89 (the first crossing after the long reverse is inaccurate and fails the verify look; the second sweep fixes it). The verify look saw the arm every time. An earlier run's +90 and +180 corrections (-55.5, -147) may have been about 33 deg off; not reproduced. Still to do: P4, P5 (TeleOp after a salvo), P6 (abort) |
| P2/P3 result, power cycle | 2026-11-01, TeleOp after a power cycle with the pusher far from HOLD | The zero was assumed at INIT, so the arm was really about 151 deg off. The first salvo ran from there; its check missed the arm, swept, corrected -150.8 deg and verified in 4.1 s; every later check was OK in 1.2 s, parking at 323.9-327.3 deg. The first salvo after a power-up still runs from the unverified zero |
| P8 | Start-up check: power cycle with the pusher anywhere (track empty), run TeleOp - Competition, press Start and touch nothing | The warning "Pusher zero not verified" shows, then about half a second after Start a check runs (seek, look, and a sweep if the arm is not where the zero says), the zero is verified or corrected, the arm goes to HOLD, the warning clears. Start with elements in the track: no start-up check (logged), the first salvo's check does it. Launch right after Start: the check aborts |
| P9 | Manual calibration sweep: TeleOp, track empty, hold the operator's left bumper and press Y (the intake stops by itself for the sweep and the log says so; it runs again when the sweep ends or is stopped) | The screen's Pusher check goes straight to sweep (about 3-6 s), the zero is corrected (a small correction when it was right), verify, the arm returns to HOLD; the log has `COMMAND pusher calibration sweep` and `PUSHER CHECK SWEEP: calibration sweep ...`. With elements held, the intake on, or a launch queued: refused with a NOTICE and the arm does not move. Pressing it again during the sweep stops it (the arm goes forward to HOLD). Y alone still stows the ramp |
| P10 | Phantom element after a salvo (2026-11-02 logs: the arm rests 40-60 deg past HOLD and CS4 read it as 1 element in an empty robot, so 25 of 110 post-salvo checks were skipped and the manual sweep was refused with "1 element(s) held"; `Mechanisms.ARM_NEAR_AHEAD_DEG` 25 -> 70): launch a full load empty, watch the screen and the `count` / `slots` log columns | After the salvo the count reads 0 and the slots `....` with the arm at rest; the `check` column then runs (seek, look, ok); left bumper + Y starts the sweep instead of being refused; with 3 real elements held the count reads 3, not 4 |
| P8/P9 result 2026-11-01 | Start-up check and manual sweep on the robot | The start-up check worked (1.3 s, OK). All 4 manual sweeps aborted ("an element is in the robot": the arm passes CS1 to CS3 and reads as an element) and 5 presses were refused ("1 element held": the arm resting 5-15 deg past HOLD flickers A/O at CS4 for seconds at a time, a phantom count). Fixed: sweeps ignore all counts; an unclassified reading at CS4 near the arm is not counted. **Still open:** CS4 sees the arm over a wide zone with a long tail toward HOLD (the "edge"), so ARM_CENTER_DEG and the slip window need a proper map of the sensor's response against angle; re-test P9 after reinstalling |
| T1 | Target lock: TeleOp, load 4 elements (it engages by itself, the log says `TARGET LOCK on (full)`), then also press the operator's A with fewer than 4 (`on (manual)`; with the automatic lock on, A cancels it and it stays off until the robot is next full; A again turns it off; launch: the log says `TARGET LOCK released: the salvo is done` and the lock is off, manual or automatic), anywhere on the field | The robot turns to point its front (the intake and launcher end) at the open cell and holds it, while the sticks still drive; the screen shows ON, the bearing and the error; ALIGNED on the screen when within 2 deg; letting go returns the turn stick. Try it from the start, from a corner, and after spinning the robot by hand; heading at the wrap (179 / -179) must not spin the long way |
| T2 | Target lock: hold the left bumper and nudge with the right stick while locked (the stick alone does nothing); drive from near one of your alliance's cells to near the other | The aim moves up to 15 deg either way and is forgotten on release; the target cell switches by itself to the nearer of your two cells (with 6 in of hysteresis, no flip-flop) and the lock follows it; the log shows `OPEN CELL -> ...` |
| T3 | Target lock and launches: lock on, wait for ALIGNED, launch (driver A), several distances; also launch with fewer than 4 elements, locked and not locked | Launches land where the launch map said; the lock aims the robot's centerline (the launcher-offset compensation is ON again (2026-11-02, `TargetLock.LAUNCHER_OFFSET_LEFT_IN` = -2.5, the launcher is 2.5 in right of the centerline; 0 turns it off; test: BLUE launches at the rear and audience cells with the D-pad marks); the 2026-11-02 runs showed position-dependent aim errors that it did not explain: launches at the cell on the far side from the start wall (blue audience, red rear) needed about 4.5 in more, so `TargetLock.FAR_CELL_EXTRA_LEFT_IN` is added for that cell only (-4.5 first, which landed LEFT at 34 to 36 in and needed a 9.9 deg right nudge at 47 in, all at headings past +10 deg from straight at the rear wall; now -3.0, a compromise; the needed amount looks heading- or position-dependent, so mark launches from y > 10 and from y 0 to 5 before fitting a fade); retune it from the marks; it looks tied to the heading relative to the start), so if they land consistently left or right, check that offset and then set `TargetLock.AIM_OFFSET_DEG`; if it hunts or is slow, tune `kP` and `kD` (a simulation of the measured turn response gave about 1.1 s for a half turn) |
| T10 | Duo Garden Side (built 2026-11-04 on branch `duo-autos`, not yet run on the robot; design/duo-autos.md): alone on the field first, blue and red: INIT pose check, the preload launch at the rear cell, the Blue Wall flower (watch the first ram and `WALL_FLOWER_DX/DY`), the launch-spot return, the wait (`AUTO WAIT` line), the garden, the second launch, the park; read the `AUTO TIMING` lines for the 30 s fit | Every leg ends on its target, no contact with a hive support, the ramp stowed at the end; the timing lines say where the 30 s runs out (G probably overruns: see the doc) | |
| T9 | Duo Flower Side (built 2026-11-04 on branch `duo-autos`, not yet run on the robot; design/duo-autos.md): alone first, then with the garden side as the partner: back flush to the audience wall, the wait for `F_FIRST_LAUNCH_NOT_BEFORE_SEC`, the preload, the flower, the second launch at the audience spot, the park via the point | The first launch starts after the partner's tip finished (log both robots' times); the aim lands in the cell on each launch (trims `F_*_TRIM_DEG`); the park breaks the loading-zone line | |
| T8 | Flywheel FF+P controller (developed 2026-11-04, not run on the robot; `Flywheel.CONTROL_MODE` 1 in the Dashboard, 0 is the bang-bang and the default): run the TeleOp with mode 1, watch the speed hold near the setpoint (the fly_vel column) and a salvo's dips; then compare with `python tools/logs/launch_stability.py` against the bang-bang runs; try kP 0.003 to 0.005 first and back off if the speed oscillates | Idle speed within about 1 percent of the setpoint (no +3 percent overshoot), no hunting, the speed back near the setpoint within about 0.3 s after each pollen, the preload spin-up no slower than before | |
| T7 | Hold position while launching (built 2026-11-04, not yet run on the robot; `drive/HoldPosition.java`): launch with the lock held from a settled stop, with Hive Dash and with Auto Launch; push the robot gently and then hard by hand during the salvo; move a stick during it | A `HOLD latched` line before each salvo, the robot returns to the latched pose after a push, the stick cancels it at once, `HOLD released` lines give the peak error and command; check the drive motors for heat after a few salvos and that a salvo from a held pose lands as well as before | |
| T6 | Pusher lock (built 2026-11-04, not yet run on the robot; design/mechanisms.md): load 4 pollen (`Mechanisms.PUSHER_LOCK_WHEN_FULL` is the off switch; there is no manual lock control): the arm should lock by itself 1 s after full, move to about 90 deg and hold, the log's `slots` column shows what CS1 to CS3 read with the arm parked, the intake runs OUT, the count stays frozen; LB + X again unlocks to HOLD; a launch from the lock (310 deg) and a reject from the lock; then the automatic lock on a full robot while driving hard, in the TeleOp and in the auto (does the lock delay the preload? the flower gather?) | The arm rests at 80 to 100 deg; no pollen launched by the lock; the unlock (a reject, or holding the right trigger) returns it to within 8 deg of HOLD; a launch from the lock clears all four; the full load no longer flickers 3 / 4 while driving | |
| T5 | Hive Dash (built 2026-11-04, not yet run on the robot; design/hive-dash.md): `HiveDash.DRY_RUN` first (hold D-pad left / right, watch the route on the Dashboard field view and the `HIVE DASH start` line, nothing drives); then low power (`MAX_POWER` 0.5) with a spotter: audience and rear cells for both alliances, from the launch spots, the garden corners and under the hive (the field origin), with `UNDER_HIVE_ROUTES` on; check the cancel on release, the refusal with an empty robot / in backup mode, the stop at the launch spot, ALIGNED then the launch, and the strafe-to-forward ratio from the `HIVE DASH sample` lines | Each press ends in a launch from the zone or a clean cancel; no contact with a support; the lock aligns in under 0.5 s after arriving; speeds match the TeleOp logs (strafe about 0.75 of forward) | |
| T4 | Auto-launch (prototype, built 2026-11-03, not yet run): operator RB on, drive into a launch zone with the lock on (full and by hand), at the audience and rear cells | The chassis stops on entering the zone (`AUTO LAUNCH: entered`), one salvo goes when ALIGNED and stopped (`AUTO LAUNCH: ... | aim line`), then the lock releases; nothing happens with RB off, with 0 elements, or outside the zone; if it does not launch it gives up after 2.5 s (`gave up`). Mark the launches; zone centers and radius are `AutoLaunch` values in the Dashboard (the audience zone is from geometry, check it) |
| P4 | Cover CS4 (or unplug it) and press A | LOST with a reason, a warning in TeleOp; the arm is not left jammed |
| P5 | TeleOp: launch a full robot empty, wait (intake off) | After the salvo the `check` log column goes seek, look, return, ok; `PUSHER CHECK OK` in the log; the next launch is not delayed beyond about 1.5 s |
| P6 | TeleOp: start a check, then press launch or reject (the intake toggle no longer aborts it) | The check aborts at once (`PUSHER CHECK aborted` in the log), the arm goes forward to HOLD, the request is carried out |
| P7 | TeleOp: leave the intake toggled on, launch a partial load until the robot reads empty | The check runs with the intake running (the 2026-11-01 run skipped it six times because the intake was on); if an element reaches CS1 during it, the check aborts (`an element is in the robot`) and the arm goes forward to HOLD |
| F1 | Jam handling (pusher faults are OFF for now, `Pusher.FAULTS_ENABLED` = false): block the arm by hand during a salvo, then press reject (B); block it during a reject, then press launch (A) | No fault or warning; a move that cannot finish stops by itself after `Pusher.MOVE_TIMEOUT_S` (3 s) and stays where it is (no return to HOLD, no nudge to HOLD, no automatic pusher check) until the next salvo or reject, with the log saying `gave up`; B during a salvo cancels it and reverses to HOLD; A during a reject cancels it and launches; the next launch works at once. With the flag true the old fault (Back clears) is back |
| F2 | Element counter stops reading (unplug a color sensor): TeleOp | The screen warns "ELEMENT COUNTER NOT READING"; the intake keeps running IN (no 4-element stop, no auto lock when full); holding the operator's left trigger pushes extras out. The old operator L3 and R3 overrides were removed from the controls (2026-11-02) |
| F4 | Backup mode by hand: TeleOp, driver holds LB + X (the screen counts the 1 s; letting go early does nothing) | Both pads give three rumble taps (a rumble pad only); the screen shows the BACKUP MODE banner; stick up is the robot's front; target lock does nothing (RT-bumper and full); LB + RB does nothing; the flywheel is set for 42 in (`BACKUP_LAUNCH_DISTANCE_IN`, the launch map's speed plus the trim); LB + X again does nothing; the log has `BACKUP MODE ENTERED`, `drive_mode` is `backup`, and the only way out is to stop the opmode |
| F5 | Backup mode by a Pinpoint fault: TeleOp with the robot on blocks, unplug the Pinpoint cable: once for under 0.5 s, once for longer | A quick unplug (under 0.5 s) logs `PINPOINT fault` then `recovered ... (a blip)` and stays field-oriented; a longer one enters backup mode as in F4 with `Pinpoint status ...` or the exception as the reason |
| F6 | Lost flywheel encoder: TeleOp, unplug the flywheel encoder cable (robot on blocks), let the flywheel run | After about 1 s the log says `FLYWHEEL ENCODER LOST` with the numbers, both pads give two taps, the screen warns, the wheel runs at about `LOST_BASE_POWER`; the speed trim (left bumper + D-pad) moves the power in steps of about 0.011; a launch is no longer dropped (ready 2 s after the switch). Control: a normal spin-up and a salvo never trip it, and an unplug-replug during the 1 s window does not latch |
| F7 | Flywheel duty cycle: TeleOp, let the flywheel hold 1350 ticks/s for 10 s on a fresh battery (no launch), then read the log | The average of the `fly_power` column over the steady part is the number to put in `Flywheel.LOST_BASE_POWER` (found 2026-11-03 from the existing TeleOP logs instead: 0.40 at 1350, now set; this row is only to confirm it on a fresh battery, and to check how far the speed dips and recovers in a salvo without the encoder); `fly_amps` shows the running current (the open-loop power is not voltage compensated, so the speed trim is how the drivers correct for a sagging battery) |
| F3 | Remove a mechanism from the hub configuration (for example the ramp) | Coordinator keeps running with that mechanism skipped |
| D1 | Drive while firing | Pose stays correct, mechanisms respond, no lockups |
| D2 | The target cell follows the pose (the nearer of the alliance's two cells, `Mechanisms.setAutoTarget`); no D-pad switch any more | The launch distance and setpoint, and the target lock, use the nearer cell; it switches once when you cross the middle |

Also in this phase:
- [ ] **Re-tune `COAST_LOOKAHEAD_S` in this loop** (Risk 2) and confirm the salvo lands consistently near HOLD (Risk 1 is resolved, so this is about repeatability, not a lock-out).
- [ ] **Driver and operator feel**: precision mode (30%), turn scale, trigger thresholds, button layout. Collect changes from the drivers; do not change several at once.
- [ ] **Alliance select at INIT** works for both alliances and sets the start pose and the aimed HIVE.

Exit: every row passes three times; the pusher lands inside what the coordinator needs.

---

## Phase 5: Calibration data

- [x] **Launch map first pass, 2026-11-01** (Launch Map Tuner, charged battery, pose distances after the Pinpoint was verified, 18 salvos, no pusher fault): (28.6 in, 1325), (32.0, 1350), (35.8, 1350), (42.3, 1350), (46.7, 1375), (62.0, 1450), (82.2, 1650) ticks/s, all to the red audience CELL from the audience wall and the corners; now in `LaunchMap.TABLE`. The speed keys in TeleOp now trim the map's speed (`Mechanisms.speedTrim`). **Not recorded:** how many of each 4-element salvo scored, nothing measured beyond 82 in or below 28.6 in (the end speeds are held), and no check with nectar. Redo or extend as needed.
- [ ] **Launch map, remaining** (Launch Map Tuner): record (distance, setpoint) pairs from the closest legal launching position to the farthest you intend to use, for each target CELL that matters. Validate each point with a full 4-ball salvo and count how many score. Paste the lines into `LaunchMap.TABLE`. Record the battery voltage.
- [ ] **Same speed for pollen and nectar** (the team's decision): confirm both score from the same setpoint, or note the exception.
- [ ] **Final values committed**: pusher powers and `COAST_LOOKAHEAD_S`, `FEED_FORWARD`, ramp positions and `TRAVEL_SEC`, offsets, footprint. Re-run Phase 4's smoke rows afterward.

Exit: the launch map is populated and the coordinator no longer warns "Launch map empty".

---

## Phase 6: Autos on the real mechanisms

- [x] **Wire the real mechanisms** into `AutonHiveVolley` and `AutonBlueStudents` in place of the simulated delays (done 2026-11-01, **not yet run on the robot**). The coordinator is built in INIT with the pusher reference `ASSUME_AT_HOLD` and updated every loop (also in the final loop of Blue Students); the flywheel spins from Start and stays up; the INIT screen shows the mechanisms found and the preload the element counter sees (expect 4); the log samples carry the mechanisms' columns. Blue Students: `launch()` for each volley, `setIntake` around the garden run, the open CELL set to rear for the first volley and audience after the first tip, the ramp deployed on the way to the flower (deploying turns the intake on), a `FLOWER_DWELL_SEC` (0.8 s) at the flower, and `stowNow()` at the launching spot. Hive Volley: `launch()` after the LEAVE drive.
  - **Why `stowNow()`:** the launching spot at the audience wall (about 63.5, 11) is about 14 in from the flower center (68.6, 24), inside the placeholder `CLEAR_DISTANCE_IN` of 24, so the operator's pose-checked stow would never release there. The auto path is what guarantees the ramp is clear, so it stows without the check. Revisit once the real ramp reach is measured.
  - **Open:** the flywheel needs about a second to spin up from rest, so the first volley waits for it (`LAUNCH_TIMEOUT_SEC` 3 s); the time budget of the whole auto against the 30 s period has not been measured with the real mechanisms.
- [ ] **Update `tools/autosim`** if the paths or timings change, and re-check the legality and 4 s leg timeouts. (The page still estimates a volley as `SPIN_UP_SEC` + `LAUNCH_SEC` = 2.5 s; the autos no longer have those constants. Replace the estimate with the real volley time from the `SALVO started` / `SALVO done` log events, and add the ramp deploy and stow waits and the flower dwell.)
- [ ] **Start-pose legality** at the field: robot touching the wall, inside its alliance half, not in the loading zone or flower volume (rule G304), in the starting configuration with the ramp stowed.
- [ ] **Three consecutive runs of each auto**, from the legal start, without a fault.
- [ ] **Auto to TeleOp handoff**: the autos save their final pose to `PoseStorage` and **TeleOp - Competition** starts from it (the INIT screen says "AUTO pose, saved N s ago"; D-pad up on gamepad 1 discards it), keeping the pusher reference (`CONTINUE`). Test: run an auto, then immediately TeleOp; pose and pusher zero are still right.

Exit: both autos complete three times; the handoff keeps pose and pusher zero.

---

## Phase 7: Match readiness

- [ ] **Full-match rehearsals** (auto, then TeleOp for the full period, three times) with the drivers, including at a **mid-drained battery** (recheck the pusher landing and a launch or two).
- [ ] **Thermal and wear**: after a rehearsal, check the pusher, flywheel and intake motors and drivetrain are not hot; check fasteners and wires again.
- [ ] **Inspection list** against the current manual: size in the starting configuration, wiring, battery mounting, hub configuration, anything the manual requires. (This plan does not restate rules; read them from the manual.)
- [ ] **Pit power-on checklist** (one page): battery, hub config selected, **pusher at HOLD before init**, ramp stowed, alliance chosen, pose seeded, Dashboard optional.
- [ ] **Freeze**: tag `integration-v1`, merge into `main`, keep a copy of the app build that was used.

---

## Record sheet

Fill in as you go. Date, battery voltage and who ran it next to each line.

| Item | Value | Where it goes | Date / volts |
|---|---|---|---|
| Hub ids | | `ChassisProfile` | |
| Footprint depth / width (in) | | profile | |
| Pinpoint X / Y offset (mm) | | profile | |
| Verify-auto final error (fwd / lateral, in) | | notes | |
| Intake direction, current free / loaded (A) | | profile / notes | |
| Pusher `TICKS_PER_REV` | | `Pusher` | |
| Pusher `SWEEP_POWER` / `APPROACH_POWER` | | `Pusher` | |
| Pusher `STOP_EARLY_DEG` / `COAST_LOOKAHEAD_S` | | `Pusher` | |
| Pusher landing error over 10 salvos (min / max deg) | | notes | |
| Flywheel `FEED_FORWARD` | | `Flywheel` | |
| Flywheel spin-up to ready (s) at speeds | | notes | |
| Flywheel lowest speed in a 4-ball burst (% of setpoint) | | notes | |
| Ramp stowed / deployed L, R | | profile | |
| Ramp `TRAVEL_SEC`, `CLEAR_DISTANCE_IN` | | `Ramp` | |
| Loop time: Launch Map Tuner / TeleOp (ms) | | notes | |
| Battery minimum under load (V) | | notes | |
| Launch map points | | `LaunchMap.TABLE` | |
