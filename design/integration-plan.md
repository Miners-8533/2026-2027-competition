# Robot integration plan

How we take the assembled robot from "everything is bolted on" to "match ready". Every item is a checkbox with a pass criterion; record the measured numbers in the **Record sheet** at the end and commit them after each phase.

Ground rules
- **One change at a time**, then re-test. Commit after each phase passes (`integration` branch, see Phase 0).
- **Wheels off the floor** (robot on blocks) for every first run of a mechanism or drive change. The **gate has no hard stops** and the **flywheel is exposed**: hands and cables clear, eyes protected.
- **Fresh, charged battery** for calibration; write the battery voltage next to every tuned number. Gate and flywheel behavior depends on voltage, so also re-check at a mid-drained battery (Phase 7).
- **A phase is done only when its exit criterion holds three times in a row.** One lucky pass does not count.
- A test that fails gets a one-line note (what, what was on screen) before anyone changes code.

---

## Status, 2026-10-31

**Reported by the team:** integration is finished for TeleOp. The `stop` servo entry is deleted from the hub configuration.

**Deferred on purpose (affects autos only, not TeleOp):** finish the Pinpoint localization (pod offsets for `ROBOT_1` are still the prototype's) and the DriveToPose PID tuning. The autos also still use simulated shots instead of the real mechanisms (Phase 6).

**What was checked from the code side right after that report** (so the claim is not simply taken on trust):
- The repo has **no commits or edits beyond the integration branch's own**, and `ShotMap.TABLE` is **still empty**. Nothing tuned during integration has been written into the code.
- The Dashboard on the robot hub showed every tunable at its **code default**: gate `COAST_LOOKAHEAD_S` 0 and `SWEEP_POWER` 0.6, flywheel `FEED_FORWARD` 0.2, `DriveToPose` gains unchanged. Dashboard values reset on every app install or restart, so tuned values only survive if they are written into the code.
- The build installed on the hub was **older than the latest commits**: the Dashboard listed no `TeleOpCompetition`, and `SWEEP_POWER` still read 0.6 (the repo has 0.3).

So the numbers to commit (below) still need to be recorded, and the competition TeleOp has not been run yet.

## Values still to write into the code (Phase 5, "final values committed")

Anything tuned live is lost on the next install. For each, record the final value and commit it: gate `TICKS_PER_REV`, `ENCODER_SIGN`, `SWEEP_POWER`, `APPROACH_POWER`, `STOP_EARLY_DEG`, `COAST_LOOKAHEAD_S`, `REJECT_POWER`; flywheel `FEED_FORWARD` and `ENCODER_SIGN`; `Mechanisms` manual setpoint start and fire timeout; the shot map points; depollinator `CLEAR_DISTANCE_IN`; any drive, intake, gate or flywheel direction changes in `ROBOT_1`.

---

## Where we are starting from

| Area | Status | Source |
|---|---|---|
| Mecanum drive, Pinpoint (prototype chassis) | Verified: directions, pod directions, offsets tuned, 48 in verify autos acceptable | earlier sessions |
| Pinpoint on this robot | **Not verified.** DRAFT_1 pod directions were verified, but its **offsets are the prototype's**. Footprint measured 2026-10-31: **17 deep x 13.75 wide** | `ChassisProfile.ROBOT_1` |
| Gate | Bench-tested: thresholding, salvo/reject, lands 5-12 deg past HOLD. `COAST_LOOKAHEAD_S` still 0 | bench |
| Flywheel | Bench-tested: direction, top speed about 2200 ticks/s. `FEED_FORWARD` not tuned; second motor removed | bench |
| Intake | **Never run on hardware.** `intakeDirection` is marked unverified | `ChassisProfile` |
| Depollinator | **Never run.** All four servo positions are NaN, so it will not move | `ChassisProfile` |
| Shot map | **Empty** (coordinator falls back to the manual setpoint) | `ShotMap.TABLE` |
| Coordinator + TeleOp Draft | **Never run on hardware** | `Mechanisms`, `TeleOpDraft` |
| Autos | Use **simulated shots** (delays), not the real mechanisms | `AutonHiveVolley`, `AutonBlueStudents` |
| TeleOp | **TeleOp - Competition** written 2026-10-31: field-oriented drive with a heading reset, two controllers, pose carried over from the auto through `PoseStorage`. **Not yet run on the robot** (`TeleOpDraft` stays as the older robot-oriented draft) | `TeleOpCompetition` header |
| Hub id | Unregistered (see the decision in Phase 0) | `ChassisProfile` |

---

## Risks to settle first

These come from reading the code; each one can make a later test fail for a reason that looks like something else.

1. **RESOLVED 2026-10-31: the coordinator no longer needs the gate to land within 8 deg of HOLD.**
   `Mechanisms.update` used `gate.atHold()` to finish a salvo, to start the next one, and to allow the intake. The gate rests a few degrees past HOLD after a salvo (5-12 deg on the bench), and `HOLD_TOLERANCE_DEG` is 8, so a salvo that rested past 8 deg locked the coordinator out ("SALVO running" forever, no intake, no next fire), and nothing drives an overshoot back. **Fix (done):** the intake is no longer gated on the gate position at all (rule 1 removed), and fire and "salvo finished" use `gate.isStopped()` (idle and not moving, wherever it rests). A salvo is a full turn from the nearest HOLD, so it works from any resting position. Landing close to HOLD is still worth tuning (Phase 2.3) but nothing locks up if it does not.
2. **`COAST_LOOKAHEAD_S` includes loop delay, so it depends on the loop time.** The bench loop was 27 ms. The full robot adds the Pinpoint I2C read, four drive writes and the intake. Calibrate the gate landing **in the final TeleOp loop** (Phase 4), not only in Shot Test.
3. **The gate's zero is "assumed at HOLD" at init.** The encoder is relative. The gate must be physically at HOLD when the robot is initialized (or after an auto, the saved reference is used). Write this into the pit power-on checklist (Phase 7), and test the failure: initialize with the gate away from HOLD and see what the driver sees.
4. **Registering a hub changes what the bench hubs do.** Once any hub id is registered in `ChassisProfile`, every unregistered hub **throws** instead of falling back to the default. The temporary gate and flywheel bench hubs must keep running until the benches are retired. See the Phase 0 decision.
5. **Battery sag and brown-out.** The flywheel runs at full power below its setpoint, plus four drive motors, intake and gate. Log the voltage during a spin-up while driving (Phase 3.2).
6. **Pose accuracy near the flowers.** The "stow only when clear of a flower" rule trusts the Pinpoint pose and the flower coordinates read from the manual (about 1 in, wall offset estimated). Test it at all four flowers (Phase 4).

---

## Phase 0: Prepare

- [ ] **Branch.** Create `integration` from `flywheel-bench` (it already contains `main` plus the gate and flywheel work). `main` stays as is until Phase 3 passes. *Pass: `git log` shows the gate and flywheel commits on the branch.*
- [ ] **Hub configuration.** See [hub-configuration.md](hub-configuration.md) for the step-by-step checklist and the **Hardware Check** opmode that verifies it. On the robot's Control Hub, create one configuration with exactly these names (the code looks them up by name):

  | Device | Name | Type |
  |---|---|---|
  | Drive | `leftFront`, `leftBack`, `rightFront`, `rightBack` | motors |
  | Odometry | `pinpoint` | goBILDA Pinpoint (I2C) |
  | Intake | `intake` | motor, no encoder needed |
  | Flywheel | `fly_motor_enc` | motor with encoder |
  | Gate | `gate` | motor with encoder |
  | Depollinator | `depolLeft`, `depolRight` | servos |

  `fly_follow` is no longer used. *Pass: the Driver Station shows the configuration active, no "device not found" on any opmode init.*
- [ ] **Port map.** Write down (and photograph) which port each device is on, and which hub. If an Expansion Hub is used, note it; its commands cost extra loop time.
- [x] **Decision: robot profile and hub id.** Done: a new `ROBOT_1` profile (copied from `DRAFT_1`, to be tuned in Phases 1-2) is registered to the robot's hub (serial `5462f57f0f18d316`, Android id `7c789032eb35f27b`). The gate and flywheel benches are retired.
- [ ] **Register the other test chassis' hub.** Because a hub is now registered, the remaining test chassis' hub **throws** ("Unrecognized Control Hub") on its next opmode run until its ids are added to a profile (`PROTOTYPE` or `DRAFT_1`, whichever it is). Run **Chassis Info** on it (it still runs unregistered) or connect it over USB and read the ids, then add them.
- [ ] **Safety setup.** Robot on blocks; gate and flywheel guarded; stop procedure agreed (who holds the Driver Station Stop); battery strap and wiring strain relief checked.
- [ ] **Record sheet started** (end of this file).

Exit: configuration active, profile decision made, branch created.

---

## Phase 1: Walk-through (no code, then a smoke test)

- [ ] **Wiring.** Every connector seated and strain-relieved; no wire near the gate arm, flywheel or intake; Pinpoint cable and pods secure; no wire that can be caught by the depollinator ramp.
- [ ] **Mechanical.** Gate arm turns freely through a full turn by hand with no element in the track; flywheel spins freely by hand; intake rotates freely; ramp servos move without binding (by hand with power off, gently). Fasteners tight; thread locker where appropriate.
- [x] **Size.** (Confirmed 2026-10-31: the robot fits the 18 in cube with the depollinator stowed.) With the depollinator **stowed**, the robot fits the starting cube (rule R102, 18 x 18 x 18 in) in the starting configuration. Footprint **measured: 17 deep x 13.75 wide** and entered into `ROBOT_1` (the autos, the TeleOp and the shot-map tuner all derive their start X from the depth, so the start is now x = -63.5). The height was not recorded as a number, only confirmed to fit.
- [ ] **Power-up smoke test.** Battery in, hub boots, Driver Station connects, **Chassis Info** shows the expected profile, nothing moves on its own. *Pass: no unexpected motion, no warnings.*

Exit: robot powers up safely and the footprint is recorded.

---

## Phase 2: Each subsystem on the real robot

Run the existing bring-up opmodes. The bench numbers are a starting point, **not** a result: the robot's gearing, loading and wiring can differ.

### 2.1 Drive and localization
- [ ] **Motor Map Test**: every wheel moves the robot in its labeled direction (forward for the forward test). Fix directions in the profile if not.
- [ ] **Open Loop Test (D-pad)**: clean forward / back / strafe / rotate, no cross-coupling. Record the drift for each.
- [ ] **Pinpoint Push Test**: every step passes (pod directions and axis signs), following the opmode's own prompts.
- [ ] **Pinpoint Offset Check**: tune `xOffsetMM`, `yOffsetMM` **for this robot**. Pass: the wobble and the forward and strafe error estimates are as small as on the prototype (its tuned result was a wobble of 0.72 in with estimates near 0).
- [ ] **Verify autos** (Auton Verify - Forward 48 and back, and Lateral 48 and back): final error acceptable to the team. Record the numbers; the prototype's results were judged acceptable, so use those as the bar.
- [ ] **Tuners** (Axial, Lateral, Heading at 48 in): gains adjusted until the verify autos settle without oscillation.

### 2.2 Intake
- [ ] **Direction**: positive intake power pulls elements **in** (fix `intakeDirection` if not). Outtake (-0.5) pushes out.
- [ ] **With real elements**: both pollen and nectar feed in cleanly, no jam on the way to the gate, none escape behind the robot.
- [ ] **Record intake current** while running free and while pulling an element, for a future jam detector (the team decided the driver handles jams for now but wants current-based detection later).

### 2.3 Transport gate
- [ ] **Sign check** (Transport Gate Test): a positive command counts forward revolutions; velocity sign right. Set `ENCODER_SIGN` if not.
- [ ] **TICKS_PER_REV** with the 10-turn procedure on the **installed** gate (bench value 751.8; the robot may have different gearing).
- [ ] **Mark HOLD** at the physical HOLD position (the arm at the back, limiting entry to 4).
- [ ] **Tune** `SWEEP_POWER`, `APPROACH_POWER`, `STOP_EARLY_DEG`, then `COAST_LOOKAHEAD_S` from the "Gate last settle" suggestion. Do this **with the gate loaded**: 0, 1, 2 and 4 elements in the track. Loaded salvos can need more power.
- [ ] **Reject** with elements in the track: reverses and finishes on the nearest HOLD behind.
- [ ] **Exit**: 10 salvos in a row (including with 4 elements) with no fault, each resting consistently near HOLD (read "Error to nearest HOLD" and "Gate last settle"; the target is a few degrees, but a salvo that rests a little past HOLD no longer locks anything out, see Risk 1). Record the min and max landing error.

### 2.4 Flywheel
- [ ] **Direction and sign**: spins the shooting way, speed reads positive (the bench fix flipped the flywheel direction; re-confirm on the robot).
- [ ] **FEED_FORWARD** tuned at about 600, 1000 and 1500 ticks/s so the hold power is just enough (single motor now).
- [ ] **Spin-up time** to ready at the speeds you expect to shoot at; **dip and recovery** over a 4-ball burst (use the Y measurement). Record; this is the data that tells us whether one motor is enough.

### 2.5 Depollinator
- [ ] **Measure** the four positions (left/right stowed and deployed) with **Depollinator Test** and enter them in the profile. Both servos must be in the same mode (positional).
- [ ] **TRAVEL_SEC**: time from command to settled; set with margin.
- [ ] **Stowed** position keeps the robot inside the starting cube; **deployed** reaches the flower opening without touching the floor.
- [ ] **CLEAR_DISTANCE_IN**: measure the real ramp reach from the robot center; set it from that, not the 24 in placeholder.

Exit: each subsystem passes its own checklist, all numbers are in the Record sheet and committed.

---

## Phase 3: Mechanisms together (no coordinator yet)

- [ ] **Shot Test on the robot.** Real elements (both kinds) through the gate into the flywheel. Salvo (Y) and dry salvo (RB), reverse (X). Record per speed: speed at release, lowest speed during the salvo, whether all four elements leave and in what order. *Pass: repeatable, no gate fault, no element stuck.*
- [ ] **Intake feeding the gate**: run the intake with the gate at HOLD; elements stop at the gate arm, exactly 4 fit, a 5th is held out. (Rule G407: at most 4 controlled.)
- [ ] **Everything powered while driving.** Drive around while the flywheel spins up and the intake runs. Record the **battery voltage minimum** and the **loop time**. *Pass: no brown-out, no hub reset, no pose jumps from the Pinpoint while the flywheel spins.*
- [ ] **Loop time**: record it. If it is much above 27 ms, note which opmode and what was running (Risk 2).

Exit: the mechanisms work together without a coordinator, and loop time and voltage are recorded. Merge `integration` into `main` here if you want a stable base.

---

## Phase 4: Coordinator and TeleOp (the five rules)

Run **TeleOp - Draft (mechanisms)**. Each row passes three times in a row.

| # | Test | Expected |
|---|---|---|
| R1 | Hold intake (operator RT) while the gate is mid-salvo, resting past HOLD, or anywhere else | The intake runs regardless of the gate's position (rule 1 removed) |
| R1b | Initialize with the gate not at HOLD (Risk 3) | Behavior understood and written into the pit checklist |
| R1c | Fire several salvos in a row, the gate resting a few degrees past HOLD each time | Each fires; nothing sticks on "SALVO running"; the flywheel auto-stops after each |
| R2 | Deploy (X), then stow (Y) | Deploy turns the intake on; stow turns it off unless RT is held |
| R2b | Deploy with the gate away from HOLD | Ramp deploys and the intake turns on (the intake is not gated on the gate) |
| R3 | Fire (A) with the flywheel cold | Spins up, then fires the moment it is ready |
| R3b | Fire with the flywheel already spinning | Fires at once |
| R3c | Fire when the flywheel cannot reach the setpoint (set it too high) | Drops after the timeout with a NOTICE, no stuck state |
| R3d | After a salvo | The flywheel stops about 0.5 s later by itself |
| R4 | Reject (B) with elements | Gate reverses with outtake for the whole move, then both stop |
| R4b | Press reject or fire while a salvo or reject is running | Ignored, no stuck state |
| R5 | Stow (Y) while under each of the **four flowers** | Stow waits ("Stow waiting"), then happens once the robot drives clear |
| F1 | Gate fault (Back clears) | Warning shown, salvos blocked until cleared, clears cleanly |
| F2 | Flywheel encoder fault | Flywheel stops with a message; clears with Back |
| F3 | Remove a mechanism from the hub configuration (for example the depollinator) | Coordinator keeps running with that mechanism skipped |
| D1 | Drive while firing | Pose stays correct, mechanisms respond, no lockups |
| D2 | Open-cell switch (D-pad left/right) after a tip | Distance and setpoint now aim at the other side's cell |

Also in this phase:
- [ ] **Re-tune `COAST_LOOKAHEAD_S` in this loop** (Risk 2) and confirm the salvo lands consistently near HOLD (Risk 1 is resolved, so this is about repeatability, not a lock-out).
- [ ] **Driver and operator feel**: precision mode (30%), turn scale, trigger thresholds, button layout. Collect changes from the drivers; do not change several at once.
- [ ] **Alliance select at INIT** works for both alliances and sets the start pose and the aimed HIVE.

Exit: every row passes three times; the gate lands inside what the coordinator needs.

---

## Phase 5: Calibration data

- [ ] **Shot map** (Shot Map Tuner): record (distance, setpoint) pairs from the closest legal shooting position to the farthest you intend to use, for each target CELL that matters. Validate each point with a full 4-ball salvo and count how many score. Paste the lines into `ShotMap.TABLE`. Record the battery voltage.
- [ ] **Same speed for pollen and nectar** (the team's decision): confirm both score from the same setpoint, or note the exception.
- [ ] **Final values committed**: gate powers and `COAST_LOOKAHEAD_S`, `FEED_FORWARD`, depollinator positions and `TRAVEL_SEC`, offsets, footprint. Re-run Phase 4's smoke rows afterward.

Exit: the shot map is populated and the coordinator no longer warns "Shot map empty".

---

## Phase 6: Autos on the real mechanisms

- [ ] **Wire the real mechanisms** into `AutonHiveVolley` and `AutonBlueStudents` in place of the simulated delays (`Mechanisms.fire()`, `reject()`, `setIntake()`, `deploy()`, `stow()` exist for this; the coordinator's `update` goes in the per-loop callback). The gate reference for autos is `ASSUME_AT_HOLD`.
- [ ] **Update `tools/autosim`** if the paths or timings change, and re-check the legality and 4 s leg timeouts.
- [ ] **Start-pose legality** at the field: robot touching the wall, inside its alliance half, not in the loading zone or flower volume (rule G304), in the starting configuration with the depollinator stowed.
- [ ] **Three consecutive runs of each auto**, from the legal start, without a fault.
- [ ] **Auto to TeleOp handoff**: the autos save their final pose to `PoseStorage` and **TeleOp - Competition** starts from it (the INIT screen says "AUTO pose, saved N s ago"; Back on gamepad 1 discards it), keeping the gate reference (`CONTINUE`). Test: run an auto, then immediately TeleOp; pose and gate zero are still right.

Exit: both autos complete three times; the handoff keeps pose and gate zero.

---

## Phase 7: Match readiness

- [ ] **Full-match rehearsals** (auto, then TeleOp for the full period, three times) with the drivers, including at a **mid-drained battery** (recheck the gate landing and a shot or two).
- [ ] **Thermal and wear**: after a rehearsal, check the gate, flywheel and intake motors and drivetrain are not hot; check fasteners and wires again.
- [ ] **Inspection list** against the current manual: size in the starting configuration, wiring, battery mounting, hub configuration, anything the manual requires. (This plan does not restate rules; read them from the manual.)
- [ ] **Pit power-on checklist** (one page): battery, hub config selected, **gate at HOLD before init**, depollinator stowed, alliance chosen, pose seeded, Dashboard optional.
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
| Gate `TICKS_PER_REV` | | `TransportGate` | |
| Gate `SWEEP_POWER` / `APPROACH_POWER` | | `TransportGate` | |
| Gate `STOP_EARLY_DEG` / `COAST_LOOKAHEAD_S` | | `TransportGate` | |
| Gate landing error over 10 salvos (min / max deg) | | notes | |
| Flywheel `FEED_FORWARD` | | `Flywheel` | |
| Flywheel spin-up to ready (s) at speeds | | notes | |
| Flywheel lowest speed in a 4-ball burst (% of setpoint) | | notes | |
| Depollinator stowed / deployed L, R | | profile | |
| Depollinator `TRAVEL_SEC`, `CLEAR_DISTANCE_IN` | | `Depollinator` | |
| Loop time: Shot Test / TeleOp (ms) | | notes | |
| Battery minimum under load (V) | | notes | |
| Shot map points | | `ShotMap.TABLE` | |
