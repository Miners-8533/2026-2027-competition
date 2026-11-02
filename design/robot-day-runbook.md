# Robot-day runbook

What to do, in order, when the robot is in front of us. It strings together the checks, calibrations and tests that are spread over [integration-plan.md](integration-plan.md), [pit-check.md](pit-check.md), [hub-configuration.md](hub-configuration.md) and [run-logs.md](run-logs.md), so nobody has to hunt for the next step. The plan has the pass criteria and the record sheet; this is the sequence.

**Ground rules** (from the plan, repeated because they matter most)
- **Wheels off the floor** (robot on blocks) for the first run of anything. The **pusher has no hard stops** and the **flywheel is exposed**: hands, sleeves and cables clear, eyes protected.
- **One change at a time**, then re-test. A test that fails gets a one-line note (what, what was on screen) before anyone edits code.
- **A fresh, charged battery** for calibration; write the voltage next to every number you record.
- **Tuned values live only in the code.** Dashboard values reset on every install or app restart. Write each number into the record sheet as you get it, and commit the batch at the end of each block.
- **Never bind anything to Start** (Driver Station gamepad assignment uses Start + A and Start + B), and no driver stick-button controls.

---

## 0. Before leaving the laptop

- [ ] On the `integration` branch, working tree clean, `git log -1` noted (write the commit id on the record sheet).
- [ ] Build and **install the latest build** on the hub. Confirm the opmode list shows **Pit Check (robot on blocks)** and **TeleOp - Competition**. (An older build was on the hub last time: check this, not just that an install happened.)
- [ ] The hub's saved configuration is `ROBOT_1`, active. Chassis profile reads `ROBOT_1` on any INIT screen.
- [ ] To carry: USB cable for the hub (log pulls), blocks, a tape measure, a few pollen and both colors of nectar, two charged batteries and the charger, spare color-sensor cable, a laptop with the repo and adb.

## 1. Every power-up (the pit card)

Do this each time the robot is powered on or the app restarted, before INIT of anything that moves a mechanism.

- [ ] Battery charged and seated; note the voltage.
- [ ] Hub configuration `ROBOT_1` selected and active.
- [ ] **Pusher arm at HOLD** (the back of the arm limiting entry to 4 elements). The zero is assumed here at INIT.
- [ ] **Ramp stowed**, inside the starting cube. Intake and track **empty** (except the 4-element preload for an auto).
- [ ] Both gamepads assigned: **Start + A** (driver), **Start + B** (operator) on the Driver Station.
- [ ] For an auto or a TeleOp: robot placed at its start pose against the wall, **still** from INIT until Start (the Pinpoint recalibrates at INIT). With no auto, the TeleOp's default start is blue (-63.5, +16) against the rear wall facing +X, red (+63.5, -16) against the audience wall facing -X (the front, the intake and shooter end, faces away from the wall).
- [ ] INIT screen: Pinpoint `OK`, the right mechanisms present, element counter `reading`. Anything else, stop and fix it before Start.

## 2. The day, in order

Each block ends with: pull the logs, write the numbers on the record sheet, commit.

### Block A: Pit Check (robot on blocks)

1. Run **Pit Check (robot on blocks)**. Track empty, ramp free to swing. Press A for each moving step; B if what you saw was wrong; X to skip; Back stops everything.
2. Fix every FAIL using the fix text shown, then **rerun until `PIT CHECK ALL PASS`**. A skipped step is not a pass.
3. If the pusher return fails, put the arm back at HOLD by hand before anything else.
4. Pull `pitcheck.log` (see "Getting the logs") and keep it with the record sheet.

Likely first-run surprises: the 0.3 power flywheel step may not reach 100 ticks/s (tune `FLYWHEEL_POWER` or the threshold in the Dashboard); the color-sensor baseline may not match today's lighting (Block B fixes that).

### Block B: Element counter baseline (item 5)

1. Run **Color Sensors Test (CS1-CS4)** with the track **empty**. Press **A**.
2. From `adb logcat -s ColorSensors`, copy the `BASELINE` line into the table in `ElementCounter` (`BASE_ALPHA`, `BASE_DIST`, `BASE_SHARES`).
3. Rebuild and reinstall, then rerun **Pit Check** (its first, automatic part takes a few seconds): CS1 to CS4 must now read empty.
4. Red nectar: put red nectar in front of CS2, CS3 and CS4 and confirm it still reads red (the live re-check that is still open).

Redo this in the pit at each venue if the lighting changes.

### Block C: Pusher calibration (items 3, 4 and `COAST_LOOKAHEAD_S`)

1. Run **Pusher Sensor Matrix (CS4)**: jog the arm to HOLD, **A** to mark it, **right bumper** to run (about a minute, default 3x gain). Read `adb logcat -s PusherSensorMatrix`.
2. Record the **"ARC ... ahead of the mark"** at powers 0.15, 0.3 and 0.6: the team expects the arm to be seen about 90 deg ahead of HOLD. This is the HOLD to CS4 offset.
3. **`TICKS_PER_REV` check:** the arc's RISE and CENTER spread from turn to turn should be small (the matrix measured 0.07 / 0.39 / 1.01 deg at powers 0.15 / 0.3 / 0.6). If the arc drifts turn after turn, `TICKS_PER_REV` is wrong.
4. **`COAST_LOOKAHEAD_S`:** from the logs of these runs and the settle readouts in Shot Test, either find a value that makes the landing consistent or remove the lookahead. Record the decision.
5. **Item 4:** choose the smallest belt slip worth catching, in degrees (a skipped tooth is 360 / the number of teeth on the pusher pulley). It decides whether the parked check (about +/-15 deg) is enough.
6. Put the arm back at HOLD.

7. **Pusher Check Test (CS4)** (the pusher check, built from these numbers): the arm at HOLD at INIT, track empty. **A** runs it (expect OK in about 1-2 s); **X** or **Y** fakes a one-tooth slip, then **A** (expect one continuous sweep that finds the arm, corrects the zero by about 18 deg and returns to HOLD in 2-6 s), then **A** again (expect OK); **RB** fakes five teeth, then **A** (the same, correcting about 90 deg). Rows P1 to P6 in the plan. If the arm is not at the arc, tune `PusherCheck.ARM_CENTER_DEG` in the Dashboard and commit the value (the sweep needs no delay constant: it averages a forward and a return crossing).
8. If the check is slow or the arm is misjudged, `Mechanisms.AUTO_PUSHER_CHECK` (Dashboard) turns the automatic check off.

### Block D: Shot Test with real elements (plan Phase 3)

1. **Shot Test (pusher + flywheel)**: set HOLD with the left stick and **B**, flywheel on with **A**, set the speed with the D-pad.
2. With real elements (pollen and both nectars): **Y** = fire (waits for the flywheel), **RB** = dry salvo, **X** = reverse. Record per speed: speed at release, lowest speed during the salvo, whether all four leave and in what order.
3. Tune `FEED_FORWARD` (Dashboard) so the hold power is just enough at about 600, 1000 and 1500 ticks/s. Tie it to the minimum shot speed.
4. Record the pusher's settle after each salvo (the `Pusher last settle` readout) over **10 salvos in a row**, including with 4 elements: min and max landing error, no fault.
5. Intake feeding the pusher: with the pusher at HOLD, exactly 4 fit and a 5th is held out.
6. **Avoid LB (return to HOLD) when the arm rests more than 8 deg past HOLD**: it turns nearly a full turn forward and fires the stack. This is an open decision (see "Open decisions").
7. Everything powered while driving: drive with the flywheel spinning and the intake running. Record the **minimum battery voltage** and the **loop time**; no brown-out, no Pinpoint jumps.

### Block E: TeleOp - Competition (plan Phase 4)

Run first with the robot on blocks, then on the floor. Each row of the plan's Phase 4 table must pass **three times in a row**. In this order:

1. INIT checks: alliance select for both alliances; Pinpoint OK; the INIT screen shows the mechanisms and the counter.
2. Drive: field-oriented stick up = away from the driver on both alliances; precision mode; **hold left + right bumper 1 s** reseeds the pose (also the fix for a wrong heading). If a stick direction is wrong at the field, correct `BLUE_DRIVER_FORWARD_DEG` / `RED_DRIVER_FORWARD_DEG`.
3. Intake toggle (operator RT), outtake (LT held), deploy (X) and stow (Y), including **stow under each of the four flowers** (it must wait, then release when clear). Measure the real ramp reach and set `CLEAR_DISTANCE_IN` (placeholder 24).
4. Fire (A) cold and warm, fire when the setpoint is too high (it must drop after the timeout, not stick), reject (B), the flywheel toggles (RB / LB), the setpoint D-pad.
5. **Element counter rows C1 to C5:** 1 to 4 elements fill from CS4; the 4th stops the intake about half a second later and clears the toggle; a salvo from full drops the count to 0; opponent nectar raises a warning only. Note the loop time with the counter on.
6. **Jam handling** (the pusher does not fault for now: B cancels a salvo, A cancels a reject), a salvo chain with the arm resting past HOLD (each must fire; none sticks).
7. **Overrides (F2):** operator L3 (counter), operator R3 (flywheel open loop). **Backup mode** (no way back within a run): driver holds LB + X for 1 s, or a Pinpoint fault lasting 0.5 s: robot-oriented drive, fixed shot speed for 42 in, no target lock, three rumble taps on both pads. Each shows on screen and as an `OVERRIDE` log line, and pressing again restores the normal behavior. In open loop, compare `fly_vel` in the log to the target to see whether power = target / 2200 is a usable speed.
8. Drive while firing: pose stays correct, nothing locks up. The target cell is picked automatically (the nearer of your alliance's two cells, switching only when the other is 6 in closer): drive across the middle and watch it switch once, not flip-flop.
9. Driver and operator feel (precision scale, turn scale, trigger thresholds, layout): collect changes from the drivers, change one at a time.

### Block F: Shot map (plan Phase 5)

1. **Shot Map Tuner** (single gamepad; it runs the real mechanisms since 2026-11-01): right trigger loads elements (the intake stops itself at 4), **X** spins the flywheel, D-pad up/down sets the speed, **Y** fires a salvo, **B** records (distance, speed), **A** undoes the last point, D-pad left/right picks the target cell, left bumper follows the recorded points, Back clears a pusher fault (hold it 1 s to reseed the pose).
2. For each distance, from the closest legal shooting position to the farthest you intend to use, for each target cell that matters: drive there (the distance is the Pinpoint's, or type a tape-measured one into `MANUAL_DISTANCE_IN`), load 4, fire, and raise or lower the speed until the shots score; validate with a full 4-element salvo and count the scores; then **B**.
3. Paste the printed lines (also in `bringup.log` as `RECORD` and a final `SHOT MAP` event) into `ShotMap.TABLE`. Confirm pollen and nectar both score from the same speed, or note the exception.

### Block G: Pinpoint offsets and drive PID (autos only)

1. **Pinpoint Push Test**: every step passes.
2. **Pinpoint Offset Check**: tune `xOffsetMM` and `yOffsetMM` for this robot (the profile still has the prototype's). Also check that after the spins the robot faces the way it started.
3. **Axial, Lateral and Heading tuners** at 48 in, then the two **Auton Verify** opmodes. Record the final error; the prototype's results are the bar.

### Block H: Autos (plan Phase 6)

1. Check the start pose at the field: touching the wall, inside the alliance half, not in the loading zone or flower volume, ramp stowed, **4 elements preloaded**, pusher at HOLD.
2. Run **Auton - Hive Volley (Blue)** and **Auton - Blue Students** three times each from the legal start, without a fault. Watch the INIT screen: Pinpoint OK, preload `4 elements seen`.
3. Time each against the auto period (the real volley and ramp waits are in the log).
4. **Handoff:** run an auto, then **TeleOp - Competition** immediately. The INIT screen should say `AUTO pose, saved N s ago`, and the pose and the pusher zero should still be right after the first salvo.

### Block I: Rehearsal and freeze (plan Phase 7)

- Full-match rehearsals (auto then the full TeleOp period), three times with the drivers, including at a **mid-drained battery** (recheck the pusher landing and a shot or two).
- Check motor and drivetrain temperature, fasteners and wires.
- Run **Pit Check** one last time; it must read `ALL PASS`.
- Inspection list against the current manual, then freeze: tag `integration-v1`, merge into `main`, keep a copy of the build used.

---

## 3. Getting the logs (hub on USB)

```bash
adb pull /sdcard/FIRST/runlogs runlogs
```

Gives `pitcheck.log`, `auto.log`, `teleop.log` and their `*_prev.log` (only the last two runs of each kind are kept: pull before a third run). For the test opmodes' own logs:

| Command | What |
|---|---|
| `adb logcat -d -s PitCheck` | pit check results |
| `adb logcat -d -s ColorSensors` | element counter baseline and readings |
| `adb logcat -d -s PusherSensorMatrix` | the pusher arc and ranking |
| `adb logcat -d -s HardwareCheck` | the read-only hardware report |

Then ask Claude Code to read the folder. After a crash, the first `#` line of the new log says how the previous run ended.

The FTC Dashboard (live tunables) is at `192.168.43.1:8080/dash` over the hub's Wi-Fi, or run `adb forward tcp:8080 tcp:8080` and open `localhost:8080/dash` over USB.

## 4. Numbers to write down, then commit

Fill the plan's **record sheet**, then write these into the code and commit:

| Value | File |
|---|---|
| Pusher `TICKS_PER_REV`, `SWEEP_POWER`, `APPROACH_POWER`, `STOP_EARLY_DEG`, `COAST_LOOKAHEAD_S`, `REJECT_POWER` | `Pusher.java` |
| Flywheel `FEED_FORWARD`, `ENCODER_SIGN` | `Flywheel.java` |
| `Mechanisms` manual setpoint start (now 1350) and `FIRE_TIMEOUT_SEC` | `Mechanisms.java` |
| Shot map points | `ShotMap.TABLE` |
| Element counter baseline (`BASE_ALPHA`, `BASE_DIST`, `BASE_SHARES`) | `ElementCounter.java` |
| Ramp `CLEAR_DISTANCE_IN` | `Ramp.java` |
| Pinpoint offsets `xOffsetMM`, `yOffsetMM` | `ChassisProfile.ROBOT_1` |
| DriveToPose gains | `DriveToPose.java` |
| Driver feel (`TURN_SCALE`, `PRECISION_SCALE`, thresholds) | `TeleOpCompetition.java` |

Rebuild and reinstall after committing, and confirm the Dashboard shows the new values.

## 5. Controls card

**INIT (gamepad 1):** D-pad right = BLUE, D-pad left = RED (only when there is no auto pose); D-pad up = discard the auto pose.

**Driver (gamepad 1):** left stick drives (field-oriented, up = away from the driver); right stick X turns; right bumper held = precision (30%); **A** = fire (also cancels a running reject; works with any number of elements); **target lock engages by itself when the robot is full** and stays on through the salvo (the robot turns to point its front at the open cell while you keep driving; hold the left bumper and use the right stick to nudge the aim up to 15 deg (the stick does nothing while locked without the bumper); a buzz and ALIGNED mean on target); **hold left + right bumper 1 s** = reseed the pose; **hold left bumper + X for 1 s** = BACKUP MODE (robot-oriented drive, flywheel fixed for 42 in, no target lock, no way back this run).

**Operator (gamepad 2):** the intake is automatic (IN until 4 elements, then OUT); **hold right trigger** = force IN even when full; **hold left trigger** = force OUT; **A** = target lock by hand, on / off, full or not (a finished salvo releases any lock); **hold left bumper + B** reject (also cancels a running salvo: the way out of a jam); **X** deploy ramp; **Y** stow ramp (at once, no flower check); **hold left bumper + Y** pusher calibration sweep (track empty, intake off); the flywheel has no control (it runs from Start and never stops); **hold left bumper + D-pad up/down** = speed trim +/- 25 ticks/s on the shot map (the target cell is the nearest of the alliance's two, automatically); (the pusher does not fault for now, `Pusher.FAULTS_ENABLED` = false); both controllers rumble once when the robot becomes full (a rumble pad only; the F310 has none); **left stick button** = ignore the element counter; **right stick button** = flywheel open loop.

## 6. If something is wrong

| What you see | Likely cause | Where to look |
|---|---|---|
| A mechanism does nothing, no error | Misspelled or missing config name (the coordinator skips it silently) | INIT screen "Mechanisms present"; Pit Check "Configuration" |
| Pinpoint not OK on INIT | Robot moved after INIT, pod or I2C cable, or the start pose is wrong | restart the opmode with the robot still; Pit Check "Pinpoint" |
| "ELEMENT COUNTER NOT READING" | A color sensor dropped off its bus or the cable is loose | Pit Check lines CS1-CS4; use the operator L3 override in a match |
| Count wrong or "FULL" with an empty track | The baseline does not match today's lighting | Block B |
| Fire is dropped ("flywheel not ready in time") | Setpoint too high, battery low, or the encoder is dead | `fly_vel` in `teleop.log`; Pit Check "Flywheel encoder"; operator R3 override |
| Pusher angles or salvos look off after a run | Stale zero (belt slip, or the arm moved) | the pusher check corrects it after the next salvo (`PUSHER CHECK` lines in the log; a LOST warning means it could not); otherwise put the arm at HOLD and restart the app |
| "SALVO running" never ends, or "Pusher STALLED" | Jam (the pusher does not fault for now) | B cancels a salvo, A cancels a reject; read `last settle` and the log's `PUSHER` lines |
| Field-oriented sticks feel wrong | Heading drift or the wrong driver-forward direction | hold LB + RB to reseed the pose (resets the heading); hold LB + X for backup mode if the Pinpoint is bad; correct `*_DRIVER_FORWARD_DEG` |
| An opmode throws "Unrecognized Control Hub" | This hub is not registered in `ChassisProfile` | run Chassis Info and add its id |
| Values you tuned are back to the old ones | They were only in the Dashboard; the app was reinstalled or restarted | Block 4: write them into the code |

## 7. Open decisions to settle on the day

- **`Pusher.hold()` / Shot Test LB:** turns nearly a full turn forward when the arm rests more than 8 deg past HOLD (fires the stack); salvos land 5-12 deg past. Decide the behavior.
- **`COAST_LOOKAHEAD_S`:** find a value from the logs, or remove the lookahead.
- **Smallest belt slip to catch (item 4):** picks the pusher check's tolerance.
- **Pusher zero in TeleOp:** it trusts the saved zero; the automatic pusher check now corrects a slip after a salvo (not yet run on the robot).
- **Autos' time budget** with the real mechanisms against the auto period, and whether to stow at the shooting spot without the pose check (it does now) once the real ramp reach is known.
- Whether the operator's stick-button overrides are too easy to press by accident.
