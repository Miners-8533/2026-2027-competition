# Robot test checklist, 2026-11-05 (build `integration` d094a54)

Everything below has never run on the robot. Rows (T1..) are in integration-plan.md. Stop and note it if anything looks wrong; do not change several things at once.

## 0. Setup
- [ ] Flash `integration`, battery fresh, Dashboard open (192.168.43.1:8080/dash), a spotter at the robot
- [ ] Dashboard values appear at the start of the log; INIT: no twitch of the pusher
- [ ] Ramp at INIT (new, T12): run an auto or the ramp deployed (X) then stop the opmode, so it sags. Press INIT on the TeleOp: the ramp does not move and the screen says "position unknown". Press Start: it stows (log: `COMMAND ramp stow at Start`)

## 1. Color Sensor Study (first: the transport was recolored, so the baselines are probably stale)
- [ ] Pusher at HOLD, track empty. Step 1: A, 8 s, robot still. Write down the 3 BASELINE lines
- [ ] Step 2 (empty sweep): the arm reads PUSHER_ARM at CS4; nothing else lit that should not be
- [ ] Hold X to load 4 pollen. Step 3: all four sensors lit with 4 pollen
- [ ] Step 4 (slow reject): pollen read POLLEN as they pass each sensor
- [ ] Paste the baselines into `ElementCounter`, rebuild, re-flash. Check the `ARM_*` and `POLLEN_*` thresholds if steps 2 or 4 misclassified

## 1b. Pinpoint adopted after an auto (T13)
- [ ] Run Auton - Solo, then initialize the TeleOp at once: INIT says "CONTINUING from the auto", no recalibration pause, pose matches the robot; log `PINPOINT START mode ADOPTED`
- [ ] Repeat but push the robot about 6 in and turn it about 10 deg by hand first: the screen shows the difference with CHECK, and the pose follows the real robot
- [ ] TeleOp with no auto (app restarted, or over 5 min): reset and seeded to the default start as before; D-pad up in INIT after an auto reseeds the default start
- [ ] Save the TeleOp and auto logs from this: the PINPOINT lines are what we debug from

## 2. TeleOp basics (T1 to T3, F4)
- [ ] Controls: driver A launches, LB chords, precision on LT, target lock on RB (new 2026-11-05), reseed LB + RT (was LB + RB); operator A target lock, LB+B reject, LB+Y sweep
- [ ] Intake always on: IN under 4, OUT at 4; one rumble when full; the count stops flickering 3 / 4
- [ ] Target lock on by itself when full, off after the salvo; launch at all four cells (marks: far-cell extra -3.0, offset -2.2)
- [ ] Backup mode: LB + X held 1 s (banner, three taps)
- [ ] Pusher: no faults, a launch and a reject cancel each other, count reads 0 after a salvo (P10)

## 3. Pusher lock (T6)
- [ ] Changed 2026-11-05: a full robot alone just stops the intake (arm stays at HOLD, no OUT). Hold RB (target lock) with pollen held: the arm parks about 90 deg past HOLD, intake OFF, count frozen; release RB without launching: back to HOLD after 0.5 s and the intake runs again; a Hive Dash locks it on the way
- [ ] Launch from the lock clears all four; unlock with reject works; no delay to the launch

## 4. Flywheel (F7)
- [ ] Bang-bang: speed holds at 1325 / 1350 with no creep on a fresh battery (`FEED_FORWARD` 0.35 is at the edge; lower it if the speed creeps above the setpoint)

## 5. Auto Launch (T4)
- [ ] Operator RB on, drive into the audience and rear zones: it stops, aligns, one salvo, lock releases
- [ ] Nothing with RB off, 0 elements, or outside a zone; gives up after 2.5 s

## 6. Hive Dash (T5)
- [ ] `HiveDash.DRY_RUN` true: hold D-pad left / right, check the route on the field view, nothing drives
- [ ] DRY_RUN false, `MAX_POWER` now 0.85 (was 0.5), `TURN_MAX` still 0.5: arrivals still within 2.5 in, no overshoot, no contact with a support; lower `MAX_POWER` in the Dashboard if it overshoots: both cells, both alliances, from the launch spots, a garden corner, under the hive
- [ ] No contact with a support; release cancels; stops at the spot, ALIGNED, launches; no overshoot at the last leg

## 7. Hold position (T7)
- [ ] Launch from a settled stop: `HOLD latched` line; push gently then hard, it returns; a stick cancels it
- [ ] Watch for wind-up or overshoot, and drive motor heat after a few salvos

## 8. Park Dash (T11)
- [ ] `ParkDash.DRY_RUN` true: D-pad up / down, read `PARK DASH start`
- [ ] DRY_RUN false: ramp stows at the start; ends in the loading zone corner (up = driver's left); holds while held and resists a push; release cancels
- [ ] Refused with a rumble when touching a support or in backup mode; left / right still Hive Dash; the two never run together

## 8b. Loop profile (new, every run)
- [ ] After the TeleOp and auto runs: pull the logs and run `python tools/logs/loop_profile.py` (and `--busiest` for the worst windows). Note pose, drive_write, telemetry, mechanisms (and mech.counter), other; compare the TeleOp with the 28 to 30 ms of 2026-11-03, and look at hive_dash / park_dash / hold maximums during a dash

- [ ] Drive write deadband (T14): the robot drives, creeps, stops and holds the same as before (try `MecanumDrive.writeDeadband` 0 in the Dashboard to compare); the profile's `drive_skipped` count is above 0 with steady sticks

## 8c. Controls and jostle changes of 2026-11-05 (evening)
- [ ] Driver right BUMPER (hold) is the target lock; the right trigger does nothing; LB + RB does nothing; the nudge (LB + right stick while RB is held) works and does not reseed
- [ ] Reseed is LB + right TRIGGER, hold 1 s (the screen counts it); the pose goes to the default start
- [ ] Operator D-pad left / right: the remembered aim moves 3 deg per report (max 10; log `REPORT landed ...`)
- [ ] Autos' flower gather: the log's `AUTO JOSTLE start` line says press 1.0 s first, back 0.25 in at power 0.3, press 1.0 s per cycle, at most 1 cycle (`Jostle.MAX_CYCLES`; the jostle now ends after one back-and-forth, `AUTO JOSTLE ended ... the 1 back-and-forth cycle(s) are done`); check the robot actually moves back (about 0.25 to 0.75 in) and does not stay jammed; raise `Jostle.BACK_POWER` if it does not move; check that the duo and Solo timing still fits 30 s
- [ ] Autos: the intake stays IN from the flower ram until the flower salvo is done, even at 4 held (log: `COMMAND intake forced IN (auto, through the flower salvo) -> ON`, then `off` after the salvo; `INTAKE mode` lines): pollen still coming off the ramp keep coming in while the robot backs off and aims; watch the held count (it can exceed 4: the extra stays for the TeleOp, there is no second salvo) and that the salvo still feeds cleanly with the intake running

## 8d. Pusher active hold (T15)
- [ ] Idle with the arm parked: push it 10 to 20 deg by hand, let go: it returns; `pusher_hold` is 0.05 to 0.15 while it corrects; no `GAVE UP` lines; the motor is not warm after a long idle
- [ ] Over a session: the pusher check's zero corrections are small (a few deg), salvos start from within a few deg; compare with `Pusher.HOLD_SERVO_ENABLED` false if it looks wrong

## 8e. Hive Dash under the hive (lane centering, 2026-11-05)
- [ ] Dashes that cross the lane (rear to audience and back, from the far side of the field) at 0.85: the route now goes along the middle line (log: `via (34, 0) (-34, 0)` or around a helper corner); no contact with a support, no stuck robot; note the entry speed and any drift at the gate (the `HIVE DASH sample` lines)

## 8f. Intake current watch (2026-11-05)
- [ ] Run the intake in and out with nothing in it: `intake_amps` about 1.0 A in, 1.5 A out; no `INTAKE HIGH CURRENT` / `INTAKE LOW CURRENT` lines in normal use; if the intake sticks again, the log shows the current and the event lines; adjust `Intake.HIGH_AMPS` / `LOW_AMPS` from what a real jam reads

## 8g. Solo final gather (T16)
- [ ] `Auton - Solo`, blue and red: after the flower salvo it crosses outside the hive to the alliance-wall flower, rams, jostles one cycle, backs out, and parks in the hive-side corner; note the finish time (nominal about 29.8 s) and whether the park is complete before 30 s; `AUTO FINAL GATHER starts at ... s` line, pollen held at the end
- [ ] The intake works (check `intake_amps` and the `INTAKE ...CURRENT` lines if it does not); the ramp stows on the way to the corner; no contact with the Blue Wall flower stand or the hive; if it runs late the jostle ends at 26.5 s and the park still completes

## 8h. Solo path speeds (T17)
- [ ] `Auton - Solo`, blue and red: compare the `DRIVE` leg times with the 23:07 run (helper 3.01, garden pre-position 1.37, flower pre-position 1.03, back-out 0.95, launch spot 2 0.98 s); arrival errors at the launch spots under about 2.5 in, at the flower pre-positions under 1 in; no overshoot into a wall, support or flower stand
- [ ] The final gather's wall flower pre-position and back-out no longer run to their timeouts (they stalled 1.9 in and 1.7 in short in the 23:50 and 23:59 runs): each ends within `WALL_PRE_POS_TOL_IN` 2.5 in / `WALL_BACKOUT_TOL_IN` 2.5 in in about 1 s, and the ram still reaches the flower
- [ ] If a leg misbehaves: `AutonSolo.LEG_MAX_POWER` back to 0.6, `CROSS_FIELD_MAX_POWER` to 0.85, `PASS_SETTLE_SEC` to 0.2 (all on the Dashboard)

## 8i. Hive Dash: aim, flywheel, spots and flowers (T18)
- [ ] Dashes to each of the four cells (both alliances) from the near and the far side: the robot has finished turning on arrival and launches at once (read arrival to `HIVE DASH launching`); no wait for the flywheel above the band (`DASH launch speed fixed ...` at the start of each dash)
- [ ] The robot stops 8 in from the wall (x +-55.4) and nothing is pinned against the wall; with a pollen lying near the old spot the dash still arrives; the launch from the new spot (34 in, flat part of the map) lands
- [ ] No dash touches a flower stand: rear flower from the rear wall side, audience flower from the loading zone; right after gathering at a flower the dash starts (no refusal) and leaves it

## 8j. Hive Dash overshoot, corners and flower clearance (T19)
- [ ] Dashes to the audience cell from the Blue Wall side (it now goes under the hive), from the far side and from beside the hive, and to the rear cell: arrival within 2.5 in, stopped quickly, no coasting 2 to 8 in past the spot; the speed at arrival is about 40 in/s or less
- [ ] The helper corners (36, +-40) are taken at about 0.5 power (watch the sample rows: the speed falls before the via point) and the robot does not swing wide toward a flower
- [ ] No contact with the audience or rear flower stand on any dash; the first dash after gathering at a flower leaves it normally; the dash time rises by under about 0.5 s (`HiveDash.APPROACH_POWER` / `CORNER_POWER` 0 turn the caps off if it is too slow)

## 8k. Duo Primary (T20)
- [ ] `Auton - Duo Primary` (rear wall start, blue and red): INIT pose OK; read the `AUTO TIMING` lines for every step and where the 30 s ended; the order is preload (rear cell), Blue Wall flower (audience cell), audience flower (rear cell), garden (rear cell), park; no `TIMED OUT` on the pre-positions; every salvo is launched whatever the counter says, so a dry run with no pollen fires all four (the autos no longer skip on an empty count)
- [ ] The two spots: the audience one (50, 26.3) and the rear one (-50, 26.3) launch from about 32 in with the aim at each cell; check the landings and set `DuoAutos.P_*_TRIM_DEG`; `P_GARDEN_LATEST_START_SEC` is 18 s, so the garden is skipped and the park should end near 25 s (`AUTO SKIP garden` in the log); `P_AUD_FLOWER_LATEST_START_SEC` stays 99

## 9. If time
- [ ] F5 Pinpoint unplug (robot on blocks); F6 flywheel encoder unplug (on blocks)
- [ ] Pull the logs from the real hub: `python tools/logs/pull_logs.py`

## 10. Duo autos (T9, T10; design/duo-autos.md), alone on the field first
- [ ] Auton - Duo Garden Side (rear wall start): INIT pose OK; blue and red; the preload launch, the Blue Wall flower gather, the wait, then the garden and the last launch and park. Expect it to overrun 30 s: note where it is at 30 s from the `AUTO TIMING` lines (the waits are Dashboard values in `DuoAutos`)
- [ ] Auton - Duo Flower Side (audience wall start): INIT pose OK; blue and red; waits until `F_FIRST_LAUNCH_NOT_BEFORE_SEC`, launches, gathers the audience flower, launches, parks
- [ ] Then both together if a partner robot is available

Result notes:
