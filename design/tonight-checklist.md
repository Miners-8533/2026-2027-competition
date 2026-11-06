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
- [ ] Controls: driver A launches, LB chords, precision on RT; operator A target lock, LB+B reject, LB+Y sweep
- [ ] Intake always on: IN under 4, OUT at 4; one rumble when full; the count stops flickering 3 / 4
- [ ] Target lock on by itself when full, off after the salvo; launch at all four cells (marks: far-cell extra -3.0, offset -2.2)
- [ ] Backup mode: LB + X held 1 s (banner, three taps)
- [ ] Pusher: no faults, a launch and a reject cancel each other, count reads 0 after a salvo (P10)

## 3. Pusher lock (T6)
- [ ] Changed 2026-11-05: a full robot alone just stops the intake (arm stays at HOLD, no OUT). Hold RT (target lock) with pollen held: the arm parks about 90 deg past HOLD, intake OFF, count frozen; release RT without launching: back to HOLD after 0.5 s and the intake runs again; a Hive Dash locks it on the way
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

## 9. If time
- [ ] F5 Pinpoint unplug (robot on blocks); F6 flywheel encoder unplug (on blocks)
- [ ] Pull the logs from the real hub: `python tools/logs/pull_logs.py`

## 10. Duo autos (T9, T10; design/duo-autos.md), alone on the field first
- [ ] Auton - Duo Garden Side (rear wall start): INIT pose OK; blue and red; the preload launch, the Blue Wall flower gather, the wait, then the garden and the last launch and park. Expect it to overrun 30 s: note where it is at 30 s from the `AUTO TIMING` lines (the waits are Dashboard values in `DuoAutos`)
- [ ] Auton - Duo Flower Side (audience wall start): INIT pose OK; blue and red; waits until `F_FIRST_LAUNCH_NOT_BEFORE_SEC`, launches, gathers the audience flower, launches, parks
- [ ] Then both together if a partner robot is available

Result notes:
