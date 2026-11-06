# Idea queue

Ideas agreed to keep but not to build yet, newest decisions at the top of each list. Move an idea out of here when it is started. (Started 2026-11-04.)

## Queue (in order)

1. **Automatic ramp stow in the TeleOp** (added 2026-11-05). While the ramp is deployed, stow it by itself once the robot has driven a set distance away from the flower it was gathering at, so nobody has to remember the operator's Y. Ideas: remember the nearest flower when the ramp deploys, stow when the robot is farther than a Dashboard distance from its center (the old auto rule was 24 in from the flower center; the pose check that `Mechanisms.requestStow` / `stowPending` already does is the place to reuse), log it like the other COMMAND lines, and keep the operator's X (deploy) and Y (stow at once) as overrides. The distance is to be chosen; the ramp must not be stowed while still on the flower (the pollen are still coming in), so measure from the flower, not from where the robot was when it deployed.
2. **Shot journal.** The operator taps the result after each salvo (made or missed, with where it landed) so the log gives a hit rate per position, cell and distance, which is what the aim-depth idea and the per-spot trims need. The D-pad reports (landed left / right / low / high) already exist; "made" is the new one. **Waiting on the user's choice of operator controls** (the operator's free buttons: A, and the stick buttons; never Start or Back).
3. **Flower Dash** (low priority). One TeleOp button that runs the auto's flower routine: drive to the flower pre-position, deploy the ramp, ram, jostle until full, back off. Reuses the Hive Dash route and follower plumbing and the auto's `Jostle`.
4. **Use the spare seconds in the solo auto** (bottom of the queue; the autos need refinement first). `Auton - Solo` finishes in about 24 s of the 30 s; the spare time could fetch more pollen if the field has any worth taking. Needs the field's element layout and the scoring rules.
5. **Jam resolution feature** (added 2026-11-05; the user's list gave only the name, so the behavior is still to be defined together). What exists today: a pusher move that cannot finish stops after `Pusher.MOVE_TIMEOUT_S` (3 s) and leaves the arm where it is, the operator's left bumper + B (reject) cancels a salvo and reverses, the pusher's and the intake's motor currents are logged (`pusher_amps`, `intake_amps`), and the operator can force the intake OUT (left trigger). Open: what counts as a jam (pusher stall, intake current, no movement), what the robot should do by itself (back the intake out, reverse and retry the pusher), and what the operator sees.

## Do later (agreed 2026-11-04)

- **End every auto with the ramp stowed** (still worth doing for the next auto's start and the field: a stowed ramp is not in the way). Since 2026-11-05 the TeleOp no longer commands the ramp in INIT (G403: no powered movement in the 8 s transition; it stows once at Start), and the other INIT commands were checked (power 0 only), so a deployed or stuck ramp no longer breaks the transition. `Auton - Solo` already stows before its last launch; check the duo autos and add a final stow if a path can end deployed.
- Solo's park at (51.1, 54, 180) only has to break the zone line (user: fine as is); moving it in by 1 in more is the option if it ever looks marginal.
- The duo autos' timing (the waits, G's last launch and park) is tuned from their first logged runs: do not optimise before then.

## Decided against (2026-11-04)

- Launch-ready rumble: Auto Launch and Hive Dash are the primary launch methods, so the driver is not the one timing the launch.
- Heading snap to 0 / 90 / 180 / 270: driving into the wall squares the robot up already.

## Done from the same list

- **Implemented 2026-11-05** (not yet run on the robot): the driver's target lock on the right bumper with the reseed chord on left bumper + right trigger (the old chord LB + RB would have fired while holding RB for the lock and pressing LB for the nudge); the operator's D-pad aim step 1.5 to 3 deg (maximum still 10); the gentler flower jostle (`Jostle.START_HOLD_SEC` 1.0 s pressing after the ram, `BACK_IN` 0.25 in at the new `BACK_POWER` 0.3 so it does not overshoot, `PRESS_HOLD_SEC` 1.0 s on the forward stroke, and (later the same night, after the user saw it work much better) `MAX_CYCLES` 1: it ends after a single back-and-forth; it affects `Auton - Solo` and both duo autos, whose flower timing needs a look in the logs: the jostle now has 1 s less shaking inside its deadline).
- Hold position while launching (`drive/HoldPosition.java`, test row T7), the Dashboard values logged at the start of every run (`util/ConfigDump.java`), the sensor baseline health check (`tools/logs/baseline_health.py`). (A switchable flywheel FF+P controller was built 2026-11-04 and removed 2026-11-05: bang-bang only; commit 4001aba has it.)
