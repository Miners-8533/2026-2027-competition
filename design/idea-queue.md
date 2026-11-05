# Idea queue

Ideas agreed to keep but not to build yet, newest decisions at the top of each list. Move an idea out of here when it is started. (Started 2026-11-04.)

## Queue (in order)

1. **Shot journal.** The operator taps the result after each salvo (made or missed, with where it landed) so the log gives a hit rate per position, cell and distance, which is what the aim-depth idea and the per-spot trims need. The D-pad reports (landed left / right / low / high) already exist; "made" is the new one. **Waiting on the user's choice of operator controls** (the operator's free buttons: A, and the stick buttons; never Start or Back).
2. **Flower Dash** (low priority). One TeleOp button that runs the auto's flower routine: drive to the flower pre-position, deploy the ramp, ram, jostle until full, back off. Reuses the Hive Dash route and follower plumbing and the auto's `Jostle`.
3. **Use the spare seconds in the solo auto** (bottom of the queue; the autos need refinement first). `Auton - Solo` finishes in about 24 s of the 30 s; the spare time could fetch more pollen if the field has any worth taking. Needs the field's element layout and the scoring rules.

## Do later (agreed 2026-11-04)

- **End every auto with the ramp stowed** (G403: no powered movement in the 8 s transition; the TeleOp's INIT commands the ramp servos to stowed, so a deployed ramp would move when INIT is pressed in the transition). `Auton - Solo` already stows before its last launch; check the duo autos and add a final stow if a path can end deployed. Also check the other servo / motor commands in the TeleOp INIT for twitch.
- Solo's park at (51.1, 54, 180) only has to break the zone line (user: fine as is); moving it in by 1 in more is the option if it ever looks marginal.
- The duo autos' timing (the waits, G's last launch and park) is tuned from their first logged runs: do not optimise before then.

## Decided against (2026-11-04)

- Launch-ready rumble: Auto Launch and Hive Dash are the primary launch methods, so the driver is not the one timing the launch.
- Heading snap to 0 / 90 / 180 / 270: driving into the wall squares the robot up already.

## Done from the same list

- Hold position while launching (`drive/HoldPosition.java`, test row T7), the Dashboard values logged at the start of every run (`util/ConfigDump.java`), the sensor baseline health check (`tools/logs/baseline_health.py`), and the switchable flywheel controller (`Flywheel.CONTROL_MODE` 1, simulation in `tools/flywheel/ffp_sim.py`, test row T8).
