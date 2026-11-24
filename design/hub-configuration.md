# Hub configuration checklist (ROBOT_1)

**The device table (names, types, hubs and ports) moved to [robot_design/hub-configuration.md](../robot_design/hub-configuration.md) on 2026-11-17.** That page is the record of what is on the robot. This page keeps only the procedure: how to set a hub up and how to check it.

Phase 0 of [integration-plan.md](integration-plan.md). The code finds every device **by name** in the Driver Station's active configuration, and the coordinator **silently skips** a mechanism whose name it cannot find. So a misspelled name looks like "the mechanism does nothing", not like an error. This checklist exists to rule that out, and the **Pit Check** opmode verifies it.

**Naming (2026-10-31):** the mechanisms are called the **pusher** (formerly the gate or transport gate) and the **ramp** (formerly the depollinator). The hardware-configuration names did NOT change: the pusher motor is still `gate`, and the ramp servos are still `depolLeft` and `depolRight`.

## Steps on the Driver Station

- [ ] Open **Configure Robot**, create a **New** configuration, and scan/select the **Control Hub Portal** (and the Expansion Hub).
- [ ] Add each device from the tables in [robot_design/hub-configuration.md](../robot_design/hub-configuration.md), with the **exact name** and the type that matches the real part. For the motors, pick the actual model (it sets the encoder metadata; our code runs the motors without the hub's closed-loop modes).
- [ ] Do **not** carry over stale entries from an older configuration (`transport`, `fly_follow`, `hood`, `stop`). `limelight` is kept on purpose for later.
- [ ] Save it with a clear name (`ROBOT_1`) and **activate** it.
- [ ] If a port differs from the tables, correct the tables in `robot_design/hub-configuration.md` (or copy the ports from Pit Check, next section), and photograph the wiring.

## Verify (the robot on blocks, nothing moves)

Run each opmode in order. All three only read. (Pit Check is hidden from the Driver Station with `@Disabled` at present: remove that line and its import to use it.)

- [x] **Pit Check** (passed 2026-10-31: all ten devices present, ports recorded, Pinpoint READY, turning the flywheel by hand moved only `fly_motor_enc` and turning the pusher arm moved only `gate`; `limelight` stays on purpose, so expect it in the "configured but not used" list)
  - `RESULT: all required devices are present`. A `MISSING` or `WRONG TYPE` line names the problem entry.
  - Every device line shows the hub and port you expected.
  - Turn each part **by hand** and watch the encoders: the flywheel by hand changes only `fly_motor_enc`, the pusher arm changes only `gate`. If two encoders move together, or the wrong name moves, two cables are crossed.
  - `pinpoint`: `status READY`, and a loop time of 500-1100 us. `FAULT_*` means a pod is not detected.
  - Both hubs listed with a battery voltage that makes sense (a charged battery reads about 12-13 V; note it).
  - The "configured but not used" list is empty (or only holds devices you chose to keep, such as `imu`).
  - Pit Check does **not** look at `CS5` or `fly_pos`: check those in the TeleOp step below.
- [x] **Chassis profile**: `Matched profile: ROBOT_1` (not "NONE"). The Pit Check shows `Chassis profile: ROBOT_1`; an unregistered hub throws "Unrecognized Control Hub" with its ids. (Chassis Info and Pit Check were folded into the Pit Check on 2026-11-02.)
- [ ] **TeleOp - Competition**, after pressing Start: the telemetry line `Mechanisms present` reads `intake true, flywheel true, pusher true, ramp true`, the INIT screen shows no "no hub registered yet" warning, and the log has a `CS5 ENTRY COUNTER started` line (CS5 is found).

## If something is wrong

| Pit Check says | Likely cause | Fix |
|---|---|---|
| `MISSING from the hub configuration` | Typo, wrong case, or the device was never added | Rename it in Configure Robot, save, **re-activate**, re-run |
| `WRONG TYPE` | A servo configured as a motor (or the reverse), or the Pinpoint as a generic I2C device | Delete the entry and re-add it with the right type |
| A device is present but the part does nothing | Configured on a different port than the cable | Compare the Pit Check port with the real cable; move one or the other |
| Encoder counts do not change for the right part | Encoder cable on the wrong motor port, or unplugged | Re-plug; spinning the part by hand must change only its own name |
| Pinpoint `FAULT_*` or no response | Pod cable, I2C cable, or wrong bus | Re-seat; check it is on an I2C bus of the Control Hub |
| An opmode throws `Unrecognized Control Hub` | That hub is not registered in `ChassisProfile` | Read the ids in the error and add one (see integration-plan.md, Phase 0) |

## After it passes

- [ ] Commit any corrected ports in `robot_design/hub-configuration.md`.
- [ ] Back up the configuration file. Once it is saved on the hub, copy `/sdcard/FIRST/<name>.xml` into the repo (`robot_design/ROBOT_1.xml`) so a replaced hub can be set up identically. Ask me and I will pull it over USB.
