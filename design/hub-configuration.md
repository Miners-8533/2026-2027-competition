# Hub configuration checklist (ROBOT_1)

Phase 0 of [integration-plan.md](integration-plan.md). The code finds every device **by name** in the Driver Station's active configuration, and the coordinator **silently skips** a mechanism whose name it cannot find. So a misspelled name looks like "the mechanism does nothing", not like an error. This checklist exists to rule that out, and the **Hardware Check** opmode verifies it.

**Naming (2026-10-31):** the mechanisms are called the **pusher** (formerly the gate or transport gate) and the **ramp** (formerly the depollinator). The hardware-configuration names did NOT change: the pusher motor is still `gate`, and the ramp servos are still `depolLeft` and `depolRight`.

## What is on the hub today

Read over USB from the robot's Control Hub (serial `5462f57f0f18d316`). The only saved configuration is `test.xml` (the old bench setup):

| Hub | Port | Name in `test.xml` | Verdict |
|---|---|---|---|
| Expansion Hub 2 | motor 0 | `intake` | keep the name |
| Expansion Hub 2 | motor 1 | `transport` | **stale**, delete (nothing uses it) |
| Control Hub | motor 0 | `gate` | keep |
| Control Hub | motor 1 | `fly_motor_enc` | keep |
| Control Hub | motor 2 | `fly_follow` | **stale**, the second motor was removed |
| Control Hub | motor 3 | `rightFront` | keep (only one drive motor present) |
| Control Hub | servo 0, 2 | `hood`, `stop` | **stale** (last year's), delete |
| Control Hub | I2C bus 1 | `pinpoint` | keep |
| Control Hub | I2C bus 0 | `imu` | not used by our code |
| Ethernet | | `limelight` | **keep on purpose**: a Limelight will be added later; no code uses it yet |

**Missing for the robot:** `leftFront`, `leftBack`, `rightBack`, `depolLeft`, `depolRight`.

The old configuration also shows an **Expansion Hub** (address 2). Seven motors (four drive, intake, flywheel, pusher) do not fit on one hub, so both hubs are needed.

## Required configuration

Names are case-sensitive and must match exactly.

| Device | Name | Config type | Hub | Port | Notes |
|---|---|---|---|---|---|
| Drive, left front | `leftFront` | motor (the actual model) | Control Hub (173) | motor 0 | no encoder needed |
| Drive, left back | `leftBack` | motor | Control Hub (173) | motor 1 | |
| Drive, right front | `rightFront` | motor | Control Hub (173) | motor 2 | |
| Drive, right back | `rightBack` | motor | Control Hub (173) | motor 3 | |
| Odometry | `pinpoint` | goBILDA Pinpoint | Control Hub (173) | I2C bus 1, addr 0x31 | needs both pods plugged into the Pinpoint |
| Intake | `intake` | motor | Expansion Hub (2) | motor 2 | no encoder needed |
| Flywheel | `fly_motor_enc` | motor | Expansion Hub (2) | motor 1 | **encoder cable plugged in** (the code reads its velocity) |
| Pusher | `gate` | motor, goBILDA 5203 series (26.9:1) | Expansion Hub (2) | motor 0 | **encoder cable plugged in** (position and velocity) |
| Ramp, left | `depolLeft` | Servo | Control Hub (173) | servo 5 | positional mode, not continuous |
| Ramp, right | `depolRight` | Servo | Control Hub (173) | servo 4 | same mode as the left one |
| Element counter | `CS1`, `CS2`, `CS3`, `CS4` | REV Color Sensor V3 | (see Hardware Check) | one I2C bus each | numbered along the track: intake, CS1, CS2, CS3, CS4, flywheel. They all have the same I2C address, so each must be on its own bus |

Ports above were read by Hardware Check on 2026-10-31 (battery 12.2 V). Pinpoint: status READY, loop 637-662 us. **Note:** the pusher and flywheel are on the **Expansion Hub**, where the benches had them on the Control Hub, so the loop time and the pusher's `COAST_LOOKAHEAD_S` must be re-measured here (the benches' numbers do not carry over).

`fly_follow` must **not** be configured (the code no longer uses it, and Hardware Check flags it as stale).

### Where to plug things in

- **Keep the latency-sensitive devices on the Control Hub:** the pusher and flywheel (their encoder reads set the pusher's landing accuracy and the flywheel's speed check) and the Pinpoint. Commands to the Expansion Hub travel over a serial link through the Control Hub, so we expect them to be a little slower. That is unmeasured, so it is a preference, not a rule.
- This is advice for planning; if the robot is already wired, **record what it is** and move on. Rewire only if Phase 3 shows a loop-time problem.

## Steps on the Driver Station

- [ ] Open **Configure Robot**, create a **New** configuration, and scan/select the **Control Hub Portal** (and the Expansion Hub).
- [ ] Add each device from the table, with the **exact name** and the type that matches the real part. For the motors, pick the actual model (it sets the encoder metadata; our code runs the motors without the hub's closed-loop modes).
- [ ] Do **not** carry over the stale entries from `test.xml` (`transport`, `fly_follow`, `hood`, `stop`). `limelight` is kept on purpose for later.
- [ ] Save it with a clear name (for example `ROBOT_1`) and **activate** it.
- [ ] Fill in the **Hub** and **Port** columns above, or copy them from Hardware Check (next section), and photograph the wiring.

## Verify (the robot on blocks, nothing moves)

Run each opmode in order. All three only read.

- [x] **Hardware Check** (passed 2026-10-31: all ten devices present, ports recorded above, Pinpoint READY, turning the flywheel by hand moved only `fly_motor_enc` and turning the pusher arm moved only `gate`; remaining: delete the stale `stop` entry; `limelight` stays on purpose, so expect it in the "configured but not used" list)
  - `RESULT: all required devices are present`. A `MISSING` or `WRONG TYPE` line names the problem entry.
  - Every device line shows the hub and port you expected. Copy them into the port table above.
  - Turn each part **by hand** and watch the encoders: the flywheel by hand changes only `fly_motor_enc`, the pusher arm changes only `gate`. If two encoders move together, or the wrong name moves, two cables are crossed.
  - `pinpoint`: `status READY`, and a loop time of 500-1100 us. `FAULT_*` means a pod is not detected.
  - Both hubs listed with a battery voltage that makes sense (a charged battery reads about 12-13 V; note it).
  - The "configured but not used" list is empty (or only holds devices you chose to keep, such as `imu`).
- [x] **Chassis Info**: `Matched profile: ROBOT_1` (not "NONE"). Hardware Check reported `Chassis profile: ROBOT_1` on 2026-10-31.
- [ ] **TeleOp - Draft (mechanisms)**, in INIT: the telemetry line `Mechanisms present` reads `intake true, flywheel true, pusher true, ramp true`, and the init screen shows no "no hub registered yet" warning.

## If something is wrong

| Hardware Check says | Likely cause | Fix |
|---|---|---|
| `MISSING from the hub configuration` | Typo, wrong case, or the device was never added | Rename it in Configure Robot, save, **re-activate**, re-run |
| `WRONG TYPE` | A servo configured as a motor (or the reverse), or the Pinpoint as a generic I2C device | Delete the entry and re-add it with the right type |
| A device is present but the part does nothing | Configured on a different port than the cable | Compare the Hardware Check port with the real cable; move one or the other |
| Encoder counts do not change for the right part | Encoder cable on the wrong motor port, or unplugged | Re-plug; spinning the part by hand must change only its own name |
| Pinpoint `FAULT_*` or no response | Pod cable, I2C cable, or wrong bus | Re-seat; check it is on an I2C bus of the Control Hub |
| An opmode throws `Unrecognized Control Hub` | That hub is not registered in `ChassisProfile` | Run Chassis Info on it and add its id (see integration-plan.md, Phase 0) |

## After it passes

- [ ] Commit the filled-in port table in this file.
- [ ] Back up the configuration file. Once it is saved on the hub, copy `/sdcard/FIRST/<name>.xml` into the repo (for example `design/hub-config/ROBOT_1.xml`) so a replaced hub can be set up identically. Ask me and I will pull it over USB.
