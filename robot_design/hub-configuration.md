# Hub configuration (ROBOT_1)

The hardware and software meet here. The code finds every device **by its configuration name**, so the names, types and ports below are part of the software's contract with the robot.

The setup steps and what to do when a check fails are in the section *Setting up and checking a hub* below; the Pit Check opmode is described in [specs/support-opmodes.md](../specs/support-opmodes.md) section 1.

## The hubs

| Hub | Address | Identity |
|---|---|---|
| REV Control Hub | module 173 | serial `5462f57f0f18d316`, Android id `7c789032eb35f27b` |
| REV Expansion Hub | module 2 | connected to the Control Hub (commands to it travel through the Control Hub) |

The Control Hub's identity selects the `ROBOT_1` chassis profile. A hub that is not registered in the code makes every opmode stop with "Unrecognized Control Hub" and its ids, so a build can never run on the wrong chassis with the wrong directions.

Both hubs are needed: seven motors do not fit on one.

## Devices

Names are case-sensitive and must match exactly. Ports are from the Pit Check logs of 2026-11-03 and 2026-11-05 (identical).

### Motors

| Name | Part | Hub | Port | Encoder cable | Direction (`ROBOT_1`) | At zero power |
|---|---|---|---|---|---|---|
| `leftFront` | drive, left front | Control | motor 0 | not used | REVERSE | brake |
| `leftBack` | drive, left back | Control | motor 1 | not used | REVERSE | brake |
| `rightFront` | drive, right front | Control | motor 2 | not used | FORWARD | brake |
| `rightBack` | drive, right back | Control | motor 3 | not used | FORWARD | brake |
| `gate` | pusher (goBILDA 5203 series, 26.9:1) | Expansion | motor 0 | **required** (position and speed) | FORWARD | brake |
| `fly_motor_enc` | flywheel | Expansion | motor 1 | **required** (speed) | REVERSE | float |
| `intake` | intake | Expansion | motor 2 | not used | FORWARD | brake |

Expansion Hub motor 3 is free.

Every motor is run as a plain duty cycle (the hubs' closed-loop encoder modes are never used). The pusher and flywheel encoders are read as sensors only. For each motor the configured type should be the real model.

### Servos

| Name | Part | Hub | Port | Mode |
|---|---|---|---|---|
| `depolLeft` | ramp, left | Control | servo 5 | positional |
| `depolRight` | ramp, right | Control | servo 4 | positional, same mode as the left |
| `fly_pos` | flywheel position | **NOT RECORDED** | **NOT RECORDED** | positional |

### I2C

| Name | Part | Hub | Bus | Address |
|---|---|---|---|---|
| `pinpoint` | goBILDA Pinpoint odometry computer | Control | 1 | 0x31 |
| `CS1` | REV Color Sensor V3, nearest the intake of the four | Expansion | 1 | 0x52 |
| `CS2` | REV Color Sensor V3, at the top of the feed ramp since the move reported 2026-11-18 (same hub, bus and name as before) | Expansion | 2 | 0x52 |
| `CS3` | REV Color Sensor V3 | Expansion | 3 | 0x52 |
| `CS4` | REV Color Sensor V3, nearest the flywheel | Control | 3 | 0x52 |
| `CS5` | REV Color Sensor V3, at the intake entrance | **NOT RECORDED** | **NOT RECORDED** | 0x52 |

All five color sensors have the same address, so each must be alone on its bus. By elimination `CS5` is on Control Hub bus 2 or Expansion Hub bus 0 (an inference: Control Hub bus 0 holds the built-in IMU). Record which.

### Configured but not used by the code

| Name | What | Note |
|---|---|---|
| `imu` | the Control Hub's built-in IMU, bus 0 | the Pinpoint supplies the heading |
| `limelight` | a Limelight on Ethernet | kept on purpose for later; no code uses it |

`fly_follow` (a second flywheel motor) must **not** be configured. The motor was removed from the robot and Pit Check reports the name as stale.

## What the code does when a name is missing

| Device | Missing means |
|---|---|
| The four drive motors, `pinpoint` | the opmode fails at INIT with a device-not-found error |
| `intake`, `fly_motor_enc`, `gate` | that mechanism is silently left out; everything else runs |
| `depolLeft`, `depolRight` | the ramp is left out unless both are present |
| `CS1`, `CS3`, `CS4` | the element counter is left out unless all three are present (`CS2` has not been in the track since 2026-11-18 and is not needed for it) |
| `CS2` | no recording of the launches; nothing else depends on it. The code does not read it at all since 2026-11-20 (`LaunchSensor.CS2_PRESENT` false) |
| `CS5` | no entry count: the robot never declares itself full and the screen shows `? HELD`. The code does not read it at all since 2026-11-20 (`EntryCounter.CS5_PRESENT` false), so that is the present state whether it is configured or not |
| `fly_pos` | the flywheel position is not commanded |

So a misspelled name looks like "the mechanism does nothing", not like an error. The INIT screen of the competition TeleOp lists the mechanisms it found.

**Pit Check covers** the drive motors, `intake`, `fly_motor_enc`, `gate`, the two ramp servos, `pinpoint`, `CS1` to `CS4`, and a stale `fly_follow`. From 2026-11-20 it also reports `CS5`, `CS2` as the launch sensor, and `fly_pos`: whether each is configured, and for the two sensors the state of its switch in the code. A sensor whose switch is off passes whether it is configured or not; `CS5` fails only when its switch is on and the device is missing; `fly_pos` is optional and passes either way. None of the three is exercised: with its switch on, confirm `CS5` from the log line `CS5 ENTRY COUNTER started`.

## Setting up and checking a hub

The code finds every device **by name** in the Driver Station's active configuration, and the coordinator silently skips a mechanism whose name it cannot find (the section above). This checklist rules that out; Pit Check verifies it ([specs/support-opmodes.md](../specs/support-opmodes.md) section 1). It was `design/hub-configuration.md` until 2026-11-23.

### Steps on the Driver Station

- [ ] Open **Configure Robot**, create a **New** configuration, and scan or select the **Control Hub Portal** (and the Expansion Hub).
- [ ] Add each device from the tables above, with the **exact name** and the type that matches the real part. For the motors, pick the actual model (it sets the encoder metadata; our code runs the motors without the hub's closed-loop modes).
- [ ] Do **not** carry over stale entries from an older configuration (`transport`, `fly_follow`, `hood`, `stop`). `limelight` is kept on purpose for later.
- [ ] Save it with a clear name (`ROBOT_1`) and **activate** it.
- [ ] If a port differs from the tables, correct the tables on this page (or copy the ports from Pit Check's `Ports` line) and photograph the wiring.

### Verify (the robot on blocks, nothing moves)

Run each in order. In Pit Check nothing moves until a step is confirmed with A; the first steps only read.

- [ ] **Pit Check** (first passed 2026-10-31):
  - `RESULT: all required devices are present`. A `MISSING` or `WRONG TYPE` line names the problem entry.
  - Every device line shows the hub and port you expected.
  - Turn each part **by hand** and watch the encoders: the flywheel by hand changes only `fly_motor_enc`, the pusher arm changes only `gate`. If two encoders move together, or the wrong name moves, two cables are crossed.
  - `pinpoint`: `status READY` and a sane loop time. `FAULT_*` means a pod is not detected.
  - Both hubs listed with a battery voltage that makes sense (a charged battery reads about 12 to 13 V; note it).
  - The "configured but not used" list is empty, or holds only devices kept on purpose (`limelight`, `imu`).
  - `CS5`, `CS2` and `fly_pos` are reported as configured or not, with the state of each sensor's switch in the code; none of them is exercised.
  - `Chassis profile: ROBOT_1`, not "NONE". An unregistered hub makes every opmode stop with "Unrecognized Control Hub" and its ids.
- [ ] **TeleOp - Competition**, after pressing Start: the telemetry line `Mechanisms present` reads `intake true, flywheel true, pusher true, ramp true`, and the INIT screen shows no "no hub registered yet" warning. With CS5's switch on, the log has a `CS5 ENTRY COUNTER started` line; with it off (the present state), a `CS5 SWITCHED OFF` line.

### If something is wrong

| Pit Check says | Likely cause | Fix |
|---|---|---|
| `MISSING from the hub configuration` | a typo, the wrong case, or the device was never added | rename it in Configure Robot, save, **re-activate**, run again |
| `WRONG TYPE` | a servo configured as a motor (or the reverse), or the Pinpoint as a generic I2C device | delete the entry and add it again with the right type |
| A device is present but the part does nothing | configured on a different port than the cable | compare the Pit Check port with the real cable; move one or the other |
| Encoder counts do not change for the right part | the encoder cable is on the wrong motor port, or unplugged | re-plug; turning the part by hand must change only its own name |
| Pinpoint `FAULT_*` or no response | a pod cable, the I2C cable, or the wrong bus | re-seat; check it is on an I2C bus of the Control Hub |
| An opmode throws `Unrecognized Control Hub` | that hub is not registered in `ChassisProfile` | read the ids in the error and register the hub there ([specs/drive.md](../specs/drive.md)) |

### After it passes

- [ ] Commit any corrected ports on this page.
- [ ] Back up the configuration file (next section).

## Saved configuration file

The active configuration is named `ROBOT_1` on the hub. **NOT RECORDED:** no copy of the file is in the repository. Pull `/sdcard/FIRST/ROBOT_1.xml` over USB and keep it here, so a replaced hub can be set up identically.

## Power

- One FTC 12 V battery feeds both hubs. Each hub reports its own voltage; the code reads it once a second.
- Pit Check fails a battery under 12.0 V at rest.
- The flywheel spinning up at full power pulls the battery to about 9.4 to 9.9 V for a moment on a healthy battery.
- **A weak battery can drop the Expansion Hub.** On 2026-11-06 a sag to 7.4 V lost the Expansion Hub mid-run: the flywheel, pusher and intake stopped answering and the flywheel encoder looked dead. Check the battery first when those three fail together.

## Bus load (why the wiring matters to the software)

Each hub read or write is a blocking round trip of a few milliseconds, and commands to the Expansion Hub pass through the Control Hub. The whole control loop runs about 26 to 32 ms in the TeleOp. Measured costs:

| Operation | Cost |
|---|---|
| Pinpoint pose read | about 6 ms |
| One standard color sensor read (color and distance) | about 13 ms |
| `CS5` two-byte proximity read | about 3 ms alone, 6 to 7 ms when queued behind the main loop |
| A motor current read | 2 to 3 ms |
| Motor encoder positions and speeds | one bulk read per hub per loop (bulk caching is automatic) |

The pusher and the flywheel are on the Expansion Hub. Their timing was calibrated there; moving either to the Control Hub would change the pusher's landing and need a re-check.

## Driver station and controllers

- Two Logitech F310 gamepads: gamepad 1 is the driver (Start + A), gamepad 2 the operator (Start + B). They have **no rumble**.
- Because Start assigns the gamepads, nothing is ever bound to Start.
- The Driver Station telemetry area shows about 15 to 16 lines of about 36 characters and formats text only in its HTML display mode (sizes by nested `<big>` and `<small>`, text colors, no background colors).
- The FTC Dashboard is reachable at `192.168.43.1:8080/dash` on the hub's Wi-Fi, or over USB with `adb forward tcp:8080 tcp:8080`.
