# Element counting

The robot counts what it holds in two independent ways.

| Count | Sensors | Answers | Used for |
|---|---|---|---|
| **Slot count** | CS1 to CS4 along the track | which kind of element sits at each of four places | the pusher check (the track reads empty), the opponent-nectar warning, seeing the pusher arm, the log |
| **Entry count** | CS5 at the entrance | how many elements have come in since the track was emptied | **how many are held, and full**: FULL and its sequence, the driver screen, the end of a flower jostle in the autos, the "Full" warning, the launched tally |

**The entry count is the count the robot relies on** (team decision, 2026-11-17; built 2026-11-17, not run on the robot). The slot count undercounts in a crowded track, which is the worse error on this robot, so it no longer says how many are held. One switch ("count by entries", on) returns the jostle end, the "Full" warning, the non-always-on intake stop and the launched tally to the slot count, in case the entry count misbehaves on the robot. With CS5 missing or not answering the robot is never full by it and there is no fall-back: a jostle ends on its deadline and the screen shows a question mark. How the coordinator uses the counts is in [mechanisms.md](mechanisms.md).

Sensor physics and measured signatures: [robot_design/sensors.md](../robot_design/sensors.md).

## 1. Slot count (CS1 to CS4)

### 1.1 Reading

- **One sensor is read every 40 ms**, in rotation, and never more than one per loop. A full refresh of all four takes about 160 ms. (Each read blocks about 13 ms and the sensors only produce a new value every 100 ms.)
- The counter is **valid** only while every sensor has been read within the last 1 s. Callers must not act on it otherwise.
- All four sensors must be configured, or the counter does not exist.

### 1.2 Per-sensor result

Each sensor reports one of: EMPTY, POLLEN, RED NECTAR, BLUE NECTAR, OTHER, PUSHER ARM.

**Presence** (is anything there) is judged against that sensor's own stored empty reading, with separate on and off thresholds so a reading near a threshold does not flicker.

| Test | Turns ON when | Turns OFF only when all are below |
|---|---|---|
| Color shift: distance between the red/green/blue shares and the empty shares | 0.05 | 0.03 |
| Brightness ratio to the empty level (and at least 0.02 above it) | 2.0 | 1.6 |
| Nearer than the empty distance by | 1.5 cm | 0.8 cm |

A new presence state must persist 150 ms before it is reported.

**Kind** (what a present object is), tested in this order on the color shares:

1. **Pusher arm** (CS4 only): blue at least 0.36, red at most 0.28, green within 0.06 of blue, and at least 4.0 cm away.
2. **Red nectar:** red at least 0.35, red not below green, blue under 0.30.
3. **Blue nectar:** blue at least 0.44, blue at least 0.08 above green, red at most 0.25.
4. **Pollen:** green at least 1.5 times blue, red at most 0.35.
5. Otherwise **other**.

### 1.3 The count

- **Count** = the number of sensors that see an element. The pusher arm is not an element.
- The reported count changes only after the raw count has been steady:

| Change | Must be steady for |
|---|---|
| Rising | 150 ms |
| Rising to 4 (full) | 1000 ms |
| Falling | 300 ms |

The end sensor flickers for about a second while an element moves, hence the windows. A fourth element is reported about 1.3 s after it is first seen.

- **Gap flag:** an element seen without a neighbor on its flywheel side is still moving or a misread. It still counts.
- **Full:** the count is 4.

### 1.4 Things that are kept out of the count

| Situation | Effect |
|---|---|
| The arm is near CS4 (from 70 deg behind HOLD to 70 deg ahead of it) | an OTHER reading at CS4 is taken as the arm's edge and not counted; pollen and nectar readings still count |
| A pusher check is running | CS4 is not counted |
| A pusher check is sweeping | no sensor is counted (the arm passes all of them) |
| For 0.8 s after a pusher check | no sensor is counted |
| The pusher is locked | the count is **frozen** at its value; the sensors are still read and logged |

### 1.5 Other answers

- Whether CS4 sees the pusher arm now, and how many times CS4 has been read (the pusher check waits for fresh reads).
- The first sensor, from the intake end, that holds the **opponent's** nectar (red for a blue robot, blue for a red one), or none.
- The kind of the element at CS4 (it launches first).
- A compact text of the four slots for the log, for example `.PRB` (P pollen, R red nectar, B blue nectar, O other, A arm, a dot for empty).
- For each sensor: the latest brightness, distance, shares, and color shift from its empty reading.

### 1.6 The stored empty readings

The empty readings are constants in the code (values in [robot_design/sensors.md](../robot_design/sensors.md)). They cannot be measured at the start of a match because the robot starts loaded. They are re-measured in the pit when the lighting or the track changes.

To catch a stale baseline, while the track reads empty the coordinator logs the four live color shifts once a second, and logs the stored baselines once at the start of every run.

### 1.7 Parameters

| Name | Value |
|---|---|
| Read period | 40 ms |
| Stale after | 1000 ms |
| Presence debounce | 150 ms |
| Count steady: rise / full / fall | 150 / 1000 / 300 ms |
| Capacity | 4 |
| Software gain | 1.0 |

## 2. Entry count (CS5)

### 2.1 Sampling

- A **separate thread** reads CS5's raw proximity (one small read) every 25 ms and runs the counting rule on each sample. It starts with the first coordinator update, so nothing is read during INIT, and stops by itself when updates stop for 1.5 s.
- The sensor is put into its fast proximity mode when the counter is built. If that fails the count is useless and the log says so.
- The thread never writes the log. The main loop collects finished entries and the raw samples from it.
- The counter is **healthy** while samples keep arriving; no sample for 0.5 s is reported as not answering.

The period is a trade against the main loop: at 25 ms the thread costs nothing measurable; at 20 ms the loop slowed by several milliseconds.

### 2.2 The empty level

- The empty level is the **90th percentile** of the first second of samples. The empty reading flickers between two neighbouring counts and the percentile makes the level land on the upper one consistently. **The intake must be clear for the first second of a run.**
- Afterwards the level can only be **lowered**: when the last 2 s were quiet (a range of at most 4 counts) and their median is more than 1 count below it. An object parked in front of the sensor can therefore never raise it.

### 2.3 Edge counting (the rule in use)

An entry is the **rise** of the signal:

1. The signal reaches 6 counts above the empty level and stays there for 2 samples in a row: **one entry**, counted at that moment (about 50 ms after the element arrives).
2. No further entry is possible until the signal has come back to 2 counts above the empty level or lower.

Consequences, both accepted:

- An element that stays under the sensor is one entry for as long as it stays.
- A clump whose elements touch, so the signal never dips, counts once.

If the signal stays above the re-arm level for 10 s, a note is logged (an element is parked there, or the empty level has moved) and no entries come until it drops.

### 2.4 Peak counting (the earlier rule, kept behind a switch)

With edge counting switched off, an entry is a local peak of the signal:

- a peak counts when it stands at least 6 counts above its surroundings (prominence), at least 0.06 s after the previous peak;
- **merge rule:** peaks less than 0.4 s apart form a group; a group containing a peak of 45 counts or more counts once (one hard arrival that rings), while a group of small peaks counts peak by peak;
- **valley rule (off by default):** a peak whose preceding valley is more than a set amount above the empty level is rejected, which removes the wiggles of a parked element.

It overcounted in real driving (a parked or rolling element makes several peaks), which is why edge counting replaced it. The recorded test runs still replay through it.

### 2.5 What the coordinator does with entries

- Each entry is logged with its height, the proximity value, and its time.
- An entry whose moment fell while the intake was running OUT, or within 0.4 s after, is **not** added to the held count: elements leaving pass the same sensor.
- The **held count** restarts at 0 when a salvo starts or a reject finishes.
- The held count reaching 4 declares **FULL**.
- The driver screen shows the held count, or a question mark when CS5 is missing or not answering.

**The starting load (built 2026-11-17, not run on the robot).** CS5 only sees elements come in, so the held count does not start at 0 when the load is known:

| Opmode | Held at the first loop |
|---|---|
| An auto | 4, the preload the rules require (a tunable value; 0 for practice without a preload) |
| A TeleOp that follows an auto | the auto's last saved count |
| A TeleOp restarted in a match | its own last saved count |
| A TeleOp with nothing fresh saved | 0 |
| Any other opmode | 0 |

- The autos and the competition TeleOp **save the held count on every loop**, beside the pose ([drive.md](drive.md) section 5). The TeleOp keeps saving it in backup mode.
- A saved count is used only while it is as fresh as a saved pose must be (300 s), and it is lost when the robot app restarts.
- FULL is not saved. It follows from the count: an opmode that starts with 4 held declares FULL on its first update. Only the TeleOp in FULL AUTO acts on it.
- Nothing can be counted while no opmode runs. An element that rolls in or out during the transition or a restart is missed, and the next salvo or reject puts the count right.
- The TeleOp's INIT screen shows the count it will start with and how old the save is. D-pad up in INIT discards it, together with the saved pose ([teleop.md](teleop.md)).
- With no CS5 in the configuration nothing is saved or restored.

Two further guards exist and are switched off: requiring the slot count to agree before FULL is declared, and resetting the held count when the slot count has read 0 for a set time.

### 2.6 Parameters

| Name | Value |
|---|---|
| Sample period | 25 ms |
| Empty level measured over | first 1.0 s |
| Edge: rise threshold / re-arm threshold / samples to confirm | +6 / +2 counts / 2 |
| Edge counting in use | yes |
| Peak rule: prominence / minimum separation | 6 counts / 0.06 s |
| Peak rule: merge height / merge gap | 45 counts / 0.4 s |
| Peak rule: valley limit | off |
| Thread stops after no update for | 1.5 s |
| Thread statistics logged every | 15 s |

### 2.7 Known limits

- Characterized with pollen only.
- Elements fed from the floor in tight groups can undercount.
- The signal level depends on how the sensor is mounted.
- Whether the entry count alone is good enough to act on has not been proven in matches.

## 3. Status

- Slot count: used at Hartland.
- Entry count: added after Hartland. Peak counting and a first version of the FULL actions ran in practice on 2026-11-14. Edge counting was validated only by replaying recorded runs; it has not run on the robot.

## 4. Verification by replay

The counting rule is one pure piece of logic with no robot dependencies, so recorded proximity traces can be replayed through exactly the code the robot runs. A PC check does this against recorded runs with known answers (a hand-fed run of 31 elements, and TeleOp runs with known loads). Any change to the rule should be replayed before it goes on the robot.

## 5. Target T3: the entry count is the count

**Built 2026-11-17, not run on the robot.** This section records what the target was and what is left of it.

**The weakness was** two counts with each consumer choosing its own: FULL and the screen used the entry count; the end of a flower jostle in the autos, the "Full" warning and the non-always-on intake stop used the slot count; the launched tally took the higher of the two. The robot could be full for one purpose and not for another in the same loop. And the entry count started at 0 in every opmode.

**Decision (team, 2026-11-17).** The CS5 entry count is the count the robot relies on. The count is remembered from the auto to the TeleOp and through an unexpected TeleOp restart. With CS5 missing or not answering there is no fall-back to the slot count: the Hartland Qualifier was played with practically no count.

**What was built.**

| Item | Where |
|---|---|
| Full, for every consumer, is the entry count reaching 4 | the top of this page; [mechanisms.md](mechanisms.md) |
| The launched tally takes the entry count | [mechanisms.md](mechanisms.md) section 3 |
| The count starts from the known load and is saved every loop | section 2.5 |
| One switch back to the slot count for the four consumers that used it | the top of this page |

**What is left.**

- The two guards that sit switched off beside the count (slot agreement before FULL; a reset when the slots read 0) are still in the code. They go when the entry count has run in a match.
- The slot count's own windows for "full" (section 1.3) are still computed. They are only used with the switch off.
- **Touching elements count once**, so a clump can still put a fifth element in the robot (section 2.7). The answer to that is in the counting rule and in how the sensor is mounted, not in a second count.
- The counter was characterized with pollen only; it needs a check with nectar.

**Waits on** the edge counter's first run on the robot. Until then the whole of this is unproven, and the switch is the way back.
