# Element counting

The robot counts what it holds in two independent ways.

| Count | Sensors | Answers | Used for |
|---|---|---|---|
| **Slot count** | CS1 to CS4 along the track | which kind of element sits at each of four places | the pusher check, opponent-nectar warning, the log, part of the launched tally |
| **Entry count** | CS5 at the entrance | how many elements have come in since the track was emptied | declaring FULL, the number on the driver screen |

Neither is trusted alone. The slot count misses elements between sensors or in a crowded track; the entry count undercounts touching elements. How the coordinator combines them is in [mechanisms.md](mechanisms.md).

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

Not in the code. See "As built and target" in the [README](README.md).

### 5.1 The weakness

There are two counts, and each consumer chose its own:

| Consumer | Uses today |
|---|---|
| FULL, the FULL sequence, arming the auto-launch | the entry count only |
| The number on the driver screen | the entry count |
| The end of a flower jostle in the autos | the slot count only |
| The "Full" warning; the intake stop in the non-always-on mode | the slot count only |
| The launched tally | the higher of the two |
| Whether a pusher check may start | the slot count reading 0 |

So the robot can be full for one purpose and not for another in the same loop, and the rules are split between the two counters and the coordinator.

A second gap shows once CS5 is relied on: **the entry count starts at 0 in every opmode.** An auto starts with 4 preloaded, and a TeleOp that follows an auto starts with whatever the last gather left aboard. In both the count reads low until the first salvo.

### 5.2 Decision (team, 2026-11-17)

**The CS5 entry count is the count the robot relies on.** The slot count is being retired as a count: it undercounts, and on this robot an undercount is the worse error.

Decided the same day:

| Point | Decision |
|---|---|
| The count between opmodes | remembered from the auto to the TeleOp, and through an unexpected TeleOp restart |
| CS5 missing or not answering | no fall-back to the slot count. The Hartland Qualifier was played with practically no count |

### 5.3 Requirement

1. "How many are held" and "is it full" have one answer, from the entry count. Every consumer uses it.
2. The slot sensors keep the jobs CS5 cannot do: what kind of element sits where, whether the track reads empty, and seeing the pusher arm.
3. The count is right from the first loop of an opmode, not from the first salvo.
4. A missing or silent CS5 degrades the answer to "unknown". It never blocks collecting or launching ([architecture.md](architecture.md) section 4.8).

### 5.4 Design

**Held** is the entry held count of section 2.5. **Full** is held reaching 4.

| Consumer | Uses |
|---|---|
| FULL, the FULL sequence, arming the auto-launch | held, as now |
| The driver screen | held, as now |
| The end of a flower jostle in the autos | full by held (the idea recorded in [autos.md](autos.md) section 10) |
| The "Full" warning; the intake stop in the non-always-on mode | full by held |
| The launched tally | held at the start of the salvo |

**What the slot sensors still do.**

| Job | Used by |
|---|---|
| The kind at each slot | the opponent-nectar warning; the log |
| "The track reads empty" | whether a pusher check may start; the empty-reading health line |
| Seeing the pusher arm at CS4 | the pusher check |

They no longer produce a number that anything acts on. The windows of section 1.3 remain only as far as "reads empty" needs them, and the slot text stays in the log.

**The starting load.** The held count is **saved on every loop** by the autos and the TeleOp, together with the pose ([drive.md](drive.md) section 5), and an opmode starts from what is known:

| Opmode | Held at the first loop |
|---|---|
| An auto | 4, the preload the rules require |
| A TeleOp that follows an auto | the auto's last saved count |
| A TeleOp restarted in a match | its own last saved count |
| A TeleOp with nothing fresh saved | 0, and the INIT screen says so |
| A support opmode | 0 |

- The saved count is used on the same terms as the saved pose: only while fresh, and lost when the robot app restarts.
- FULL is not saved. It follows from the count: a TeleOp that starts with 4 held declares FULL on its first update.
- Entries cannot be counted while no opmode runs. An element that rolls in or out during the transition or a restart is missed, and the next salvo or reject puts the count right.
- The INIT screen of the TeleOp shows the count it will start with and where it came from.

The INIT screens of the autos keep showing what the slot sensors see, as a check that the preload is in.

**The resets,** all in one place, are those of section 2.5: a salvo starts or a reject finishes; the put-back after a salvo that did not complete; entries while the intake ran OUT are ignored.

**When CS5 is missing or not answering:** held is unknown and the screen shows a question mark, as now. FULL is not declared, a jostle ends on its deadline, and the operator launches or rejects by eye. There is **no fall-back to the slot count**; that would keep the retired rule alive in the one situation where nobody is watching it.

**The limits that stay** are those of section 2.7. Touching elements count once, so a clump can still put a fifth element in the robot. The answer to that is in the counting rule and in how the sensor is mounted, not in a second count.

### 5.5 What would behave differently

| Today | With the target |
|---|---|
| A flower jostle in an auto ends when the slot count reads 4, about 1.3 s after the fourth element is first seen | it ends when the fourth element enters |
| The "Full" warning and FULL can disagree | they cannot |
| The launched tally takes the higher of two counts | it takes held |
| The count is 0 at the start of every opmode | it starts from the preload, the auto's count, or the count before a restart |
| Two guards exist and are switched off (slot agreement before FULL; a reset when the slots read 0) | deleted |

In COUNT ONLY mode nothing acts on FULL, as now.

### 5.6 Waits on

- The edge counter's first run on the robot. Nothing else rests on it until then.
- A check with nectar. The counter was characterized with pollen only.
