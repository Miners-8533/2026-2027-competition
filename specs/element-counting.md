# Element counting

The robot counts what it holds in two independent ways.

| Count | Sensors | Answers | Used for |
|---|---|---|---|
| **Slot count** | CS1, CS3 and CS4 along the track. CS2 left the slot count on 2026-11-18, when it was moved to the top of the feed ramp (section 1.8); it is recorded there for the launch count (target T11, section 6) | which kind of element sits at each of three places | the pusher check (the track reads empty), the opponent-nectar warning, seeing the pusher arm, the log |
| **Entry count** | CS5 at the entrance | how many elements have come in since the track was emptied, and what each one is (section 2.8, built 2026-11-17, not run on the robot) | **how many are held, and full**: FULL and its sequence, the driver screen, the end of a flower jostle in the autos, the "Full" warning, the launched tally. The kinds make the **load order**, which is logged and not yet acted on |

**The entry count is the count the robot relies on** (team decision, 2026-11-17; built 2026-11-17; first run on the robot in ten practice TeleOps of 2026-11-17, where it undercounted: section 2.7). The slot count undercounts in a crowded track, which is the worse error on this robot, so it no longer says how many are held. One switch ("count by entries", on) returns the jostle end, the "Full" warning, the non-always-on intake stop and the launched tally to the slot count, in case the entry count misbehaves on the robot. With CS5 missing or not answering the robot is never full by it and there is no fall-back: a jostle ends on its deadline and the screen shows a question mark. How the coordinator uses the counts is in [mechanisms.md](mechanisms.md).

Sensor physics and measured signatures: [robot_design/sensors.md](../robot_design/sensors.md).

## 1. Slot count (CS1 to CS4)

### 1.1 Reading

- **One sensor is read every 40 ms**, in rotation, and never more than one per loop. A full refresh of the three takes about 120 ms. (Each read blocks about 13 ms and the sensors only produce a new value every 100 ms.)
- The counter is **valid** only while every sensor has been read within the last 1 s. Callers must not act on it otherwise.
- The three track sensors must be configured, or the counter does not exist. CS2 is not needed.

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
- **Full:** the count is 4. With three sensors in the track this can no longer happen (section 1.8).

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
- A compact text of the slots for the log, for example `.-RB`: CS1 to CS4 in the order of their numbers (P pollen, R red nectar, B blue nectar, O other, A arm, a dot for empty). The second place is always a dash: CS2 is not a place along the track. The text keeps four places so that older logs and the tools read alike.
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

### 1.8 CS2 is no longer in the track (2026-11-18)

**CS2 was moved to the top of the feed ramp, past the HOLD position** (team, 2026-11-18), to count launches. Its hub, bus and name are unchanged. The runs of 2026-11-17 still had it in its old place.

**Built 2026-11-18, not run on the robot:** the slot count works with the three sensors left in the track.

- CS2 is not read by the slot count, is never an element, has no empty reading, and is not needed for the counter to exist or to be valid. Its numbers in the log's `COUNTER EMPTY` lines are 0.
- The read rotation is CS1, CS3, CS4.
- The gap flag takes CS1 and CS3 as neighbours.
- The Pit Check asks only that CS2 answers. The Color Sensor Study measures no baseline for it.
- CS2 is read by its own reader instead (section 6.1).

**What follows from three sensors:**

| Effect | Consequence |
|---|---|
| The count is 3 at most | the slot count can never give full. The switch back to the slot count (the top of this page) therefore no longer gives a working "full": the jostle would end on its deadline and the non-always-on intake would never stop. The entry count is the only count that can say full |
| The autos' INIT screen shows the slot count of the preload | it reads 3 with 4 loaded |
| Section 2.7 takes "all four slots read pollen" as "at least 4 aboard" | true for the runs of 2026-11-17 only. From now on three slots reading pollen means at least 3 |
| `tools/logs/launch_stability.py` picks salvos that began with four full slots for its per-pollen figure | it finds none in new logs |

**Not checked on the robot:** that the three track sensors still read as before (the stored empty readings of CS1, CS3 and CS4 are unchanged), and whether the pusher arm, sweeping through the feed ramp, disturbs anything now that CS2 is not masked by the slot count's rules.

## 2. Entry count (CS5)

### 2.1 Sampling

- A **separate thread** reads CS5's raw proximity (one small read) every 25 ms and runs the counting rule on each sample. It starts with the first coordinator update, so nothing is read during INIT, and stops by itself when updates stop for 1.5 s.
- Some of those reads are a **longer read that also returns the color** (section 2.8). It replaces the small read, it is not added to it, and the count gets the same proximity value from either.
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
2. No further entry is possible until the signal has come back to 4 counts above the empty level or lower (2 counts until 2026-11-18; the change has not run on the robot).

Consequences, both accepted:

- An element that stays under the sensor is one entry for as long as it stays.
- A clump whose elements touch, so the signal never dips to the re-arm level, counts once.

**Why the re-arm level is 4.** A load picked up while driving comes in as one burst of about a second, and between its elements the signal only falls to 3 to 5 counts above the empty level. With the re-arm at 2 the robot counted 1 to 3 for a load of 4 (section 2.7). At 5 the count runs over on a pile-up. The rise stays at 6: at the 25 ms sample period a rise of 7 misses weak single elements, which are that high for one sample only.

If the signal stays above the re-arm level for 10 s, a note is logged (an element is parked there, or the empty level has moved) and no entries come until it drops.

### 2.4 Peak counting (the earlier rule, kept behind a switch)

With edge counting switched off, an entry is a local peak of the signal:

- a peak counts when it stands at least 6 counts above its surroundings (prominence), at least 0.06 s after the previous peak;
- **merge rule:** peaks less than 0.4 s apart form a group; a group containing a peak of 45 counts or more counts once (one hard arrival that rings), while a group of small peaks counts peak by peak;
- **valley rule (off by default):** a peak whose preceding valley is more than a set amount above the empty level is rejected, which removes the wiggles of a parked element.

It overcounted in real driving (a parked or rolling element makes several peaks), which is why edge counting replaced it. The recorded test runs still replay through it.

### 2.5 What the coordinator does with entries

- Each entry is logged with its number, its height, the proximity value, and its time. The number is 1 for the first entry of the run and counts every entry, added to the held count or not. Its kind is logged when it arrives, under the same number (section 2.8).
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
| Edge: rise threshold / re-arm threshold / samples to confirm | +6 / +4 counts / 2 |
| Edge counting in use | yes |
| Peak rule: prominence / minimum separation | 6 counts / 0.06 s |
| Peak rule: merge height / merge gap | 45 counts / 0.4 s |
| Peak rule: valley limit | off |
| Thread stops after no update for | 1.5 s |
| Thread statistics logged every | 15 s |
| Kinds: in use | yes |
| Kinds: a pass begins at / ends after | +2 counts / more than 2 samples below that |
| Kinds: a pass is closed after its entry (with no entry: after its start) by | 0.5 s |
| Kinds: color read with nothing in front | one read in 8 (every 200 ms) |
| Kinds: empty color level | the median of the last 3 empty color samples |
| Kinds: least rise that can be named | 60 counts (green + blue + red, summed over the pass) |
| Kinds: red nectar / blue nectar | red share 0.55 or more / blue share 0.40 or more (**first guesses**) |
| Kinds: pollen | green share 0.45 or more and blue share 0.20 or less |

### 2.7 Known limits

- The count was characterized with pollen only.
- **It undercounted in its first robot runs** (ten practice TeleOps of 2026-11-17, re-arm at 2). The sensor and its thread were sound throughout: no read errors, the empty level 65 or 66 in every run. But at the 37 salvos where all four slots read pollen, so at least 4 were aboard (count-only mode, loads begun on an emptied track), the entry count read under 4 at 19, 4 at 14 and 5 at 4.
- **The re-arm at 4 is judged by replay only**, against the slots as a lower limit: the same 37 loads give under 4 at 5, 4 at 22, 5 at 9 and 6 at 1. Whether a 5 is a fifth element or an overcount is not known. No run has a true count per load.
- An entry whose moment falls while the intake runs OUT, or within 0.4 s after, is not counted (section 2.5). After a short OUT by the operator this dropped real entries: 11 in the ten runs.
- Elements that touch closely enough that the signal stays above the re-arm level still count once.
- The signal level depends on how the sensor is mounted.
- Whether the entry count alone is good enough to act on has not been proven in matches.
- **No nectar has been recorded passing CS5 with its color.** The two nectar limits are guesses and the load order cannot be trusted for nectar until a run with nectar has set them.
- Touching elements that count once get one kind, from the colors of both.
- An element the robot did not see come in (a preload, or one carried from an opmode that had no kind for it) has no kind.

### 2.8 The kind of each entry, and the load order

**Built 2026-11-17, not run on the robot. Nothing acts on it: it is logged and shown on the verbose screen.**

**Purpose.** The slot sensors name what sits at four places, and miss elements between them. To launch a mixed load of pollen and nectar the robot has to know what it holds **in order**. CS5 sees every element once, as it comes in, so the kinds it gives, in the order of the entries, are the load order.

**Reading the color.**

- The thread's read is the longer read (status, proximity and the infrared, green, blue and red channels in one transaction) in three cases: through the first second of a run, while the empty proximity level is being measured (section 2.2); on every read of a pass, from its beginning until it is closed; and otherwise on one read in 8: seven small reads follow each longer read.
- Every other read is the small proximity read, as before. In the recorded run about a fifth of the reads carried the color.
- The sensor's color is set to produce a new 16-bit value every 25 ms when the counter is built.
- A color value is used only when the sensor reports it as new since the last color read.
- A longer read that fails is followed in the same sample by the small read, so the count misses no sample. The next read is a longer read again.
- If the color setting fails, or three color reads in a row fail, the color is off for the rest of the run: the thread reads the proximity alone, the count is not affected, and every kind is unknown. The log says so.
- The kinds need edge counting (section 2.3). With peak counting there are none.
- One switch turns the kinds off and returns the thread to the small read alone.

**An empty color sample** is a new color value read while no pass is open and nothing is parked (below):

- every one of the first second. The empty proximity level is not known yet, and the intake must be clear for that second anyway (section 2.2);
- afterwards, every one read with the proximity under 2 counts above empty.

**The empty color level** is the median of the last 3 empty color samples, each channel on its own. With two samples so far it is their mean, with one it is that sample. It is fixed at the start of a pass. A pass that begins before there is any empty color sample adds nothing to its sums, and its entry's kind is unknown.

**A pass.**

1. It begins when the proximity is 2 counts or more above the empty level. That is below the count's own threshold of 6, so the color is already being read when the entry is counted.
2. On every new color value of the pass, the rise of each channel over the empty color level is added to that channel's sum. A value read on the sample that ends or closes the pass is added before it ends.
3. It ends when the proximity has been below 2 counts for more than 2 samples in a row. The color is an average over 25 ms and lags the proximity, so the samples just after the element has gone still carry it.
4. A pass in which no entry was counted gives nothing.
5. If the element is still in front 0.5 s after its entry, the pass is closed with what it has, and the element is **parked** (below).
6. A pass that has no entry 0.5 s after it began is dropped, and what is in front is parked the same way.
7. If a second entry is counted before the first one's pass has ended, the first is closed at that moment and a new pass begins for the second, from the same empty color level. A color value read on that very sample belongs to the second.

**Parked.** No pass is open and no empty color sample is taken. It lasts until the proximity has been below 2 counts for more than 2 samples in a row, or until an entry is counted, which begins a new pass on that sample. While it lasts the thread reads as it does with nothing in front: one read in 8 is the longer read, and its color is not used.

**The kind rule.** A channel whose sum is negative counts as 0. The three sums' shares of their total are tested in this order:

| Order | Test | Kind |
|---|---|---|
| 1 | no color value in the pass, or the total rise is under 60 counts | unknown |
| 2 | red share 0.55 or more | red nectar |
| 3 | blue share 0.40 or more | blue nectar |
| 4 | green share 0.45 or more and blue share 0.20 or less | pollen |
| 5 | anything else | unknown |

- Pollen, measured (31 pollen, 2026-11-14): green 0.52 to 0.65, blue 0.00 to 0.13, red 0.31 to 0.46.
- **The nectar limits are first guesses**, from the colors themselves. Every result is logged with its three shares, its total rise and its number of color values, so the limits can be set from the first run with nectar.

**The load order** is a text of one letter per held element, in the order they came in: `P` pollen, `R` red nectar, `B` blue nectar, `?` unknown. The first letter is the element nearest the flywheel, the one a salvo launches first. Empty when nothing is held.

- An entry that is added to the held count (section 2.5) is added to the load order as `?`. Its kind arrives when its pass closes, usually within about 0.25 s and at most 0.5 s after the entry, and replaces the `?`.
- A kind whose entry is no longer in the load (it was not counted as held, or the track has been emptied since) is logged and dropped.
- The load order is emptied whenever the held count restarts at 0.
- A salvo that did not complete puts its elements back in front of whatever has entered since, as it puts the count back.
- A count that starts from a known load (section 2.5) starts with the load order saved with it. The count decides the length: letters beyond the count are dropped, and missing letters are `?`. A letter other than `P`, `R` or `B` is `?`. A preload starts as `????`.
- The autos and the competition TeleOp save the load order with the held count on every loop.

**Where it shows.** The log: one line per kind, the load order on the line of a salvo's start and on the line of the count's start value, and a column in every sample row ([logging.md](logging.md), [mechanisms.md](mechanisms.md) section 14). The verbose screen and the TeleOp's INIT screen. Not the match screen.

**The log lines.** No tool reads them yet. Their form is given here so that the limits can be read off the log of any build in the same way.

- The kind of an entry: `CS5 KIND <n>: <KIND> (shares green 0.58, blue 0.05, red 0.37; rise 412 counts over 9 color samples, 0.21 s) | load order PP?`
  - `<n>` is the entry's number, the one on its `CS5 ENTRY <n>` line (section 2.5).
  - `<KIND>` is `POLLEN`, `RED_NECTAR`, `BLUE_NECTAR` or `UNKNOWN`.
  - The shares have two decimals and the rise none. The time has two decimals and runs from the beginning of the pass to the result.
  - An unknown kind says why, after `; ` and before the closing bracket: `no color sample in the pass`, `no empty color level yet`, `the rise is under 60 counts` or `the shares fit no kind`.
  - When the entry is no longer in the load, the line ends `| not in the load (it was not counted as held, or the track has been emptied since)` in place of the load order.
- An empty load order is written `-` in these lines. In the sample column it is empty.
- The count's start value: `ENTRY COUNT starts at <n> (was <m>), load order <text>: <why>`.
- A salvo's start: the `SALVO started` line ends ` | load order <text> (the first letter leaves first)`.
- Three failed color reads in a row: `CS5 KIND OFF: three color reads in a row failed (<error>): the color is off for this run, every kind is UNKNOWN`.
- A failed color setting gives no `CS5 KIND OFF` line. The `CS5 ENTRY COUNTER started` line ends with whether the kinds are on and with the result of the setting: `color preset applied (16-bit, a new value every 25 ms)` or `COLOR PRESET FAILED (<error>): the kind of an entry will be UNKNOWN`.
- The thread's statistics line (every 15 s) ends `color: <n> reads in all, <n> failed, empty level <green>/<blue>/<red>` (the empty color level, in whole counts), followed by `, OFF` once the color reads were given up, or by `, switched off` with the switch off. The empty level is `-` before the first empty color sample. With the kinds never on (peak counting, or a failed setting) it ends `color: not read`.

**What is left.**

- A run with red and blue nectar fed through the intake, to set the two nectar limits.
- A check of the main loop's time with the color reads on, against a run with them off.
- Whatever is to act on the load order: where the pusher stops between elements, the flywheel position per element, the opponent-nectar warning from CS5 in place of the slot sensors. None of it is designed here.

## 3. Status

- Slot count: used at Hartland with four sensors. With three (CS2 moved to the feed ramp after the runs of 2026-11-17, section 1.8): built 2026-11-18, not run.
- CS2 at the top of the feed ramp: recorded through every salvo since 2026-11-18 (section 6.1), not run. Nothing counts launches yet.
- Entry count: added after Hartland. Peak counting and a first version of the FULL actions ran in practice on 2026-11-14. Edge counting, the count as the robot's only count, the count carried between opmodes and the FULL sequence ran in ten practice TeleOps on 2026-11-17 (six in count-only mode, four acting on FULL), with the re-arm at 2. The re-arm at 4 (2026-11-18) has not run. What the FULL sequence did in those runs has not been examined beyond the count.
- Kinds and the load order: built 2026-11-17 from the one recorded run that holds CS5's color (31 pollen). Not run on the robot, and never tried on nectar.

## 4. Verification by replay

The counting rule is one pure piece of logic with no robot dependencies, so recorded proximity traces can be replayed through exactly the code the robot runs. A PC check does this against recorded runs with known answers (a hand-fed run of 31 elements, also thinned to the robot's 25 ms; TeleOp runs with known loads) and against the driven loads of three of the 2026-11-17 runs, where the slots give the least each load can have held. Any change to the rule should be replayed before it goes on the robot.

The kind rule and the pass logic are a second pure piece, checked the same way: the recorded pollen run is replayed with the color taken only on the reads where the robot would take it, at 20 ms and thinned to 25 ms (30 of its 31 entries are named pollen, the other is unknown, none is named nectar), and made-up signals cover the nectar kinds, a parked element, touching elements and a pass with no entry. The nectar cases are made up, so they check the logic and not the limits.

## 5. Target T3: the entry count is the count

**Built 2026-11-17, first run on the robot the same day.** This section records what the target was and what is left of it.

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
- **The count ran low in its first runs** (section 2.7), so a load could reach five. The re-arm level was raised for that on 2026-11-18; elements that touch closely still count once. The answer to that is in the counting rule and in how the sensor is mounted, not in a second count.
- The counter was characterized with pollen only; it needs a check with nectar. The kinds (section 2.8) need the same run.

**Waits on** a run with the re-arm at 4 and a true count per load (someone counting what each salvo launches). The switch is the way back.

## 6. Target T11: CS2 counts what a salvo launches

**Decided by the team on 2026-11-18, when CS2 was moved to the top of the feed ramp. Its first two stages are built and not run: CS2 has left the slot count and is recorded through every salvo. Nothing counts launches yet.**

**The weakness.** Nothing on the robot sees an element leave. The launched tally is the held count at the start of a salvo, so it is only as good as the entry count, and the entry count has no true number to be judged against (section 2.7). A salvo that leaves an element behind is not noticed.

**Target.** CS2 counts the elements that go up the feed ramp in each salvo: the **launch count**.

| Use | What it replaces |
|---|---|
| The true number of each load, in every log, to judge the entry count's rule | someone counting what each salvo launches |
| The launched tally | the held count at the start of the salvo |
| What is still held after a salvo: the held count less the launch count | the held count restarting at 0 at every salvo |
| A salvo that launched fewer than were held | nothing: it is not noticed today |

The launch count does not declare FULL. It arrives after the elements have gone.

**What has to be measured first.** None of this is known for CS2's new place:

- how high and how long the reading of one launched element is, for pollen and for nectar;
- whether the elements of one salvo are apart in the reading, or run together as the driven loads do at CS5;
- what the pusher arm reads as it follows the last element through, and at which angle;
- how often CS2 must be read to see every element, and what that costs the main loop **during a salvo**, when the loop time sets how exactly the pusher stops.

**Reading.** CS2's standard readings (a new value every 100 ms, read once in 160 ms) cannot follow a salvo, which launches four elements in about 1.2 s. The launch count needs CS2's fast proximity reading, as CS5 has (section 2.1), and only while a salvo runs. In that mode CS2 no longer gives a usable distance, so it could not stay in the slot count.

**Stages.** Each is run on the robot before the next is built.

| Stage | What | Changes behavior | State |
|---|---|---|---|
| 1 | **CS2 leaves the slot count** (section 1.8) | yes: the slot count reads at most 3 | built 2026-11-18, not run |
| 2 | **Record.** CS2 is read fast through every salvo and every sample is logged (section 6.1). The team wanted the data from the competition TeleOp, so this is in the match opmodes and not in a study opmode | only the loop time during a salvo | built 2026-11-18, not run |
| 3 | **Count in shadow.** A rule made from the recordings counts in the match opmodes and is logged beside the held count at every salvo. Nothing acts on it | no more than stage 2 | not built: needs recordings |
| 4 | **Use it:** the launched tally, what is still held after a salvo, and a warning for a salvo that launched fewer than were held | yes | not built |

### 6.1 The recording (built 2026-11-18, not run on the robot)

A reader of its own, on its own thread, reads CS2's raw proximity (one small read, as for CS5). It counts nothing and nothing acts on it.

- **Sensor setting:** fast proximity mode, set when the reader is built (INIT). If that fails the log says so and the samples are too slow to follow a salvo.
- **Fast stretch:** one read every 12.5 ms from the moment a launch is queued, through the salvo, until 0.6 s after it. **12.5 ms is a first value**: how long a launched element is in view is not known.
- **Otherwise:** one read every 250 ms. These give the empty level (the median of the last 9) and show anything that stays in front of the sensor between salvos.
- The thread starts with the first coordinator update, so nothing is read during INIT, and stops by itself when updates stop for 1.5 s. One switch turns the reader off.
- It runs in every opmode built on the coordinator (the competition TeleOp, the autos, the tuners), wherever a `CS2` is configured.

**In the log.**

- Sample column `cs2f`, the last column of the coordinator's part of every row: every raw sample since the previous row as `prox@-ms`, ms before that row's own time, the form of CS5's `cs5f`. During a salvo the rows are written every loop, so each sample can be set against the pusher's angle.
- `CS2 LAUNCH SENSOR started ...`: the settings, and whether the fast proximity setting applied.
- `CS2 LAUNCH SENSOR fast stretch: ...`, once after every salvo: the number of samples, the real interval (mean, longest, how many late), the read time and its share of the hub, the lowest and highest proximity, the empty level, read errors.

**The cost is not known.** The fast reads take hub time from the main loop while a salvo runs, and the loop time then sets how exactly the pusher stops. CS5's reader at 20 ms cost the loop about 6 ms (section 2.1); this one runs beside it. The first run has to compare the loop time and the pusher's rest position through salvos with the reader on and off, and settle the period.

**What the first recordings have to answer** is the list under "What has to be measured first" above. The held count and the slots at the start of each salvo, already in the log, say roughly how many elements each recording holds; a run with known loads says it exactly.

**Waits on** the first run on the robot.
