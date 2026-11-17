# Pusher and pusher check

The arm that sweeps the stack of elements into the flywheel, and the routine that verifies where the arm really is. When each move is requested is decided by the coordinator ([mechanisms.md](mechanisms.md)).

Hardware and angles: [robot_design/mechanisms.md](../robot_design/mechanisms.md). How CS4 sees the arm: [robot_design/sensors.md](../robot_design/sensors.md).

## 1. Position model

- The arm's position is an angle that accumulates across revolutions, measured forward from **HOLD** (0 degrees). A "HOLD" is any whole number of revolutions.
- All targets are absolute multiples of a revolution plus an offset, so landing errors never accumulate.
- **The zero.** The encoder is relative and is never reset. The zero is the encoder count that corresponds to HOLD.
  - *Assume at HOLD* (autos, a TeleOp with no auto before it): the zero is wherever the arm is when the pusher is built. The arm must physically be at HOLD.
  - *Continue* (the TeleOp after an auto): the zero saved by the previous opmode is reused.
  - The zero is saved in memory for the life of the robot app, with a flag **zero verified**. The flag is false whenever the zero is only an assumption and becomes true when a pusher check has seen the arm where the zero says, or has corrected it.
  - The zero can be redefined ("the arm was truly at angle A when the old zero said X"); the correction keeps the revolution count, so it is always the smallest shift.
- **Not specified:** when the pusher counts as *not calibrated* (a warning in [mechanisms.md](mechanisms.md) section 11 and a precondition of the pusher check). With the two ways of setting the zero above, a pusher that has been built always has a zero.

## 2. Control principle

Deliberately plain: **no PID and no speed loop** for moves. A move runs the motor at a fixed power and cuts it when the position crosses a threshold; the motor brake does the rest. (A PID and a velocity loop were tried on the bench and were jerky and hard to tune.)

A move ends when the remaining distance is at most

> stop-early angle + current speed x coast lookahead

The coast lookahead is 0 at present, so the cut is a fixed 3 degrees before the target.

The hub is read once per loop (position and speed) and the motor is written only when its power changes. The motor current is read only while a move might be blocked, and otherwise every 250 ms.

## 3. States and moves

| State | Meaning |
|---|---|
| HOLD | idle and braked, wherever the arm came to rest (not necessarily at a HOLD angle) |
| SALVO | forward move that launches the stack |
| RETURN | forward move to a HOLD |
| REJECT | reverse move |
| SEEK | move to a given position, either direction, at a given power |
| JOG | open-loop power, for bring-up only |
| STALLED | faulted (only possible when faults are enabled, which they are not) |

**Salvo.** Target = the nearest HOLD plus the salvo angle (400 deg). The arm therefore turns a full revolution past a HOLD and rests about 40 deg beyond the next one, so the last element is surely out. Because the target is computed from the *nearest HOLD* and not from where the arm is, a salvo started from that rest position travels about 355 deg, and one started from the lock position travels 310 deg. Nothing accumulates.

**Return (hold command).** Forward only. If the arm is short of the nearest HOLD by more than the tolerance, go forward to it. If it is past by more than the tolerance, go forward to the *next* HOLD (almost a full turn). Within the tolerance, just stop.

> Consequence: a return commanded with the arm resting past a HOLD sweeps the track and would launch anything in it. The coordinator only uses it with an empty track.

**Reject.** Reverse to the HOLD nearest to "current position minus the reject angle (330 deg)". From HOLD that is a full reverse turn.

**Forward moves** run at the sweep power and drop to the approach power for the last approach angle. **Reject** runs at the reject power with the same approach rule. All three powers are 0.30 at present.

**Seek.** Go to a given absolute position at a given power, forward or reverse, with the same early cut. Used by the pusher check and for the lock.

**Lock and unlock.** *Lock* is a seek to 90 deg forward of the nearest HOLD: the arm lies across the intake entrance. It nudges the stack forward but does not launch anything. *Unlock* is a seek back to that HOLD, **reverse only**; a forward return from the lock would sweep the stack into the flywheel. If the arm is already at or behind the HOLD, unlock does nothing. The arm counts as *locked* when it is idle, stopped and within 12 deg of the lock angle.

**Other commands.** *Cancel move* and *abort seek* stop the arm where it is. *Stop now* is for the end of an opmode: it stops and removes all power, including the active hold.

**Queries the coordinator relies on.**

| Query | True when |
|---|---|
| At HOLD | idle, stopped, within 8 deg of a HOLD |
| Stopped | idle and not moving, wherever it rests. A salvo may start from here |
| Locked | idle, stopped, within 12 deg of the lock angle |
| In launch sweep | a salvo is running and has turned less than 360 deg past the HOLD it started from |
| Gave up | the last move did not finish; cleared when the next move starts |

## 4. At rest

**Settle nudge.** If the arm comes to rest *short* of a HOLD by more than the tolerance but no more than 45 deg, it nudges forward to the HOLD after 0.3 s. It is never driven back from an overshoot. The pusher check switches the nudge off while it parks the arm on purpose.

**Active hold.** The motor brake resists motion but not a steady push, so the stack's pressure can walk the arm away. Once the arm has been still for 0.3 s, wherever it rests, that angle is latched. If it drifts more than the dead band, a proportional power pushes it back:

- power = gain x error, at least the minimum and at most the maximum, rounded to 0.01 steps (each change is a hub write);
- if the maximum is needed for 2 s without holding, the hold lets go until the next move, so the motor is not cooked;
- it is not used after a move gave up.

It reports once per rest when it had to correct more than about 4 deg, and when it gave up.

## 5. A move that cannot finish

**Faults are disabled.** There is no stall fault and no progress fault, so the drive team never has a fault to read or clear in a match. Instead, a move that has not finished within 3 s stops where it is, the pusher goes idle with *gave up* set, and nothing nudges or checks the arm until the next move is commanded. (With faults enabled, a stall above 6 A at zero speed, or a move gaining under 2 deg in 1.5 s, would enter STALLED until cleared.)

**Automatic jam resolution.** Applies only to forward moves that push elements: a salvo, a return, and the lock seek. It never applies to a reject or an unlock (backing off from those would sweep elements forward) and never to the pusher check's own seeks (a back-off would corrupt the check's measurement).

1. **Detect.** After 0.4 s into the move, the arm is slower than 20 ticks/s with more than 1.2 A for 0.3 s.
2. **Back off** in reverse at 0.25 power:
   - a jammed **salvo or return** backs off 45 deg from where it is stuck;
   - a jammed **lock** backs off to 45 deg *behind* the HOLD, deliberately past it, to take a run-up;
   - never less than 10 deg.
3. **Resume** the original move toward its original target.
4. At most 2 back-offs per move in the TeleOp and 1 in an auto. The move's own 3 s timer keeps running throughout, so a jam never costs more than an unresolved move did before.
5. When the back-offs are used up, the move gives up at once and the jam is flagged as unresolved until the next move starts.

Reports: a line when a jam is found, when a jammed move finishes, and when it is given up; a one-time "jam cleared" notice for the drivers.

## 6. Parameters

| Name | Value |
|---|---|
| Ticks per arm revolution | 751.8 |
| Encoder sign | -1 |
| Sweep power / approach power / reject power | 0.30 / 0.30 / 0.30 |
| Approach angle | 30 deg |
| Stop-early angle | 3 deg |
| Coast lookahead | 0 s |
| Salvo angle | 400 deg |
| Reject angle | 330 deg |
| HOLD tolerance | 8 deg |
| Stopped below | 30 ticks/s |
| Settle nudge: at most | 45 deg short |
| Active hold: gain / minimum / maximum power | 0.012 per deg / 0.05 / 0.15 |
| Active hold: dead band / latch after / give up after | 2 deg / 0.3 s / 2 s |
| Default seek power | 0.2 |
| Lock angle / lock power / lock tolerance | 90 deg / 0.25 / 12 deg |
| Move timeout | 3 s |
| Jam: current above / speed below / for / after | 1.2 A / 20 ticks/s / 0.3 s / 0.4 s into the move |
| Jam: grace after resuming | 0.3 s |
| Jam back-off: salvo / lock (behind HOLD) / minimum / power | 45 deg / 45 deg / 10 deg / 0.25 |
| Jam retries: TeleOp / auto | 2 / 1 |

## 7. Pusher check

**Purpose.** Verify, and if needed correct, the pusher's zero using CS4, which sees the arm once per turn. It catches a zero that was assumed wrongly at power-up and any belt slip (one tooth is 18 deg, more than the window in which the parked arm is seen).

**Preconditions.** The arm is idle and stopped, the pusher is calibrated, and the element counter is reading all four sensors. The coordinator additionally requires an empty track.

**Stages.**

1. **Park.** Seek (in reverse, from the usual rest position) to the angle where CS4 should see the arm's center: 326.7 deg forward of HOLD. The seek target is moved 4.4 deg back toward where the arm comes from, because a seek coasts about that far past its target.
2. **Look.** Wait until the arm has been still 0.25 s and CS4 has delivered 3 fresh readings (at most 1.5 s), then ask whether CS4 classifies what it sees as the pusher arm.
   - **Seen:** the zero is right. Go to step 5. Result **OK**.
   - **Not seen:** go to step 3.
3. **Sweep.** Turn forward continuously at a slow power for up to 420 deg, reading CS4 every loop, until the arm has crossed the sensor completely (not seen, then seen, then not seen again). The sweep covers more than a full turn because, wherever the belt slipped to, the arm must pass the sensor within one turn.
4. **Cross back and correct.** Reverse 90 deg to cross the sensor the other way. The sensor reports the arm late, which shifts each crossing in its direction of travel; the **average of the two crossing centers** is the true position with no delay constant. Redefine the zero so that this point reads 326.7 deg. If the return crossing is not seen, use the forward crossing alone, corrected by speed x 0.15 s. Then park and look again (step 1 and 2) to **verify**.
   - Verified: go to step 5. Result **SWEEP**, with the total correction in degrees and teeth.
   - Not verified: sweep again, up to 3 sweeps in all, then result **LOST**.
5. **Return.** Send the arm forward to HOLD.

**Other rules.**

- A sweep that ends with the arm still in view goes once more for a whole crossing.
- A sweep that never sees the arm, a pusher fault, or 25 s in total gives **LOST**: the zero is not trusted and the coordinator warns.
- **Abort** (the coordinator calls it when something else needs the pusher, or the counter stops reading): the seek stops, the arm is sent forward to HOLD, result **ABORTED**. A previous LOST stays LOST.
- OK or SWEEP sets *zero verified*.
- **Calibration sweep** (started by the operator): skips steps 1 and 2 and goes straight to the sweep. The caller must make sure the track is empty.
- The check drives the pusher only through seeks and the hold command, and switches the settle nudge off while it runs.

**Typical durations:** about 1 s when the zero is right; 2 to 4 s when the arm is behind where the zero says; up to about 6 s when it is ahead.

| Name | Value |
|---|---|
| Arm center seen by CS4 | 326.7 deg forward of HOLD |
| Pulley teeth | 20 |
| Settle before a look | 0.25 s |
| Fresh CS4 readings for a look / look timeout | 3 / 1.5 s |
| Seek coast compensation | 4.4 deg |
| Sweep power / sweep angle / back angle | 0.15 / 420 deg / 90 deg |
| Maximum sweeps | 3 |
| Sensor delay used when only one crossing is seen | 0.15 s |
| Total time limit | 25 s |

**Status:** the check and the calibration sweep ran on the robot before Hartland. The jam resolution was in the Hartland build; excluding the check's seeks from it was added afterwards and has not run.
