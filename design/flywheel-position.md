# Flywheel position servo (PROTOTYPE, branch `flywheel-servo-prototype`, 2026-11-15)

A servo moves the **flywheel** between two positions: one for launching **pollen**, one for **nectar**. The servo's channel is not decided yet, so nothing in the robot depends on it: it is off by default and the hardware configuration does not need it.

## What is built
- `subsystems/FlywheelPosition.java`: a plain position servo (like the `Ramp`): `set(POLLEN | NECTAR)`, `toggle()`, `isSettled()` (TRAVEL_SEC since the last command, no feedback). Dashboard values: `SERVO_NAME` ("fly_pos"), `REVERSED`, `POLLEN_POSITION` and `NECTAR_POSITION` (**NaN until measured: nothing moves until both are set**), `TRAVEL_SEC` (0.6, to be measured). Nothing is commanded when it is built.
- `opmodes/bringup/FlywheelPositionTest.java` (**Flywheel Position Test**): lists every servo in the Robot Configuration and lets you pick one in INIT (D-pad left / right), so the channel does not have to be decided first; nothing is commanded at Start, **the first command moves the servo to `START_POSITION` (0.50)**. A = pollen, B = nectar, Y = switch, D-pad up / down = +/- 0.01 on the position being adjusted (LB: 0.05), X = reverse the direction. The screen and the log give the line to paste (`SERVO_NAME`, `REVERSED`, `POLLEN_POSITION`, `NECTAR_POSITION`).
- `Mechanisms` (everything behind `FLYWHEEL_POSITION_ENABLED`, false): `flywheelPosition` is built with the rest (null when the servo is not in the configuration); when enabled and both positions are measured the TeleOp **starts in the POLLEN position** (the first command, at Start, like the ramp's stow), a **launch waits until the servo has settled** (`LAUNCH held: the flywheel servo is still moving`; the launch timeout still applies), and `toggleLaunchElement()` switches the position (not while a launch or reject is running).
- `TeleOpCompetition`: the **operator's A** (alone; LB + A is still the launch grid toggle) switches pollen / nectar; the screen shows `NECTAR MODE` (cyan) while it is in the nectar position; the verbose screen has a `Flywheel position` line.

## Bring-up, in order
1. Add the servo to the Robot Configuration (name it `fly_pos`, or any name and put it in `SERVO_NAME`), and restart the robot app.
2. **Flywheel Position Test**, with the flywheel clear: pick the servo, find the pollen and nectar positions that launch best, check the direction (X), time how long the servo takes between the two.
3. Paste `POLLEN_POSITION`, `NECTAR_POSITION`, `REVERSED` and `TRAVEL_SEC` into `FlywheelPosition` (or the Dashboard), set `Mechanisms.FLYWHEEL_POSITION_ENABLED` true.
4. TeleOp: at Start the flywheel goes to the pollen position; the operator's A switches; a launch right after a switch waits for the servo.

## Open questions (not decided)
- Which servo channel, and the servo's travel and its mechanical limits.
- Whether nectar needs its own **speed and aim** (the launch map, the 2-D grid and the aim trims are all measured with pollen): probably a second launch map selected by the position. The Launch Tuner would need a nectar mode.
- How the position is chosen in a match: the operator's A now; automatically from what the robot holds (the counter classifies nectar, but it does not see every element) or from CS5's color is possible later.
- G403: the first command comes at Start (like the ramp), not in INIT.
- The autos: they never move it (`FLYWHEEL_POSITION_ENABLED` is read everywhere but nothing in the autos calls `set`); a position left over from the TeleOp stays where it is until the next command (the servo sags when the opmode ends, as the ramp's does).
