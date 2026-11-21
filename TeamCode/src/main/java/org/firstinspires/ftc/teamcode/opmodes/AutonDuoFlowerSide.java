package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.LaunchGridMap;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.InstantAction;
import org.firstinspires.ftc.teamcode.motion.Jostle;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;

/**
 * Duo auto, the FLOWER side (the audience wall): the partner runs {@link AutonDuoGardenSide}. Blue frame; red is turned 180 deg about the center.
 *
 *   1. Start with the back flush to the audience wall (heading 180); the flywheel spins from Start at the 2-D launch grid's speed and the pusher goes to the LAUNCH-READY position.
 *   2. Aim at the audience cell at once (the formula aim, from the start pose), then hold still until F_FIRST_LAUNCH_NOT_BEFORE_SEC (9.0 s: the partner's preload salvo and the tip) and launch the preload
 *      the moment it passes (the cell the tip just opened; it needs 8 pollen): the robot is aimed, at speed and at launch-ready through the wait (2026-11-21, user; the aim came after the wait before).
 *   3. Deploy the ramp, gather the audience-wall flower as the solo auto does (ram, one jostle cycle, ended early when full or at F_JOSTLE_DEADLINE_SEC), back straight out, go to the Solo auto's launch
 *      spot, blue (56, 36) (the ramp stows during the turn, the pusher goes to the launch-ready position) and launch the flower pollen with the grid's speed and aim: the 8th pollen, which tips that cell.
 *   4. Park in the audience-side corner of the loading zone, straight from the launch spot, ramp stowed.
 *
 * 2026-11-21 (user: the learnings of the three enabled autos, Solo, Daryl and Triple Tip; the app builds, NOT RUN on the robot, and this auto has never run): the launch-ready pusher for both salvos (it was the
 * 90 deg lock for the preload only), the grid's flywheel speed from Start, the flower salvo from (56, 36) with the grid's speed and aim (it was (61.5, 7.5) at the start wall with the formula aim), the faster
 * legs (0.85, as Solo and Daryl; they were 0.6), the park 2 in farther from the field center and without its via point. The jostle's strokes, the ram and the launch legs' tolerances are shared values that
 * already changed with the others. Shown on the Driver Station again from 2026-11-21 (user).
 */
@Autonomous(name = "Auton - Duo Flower Side", group = "Auto")
public class AutonDuoFlowerSide extends AutonDuoBase {

    @Override
    protected String autoName() {
        return "Auton - Duo Flower Side";
    }

    @Override
    protected Pose2d blueStart(ChassisProfile profile) {
        return DuoKit.pose(wallStartX(profile, 1.0), DuoAutos.F_START_Y_IN, 180.0);
    }

    @Override
    protected String placementText(boolean blue) {
        return "Back against the AUDIENCE wall at y " + (blue ? "+" : "-") + DuoAutos.F_START_Y_IN + ", facing the field.";
    }

    @Override
    protected Action buildAuto(Context c) {
        MecanumDrive drive = c.drive;
        Mechanisms m = c.mechanisms;
        DuoKit k = c.kit;
        boolean blue = c.blue;
        k.fastLegs(DuoAutos.P_LEG_MAX_POWER, DuoAutos.P_CROSS_MAX_POWER, DuoAutos.P_PASS_SETTLE_SEC);   // the Solo and Daryl autos' leg speeds (2026-11-21; they were the drive's default 0.6)

        HiveTarget cell = DuoKit.cellForAlliance(HiveTarget.BLUE_AUDIENCE, blue);   // blue: audience, red: rear
        m.setOpenCell(cell);

        // 2026-11-21: the Solo auto's launch spot, blue (56, 36) / red (-56, -36), with the 2-D grid's aim (the bearing plus the grid's residual there) and speed; it was (61.5, 7.5) with the formula aim
        Pose2d spotXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.F_FLOWER_LAUNCH_X, DuoAutos.F_FLOWER_LAUNCH_Y, 0.0), blue);
        LaunchGridMap.Result grid = LaunchGridMap.lookup(cell, spotXY.position.x, spotXY.position.y);
        double aim = grid.hasValues()
                ? Math.atan2(cell.y - spotXY.position.y, cell.x - spotXY.position.x) + Math.toRadians(grid.residualDeg)
                : TargetLock.aimHeadingRad(spotXY.position.x, spotXY.position.y, cell, 0.0);
        Pose2d launchSpot = new Pose2d(spotXY.position.x, spotXY.position.y, aim + Math.toRadians(DuoAutos.F_FLOWER_TRIM_DEG));
        Pose2d flowerPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerRam = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerBack = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.F_PARK_X, DuoAutos.F_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.PARK_HEADING_DEG), blue);

        return new SequentialAction(
                // the preload into the audience cell from the start pose: the flywheel speed from the 2-D grid (the formula aim and the position are unchanged) and the pusher at the LAUNCH-READY position
                // (2026-11-21, as Solo and Daryl; it was the 90 deg lock), where it stays through the wait for the partner's tip
                new InstantAction(() -> m.setLaunchGrid(true, "Auton - Duo Flower Side, from Start (the preload: the grid's flywheel speed only)")),
                new InstantAction(m::requestLaunchReady),
                // aim first, then the wait, then the salvo at once (2026-11-21, user: aimed and ready during the wait; the aim came after the wait before, so the salvo started a few tenths later)
                k.aimWaitLaunch(drive, m, cell, blue, "preload", DuoAutos.F_PRELOAD_TRIM_DEG, "first launch (partner's tip)", DuoAutos.F_FIRST_LAUNCH_NOT_BEFORE_SEC, false),

                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("To the flower pre-position, turning to the flower"),
                k.leg("flower pre-position", drive, flowerPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: flower (held IN, whatever the count, until the flower salvo is done)"),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the flower", drive, flowerRam),
                k.timed("flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.F_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.note("Back off the flower to the launch spot (intake still ON)"),
                k.backOutLeg("flower back-out (straight)", drive, flowerBack),
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position while the robot turns to the launch spot
                k.stowWhenTurned(k.launchLeg("launch spot", drive, launchSpot), drive, m, flowerPre.heading.toDouble()),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.launchOnGrid(drive, m, cell, blue, "flower pollen", DuoAutos.F_FLOWER_TRIM_DEG),

                k.note("Park (straight from the launch spot: it is clear of the flower and the support rails)"),
                k.parkLeg("park (full power)", drive, park),
                k.timed("ramp stow (final)", k.rampTo(m, false)),
                new InstantAction(m::stopFlywheel)
        );
    }
}
