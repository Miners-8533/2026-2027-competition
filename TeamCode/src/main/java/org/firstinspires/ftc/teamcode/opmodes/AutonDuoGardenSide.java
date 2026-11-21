package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
import org.firstinspires.ftc.teamcode.drive.LaunchGridMap;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TargetLock;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.InstantAction;
import org.firstinspires.ftc.teamcode.motion.Jostle;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.motion.SquareOnWall;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;

/**
 * Duo auto, the GARDEN side (the rear wall): the partner runs {@link AutonDuoFlowerSide}. Blue frame; red is turned 180 deg about the center. Every salvo goes into the rear cell (red: the audience cell).
 *
 *   1. Start like the solo auto (back to the rear wall, y 16, heading 0). The flywheel spins at the 2-D launch grid's speed and the pusher goes to LAUNCH-READY; launch the preload into the rear cell from the
 *      start pose with the formula aim (the cell starts with 3 nectar: the 4 pollen tip the hive).
 *   2. Gather the Blue Wall flower (the audience-wall flower turned 90 deg: ram, one jostle cycle, back straight out).
 *   3. To the rear launch spot, blue (-55.6, 26.2) (the Triple Tip's), the ramp stowing and the pusher going to launch-ready; aim at once with the grid's aim and REST AIMED until
 *      G_SECOND_LAUNCH_NOT_BEFORE_SEC, then launch the flower pollen.
 *   4. The garden as the Solo auto gathers it (one leg to the end of the pickup, the last 12.5 in at 0.5 power, then the squaring for at most 0.8 s), back to the rear launch spot, aim at once and launch the
 *      garden pollen not before G_LAST_LAUNCH_NOT_BEFORE_SEC: the 8th, which tips the cell.
 *   5. Park at (22, 55.1) through the via point (-20, 44), as the Daryl and the Triple Tip autos do; ramp stowed.
 *
 * THE LAST TWO SALVOS ARE AS LATE AS THE PARK ALLOWS with a 1 s buffer (user, 2026-11-21): the last one at 24.8 s (30 - 1.0 buffer - 3.2 park - 1.0 salvo) and the one before it at 19.3 s (5.5 s earlier: its own
 * 1.0 s, then the garden, the squaring, the leg back and the aim, about 4.5 s). Both are ESTIMATES until this has run: see DuoAutos.G_LAST_LAUNCH_NOT_BEFORE_SEC.
 *
 * 2026-11-21 (user: the learnings of the other autos; the app builds, NOT RUN on the robot, and this auto has never run): the launch-ready pusher for every salvo (it was the 90 deg lock for the preload only), the
 * grid's flywheel speed from Start, the two later salvos from the Triple Tip's rear launch spot with the grid's speed and aim (they were from (-61.5, 15.25) at the start wall with the formula aim), the Solo's
 * one-leg garden with the 0.8 s squaring (it was the three-step garden with the full squaring), legs at 0.85 power (they were 0.6), the park at (22, 55.1) through (-20, 44) (it was (26, 53.1) through (-36, 40)),
 * and the two waits above (it was one wait, until 25 s, before the flower salvo, which could not fit the 30 s). Shown on the Driver Station again.
 */
@Autonomous(name = "Auton - Duo Garden Side", group = "Auto")
public class AutonDuoGardenSide extends AutonDuoBase {

    @Override
    protected String autoName() {
        return "Auton - Duo Garden Side";
    }

    @Override
    protected Pose2d blueStart(ChassisProfile profile) {
        return DuoKit.pose(wallStartX(profile, -1.0), DuoAutos.G_START_Y_IN, 0.0);
    }

    @Override
    protected String placementText(boolean blue) {
        return "Back against the REAR wall at y " + (blue ? "+" : "-") + DuoAutos.G_START_Y_IN + ", facing the field.";
    }

    @Override
    protected Action buildAuto(Context c) {
        MecanumDrive drive = c.drive;
        Mechanisms m = c.mechanisms;
        DuoKit k = c.kit;
        boolean blue = c.blue;
        ChassisProfile profile = c.profile;
        k.fastLegs(DuoAutos.P_LEG_MAX_POWER, DuoAutos.P_CROSS_MAX_POWER, DuoAutos.P_PASS_SETTLE_SEC);   // the Solo and Daryl autos' leg speeds (2026-11-21; they were the drive's default 0.6)

        HiveTarget cell = DuoKit.cellForAlliance(HiveTarget.BLUE_REAR, blue);   // blue: rear, red: audience
        m.setOpenCell(cell);

        // the rear launch spot of the two later salvos (2026-11-21): the Triple Tip's, blue (-55.6, 26.2) / red (55.6, -26.2), with the 2-D grid's aim (the bearing plus the grid's residual there) and speed
        Pose2d spotXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_REAR_LAUNCH_X, DuoAutos.G_REAR_LAUNCH_Y, 0.0), blue);
        LaunchGridMap.Result grid = LaunchGridMap.lookup(cell, spotXY.position.x, spotXY.position.y);
        double aim = grid.hasValues()
                ? Math.atan2(cell.y - spotXY.position.y, cell.x - spotXY.position.x) + Math.toRadians(grid.residualDeg)
                : TargetLock.aimHeadingRad(spotXY.position.x, spotXY.position.y, cell, 0.0);
        Pose2d launchSpot = new Pose2d(spotXY.position.x, spotXY.position.y, aim + Math.toRadians(DuoAutos.G_REAR_TRIM_DEG));

        // the Blue Wall flower: the audience-wall flower's poses turned 90 deg counterclockwise about the center (heading 90, facing the Blue Wall), plus the Dashboard nudge
        Pose2d wallPre = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);
        Pose2d wallRam = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);
        Pose2d wallBack = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);

        // the garden as the Solo auto gathers it: one leg to the end of the pickup at the angled heading, then the squaring on the side wall and the Blue Wall
        Pose2d gardenEnd = DuoKit.forAlliance(DuoKit.pose(DuoAutos.GARDEN_END_X, DuoAutos.GARDEN_END_Y, DuoAutos.GARDEN_HEADING_DEG), blue);
        double sideIn = profile.originFromSideIn();
        double squareX = -(FieldGeometry.HALF_IN - sideIn - DuoAutos.SQUARE_GAP_IN);
        Pose2d squarePre = DuoKit.forAlliance(DuoKit.pose(squareX, DuoAutos.GARDEN_END_Y, 90.0), blue);
        double squareFlushX = (blue ? -1.0 : 1.0) * (FieldGeometry.HALF_IN - sideIn);
        double squareFlushY = (blue ? 1.0 : -1.0) * (FieldGeometry.HALF_IN - profile.originFromFrontIn());
        double squareHeadingRad = Math.toRadians(blue ? 90.0 : 270.0);

        Pose2d parkVia = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_PARK_VIA_X, DuoAutos.G_PARK_VIA_Y, DuoAutos.G_PARK_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_SIDE_PARK_X, DuoAutos.G_SIDE_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.G_PARK_HEADING_DEG), blue);

        return new SequentialAction(
                // 1. the preload into the rear cell from the start pose: the grid's flywheel speed (the formula aim and the position are unchanged) and the pusher at the LAUNCH-READY position
                new InstantAction(() -> m.setLaunchGrid(true, "Auton - Duo Garden Side, from Start (the preload: the grid's flywheel speed only)")),
                new InstantAction(m::requestLaunchReady),
                k.launch(drive, m, cell, blue, "preload", DuoAutos.G_PRELOAD_TRIM_DEG),

                // 2. the Blue Wall flower
                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("To the Blue Wall flower pre-position, turning to it"),
                k.wallPreLeg("wall flower pre-position", drive, wallPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: Blue Wall flower (held IN, whatever the count, for the rest of the auto)"),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the wall flower", drive, wallRam),
                k.timed("flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.G_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.backOutLeg("wall flower back-out (straight)", drive, wallBack, DuoAutos.WALL_BACKOUT_TOL_IN),
                new InstantAction(m::requestStowNow),   // clear of the flower: the ramp stows on the way

                // 3. the rear launch spot: aimed at once, the flower pollen not before G_SECOND_LAUNCH_NOT_BEFORE_SEC (as late as the garden, the last salvo and the park allow)
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position on this leg
                k.launchLeg("launch spot 1", drive, launchSpot),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.aimWaitLaunch(drive, m, cell, blue, "flower pollen", DuoAutos.G_REAR_TRIM_DEG, "second salvo (as late as the rest allows)", DuoAutos.G_SECOND_LAUNCH_NOT_BEFORE_SEC, true),

                // 4. the garden as the Solo auto gathers it, back to the same spot, the garden pollen not before G_LAST_LAUNCH_NOT_BEFORE_SEC (as late as the park allows)
                k.note("Intake ON: garden in ONE leg from the rear launch spot to the end of the pickup at the angled heading, the last " + DuoAutos.D_GARDEN_SLOW_ZONE_IN + " in at limited power"),
                m.setIntake(true),
                k.gardenLeg("garden (launch spot to the pickup end)", drive, gardenEnd),
                k.note("Square on the wall"),
                k.passLeg("square approach", drive, squarePre),
                k.timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY).withTotalLimit(DuoAutos.G_GARDEN_SQUARING_MAX_SEC)),
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position on this leg
                k.launchLeg("launch spot 2", drive, launchSpot),
                k.aimWaitLaunch(drive, m, cell, blue, "garden pollen", DuoAutos.G_REAR_TRIM_DEG, "last salvo (as late as the park allows)", DuoAutos.G_LAST_LAUNCH_NOT_BEFORE_SEC, true),

                // 5. the park through the via point (a straight run from the launch spot passes within an inch of the hive's foot bar)
                k.note("Park through the via point (clear of the hive's foot bar)"),
                k.crossLeg("park via point", drive, parkVia, true),
                k.parkLeg("park (full power)", drive, park),
                k.timed("ramp stow (final)", k.rampTo(m, false)),
                new InstantAction(m::stopFlywheel)
        );
    }

    /** Adds the Dashboard nudge to a blue-frame Blue Wall flower pose. */
    private static Pose2d nudge(Pose2d p) {
        return new Pose2d(p.position.x + DuoAutos.WALL_FLOWER_DX_IN, p.position.y + DuoAutos.WALL_FLOWER_DY_IN, p.heading.toDouble());
    }
}
