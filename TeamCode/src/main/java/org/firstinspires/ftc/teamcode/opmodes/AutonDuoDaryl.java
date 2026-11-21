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
 * Duo DARYL (2026-11-06, the user's design): the rear-wall start, garden first. Blue frame; red is turned 180 deg about the center.
 *
 *   1. Start at the rear wall (back flush, y 16, heading 0); the flywheel spins from Start and the pusher moves to the lock position meanwhile (Mechanisms.preparePusherForLaunch: the preload salvo turns
 *      310 deg instead of 400). Launch the preload into the REAR cell.
 *   2. (2026-11-07: the solo auto's garden instead, angled pre-position, slow pickup, squaring approach, then the squaring; the one-leg version ended too far from the wall, D_GARDEN_SLOW_* and DuoKit.gardenLeg are unused for now.) Before: the garden in ONE leg from the start straight to the wall squaring pose, the last D_GARDEN_SLOW_ZONE_IN (12.5 in: four pollen of about 2.8 in end to end plus margin) limited to D_GARDEN_SLOW_POWER
 *      (0.5), then square on the wall (side wall, then front wall).
 *   3. Around the outside of the hive via the east helper to the audience launch spot (50, 26.3); launch into the AUDIENCE cell (the far cell: the aim takes the far-cell extra).
 *   4. The Blue Wall flower (the audience flower's poses turned 90 deg): ram, one-cycle jostle, back out; the ramp stows.
 *   5. (2026-11-21, the user) The Blue Wall flower's pollen go into the ORIGINAL cell, the one the preload went into (blue the rear cell, red the audience cell): from the back-out straight to the rear launch
 *      spot, blue (-55.6, 26.2) (the Triple Tip auto's, 35 in away, on the same side of the hive), the ramp stowing and the pusher going to launch-ready on the way, then the 2-D grid's speed and aim.
 *   6. Park in the hive-side corner of the loading zone, (22, 55.1), through the via point (-20, 44) (clear of the hive's foot bar) at heading 30 deg, as the Triple Tip auto does; ramp stowed.
 * Before 2026-11-21 steps 5 to 8 were: the partner's window (not back at the audience spot before 18 s), a second salvo into the AUDIENCE cell from (56, 36), the audience-wall flower (skipped when late, its
 * pollen kept for the TeleOp) and the park at heading 90. Those are gone: no wait, no second trip around the hive, no audience flower. Built 2026-11-21, not run on the robot.
 * Faster legs than the other duo autos (the solo auto's, DuoAutos.P_*); always launches every salvo (the autos never skip on an empty count). The estimate (design/duo-autos.md): about 22 s.
 */
@Autonomous(name = "Auton - Duo Daryl", group = "Auto")
public class AutonDuoDaryl extends AutonDuoBase {

    @Override
    protected String autoName() {
        return "Auton - Duo Daryl";
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
        k.fastLegs(DuoAutos.P_LEG_MAX_POWER, DuoAutos.P_CROSS_MAX_POWER, DuoAutos.P_PASS_SETTLE_SEC);

        HiveTarget rearCell = DuoKit.cellForAlliance(HiveTarget.BLUE_REAR, blue);        // blue: rear, red: audience
        HiveTarget audCell = DuoKit.cellForAlliance(HiveTarget.BLUE_AUDIENCE, blue);     // blue: audience, red: rear
        m.setOpenCell(rearCell);

        // the audience launch spot: the heading is the computed aim at the audience cell with the far-cell extra (the solo auto's spot, on the audience flower's line)
        // 2026-11-20 (user): the Solo auto's launch spot, blue (56, 36) / red (-56, -36), one spot for both audience-cell salvos, with the 2-D grid's aim (the bearing plus the grid's residual there) and speed
        Pose2d audXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.D_LAUNCH_X, DuoAutos.D_LAUNCH_Y, 0.0), blue);
        LaunchGridMap.Result audGrid = LaunchGridMap.lookup(audCell, audXY.position.x, audXY.position.y);
        double audAim = audGrid.hasValues()
                ? Math.atan2(audCell.y - audXY.position.y, audCell.x - audXY.position.x) + Math.toRadians(audGrid.residualDeg)
                : TargetLock.aimHeadingRad(audXY.position.x, audXY.position.y, audCell, 0.0);
        Pose2d audSpot = new Pose2d(audXY.position.x, audXY.position.y, audAim + Math.toRadians(DuoAutos.D_GRID_TRIM_DEG));

        // the garden as in the solo auto (2026-11-07, user: the one-leg garden ended too far from the wall): the angled pre-position, the slow pickup, the squaring approach, then the squaring
        Pose2d gardenPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.GARDEN_PRE_X, DuoAutos.GARDEN_PRE_Y, DuoAutos.GARDEN_HEADING_DEG), blue);
        Pose2d gardenEnd = DuoKit.forAlliance(DuoKit.pose(DuoAutos.GARDEN_END_X, DuoAutos.GARDEN_END_Y, DuoAutos.GARDEN_HEADING_DEG), blue);
        double sideIn = profile.originFromSideIn();
        double squareX = -(FieldGeometry.HALF_IN - sideIn - DuoAutos.SQUARE_GAP_IN);
        Pose2d squarePre = DuoKit.forAlliance(DuoKit.pose(squareX, DuoAutos.GARDEN_END_Y, 90.0), blue);
        double squareFlushX = (blue ? -1.0 : 1.0) * (FieldGeometry.HALF_IN - sideIn);
        double squareFlushY = (blue ? 1.0 : -1.0) * (FieldGeometry.HALF_IN - profile.originFromFrontIn());
        double squareHeadingRad = Math.toRadians(blue ? 90.0 : 270.0);

        // the Blue Wall flower: the audience-wall flower's poses turned 90 deg counterclockwise, plus the Dashboard nudge
        Pose2d wallPre = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);
        Pose2d wallRam = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);
        Pose2d wallBack = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);

        // the rear launch spot (2026-11-21, user: the Blue Wall flower's pollen go into the ORIGINAL cell): the Triple Tip auto's spot, blue (-55.6, 26.2) / red (55.6, -26.2), with the 2-D grid's aim and speed
        Pose2d rearXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.D_REAR_LAUNCH_X, DuoAutos.D_REAR_LAUNCH_Y, 0.0), blue);
        LaunchGridMap.Result rearGrid = LaunchGridMap.lookup(rearCell, rearXY.position.x, rearXY.position.y);
        double rearAim = rearGrid.hasValues()
                ? Math.atan2(rearCell.y - rearXY.position.y, rearCell.x - rearXY.position.x) + Math.toRadians(rearGrid.residualDeg)
                : TargetLock.aimHeadingRad(rearXY.position.x, rearXY.position.y, rearCell, 0.0);
        Pose2d rearSpot = new Pose2d(rearXY.position.x, rearXY.position.y, rearAim + Math.toRadians(DuoAutos.D_REAR_TRIM_DEG));

        Pose2d eastHelper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_EAST_HELPER_X, DuoAutos.P_EAST_HELPER_Y, DuoAutos.P_HELPER_HEADING_DEG), blue);
        Pose2d parkVia = DuoKit.forAlliance(DuoKit.pose(DuoAutos.D_PARK_VIA_X, DuoAutos.D_PARK_VIA_Y, DuoAutos.D_PARK_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.D_PARK_X, DuoAutos.D_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.D_PARK_HEADING_DEG), blue);

        return new SequentialAction(
                // 1. the preload into the rear cell, the pusher going to the LAUNCH-READY position (2026-11-20, user; it was the 90 deg lock) and the flywheel speed from the 2-D grid (the formula aim and the position are unchanged)
                new InstantAction(() -> m.setLaunchGrid(true, "Auton - Duo Daryl, from Start (the preload: the grid's flywheel speed only)")),
                new InstantAction(m::requestLaunchReady),
                k.launch(drive, m, rearCell, blue, "preload", DuoAutos.D_PRELOAD_TRIM_DEG),

                // 2. the garden as in the solo auto: pre-position, slow pickup, squaring approach, squaring
                k.note("Garden pre-position"),
                k.passLeg("garden pre-position", drive, gardenPre),
                k.note("Intake ON: garden, slow"),
                m.setIntake(true),
                k.slowLeg("garden pickup (slow)", drive, gardenEnd),
                k.note("Square on the wall"),
                k.passLeg("square approach", drive, squarePre),
                k.timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY)),

                // 3. around the hive to the audience launch spot, the audience cell
                k.note("Around the outside of the hive support rails, intake still ON so every element gets in"),
                k.crossLeg("east helper (outside the support rails)", drive, eastHelper, true),
                new InstantAction(() -> m.setOpenCell(audCell)),
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position on this last leg
                k.launchLeg("audience launch spot", drive, audSpot),
                k.launchOnGrid(drive, m, audCell, blue, "garden pollen", DuoAutos.D_GRID_TRIM_DEG),

                // 4. the Blue Wall flower
                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("Back around the outside to the Blue Wall flower pre-position, turning to it"),
                k.crossLeg("east helper (outside the support rails)", drive, eastHelper, true),
                k.wallPreLeg("wall flower pre-position", drive, wallPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: Blue Wall flower (held IN, whatever the count, to the end of the auto)"),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the wall flower", drive, wallRam),
                k.timed("wall flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.D_WALL_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.backOutLeg("wall flower back-out (straight)", drive, wallBack, DuoAutos.WALL_BACKOUT_TOL_IN),
                new InstantAction(m::requestStowNow),   // clear of the flower: the ramp stows on the way

                // 5. the Blue Wall flower's pollen into the ORIGINAL cell (2026-11-21, user; it was a second salvo into the audience cell after the partner's window, then the audience flower)
                k.note("To the rear launch spot: the Blue Wall flower's pollen go into the original cell (the preload's)"),
                new InstantAction(() -> m.setOpenCell(rearCell)),   // the flywheel follows the original cell's distance from here
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position on this leg
                k.launchLeg("rear launch spot", drive, rearSpot),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.launchOnGrid(drive, m, rearCell, blue, "wall flower pollen (original cell)", DuoAutos.D_REAR_TRIM_DEG),

                // 6. the park in the hive-side corner of the loading zone, through the via point (a straight run from the launch spot passes within an inch of the hive's foot bar)
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
