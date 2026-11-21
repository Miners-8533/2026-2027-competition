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
 * AUTON - BLASER'S GAMBIT (2026-11-21, the user's design; the app builds, NOT RUN on the robot): a triple tip from the AUDIENCE wall, the wall we do not normally start at. Every piece is one an enabled auto
 * already has (the Flower Side's start, the Solo's flower and garden, the Daryl's and the Triple Tip's rear launch spot and park); the values are in DuoAutos (B_* and the shared ones). Blue frame; red is
 * turned 180 deg about the center. The partner starts at the rear wall and tips the hive with its preload (tip 1).
 *
 *   1. Start with the back flush to the audience wall (heading 180). The flywheel spins at the 2-D launch grid's speed, the pusher goes to LAUNCH-READY, and the robot aims at the audience cell at once and
 *      rests aimed until B_FIRST_LAUNCH_NOT_BEFORE_SEC (7.0 s: the partner's preload salvo and the tip); the preload goes the moment it passes.
 *   2. The audience-wall flower (ram, one jostle cycle, back straight out), then the Solo launch spot, blue (56, 36), and its pollen into the audience cell with the grid's speed and aim: 8 pollen, tip 2.
 *   3. Around the east helper to the Blue Wall flower (this long leg at B_LONG_LEG_POWER), ram, jostle, back out; straight to the rear launch spot, blue (-55.6, 26.2) (the Triple Tip's), and its pollen into
 *      the OTHER cell, the rear one, which tip 2 turned up.
 *   4. The garden as the Solo auto gathers it (one leg to the end of the pickup, the last 12.5 in at 0.5 power, then the squaring for at most 0.8 s), back to the rear launch spot, and its pollen into the rear
 *      cell: 8 pollen, tip 3.
 *   5. The park is TRIED, through the via point (-20, 44): THE SALVOS COME BEFORE THE PARK in this auto (user, 2026-11-21), so nothing is skipped to save the park, and by the estimate the 30 s end before the
 *      robot gets there (the last salvo ends at about 28.9 s, the park would need about 31.9 s).
 */
@Autonomous(name = "Auton - Blaser's Gambit", group = "Auto")
public class AutonBlasersGambit extends AutonDuoBase {

    @Override
    protected String autoName() {
        return "Auton - Blaser's Gambit";
    }

    @Override
    protected Pose2d blueStart(ChassisProfile profile) {
        return DuoKit.pose(wallStartX(profile, 1.0), DuoAutos.B_START_Y_IN, 180.0);
    }

    @Override
    protected String placementText(boolean blue) {
        return "Back against the AUDIENCE wall at y " + (blue ? "+" : "-") + DuoAutos.B_START_Y_IN + ", facing the field.";
    }

    @Override
    protected Action buildAuto(Context c) {
        MecanumDrive drive = c.drive;
        Mechanisms m = c.mechanisms;
        DuoKit k = c.kit;
        boolean blue = c.blue;
        ChassisProfile profile = c.profile;
        k.fastLegs(DuoAutos.P_LEG_MAX_POWER, DuoAutos.P_CROSS_MAX_POWER, DuoAutos.P_PASS_SETTLE_SEC);

        HiveTarget audCell = DuoKit.cellForAlliance(HiveTarget.BLUE_AUDIENCE, blue);   // blue: audience, red: rear (the cell on our start side: the preload and the first flower)
        HiveTarget rearCell = DuoKit.cellForAlliance(HiveTarget.BLUE_REAR, blue);      // blue: rear, red: audience (the other cell: the second flower and the garden)
        m.setOpenCell(audCell);

        // the two launch spots, each with the 2-D grid's aim (the bearing plus the grid's residual there): the Solo's for the audience cell, the Triple Tip's for the rear cell
        Pose2d audSpot = gridSpot(DuoAutos.B_AUD_LAUNCH_X, DuoAutos.B_AUD_LAUNCH_Y, audCell, DuoAutos.B_AUD_TRIM_DEG, blue);
        Pose2d rearSpot = gridSpot(DuoAutos.B_REAR_LAUNCH_X, DuoAutos.B_REAR_LAUNCH_Y, rearCell, DuoAutos.B_REAR_TRIM_DEG, blue);

        // the audience-wall flower: the solo auto's poses
        Pose2d flowerPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerRam = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerBack = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);

        // the Blue Wall flower: the audience-wall flower's poses turned 90 deg counterclockwise, plus the Dashboard nudge
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

        Pose2d eastHelper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_EAST_HELPER_X, DuoAutos.P_EAST_HELPER_Y, DuoAutos.P_HELPER_HEADING_DEG), blue);
        Pose2d parkVia = DuoKit.forAlliance(DuoKit.pose(DuoAutos.B_PARK_VIA_X, DuoAutos.B_PARK_VIA_Y, DuoAutos.B_PARK_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.B_PARK_X, DuoAutos.B_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.B_PARK_HEADING_DEG), blue);

        return new SequentialAction(
                // 1. the preload into the audience cell from the start pose: the grid's flywheel speed, the pusher at launch-ready, aimed at once (the formula aim), the salvo when the wait is over
                new InstantAction(() -> m.setLaunchGrid(true, "Auton - Blaser's Gambit, from Start (the preload: the grid's flywheel speed only)")),
                new InstantAction(m::requestLaunchReady),
                k.aimWaitLaunch(drive, m, audCell, blue, "preload", DuoAutos.B_PRELOAD_TRIM_DEG, "first launch (partner's tip)", DuoAutos.B_FIRST_LAUNCH_NOT_BEFORE_SEC, false),

                // 2. the audience-wall flower, then its pollen into the audience cell from the Solo launch spot (tip 2)
                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("To the audience flower pre-position, turning to the flower"),
                k.leg("flower pre-position", drive, flowerPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: audience flower (held IN, whatever the count, for the rest of the auto)"),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the flower", drive, flowerRam),
                k.timed("flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.B_AUD_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.backOutLeg("flower back-out (straight)", drive, flowerBack),
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position while the robot turns to the launch spot
                k.stowWhenTurned(k.launchLeg("audience launch spot", drive, audSpot), drive, m, flowerPre.heading.toDouble()),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.launchOnGrid(drive, m, audCell, blue, "audience flower pollen", DuoAutos.B_AUD_TRIM_DEG),

                // 3. the Blue Wall flower, then its pollen into the OTHER cell (the rear one, which tip 2 turned up) from the Triple Tip's rear launch spot
                new InstantAction(() -> m.setOpenCell(rearCell)),   // the flywheel follows the rear cell's distance from here
                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("Around the outside of the hive to the Blue Wall flower pre-position, turning to it"),
                k.crossLeg("east helper (outside the support rails)", drive, eastHelper, true),
                k.wallPreLeg("wall flower pre-position", drive, wallPre, DuoAutos.B_LONG_LEG_POWER),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the wall flower", drive, wallRam),
                k.timed("wall flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.B_WALL_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.backOutLeg("wall flower back-out (straight)", drive, wallBack, DuoAutos.WALL_BACKOUT_TOL_IN),
                new InstantAction(m::requestStowNow),   // clear of the flower: the ramp stows on the way
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position on this leg
                k.launchLeg("rear launch spot 1", drive, rearSpot, DuoAutos.B_LONG_LEG_POWER),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.launchOnGrid(drive, m, rearCell, blue, "wall flower pollen", DuoAutos.B_REAR_TRIM_DEG),

                // 4. the garden as the Solo auto gathers it, then its pollen into the rear cell from the same spot (tip 3)
                k.note("Intake ON: garden in ONE leg from the rear launch spot to the end of the pickup at the angled heading, the last " + DuoAutos.D_GARDEN_SLOW_ZONE_IN + " in at limited power"),
                m.setIntake(true),
                k.gardenLeg("garden (launch spot to the pickup end)", drive, gardenEnd),
                k.note("Square on the wall"),
                k.passLeg("square approach", drive, squarePre),
                k.timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY).withTotalLimit(DuoAutos.B_GARDEN_SQUARING_MAX_SEC)),
                new InstantAction(m::requestLaunchReady),   // the arm goes to the launch-ready position on this leg
                k.launchLeg("rear launch spot 2", drive, rearSpot, DuoAutos.B_LONG_LEG_POWER),
                k.launchOnGrid(drive, m, rearCell, blue, "garden pollen", DuoAutos.B_REAR_TRIM_DEG),

                // 5. the park is TRIED (the salvos come first in this auto: nothing above is skipped to save it), through the via point clear of the hive's foot bar
                k.note("Park attempt through the via point (the salvos came first; the 30 s may end on the way)"),
                k.crossLeg("park via point", drive, parkVia, true),
                k.parkLeg("park (full power)", drive, park),
                k.timed("ramp stow (final)", k.rampTo(m, false)),
                new InstantAction(m::stopFlywheel)
        );
    }

    /** The blue-frame launch spot for this alliance with the 2-D grid's aim at the cell (the bearing plus the grid's residual there; the formula aim where the grid has no values) plus the trim. */
    private static Pose2d gridSpot(double blueX, double blueY, HiveTarget cell, double trimDeg, boolean blue) {
        Pose2d xy = DuoKit.forAlliance(DuoKit.pose(blueX, blueY, 0.0), blue);
        LaunchGridMap.Result grid = LaunchGridMap.lookup(cell, xy.position.x, xy.position.y);
        double aim = grid.hasValues()
                ? Math.atan2(cell.y - xy.position.y, cell.x - xy.position.x) + Math.toRadians(grid.residualDeg)
                : TargetLock.aimHeadingRad(xy.position.x, xy.position.y, cell, 0.0);
        return new Pose2d(xy.position.x, xy.position.y, aim + Math.toRadians(trimDeg));
    }

    /** Adds the Dashboard nudge to a blue-frame Blue Wall flower pose. */
    private static Pose2d nudge(Pose2d p) {
        return new Pose2d(p.position.x + DuoAutos.WALL_FLOWER_DX_IN, p.position.y + DuoAutos.WALL_FLOWER_DY_IN, p.heading.toDouble());
    }
}
