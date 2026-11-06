package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
import org.firstinspires.ftc.teamcode.drive.FieldGeometry;
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
 * Duo PRIMARY (2026-11-06, the user's design): the rear-wall start of {@link AutonDuoGardenSide} with a longer program, for the alliance whose partner does the lighter job. Blue frame; red is
 * turned 180 deg about the center. The hive cells in a tip sequence: the upward cell starts with 3 nectar (3 pollen tip it), the cell that then faces up is empty and needs 8.
 *
 *   1. Start at the rear wall (back flush, y 16, heading 0); the flywheel spins from Start. Launch the preload into the REAR cell (tip 1).
 *   2. The Blue Wall flower (the audience-wall flower turned 90 deg): ram, one-cycle jostle (ends by P_WALL_JOSTLE_DEADLINE_SEC), back out; the ramp stows.
 *   3. Around the outside of the hive via the east helper to the mid-field spot on the audience flower's line (P_AUD_LAUNCH, the solo auto's launch spot), launch into the AUDIENCE cell (the far cell:
 *      the aim takes the far-cell extra).
 *   4. The audience flower from there (the solo auto's flower poses), back out. [skipped when later than P_AUD_FLOWER_LATEST_START_SEC]
 *   5. Back across the north side (east helper, west helper) to the mid-field spot on the rear side (P_REAR_LAUNCH, the mirror of the audience one) and launch into the REAR cell.
 *   6. The garden as in the solo auto (angled approach, slow pickup, square on the wall), back to the same spot, launch into the REAR cell. [skipped when later than P_GARDEN_LATEST_START_SEC]
 *   7. Park in the hive-side corner of the loading zone (the garden side's park, (26, 60)) through the west helper, ramp stowed.
 * The garden (step 6) is skipped when it would start after P_GARDEN_LATEST_START_SEC (18 s) so the park keeps its time: the 2026-11-06 01:57 run, with the garden, reached it at 21.8 s and the park was cut
 * off at 30 s. The user will optimize the garden gather and the paths later (design/duo-autos.md). Faster legs than the other duo autos: DuoAutos.P_LEG_MAX_POWER and friends.
 */
@Autonomous(name = "Auton - Duo Primary", group = "Auto")
public class AutonDuoPrimary extends AutonDuoBase {

    @Override
    protected String autoName() {
        return "Auton - Duo Primary";
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
        double audExtra = TargetLock.farCellExtraLeftIn(audCell, blue);                   // the far cell from the rear-wall start
        m.setOpenCell(rearCell);

        // the two launch spots: the headings are the computed aims at the cell of each (the audience one with the far-cell extra), AimAtCell takes out what is left
        Pose2d audXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_AUD_LAUNCH_X, DuoAutos.P_AUD_LAUNCH_Y, 0.0), blue);
        Pose2d audSpot = new Pose2d(audXY.position.x, audXY.position.y, TargetLock.aimHeadingRad(audXY.position.x, audXY.position.y, audCell, audExtra));
        Pose2d rearXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_REAR_LAUNCH_X, DuoAutos.P_REAR_LAUNCH_Y, 0.0), blue);
        Pose2d rearSpot = new Pose2d(rearXY.position.x, rearXY.position.y, TargetLock.aimHeadingRad(rearXY.position.x, rearXY.position.y, rearCell, 0.0));

        // the Blue Wall flower: the audience-wall flower's poses turned 90 deg counterclockwise (heading 90, facing the Blue Wall), plus the Dashboard nudge
        Pose2d wallPre = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);
        Pose2d wallRam = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);
        Pose2d wallBack = DuoKit.forAlliance(nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG))), blue);

        // the audience flower: the solo auto's poses
        Pose2d flowerPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerRam = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerBack = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);

        // the helpers north of the hive, the garden as in the solo auto, and the park
        Pose2d eastHelper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_EAST_HELPER_X, DuoAutos.P_EAST_HELPER_Y, DuoAutos.P_HELPER_HEADING_DEG), blue);
        Pose2d westHelper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.HELPER_X, DuoAutos.HELPER_Y, DuoAutos.P_HELPER_HEADING_DEG), blue);
        Pose2d gardenPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.GARDEN_PRE_X, DuoAutos.GARDEN_PRE_Y, DuoAutos.GARDEN_HEADING_DEG), blue);
        Pose2d gardenEnd = DuoKit.forAlliance(DuoKit.pose(DuoAutos.GARDEN_END_X, DuoAutos.GARDEN_END_Y, DuoAutos.GARDEN_HEADING_DEG), blue);
        double sideIn = profile.originFromSideIn();
        double squareX = -(FieldGeometry.HALF_IN - sideIn - DuoAutos.SQUARE_GAP_IN);
        Pose2d squarePre = DuoKit.forAlliance(DuoKit.pose(squareX, DuoAutos.GARDEN_END_Y, 90.0), blue);
        double squareFlushX = (blue ? -1.0 : 1.0) * (FieldGeometry.HALF_IN - sideIn);
        double squareFlushY = (blue ? 1.0 : -1.0) * (FieldGeometry.HALF_IN - profile.originFromFrontIn());
        double squareHeadingRad = Math.toRadians(blue ? 90.0 : 270.0);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_PARK_X, DuoAutos.G_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.PARK_HEADING_DEG), blue);

        return new SequentialAction(
                // 1. the preload into the rear cell
                k.launch(drive, m, rearCell, blue, "preload", DuoAutos.P_PRELOAD_TRIM_DEG),

                // 2. the Blue Wall flower
                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("To the Blue Wall flower pre-position, turning to it"),
                k.wallPreLeg("wall flower pre-position", drive, wallPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: Blue Wall flower (held IN, whatever the count, until its salvo is done)"),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the wall flower", drive, wallRam),
                k.timed("wall flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.P_WALL_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.backOutLeg("wall flower back-out (straight)", drive, wallBack, DuoAutos.WALL_BACKOUT_TOL_IN),
                new InstantAction(m::requestStowNow),   // clear of the flower: the ramp stows on the way

                // 3. the audience cell from the mid-field spot on the audience flower's line
                k.note("Around the outside of the hive to the audience launch spot (intake still ON)"),
                k.crossLeg("east helper (outside the support rails)", drive, eastHelper, true),
                new InstantAction(() -> m.setOpenCell(audCell)),
                k.launchLeg("audience launch spot", drive, audSpot),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.launch(drive, m, audCell, blue, "wall flower pollen", DuoAutos.P_WALL_TRIM_DEG, Double.NaN),
                k.note("Intake OFF (the wall flower salvo is done)"),
                m.forceIntakeIn(false),
                m.setIntake(false),

                // 4. the audience flower
                k.unlessLate("audience flower", DuoAutos.P_AUD_FLOWER_LATEST_START_SEC, new SequentialAction(
                        new InstantAction(m::requestDeploy),   // deploys on the way
                        k.note("To the audience flower pre-position, turning to it"),
                        k.leg("audience flower pre-position", drive, flowerPre),
                        k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                        k.note("Intake ON: audience flower (held IN, whatever the count, until its salvo is done)"),
                        m.setIntake(true),
                        m.forceIntakeIn(true),
                        k.ramLeg("ram the audience flower", drive, flowerRam),
                        k.timed("audience flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.P_AUD_JOSTLE_DEADLINE_SEC),
                                () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                        k.backOutLeg("audience flower back-out (straight)", drive, flowerBack),
                        new InstantAction(m::requestStowNow))),

                // 5. back across the north side to the rear-side mid-field spot, the rear cell
                k.note("Back across the north side of the hive to the rear launch spot (intake still ON)"),
                k.crossLeg("east helper (outside the support rails)", drive, eastHelper, true),
                k.crossLeg("west helper (outside the support rails)", drive, westHelper, true),
                new InstantAction(() -> m.setOpenCell(rearCell)),
                k.launchLeg("rear launch spot", drive, rearSpot),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.launch(drive, m, rearCell, blue, "audience flower pollen", DuoAutos.P_AUD_FLOWER_TRIM_DEG),
                k.note("Intake OFF (the audience flower salvo is done)"),
                m.forceIntakeIn(false),
                m.setIntake(false),

                // 6. the garden, back to the rear spot, the rear cell
                k.unlessLate("garden", DuoAutos.P_GARDEN_LATEST_START_SEC, new SequentialAction(
                        k.note("Garden pre-position"),
                        k.passLeg("garden pre-position", drive, gardenPre),
                        k.note("Intake ON: garden, slow"),
                        m.setIntake(true),
                        k.slowLeg("garden pickup (slow)", drive, gardenEnd),
                        k.note("Square on the wall"),
                        k.passLeg("square approach", drive, squarePre),
                        k.timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY)),
                        k.note("Back to the rear launch spot (intake still ON so every element gets in)"),
                        k.launchLeg("rear launch spot 2", drive, rearSpot),
                        m.setIntake(false),
                        k.launch(drive, m, rearCell, blue, "garden pollen", DuoAutos.P_GARDEN_TRIM_DEG))),

                // 7. the park in the hive-side corner of the loading zone
                k.note("Park"),
                k.passLeg("park helper (clear of the hive supports)", drive, westHelper),
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
