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
import org.firstinspires.ftc.teamcode.motion.SleepAction;
import org.firstinspires.ftc.teamcode.motion.SquareOnWall;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;

/**
 * CYBUGS (2026-11-07, the user's design, built from the Duo Daryl): the rear-wall start, garden first, no audience flower. Blue frame; red is turned 180 deg about the center.
 *
 *   1. Start at the rear wall (back flush, y 16, heading 0); the flywheel spins from Start and the pusher moves to the lock position meanwhile. Launch the preload into the REAR cell.
 *   2. The garden as in the solo auto: angled pre-position, slow pickup, squaring approach, then the squaring on the side and front walls.
 *   3. Around the outside of the hive via the east helper to the audience launch spot (50, 26.3); launch the garden pollen into the AUDIENCE cell (the far cell, trim DuoAutos.D_GARDEN_TRIM_DEG).
 *   4. Back across to the Blue Wall flower (the audience flower's poses turned 90 deg): ram, one-cycle jostle, back out. No waits for the partner.
 *   5. To the REAR launch spot at the start wall (-61.5, 15.25, the Garden Side's spot), wait DuoAutos.C_LAST_SALVO_WAIT_SEC (1 s, the flywheel keeps spinning), then launch the flower pollen into the REAR cell (the cell near the start; trim DuoAutos.C_REAR_TRIM_DEG).
 *   6. The intake runs OUT from here to the end (Mechanisms.setManualOuttake, the TeleOp's left-trigger outtake), so it does not pick up game elements on the way to the park (user, 2026-11-07: nectar kept getting picked up). The second garden pass was dropped.
 *   7. Park in the hive-side corner of the loading zone (26, 53.1) through the helper point (-36, 40), ramp stowed.
 * The intake runs IN from the start (AutonDuoBase) until the last salvo. Estimated at about 24.7 s (design/duo-autos.md).
 */
@Autonomous(name = "Auton - Cybugs", group = "Auto")
public class AutonCybugs extends AutonDuoBase {

    @Override
    protected String autoName() {
        return "Auton - Cybugs";
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
        Pose2d audXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_AUD_LAUNCH_X, DuoAutos.P_AUD_LAUNCH_Y, 0.0), blue);
        double audExtra = TargetLock.farCellExtraLeftIn(audCell, blue);
        // the spot headings include the salvo's aim trim (2026-11-07): the leg arrives pointed at the trimmed aim and the aim step only fine-tunes
        double audAim = TargetLock.aimHeadingRad(audXY.position.x, audXY.position.y, audCell, audExtra);
        Pose2d audSpot = new Pose2d(audXY.position.x, audXY.position.y, audAim + Math.toRadians(DuoAutos.D_GARDEN_TRIM_DEG));

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

        Pose2d eastHelper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_EAST_HELPER_X, DuoAutos.P_EAST_HELPER_Y, DuoAutos.P_HELPER_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_PARK_X, DuoAutos.G_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.PARK_HEADING_DEG), blue);
        Pose2d helper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.HELPER_X, DuoAutos.HELPER_Y, DuoAutos.PARK_HEADING_DEG), blue);

        // the rear launch spot at the start wall (the Garden Side's spot; the preload fires from beside it): the heading is the aim at the rear cell plus the salvo's trim, so the leg arrives pointed at it
        Pose2d rearXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_LAUNCH_X, DuoAutos.G_LAUNCH_Y, 0.0), blue);
        Pose2d rearSpot = new Pose2d(rearXY.position.x, rearXY.position.y,
                TargetLock.aimHeadingRad(rearXY.position.x, rearXY.position.y, rearCell, 0.0) + Math.toRadians(DuoAutos.C_REAR_TRIM_DEG));

        return new SequentialAction(
                // 1. the preload into the rear cell, the pusher already at the lock position
                new InstantAction(m::preparePusherForLaunch),
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
                k.launchLeg("audience launch spot", drive, audSpot),
                k.launch(drive, m, audCell, blue, "garden pollen", DuoAutos.D_GARDEN_TRIM_DEG, Double.NaN),

                // 4. the Blue Wall flower
                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("Back around the outside to the Blue Wall flower pre-position, turning to it"),
                k.crossLeg("east helper (outside the support rails)", drive, eastHelper, true),
                k.wallPreLeg("wall flower pre-position", drive, wallPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: Blue Wall flower (held IN, whatever the count, until its salvo is done)"),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the wall flower", drive, wallRam),
                k.timed("wall flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.D_WALL_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.backOutLeg("wall flower back-out (straight)", drive, wallBack, DuoAutos.WALL_BACKOUT_TOL_IN),
                new InstantAction(m::requestStowNow),   // clear of the flower: the ramp stows on the way

                // 5. to the rear launch spot at the start wall and launch into the REAR cell (the cell near the start)
                new InstantAction(() -> m.setOpenCell(rearCell)),   // the flywheel follows the new distance on the way
                k.note("To the rear launch spot at the start wall (intake still ON)"),
                k.launchLeg("rear launch spot", drive, rearSpot),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.timed("wait before the last salvo", new SleepAction(DuoAutos.C_LAST_SALVO_WAIT_SEC)),   // the flywheel keeps spinning (user, 2026-11-07)
                k.launch(drive, m, rearCell, blue, "flower pollen", DuoAutos.C_REAR_TRIM_DEG),

                // 6. the last salvo is done: the intake runs OUT for the rest of the auto so it does not pick up game elements on the way to the park (user, 2026-11-07: nectar kept getting picked up)
                k.note("Intake OUT: the last salvo is done, nothing more is picked up"),
                m.forceIntakeIn(false),
                new InstantAction(() -> m.setManualOuttake(true)),

                // 7. the park in the hive-side corner of the loading zone through the helper point (the Garden Side's park route)
                k.note("Park"),
                k.passLeg("park helper (clear of the Blue Wall flower and the hive supports)", drive, helper),
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
