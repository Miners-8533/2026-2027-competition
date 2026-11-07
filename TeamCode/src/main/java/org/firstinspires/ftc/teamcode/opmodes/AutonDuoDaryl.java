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
 * Duo DARYL (2026-11-06, the user's design): the rear-wall start, garden first. Blue frame; red is turned 180 deg about the center.
 *
 *   1. Start at the rear wall (back flush, y 16, heading 0); the flywheel spins from Start and the pusher moves to the lock position meanwhile (Mechanisms.preparePusherForLaunch: the preload salvo turns
 *      310 deg instead of 400). Launch the preload into the REAR cell.
 *   2. (2026-11-07: the solo auto's garden instead, angled pre-position, slow pickup, squaring approach, then the squaring; the one-leg version ended too far from the wall, D_GARDEN_SLOW_* and DuoKit.gardenLeg are unused for now.) Before: the garden in ONE leg from the start straight to the wall squaring pose, the last D_GARDEN_SLOW_ZONE_IN (12.5 in: four pollen of about 2.8 in end to end plus margin) limited to D_GARDEN_SLOW_POWER
 *      (0.5), then square on the wall (side wall, then front wall).
 *   3. Around the outside of the hive via the east helper to the audience launch spot (50, 26.3); launch into the AUDIENCE cell (the far cell: the aim takes the far-cell extra).
 *   4. The Blue Wall flower (the audience flower's poses turned 90 deg): ram, one-cycle jostle, back out.
 *   5. The partner's window (an ABSOLUTE time, not a pause): the robot does not leave the flower before D_SECOND_SHOT_NOT_BEFORE_SEC (18 s) minus the drive back, and does not shoot before it.
 *   6. Back to the audience launch spot, launch into the AUDIENCE cell again (the intake stays IN through this salvo for the stragglers).
 *   7. The audience flower (the solo auto's poses): ram, jostle, back out; skipped when it would start later than D_AUD_FLOWER_LATEST_START_SEC so the park keeps its time. Its pollen stay in the robot for the TeleOp.
 *   8. Park in the hive-side corner of the loading zone (26, 53.1), ramp stowed.
 * Faster legs than the other duo autos (the solo auto's, DuoAutos.P_*); always launches every salvo (the autos never skip on an empty count). The estimate (design/duo-autos.md): about 29.7 s with every step.
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
        Pose2d audXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_AUD_LAUNCH_X, DuoAutos.P_AUD_LAUNCH_Y, 0.0), blue);
        double audExtra = TargetLock.farCellExtraLeftIn(audCell, blue);
        Pose2d audSpot = new Pose2d(audXY.position.x, audXY.position.y, TargetLock.aimHeadingRad(audXY.position.x, audXY.position.y, audCell, audExtra));

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

        // the audience flower: the solo auto's poses
        Pose2d flowerPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerRam = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerBack = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);

        Pose2d eastHelper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.P_EAST_HELPER_X, DuoAutos.P_EAST_HELPER_Y, DuoAutos.P_HELPER_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_PARK_X, DuoAutos.G_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.PARK_HEADING_DEG), blue);

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
                m.setIntake(false),
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

                // 5. the partner's window: not back at the spot before D_SECOND_SHOT_NOT_BEFORE_SEC (an absolute time): leave the flower no earlier than that minus the drive back
                k.waitUntil("leave the flower (partner parking)", DuoAutos.D_SECOND_SHOT_NOT_BEFORE_SEC - DuoAutos.D_RETURN_TRAVEL_SEC),

                // 6. back to the audience launch spot, the audience cell again
                k.note("Around the outside of the hive to the audience launch spot (intake still ON)"),
                k.crossLeg("east helper (outside the support rails)", drive, eastHelper, true),
                k.launchLeg("audience launch spot", drive, audSpot),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.waitUntil("second shot (partner parked)", DuoAutos.D_SECOND_SHOT_NOT_BEFORE_SEC),
                k.launch(drive, m, audCell, blue, "wall flower pollen", DuoAutos.D_WALL_TRIM_DEG, Double.NaN),
                k.note("Intake OFF (the wall flower salvo is done)"),
                m.forceIntakeIn(false),
                m.setIntake(false),

                // 7. the audience flower, skipped when late; its pollen stay for the TeleOp
                k.unlessLate("audience flower", DuoAutos.D_AUD_FLOWER_LATEST_START_SEC, new SequentialAction(
                        new InstantAction(m::requestDeploy),   // deploys on the way
                        k.note("To the audience flower pre-position, turning to it"),
                        k.leg("audience flower pre-position", drive, flowerPre),
                        k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                        k.note("Intake ON: audience flower (held IN to the park; the pollen stay for the TeleOp)"),
                        m.setIntake(true),
                        m.forceIntakeIn(true),
                        k.ramLeg("ram the audience flower", drive, flowerRam),
                        k.timed("audience flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.D_AUD_JOSTLE_DEADLINE_SEC),
                                () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                        k.backOutLeg("audience flower back-out (straight)", drive, flowerBack),
                        new InstantAction(m::requestStowNow))),

                // 8. the park in the hive-side corner of the loading zone
                k.note("Park"),
                k.parkLeg("park (full power)", drive, park),
                k.timed("ramp stow (final)", k.rampTo(m, false)),
                m.forceIntakeIn(false),
                m.setIntake(false),
                new InstantAction(m::stopFlywheel)
        );
    }

    /** Adds the Dashboard nudge to a blue-frame Blue Wall flower pose. */
    private static Pose2d nudge(Pose2d p) {
        return new Pose2d(p.position.x + DuoAutos.WALL_FLOWER_DX_IN, p.position.y + DuoAutos.WALL_FLOWER_DY_IN, p.heading.toDouble());
    }
}
