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
 * Duo auto, the GARDEN side (the rear wall): the partner runs {@link AutonDuoFlowerSide}. Blue frame; red is turned 180 deg about the center.
 *
 *   1. Start like the solo auto (back to the rear wall, y 16, heading 0); the flywheel spins from Start. Launch the preload into the rear cell, which starts with 3 nectar: the 4 pollen tip the hive.
 *   2. Gather the Blue Wall flower (the audience-wall flower turned 90 deg: facing +Y, ram, jostle until full or G_JOSTLE_DEADLINE_SEC) while the partner launches and the hive tips again.
 *   3. Back to the rear launch spot (the ramp stows during the turn) and wait until G_SECOND_LAUNCH_NOT_BEFORE_SEC (the partner's tip is over), then launch the flower pollen.
 *   4. The garden as in the solo auto (angled approach, slow pickup, square on the wall), back to the rear launch spot and launch the garden pollen: the 8th, which tips the cell.
 *   5. Park in the loading zone through a helper point clear of the hive supports, ramp stowed.
 * The first runs will show how this fits the 30 s; nothing is cut yet (design/duo-autos.md).
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

        HiveTarget cell = DuoKit.cellForAlliance(HiveTarget.BLUE_REAR, blue);   // blue: rear, red: audience
        m.setOpenCell(cell);

        Pose2d spotXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_LAUNCH_X, DuoAutos.G_LAUNCH_Y, 0.0), blue);
        double aim = TargetLock.aimHeadingRad(spotXY.position.x, spotXY.position.y, cell, 0.0);
        Pose2d launchSpot = new Pose2d(spotXY.position.x, spotXY.position.y, aim);

        // the Blue Wall flower: the audience-wall flower's poses turned 90 deg counterclockwise about the center (heading 90, facing the Blue Wall), plus the Dashboard nudge
        Pose2d wallPreB = nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG)));
        Pose2d wallRamB = nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG)));
        Pose2d wallBackB = nudge(DuoKit.rotate90(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG)));
        Pose2d wallPre = DuoKit.forAlliance(wallPreB, blue);
        Pose2d wallRam = DuoKit.forAlliance(wallRamB, blue);
        Pose2d wallBack = DuoKit.forAlliance(wallBackB, blue);

        // the garden, as in the solo auto
        Pose2d gardenPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.GARDEN_PRE_X, DuoAutos.GARDEN_PRE_Y, DuoAutos.GARDEN_HEADING_DEG), blue);
        Pose2d gardenEnd = DuoKit.forAlliance(DuoKit.pose(DuoAutos.GARDEN_END_X, DuoAutos.GARDEN_END_Y, DuoAutos.GARDEN_HEADING_DEG), blue);
        double sideIn = profile.originFromSideIn();
        double squareX = -(FieldGeometry.HALF_IN - sideIn - DuoAutos.SQUARE_GAP_IN);
        Pose2d squarePre = DuoKit.forAlliance(DuoKit.pose(squareX, DuoAutos.GARDEN_END_Y, 90.0), blue);
        double squareFlushX = (blue ? -1.0 : 1.0) * (FieldGeometry.HALF_IN - sideIn);
        double squareFlushY = (blue ? 1.0 : -1.0) * (FieldGeometry.HALF_IN - profile.originFromFrontIn());
        double squareHeadingRad = Math.toRadians(blue ? 90.0 : 270.0);

        Pose2d helper = DuoKit.forAlliance(DuoKit.pose(DuoAutos.HELPER_X, DuoAutos.HELPER_Y, DuoAutos.PARK_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.G_PARK_X, DuoAutos.G_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.PARK_HEADING_DEG), blue);

        return new SequentialAction(
                new InstantAction(m::preparePusherForLaunch),   // the pusher moves to the lock position while the flywheel spins up: the preload salvo turns 310 deg instead of 400
                k.launch(drive, m, cell, blue, "preload", DuoAutos.G_PRELOAD_TRIM_DEG),

                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("To the Blue Wall flower pre-position, turning to it"),
                k.leg("wall flower pre-position", drive, wallPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: Blue Wall flower (held IN, whatever the count, until the flower salvo is done)"),
                m.setIntake(true),
                m.forceIntakeIn(true),
                k.ramLeg("ram the wall flower", drive, wallRam),
                k.timed("flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.G_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.note("Back off the flower to the launch spot (intake still ON)"),
                k.backOutLeg("wall flower back-out (straight)", drive, wallBack),
                k.stowWhenTurned(k.launchLeg("launch spot 1", drive, launchSpot), drive, m, wallPre.heading.toDouble()),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.waitUntil("second launch (partner's tip)", DuoAutos.G_SECOND_LAUNCH_NOT_BEFORE_SEC),
                k.launch(drive, m, cell, blue, "flower pollen", DuoAutos.G_FLOWER_TRIM_DEG),

                k.note("Garden pre-position"),
                k.passLeg("garden pre-position", drive, gardenPre),
                k.note("Intake ON: garden, slow"),
                m.setIntake(true),
                k.slowLeg("garden pickup (slow)", drive, gardenEnd),
                k.note("Square on the wall"),
                k.passLeg("square approach", drive, squarePre),
                k.timed("square on the wall", new SquareOnWall(drive, squareHeadingRad, squareFlushX, squareFlushY)),
                k.note("Back to the launch spot (intake still ON so every element gets in)"),
                k.launchLeg("launch spot 2", drive, launchSpot),
                k.launch(drive, m, cell, blue, "garden pollen", DuoAutos.G_GARDEN_TRIM_DEG),

                k.note("Park"),
                k.passLeg("park helper (clear of the hive supports)", drive, helper),
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
