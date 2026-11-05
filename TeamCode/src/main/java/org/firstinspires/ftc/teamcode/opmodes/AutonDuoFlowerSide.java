package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ChassisProfile;
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
 *   1. Start with the back flush to the audience wall (heading 180); the flywheel spins from Start.
 *   2. Wait until F_FIRST_LAUNCH_NOT_BEFORE_SEC (the partner's preload salvo and the 4 s tip), then launch the preload into the audience cell (which the tip just opened; it needs 8 pollen).
 *   3. Deploy the ramp, gather the audience-wall flower as the solo auto does (ram, jostle until full or F_JOSTLE_DEADLINE_SEC), back straight out, turn to the audience launch spot
 *      (the ramp stows during the turn) and launch the flower pollen: the 8th pollen, which tips that cell.
 *   4. Park in the loading zone (via a point clear of the flower), ramp stowed.
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

        HiveTarget cell = DuoKit.cellForAlliance(HiveTarget.BLUE_AUDIENCE, blue);   // blue: audience, red: rear
        m.setOpenCell(cell);

        Pose2d spotXY = DuoKit.forAlliance(DuoKit.pose(DuoAutos.F_LAUNCH_X, DuoAutos.F_LAUNCH_Y, 0.0), blue);
        double aim = TargetLock.aimHeadingRad(spotXY.position.x, spotXY.position.y, cell, 0.0);
        Pose2d launchSpot = new Pose2d(spotXY.position.x, spotXY.position.y, aim);
        Pose2d flowerPre = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_PRE_X, DuoAutos.FLOWER_PRE_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerRam = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X + DuoAutos.FLOWER_RAM_OVERSHOOT_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d flowerBack = DuoKit.forAlliance(DuoKit.pose(DuoAutos.FLOWER_AT_X - DuoAutos.FLOWER_BACKOFF_IN, DuoAutos.FLOWER_AT_Y, DuoAutos.FLOWER_HEADING_DEG), blue);
        Pose2d parkVia = DuoKit.forAlliance(DuoKit.pose(DuoAutos.F_PARK_VIA_X, DuoAutos.F_PARK_VIA_Y, DuoAutos.PARK_HEADING_DEG), blue);
        Pose2d park = DuoKit.forAlliance(DuoKit.pose(DuoAutos.F_PARK_X, DuoAutos.F_PARK_Y + DuoAutos.PARK_OVERSHOOT_IN, DuoAutos.PARK_HEADING_DEG), blue);

        return new SequentialAction(
                k.waitUntil("first launch (partner's tip)", DuoAutos.F_FIRST_LAUNCH_NOT_BEFORE_SEC),
                k.launch(drive, m, cell, blue, "preload", DuoAutos.F_PRELOAD_TRIM_DEG, false),

                new InstantAction(m::requestDeploy),   // deploys on the way
                k.note("To the flower pre-position, turning to the flower"),
                k.leg("flower pre-position", drive, flowerPre),
                k.timed("ramp deploy (wait)", k.rampTo(m, true)),
                k.note("Intake ON: flower"),
                m.setIntake(true),
                k.ramLeg("ram the flower", drive, flowerRam),
                k.timed("flower jostle", new Jostle(drive, k.jostleDone(m, DuoAutos.F_JOSTLE_DEADLINE_SEC),
                        () -> m.counter == null ? 0 : m.counter.count(), DuoAutos.FLOWER_PAUSE_MAX_SEC)),
                k.note("Back off the flower to the launch spot (intake still ON)"),
                k.backOutLeg("flower back-out (straight)", drive, flowerBack),
                k.stowWhenTurned(k.launchLeg("launch spot", drive, launchSpot), drive, m, flowerPre.heading.toDouble()),
                m.setIntake(false),
                k.timed("ramp stow (wait)", k.rampTo(m, false)),
                k.launch(drive, m, cell, blue, "flower pollen", DuoAutos.F_FLOWER_TRIM_DEG, true),

                k.note("Park"),
                k.passLeg("park via point", drive, parkVia),
                k.parkLeg("park (full power)", drive, park),
                k.timed("ramp stow (final)", k.rampTo(m, false)),
                new InstantAction(m::stopFlywheel)
        );
    }
}
