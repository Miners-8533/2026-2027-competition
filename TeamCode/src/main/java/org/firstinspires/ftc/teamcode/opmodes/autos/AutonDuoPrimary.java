package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * Auton - Duo Primary (specs/autos.md section 7): the longest program. With the garden it ran past 30 s and lost its
 * park, hence the skip. Hidden: remove the @Disabled line to bring it back.
 */
@Disabled
@Autonomous(name = "Auton - Duo Primary", group = "1 Competition", preselectTeleOp = "TeleOp - Competition")
public final class AutonDuoPrimary extends AutoBase {
    @Override protected String title() { return "Auton - Duo Primary"; }
    @Override protected Pose blueStart() { return Field.startRear(profile.originToBack); }
    @Override protected String placement() { return "back flush against the rear wall"; }
    @Override protected double jostleLimit() { return AutoParams.DUO_JOSTLE_LIMIT; }

    @Override
    protected Action program() {
        Pose audShot = aimed(AutoPoses.audienceLaunch(), Cell.BLUE_AUDIENCE, AutoParams.AUDIENCE_CELL_TRIM_DEG);
        Pose rearShot = aimed(AutoPoses.rearLaunch(), Cell.BLUE_REAR, 0);
        return Actions.sequence(
                launch("preload", Cell.BLUE_REAR, 0),
                gather(true, AutoParams.PRIMARY_WALL_JOSTLE_DEADLINE),
                backOut(true),
                setTarget(Cell.BLUE_AUDIENCE),
                withStowDuringTurn(passThrough("east helper", via(Field.helperEast(), audShot), true), true),
                toLaunchSpot("audience launch spot", audShot, false),
                launch("wall flower pollen", Cell.BLUE_AUDIENCE, AutoParams.AUDIENCE_CELL_TRIM_DEG),
                gather(false, AutoParams.PRIMARY_AUDIENCE_JOSTLE_DEADLINE),
                backOut(false),
                setTarget(Cell.BLUE_REAR),
                withStowDuringTurn(passThrough("east helper", via(Field.helperEast(), rearShot), false), false),
                passThrough("west helper", via(Field.helperWest(), rearShot), true),
                toLaunchSpot("rear launch spot", rearShot, false),
                launch("audience flower pollen", Cell.BLUE_REAR, 0),
                unlessLate(AutoParams.PRIMARY_GARDEN_LATEST_START, "garden pickup and its launch", Actions.sequence(
                        gardenPickup(),
                        toLaunchSpot("rear launch spot", rearShot, false),
                        launch("garden pollen", Cell.BLUE_REAR, 0))),
                passThrough("west helper", via(Field.helperWest(), Field.parkAuto(false)), false),
                park(false));
    }
}
