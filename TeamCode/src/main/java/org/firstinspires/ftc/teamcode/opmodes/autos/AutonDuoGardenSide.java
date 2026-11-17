package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * Auton - Duo Garden Side (specs/autos.md section 6): one half of a matched pair with Duo Flower Side. Each robot
 * launches only at the cell on its own start side, so neither needs an allowance for the far cell; all aim trims are 0.
 * As written it does not fit 30 s if its wait is really 25 s: the waits were first guesses to be tuned from logged runs.
 */
@Autonomous(name = "Auton - Duo Garden Side", group = "1 Competition", preselectTeleOp = "TeleOp - Competition")
public final class AutonDuoGardenSide extends AutoBase {
    @Override protected String title() { return "Auton - Duo Garden Side"; }
    @Override protected Pose blueStart() { return Field.startRear(profile.originToBack); }
    @Override protected String placement() { return "back flush against the rear wall"; }
    @Override protected double legPower() { return 0.6; }
    @Override protected double crossPower() { return 0.85; }
    @Override protected double passSettle() { return 0.2; }
    @Override protected double jostleLimit() { return AutoParams.DUO_JOSTLE_LIMIT; }

    @Override
    protected Action program() {
        Pose shot = aimed(AutoPoses.rearAtStart(), Cell.BLUE_REAR, 0);
        return Actions.sequence(
                launch("preload", Cell.BLUE_REAR, 0),
                gather(true, AutoParams.GARDEN_SIDE_JOSTLE_DEADLINE),
                backOut(true),
                withStowDuringTurn(toLaunchSpot("rear spot at the start wall", shot, false), true),
                waitUntil(AutoParams.GARDEN_SIDE_SECOND_LAUNCH_NOT_BEFORE, "the partner's turn at the hive"),
                launch("flower pollen", Cell.BLUE_REAR, 0),
                gardenPickup(),
                toLaunchSpot("rear spot at the start wall", shot, false),
                launch("garden pollen", Cell.BLUE_REAR, 0),
                passThrough("west helper", via(Field.helperWest(), Field.parkAuto(false)), false),
                park(false));
    }
}
