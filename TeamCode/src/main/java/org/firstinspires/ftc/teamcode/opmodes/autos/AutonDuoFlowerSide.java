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
 * Auton - Duo Flower Side (specs/autos.md section 6): the other half of the pair, from the audience wall.
 * Hidden: complete code, disabled from the Driver Station list. Remove the @Disabled line to bring it back.
 */
@Disabled
@Autonomous(name = "Auton - Duo Flower Side", group = "1 Competition", preselectTeleOp = "TeleOp - Competition")
public final class AutonDuoFlowerSide extends AutoBase {
    @Override protected String title() { return "Auton - Duo Flower Side"; }
    @Override protected Pose blueStart() { return Field.startAudience(profile.originToBack); }
    @Override protected String placement() { return "back flush against the audience wall"; }
    @Override protected double legPower() { return 0.6; }
    @Override protected double crossPower() { return 0.85; }
    @Override protected double passSettle() { return 0.2; }
    @Override protected double jostleLimit() { return AutoParams.DUO_JOSTLE_LIMIT; }

    @Override
    protected Action program() {
        Pose shot = aimed(AutoPoses.audienceAtWall(), Cell.BLUE_AUDIENCE, 0);
        return Actions.sequence(
                // The partner's preload salvo, the tip, a margin.
                waitUntil(AutoParams.FLOWER_SIDE_FIRST_LAUNCH_NOT_BEFORE, "the partner's preload and the tip"),
                launch("preload", Cell.BLUE_AUDIENCE, 0),
                gather(false, AutoParams.FLOWER_SIDE_JOSTLE_DEADLINE),
                backOut(false),
                withStowDuringTurn(toLaunchSpot("audience spot at the wall", shot, false), false),
                launch("flower pollen", Cell.BLUE_AUDIENCE, 0),
                passThrough("via (46, 32)", via(AutoPoses.flowerSideVia(), Field.parkAuto(true)), false),
                park(true));
    }
}
