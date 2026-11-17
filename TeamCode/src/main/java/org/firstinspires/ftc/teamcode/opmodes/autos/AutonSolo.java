package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * Auton - Solo (specs/autos.md section 3): the partner only parks. Rear wall start; cells: rear, then audience.
 * Nominal duration about 29 s.
 */
@Autonomous(name = "Auton - Solo", group = "1 Competition", preselectTeleOp = "TeleOp - Competition")
public final class AutonSolo extends AutoBase {
    @Override protected String title() { return "Auton - Solo"; }
    @Override protected Pose blueStart() { return Field.startRear(profile.originToBack); }
    @Override protected String placement() { return "back flush against the rear wall"; }
    @Override protected double jostleLimit() { return AutoParams.SOLO_JOSTLE_LIMIT; }

    @Override
    protected Action program() {
        Pose gardenShot = aimed(AutoPoses.audienceLaunch(), Cell.BLUE_AUDIENCE, AutoParams.SOLO_GARDEN_TRIM_DEG);
        Pose flowerShot = aimed(AutoPoses.audienceLaunch(), Cell.BLUE_AUDIENCE, AutoParams.SOLO_FLOWER_TRIM_DEG);
        Action finish;
        if (AutoParams.SOLO_FINAL_GATHER) {
            // The final gather: the pollen stay in the robot for the TeleOp.
            finish = Actions.sequence(
                    passThrough("east helper", via(Field.helperEast(), AutoPoses.flowerPre(true)), true),
                    gather(true, AutoParams.SOLO_FINAL_JOSTLE_DEADLINE),
                    backOut(true),
                    stowNow(),
                    park(false));
        } else {
            finish = ordinary("plain park", AutoPoses.soloPlainPark());
        }
        return Actions.sequence(
                launch("preload", Cell.BLUE_REAR, 0),
                gardenPickup(),
                setTarget(Cell.BLUE_AUDIENCE),
                passThrough("east helper", via(Field.helperEast(), gardenShot), true),
                toLaunchSpot("audience launch spot", gardenShot, true),
                launch("garden pollen", Cell.BLUE_AUDIENCE, AutoParams.SOLO_GARDEN_TRIM_DEG),
                gather(false, AutoParams.SOLO_JOSTLE_DEADLINE),
                backOut(false),
                withStowDuringTurn(toLaunchSpot("audience launch spot", flowerShot, false), false),
                launch("flower pollen", Cell.BLUE_AUDIENCE, AutoParams.SOLO_FLOWER_TRIM_DEG),
                finish);
    }
}
