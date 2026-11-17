package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * Auton - Cybugs (specs/autos.md section 5): Daryl's first four steps, then a finish that stops collecting.
 * It ends about 5 s early (measured 24.9 to 25.9 s). Because the outtake counts as busy, the automatic pusher check
 * after the last salvo is skipped; the TeleOp's start-up check covers it.
 */
@Autonomous(name = "Auton - Cybugs", group = "1 Competition", preselectTeleOp = "TeleOp - Competition")
public final class AutonCybugs extends AutonDuoDaryl {
    @Override protected String title() { return "Auton - Cybugs"; }

    @Override
    protected Action program() {
        Pose shot = aimed(AutoPoses.rearAtStart(), Cell.BLUE_REAR, AutoParams.CYBUGS_REAR_TRIM_DEG);
        return Actions.sequence(
                firstFour(),
                backOut(true),
                setTarget(Cell.BLUE_REAR),
                withStowDuringTurn(toLaunchSpot("rear spot at the start wall", shot, false), true),
                mech.waitRampSettled(),
                Actions.sleep(AutoParams.CYBUGS_WAIT_BEFORE_LAST_SALVO),     // with the flywheel spinning
                launch("flower pollen", Cell.BLUE_REAR, AutoParams.CYBUGS_REAR_TRIM_DEG),
                intakeOutForTheRest(),       // so nothing more is picked up on the way to the park
                passThrough("west helper", via(Field.helperWest(), Field.parkAuto(false)), false),
                park(false));
    }
}
