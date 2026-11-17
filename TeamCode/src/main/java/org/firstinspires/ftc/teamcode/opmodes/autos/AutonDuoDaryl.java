package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;

/**
 * Auton - Duo Daryl (specs/autos.md section 4): garden first; the partner gets a window in the middle.
 * The 18 s is the partner's window: our preload ends at about 3 s, a tip takes 4 s, plus 2 s; the partner launches at
 * about 9 s and is parked by about 18 s. Estimated duration 28 to 30 s, so the last gather is skipped whenever anything runs late.
 */
@Autonomous(name = "Auton - Duo Daryl", group = "1 Competition", preselectTeleOp = "TeleOp - Competition")
public class AutonDuoDaryl extends AutoBase {
    @Override protected String title() { return "Auton - Duo Daryl"; }
    @Override protected Pose blueStart() { return Field.startRear(profile.originToBack); }
    @Override protected String placement() { return "back flush against the rear wall"; }
    @Override protected double jostleLimit() { return AutoParams.DUO_JOSTLE_LIMIT; }

    /** Steps 1 to 4, shared with Cybugs: preload, garden, the garden pollen at the audience cell, the Blue Wall flower. */
    protected Action firstFour() {
        Pose shot = aimed(AutoPoses.audienceLaunch(), Cell.BLUE_AUDIENCE, AutoParams.AUDIENCE_CELL_TRIM_DEG);
        return Actions.sequence(
                launch("preload", Cell.BLUE_REAR, 0),
                gardenPickup(),
                setTarget(Cell.BLUE_AUDIENCE),
                passThrough("east helper", via(Field.helperEast(), shot), true),
                toLaunchSpot("audience launch spot", shot, true),
                launch("garden pollen", Cell.BLUE_AUDIENCE, AutoParams.AUDIENCE_CELL_TRIM_DEG),
                passThrough("east helper", via(Field.helperEast(), AutoPoses.flowerPre(true)), false),
                gather(true, AutoParams.DARYL_WALL_JOSTLE_DEADLINE));
    }

    @Override
    protected Action program() {
        Pose shot = aimed(AutoPoses.audienceLaunch(), Cell.BLUE_AUDIENCE, AutoParams.AUDIENCE_CELL_TRIM_DEG);
        return Actions.sequence(
                firstFour(),
                // Do not leave the flower before the second launch time less the drive back.
                waitUntil(AutoParams.DARYL_SECOND_LAUNCH_NOT_BEFORE - AutoParams.DARYL_RETURN_TRAVEL, "do not leave the flower early"),
                backOut(true),
                withStowDuringTurn(passThrough("east helper", via(Field.helperEast(), shot), false), true),
                toLaunchSpot("audience launch spot", shot, false),
                mech.waitRampSettled(),
                waitUntil(AutoParams.DARYL_SECOND_LAUNCH_NOT_BEFORE, "the partner's window"),
                launch("flower pollen", Cell.BLUE_AUDIENCE, AutoParams.AUDIENCE_CELL_TRIM_DEG),
                // Its pollen stay for the TeleOp.
                unlessLate(AutoParams.DARYL_AUDIENCE_FLOWER_LATEST_START, "audience flower gather", Actions.sequence(
                        gather(false, AutoParams.DARYL_AUDIENCE_JOSTLE_DEADLINE),
                        backOut(false),
                        stowNow())),
                park(false));
    }
}
