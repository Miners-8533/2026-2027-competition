package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * AUTON - TRIPLE TIP (2026-11-20, the user's design; built, not run on the robot). The Solo auto ({@link AutonSolo}) up to and including the first flower gather: the preload into the first cell (tip 1), the garden,
 * the garden salvo into the second cell from the Solo's launch spot, the audience-wall flower (blue; red is turned 180 deg). The PARTNER waits and launches with our garden salvo, so the second cell tips (tip 2) and
 * the first cell faces up again. Then, instead of the Solo's flower salvo: around the OUTSIDE of the hive on our own half (never under it: the lane under the hive is the field's center line) to the first cell's
 * launch spot, the flower pollen into the first cell, the alliance-wall flower, its pollen into the first cell from the same spot (8 pollen: tip 3), and the park. Every number is in AutonSolo (the REAR FINISH
 * values, and everything the two autos share), so a change to the Solo's start changes this one too.
 */
@Autonomous(name = "Auton - Triple Tip", group = "Auto")
public class AutonTripleTip extends AutonSolo {

    @Override
    protected String autoName() {
        return "Auton - Triple Tip";
    }

    @Override
    protected boolean rearFinish() {
        return true;
    }
}
