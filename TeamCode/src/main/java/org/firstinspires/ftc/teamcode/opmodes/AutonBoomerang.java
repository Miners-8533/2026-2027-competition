package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * AUTON - BOOMERANG (2026-11-21, the user: Solo, but the last salvo into the other hive cell, then park; built, not run on the robot). The Solo auto ({@link AutonSolo}) up to and including the wall
 * flower's gather and back-out: the preload into the first cell, the garden and the audience-wall flower into the second cell from the Solo launch spot, the alliance-wall flower. Then, instead of the
 * Solo's third salvo into the second cell from that same spot (81 in away, across the field): straight to the first cell's launch spot (BOOMERANG_LAUNCH, blue (-57.6, 26.2) from 2026-11-21: 2 in nearer the wall than the Triple Tip's and the Daryl's (-55.6, 26.2),
 * 37 in away), the wall flower's pollen into the FIRST cell (the one the preload went into: blue the rear cell, red the audience cell) with the 2-D grid's speed and aim, and the park at FINAL_PARK
 * through REAR_PARK_VIA at REAR_PARK_HEADING_DEG, as those two autos park. Every number is in AutonSolo, so a change to the Solo changes this one too.
 */
@Autonomous(name = "Auton - Boomerang", group = "Auto")
public class AutonBoomerang extends AutonSolo {

    @Override
    protected String autoName() {
        return "Auton - Boomerang";
    }

    @Override
    protected boolean lastSalvoIntoFirstCell() {
        return true;
    }
}
