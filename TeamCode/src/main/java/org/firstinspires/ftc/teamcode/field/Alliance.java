package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;

/** The alliance. Everything is defined for blue; red is blue rotated 180 degrees about the field center (specs/field.md). */
public enum Alliance {
    BLUE, RED;

    public boolean isRed() { return this == RED; }

    public static Alliance of(boolean red) { return red ? RED : BLUE; }

    /** A blue-frame pose, as this alliance drives it. */
    public Pose pose(Pose blue) { return this == BLUE ? blue : blue.rotated180(); }

    /** A blue-frame point, as this alliance uses it. */
    public Vec point(Vec blue) { return this == BLUE ? blue : blue.rotated180(); }

    /** A blue-frame cell, as this alliance aims at it (blue rear corresponds to red audience). */
    public Cell cell(Cell blue) { return this == BLUE ? blue : blue.rotated(); }

    /** The field direction "stick up" means: the way this alliance's drivers look (blue -Y, red +Y). */
    public double driverForwardRad() { return this == BLUE ? -Math.PI / 2 : Math.PI / 2; }

    /** The cell on the driver's left (blue: audience, red: rear) or right. */
    public Cell cellOnDriversLeft(boolean left) {
        boolean rear = (this == BLUE) != left;
        return Cell.of(this, rear);
    }
}
