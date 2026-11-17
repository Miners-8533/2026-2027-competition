package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.math.Vec;

/** A hive cell the launcher is aimed at (specs/field.md section 2). "Rear" and "audience" name the side the cell opens toward. */
public enum Cell {
    BLUE_REAR(Alliance.BLUE, true),
    BLUE_AUDIENCE(Alliance.BLUE, false),
    RED_REAR(Alliance.RED, true),
    RED_AUDIENCE(Alliance.RED, false);

    public static final double CELL_X = 21.5, CELL_Y = 12.75;

    public final Alliance alliance;
    public final boolean rear;

    Cell(Alliance alliance, boolean rear) {
        this.alliance = alliance;
        this.rear = rear;
    }

    public static Cell of(Alliance a, boolean rear) {
        return a == Alliance.BLUE ? (rear ? BLUE_REAR : BLUE_AUDIENCE) : (rear ? RED_REAR : RED_AUDIENCE);
    }

    public double x() { return rear ? -CELL_X : CELL_X; }

    public double y() { return alliance == Alliance.BLUE ? CELL_Y : -CELL_Y; }

    public Vec point() { return new Vec(x(), y()); }

    /** +1 when the cell opens toward +X (the audience wall), -1 toward -X. */
    public double openSign() { return rear ? -1 : 1; }

    /** The heading of a robot standing square to the wall on the open side, facing the cell. */
    public double faceHeading() { return rear ? 0 : Math.PI; }

    /** The far cell is on the opposite side of the field from the alliance's usual start wall. */
    public boolean isFar() { return (alliance == Alliance.BLUE) != rear; }

    /** The cell this one becomes under the 180 degree rotation. */
    public Cell rotated() {
        switch (this) {
            case BLUE_REAR: return RED_AUDIENCE;
            case BLUE_AUDIENCE: return RED_REAR;
            case RED_REAR: return BLUE_AUDIENCE;
            default: return BLUE_REAR;
        }
    }

    public String label() { return rear ? "REAR" : "AUDIENCE"; }
}
