package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.aiming.LaunchGrid;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;

/**
 * Part of the coordinator (specs/mechanisms.md section 2): the target cell, the launch distance and the flywheel
 * setpoint. Pure logic.
 */
public final class SpeedSelector {
    /** The snapshot. */
    public static final class In {
        public Pose pose;
        public Alliance alliance;
        public boolean followsPose;
        /** The cell the opmode set (used when the target does not follow the pose). */
        public Cell setCell;
        public boolean backup;
        public double backupDistance;
        /** While a Hive Dash runs: its cell, its launch spot and the distance from that spot. Null otherwise. */
        public Cell dashCell;
        public Vec dashSpot;
        public double dashDistance;
        public double speedTrim;
        public boolean gridAllowed;
        public boolean flywheelLost;
    }

    public Cell cell;
    public double distance, setpoint, mapSpeed;
    /** The grid's answer for the spot, whether the grid is on or off; null when there is no pose. */
    public LaunchGrid.Result grid;
    public boolean gridUsed, mapEmpty;

    public void update(In in) {
        // 1. Target cell.
        if (in.dashCell != null) {
            cell = in.dashCell;
        } else if (in.followsPose && !in.backup && in.pose != null) {
            Cell rear = Cell.of(in.alliance, true), aud = Cell.of(in.alliance, false);
            double dRear = Aim.distance(in.pose.x, in.pose.y, rear), dAud = Aim.distance(in.pose.x, in.pose.y, aud);
            if (cell == null || cell.alliance != in.alliance) {
                cell = dRear <= dAud ? rear : aud;
            } else {
                // The nearer cell, switching only when the other is clearly closer, so it does not flip-flop.
                double mine = cell.rear ? dRear : dAud, other = cell.rear ? dAud : dRear;
                if (other + MechParams.CELL_HYSTERESIS_IN < mine) cell = cell.rear ? aud : rear;
            }
        } else if (in.setCell != null) {
            cell = in.setCell;
        } else if (cell == null) {
            cell = Cell.of(in.alliance, true);
        }
        // 2. Distance.
        double x = in.pose == null ? 0 : in.pose.x, y = in.pose == null ? 0 : in.pose.y;
        if (in.backup) distance = in.backupDistance;
        else if (in.dashCell != null) distance = in.dashDistance;
        else distance = Aim.distance(x, y, cell);
        // 3. Setpoint: the launch map's speed for that distance plus the operator's speed trim.
        mapEmpty = LaunchMap.isEmpty();
        mapSpeed = mapEmpty ? MechParams.MANUAL_SETPOINT : LaunchMap.speed(distance);
        setpoint = mapSpeed + in.speedTrim;
        // 4. Launch grid: looked up for the launch spot during a dash; never in backup mode.
        gridUsed = false;
        grid = null;
        if (!in.backup && in.pose != null) {
            double gx = in.dashSpot != null ? in.dashSpot.x : x, gy = in.dashSpot != null ? in.dashSpot.y : y;
            grid = LaunchGrid.lookup(cell, gx, gy);
            if (in.gridAllowed && LaunchGrid.ENABLED && grid.hasValues() && !Double.isNaN(grid.speed)) {
                setpoint = grid.speed + in.speedTrim;
                gridUsed = true;
            }
        }
        // 5. Flywheel encoder lost: the base speed plus the trim, whatever the distance.
        if (in.flywheelLost) setpoint = Flywheel.lostBaseSpeed() + in.speedTrim;
    }
}
