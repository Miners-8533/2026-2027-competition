import org.firstinspires.ftc.teamcode.drive.HiveRoute;
import org.firstinspires.ftc.teamcode.drive.LaunchGrid;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

/**
 * Writes the launching spots the Launch Grid Tuner visits (LaunchGrid.generate, the blue hive's cells, the tuner's settings: 12 in pitch, 75 deg, 3 in wall margin) to
 * tools/launchgrid/data/lattice_<CELL>.csv: cell, x, y, aim heading (deg), distance (in), slid. make_table.py reads them to find the spots nobody measured (and estimates them
 * from the nearest measured ones). Run: tools\launchgrid\dump-lattice.cmd
 */
public class DumpLattice {
    static final double LAUNCHER_OFFSET_LEFT_IN = -2.5, FAR_EXTRA_LEFT_IN = -3.0;   // TargetLock's values at 2026-11-13

    public static void main(String[] args) throws IOException {
        String dir = args.length > 0 ? args[0] : "tools/launchgrid/data";
        HiveRoute.Params p = new HiveRoute.Params();
        p.robotHeightIn = 18.0;
        p.marginIn = 1.5;
        p.flowerClearanceIn = 3.0;
        for (HiveTarget cell : new HiveTarget[]{HiveTarget.BLUE_REAR, HiveTarget.BLUE_AUDIENCE}) {
            double extra = cell.x > 0 ? FAR_EXTRA_LEFT_IN : 0.0;   // blue: the audience cell is the far one
            LaunchGrid.Aim aim = (x, y) -> {
                double dx = cell.x - x, dy = cell.y - y;
                double par = Math.asin(Math.max(-0.9, Math.min(0.9, (LAUNCHER_OFFSET_LEFT_IN + extra) / Math.hypot(dx, dy))));
                return Math.atan2(dy, dx) - par;
            };
            List<LaunchGrid.Point> pts = LaunchGrid.generate(cell, 12.0, 75.0, 12.0, 71.0 - 3.0, aim, p);
            try (FileWriter w = new FileWriter(dir + "/lattice_" + cell.name() + ".csv")) {
                w.write("cell,x,y,heading_deg,distance_in,slid\n");
                for (LaunchGrid.Point q : pts) {
                    w.write(String.format(Locale.US, "%s,%.2f,%.2f,%.1f,%.1f,%d\n", cell.name(), q.x, q.y, Math.toDegrees(q.headingRad), q.distanceIn, q.slid ? 1 : 0));
                }
            }
            System.out.println(cell + ": " + pts.size() + " spots");
        }
    }
}
