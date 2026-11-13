import org.firstinspires.ftc.teamcode.drive.LaunchGridMap;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Evaluates LaunchGridMap.lookup on a fine grid of field positions for one cell and writes cell, x, y, speed, residual, zone, estimated to a CSV (for the surface pictures:
 * tools/launchgrid/plot_map.py). Usage: DumpMap CELL out.csv [step_in]
 */
public class DumpMap {
    public static void main(String[] args) throws IOException {
        HiveTarget cell = HiveTarget.valueOf(args[0]);
        double step = args.length > 2 ? Double.parseDouble(args[2]) : 2.0;
        try (FileWriter w = new FileWriter(args[1])) {
            w.write("cell,x,y,speed,residual_deg,zone,estimated\n");
            for (double x = -70; x <= 70.01; x += step) {
                for (double y = -70; y <= 70.01; y += step) {
                    LaunchGridMap.Result r = LaunchGridMap.lookup(cell, x, y);
                    w.write(String.format(Locale.US, "%s,%.1f,%.1f,%.1f,%.2f,%s,%d\n", cell, x, y, r.speed, r.residualDeg, r.zone, r.estimated ? 1 : 0));
                }
            }
        }
        System.out.println("wrote " + args[1]);
    }
}
