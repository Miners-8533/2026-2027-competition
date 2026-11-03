package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.PoseVelocity2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.math.Vector2d;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Drives the robot to a target field pose and holds it, using three independent PID
 * controllers on field-X error, field-Y error, and heading error. The field-frame
 * translation command is rotated into the robot frame and handed to
 * {@link MecanumDrive#setDrivePowers}.
 *
 * Finishes when position and heading are within tolerance for {@link #SETTLE_SEC},
 * or when {@link #TIMEOUT_SEC} elapses (whichever first). Stops the drive on finish.
 *
 * All gains and tolerances are {@code @Config} statics so they can be tuned live in
 * FTC Dashboard while a tuning OpMode runs. Every {@link #run} copies the statics
 * into the controllers, so edits take effect immediately.
 */
@Config
public final class DriveToPose implements Action {

    // --- axial (field X) ---
    public static double AXIAL_kP = 0.05;
    public static double AXIAL_kI = 0.0;
    public static double AXIAL_kD = 0.006;
    public static double AXIAL_kS = 0.0;

    // --- lateral (field Y) --- strafing is weaker, usually needs a bit more P
    public static double LATERAL_kP = 0.06;
    public static double LATERAL_kI = 0.0;
    public static double LATERAL_kD = 0.008;
    public static double LATERAL_kS = 0.0;

    // --- heading (radians of error) ---
    public static double HEADING_kP = 1.5;
    public static double HEADING_kI = 0.0;
    public static double HEADING_kD = 0.08;
    public static double HEADING_kS = 0.0;

    public static double MAX_OUTPUT = 0.6;      // per-axis power clamp (keep modest while tuning)
    public static double MAX_INTEGRAL = 0.25;

    public static double POS_TOLERANCE_IN = 1.0;
    public static double HEADING_TOLERANCE_DEG = 2.0;
    public static double SETTLE_SEC = 0.20;     // time within tolerance before "done"
    public static double TIMEOUT_SEC = 4.0;     // hard cap

    private final MecanumDrive drive;
    private final Pose2d target;
    private double timeoutSec = TIMEOUT_SEC;

    private final PidController xCtrl = new PidController(AXIAL_kP, AXIAL_kI, AXIAL_kD);
    private final PidController yCtrl = new PidController(LATERAL_kP, LATERAL_kI, LATERAL_kD);
    private final PidController hCtrl = new PidController(HEADING_kP, HEADING_kI, HEADING_kD);

    private final ElapsedTime timeout = new ElapsedTime();
    private final ElapsedTime settle = new ElapsedTime();
    private boolean initialized = false;

    public DriveToPose(MecanumDrive drive, Pose2d target) {
        this.drive = drive;
        this.target = target;
    }

    /** Use this hard cap, seconds, for this drive instead of {@link #TIMEOUT_SEC} (a long leg needs more than the default). */
    public DriveToPose withTimeout(double seconds) {
        timeoutSec = seconds;
        return this;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!initialized) {
            initialized = true;
            timeout.reset();
            settle.reset();
            xCtrl.reset();
            yCtrl.reset();
            hCtrl.reset();
            Pose2d from = drive.getPose();
            RunLog.event(String.format("DRIVE start: (%.1f, %.1f, %.0f deg) -> (%.1f, %.1f, %.0f deg)",
                    from.position.x, from.position.y, from.heading.toDegrees(),
                    target.position.x, target.position.y, target.heading.toDegrees()));
        }
        applyGains();

        drive.updatePoseEstimate();
        Pose2d pose = drive.getPose();
        double heading = pose.heading.toDouble();

        double errX = target.position.x - pose.position.x;                 // field frame
        double errY = target.position.y - pose.position.y;                 // field frame
        double errH = Rotation2d.normalize(target.heading.toDouble() - heading);

        double vxField = xCtrl.calculate(errX);
        double vyField = yCtrl.calculate(errY);
        double omega = hCtrl.calculate(errH);

        // field-frame translation -> robot frame (rotate by -heading)
        Vector2d robotVel = new Vector2d(vxField, vyField).rotated(-heading);
        drive.setDrivePowers(new PoseVelocity2d(robotVel, omega));

        double posError = Math.hypot(errX, errY);
        boolean inTolerance = posError < POS_TOLERANCE_IN
                && Math.abs(errH) < Math.toRadians(HEADING_TOLERANCE_DEG);
        if (!inTolerance) {
            settle.reset();
        }
        boolean done = (inTolerance && settle.seconds() >= SETTLE_SEC)
                || timeout.seconds() >= timeoutSec;

        packet.put("dtp/errX_in", errX);
        packet.put("dtp/errY_in", errY);
        packet.put("dtp/errH_deg", Math.toDegrees(errH));
        packet.put("dtp/posError_in", posError);
        packet.put("dtp/x_in", pose.position.x);
        packet.put("dtp/y_in", pose.position.y);
        packet.put("dtp/h_deg", pose.heading.toDegrees());
        packet.put("dtp/targetX_in", target.position.x);
        packet.put("dtp/targetY_in", target.position.y);
        packet.put("dtp/targetH_deg", target.heading.toDegrees());

        if (done) {
            drive.stop();
            RunLog.event(String.format("DRIVE %s after %.2f s: error %.2f in (x %+.2f, y %+.2f), heading %+.1f deg",
                    inTolerance ? "settled" : "TIMED OUT", timeout.seconds(), posError, errX, errY, Math.toDegrees(errH)));
        }
        return !done;
    }

    private void applyGains() {
        xCtrl.kP = AXIAL_kP;   xCtrl.kI = AXIAL_kI;   xCtrl.kD = AXIAL_kD;   xCtrl.kStatic = AXIAL_kS;
        yCtrl.kP = LATERAL_kP; yCtrl.kI = LATERAL_kI; yCtrl.kD = LATERAL_kD; yCtrl.kStatic = LATERAL_kS;
        hCtrl.kP = HEADING_kP; hCtrl.kI = HEADING_kI; hCtrl.kD = HEADING_kD; hCtrl.kStatic = HEADING_kS;
        for (PidController c : new PidController[]{xCtrl, yCtrl, hCtrl}) {
            c.maxOutput = MAX_OUTPUT;
            c.maxIntegral = MAX_INTEGRAL;
        }
    }
}
