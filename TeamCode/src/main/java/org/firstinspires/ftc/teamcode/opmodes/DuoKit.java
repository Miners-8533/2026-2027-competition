package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Rotation2d;
import org.firstinspires.ftc.teamcode.motion.Action;
import org.firstinspires.ftc.teamcode.motion.AimAtCell;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.motion.InstantAction;
import org.firstinspires.ftc.teamcode.motion.SequentialAction;
import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;

/**
 * The building blocks of the two duo autos: the drive legs with their timeouts, the timed steps (an {@code AUTO TIMING} log line each), the launch (aim, then launch, always: never skipped for an empty count), the ramp helpers, the wait until a time, and the jostle's end condition. They are the same pieces as in {@code AutonSolo}, with their numbers from {@link DuoAutos}.
 */
final class DuoKit {

    final ElapsedTime autoClock = new ElapsedTime();
    final List<String> timings = new ArrayList<>();
    private final Telemetry telemetry;

    // The Primary asks for the solo auto's faster legs (fastLegs); NaN keeps the old behavior of the other two duo autos.
    private double legPower = Double.NaN, crossPower = Double.NaN, passSettleSec = Double.NaN;

    DuoKit(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /** Every ordinary leg at legMaxPower, the cross-field legs at crossMaxPower, and pass-through legs ending after passSettleSec within tolerance. */
    void fastLegs(double legMaxPower, double crossMaxPower, double passSettle) {
        legPower = legMaxPower;
        crossPower = crossMaxPower;
        passSettleSec = passSettle;
    }

    /** A drive to the target with the leg power (when set) and a hard cap whose speed scales with it. */
    private DriveToPose baseDrive(MecanumDrive drive, Pose2d target) {
        DriveToPose d = new DriveToPose(drive, target);
        double scale = 1.0;
        if (!Double.isNaN(legPower)) {
            d.withMaxOutput(legPower);
            scale = legPower / 0.6;
        }
        return d.withDistanceTimeout(DuoAutos.TIMEOUT_BASE_SEC, DuoAutos.TIMEOUT_IN_PER_S * scale, DuoAutos.TIMEOUT_MIN_SEC, DuoAutos.TIMEOUT_TURN_EXTRA_SEC, DuoAutos.TIMEOUT_MAX_SEC);
    }

    // ---- poses ----

    static Pose2d pose(double x, double y, double headingDeg) {
        return new Pose2d(x, y, Math.toRadians(headingDeg));
    }

    /** The blue-frame pose as this alliance sees it: red is the blue pose turned 180 deg about the field center. */
    static Pose2d forAlliance(Pose2d p, boolean blue) {
        return blue ? p : new Pose2d(-p.position.x, -p.position.y, p.heading.toDouble() + Math.PI);
    }

    /** The blue-frame cell as this alliance's own: blue rear is red audience and blue audience is red rear (the 180 deg turn). */
    static HiveTarget cellForAlliance(HiveTarget blueCell, boolean blue) {
        if (blue) return blueCell;
        return blueCell == HiveTarget.BLUE_REAR ? HiveTarget.RED_AUDIENCE : HiveTarget.RED_REAR;
    }

    /** A pose turned 90 deg counterclockwise about the field center: the audience-wall flower's poses become the Blue Wall flower's. */
    static Pose2d rotate90(Pose2d p) {
        return new Pose2d(-p.position.y, p.position.x, p.heading.toDouble() + Math.PI / 2.0);
    }

    // ---- steps ----

    Action note(String message) {
        return new InstantAction(() -> {
            telemetry.log().add(message);
            RunLog.event("AUTO " + message);
        });
    }

    /** Runs the action and logs how long it took, and when it ended, once it finishes (an "AUTO TIMING" line, one per step). */
    Action timed(String label, Action inner) {
        return new Action() {
            private boolean started = false;
            private double begin;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    begin = autoClock.seconds();
                }
                boolean more = inner.run(packet);
                if (!more) {
                    double now = autoClock.seconds();
                    RunLog.event(String.format(Locale.US, "AUTO TIMING %-30s %5.2f s   (%5.2f to %5.2f s into the auto)", label, now - begin, begin, now));
                    timings.add(String.format(Locale.US, "%s %.2f", label, now - begin));
                }
                return more;
            }
        };
    }

    /** Waits until this many seconds into the auto (a partner's tip, with the flywheel running), and logs how long it waited. Does nothing if the time has passed. */
    Action waitUntil(String label, double sec) {
        return new Action() {
            private boolean started = false;
            private double begin;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    begin = autoClock.seconds();
                }
                if (autoClock.seconds() >= sec) {
                    RunLog.event(String.format(Locale.US, "AUTO WAIT %-26s until %.1f s: waited %.2f s (%s)", label, sec, autoClock.seconds() - begin,
                            begin >= sec ? "the time had already passed" : "on time"));
                    return false;
                }
                return true;
            }
        };
    }

    // ---- drive legs ----

    Action leg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, baseDrive(drive, target));
    }

    /** The Blue Wall flower's pre-position: near enough (the solo auto's wall tolerances), because it stalled 1.9 in short of 1 in. */
    Action wallPreLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, baseDrive(drive, target).withTolerance(DuoAutos.WALL_PRE_POS_TOL_IN, DuoAutos.WALL_PRE_HEADING_TOL_DEG));
    }

    /** The garden leg of the Daryl auto: from where the robot is straight to the wall squaring pose, the last D_GARDEN_SLOW_ZONE_IN inches limited to D_GARDEN_SLOW_POWER (the pollen are in a line of 4). */
    Action gardenLeg(String label, MecanumDrive drive, Pose2d target) {
        DriveToPose d = baseDrive(drive, target).withTolerance(DuoAutos.COARSE_POS_IN, DuoAutos.COARSE_HEADING_DEG)
                .withSlowZone(DuoAutos.D_GARDEN_SLOW_ZONE_IN, DuoAutos.D_GARDEN_SLOW_POWER);
        if (!Double.isNaN(passSettleSec)) d.withSettle(passSettleSec);
        return timed("drive: " + label, d);
    }

    Action passLeg(String label, MecanumDrive drive, Pose2d target) {
        DriveToPose d = baseDrive(drive, target).withTolerance(DuoAutos.COARSE_POS_IN, DuoAutos.COARSE_HEADING_DEG);
        if (!Double.isNaN(passSettleSec)) d.withSettle(passSettleSec);
        return timed("drive: " + label, d);
    }

    /** A launch-spot leg: near is enough (the aim step turns onto the cell). */
    Action launchLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, baseDrive(drive, target).withTolerance(DuoAutos.LAUNCH_POS_TOL_IN, DuoAutos.LAUNCH_HEADING_TOL_DEG));
    }

    Action crossLeg(String label, MecanumDrive drive, Pose2d target, boolean pass) {
        double maxPower = Double.isNaN(crossPower) ? DuoAutos.CROSS_FIELD_MAX_POWER : crossPower;
        DriveToPose d = new DriveToPose(drive, target).withMaxOutput(maxPower).withDistanceTimeout(DuoAutos.TIMEOUT_BASE_SEC, DuoAutos.TIMEOUT_IN_PER_S * maxPower / 0.6,
                DuoAutos.TIMEOUT_MIN_SEC, DuoAutos.TIMEOUT_TURN_EXTRA_SEC, DuoAutos.TIMEOUT_MAX_SEC);
        d.withTolerance(pass ? DuoAutos.COARSE_POS_IN : DuoAutos.LAUNCH_POS_TOL_IN, pass ? DuoAutos.COARSE_HEADING_DEG : DuoAutos.LAUNCH_HEADING_TOL_DEG);
        if (pass && !Double.isNaN(passSettleSec)) d.withSettle(passSettleSec);
        return timed("drive: " + label, d);
    }

    Action slowLeg(String label, MecanumDrive drive, Pose2d target) {
        double maxPower = DuoAutos.GARDEN_PICKUP_MAX_POWER;
        return timed("drive: " + label, new DriveToPose(drive, target).withMaxOutput(maxPower).withTolerance(DuoAutos.COARSE_POS_IN, DuoAutos.COARSE_HEADING_DEG)
                .withDistanceTimeout(DuoAutos.TIMEOUT_BASE_SEC, DuoAutos.SLOW_IN_PER_S_PER_POWER * maxPower, DuoAutos.TIMEOUT_MIN_SEC, DuoAutos.TIMEOUT_TURN_EXTRA_SEC, DuoAutos.TIMEOUT_MAX_SEC));
    }

    Action backOutLeg(String label, MecanumDrive drive, Pose2d target) {
        return backOutLeg(label, drive, target, DuoAutos.BACKOUT_TOL_IN);
    }

    Action backOutLeg(String label, MecanumDrive drive, Pose2d target, double tolIn) {
        return timed("drive: " + label, baseDrive(drive, target).withTolerance(tolIn, DuoAutos.COARSE_HEADING_DEG));
    }

    /** Drives hard at a target beyond an obstacle for a fixed time (the flower ram). */
    Action ramLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withMaxOutput(DuoAutos.FLOWER_RAM_POWER).withTolerance(0.5, 5.0).withTimeout(DuoAutos.FLOWER_RAM_SEC));
    }

    Action parkLeg(String label, MecanumDrive drive, Pose2d target) {
        return timed("drive: " + label, new DriveToPose(drive, target).withMaxOutput(DuoAutos.PARK_MAX_POWER)
                .withTolerance(DuoAutos.PARK_POS_TOL_IN, DuoAutos.PARK_HEADING_TOL_DEG).withTimeout(DuoAutos.PARK_TIMEOUT_SEC));
    }

    // ---- ramp ----

    /** Commands the ramp only if it is not already heading there (a repeated command restarts its travel timer), then waits until it has settled. */
    Action rampTo(Mechanisms mechanisms, boolean deployed) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (mechanisms.ramp == null || !mechanisms.ramp.isConfigured()) return false;
                if (!started) {
                    started = true;
                    if (mechanisms.ramp.isDeployedTarget() != deployed) {
                        if (deployed) mechanisms.requestDeploy();
                        else mechanisms.requestStowNow();
                    }
                }
                return !mechanisms.ramp.isSettled();
            }
        };
    }

    /** Runs {@code inner} (a drive) and commands the ramp to stow once the robot has turned DuoAutos.STOW_TURN_DEG from flowerHeadingRad (once). */
    Action stowWhenTurned(Action inner, MecanumDrive drive, Mechanisms mechanisms, double flowerHeadingRad) {
        return new Action() {
            private boolean stowed = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                boolean running = inner.run(packet);
                if (!stowed) {
                    Pose2d p = drive.getPose();
                    double turnedDeg = Math.toDegrees(Math.abs(Rotation2d.normalize(p.heading.toDouble() - flowerHeadingRad)));
                    if (turnedDeg >= DuoAutos.STOW_TURN_DEG) {
                        stowed = true;
                        mechanisms.requestStowNow();
                        RunLog.event(String.format(Locale.US, "AUTO ramp stow during the turn: %.0f deg from the flower heading, pose (%.1f, %.1f)", turnedDeg, p.position.x, p.position.y));
                    }
                }
                return running;
            }
        };
    }

    // ---- launch and jostle ----

    /**
     * Aim at the cell with the lock, launch one salvo, and finish when it is done. The salvo is ALWAYS launched, whatever the element counter says (the user, 2026-11-06: a counter that under-reads
     * must never throw a salvo away). extraLeftIn 0: no far-cell correction (the duo autos launch at the cell on their own start side).
     */
    Action launch(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label, double trimDeg) {
        return launch(drive, mechanisms, cell, blue, label, trimDeg, 0.0);
    }

    /** As above with the sideways aim extra: NaN = the far-cell extra of the cell on the opposite side from the alliance's start wall (TargetLock.farCellExtraLeftIn). */
    Action launch(MecanumDrive drive, Mechanisms mechanisms, HiveTarget cell, boolean blue, String label, double trimDeg, double extraLeftIn) {
        return new SequentialAction(
                note("Aim and launch: " + label),
                timed("aim: " + label, new AimAtCell(drive, cell, blue, trimDeg, extraLeftIn)),
                timed("launch (wait + salvo): " + label, mechanisms.launch()));
    }

    /**
     * Runs {@code inner} only if the auto is not later than latestStartSec when this step is reached; otherwise logs AUTO SKIP and does nothing (so the park still has its time). Decided once.
     */
    Action unlessLate(String label, double latestStartSec, Action inner) {
        return new Action() {
            private boolean decided = false, run = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!decided) {
                    decided = true;
                    run = autoClock.seconds() <= latestStartSec;
                    RunLog.event(String.format(Locale.US, run ? "AUTO %s starts at %.2f s into the auto (latest start %.1f s)" : "AUTO SKIP %s: it would start at %.2f s, later than %.1f s, so the park keeps its time",
                            label, autoClock.seconds(), latestStartSec));
                }
                return run && inner.run(packet);
            }
        };
    }

    /** The jostle is done when the robot is full or deadlineSec into the auto has passed (logged once). */
    BooleanSupplier jostleDone(Mechanisms mechanisms, double deadlineSec) {
        return new BooleanSupplier() {
            private boolean logged = false;

            @Override
            public boolean getAsBoolean() {
                if (mechanisms.isFull()) return true;
                if (autoClock.seconds() >= deadlineSec) {
                    if (!logged) {
                        logged = true;
                        RunLog.event(String.format(Locale.US, "AUTO jostle deadline reached at %.2f s into the auto: moving on with %d held", autoClock.seconds(),
                                mechanisms.counter == null ? -1 : mechanisms.counter.count()));
                    }
                    return true;
                }
                return false;
            }
        };
    }
}
