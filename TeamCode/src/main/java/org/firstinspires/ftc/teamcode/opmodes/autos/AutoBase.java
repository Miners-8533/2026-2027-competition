package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.aiming.Aim;
import org.firstinspires.ftc.teamcode.drive.Handover;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.motion.ActionRunner;
import org.firstinspires.ftc.teamcode.motion.AimAtCell;
import org.firstinspires.ftc.teamcode.motion.DriveToPose;
import org.firstinspires.ftc.teamcode.motion.Jostle;
import org.firstinspires.ftc.teamcode.motion.SquareOnWall;
import org.firstinspires.ftc.teamcode.opmodes.TeamOpMode;
import org.firstinspires.ftc.teamcode.robot.ChassisProfile;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The common frame of the six autonomous programs (specs/autos.md sections 1 and 2): INIT, Start, the end, and the
 * pieces every program is built from (drive legs, aim and launch, the garden pickup, the flower gather, waits and
 * skips, the park). Every path is written for blue; red is the same path rotated 180 degrees about the field center.
 */
public abstract class AutoBase extends TeamOpMode {
    /** Remembered between runs of any auto until the app restarts. */
    private static Alliance chosen = Alliance.BLUE;

    protected Alliance alliance = chosen;
    private double startedAt = -1;
    private final List<String> timings = new ArrayList<>();

    /** The start pose, blue frame. */
    protected abstract Pose blueStart();

    /** Where to place the robot, for the INIT screen. */
    protected abstract String placement();

    /** The program: one sequential action. */
    protected abstract Action program();

    /** Leg power, cross-field power and the settle time of a pass-through point (specs/autos.md section 2.1). */
    protected double legPower() { return 0.85; }
    protected double crossPower() { return 1.0; }
    protected double passSettle() { return 0.0; }
    /** The jostle's own time limit: 9 s in Solo, 12 s in the duo programs. */
    protected double jostleLimit() { return 12.0; }

    @Override protected RunLog.Kind logKind() { return RunLog.Kind.AUTO; }

    protected double sinceStart() { return startedAt < 0 ? 0 : Clock.now() - startedAt; }

    @Override
    protected final void body() throws InterruptedException {
        profile = ChassisProfile.get();
        // INIT: reset and seed at the start pose (the robot must be in place and still); the pusher zero is assumed at HOLD;
        // the ramp is commanded stowed and the flywheel position to POLLEN.
        drive = MecanumDrive.resetAndSeed(hardwareMap, profile, alliance.pose(blueStart()));
        mech = new Mechanisms(hardwareMap, profile, alliance, Mechanisms.Settings.auto());
        Edge right = new Edge(), left = new Edge();
        while (opModeInInit()) {
            drive.updatePose();
            mech.initLoop();
            boolean r = right.pressed(gamepad1.dpad_right), l = left.pressed(gamepad1.dpad_left);
            if (r || l) {
                Alliance a = r ? Alliance.BLUE : Alliance.RED;
                if (a != alliance) {
                    alliance = chosen = a;
                    mech.setAlliance(a);
                    drive.localizer.seed(a.pose(blueStart()));     // a change re-seeds the pose
                    RunLog.eventf("PINPOINT re-seeded at INIT for %s: %s", a, a.pose(blueStart()));
                }
            }
            initScreen();
            RunLog.tick();
            idle();
        }
        if (isStopRequested()) return;

        // At Start.
        markStarted();
        startedAt = Clock.now();
        Handover.dropAims();            // the auto has reseeded the Pinpoint: the remembered aims belonged to the old pose
        mech.setSpin(true);             // the flywheel runs from Start and stays up between launches
        mech.prePositionPreload();      // to the lock angle while the flywheel spins up, so the preload salvo is shorter
        RunLog.eventf("SETUP profile %s, alliance %s, start pose default %s (%s), mechanisms: %s, held count starts at %s",
                profile.name, alliance, drive.pose(), drive.localizer.seedConfirmed() ? "seed confirmed" : "SEED NOT CONFIRMED", mech.found(), mech.heldText());
        RunLog.sampler("x,y,hdg_deg," + MecanumDrive.LOG_COLUMNS.replace(",a_lf,a_lb,a_rb,a_rf", "") + "," + Mechanisms.LOG_COLUMNS
                + ",a_lf,a_lb,a_rb,a_rf", RunLog.NORMAL_PERIOD_S, new RunLog.Sampler() {
                    @Override public String row() {
                        Pose p = drive.pose();
                        return String.format(Locale.US, "%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%s,%.1f,%.1f,%.1f,%.1f", p.x, p.y, p.headingDeg(),
                                drive.cmdForward(), drive.cmdLeft(), drive.cmdTurn(), mech.logRow(),
                                drive.wheelCheck.lastAmps(0), drive.wheelCheck.lastAmps(1), drive.wheelCheck.lastAmps(2), drive.wheelCheck.lastAmps(3));
                    }
                });

        // The program, with the flywheel stopped as the last step of the sequence.
        Action all = Actions.sequence(program(), Actions.instant(new Runnable() {
            @Override public void run() {
                mech.setSpin(false);
                drive.stop();
                StringBuilder sb = new StringBuilder("AUTO TIMING all steps:");
                for (String s : timings) sb.append(' ').append(s).append(';');
                RunLog.event(sb.toString());
                RunLog.eventf("AUTO finished at %.2f s", sinceStart());
            }
        }));
        ActionRunner.EveryLoop everyLoop = new ActionRunner.EveryLoop() {
            @Override public void run() {
                mech.update(drive.pose(), true);
                Handover.savePose(drive.pose(), alliance.isRed());
                Handover.saveCount(mech.heldCountForSave());
            }
        };
        ActionRunner.run(this, drive, all, everyLoop);
        // Until the opmode is stopped, the pose keeps being read and saved and the mechanisms keep updating.
        while (opModeIsActive()) {
            drive.updatePose();
            everyLoop.run();
            RunLog.tick();
            telemetry.addLine("Auto finished at " + drive.pose());
            telemetry.update();
        }
    }

    /** Whether it finished or was stopped, the final pose and alliance are saved for the TeleOp. */
    @Override
    protected void saveAtEnd() {
        if (drive == null || startedAt < 0) return;
        Handover.savePose(drive.pose(), alliance.isRed());
        if (mech != null) Handover.saveCount(mech.heldCountForSave());
        RunLog.eventf("PINPOINT hand-over: saved %s for %s, held count %s", drive.pose(), alliance, mech == null ? "-" : mech.heldText());
    }

    private void initScreen() {
        Pose p = drive.pose(), s = alliance.pose(blueStart());
        boolean ok = drive.localizer.seedConfirmed() && p.distTo(s) <= 1.5 && Math.abs(Math.toDegrees(Angle.diff(p.h, s.h))) <= 3.0;
        telemetry.addLine(title() + "  " + alliance + "  (D-pad right BLUE / left RED)");
        telemetry.addLine("Place: " + placement() + ", at " + s);
        telemetry.addLine(String.format(Locale.US, "Pose %s: %s", p, ok ? "OK" : "NOT OK" + (drive.localizer.seedConfirmed() ? "" : " (seed not confirmed)")));
        if (ChassisProfile.noHubRegistered()) telemetry.addLine("no hub registered yet");
        telemetry.addLine("Found: " + mech.found());
        telemetry.addLine("Slot count: " + (mech.slots == null ? "no sensors" : !mech.slots.valid() ? "not reading"
                : mech.slots.count() + " (" + mech.slots.slots() + "), 4 are expected"));
        telemetry.update();
    }

    // ================= Pieces =================

    /** A blue-frame pose as this alliance drives it, worked out when the leg starts. */
    protected DriveToPose.Target at(final Pose blue) {
        return new DriveToPose.Target() {
            @Override public Pose get() { return alliance.pose(blue); }
        };
    }

    protected Cell cell(Cell blue) { return alliance.cell(blue); }

    /** A blue-frame spot with the heading that already includes the aim and the trim, so the aim step only fine-tunes. */
    protected Pose aimed(Vec blueSpot, Cell blueCell, double trimDeg) {
        return new Pose(blueSpot.x, blueSpot.y, Aim.forAuto(blueSpot.x, blueSpot.y, blueCell, trimDeg));
    }

    /** A blue-frame point with the heading of another pose. */
    protected Pose via(Vec bluePoint, Pose headingOf) { return new Pose(bluePoint.x, bluePoint.y, headingOf.h); }

    /** Ordinary leg: ends within 1 in and 2 deg, after 0.2 s. */
    protected Action ordinary(String name, Pose blue) {
        return new DriveToPose(drive, name, at(blue)).maxPower(legPower()).distanceTimeout();
    }

    /** Pass-through point: 3 in, 10 deg, at once (or after the program's pass-through settle). The robot does not need to stop well. */
    protected Action passThrough(String name, Pose blue, boolean cross) {
        return new DriveToPose(drive, name, at(blue)).maxPower(cross ? crossPower() : legPower()).tolerance(3.0, 10.0).settle(passSettle()).distanceTimeout();
    }

    /** Launch spot: deliberately loose (2.5 in, 6 deg); the aim step that follows turns the robot onto the cell. */
    protected Action toLaunchSpot(String name, Pose blue, boolean cross) {
        return new DriveToPose(drive, name, at(blue)).maxPower(cross ? crossPower() : legPower()).tolerance(2.5, 6.0).distanceTimeout();
    }

    /** Aim at the cell with this launch's one trim, then launch one salvo and wait for it to complete or be dropped. A salvo is always launched. */
    protected Action launch(final String name, final Cell blueCell, double trimDeg) {
        return step("launch " + name, Actions.sequence(
                Actions.instant(new Runnable() {
                    @Override public void run() { mech.setTargetCell(cell(blueCell)); }
                }),
                new AimAtCell(drive, new AimAtCell.CellSource() {
                    @Override public Cell get() { return cell(blueCell); }
                }, trimDeg, false),
                mech.launchAction("auto " + name)));
    }

    protected Action setTarget(final Cell blueCell) {
        return Actions.instant(new Runnable() {
            @Override public void run() { mech.setTargetCell(cell(blueCell)); }
        });
    }

    /** The garden pickup: pre-position, slow pickup along the wall, square approach, square on the wall. */
    protected Action gardenPickup() {
        final double hw = profile.halfWidth(), front = profile.originToFront();
        return step("garden", Actions.sequence(
                ordinary("garden pre-position", AutoPoses.gardenPre()),
                new DriveToPose(drive, "garden pickup", at(AutoPoses.gardenEnd())).maxPower(0.35).tolerance(3.0, 10.0).distanceTimeout(),
                new DriveToPose(drive, "square approach", at(AutoPoses.squareApproach(hw))).maxPower(legPower()).tolerance(2.5, 6.0).distanceTimeout(),
                new SquareOnWall(drive, new SquareOnWall.Expected() {
                    @Override public Pose get() { return alliance.pose(AutoPoses.squareFlush(hw, front)); }
                })));
    }

    /**
     * The flower gather up to the end of the jostle: deploy the ramp on the way, drive to the pre-position facing the
     * flower, wait for the ramp, force the intake in, ram, jostle.
     *
     * @param wall     true = the Blue Wall flower, false = the audience-wall flower
     * @param deadline the jostle's deadline in seconds since Start
     */
    protected Action gather(final boolean wall, final double deadline) {
        String which = wall ? "wall flower" : "audience flower";
        return step("gather " + which, Actions.sequence(
                Actions.instant(new Runnable() {
                    @Override public void run() { mech.deployRamp(); }
                }),
                new DriveToPose(drive, which + " pre-position", at(AutoPoses.flowerPre(wall))).maxPower(legPower())
                        .tolerance(2.5, wall ? 4.0 : 6.0).distanceTimeout(),
                mech.waitRampSettled(),
                Actions.instant(new Runnable() {
                    @Override public void run() { mech.setAutoForceIn(true); }
                }),
                // The ram: at a target beyond the flower, never reached, on a fixed clock.
                new DriveToPose(drive, which + " ram", at(AutoPoses.ramTarget(wall))).maxPower(0.6).timeout(0.6).timed(),
                new Jostle(drive, new Actions.Condition() {
                    @Override public boolean get() { return mech.full() || sinceStart() >= deadline; }
                }, new Jostle.Held() {
                    @Override public int get() { return mech.heldCount(); }
                }, jostleLimit()),
                Actions.instant(new Runnable() {
                    @Override public void run() { mech.setAutoForceIn(false); }
                })));
    }

    /** Back out straight before any turn: turning at the flower catches the ramp on the stand. */
    protected Action backOut(final boolean wall) {
        return new DriveToPose(drive, "back out", new DriveToPose.Target() {
            @Override public Pose get() {
                Pose p = drive.pose();
                double h = alliance.pose(AutoPoses.flower(wall)).h;
                return new Pose(p.x - AutoPoses.BACK_OUT_IN * Math.cos(h), p.y - AutoPoses.BACK_OUT_IN * Math.sin(h), h);
            }
        }).maxPower(legPower()).tolerance(wall ? 2.5 : 1.5, 10.0).distanceTimeout();
    }

    protected Action stowNow() { return mech.stowNowAction(); }

    /** Stows the ramp during the following turn, as soon as the robot has turned 45 deg from the flower heading. Run it in parallel with the leg. */
    protected Action stowWhenTurned(final boolean wall) {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                double h = alliance.pose(AutoPoses.flower(wall)).h;
                if (Math.abs(Math.toDegrees(Angle.diff(drive.pose().h, h))) < 45.0) return true;
                mech.stowRampNow();
                return false;
            }
        };
    }

    /** A leg with the ramp stowing during its turn; if the leg ends without the turn, the ramp is stowed then. */
    protected Action withStowDuringTurn(Action leg, boolean wall) {
        return Actions.sequence(Actions.parallel(leg, stowUntil(leg, wall)), Actions.instant(new Runnable() {
            @Override public void run() {
                if (mech.ramp != null && mech.ramp.deployed()) mech.stowRampNow();
            }
        }));
    }

    private Action stowUntil(final Action leg, final boolean wall) {
        final Action stow = stowWhenTurned(wall);
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                if (leg instanceof DriveToPose && ((DriveToPose) leg).done()) return false;
                return stow.run(p);
            }
        };
    }

    /** Wait until an absolute time since Start. A robot cannot see its partner, so coordination is by the clock. */
    protected Action waitUntil(final double seconds, final String why) {
        return new Action() {
            boolean logged;

            @Override public boolean run(TelemetryPacket p) {
                if (!logged) {
                    logged = true;
                    RunLog.eventf("AUTO WAIT until %.1f s (%s): it is %.2f s%s", seconds, why, sinceStart(), sinceStart() >= seconds ? ", already passed" : "");
                }
                return sinceStart() < seconds;
            }
        };
    }

    /** A block of steps that runs only if it starts no later than a given time; otherwise it is skipped so that the park keeps its time. */
    protected Action unlessLate(final double latestStart, final String name, final Action block) {
        return new Action() {
            int decided;      // 0 = not yet, 1 = run, 2 = skip

            @Override public boolean run(TelemetryPacket p) {
                if (decided == 0) {
                    decided = sinceStart() <= latestStart ? 1 : 2;
                    if (decided == 2) RunLog.eventf("AUTO SKIP %s: it is %.2f s, later than %.1f s", name, sinceStart(), latestStart);
                }
                return decided == 1 && block.run(p);
            }
        };
    }

    /** Full power to the park pose, facing the Blue Wall, then make sure the ramp is stowed. */
    protected Action park(boolean audienceSide) {
        return step("park", Actions.sequence(
                new DriveToPose(drive, "park", at(Field.parkAuto(audienceSide))).maxPower(1.0).tolerance(2.5, 5.0).timeout(3.0),
                Actions.instant(new Runnable() {
                    @Override public void run() {
                        if (mech.ramp != null && mech.ramp.deployed()) mech.stowRampNow();
                    }
                })));
    }

    /** Wraps a step so its duration and its start and end since Start are logged. */
    protected Action step(final String name, final Action a) {
        return new Action() {
            double start = -1;

            @Override public boolean run(TelemetryPacket p) {
                if (start < 0) start = sinceStart();
                boolean more = a.run(p);
                if (!more) {
                    double end = sinceStart();
                    timings.add(String.format(Locale.US, "%s %.2f s", name, end - start));
                    RunLog.eventf("AUTO TIMING %s: %.2f s (%.2f to %.2f)", name, end - start, start, end);
                }
                return more;
            }
        };
    }

    protected Action intakeOutForTheRest() {
        return Actions.instant(new Runnable() {
            @Override public void run() {
                RunLog.event("AUTO intake OUT for the rest of the program");
                mech.setOuttake(true);
            }
        });
    }

    /** For subclasses that need the intake's present mode in a note. */
    protected Intake.Mode intakeMode() { return mech.intake == null ? Intake.Mode.OFF : mech.intake.mode(); }
}
