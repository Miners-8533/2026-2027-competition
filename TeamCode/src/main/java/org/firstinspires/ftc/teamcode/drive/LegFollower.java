package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.math.Rotation2d;

/**
 * The part of following a planned route (the waypoints of a {@link HiveRoute.Plan}) that the Hive Dash and the Park Dash share (2026-11-16; each had its own copy, and the copies had drifted):
 * which leg the robot is on, when a via point counts as reached, when the goal is reached, and the translation command in the FIELD frame for this loop. What differs between the dashes
 * stays in each of them: their Dashboard values (passed in a {@link Config} every loop), what a new leg does (the heading it wants, the log line: {@link ViaListener}), the turn, the lock
 * and what happens on arrival.
 *
 * The follower: on every leg but the last the robot drives along the leg at the full power (less the corner slow-down, when the dash uses one) with a sideways correction back onto the
 * leg's line; a via point is reached when the robot is within passTolIn of it along the leg, or past it. The last leg heads straight for the goal point, slowing with the distance, so a
 * robot that overshoots it or passes it to the side is driven back.
 *
 * No hardware and no SDK types: tools/dash/run-sim.cmd compiles it for the PC with the dashes.
 */
final class LegFollower {

    /** One dash's values. Filled from its Dashboard statics on every loop, so a change on the Dashboard is used at once. */
    static final class Config {
        double maxPower, goalTolIn, passTolIn;
        /** The last leg: power per inch from the goal, and the least power. */
        double alongKp, minFinalPower;
        /** The other legs: power per inch off the leg's line, and its limit. */
        double crossKp, crossMax;
        /** The last leg's power is at most approachPower within approachRadiusIn of the goal. 0: no limit (the Park Dash). */
        double approachPower, approachRadiusIn;
        /**
         * The corner slow-down: the power at a via point where the route turns cornerFullDeg or more is cornerPower (none below cornerFreeDeg, in proportion between), rising by
         * cornerRampPerIn for every inch still to go to it. cornerPower 0: none (the Park Dash).
         */
        double cornerPower, cornerFreeDeg, cornerFullDeg, cornerRampPerIn;
    }

    /** Called when a via point is reached, before the next leg is looked at: nextLegIndex is the leg that starts there. */
    interface ViaListener {
        void viaReached(int nextLegIndex, double[] via);
    }

    /** The answer for one loop. */
    static final class Step {
        /** The goal is reached (within goalTolIn on the last leg): the other values but legIndex and goalDist mean nothing. */
        boolean arrived;
        /** The leg the robot is on now (after any via points reached in this loop). */
        int legIndex;
        /** True on the last leg. */
        boolean last;
        double goalDist;
        /** The translation command in the field frame. */
        double vfx, vfy;
    }

    private LegFollower() {}

    /**
     * One loop of the follower for a robot at (x, y) on leg legIndex of the waypoints, with the goal at (goalX, goalY) (the last waypoint). At most 4 via points are passed in one loop.
     * The answer is written into out and returned.
     */
    static Step follow(double[][] waypoints, int legIndex, double x, double y, double goalX, double goalY, Config c, ViaListener onVia, Step out) {
        int legs = waypoints.length - 1;
        double goalDist = Math.hypot(x - goalX, y - goalY);
        double ux = 0, uy = 0, nx = 0, ny = 0, along = 0, cross = 0, remaining = 0;
        out.arrived = false;
        out.goalDist = goalDist;
        for (int guard = 0; guard < 4; guard++) {
            double[] a = waypoints[legIndex], b = waypoints[legIndex + 1];
            double len = Math.hypot(b[0] - a[0], b[1] - a[1]);
            ux = len < 1e-6 ? 1.0 : (b[0] - a[0]) / len;
            uy = len < 1e-6 ? 0.0 : (b[1] - a[1]) / len;
            nx = -uy;
            ny = ux;
            along = (x - a[0]) * ux + (y - a[1]) * uy;
            cross = (x - a[0]) * nx + (y - a[1]) * ny;
            remaining = len - along;
            boolean last = legIndex == legs - 1;
            if (last) {
                if (goalDist <= c.goalTolIn) {
                    out.arrived = true;
                    out.legIndex = legIndex;
                    out.last = true;
                    out.vfx = 0.0;
                    out.vfy = 0.0;
                    return out;
                }
                break;
            }
            if (remaining > c.passTolIn && along <= len) break;
            legIndex++;
            onVia.viaReached(legIndex, b);
        }
        boolean last = legIndex == legs - 1;
        double vfx, vfy;
        if (last) {
            // the last leg heads straight for the goal point: a robot that overshoots it or passes it to the side is driven back (power along the leg kept pushing it on past)
            double v = clamp(c.alongKp * goalDist, c.minFinalPower, c.maxPower);
            if (c.approachPower > 0.0 && goalDist <= c.approachRadiusIn) v = Math.min(v, Math.max(c.approachPower, c.minFinalPower));
            double d = Math.max(goalDist, 1e-6);
            vfx = (goalX - x) / d * v;
            vfy = (goalY - y) / d * v;
        } else {
            double vC = clamp(-c.crossKp * cross, -c.crossMax, c.crossMax);
            double vMag = c.maxPower;
            if (c.cornerPower > 0.0 && c.cornerFullDeg > c.cornerFreeDeg) {
                // slow down for the turn at the via point ahead: the next leg's direction against this one's
                double[] via = waypoints[legIndex + 1], next = waypoints[legIndex + 2];
                double turnDeg = Math.toDegrees(Math.abs(Rotation2d.normalize(Math.atan2(next[1] - via[1], next[0] - via[0]) - Math.atan2(uy, ux))));
                double f = clamp((turnDeg - c.cornerFreeDeg) / (c.cornerFullDeg - c.cornerFreeDeg), 0.0, 1.0);
                double cornerPower = c.maxPower + (Math.min(c.cornerPower, c.maxPower) - c.maxPower) * f;
                vMag = Math.min(c.maxPower, cornerPower + c.cornerRampPerIn * Math.max(0.0, remaining));
            }
            vfx = ux * vMag + nx * vC;
            vfy = uy * vMag + ny * vC;
        }
        out.legIndex = legIndex;
        out.last = last;
        out.vfx = vfx;
        out.vfy = vfy;
        return out;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
