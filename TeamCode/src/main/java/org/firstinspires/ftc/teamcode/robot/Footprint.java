package org.firstinspires.ftc.teamcode.robot;

/** The robot's outline around its origin, inches. Pure data, so the planners and the PC checks can use it without the SDK. */
public final class Footprint {
    public final double front, back, halfWidth, height;

    public Footprint(double front, double back, double halfWidth, double height) {
        this.front = front;
        this.back = back;
        this.halfWidth = halfWidth;
        this.height = height;
    }

    /** ROBOT_1 from CAD (robot_design/chassis.md). The height is not recorded; the planners take 18 in. */
    public static final Footprint ROBOT_1 = new Footprint(17.48 - 7.598, 7.598, 13.70 / 2, 18.0);

    /** Distance from the origin to the farthest corner. */
    public double farCorner() { return Math.hypot(Math.max(front, back), halfWidth); }

    /** The four corners in the field frame for a pose: front-left, front-right, back-right, back-left. */
    public double[][] corners(double x, double y, double h) {
        double c = Math.cos(h), s = Math.sin(h);
        double[][] local = {{front, halfWidth}, {front, -halfWidth}, {-back, -halfWidth}, {-back, halfWidth}};
        double[][] out = new double[4][2];
        for (int i = 0; i < 4; i++) {
            out[i][0] = x + local[i][0] * c - local[i][1] * s;
            out[i][1] = y + local[i][0] * s + local[i][1] * c;
        }
        return out;
    }
}
