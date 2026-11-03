package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

/**
 * Field dimensions the code uses, in inches, measured from the field center (the pose frame's origin).
 *
 * HALF_IN: the distance from the center to each perimeter wall. The team measured the field on 2026-11-03 as about 142 x 142 in, inside wall face to
 * inside wall face, the same in x and y and symmetric, so the walls are at +-71 in (the manual's nominal field is 144, which gave 72). 71 is an
 * ASSUMPTION for now (user, 2026-11-03: use 71 everywhere and remember it); it is read when an opmode starts, so a Dashboard edit takes effect on the next
 * opmode start. The HIVE cell positions (HiveTarget) are measured from the field center and do not depend on it.
 */
@Config
public final class FieldGeometry {

    public static double HALF_IN = 71.0;

    private FieldGeometry() {}
}
