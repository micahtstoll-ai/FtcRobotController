package org.firstinspires.ftc.teamcode.vision;

/**
 * Where the AprilTags physically sit on the field. Used by the pipeline
 * scheduler to decide whether it is worth switching to the AprilTag
 * pipeline right now - if no tag is even in the camera's field of view
 * from the robot's current pose, there is no point.
 *
 * <p>The season's game manual publishes the exact positions and heights
 * of each tag. Fill in {@link #TAGS} with those numbers before enabling
 * AprilTag corrections.
 *
 * <p>Convention: field X and Y in inches; heading angle would be needed
 * for tags mounted at an angle - not tracked here since the scheduler
 * only needs positions.
 *
 * <p>TODO(other-agent): replace the placeholder entries below with the
 * actual Biobuzz 2026-27 tag layout from the game manual.
 */
public final class AprilTagFieldMap {

    /** One tag's location on the field. */
    public static final class Tag {
        public final int id;
        public final double fieldX;
        public final double fieldY;

        public Tag(int id, double fieldX, double fieldY) {
            this.id = id;
            this.fieldX = fieldX;
            this.fieldY = fieldY;
        }
    }

    /**
     * PLACEHOLDER. Replace with the real Biobuzz 2026-27 tag positions
     * (from the season game manual) before enabling AprilTag corrections.
     *
     * <p>Layout note: FTC fields are 144 x 144 inches. The four tags below
     * are placed one per wall at the center of each wall, as a plausible
     * default. They are almost certainly wrong for your season.
     */
    public static final Tag[] TAGS = new Tag[] {
            new Tag(1,  72.0,   0.0),   // near wall midpoint
            new Tag(2, 144.0,  72.0),   // right wall midpoint
            new Tag(3,  72.0, 144.0),   // far wall midpoint
            new Tag(4,   0.0,  72.0),   // left wall midpoint
    };

    private AprilTagFieldMap() { }

    /**
     * True if any known tag lies within a heading tolerance of the robot's
     * forward. Used by the scheduler to short-circuit pipeline switches
     * that would fail to see any tag anyway.
     *
     * @param robotX             robot field X, inches
     * @param robotY             robot field Y, inches
     * @param robotHeadingRad    robot heading, radians CCW from field +X
     * @param toleranceRad       half of the camera's usable horizontal FOV
     * @return true if at least one tag is within the FOV cone
     */
    public static boolean anyTagWithinBearing(double robotX, double robotY,
                                              double robotHeadingRad,
                                              double toleranceRad) {
        for (Tag t : TAGS) {
            double dx = t.fieldX - robotX;
            double dy = t.fieldY - robotY;
            if (dx == 0 && dy == 0) continue;
            double bearingToTag = Math.atan2(dy, dx);
            double err = wrapToPi(bearingToTag - robotHeadingRad);
            if (Math.abs(err) <= toleranceRad) return true;
        }
        return false;
    }

    private static double wrapToPi(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
