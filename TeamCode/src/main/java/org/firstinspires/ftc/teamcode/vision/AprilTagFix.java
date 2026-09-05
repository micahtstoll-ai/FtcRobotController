package org.firstinspires.ftc.teamcode.vision;

/**
 * A single AprilTag-based pose reading: "the robot is at this field
 * position, from this tag, at this moment."
 *
 * <p>This is a value type: once built it cannot change. The
 * {@link AprilTagCorrector} produces these when it has a fresh valid
 * reading. The consumer (usually an OpMode) applies each one to Pinpoint
 * as a hard reset - AprilTag readings always take priority over
 * accumulated odometry.
 *
 * <p>See {@link AprilTagFieldMap} for the known tag positions this pose
 * is computed against.
 */
public final class AprilTagFix {

    /** Robot field X in inches. */
    public final double fieldX;

    /** Robot field Y in inches. */
    public final double fieldY;

    /** Robot heading in radians, CCW from field +X. */
    public final double headingRad;

    /** When we observed this fix (nanoseconds, System.nanoTime). */
    public final long timestampNanos;

    /** Which tag id the pose was derived from. */
    public final int tagId;

    /**
     * How confident the vision layer is in this reading, 0 to 1. A default
     * corrector may want to reject fixes below some threshold; the
     * scaffold shipped here does not enforce that.
     */
    public final double confidence;

    public AprilTagFix(double fieldX, double fieldY, double headingRad,
                       long timestampNanos, int tagId, double confidence) {
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.headingRad = headingRad;
        this.timestampNanos = timestampNanos;
        this.tagId = tagId;
        this.confidence = confidence;
    }

    @Override
    public String toString() {
        return String.format("AprilTagFix[tag=%d x=%.1f y=%.1f h=%.1f conf=%.2f]",
                tagId, fieldX, fieldY, Math.toDegrees(headingRad), confidence);
    }
}
