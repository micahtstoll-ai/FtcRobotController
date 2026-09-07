package org.firstinspires.ftc.teamcode.vision;

/**
 * Decides when the Limelight should switch from the ball-cluster pipeline
 * to the AprilTag pipeline so the robot can correct its Pinpoint pose.
 *
 * <p>The scheduler switches to AprilTag mode when ALL of these are true:
 * <ol>
 *   <li>The robot has moved at least {@link #TRIGGER_DISTANCE_IN} inches
 *       since the last successful correction (or ever, for the first
 *       correction). Drift accumulates with motion, not time - a robot
 *       sitting still does not drift.</li>
 *   <li>The next hunt target is at least
 *       {@link #MIN_TARGET_DISTANCE_FOR_SCAN_IN} inches away, so there is
 *       enough transit time to spend a pipeline switch on the tag scan
 *       without wrecking the hunt. Close targets stay on clusters.</li>
 *   <li>At least one known tag is within
 *       {@link #FOV_TOLERANCE_RAD} of robot-forward, per
 *       {@link AprilTagFieldMap#anyTagWithinBearing}. No point switching
 *       to see a tag that is not there.</li>
 * </ol>
 *
 * <p>The OpMode owns the actual {@code pipelineSwitch(...)} call on the
 * Limelight; this class only decides. Pattern:
 * <pre>
 *     int desired = scheduler.desiredPipeline(pose, distanceToTarget);
 *     if (desired != camera.raw().getPipelineIndex()) {
 *         camera.raw().pipelineSwitch(desired);
 *     }
 *     if (desired == APRILTAG_PIPELINE) {
 *         if (corrector.tryApplyCorrection(camera.raw(), pinpoint, nowNanos)) {
 *             scheduler.markCorrectionApplied(pose.getX(...), pose.getY(...));
 *         }
 *     }
 * </pre>
 *
 * <p>This class does not itself switch pipelines or read tag data. Those
 * responsibilities live with the OpMode and {@link AprilTagCorrector}.
 */
public class LimelightPipelineScheduler {

    /**
     * Cluster (ball) pipeline. Currently in slot 1 (slot 0 is a leftover
     * from last season that nobody has deleted). Must match
     * {@code HuntConfig.LIMELIGHT_PIPELINE_ID}.
     */
    public static final int CLUSTER_PIPELINE_ID = 1;

    /**
     * AprilTag pipeline. TODO(other-agent): set up on the Limelight web UI
     * in slot 2 (or change this constant to whichever slot you use).
     */
    public static final int APRILTAG_PIPELINE_ID = 2;

    /**
     * Move at least this many inches since the last correction before we
     * bother trying to switch pipelines. Roughly the distance over which
     * Pinpoint drift becomes worth spending vision time to correct.
     */
    public static final double TRIGGER_DISTANCE_IN = 24.0;

    /**
     * Only consider a pipeline switch when the current hunt target is at
     * least this far away. Below this, the hunt is in its final approach
     * and losing camera time to a pipeline swap would be expensive.
     */
    public static final double MIN_TARGET_DISTANCE_FOR_SCAN_IN = 48.0;

    /**
     * Half of the Limelight's usable horizontal FOV. Tolerance for
     * "a tag is in view." Limelight3A HFOV is about 63 degrees, so the
     * half-cone is about 31 degrees.
     */
    public static final double FOV_TOLERANCE_RAD = Math.toRadians(31.0);

    private double lastCorrectionX = 0.0;
    private double lastCorrectionY = 0.0;
    private boolean everCorrected = false;

    /**
     * Which pipeline id the caller SHOULD have active this loop. The
     * caller does the {@code pipelineSwitch(...)} if the value changed.
     *
     * @param robotX                robot field X, inches
     * @param robotY                robot field Y, inches
     * @param headingRad            robot heading, radians CCW from field +X
     * @param distanceToNextTarget  inches to next hunt target, or
     *                              {@link Double#POSITIVE_INFINITY} when
     *                              nothing is targeted
     * @param busyWithCloseTarget   {@code true} if the hunter is in an
     *                              inner-loop state that cannot afford a
     *                              pipeline switch (e.g. APPROACH_FINE)
     */
    public int desiredPipeline(double robotX, double robotY, double headingRad,
                               double distanceToNextTarget,
                               boolean busyWithCloseTarget) {
        if (busyWithCloseTarget) return CLUSTER_PIPELINE_ID;

        double distanceSinceCorrection = everCorrected
                ? Math.hypot(robotX - lastCorrectionX, robotY - lastCorrectionY)
                : Double.POSITIVE_INFINITY;

        if (distanceSinceCorrection < TRIGGER_DISTANCE_IN) return CLUSTER_PIPELINE_ID;
        if (distanceToNextTarget < MIN_TARGET_DISTANCE_FOR_SCAN_IN) return CLUSTER_PIPELINE_ID;
        if (!AprilTagFieldMap.anyTagWithinBearing(robotX, robotY, headingRad,
                FOV_TOLERANCE_RAD)) return CLUSTER_PIPELINE_ID;

        return APRILTAG_PIPELINE_ID;
    }

    /**
     * Tell the scheduler that a correction was just applied. Resets the
     * drift counter so the next correction cannot fire until the robot
     * moves another {@link #TRIGGER_DISTANCE_IN} inches.
     */
    public void markCorrectionApplied(double robotX, double robotY) {
        this.lastCorrectionX = robotX;
        this.lastCorrectionY = robotY;
        this.everCorrected = true;
    }

    /** True once at least one correction has been applied. Telemetry only. */
    public boolean hasEverCorrected() {
        return everCorrected;
    }
}
