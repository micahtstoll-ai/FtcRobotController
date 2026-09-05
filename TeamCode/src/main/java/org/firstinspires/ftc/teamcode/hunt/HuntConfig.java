package org.firstinspires.ftc.teamcode.hunt;

/**
 * Every tunable number the ball-hunter uses, in one place.
 *
 * <p>Nothing here does anything on its own. The other hunt classes read
 * these values so a driver or coach can adjust the robot's behavior by
 * editing one file, without touching the algorithm code.
 *
 * <p>The numbers are grouped by what they control. Each group has a short
 * note explaining what happens if you make it bigger or smaller.
 *
 * <p>All units are inches, seconds, and radians unless otherwise noted.
 */
public final class HuntConfig {

    private HuntConfig() { }

    // ---------------------------------------------------------------------
    // Robot configuration names (must match the Driver Station config).
    // ---------------------------------------------------------------------

    /** Limelight camera name in the robot config. */
    public static final String LIMELIGHT_NAME = "limelight";

    /** Which pipeline slot on the Limelight holds the ball-cluster script. */
    public static final int LIMELIGHT_PIPELINE_ID = 0;

    /** Intake motor name in the robot config. */
    public static final String INTAKE_MOTOR_NAME = "intake";

    // ---------------------------------------------------------------------
    // Camera geometry.
    //
    // TODO(micah): measure and replace these after mounting the Limelight.
    // Until they are correct, FieldLocalizer's output will be off in the
    // field frame - the hunter falls back to bearing-only pursuit in that
    // case (see USE_FIELD_FRAME below), so the code still runs.
    // ---------------------------------------------------------------------

    /** Limelight3A horizontal field of view (radians). Datasheet-typical value. */
    public static final double CAMERA_HFOV_RAD = Math.toRadians(63.0);

    /** Camera position forward of the robot's rotation center (inches). */
    public static final double CAMERA_FORWARD_IN = 0.0;

    /** Camera position to the LEFT of the robot's rotation center (inches). */
    public static final double CAMERA_LEFT_IN = 0.0;

    /** Camera yaw relative to robot-forward (radians, CCW positive). */
    public static final double CAMERA_YAW_RAD = 0.0;

    /**
     * If true, use full field-frame localization (needs the camera geometry
     * above AND distance calibration on the Limelight). If false, the hunter
     * runs in "bearing-only" mode: it just turns to keep the best cluster
     * centered and drives forward - no field math required. Start in
     * bearing-only mode, flip this on after calibration.
     */
    public static final boolean USE_FIELD_FRAME = false;

    // ---------------------------------------------------------------------
    // Phase 1 -> Phase 2 handoff.
    //
    // Phase 1 (coarse): drive toward the ball-weighted centroid of every
    //                   cluster the robot knows about.
    // Phase 2 (fine):   lock onto the single best cluster and drive at it.
    //
    // Hysteresis: we commit to phase 2 when the target is inside COMMIT_IN,
    // and only fall back to phase 1 once it drifts past RELEASE_IN. Without
    // the gap the robot would flap between the two states at the boundary.
    // ---------------------------------------------------------------------

    /** Distance below which we lock onto a single cluster (inches). */
    public static final double COMMIT_DISTANCE_IN = 24.0;

    /** Distance above which we fall back to the weighted centroid (inches). */
    public static final double RELEASE_DISTANCE_IN = 30.0;

    // ---------------------------------------------------------------------
    // Drive control gains (used in phase 2 and in bearing-only mode).
    // ---------------------------------------------------------------------

    /** Multiplier on heading error (radians -> yaw power). */
    public static final double YAW_GAIN = 1.4;

    /** Cap on yaw power the hunter will command. */
    public static final double YAW_POWER_MAX = 0.6;

    /** Multiplier on forward distance error (inches -> forward power). */
    public static final double FORWARD_GAIN = 0.04;

    /** Cap on forward power the hunter will command. */
    public static final double FORWARD_POWER_MAX = 0.6;

    /**
     * In bearing-only mode, forward power scales with how big the cluster
     * looks in frame (bigger blob = closer = slow down). Multiplied by
     * (1 - radiusNorm) then clipped by {@link #FORWARD_POWER_MAX}.
     */
    public static final double BEARING_MODE_FORWARD_POWER = 0.5;

    /** Only drive forward once heading error is at most this large (radians). */
    public static final double DRIVE_HEADING_TOLERANCE_RAD = Math.toRadians(15.0);

    /** How close to a cluster the robot has to be to call it "arrived" (inches). */
    public static final double ARRIVAL_TOLERANCE_IN = 4.0;

    // ---------------------------------------------------------------------
    // Intaking.
    // ---------------------------------------------------------------------

    /** How long to sit at a cluster with the intake running (milliseconds). */
    public static final long INTAKE_DWELL_MS = 1500L;

    /** How far to back away after a dwell, so the next scan sees fresh geometry (inches). */
    public static final double POST_INTAKE_BACKOFF_IN = 6.0;

    // ---------------------------------------------------------------------
    // World model - how the robot remembers where clusters are.
    // ---------------------------------------------------------------------

    /**
     * How close (inches) two field-frame observations have to be to count as
     * the same cluster. Too small: one cluster spawns duplicates as noise
     * jitters. Too big: two nearby real piles get merged.
     */
    public static final double WORLD_MATCH_TOLERANCE_IN = 8.0;

    /**
     * Smoothing on cluster ball counts. New reading contributes this fraction,
     * old estimate keeps (1 - this). 0.3 gives a gentle low-pass.
     */
    public static final double BALL_COUNT_EMA_ALPHA = 0.3;

    /**
     * Drop a remembered cluster after this many seconds without seeing it
     * again. Should be long enough to cover looking away and back, short
     * enough that a ball someone else picked up eventually disappears.
     */
    public static final double CLUSTER_STALE_AFTER_S = 5.0;

    // ---------------------------------------------------------------------
    // Search behavior when nothing is visible.
    // ---------------------------------------------------------------------

    /** Yaw power to use while spinning in place looking for clusters. */
    public static final double SEARCH_YAW_POWER = 0.35;

    /**
     * How many consecutive scan frames with an empty world model AND no
     * detections before the OpMode rumbles the controller. At ~50Hz loop,
     * 100 frames is about 2 seconds.
     */
    public static final int EMPTY_FRAMES_BEFORE_RUMBLE = 100;

    // ---------------------------------------------------------------------
    // Driver stick blend.
    // ---------------------------------------------------------------------

    /** Stick magnitude above which the driver's input overrides the hunter. */
    public static final double STICK_OVERRIDE_DEADZONE = 0.15;
}
