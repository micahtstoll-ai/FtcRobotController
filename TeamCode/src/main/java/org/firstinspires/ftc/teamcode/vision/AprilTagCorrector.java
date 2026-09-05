package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * Reads AprilTag data from the Limelight and, when it has a fresh valid
 * fix, hard-resets Pinpoint to the tag's estimated pose.
 *
 * <p>Policy (set by the driver): AprilTag readings always win. When a
 * valid fix is available, the implementation calls
 * {@code pinpoint.setPosition(...)} with the tag's derived pose. No
 * blending, no threshold, no smoothing. If the tag says the robot is
 * somewhere, the robot is there now.
 *
 * <p>Implementations must:
 * <ol>
 *   <li>Read whatever tag data the Limelight exposes when its AprilTag
 *       pipeline is active.</li>
 *   <li>Combine that with the known tag positions in
 *       {@link AprilTagFieldMap} to compute a field-frame robot pose.</li>
 *   <li>Reject readings that are stale, low confidence, or geometrically
 *       impossible (behind a wall, etc).</li>
 *   <li>Hard-reset Pinpoint on any surviving reading.</li>
 * </ol>
 *
 * <p>None of that is implemented in the scaffold. See
 * {@link NoOpAprilTagCorrector} for the placeholder that ships today.
 */
public interface AprilTagCorrector {

    /**
     * Called by the OpMode when the pipeline scheduler says the AprilTag
     * pipeline is active and enough time has elapsed for a clean reading.
     * Reads the camera, decides if there is a valid fix, and if so applies
     * it to Pinpoint via {@code setPosition(...)}.
     *
     * @return {@code true} if a fix was applied this call, {@code false} otherwise.
     */
    boolean tryApplyCorrection(Limelight3A camera,
                               GoBildaPinpointDriver pinpoint,
                               long nowNanos);

    /**
     * The most recent successful fix, for telemetry. Returns {@code null}
     * if no fix has ever been applied.
     */
    AprilTagFix lastFix();
}
