package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A friendly wrapper around the Limelight3A camera.
 *
 * <p>The camera itself is running the ball-cluster Python pipeline. This
 * class just handles the small amount of ceremony to talk to it:
 * <ul>
 *   <li>Find the camera in the robot config (by name)</li>
 *   <li>Pick the right pipeline number</li>
 *   <li>Start it streaming so results are always fresh</li>
 *   <li>Give back the most recent cluster reading, decoded</li>
 * </ul>
 *
 * <p>Every call to {@link #latest()} returns a non-null {@link BallClusterResult}.
 * If the camera has not seen anything, that result reports zero clusters -
 * callers do not have to null-check.
 */
public class LimelightClusterCamera {

    private final Limelight3A limelight;
    private boolean started = false;

    /**
     * @param hardwareMap  the OpMode's hardware map
     * @param configName   the Limelight's name in the robot configuration
     *                     (matches the FTC sample: {@code "limelight"})
     * @param pipelineId   which pipeline slot the ball-cluster script lives in
     *                     (Limelight pipelines are numbered 0..9)
     */
    public LimelightClusterCamera(HardwareMap hardwareMap, String configName, int pipelineId) {
        this.limelight = hardwareMap.get(Limelight3A.class, configName);
        this.limelight.pipelineSwitch(pipelineId);
    }

    /** Begin streaming from the camera. Call once in init or before the main loop. */
    public void start() {
        if (!started) {
            limelight.start();
            started = true;
        }
    }

    /** Stop streaming. Call in OpMode stop, or leave it and let the camera clean up on its own. */
    public void stop() {
        if (started) {
            limelight.stop();
            started = false;
        }
    }

    /**
     * The latest cluster reading. Never null; if the camera has not produced a
     * result yet, or the newest result carries no Python payload, this returns
     * an empty {@link BallClusterResult}.
     */
    public BallClusterResult latest() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return BallClusterResult.parse(null);
        }
        return BallClusterResult.parse(result.getPythonOutput());
    }

    /** The underlying Limelight3A object, if you need something we did not wrap. */
    public Limelight3A raw() {
        return limelight;
    }
}
