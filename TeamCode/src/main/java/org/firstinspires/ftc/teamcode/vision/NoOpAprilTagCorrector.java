package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * The placeholder {@link AprilTagCorrector} that ships today. Does
 * nothing on every call, records no fixes, applies no corrections.
 *
 * <p>Replace this with a real implementation once the AprilTag pipeline
 * is set up on the Limelight web UI and the field tag positions are
 * filled into {@link AprilTagFieldMap}. Keeping this class around after
 * the real one lands is fine - it can be used for OpModes that
 * explicitly want to run without AprilTag corrections.
 */
public class NoOpAprilTagCorrector implements AprilTagCorrector {

    @Override
    public boolean tryApplyCorrection(Limelight3A camera,
                                      GoBildaPinpointDriver pinpoint,
                                      long nowNanos) {
        return false;
    }

    @Override
    public AprilTagFix lastFix() {
        return null;
    }
}
