package org.firstinspires.ftc.teamcode.hunt;

/**
 * Simple Phase 1 strategy: drive toward the target while rotating to face
 * it - both at the same time.
 *
 * <p>Uses coupled steer + drive on a mecanum drivetrain. Every loop we:
 * <ul>
 *   <li>Compute the vector from robot to target in the robot's own frame</li>
 *   <li>Command axial + lateral wheels to move ALONG that vector - this
 *       uses mecanum strafe, so the robot moves in a straight line toward
 *       the target regardless of which way it is currently facing</li>
 *   <li>Independently command yaw to rotate toward the target</li>
 *   <li>Scale the translational speed by cos(headingError) so a badly
 *       misaligned robot naturally slows down. A small floor
 *       ({@link HuntConfig#MIN_ALIGNMENT_SCALE}) lets it still creep in
 *       while yaw catches up rather than stalling completely</li>
 * </ul>
 *
 * <p>The robot never stops-and-turns. It always moves and always rotates.
 * All three degrees of freedom - axial, lateral, yaw - are commanded
 * every loop until the strategy is told to stop.
 */
public class DirectVelocityCoarseApproach implements CoarseApproachStrategy {

    private double targetX;
    private double targetY;
    private boolean hasTarget = false;

    @Override
    public void setTarget(double fieldX, double fieldY) {
        this.targetX = fieldX;
        this.targetY = fieldY;
        this.hasTarget = true;
    }

    @Override
    public DriveCommand update(double robotX, double robotY, double robotHeadingRad) {
        if (!hasTarget) return DriveCommand.STOP;

        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double distance = Math.hypot(dx, dy);

        // Vector to target in the robot's own frame.
        // In the robot frame: +x is forward, +y is to the robot's left.
        // Rotate the field vector by -heading.
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);
        double toRobotX =  dx * cosH + dy * sinH;   // forward-component
        double toRobotY = -dx * sinH + dy * cosH;   // left-component

        // Desired heading points from robot to target in the FIELD frame.
        double desiredHeading = Math.atan2(dy, dx);
        double headingError = wrapToPi(desiredHeading - robotHeadingRad);

        // Yaw command: P controller toward zero heading error.
        double yaw = clamp(HuntConfig.YAW_GAIN * headingError,
                -HuntConfig.YAW_POWER_MAX, HuntConfig.YAW_POWER_MAX);

        // Translational speed: scaled by distance-P, capped, then scaled
        // again by an alignment factor so bad heading naturally slows us.
        double distanceSpeed = clamp(HuntConfig.FORWARD_GAIN * distance,
                0.0, HuntConfig.FORWARD_POWER_MAX);
        double alignmentScale = Math.max(HuntConfig.MIN_ALIGNMENT_SCALE,
                Math.cos(headingError));
        double speed = distanceSpeed * alignmentScale;

        // Direction is the unit vector toward the target (robot frame).
        // DriveCommand's lateral convention is +right, so flip y-left.
        double axial   = 0.0;
        double lateral = 0.0;
        if (distance > 1e-3) {
            axial   =  (toRobotX / distance) * speed;
            lateral = -(toRobotY / distance) * speed;
        }

        return new DriveCommand(axial, lateral, yaw, false);
    }

    @Override
    public double distanceToTarget(double robotX, double robotY) {
        if (!hasTarget) return Double.POSITIVE_INFINITY;
        return Math.hypot(targetX - robotX, targetY - robotY);
    }

    @Override
    public void stop() {
        hasTarget = false;
    }

    private static double wrapToPi(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
