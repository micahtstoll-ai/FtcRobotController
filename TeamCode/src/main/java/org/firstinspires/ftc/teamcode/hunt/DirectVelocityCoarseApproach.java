package org.firstinspires.ftc.teamcode.hunt;

/**
 * Simple Phase 1 strategy: point the robot at the target, drive forward.
 *
 * <p>No path planning, no follower, no library. Every loop we look at where
 * the robot is, where the target is, and command turning power proportional
 * to the heading error plus forward power proportional to the distance
 * remaining. Gains and caps live in {@link HuntConfig}.
 *
 * <p>The robot will not drive forward until it is roughly pointed at the
 * target (heading error within {@link HuntConfig#DRIVE_HEADING_TOLERANCE_RAD}).
 * That is what keeps it from cutting corners across the field.
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

        double desiredHeading = Math.atan2(dy, dx);
        double headingError = wrapToPi(desiredHeading - robotHeadingRad);

        double yaw = clamp(HuntConfig.YAW_GAIN * headingError,
                -HuntConfig.YAW_POWER_MAX, HuntConfig.YAW_POWER_MAX);

        double axial = 0.0;
        if (Math.abs(headingError) <= HuntConfig.DRIVE_HEADING_TOLERANCE_RAD) {
            axial = clamp(HuntConfig.FORWARD_GAIN * distance,
                    0.0, HuntConfig.FORWARD_POWER_MAX);
        }

        return new DriveCommand(axial, 0.0, yaw, false);
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
