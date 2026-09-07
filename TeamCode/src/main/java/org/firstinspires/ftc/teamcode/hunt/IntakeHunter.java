package org.firstinspires.ftc.teamcode.hunt;

import org.firstinspires.ftc.teamcode.vision.BallClusterResult;
import org.firstinspires.ftc.teamcode.vision.FieldLocalizer;

/**
 * The brain of the ball-hunting driver-assist mode.
 *
 * <p>The OpMode calls {@link #update} once per loop while the driver is
 * holding the assist trigger. This class decides what the robot should do
 * next and returns a {@link DriveCommand} describing that action.
 *
 * <p>The four normal states, in the order they usually happen:
 * <ol>
 *   <li><b>SCANNING</b>: the robot has no idea where any balls are. It
 *       spins slowly in place until the camera sees something.</li>
 *   <li><b>APPROACH_COARSE</b> (Phase 1): the robot remembers at least one
 *       cluster and drives toward the ball-weighted centroid of everything
 *       it remembers. Uses coupled steer + drive so it moves and rotates
 *       simultaneously.</li>
 *   <li><b>APPROACH_FINE</b> (Phase 2): once the best cluster is close
 *       enough to trust, the robot locks onto it, drives THROUGH it with
 *       the intake spinning. The intake is stationary on the front, so
 *       driving over the cluster is how balls get picked up.</li>
 *   <li><b>CONSUMED_TURNOVER</b>: one-tick state right after we pass a
 *       cluster. Removes it from the world model, resets the target, and
 *       decides where to go next (another cluster, or back to scanning).</li>
 * </ol>
 *
 * <p>A fifth state, <b>DONE_RUMBLE</b>, is entered when the world model
 * has been empty and the camera has seen nothing for a while. The OpMode
 * watches for that state and rumbles the controller.
 *
 * <p>State transitions have hysteresis at the coarse -&gt; fine boundary
 * (see {@link HuntConfig#COMMIT_DISTANCE_IN} vs.
 * {@link HuntConfig#RELEASE_DISTANCE_IN}) so the robot does not flap.
 */
public class IntakeHunter {

    public enum State {
        SCANNING,
        APPROACH_COARSE,
        APPROACH_FINE,
        CONSUMED_TURNOVER,
        DONE_RUMBLE
    }

    private final ClusterWorldModel world;
    private final CoarseApproachStrategy coarse;

    private State state = State.SCANNING;
    private int currentTargetId = -1;
    private long stateEnteredNanos = 0L;
    private int emptyFrameStreak = 0;

    // Tracking for the current APPROACH_FINE pursuit. Reset on entry.
    private double maxRadiusNormThisPursuit = 0.0;
    private int framesWithoutTargetInFine = 0;

    public IntakeHunter(ClusterWorldModel world, CoarseApproachStrategy coarse) {
        this.world = world;
        this.coarse = coarse;
    }

    /** Reset back to the start of the cycle. Called when the driver releases the assist trigger. */
    public void reset() {
        state = State.SCANNING;
        currentTargetId = -1;
        emptyFrameStreak = 0;
        maxRadiusNormThisPursuit = 0.0;
        framesWithoutTargetInFine = 0;
        coarse.stop();
    }

    public State state() { return state; }

    public int currentTargetId() { return currentTargetId; }

    /**
     * Run one iteration of the hunter.
     *
     * @param latestDetections  what the camera saw this frame
     * @param robotX            robot field X, inches
     * @param robotY            robot field Y, inches
     * @param robotHeadingRad   robot heading, radians (CCW from field +x)
     * @param nowNanos          {@code System.nanoTime()}
     * @return the {@link DriveCommand} the OpMode should apply this loop
     */
    public DriveCommand update(BallClusterResult latestDetections,
                               double robotX, double robotY, double robotHeadingRad,
                               long nowNanos) {

        if (HuntConfig.USE_FIELD_FRAME) {
            world.integrateFieldFrame(latestDetections, robotX, robotY, robotHeadingRad, nowNanos);
        } else {
            world.integrateBearingOnly(latestDetections, nowNanos);
        }

        if (world.isEmpty() && !latestDetections.hasTarget()) {
            emptyFrameStreak++;
        } else {
            emptyFrameStreak = 0;
        }

        if (emptyFrameStreak >= HuntConfig.EMPTY_FRAMES_BEFORE_RUMBLE) {
            transitionTo(State.DONE_RUMBLE, nowNanos);
            return DriveCommand.STOP;
        }

        switch (state) {
            case SCANNING:          return runScanning(nowNanos);
            case APPROACH_COARSE:   return runCoarse(robotX, robotY, robotHeadingRad, nowNanos);
            case APPROACH_FINE:     return runFine(latestDetections, robotX, robotY, robotHeadingRad, nowNanos);
            case CONSUMED_TURNOVER: return runConsumedTurnover(nowNanos);
            case DONE_RUMBLE:       return DriveCommand.STOP;
            default:                return DriveCommand.STOP;
        }
    }

    // --------------------------------------------------------------------
    // State handlers.
    // --------------------------------------------------------------------

    private DriveCommand runScanning(long nowNanos) {
        if (!world.isEmpty()) {
            transitionTo(State.APPROACH_COARSE, nowNanos);
            return DriveCommand.STOP;
        }
        return new DriveCommand(0.0, 0.0, HuntConfig.SEARCH_YAW_POWER, false);
    }

    private DriveCommand runCoarse(double rx, double ry, double rh, long nowNanos) {
        if (world.isEmpty()) {
            transitionTo(State.SCANNING, nowNanos);
            return DriveCommand.STOP;
        }

        if (HuntConfig.USE_FIELD_FRAME) {
            FieldLocalizer.FieldPosition centroid = world.weightedCentroidField();
            if (centroid == null) {
                transitionTo(State.SCANNING, nowNanos);
                return DriveCommand.STOP;
            }
            coarse.setTarget(centroid.x, centroid.y);

            ClusterWorldModel.KnownCluster best = world.bestByBallCount();
            if (best != null) {
                double dToBest = Math.hypot(best.fieldX - rx, best.fieldY - ry);
                if (dToBest <= HuntConfig.COMMIT_DISTANCE_IN) {
                    startFinePursuit(best.id, nowNanos);
                    coarse.stop();
                    return DriveCommand.STOP;
                }
            }
            return coarse.update(rx, ry, rh);
        }

        // Bearing-only fallback: coupled steer + drive toward the weighted
        // bearing, using strafe so we don't stop to turn.
        double bearing = world.weightedBearing();
        if (Double.isNaN(bearing)) {
            transitionTo(State.SCANNING, nowNanos);
            return DriveCommand.STOP;
        }

        // Once we're roughly facing the strongest signal, hand off to Fine.
        if (Math.abs(bearing) <= HuntConfig.CONSUMPTION_BEARING_PAST_RAD * 0.5) {
            ClusterWorldModel.KnownCluster best = world.bestByBallCount();
            if (best != null) {
                startFinePursuit(best.id, nowNanos);
                return DriveCommand.STOP;
            }
        }

        return bearingDrive(bearing, HuntConfig.BEARING_MODE_FORWARD_POWER, false);
    }

    private DriveCommand runFine(BallClusterResult latestDetections,
                                 double rx, double ry, double rh, long nowNanos) {

        if (HuntConfig.USE_FIELD_FRAME) {
            ClusterWorldModel.KnownCluster target = clusterById(currentTargetId);
            if (target == null) {
                currentTargetId = -1;
                transitionTo(State.APPROACH_COARSE, nowNanos);
                return DriveCommand.STOP;
            }

            // Vector from robot to target, robot-frame components.
            double dx = target.fieldX - rx;
            double dy = target.fieldY - ry;
            double distance = Math.hypot(dx, dy);
            double cosH = Math.cos(rh);
            double sinH = Math.sin(rh);
            double toRobotX =  dx * cosH + dy * sinH;   // forward-component

            // Consumption: cluster has fallen behind us by the margin.
            if (toRobotX < -HuntConfig.CONSUMPTION_PASSED_MARGIN_IN) {
                transitionTo(State.CONSUMED_TURNOVER, nowNanos);
                return DriveCommand.STOP;
            }

            // Fell too far behind to reasonably chase in fine? Back to coarse.
            if (distance > HuntConfig.RELEASE_DISTANCE_IN) {
                transitionTo(State.APPROACH_COARSE, nowNanos);
                return DriveCommand.STOP;
            }

            double toRobotY = -dx * sinH + dy * cosH;   // left-component
            double desiredHeading = Math.atan2(dy, dx);
            double headingError = wrapToPi(desiredHeading - rh);

            double yaw = clamp(HuntConfig.YAW_GAIN * headingError,
                    -HuntConfig.YAW_POWER_MAX, HuntConfig.YAW_POWER_MAX);

            double distanceSpeed = clamp(HuntConfig.FORWARD_GAIN * distance,
                    0.0, HuntConfig.FORWARD_POWER_MAX);
            double alignmentScale = Math.max(HuntConfig.MIN_ALIGNMENT_SCALE,
                    Math.cos(headingError));
            double speed = distanceSpeed * alignmentScale;

            double axial   = 0.0;
            double lateral = 0.0;
            if (distance > 1e-3) {
                axial   =  (toRobotX / distance) * speed;
                lateral = -(toRobotY / distance) * speed;
            }
            return new DriveCommand(axial, lateral, yaw, true);
        }

        // Bearing-only: use the current camera's best cluster. A cluster is
        // only "consumed" when we can be confident we drove through it, which
        // requires having gotten close enough at some point during pursuit
        // (max radiusNorm >= threshold). A momentarily-missing target is
        // treated as a transient pipeline drop and tolerated; if it stays
        // missing past a debounce and we were never close, drop back to
        // coarse rather than turning away from a target we never reached.
        BallClusterResult.Cluster best = latestDetections.getBestCluster();
        if (best == null) {
            framesWithoutTargetInFine++;
            if (framesWithoutTargetInFine >= HuntConfig.CONSUMPTION_MISSED_FRAMES) {
                if (maxRadiusNormThisPursuit >= HuntConfig.CONSUMPTION_MIN_RADIUS_NORM) {
                    transitionTo(State.CONSUMED_TURNOVER, nowNanos);
                } else {
                    transitionTo(State.APPROACH_COARSE, nowNanos);
                }
                return DriveCommand.STOP;
            }
            // Short vision dropout: coast straight forward on the last
            // known intent while the debounce counts up.
            return bearingDrive(0.0, HuntConfig.BEARING_MODE_FORWARD_POWER * 0.5, true);
        }

        framesWithoutTargetInFine = 0;
        maxRadiusNormThisPursuit = Math.max(maxRadiusNormThisPursuit, best.radiusNorm);

        double bearing = -best.xNorm * (HuntConfig.CAMERA_HFOV_RAD / 2.0);
        if (Math.abs(bearing) >= HuntConfig.CONSUMPTION_BEARING_PAST_RAD
                && maxRadiusNormThisPursuit >= HuntConfig.CONSUMPTION_MIN_RADIUS_NORM) {
            transitionTo(State.CONSUMED_TURNOVER, nowNanos);
            return DriveCommand.STOP;
        }

        // Speed scales down as the blob fills the frame (we're close).
        double speed = HuntConfig.BEARING_MODE_FORWARD_POWER * (1.0 - best.radiusNorm);
        return bearingDrive(bearing, speed, true);
    }

    private DriveCommand runConsumedTurnover(long nowNanos) {
        // One-tick housekeeping: remove the consumed cluster and pick the
        // next state based on what's left in the world model.
        if (currentTargetId > 0) {
            world.markConsumed(currentTargetId);
            currentTargetId = -1;
        }
        transitionTo(world.isEmpty() ? State.SCANNING : State.APPROACH_COARSE, nowNanos);
        return DriveCommand.STOP;
    }

    // --------------------------------------------------------------------
    // Helpers.
    // --------------------------------------------------------------------

    /**
     * Coupled steer + drive using only a bearing (no distance): drive in the
     * direction the bearing points, using mecanum strafe, while rotating to
     * center the target. The speed argument is the base translational speed
     * (already scaled by radius or similar caller logic).
     */
    private DriveCommand bearingDrive(double bearing, double speed, boolean intakeOn) {
        // Bearing is measured CCW from robot-forward. A bearing of 0 means
        // straight ahead; +pi/2 is to the robot's LEFT.
        double yaw = clamp(HuntConfig.YAW_GAIN * bearing,
                -HuntConfig.YAW_POWER_MAX, HuntConfig.YAW_POWER_MAX);
        double alignmentScale = Math.max(HuntConfig.MIN_ALIGNMENT_SCALE,
                Math.cos(bearing));
        double v = clamp(speed, 0.0, HuntConfig.FORWARD_POWER_MAX) * alignmentScale;
        double axial   = Math.cos(bearing) * v;
        double lateral = -Math.sin(bearing) * v;   // +left in robot frame -> -right for DriveCommand
        return new DriveCommand(axial, lateral, yaw, intakeOn);
    }

    private void transitionTo(State next, long nowNanos) {
        if (state != next) {
            state = next;
            stateEnteredNanos = nowNanos;
        }
    }

    /**
     * Lock onto a target and enter APPROACH_FINE with fresh pursuit
     * tracking. Both hand-off points from APPROACH_COARSE go through this
     * so the consumption debounce and proximity gate always start clean.
     */
    private void startFinePursuit(int targetId, long nowNanos) {
        currentTargetId = targetId;
        maxRadiusNormThisPursuit = 0.0;
        framesWithoutTargetInFine = 0;
        transitionTo(State.APPROACH_FINE, nowNanos);
    }

    private ClusterWorldModel.KnownCluster clusterById(int id) {
        if (id <= 0) return null;
        for (ClusterWorldModel.KnownCluster k : world.clusters()) {
            if (k.id == id) return k;
        }
        return null;
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
