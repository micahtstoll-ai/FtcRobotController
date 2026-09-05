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
 * <p>The five states, in the order they usually happen:
 * <ol>
 *   <li><b>SCANNING</b>: the robot has no idea where any balls are. It
 *       spins slowly in place until the camera sees something.</li>
 *   <li><b>APPROACH_COARSE</b> (Phase 1): the robot remembers at least one
 *       cluster and drives toward the ball-weighted centroid of everything
 *       it remembers. This handles the "you saw a big pile 8 feet away,
 *       just start heading over there" case.</li>
 *   <li><b>APPROACH_FINE</b> (Phase 2): once the best cluster is close
 *       enough to trust, the robot locks onto it, points at it, and drives
 *       right at it with the intake running.</li>
 *   <li><b>INTAKING</b>: the robot is on top of the cluster; it sits still
 *       with the intake running for a fixed dwell so the intake sweeps up
 *       whatever is there. When the dwell finishes, that cluster is marked
 *       consumed in the world model.</li>
 *   <li><b>BACKOFF</b>: the robot backs up slightly so the next scan can
 *       see the spot clearly (and does not see us blocking the field of
 *       view). Then we return to SCANNING - or, if there are still known
 *       clusters, straight to APPROACH_COARSE.</li>
 * </ol>
 *
 * <p>A sixth state, <b>DONE_RUMBLE</b>, is entered when the world model has
 * been empty and the camera has seen nothing for a while. The OpMode
 * watches for that state and rumbles the controller.
 *
 * <p>State transitions have hysteresis wherever a state boundary is a
 * threshold (see {@link HuntConfig#COMMIT_DISTANCE_IN} vs.
 * {@link HuntConfig#RELEASE_DISTANCE_IN}) so the robot does not flap.
 */
public class IntakeHunter {

    public enum State {
        SCANNING,
        APPROACH_COARSE,
        APPROACH_FINE,
        INTAKING,
        BACKOFF,
        DONE_RUMBLE
    }

    private final ClusterWorldModel world;
    private final CoarseApproachStrategy coarse;

    private State state = State.SCANNING;
    private int currentTargetId = -1;
    private long stateEnteredNanos = 0L;
    private int emptyFrameStreak = 0;

    public IntakeHunter(ClusterWorldModel world, CoarseApproachStrategy coarse) {
        this.world = world;
        this.coarse = coarse;
    }

    /** Reset back to the start of the cycle. Called when the driver releases the assist trigger. */
    public void reset() {
        state = State.SCANNING;
        currentTargetId = -1;
        emptyFrameStreak = 0;
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

        // Merge new vision into the world model.
        if (HuntConfig.USE_FIELD_FRAME) {
            world.integrateFieldFrame(latestDetections, robotX, robotY, robotHeadingRad, nowNanos);
        } else {
            world.integrateBearingOnly(latestDetections, nowNanos);
        }

        // Track the "done" streak: nothing remembered AND nothing seen.
        if (world.isEmpty() && !latestDetections.hasTarget()) {
            emptyFrameStreak++;
        } else {
            emptyFrameStreak = 0;
        }

        // If we have been empty for a while, announce done and stay there
        // until the driver releases the trigger (which calls reset()).
        if (emptyFrameStreak >= HuntConfig.EMPTY_FRAMES_BEFORE_RUMBLE) {
            transitionTo(State.DONE_RUMBLE, nowNanos);
            return DriveCommand.STOP;
        }

        switch (state) {
            case SCANNING:       return runScanning(nowNanos);
            case APPROACH_COARSE:return runCoarse(robotX, robotY, robotHeadingRad, nowNanos);
            case APPROACH_FINE:  return runFine(latestDetections, robotX, robotY, robotHeadingRad, nowNanos);
            case INTAKING:       return runIntaking(nowNanos);
            case BACKOFF:        return runBackoff(robotX, robotY, robotHeadingRad, nowNanos);
            case DONE_RUMBLE:    return DriveCommand.STOP;
            default:             return DriveCommand.STOP;
        }
    }

    // --------------------------------------------------------------------
    // State handlers.
    // --------------------------------------------------------------------

    private DriveCommand runScanning(long nowNanos) {
        // Anything in memory? Immediately start Phase 1 toward it.
        if (!world.isEmpty()) {
            transitionTo(State.APPROACH_COARSE, nowNanos);
            return DriveCommand.STOP;
        }
        // Otherwise slowly rotate to look around.
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

            // Handoff: if the best cluster is close, go to Phase 2.
            ClusterWorldModel.KnownCluster best = world.bestByBallCount();
            if (best != null) {
                double dToBest = Math.hypot(best.fieldX - rx, best.fieldY - ry);
                if (dToBest <= HuntConfig.COMMIT_DISTANCE_IN) {
                    currentTargetId = best.id;
                    transitionTo(State.APPROACH_FINE, nowNanos);
                    coarse.stop();
                    return DriveCommand.STOP;
                }
            }
            return coarse.update(rx, ry, rh);
        }

        // Bearing-only fallback: turn to face the weighted bearing, then let
        // Phase 2 handle the "drive at the biggest blob" work.
        double bearing = world.weightedBearing();
        if (Double.isNaN(bearing)) {
            transitionTo(State.SCANNING, nowNanos);
            return DriveCommand.STOP;
        }
        double yaw = clamp(HuntConfig.YAW_GAIN * bearing,
                -HuntConfig.YAW_POWER_MAX, HuntConfig.YAW_POWER_MAX);
        if (Math.abs(bearing) <= HuntConfig.DRIVE_HEADING_TOLERANCE_RAD) {
            ClusterWorldModel.KnownCluster best = world.bestByBallCount();
            if (best != null) {
                currentTargetId = best.id;
                transitionTo(State.APPROACH_FINE, nowNanos);
                return DriveCommand.STOP;
            }
        }
        return new DriveCommand(0.0, 0.0, yaw, false);
    }

    private DriveCommand runFine(BallClusterResult latestDetections,
                                 double rx, double ry, double rh, long nowNanos) {

        // If we lost the cluster we were chasing, drop back to Phase 1.
        ClusterWorldModel.KnownCluster target = clusterById(currentTargetId);
        if (target == null) {
            currentTargetId = -1;
            transitionTo(State.APPROACH_COARSE, nowNanos);
            return DriveCommand.STOP;
        }

        if (HuntConfig.USE_FIELD_FRAME) {
            double dx = target.fieldX - rx;
            double dy = target.fieldY - ry;
            double distance = Math.hypot(dx, dy);

            // If we drift too far, hand back to Phase 1 (hysteresis).
            if (distance > HuntConfig.RELEASE_DISTANCE_IN) {
                transitionTo(State.APPROACH_COARSE, nowNanos);
                return DriveCommand.STOP;
            }

            // Arrival check.
            if (distance <= HuntConfig.ARRIVAL_TOLERANCE_IN) {
                transitionTo(State.INTAKING, nowNanos);
                return new DriveCommand(0, 0, 0, true);
            }

            double desiredHeading = Math.atan2(dy, dx);
            double headingError = wrapToPi(desiredHeading - rh);
            double yaw = clamp(HuntConfig.YAW_GAIN * headingError,
                    -HuntConfig.YAW_POWER_MAX, HuntConfig.YAW_POWER_MAX);
            double axial = 0.0;
            if (Math.abs(headingError) <= HuntConfig.DRIVE_HEADING_TOLERANCE_RAD) {
                axial = clamp(HuntConfig.FORWARD_GAIN * distance,
                        0.0, HuntConfig.FORWARD_POWER_MAX);
            }
            return new DriveCommand(axial, 0.0, yaw, true);
        }

        // Bearing-only Phase 2: use the freshest camera observation of the
        // best cluster - drive forward if it is centered, turn to center it.
        BallClusterResult.Cluster best = latestDetections.getBestCluster();
        if (best == null) {
            transitionTo(State.APPROACH_COARSE, nowNanos);
            return DriveCommand.STOP;
        }
        double bearing = -best.xNorm * (HuntConfig.CAMERA_HFOV_RAD / 2.0);
        double yaw = clamp(HuntConfig.YAW_GAIN * bearing,
                -HuntConfig.YAW_POWER_MAX, HuntConfig.YAW_POWER_MAX);
        double axial = 0.0;
        boolean intake = false;
        if (Math.abs(bearing) <= HuntConfig.DRIVE_HEADING_TOLERANCE_RAD) {
            axial = clamp(HuntConfig.BEARING_MODE_FORWARD_POWER * (1.0 - best.radiusNorm),
                    0.0, HuntConfig.FORWARD_POWER_MAX);
            intake = true;
            // In bearing-only mode we have no field distance, so "arrived"
            // is: the blob fills a big fraction of the frame.
            if (best.radiusNorm >= 0.35) {
                transitionTo(State.INTAKING, nowNanos);
                return new DriveCommand(0, 0, 0, true);
            }
        }
        return new DriveCommand(axial, 0.0, yaw, intake);
    }

    private DriveCommand runIntaking(long nowNanos) {
        long elapsed = nowNanos - stateEnteredNanos;
        long dwellNanos = HuntConfig.INTAKE_DWELL_MS * 1_000_000L;
        if (elapsed < dwellNanos) {
            return new DriveCommand(0, 0, 0, true);
        }
        // Dwell complete: consume the whole cluster (per design), latch a
        // backoff destination behind us, and move to BACKOFF.
        if (currentTargetId > 0) {
            world.markConsumed(currentTargetId);
            currentTargetId = -1;
        }
        transitionTo(State.BACKOFF, nowNanos);
        return new DriveCommand(-HuntConfig.FORWARD_POWER_MAX * 0.5, 0, 0, false);
    }

    private DriveCommand runBackoff(double rx, double ry, double rh, long nowNanos) {
        // Simple time-based backoff: reverse for a short window, then hand
        // control back to the main state selection. Distance-based backoff
        // would need a "backoff start pose" - not worth the extra state.
        long elapsed = nowNanos - stateEnteredNanos;
        long backoffNanos = 400L * 1_000_000L;
        if (elapsed >= backoffNanos) {
            transitionTo(world.isEmpty() ? State.SCANNING : State.APPROACH_COARSE, nowNanos);
            return DriveCommand.STOP;
        }
        return new DriveCommand(-HuntConfig.FORWARD_POWER_MAX * 0.5, 0, 0, false);
    }

    // --------------------------------------------------------------------
    // Helpers.
    // --------------------------------------------------------------------

    private void transitionTo(State next, long nowNanos) {
        if (state != next) {
            state = next;
            stateEnteredNanos = nowNanos;
        }
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
