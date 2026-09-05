package org.firstinspires.ftc.teamcode.hunt;

import org.firstinspires.ftc.teamcode.vision.BallClusterResult;
import org.firstinspires.ftc.teamcode.vision.FieldLocalizer;

import java.util.ArrayList;
import java.util.List;

/**
 * The robot's memory of where balls are on the field.
 *
 * <p>The camera only sees what is in front of it right now. This class holds
 * on to everything it has seen recently, so the hunter can keep chasing
 * balls even when the camera briefly loses sight, and so the hunter knows
 * "we cleared that pile already, do not go back."
 *
 * <p>Each remembered cluster lives in one of two coordinate systems:
 * <ul>
 *   <li><b>Field-frame mode</b> (preferred): each cluster has a real field
 *       position in inches. New camera detections are matched to remembered
 *       clusters by how close they are on the field.</li>
 *   <li><b>Bearing-only mode</b> (fallback): each cluster is stored as an
 *       angle relative to the robot's heading. Used when the Limelight is
 *       not yet distance-calibrated. The world model is much simpler and
 *       shorter-lived in this mode.</li>
 * </ul>
 *
 * <p>Which mode is used is decided by the caller (the OpMode) based on
 * {@link HuntConfig#USE_FIELD_FRAME} and whether individual detections
 * actually carry a distance.
 */
public class ClusterWorldModel {

    /** One remembered cluster. */
    public static class KnownCluster {
        public final int id;
        /** Field X in inches (field-frame mode), or 0 in bearing-only mode. */
        public double fieldX;
        /** Field Y in inches (field-frame mode), or 0 in bearing-only mode. */
        public double fieldY;
        /**
         * Bearing to this cluster in radians, measured from the robot's
         * heading at the moment we saw it (bearing-only mode). Unused in
         * field-frame mode.
         */
        public double bearingRad;
        /** How many balls we estimate are in this cluster right now (smoothed). */
        public double estimatedBalls;
        /** The most recent apparent radius in the image [0..1]. */
        public double radiusNorm;
        /** The most recent distance reading in inches, or 0 if unknown. */
        public double distanceInches;
        /** Nanoseconds (System.nanoTime) when we last matched a fresh detection to this cluster. */
        public long lastSeenNanos;

        KnownCluster(int id) { this.id = id; }
    }

    private final List<KnownCluster> clusters = new ArrayList<>();
    private int nextId = 1;

    /** All clusters currently remembered, in insertion order. */
    public List<KnownCluster> clusters() {
        return clusters;
    }

    /** True if we do not remember any clusters right now. */
    public boolean isEmpty() {
        return clusters.isEmpty();
    }

    // --------------------------------------------------------------------
    // Field-frame updates.
    // --------------------------------------------------------------------

    /**
     * Fold a new frame of camera detections into the field-frame world model.
     *
     * @param detections   fresh clusters straight from the camera
     * @param robotX       robot field X, inches
     * @param robotY       robot field Y, inches
     * @param robotHeading robot heading, radians (CCW from field +x)
     * @param nowNanos     current time in nanoseconds ({@code System.nanoTime()})
     */
    public void integrateFieldFrame(BallClusterResult detections,
                                    double robotX, double robotY, double robotHeading,
                                    long nowNanos) {
        for (BallClusterResult.Cluster c : detections.getClusters()) {
            FieldLocalizer.FieldPosition pos = FieldLocalizer.estimate(
                    c, HuntConfig.CAMERA_HFOV_RAD,
                    HuntConfig.CAMERA_FORWARD_IN, HuntConfig.CAMERA_LEFT_IN,
                    HuntConfig.CAMERA_YAW_RAD,
                    robotX, robotY, robotHeading);
            if (pos == null) {
                continue;
            }
            KnownCluster match = nearestFieldMatch(pos.x, pos.y,
                    HuntConfig.WORLD_MATCH_TOLERANCE_IN);
            if (match == null) {
                match = newCluster();
                match.fieldX = pos.x;
                match.fieldY = pos.y;
                match.estimatedBalls = c.estimatedBalls;
            } else {
                double a = HuntConfig.BALL_COUNT_EMA_ALPHA;
                match.fieldX = (1 - a) * match.fieldX + a * pos.x;
                match.fieldY = (1 - a) * match.fieldY + a * pos.y;
                match.estimatedBalls = (1 - a) * match.estimatedBalls + a * c.estimatedBalls;
            }
            match.radiusNorm = c.radiusNorm;
            match.distanceInches = c.distanceInches;
            match.lastSeenNanos = nowNanos;
        }
        ageOut(nowNanos);
    }

    // --------------------------------------------------------------------
    // Bearing-only updates (used before the camera is distance-calibrated).
    // --------------------------------------------------------------------

    /**
     * Fold a new frame of camera detections into a bearing-only world model.
     *
     * <p>Because bearings become meaningless the moment the robot turns, this
     * model is refreshed hard each frame: everything old is thrown out and
     * replaced with what the camera sees right now. There is no long-term
     * memory in bearing-only mode - it is a snapshot.
     *
     * @param detections   fresh clusters straight from the camera
     * @param nowNanos     current time in nanoseconds
     */
    public void integrateBearingOnly(BallClusterResult detections, long nowNanos) {
        clusters.clear();
        for (BallClusterResult.Cluster c : detections.getClusters()) {
            KnownCluster k = newCluster();
            k.bearingRad = -c.xNorm * (HuntConfig.CAMERA_HFOV_RAD / 2.0);
            k.estimatedBalls = c.estimatedBalls;
            k.radiusNorm = c.radiusNorm;
            k.distanceInches = c.distanceInches;
            k.lastSeenNanos = nowNanos;
        }
    }

    // --------------------------------------------------------------------
    // Consumption + queries.
    // --------------------------------------------------------------------

    /**
     * Remove a cluster from the world model. Called after the intake dwell
     * finishes, because per the design decision we treat the whole cluster
     * as consumed. If the intake missed some balls, the next scan that sees
     * that spot will re-populate the model with what is left.
     */
    public void markConsumed(int id) {
        clusters.removeIf(k -> k.id == id);
    }

    /** The remembered cluster with the largest estimated ball count, or null. */
    public KnownCluster bestByBallCount() {
        KnownCluster best = null;
        for (KnownCluster k : clusters) {
            if (best == null || k.estimatedBalls > best.estimatedBalls) {
                best = k;
            }
        }
        return best;
    }

    /**
     * Ball-count-weighted centroid of every remembered cluster in field-frame
     * mode. Returns {@code null} if the model is empty.
     */
    public FieldLocalizer.FieldPosition weightedCentroidField() {
        if (clusters.isEmpty()) return null;
        double sumWx = 0, sumWy = 0, sumW = 0;
        for (KnownCluster k : clusters) {
            double w = Math.max(0.1, k.estimatedBalls);
            sumWx += w * k.fieldX;
            sumWy += w * k.fieldY;
            sumW += w;
        }
        return new FieldLocalizer.FieldPosition(sumWx / sumW, sumWy / sumW);
    }

    /**
     * Ball-count-weighted bearing (radians) in bearing-only mode. Returns
     * {@link Double#NaN} if the model is empty.
     */
    public double weightedBearing() {
        if (clusters.isEmpty()) return Double.NaN;
        double sumWsin = 0, sumWcos = 0;
        for (KnownCluster k : clusters) {
            double w = Math.max(0.1, k.estimatedBalls);
            sumWsin += w * Math.sin(k.bearingRad);
            sumWcos += w * Math.cos(k.bearingRad);
        }
        return Math.atan2(sumWsin, sumWcos);
    }

    // --------------------------------------------------------------------
    // Internals.
    // --------------------------------------------------------------------

    private KnownCluster newCluster() {
        KnownCluster k = new KnownCluster(nextId++);
        clusters.add(k);
        return k;
    }

    private KnownCluster nearestFieldMatch(double x, double y, double toleranceIn) {
        KnownCluster best = null;
        double bestD2 = toleranceIn * toleranceIn;
        for (KnownCluster k : clusters) {
            double dx = k.fieldX - x;
            double dy = k.fieldY - y;
            double d2 = dx * dx + dy * dy;
            if (d2 <= bestD2) {
                best = k;
                bestD2 = d2;
            }
        }
        return best;
    }

    private void ageOut(long nowNanos) {
        long maxAgeNanos = (long) (HuntConfig.CLUSTER_STALE_AFTER_S * 1_000_000_000L);
        clusters.removeIf(k -> (nowNanos - k.lastSeenNanos) > maxAgeNanos);
    }
}
