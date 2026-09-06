package org.firstinspires.ftc.teamcode.hunt;

/**
 * How the robot moves during Phase 1 - the "get close to the pile" phase.
 *
 * <p>The point of this interface is that Phase 1 can be swapped out without
 * touching the rest of the hunter. There are two obvious ways to do it:
 * <ul>
 *   <li>{@link DirectVelocityCoarseApproach} - simple proportional control
 *       toward the target. Reactive, easy to reason about, no path library
 *       required. Ships as the default.</li>
 *   <li>A Pedro Pathing implementation, once Pedro is stable in this repo.
 *       Would generate a fresh PathChain to the current target on a slow
 *       cadence (every ~500ms) and let Pedro's follower drive it. Not yet
 *       written - drop a new file next to this one, implement this
 *       interface, and swap it in the OpMode.</li>
 * </ul>
 */
public interface CoarseApproachStrategy {

    /** Point the strategy at a new target (field-frame, inches). */
    void setTarget(double fieldX, double fieldY);

    /**
     * Ask for the next drive command given the current robot pose.
     *
     * @return a {@link DriveCommand} in the robot's own frame.
     *         Intake is always off during Phase 1.
     */
    DriveCommand update(double robotX, double robotY, double robotHeadingRad);

    /**
     * Distance in inches from the robot to the current target. Used by the
     * hunter to decide when to hand off from Phase 1 to Phase 2.
     */
    double distanceToTarget(double robotX, double robotY);

    /** Cancel whatever the strategy was doing (e.g. abort a follower). */
    void stop();
}
