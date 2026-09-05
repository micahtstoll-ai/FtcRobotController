package org.firstinspires.ftc.teamcode.hunt;

/**
 * Rule for combining driver stick input with the hunter's commands.
 *
 * <p>Design: if the driver moves a stick past the deadzone, THAT axis is
 * theirs; the hunter loses that axis for the loop. If the driver is not
 * touching a stick, the hunter drives that axis. This is "assist" as
 * opposed to "hard override": the hunter can keep driving one direction
 * while the driver overrides another.
 *
 * <p>Concretely:
 * <ul>
 *   <li>Driver pushes forward stick: the driver drives forward/back, the
 *       hunter can still turn.</li>
 *   <li>Driver twists yaw stick: the driver turns, the hunter can still
 *       drive forward.</li>
 *   <li>Driver lets go: hunter drives everything.</li>
 * </ul>
 *
 * <p>Both driver input and hunter output are in the ROBOT's own frame here.
 * The OpMode is responsible for rotating field-frame stick input into the
 * robot frame before it gets to this class.
 */
public final class BlendController {

    private BlendController() { }

    /**
     * @param driverAxial      driver axial   (robot frame, -1..1)
     * @param driverLateral    driver lateral (robot frame, -1..1)
     * @param driverYaw        driver yaw     (robot frame, -1..1)
     * @param hunter           hunter's proposed command
     * @return the blended {@link DriveCommand} to send to the drivetrain.
     *         Intake follows the hunter's request - the driver cannot
     *         override intake state here.
     */
    public static DriveCommand blend(double driverAxial, double driverLateral,
                                     double driverYaw, DriveCommand hunter) {
        double axial   = takeover(driverAxial,   hunter.axial);
        double lateral = takeover(driverLateral, hunter.lateral);
        double yaw     = takeover(driverYaw,     hunter.yaw);
        return new DriveCommand(axial, lateral, yaw, hunter.intakeOn);
    }

    private static double takeover(double driver, double hunter) {
        return Math.abs(driver) > HuntConfig.STICK_OVERRIDE_DEADZONE ? driver : hunter;
    }
}
