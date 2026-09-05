package org.firstinspires.ftc.teamcode.hunt;

/**
 * A request to the drivetrain, expressed in the robot's own frame:
 * <ul>
 *   <li>{@code axial}   - forward power   (+1 forward, -1 backward)</li>
 *   <li>{@code lateral} - sideways power  (+1 right, -1 left)</li>
 *   <li>{@code yaw}     - turning power   (+1 CCW as seen from above)</li>
 * </ul>
 *
 * <p>All three are in the range [-1, 1]. The drivetrain code decides how to
 * convert this into individual motor powers.
 *
 * <p>Also carries a boolean saying whether the intake should be running,
 * because the hunter needs to say "arriving, start the intake" and the
 * OpMode is the one that owns the intake subsystem.
 */
public final class DriveCommand {

    public static final DriveCommand STOP = new DriveCommand(0, 0, 0, false);

    public final double axial;
    public final double lateral;
    public final double yaw;
    public final boolean intakeOn;

    public DriveCommand(double axial, double lateral, double yaw, boolean intakeOn) {
        this.axial = axial;
        this.lateral = lateral;
        this.yaw = yaw;
        this.intakeOn = intakeOn;
    }

    @Override
    public String toString() {
        return String.format("Drive[ax=%.2f lat=%.2f yaw=%.2f intake=%s]",
                axial, lateral, yaw, intakeOn ? "ON" : "off");
    }
}
