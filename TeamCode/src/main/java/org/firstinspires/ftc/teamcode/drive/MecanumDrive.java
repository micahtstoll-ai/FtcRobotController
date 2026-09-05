package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A small helper for the four-motor mecanum drivetrain.
 *
 * <p>Holds the four motors, applies zero-power braking, converts a
 * field-frame command into robot-frame powers via the current heading, and
 * writes the individual motor powers with normalization so nothing
 * saturates.
 *
 * <p>The math and motor-direction choices here match BasicTeleOp.java
 * exactly. BasicTeleOp is left alone (it is the tested prod path); this
 * helper is what the new experimental OpMode uses.
 */
public class MecanumDrive {

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    public MecanumDrive(HardwareMap hardwareMap) {
        this.leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        this.leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        this.rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Rotate a field-frame command into the robot frame using the current
     * heading, then apply it.
     *
     * @param fieldForward  +1 = down-field (the direction the driver was
     *                      facing when Pinpoint was zeroed)
     * @param fieldRight    +1 = to the driver's right
     * @param yaw           +1 = counter-clockwise, robot-frame; yaw is not
     *                      rotated because it is the same in both frames
     * @param headingRad    the robot's current heading in radians (CCW from
     *                      field +x)
     */
    public void driveFieldOriented(double fieldForward, double fieldRight,
                                   double yaw, double headingRad) {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double axial   =  fieldForward * cos + fieldRight * sin;
        double lateral = -fieldForward * sin + fieldRight * cos;
        driveRobotOriented(axial, lateral, yaw);
    }

    /**
     * Apply a robot-frame command directly to the wheels.
     *
     * @param axial    +1 = forward, -1 = backward
     * @param lateral  +1 = strafe right, -1 = strafe left
     * @param yaw      +1 = counter-clockwise (viewed from above)
     */
    public void driveRobotOriented(double axial, double lateral, double yaw) {
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lr = axial - lateral + yaw;
        double rr = axial + lateral - yaw;

        double max = Math.max(Math.abs(lf), Math.abs(rf));
        max = Math.max(max, Math.abs(lr));
        max = Math.max(max, Math.abs(rr));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lr /= max;
            rr /= max;
        }
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }

    /** Kill all four motors. */
    public void stop() {
        driveRobotOriented(0, 0, 0);
    }
}
