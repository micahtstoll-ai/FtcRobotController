package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware.DriveMotors;

/**
 * Mecanum drive. Owns the four drive motors and applies standard mecanum
 * kinematics with peak-normalized wheel powers.
 *
 * Frame conventions:
 *   axial   = forward (+) / reverse (-) in the robot frame
 *   lateral = right   (+) / left    (-) in the robot frame
 *   yaw     = clockwise viewed from above (+)
 *
 * Field-oriented input uses a CCW-positive heading in radians, matching what
 * Pinpoint reports with the pod directions in {@link RobotHardware}.
 */
public class MecanumDrive {

    private final DriveMotors motors;

    public MecanumDrive(HardwareMap hardwareMap) {
        this.motors = RobotHardware.getDriveMotors(hardwareMap);
    }

    /** Drive with peak wheel power capped at 1.0. */
    public void drive(double axial, double lateral, double yaw) {
        drive(axial, lateral, yaw, 1.0);
    }

    /**
     * Drive with peak wheel power capped at {@code maxWheelPower}. Wheel commands
     * are normalized so the largest magnitude equals {@code maxWheelPower} when
     * the un-normalized peak exceeds 1.0, otherwise scaled by {@code maxWheelPower}.
     * This preserves the input's relative wheel ratios while enforcing a hard cap.
     */
    public void drive(double axial, double lateral, double yaw, double maxWheelPower) {
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lr = axial - lateral + yaw;
        double rr = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                              Math.max(Math.abs(lr), Math.abs(rr)));
        double scale = (max > 1.0) ? (maxWheelPower / max) : maxWheelPower;

        motors.leftFront.setPower(lf * scale);
        motors.rightFront.setPower(rf * scale);
        motors.leftRear.setPower(lr * scale);
        motors.rightRear.setPower(rr * scale);
    }

    /**
     * Rotate a field-frame command into the robot frame, then drive. Heading is
     * CCW-positive in radians.
     */
    public void driveFieldOriented(double fieldForward, double fieldRight,
                                    double yaw, double headingRadians) {
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        double axial   = fieldForward * cos - fieldRight * sin;
        double lateral = fieldForward * sin + fieldRight * cos;
        drive(axial, lateral, yaw);
    }

    public void stop() {
        drive(0.0, 0.0, 0.0);
    }
}
