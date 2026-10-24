package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Velocity-controlled transfer that feeds game pieces into the shooter. Callers
 * pass a desired RPM each loop (positive feeds toward the shooter); zero holds
 * the transfer still under BRAKE.
 */
public class Transfer {

    // goBILDA 5203 6000 RPM (1:1, bare): 28 ticks per output rev.
    public static final double TICKS_PER_REV = 28.0;
    public static final double MAX_RPM       = 6000.0;

    // Hub velocity PIDF. Starting values from REV's recipe at nominal free speed
    // (F = 32767 / max ticks/s, P = 0.1 F, I = 0.1 P); not yet tuned on the robot.
    // Tune with "TeleOp: Tune Velocity PIDF" and copy the result here.
    public static final PIDFCoefficients PIDF = new PIDFCoefficients(1.17, 0.117, 0.0, 11.7);

    private final DcMotorEx motor;

    public Transfer(HardwareMap hardwareMap) {
        this.motor = RobotHardware.getTransfer(hardwareMap);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
    }

    public void setDesiredRpm(double rpm) {
        double clamped = Math.max(-MAX_RPM, Math.min(MAX_RPM, rpm));
        motor.setVelocity(clamped * TICKS_PER_REV / 60.0);
    }

    public double getActualRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }
}
