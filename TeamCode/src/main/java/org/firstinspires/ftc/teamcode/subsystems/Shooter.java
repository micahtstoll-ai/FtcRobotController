package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Velocity-controlled shooter flywheel. Callers set a target RPM each loop;
 * zero means coast. Spinning down is done open-loop at zero power rather than
 * by commanding zero velocity, so the controller never drives the motor in
 * reverse to brake a 6000 RPM flywheel.
 */
public class Shooter {

    // goBILDA 5203 6000 RPM (1:1, bare): 28 ticks per output rev.
    public static final double TICKS_PER_REV = 28.0;
    public static final double MAX_RPM       = 6000.0;

    // Within this band of the target the shooter reports ready to fire.
    public static final double READY_TOLERANCE_RPM = 100.0;

    // Hub velocity PIDF. Feedforward plus P only: an integral term winds up
    // while a shot drags the wheel down and overshoots on recovery, which makes
    // the next shot inconsistent. Starting F = 32767 / nominal max ticks/s,
    // P = 0.1 F; not yet tuned on the robot. Tune with "TeleOp: Tune Velocity
    // PIDF" and copy the result here.
    public static final PIDFCoefficients PIDF = new PIDFCoefficients(1.17, 0.0, 0.0, 11.7);

    private final DcMotorEx motor;
    private double targetRpm = 0.0;
    private boolean closedLoop = true;

    public Shooter(HardwareMap hardwareMap) {
        this.motor = RobotHardware.getShooter(hardwareMap);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
    }

    /** Spin toward {@code rpm}; zero or less coasts the flywheel down. */
    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(MAX_RPM, rpm));
        if (targetRpm == 0.0) {
            if (closedLoop) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                closedLoop = false;
            }
            motor.setPower(0.0);
        } else {
            if (!closedLoop) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                closedLoop = true;
            }
            motor.setVelocity(targetRpm * TICKS_PER_REV / 60.0);
        }
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getActualRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    /** True when spinning and within {@link #READY_TOLERANCE_RPM} of the target. */
    public boolean isReady() {
        return targetRpm > 0.0
                && Math.abs(getActualRpm() - targetRpm) <= READY_TOLERANCE_RPM;
    }
}
