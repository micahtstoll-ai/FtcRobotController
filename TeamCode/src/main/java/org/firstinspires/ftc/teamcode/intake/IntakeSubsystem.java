package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The intake: one motor that either pulls balls in, spits them back out, or
 * sits still. No sensor - we do not know when a ball is actually loaded.
 *
 * <p>The intake power level is a constant here so the hunter code stays
 * simple. If the mechanism needs different speeds for different situations,
 * add a second setter later - do not sprinkle magic numbers through the
 * hunter.
 */
public class IntakeSubsystem {

    /** Speed to run at when actively ingesting a ball. */
    public static final double INTAKE_POWER = 1.0;

    /** Reverse speed for un-jamming. */
    public static final double EJECT_POWER = -0.8;

    private final DcMotorEx motor;

    /**
     * @param hardwareMap  the OpMode's hardware map
     * @param configName   the intake motor's name in the robot configuration
     *                     (default: {@code "intake"})
     * @param direction    which way the motor spins to pull a ball IN
     */
    public IntakeSubsystem(HardwareMap hardwareMap, String configName,
                           DcMotorSimple.Direction direction) {
        this.motor = hardwareMap.get(DcMotorEx.class, configName);
        this.motor.setDirection(direction);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Pull balls in. */
    public void intake() {
        motor.setPower(INTAKE_POWER);
    }

    /** Spit balls out (for un-jamming). */
    public void eject() {
        motor.setPower(EJECT_POWER);
    }

    /** Stop the intake motor. */
    public void stop() {
        motor.setPower(0.0);
    }

    /** Current motor power, useful for telemetry. */
    public double power() {
        return motor.getPower();
    }
}
