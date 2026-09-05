package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Velocity-controlled intake with a slew-rate-limited command. Owns the intake
 * motor and its commanded-velocity state. Callers pass a desired RPM (positive
 * or negative for direction) each loop iteration and the subsystem ramps toward
 * it, bounding peak torque on the gearbox and chain during toggles and reversals.
 */
public class Intake {

    // Yellow Jacket 1150 RPM goBILDA 5202/5203/5204: 145.1 ticks per output rev.
    public static final double TICKS_PER_REV = 145.1;
    public static final double MIN_RPM       = 0.0;
    public static final double MAX_RPM       = 1150.0;

    // Sized below the motor's top speed so the limit binds on both spin-up and
    // reversal, not only reversal. At 1000 RPM/s: 0 -> setpoint(1000) in ~1 s,
    // full 1000 -> -1000 reversal in ~2 s.
    public static final double MAX_ACCEL_RPM_PER_SEC = 1000.0;
    private static final double MAX_ACCEL_TPS2 =
            MAX_ACCEL_RPM_PER_SEC * TICKS_PER_REV / 60.0;
    // Cap dt so a stalled first frame or a long pause cannot produce a huge jump.
    private static final double MAX_DT_SEC = 0.1;

    private final DcMotorEx motor;
    private final ElapsedTime slewTimer = new ElapsedTime();
    private double commandedTps = 0.0;

    public Intake(HardwareMap hardwareMap) {
        this.motor = RobotHardware.getIntake(hardwareMap);
    }

    /**
     * Slew the commanded velocity toward {@code desiredRpm} and push it to the
     * motor. Call every loop iteration; the slew rate is defined in RPM/s so
     * missing a loop simply produces a smaller step, not a discontinuity.
     */
    public void setDesiredRpm(double desiredRpm) {
        double desiredTps = desiredRpm * TICKS_PER_REV / 60.0;
        double dt = Math.min(slewTimer.seconds(), MAX_DT_SEC);
        slewTimer.reset();
        double maxDelta = MAX_ACCEL_TPS2 * dt;
        double error = desiredTps - commandedTps;
        if (error >  maxDelta) error =  maxDelta;
        if (error < -maxDelta) error = -maxDelta;
        commandedTps += error;
        motor.setVelocity(commandedTps);
    }

    public double getCommandedRpm() {
        return commandedTps * 60.0 / TICKS_PER_REV;
    }

    public double getActualRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }
}
