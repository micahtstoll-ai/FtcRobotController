package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The intake: one motor that pulls balls in, spits them back out, or sits
 * still. Runs the motor controller's built-in velocity PIDF loop so the
 * motor holds a target RPM even under load - much more repeatable than
 * plain {@code setPower(...)}.
 *
 * <p>A slew rate limit sits between what the caller ASKS for and what the
 * motor actually SEES. When you say "spin at 1000 RPM," the commanded value
 * ramps up from where it was, at the rate set by
 * {@link #MAX_ACCEL_RPM_PER_SEC}. That protects the intake gearbox and
 * chain from step changes (start / stop and forward-reverse reversals).
 *
 * <p>API:
 * <ul>
 *   <li>{@link #intake()} / {@link #eject()} / {@link #stop()} - shortcuts
 *       for common cases.</li>
 *   <li>{@link #setTargetRpm(double)} - fine control if you want a specific
 *       speed.</li>
 *   <li>{@link #update()} - CALL EVERY LOOP. This is what applies the slew
 *       rate and writes the motor. If you never call it, the motor keeps
 *       spinning at whatever velocity it was last told.</li>
 * </ul>
 *
 * <p>Constants and slew-rate protection are ported from the intake code
 * that used to live inline in {@code BasicTeleOp}. This subsystem is the
 * "actual intake function" without any of the gamepad-button mapping - so
 * every OpMode that needs an intake can share the same behavior.
 */
public class IntakeSubsystem {

    /** Yellow Jacket 1150 RPM series (goBILDA 5202/5203/5204): 145.1 ticks per output rev. */
    public static final double TICKS_PER_REV = 145.1;

    /** The motor's rated top speed. */
    public static final double MAX_RPM = 1150.0;

    /** The RPM {@link #intake()} runs at unless {@link #setTargetRpm(double)} says otherwise. */
    public static final double DEFAULT_INTAKE_RPM = 1000.0;

    /**
     * Cap on how fast the commanded velocity can change. Sized below the
     * motor's top speed so the limit meaningfully affects both spin-up AND
     * reversal, not just reversal. At 1000 RPM/s: 0 to 1000 RPM in about 1
     * second; full 1000 to -1000 RPM reversal in about 2 seconds.
     */
    public static final double MAX_ACCEL_RPM_PER_SEC = 1000.0;

    private static final double MAX_ACCEL_TPS2 =
            MAX_ACCEL_RPM_PER_SEC * TICKS_PER_REV / 60.0;

    /** Cap dt so a stalled first frame or a long pause cannot produce a huge jump. */
    private static final double MAX_DT_SEC = 0.1;

    private final DcMotorEx motor;
    private final ElapsedTime slewTimer = new ElapsedTime();
    private double desiredTps = 0.0;
    private double commandedTps = 0.0;

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
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slewTimer.reset();
    }

    /**
     * Set the target RPM. Positive = intake direction, negative = eject.
     * Clamped to +/- {@link #MAX_RPM}. The motor does NOT immediately jump;
     * {@link #update()} handles the ramp.
     */
    public void setTargetRpm(double rpm) {
        double clamped = clamp(rpm, -MAX_RPM, MAX_RPM);
        this.desiredTps = clamped * TICKS_PER_REV / 60.0;
    }

    /** Run in the intake direction at the default RPM. */
    public void intake() {
        setTargetRpm(DEFAULT_INTAKE_RPM);
    }

    /** Run in the eject direction at the default RPM. */
    public void eject() {
        setTargetRpm(-DEFAULT_INTAKE_RPM);
    }

    /** Stop pulling. The motor still ramps down at the slew rate. */
    public void stop() {
        this.desiredTps = 0.0;
    }

    /**
     * Called once per OpMode loop. Applies the slew rate limit and writes the
     * motor. Call this every iteration; not calling it leaves the motor at
     * whatever velocity was last written.
     */
    public void update() {
        double dt = Math.min(slewTimer.seconds(), MAX_DT_SEC);
        slewTimer.reset();
        double maxDelta = MAX_ACCEL_TPS2 * dt;
        double error = desiredTps - commandedTps;
        if (error >  maxDelta) error =  maxDelta;
        if (error < -maxDelta) error = -maxDelta;
        commandedTps += error;
        motor.setVelocity(commandedTps);
    }

    /** Current commanded RPM (after slew), for telemetry. */
    public double commandedRpm() {
        return commandedTps * 60.0 / TICKS_PER_REV;
    }

    /** Current actual RPM as reported by the encoder, for telemetry. */
    public double actualRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    /** Currently requested target RPM (pre-slew), for telemetry. */
    public double targetRpm() {
        return desiredTps * 60.0 / TICKS_PER_REV;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
