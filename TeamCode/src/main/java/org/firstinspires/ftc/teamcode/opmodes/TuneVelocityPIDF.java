package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * Live tuner for the hub velocity PIDF on the intake, transfer, and shooter.
 * Gains are edited in place on each subsystem's static PIDF object, so they
 * carry into other OpModes until the Robot Controller app restarts. Copy the
 * final numbers from telemetry into the subsystem source to keep them.
 *
 * Procedure, per mechanism:
 *   1. Hold A for ~3 s. Telemetry shows the measured peak and a suggested F.
 *   2. Set F to the suggestion (dpad up/down on F), P to about 0.1 F.
 *   3. Press B to run at target. Press BACK for a step test. Raise P until the
 *      step response is fast without oscillating; back off if it rings.
 *   4. Shooter: feed real game pieces and watch "worst dip" and recovery.
 *
 * Gamepad 1:
 *   dpad left/right   select mechanism (stops the current one)
 *   A (hold)          open-loop full-power run, measures peak speed
 *   B                 toggle closed-loop run at target
 *   left/right bumper target -/+ one step
 *   BACK              toggle step test (target alternates with half target)
 *   Y                 cycle which gain is selected (P, I, D, F)
 *   dpad up/down      selected gain x1.25 / /1.25 (up from 0 seeds 0.05)
 *   X                 zero the selected gain
 *   right stick press reset worst-dip tracker
 */
@TeleOp(name = "TeleOp: Tune Velocity PIDF", group = "Tuning")
public class TuneVelocityPIDF extends LinearOpMode {

    private static final double GAIN_FACTOR     = 1.25;
    private static final double GAIN_SEED       = 0.05;
    private static final double STEP_PERIOD_SEC = 1.5;
    private static final String[] GAIN_NAMES    = {"P", "I", "D", "F"};

    /** One tunable velocity-controlled motor. */
    private static final class Mechanism {
        final String name;
        final DcMotorEx motor;
        final double ticksPerRev;
        final double maxRpm;
        final double rpmStep;
        final PIDFCoefficients gains;
        double targetRpm;

        Mechanism(String name, DcMotorEx motor, double ticksPerRev, double maxRpm,
                  double rpmStep, double initialTargetRpm, PIDFCoefficients gains) {
            this.name        = name;
            this.motor       = motor;
            this.ticksPerRev = ticksPerRev;
            this.maxRpm      = maxRpm;
            this.rpmStep     = rpmStep;
            this.targetRpm   = initialTargetRpm;
            this.gains       = gains;
        }

        double rpm() {
            return motor.getVelocity() * 60.0 / ticksPerRev;
        }

        void applyGains() {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
        }

        void coast() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
        }
    }

    @Override
    public void runOpMode() {
        Mechanism[] mechs = {
                new Mechanism("Intake",   RobotHardware.getIntake(hardwareMap),
                        Intake.TICKS_PER_REV, Intake.MAX_RPM, 25.0, 1000.0, Intake.PIDF),
                new Mechanism("Transfer", RobotHardware.getTransfer(hardwareMap),
                        Transfer.TICKS_PER_REV, Transfer.MAX_RPM, 100.0, 3000.0, Transfer.PIDF),
                new Mechanism("Shooter",  RobotHardware.getShooter(hardwareMap),
                        Shooter.TICKS_PER_REV, Shooter.MAX_RPM, 100.0, 4000.0, Shooter.PIDF),
        };
        for (Mechanism m : mechs) {
            m.applyGains();
            m.coast();
        }

        telemetry.addLine("Velocity PIDF tuner. Start, then follow the procedure");
        telemetry.addLine("in TuneVelocityPIDF.java. Mechanisms spin when commanded.");
        telemetry.update();
        waitForStart();

        int selected = 0;
        int gainIndex = 0;
        boolean running = false;
        boolean stepTest = false;
        boolean measuring = false;
        double peakTps = 0.0;
        double worstDipRpm = 0.0;
        ElapsedTime stepTimer = new ElapsedTime();

        boolean prevLeft = false, prevRight = false, prevB = false, prevBack = false;
        boolean prevY = false, prevUp = false, prevDown = false, prevX = false;
        boolean prevLb = false, prevRb = false, prevRs = false;

        while (opModeIsActive()) {
            Mechanism m = mechs[selected];

            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            if ((left && !prevLeft) || (right && !prevRight)) {
                m.coast();
                running = false;
                stepTest = false;
                peakTps = 0.0;
                worstDipRpm = 0.0;
                selected = (selected + (right ? 1 : mechs.length - 1)) % mechs.length;
                m = mechs[selected];
            }
            prevLeft = left;
            prevRight = right;

            boolean b = gamepad1.b;
            if (b && !prevB) {
                running = !running;
                worstDipRpm = 0.0;
                if (!running) m.coast();
            }
            prevB = b;

            boolean back = gamepad1.back;
            if (back && !prevBack) {
                stepTest = !stepTest;
                stepTimer.reset();
            }
            prevBack = back;

            boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;
            if (lb && !prevLb) m.targetRpm -= m.rpmStep;
            if (rb && !prevRb) m.targetRpm += m.rpmStep;
            m.targetRpm = Math.max(0.0, Math.min(m.maxRpm, m.targetRpm));
            prevLb = lb;
            prevRb = rb;

            boolean y = gamepad1.y;
            if (y && !prevY) gainIndex = (gainIndex + 1) % GAIN_NAMES.length;
            prevY = y;

            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, x = gamepad1.x;
            boolean gainsChanged = false;
            if (up && !prevUp) {
                double g = getGain(m.gains, gainIndex);
                setGain(m.gains, gainIndex, g == 0.0 ? GAIN_SEED : g * GAIN_FACTOR);
                gainsChanged = true;
            }
            if (down && !prevDown) {
                setGain(m.gains, gainIndex, getGain(m.gains, gainIndex) / GAIN_FACTOR);
                gainsChanged = true;
            }
            if (x && !prevX) {
                setGain(m.gains, gainIndex, 0.0);
                gainsChanged = true;
            }
            prevUp = up;
            prevDown = down;
            prevX = x;
            if (gainsChanged) m.applyGains();

            boolean rs = gamepad1.right_stick_button;
            if (rs && !prevRs) worstDipRpm = 0.0;
            prevRs = rs;

            // Open-loop peak measurement overrides everything else while held.
            boolean a = gamepad1.a;
            if (a) {
                if (!measuring) {
                    measuring = true;
                    peakTps = 0.0;
                    m.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                m.motor.setPower(1.0);
                peakTps = Math.max(peakTps, Math.abs(m.motor.getVelocity()));
            } else {
                if (measuring) {
                    measuring = false;
                    m.coast();
                }
                if (running) {
                    double setpoint = m.targetRpm;
                    if (stepTest && ((int) (stepTimer.seconds() / STEP_PERIOD_SEC)) % 2 == 1) {
                        setpoint *= 0.5;
                    }
                    if (m.motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        m.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    m.motor.setVelocity(setpoint * m.ticksPerRev / 60.0);
                    if (!stepTest) worstDipRpm = Math.max(worstDipRpm, setpoint - m.rpm());
                }
            }

            double actual = m.rpm();
            telemetry.addData("Mechanism", "%s  (dpad L/R to change)", m.name);
            telemetry.addData("State", measuring ? "MEASURING PEAK"
                    : running ? (stepTest ? "RUNNING step test" : "RUNNING") : "stopped");
            telemetry.addData("Target / actual", "%.0f / %.0f rpm", m.targetRpm, actual);
            telemetry.addData("Error", "%.0f rpm", m.targetRpm - actual);
            telemetry.addData("Worst dip", "%.0f rpm (R stick to reset)", worstDipRpm);
            telemetry.addData("Selected gain", GAIN_NAMES[gainIndex]);
            telemetry.addData("Gains", "P=%.4f I=%.4f D=%.4f F=%.4f",
                    m.gains.p, m.gains.i, m.gains.d, m.gains.f);
            if (peakTps > 0.0) {
                telemetry.addData("Measured peak", "%.0f rpm", peakTps * 60.0 / m.ticksPerRev);
                telemetry.addData("Suggested F", "%.3f  (P about %.3f)",
                        32767.0 / peakTps, 0.1 * 32767.0 / peakTps);
            }
            telemetry.addLine("Copy final gains into " + m.name + ".PIDF in source.");
            telemetry.update();
        }

        for (Mechanism m : mechs) m.coast();
    }

    private static double getGain(PIDFCoefficients c, int index) {
        switch (index) {
            case 0:  return c.p;
            case 1:  return c.i;
            case 2:  return c.d;
            default: return c.f;
        }
    }

    private static void setGain(PIDFCoefficients c, int index, double value) {
        switch (index) {
            case 0:  c.p = value; break;
            case 1:  c.i = value; break;
            case 2:  c.d = value; break;
            default: c.f = value; break;
        }
    }
}
