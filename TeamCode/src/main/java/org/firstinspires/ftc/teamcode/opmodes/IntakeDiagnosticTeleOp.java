package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.intake.IntakeSubsystem;

/**
 * Runs nothing but the intake motor, so the direction, wiring, and PIDF
 * behavior can be validated before the intake gets wired into anything
 * bigger.
 *
 * <p>Controls:
 * <ul>
 *   <li>Left trigger past 25%: intake (pull balls in)</li>
 *   <li>Left bumper: eject (spit balls out)</li>
 *   <li>Neither: stop</li>
 * </ul>
 *
 * <p>Telemetry shows the target, commanded (post-slew), and actual RPM. If
 * the motor spins the WRONG direction for "intake", change the direction
 * constant in the setup below.
 *
 * <p>Does not touch the drivetrain. Safe to run.
 */
@TeleOp(name = "Intake Diagnostic", group = "intake")
public class IntakeDiagnosticTeleOp extends LinearOpMode {

    private static final double LT_THRESHOLD = 0.25;

    @Override
    public void runOpMode() {
        IntakeSubsystem intake = new IntakeSubsystem(
                hardwareMap, "intake", DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Intake ready.");
        telemetry.addLine("  LT (past 25%) -> intake");
        telemetry.addLine("  LB            -> eject");
        telemetry.addLine("  neither       -> stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > LT_THRESHOLD) {
                intake.intake();
            } else if (gamepad1.left_bumper) {
                intake.eject();
            } else {
                intake.stop();
            }

            intake.update();

            telemetry.addData("target (rpm)",    "%.0f", intake.targetRpm());
            telemetry.addData("commanded (rpm)", "%.0f", intake.commandedRpm());
            telemetry.addData("actual (rpm)",    "%.0f", intake.actualRpm());
            telemetry.update();
        }

        intake.stop();
        intake.update();
    }
}
