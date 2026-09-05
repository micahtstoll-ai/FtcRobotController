package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Driver-controlled OpMode. Field-oriented mecanum drive with a slow-mode
 * modifier and a velocity-controlled intake.
 *
 * Gamepad 1:
 *   left stick        translate on the field (up = down-field)
 *   right stick X     yaw
 *   right bumper      slow mode (hold)
 *   options / B       reset field-forward heading to current pose
 *   Y                 toggle intake on/off (starts ON)
 *   left trigger      reverse intake while held (past threshold)
 *   X                 toggle setpoint-trim mode
 *   dpad up/down      +/- RPM when in trim mode
 */
@TeleOp(name = "TeleOp: Main", group = "Linear OpMode")
public class TeleOpMain extends LinearOpMode {

    private static final double SLOW_MODE_SCALE          = 0.35;
    private static final double INTAKE_INITIAL_RPM       = 1000.0;
    private static final double INTAKE_RPM_STEP          = 25.0;
    private static final double INTAKE_TRIGGER_THRESHOLD = 0.25;

    @Override
    public void runOpMode() {
        MecanumDrive drive  = new MecanumDrive(hardwareMap);
        Intake       intake = new Intake(hardwareMap);
        GoBildaPinpointDriver pinpoint = RobotHardware.getPinpoint(hardwareMap);

        // Point the robot down-field before init: this zero becomes field-forward.
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Initialized. Point robot down-field, then start.");
        telemetry.update();

        waitForStart();

        boolean prevResetButton  = false;
        boolean prevIntakeToggle = false;
        boolean prevIntakeTrim   = false;
        boolean prevDpadUp       = false;
        boolean prevDpadDown     = false;

        double  intakeTargetRpm = INTAKE_INITIAL_RPM;
        boolean intakeEnabled   = true;
        boolean intakeTrimMode  = false;

        while (opModeIsActive()) {
            boolean resetButton = gamepad1.options || gamepad1.b;
            if (resetButton && !prevResetButton) {
                pinpoint.resetPosAndIMU();
            }
            prevResetButton = resetButton;

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            // Pinpoint reports CCW-positive heading with the pod config in
            // RobotHardware; consume it directly.
            double heading = pose.getHeading(AngleUnit.RADIANS);

            double fieldForward = -gamepad1.left_stick_y;
            double fieldRight   =  gamepad1.left_stick_x;
            double yaw          =  gamepad1.right_stick_x;

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            drive.driveFieldOriented(fieldForward * scale,
                                     fieldRight   * scale,
                                     yaw          * scale,
                                     heading);

            boolean intakeToggle = gamepad1.y;
            if (intakeToggle && !prevIntakeToggle) intakeEnabled = !intakeEnabled;
            prevIntakeToggle = intakeToggle;

            boolean intakeTrim = gamepad1.x;
            if (intakeTrim && !prevIntakeTrim) intakeTrimMode = !intakeTrimMode;
            prevIntakeTrim = intakeTrim;

            boolean dpadUp   = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (intakeTrimMode) {
                if (dpadUp   && !prevDpadUp)   intakeTargetRpm += INTAKE_RPM_STEP;
                if (dpadDown && !prevDpadDown) intakeTargetRpm -= INTAKE_RPM_STEP;
                intakeTargetRpm = Math.max(Intake.MIN_RPM,
                                  Math.min(Intake.MAX_RPM, intakeTargetRpm));
            }
            prevDpadUp   = dpadUp;
            prevDpadDown = dpadDown;

            boolean intakeReverse = gamepad1.left_trigger > INTAKE_TRIGGER_THRESHOLD;
            double desiredRpm = intakeEnabled
                    ? (intakeReverse ? -intakeTargetRpm : intakeTargetRpm)
                    : 0.0;
            intake.setDesiredRpm(desiredRpm);

            telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Intake",
                    "state=%s dir=%s trim=%s target=%.0f cmd=%.0f actual=%.0f rpm",
                    intakeEnabled ? "ON" : "OFF",
                    intakeReverse ? "REV" : "FWD",
                    intakeTrimMode ? "ON" : "off",
                    intakeTargetRpm,
                    intake.getCommandedRpm(),
                    intake.getActualRpm());
            telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Field", "fwd=%.2f right=%.2f yaw=%.2f",
                    fieldForward, fieldRight, yaw);
            telemetry.update();
        }
    }
}
