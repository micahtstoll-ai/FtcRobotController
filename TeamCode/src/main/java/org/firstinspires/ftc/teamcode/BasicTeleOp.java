package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Basic TeleOp", group = "Linear OpMode")
public class BasicTeleOp extends LinearOpMode {

    private static final double SLOW_MODE_SCALE = 0.35;

    // Yellow Jacket 1150 RPM (goBILDA 5202/5203/5204 series): 145.1 ticks per output rev.
    private static final double INTAKE_TICKS_PER_REV = 145.1;
    private static final double INTAKE_INITIAL_RPM   = 1000.0;
    private static final double INTAKE_RPM_STEP      = 25.0;
    private static final double INTAKE_MIN_RPM       = 0.0;
    private static final double INTAKE_MAX_RPM       = 1150.0;
    private static final double INTAKE_TRIGGER_THRESHOLD = 0.25;

    @Override
    public void runOpMode() {
        DcMotorEx leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx intake     = hardwareMap.get(DcMotorEx.class, "intake");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake runs a built-in PIDF velocity loop. FLOAT so game pieces can free-wheel
        // through the intake when the trigger is released.
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Pinpoint config mirrors MotorTest.java / pedroPathing/Constants.java.
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-90.0, -12.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // Point the robot down-field before init: this zero becomes field-forward.
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Initialized. Point robot down-field, then start.");
        telemetry.update();

        waitForStart();

        double headingOffset = 0.0;
        boolean prevResetButton = false;

        double  intakeTargetRpm  = INTAKE_INITIAL_RPM;
        boolean intakeEnabled    = true;   // starts on; Y toggles.
        boolean intakeTrimMode   = false;  // toggled by X; when true, dpad up/down trims setpoint
        boolean prevIntakeToggle = false;
        boolean prevIntakeTrim   = false;
        boolean prevDpadUp       = false;
        boolean prevDpadDown     = false;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double rawHeading = pose.getHeading(AngleUnit.RADIANS);

            boolean resetButton = gamepad1.options || gamepad1.b;
            if (resetButton && !prevResetButton) {
                headingOffset = rawHeading;
            }
            prevResetButton = resetButton;

            double heading = rawHeading - headingOffset;

            double fieldForward = -gamepad1.left_stick_y;
            double fieldRight   =  gamepad1.left_stick_x;
            double yaw          =  gamepad1.right_stick_x;

            // Rotate the field-frame command by -heading into the robot frame.
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double axial   =  fieldForward * cos + fieldRight * sin;
            double lateral = -fieldForward * sin + fieldRight * cos;

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            axial   *= scale;
            lateral *= scale;
            yaw     *= scale;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftRearPower   = axial - lateral + yaw;
            double rightRearPower  = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftRearPower));
            max = Math.max(max, Math.abs(rightRearPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftRearPower   /= max;
                rightRearPower  /= max;
            }

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftRearPower);
            rightRear.setPower(rightRearPower);

            // Intake controls. Not scaled by slow mode.
            // - Starts ON at setpoint. Y (edge) toggles on/off.
            // - Left trigger past threshold (while ON): reverses direction while held.
            // - X (edge): toggle trim mode.
            // - In trim mode, D-pad up/down (edge): adjust setpoint by INTAKE_RPM_STEP.
            boolean intakeToggle = gamepad1.y;
            if (intakeToggle && !prevIntakeToggle) {
                intakeEnabled = !intakeEnabled;
            }
            prevIntakeToggle = intakeToggle;

            boolean intakeTrim = gamepad1.x;
            if (intakeTrim && !prevIntakeTrim) {
                intakeTrimMode = !intakeTrimMode;
            }
            prevIntakeTrim = intakeTrim;

            boolean dpadUp   = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (intakeTrimMode) {
                if (dpadUp   && !prevDpadUp)   intakeTargetRpm += INTAKE_RPM_STEP;
                if (dpadDown && !prevDpadDown) intakeTargetRpm -= INTAKE_RPM_STEP;
                intakeTargetRpm = Math.max(INTAKE_MIN_RPM,
                                  Math.min(INTAKE_MAX_RPM, intakeTargetRpm));
            }
            prevDpadUp   = dpadUp;
            prevDpadDown = dpadDown;

            boolean intakeReverse = gamepad1.left_trigger > INTAKE_TRIGGER_THRESHOLD;
            double intakeTargetTps = intakeTargetRpm * INTAKE_TICKS_PER_REV / 60.0;
            double intakeCommandTps =
                    intakeEnabled ? (intakeReverse ? -intakeTargetTps : intakeTargetTps) : 0.0;
            intake.setVelocity(intakeCommandTps);

            telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Intake", "state=%s dir=%s trim=%s target=%.0f rpm actual=%.0f rpm",
                    intakeEnabled ? "ON" : "OFF",
                    intakeReverse ? "REV" : "FWD",
                    intakeTrimMode ? "ON" : "off",
                    intakeTargetRpm,
                    intake.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV);
            // Displayed heading is negated so a left (CCW) turn reads negative,
            // matching compass convention. The rotation math above still uses
            // the raw Pinpoint value in its native CCW-positive convention.
            telemetry.addData("Heading raw (deg)",  "%.1f", -Math.toDegrees(rawHeading));
            telemetry.addData("Heading used (deg)", "%.1f", -Math.toDegrees(heading));
            telemetry.addData("Field", "fwd=%.2f right=%.2f yaw=%.2f", fieldForward, fieldRight, yaw);
            telemetry.addData("Robot", "axial=%.2f lateral=%.2f", axial, lateral);
            telemetry.update();
        }
    }
}
