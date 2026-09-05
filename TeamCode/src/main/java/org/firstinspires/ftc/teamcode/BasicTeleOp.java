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
    private static final double INTAKE_TARGET_RPM    = 1000.0;
    private static final double INTAKE_TARGET_TPS    =
            INTAKE_TARGET_RPM * INTAKE_TICKS_PER_REV / 60.0;
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

        boolean prevResetButton = false;

        while (opModeIsActive()) {
            boolean resetButton = gamepad1.options || gamepad1.b;
            if (resetButton && !prevResetButton) {
                pinpoint.resetPosAndIMU();
            }
            prevResetButton = resetButton;

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            // Pinpoint reports heading CCW-positive (raw -90 deg after a
            // physical right turn, verified on the driver station), which
            // is what the rotation math below expects. Use the value as-is.
            double heading = pose.getHeading(AngleUnit.RADIANS);

            double fieldForward = -gamepad1.left_stick_y;
            double fieldRight   =  gamepad1.left_stick_x;
            double yaw          =  gamepad1.right_stick_x;

            // Rotate the field-frame command into the robot frame.
            // Assumes Pinpoint heading is CCW-positive in a Y-left field
            // frame (same convention MotorTest.driveFor relies on).
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double axial   = fieldForward * cos - fieldRight * sin;
            double lateral = fieldForward * sin + fieldRight * cos;

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

            // Intake: left trigger past threshold commands 1000 RPM, else 0. Not scaled
            // by slow mode.
            boolean intakeOn = gamepad1.left_trigger > INTAKE_TRIGGER_THRESHOLD;
            intake.setVelocity(intakeOn ? INTAKE_TARGET_TPS : 0.0);

            telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Intake", "cmd=%s target=%.0f rpm actual=%.0f rpm",
                    intakeOn ? "ON" : "off",
                    INTAKE_TARGET_RPM,
                    intake.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV);
            telemetry.addData("Heading raw (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Field", "fwd=%.2f right=%.2f yaw=%.2f", fieldForward, fieldRight, yaw);
            telemetry.addData("Robot", "axial=%.2f lateral=%.2f", axial, lateral);
            telemetry.addData("Wheels", "LF=%.2f RF=%.2f LR=%.2f RR=%.2f",
                    leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
            telemetry.update();
        }
    }
}
