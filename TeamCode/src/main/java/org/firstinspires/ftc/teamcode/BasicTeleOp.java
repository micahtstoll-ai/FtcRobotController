package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic TeleOp", group = "Linear OpMode")
public class BasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

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

            telemetry.addData("Sticks", "axial=%.2f lateral=%.2f yaw=%.2f", axial, lateral, yaw);
            telemetry.addData("Front", "L=%.2f R=%.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Rear",  "L=%.2f R=%.2f", leftRearPower, rightRearPower);
            telemetry.update();
        }
    }
}
