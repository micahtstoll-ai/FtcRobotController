package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Motor Test")
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            leftFront.setPower(0.3);
            rightFront.setPower(0.3);
            leftRear.setPower(0.3);
            rightRear.setPower(0.3);

            sleep(2000);

            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftRear.setPower(0.0);
            rightRear.setPower(0.0);
        }
    }
}
