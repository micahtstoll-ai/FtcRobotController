package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Motor Test")
public class MotorTest extends LinearOpMode {

    private static final double POWER = 0.3;
    private static final long SIDE_MS = 1500;
    private static final long DIAG_MS = (long) Math.round(750.0 * Math.sqrt(2.0));

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        drive( 1,  0, SIDE_MS);
        drive( 0,  1, SIDE_MS);
        drive(-1,  0, SIDE_MS);
        drive( 0, -1, SIDE_MS);

        drive( 1,  1, DIAG_MS);

        drive( 1,  1, DIAG_MS);  drive(-1, -1, DIAG_MS);
        drive(-1,  1, DIAG_MS);  drive( 1, -1, DIAG_MS);
        drive(-1, -1, DIAG_MS);  drive( 1,  1, DIAG_MS);
        drive( 1, -1, DIAG_MS);  drive(-1,  1, DIAG_MS);

        stopMotors();
    }

    private void drive(double axial, double lateral, long durationMs) {
        if (!opModeIsActive()) return;

        double lf = axial + lateral;
        double rf = axial - lateral;
        double lr = axial - lateral;
        double rr = axial + lateral;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                              Math.max(Math.abs(lr), Math.abs(rr)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lr /= max;
            rr /= max;
        }

        leftFront.setPower(lf * POWER);
        rightFront.setPower(rf * POWER);
        leftRear.setPower(lr * POWER);
        rightRear.setPower(rr * POWER);

        sleep(durationMs);
    }

    private void stopMotors() {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }
}
