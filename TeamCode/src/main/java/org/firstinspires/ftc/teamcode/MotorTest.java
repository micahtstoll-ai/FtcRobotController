package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Motor Test")
public class MotorTest extends LinearOpMode {

    private static final double POWER = 0.3;
    private static final double SIDE_IN = 36.0;
    private static final double HALF_DIAG_IN = 18.0 * Math.sqrt(2.0);

    private static final double RAMP_INCHES = 6.0;
    private static final double MIN_POWER_FRAC = 0.30;
    private static final double CROSS_TRACK_KP = 0.20;
    private static final double HEADING_KP = 0.03;
    private static final long SETTLE_MS = 100;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-90.0, -12.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSE,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            telemetry.addLine("Push robot to verify pod directions:");
            telemetry.addLine("  forward  -> X should INCREASE");
            telemetry.addLine("  left     -> Y should INCREASE");
            telemetry.addLine("If either goes the wrong way, flip that pod's");
            telemetry.addLine("EncoderDirection constant in the code.");
            telemetry.addData("X (in)", "%.2f", p.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", p.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        if (!opModeIsActive()) return;

        pinpoint.resetPosAndIMU();

        driveFor( 1,  0, SIDE_IN);
        driveFor( 0,  1, SIDE_IN);
        driveFor(-1,  0, SIDE_IN);
        driveFor( 0, -1, SIDE_IN);

        driveFor( 1,  1, HALF_DIAG_IN);

        driveFor( 1,  1, HALF_DIAG_IN);  driveFor(-1, -1, HALF_DIAG_IN);
        driveFor(-1,  1, HALF_DIAG_IN);  driveFor( 1, -1, HALF_DIAG_IN);
        driveFor(-1, -1, HALF_DIAG_IN);  driveFor( 1,  1, HALF_DIAG_IN);
        driveFor( 1, -1, HALF_DIAG_IN);  driveFor(-1,  1, HALF_DIAG_IN);

        stopMotors();
    }

    private void driveFor(double axial, double lateral, double distanceInches) {
        if (!opModeIsActive()) return;

        double dirX = axial;
        double dirY = -lateral;
        double dirNorm = Math.hypot(dirX, dirY);
        if (dirNorm < 1e-9) return;
        dirX /= dirNorm;
        dirY /= dirNorm;

        pinpoint.update();
        Pose2D start = pinpoint.getPosition();
        double startX = start.getX(DistanceUnit.INCH);
        double startY = start.getY(DistanceUnit.INCH);
        double startHeadingDeg = start.getHeading(AngleUnit.DEGREES);

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D now = pinpoint.getPosition();
            double dx = now.getX(DistanceUnit.INCH) - startX;
            double dy = now.getY(DistanceUnit.INCH) - startY;

            double progress   = dx * dirX + dy * dirY;
            double crossTrack = dx * (-dirY) + dy * dirX;

            double remaining = distanceInches - progress;
            if (remaining <= 0) break;

            double speedScale = Math.max(MIN_POWER_FRAC,
                                         Math.min(1.0, remaining / RAMP_INCHES));

            double corrPinX = crossTrack * dirY;
            double corrPinY = -crossTrack * dirX;
            double axialCorr   = CROSS_TRACK_KP * corrPinX;
            double lateralCorr = CROSS_TRACK_KP * (-corrPinY);

            double headingErr = now.getHeading(AngleUnit.DEGREES) - startHeadingDeg;
            while (headingErr > 180)  headingErr -= 360;
            while (headingErr < -180) headingErr += 360;
            double yawCorr = HEADING_KP * headingErr;

            double axialCmd   = axial   * speedScale + axialCorr;
            double lateralCmd = lateral * speedScale + lateralCorr;

            double lf = axialCmd + lateralCmd + yawCorr;
            double rf = axialCmd - lateralCmd - yawCorr;
            double lr = axialCmd - lateralCmd + yawCorr;
            double rr = axialCmd + lateralCmd - yawCorr;

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

            telemetry.addData("Target / Progress (in)", "%.2f / %.2f", distanceInches, progress);
            telemetry.addData("Cross-track (in)", "%.2f", crossTrack);
            telemetry.addData("Heading err (deg)", "%.2f", headingErr);
            telemetry.addData("Speed scale", "%.2f", speedScale);
            telemetry.update();
        }

        stopMotors();
        if (opModeIsActive()) sleep(SETTLE_MS);
    }

    private void stopMotors() {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }
}
