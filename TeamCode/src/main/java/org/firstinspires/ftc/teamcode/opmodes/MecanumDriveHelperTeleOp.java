package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/**
 * A behavioral copy of {@code BasicTeleOp}'s drivetrain routine, but built
 * on top of {@link MecanumDrive} instead of inline math.
 *
 * <p>The point of this OpMode is to verify that the extracted helper feels
 * identical to the inline version. If you can drive this and it responds
 * the same way as "Basic TeleOp" - same slow-mode, same reset button, same
 * field-oriented behavior - the helper is correct.
 *
 * <p>Only the drivetrain is exercised here. Intake is deliberately absent
 * so the test surface is small.
 */
@TeleOp(name = "Mecanum Helper Parity", group = "drive")
public class MecanumDriveHelperTeleOp extends LinearOpMode {

    private static final double SLOW_MODE_SCALE = 0.35;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-90.0, -12.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Point robot down-field, then press Start.");
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
            double heading = pose.getHeading(AngleUnit.RADIANS);

            double fieldForward = -gamepad1.left_stick_y;
            double fieldRight   =  gamepad1.left_stick_x;
            double yaw          =  gamepad1.right_stick_x;

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            fieldForward *= scale;
            fieldRight   *= scale;
            yaw          *= scale;

            drive.driveFieldOriented(fieldForward, fieldRight, yaw, heading);

            telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "normal");
            telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Field", "fwd=%.2f right=%.2f yaw=%.2f",
                    fieldForward, fieldRight, yaw);
            telemetry.update();
        }

        drive.stop();
    }
}
