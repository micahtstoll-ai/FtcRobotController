package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Odometry calibration autonomous. During init, prompts a pod-direction check
 * (push the robot around and confirm X / Y signs). On start, drives a 36-inch
 * square in cardinal directions and then eight diagonal legs, using Pinpoint
 * closed-loop for cross-track and heading correction. Encoder-direction and
 * motor-direction bugs both show up as the square failing to close.
 */
@Autonomous(name = "Auto: Calibrate Odometry")
public class AutoCalibrateOdometry extends LinearOpMode {

    private static final double POWER        = 0.3;
    private static final double SIDE_IN      = 36.0;
    private static final double HALF_DIAG_IN = 18.0 * Math.sqrt(2.0);

    private static final double RAMP_INCHES    = 6.0;
    private static final double MIN_POWER_FRAC = 0.30;
    private static final double CROSS_TRACK_KP = 0.20;
    private static final double HEADING_KP     = 0.03;
    private static final long   SETTLE_MS      = 100;

    private MecanumDrive drive;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        drive    = new MecanumDrive(hardwareMap);
        pinpoint = RobotHardware.getPinpoint(hardwareMap);
        pinpoint.resetPosAndIMU();

        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            telemetry.addLine("Push robot to verify pod directions:");
            telemetry.addLine("  forward  -> X should INCREASE");
            telemetry.addLine("  left     -> Y should INCREASE");
            telemetry.addLine("If either goes the wrong way, flip that pod's");
            telemetry.addLine("EncoderDirection in RobotHardware.");
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

        drive.stop();
    }

    /**
     * Drive along a robot-frame axis-aligned or diagonal direction for a given
     * distance, correcting cross-track error and heading drift with proportional
     * feedback from Pinpoint. axial = forward, lateral = right (unit-magnitude
     * per component; normalized inside).
     */
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
            while (headingErr >  180) headingErr -= 360;
            while (headingErr < -180) headingErr += 360;
            double yawCorr = HEADING_KP * headingErr;

            drive.drive(axial   * speedScale + axialCorr,
                        lateral * speedScale + lateralCorr,
                        yawCorr,
                        POWER);

            telemetry.addData("Target / Progress (in)", "%.2f / %.2f",
                    distanceInches, progress);
            telemetry.addData("Cross-track (in)", "%.2f", crossTrack);
            telemetry.addData("Heading err (deg)", "%.2f", headingErr);
            telemetry.addData("Speed scale", "%.2f", speedScale);
            telemetry.update();
        }

        drive.stop();
        if (opModeIsActive()) sleep(SETTLE_MS);
    }
}
