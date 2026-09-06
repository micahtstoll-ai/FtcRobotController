package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hunt.BlendController;
import org.firstinspires.ftc.teamcode.hunt.ClusterWorldModel;
import org.firstinspires.ftc.teamcode.hunt.CoarseApproachStrategy;
import org.firstinspires.ftc.teamcode.hunt.DirectVelocityCoarseApproach;
import org.firstinspires.ftc.teamcode.hunt.DriveCommand;
import org.firstinspires.ftc.teamcode.hunt.HuntConfig;
import org.firstinspires.ftc.teamcode.hunt.IntakeHunter;
import org.firstinspires.ftc.teamcode.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.vision.BallClusterResult;
import org.firstinspires.ftc.teamcode.vision.LimelightClusterCamera;

/**
 * The full driver-assist ball-hunting TeleOp - vision, world model, state
 * machine, drivetrain, intake, driver blend, and rumble all wired up.
 *
 * <p>When the driver is NOT holding LT, this behaves like a plain
 * field-oriented mecanum TeleOp - same feel as BasicTeleOp.
 *
 * <p>When the driver HOLDS LT past the trigger threshold, the robot
 * activates the hunter:
 * <ol>
 *   <li>Ask the Limelight what clusters of yellow balls it sees.</li>
 *   <li>Update the world model that remembers cluster positions.</li>
 *   <li>Let {@link IntakeHunter} decide how the robot should move and
 *       whether the intake should be running.</li>
 *   <li>Blend that decision with what the driver is doing on the sticks:
 *       any stick past a small deadzone overrides that axis; other axes
 *       stay under the hunter's control.</li>
 *   <li>Rumble the controller when the hunter decides the field is clear.</li>
 * </ol>
 *
 * <p>Marked {@link Disabled} on purpose. Remove the annotation once Pedro
 * is finalized and the team is ready to try this on the robot.
 *
 * <p>Also preserved from BasicTeleOp: right bumper = slow mode, Options
 * or B = reset Pinpoint pose + IMU (same as BasicTeleOp on master).
 */
@TeleOp(name = "Limelight Intake Assist", group = "test")
@Disabled
public class LimelightIntakeAssistTeleOp extends LinearOpMode {

    private static final double SLOW_MODE_SCALE = 0.35;
    private static final double LT_HOLD_THRESHOLD = 0.5;

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

        LimelightClusterCamera camera = new LimelightClusterCamera(
                hardwareMap, HuntConfig.LIMELIGHT_NAME, HuntConfig.LIMELIGHT_PIPELINE_ID);
        IntakeSubsystem intake = new IntakeSubsystem(
                hardwareMap, HuntConfig.INTAKE_MOTOR_NAME, DcMotorSimple.Direction.FORWARD);

        ClusterWorldModel world = new ClusterWorldModel();
        CoarseApproachStrategy coarse = new DirectVelocityCoarseApproach();
        IntakeHunter hunter = new IntakeHunter(world, coarse);

        telemetry.addLine("Ready. Point robot down-field, then start.");
        telemetry.addLine("Hold LT to hunt. Release LT for normal drive.");
        telemetry.update();

        waitForStart();
        camera.start();

        boolean prevResetButton = false;
        boolean prevAssistActive = false;
        boolean rumbleFired = false;

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

            boolean assistActive = gamepad1.left_trigger > LT_HOLD_THRESHOLD;

            if (assistActive != prevAssistActive) {
                hunter.reset();
                rumbleFired = false;
                if (!assistActive) intake.stop();
            }
            prevAssistActive = assistActive;

            if (!assistActive) {
                drive.driveFieldOriented(fieldForward, fieldRight, yaw, heading);
                intake.stop();
                intake.update();
                telemetryOff(pose, heading);
                continue;
            }

            // Assist active. Rotate driver's stick into robot frame so the
            // blender can compare like-for-like with the hunter's
            // robot-frame command. Same rotation math as MecanumDrive uses.
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double driverAxial   = fieldForward * cos - fieldRight * sin;
            double driverLateral = fieldForward * sin + fieldRight * cos;

            BallClusterResult detections = camera.latest();
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            DriveCommand hunterCmd = hunter.update(
                    detections, robotX, robotY, heading, System.nanoTime());

            DriveCommand finalCmd = BlendController.blend(
                    driverAxial, driverLateral, yaw, hunterCmd);
            drive.driveRobotOriented(finalCmd.axial, finalCmd.lateral, finalCmd.yaw);
            if (finalCmd.intakeOn) {
                intake.setTargetRpm(HuntConfig.INTAKE_HUNT_RPM);
            } else {
                intake.stop();
            }
            intake.update();

            if (hunter.state() == IntakeHunter.State.DONE_RUMBLE && !rumbleFired) {
                gamepad1.rumble(500);
                rumbleFired = true;
            }

            telemetryOn(pose, heading, hunter, detections, intake, finalCmd);
        }

        camera.stop();
        intake.stop();
        intake.update();
        drive.stop();
    }

    private void telemetryOff(Pose2D pose, double heading) {
        telemetry.addData("Assist", "off");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Pose (in)", "x=%.1f y=%.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
        telemetry.update();
    }

    private void telemetryOn(Pose2D pose, double heading, IntakeHunter hunter,
                             BallClusterResult detections, IntakeSubsystem intake,
                             DriveCommand cmd) {
        telemetry.addData("Assist", "ACTIVE (LT held)");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Pose (in)", "x=%.1f y=%.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
        telemetry.addData("Hunter state", hunter.state().name());
        telemetry.addData("Target cluster id", hunter.currentTargetId());
        telemetry.addData("Camera sees", "%d clusters, %d balls total",
                detections.getClusters().size(), detections.getTotalBalls());
        if (detections.hasTarget()) {
            telemetry.addData("Best cluster", detections.getBestCluster().toString());
        }
        telemetry.addData("Drive", cmd.toString());
        telemetry.addData("Intake (rpm)", "cmd=%.0f actual=%.0f",
                intake.commandedRpm(), intake.actualRpm());
        telemetry.update();
    }
}
