package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hunt.ClusterWorldModel;
import org.firstinspires.ftc.teamcode.hunt.CoarseApproachStrategy;
import org.firstinspires.ftc.teamcode.hunt.DirectVelocityCoarseApproach;
import org.firstinspires.ftc.teamcode.hunt.DriveCommand;
import org.firstinspires.ftc.teamcode.hunt.HuntConfig;
import org.firstinspires.ftc.teamcode.hunt.IntakeHunter;
import org.firstinspires.ftc.teamcode.vision.BallClusterResult;
import org.firstinspires.ftc.teamcode.vision.LimelightClusterCamera;

/**
 * Runs the entire hunter brain - vision, world model, state machine -
 * WITHOUT touching the wheels or the intake. Prints what it would do.
 *
 * <p>The point is to validate the state transitions, the world model, and
 * the drive commands the hunter proposes, before we trust it to actually
 * move the robot. Hold LT to activate the hunter; watch the telemetry:
 * <ul>
 *   <li>Which state it is in</li>
 *   <li>How many clusters it remembers</li>
 *   <li>The DriveCommand it would send</li>
 *   <li>Whether it thinks the intake should be running</li>
 * </ul>
 *
 * <p>Because it does not command the drivetrain or the intake, this OpMode
 * is safe to run in test space with the robot on the ground. Push the
 * robot around by hand and watch the hunter react.
 */
@TeleOp(name = "Hunter Dry Run", group = "vision")
public class HunterDryRunTeleOp extends LinearOpMode {

    private static final double LT_HOLD_THRESHOLD = 0.5;

    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-90.0, -12.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        LimelightClusterCamera camera = new LimelightClusterCamera(
                hardwareMap, HuntConfig.LIMELIGHT_NAME, HuntConfig.LIMELIGHT_PIPELINE_ID);

        ClusterWorldModel world = new ClusterWorldModel();
        CoarseApproachStrategy coarse = new DirectVelocityCoarseApproach();
        IntakeHunter hunter = new IntakeHunter(world, coarse);

        telemetry.addLine("Dry-run: hunter runs but nothing moves.");
        telemetry.addLine("Hold LT to activate the hunter and watch its plan.");
        telemetry.update();

        waitForStart();
        camera.start();

        boolean prevActive = false;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double heading = pose.getHeading(AngleUnit.RADIANS);
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);

            BallClusterResult detections = camera.latest();

            boolean active = gamepad1.left_trigger > LT_HOLD_THRESHOLD;
            if (active && !prevActive) hunter.reset();
            if (!active && prevActive) hunter.reset();
            prevActive = active;

            DriveCommand cmd;
            if (active) {
                cmd = hunter.update(detections, robotX, robotY, heading, System.nanoTime());
            } else {
                cmd = DriveCommand.STOP;
            }

            telemetry.addData("LT held", active);
            telemetry.addData("Hunter state", hunter.state().name());
            telemetry.addData("Target cluster id", hunter.currentTargetId());
            telemetry.addData("Camera sees", "%d clusters, %d balls",
                    detections.getClusters().size(), detections.getTotalBalls());
            telemetry.addData("World remembers", world.clusters().size() + " clusters");
            telemetry.addData("Would drive", cmd.toString());
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f",
                    robotX, robotY, Math.toDegrees(heading));
            telemetry.update();
        }

        camera.stop();
    }
}
