package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.BallClusterResult;
import org.firstinspires.ftc.teamcode.vision.LimelightClusterCamera;

/**
 * Runs nothing but the Limelight, so a driver can verify the camera pipeline
 * is up and seeing balls before any of the hunting logic gets wired in.
 *
 * <p>How to use:
 * <ol>
 *   <li>Configure the Limelight in the Driver Station under the name
 *       {@code "limelight"}.</li>
 *   <li>Load the Python ball-cluster pipeline in slot 0 of the Limelight web
 *       UI. (See micahtstoll-ai/limelight for the script.)</li>
 *   <li>Init this OpMode, hit Start, and point the camera at a pile of
 *       yellow balls.</li>
 *   <li>Read the Driver Station telemetry - if it says how many clusters
 *       and how many total balls are in view, the vision stack is
 *       working.</li>
 * </ol>
 *
 * <p>Does not touch the drivetrain or any actuator, so it is always safe to run.
 */
@TeleOp(name = "Camera Diagnostic", group = "vision")
public class CameraDiagnosticTeleOp extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";
    // Ball-cluster Python pipeline lives in slot 1 (slot 0 is a leftover
    // from last season). Must match HuntConfig.LIMELIGHT_PIPELINE_ID and
    // LimelightPipelineScheduler.CLUSTER_PIPELINE_ID.
    private static final int    LIMELIGHT_PIPELINE_ID = 1;

    @Override
    public void runOpMode() {
        LimelightClusterCamera camera = new LimelightClusterCamera(
                hardwareMap, LIMELIGHT_NAME, LIMELIGHT_PIPELINE_ID);

        telemetry.addLine("Camera ready. Press Start.");
        telemetry.update();

        waitForStart();
        camera.start();

        while (opModeIsActive()) {
            BallClusterResult result = camera.latest();
            telemetry.addData("Total balls in view", result.getTotalBalls());
            telemetry.addData("Clusters reported", result.getClusters().size());
            if (result.hasTarget()) {
                telemetry.addLine("--- clusters (best first) ---");
                for (int i = 0; i < result.getClusters().size(); i++) {
                    telemetry.addData("[" + i + "]", result.getClusters().get(i).toString());
                }
            } else {
                telemetry.addLine("No clusters this frame.");
            }
            telemetry.update();
        }

        camera.stop();
    }
}
