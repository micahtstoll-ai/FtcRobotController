package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Pedro Pathing configuration bound to {@link RobotHardware}. All pod and motor
 * values are sourced from RobotHardware so a wiring change updates every consumer
 * (OpModes, subsystems, and Pedro) from one place.
 */
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants();

    // PinpointConstants renames the goBILDA driver's first/second setOffsets args
    // as forwardPodY / strafePodX; values pass through unchanged.
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(RobotHardware.PINPOINT_FORWARD_POD_Y_MM)
            .strafePodX(RobotHardware.PINPOINT_STRAFE_POD_X_MM)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(RobotHardware.PINPOINT)
            .encoderResolution(RobotHardware.PINPOINT_POD_TYPE)
            .forwardEncoderDirection(RobotHardware.PINPOINT_FORWARD_DIR)
            .strafeEncoderDirection(RobotHardware.PINPOINT_STRAFE_DIR);

    // Pedro's MecanumConstants defaults are flipped from this robot's wiring;
    // override every direction explicitly from RobotHardware.
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorDirection(RobotHardware.LEFT_FRONT_DIR)
            .leftRearMotorDirection(RobotHardware.LEFT_REAR_DIR)
            .rightFrontMotorDirection(RobotHardware.RIGHT_FRONT_DIR)
            .rightRearMotorDirection(RobotHardware.RIGHT_REAR_DIR);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointConstants)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
