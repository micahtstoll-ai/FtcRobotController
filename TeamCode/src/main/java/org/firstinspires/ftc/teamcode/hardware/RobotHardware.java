package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Single source of truth for hardware-map names and physical robot configuration.
 * Any change to a hardware string or wiring convention (motor direction, odometry
 * pod offset, encoder direction) belongs here and nowhere else. OpModes, subsystems,
 * and Pedro Pathing constants all read from this class.
 */
public final class RobotHardware {

    private RobotHardware() {}

    // Hardware-map configuration names. These must match the Driver Station
    // config on the Control Hub; renaming here without renaming there fails at init.
    public static final String LEFT_FRONT  = "leftFront";
    public static final String RIGHT_FRONT = "rightFront";
    public static final String LEFT_REAR   = "leftRear";
    public static final String RIGHT_REAR  = "rightRear";
    public static final String INTAKE      = "intake";
    public static final String PINPOINT    = "pinpoint";

    // Pinpoint pod geometry, in millimeters. Signs verified with AutoCalibrateOdometry:
    // push forward -> X increases, push left -> Y increases.
    public static final double PINPOINT_FORWARD_POD_Y_MM = -90.0;
    public static final double PINPOINT_STRAFE_POD_X_MM  = -12.0;

    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Right side of the mecanum is reversed on this robot.
    public static final DcMotorSimple.Direction LEFT_FRONT_DIR  = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction LEFT_REAR_DIR   = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_REAR_DIR  = DcMotorSimple.Direction.REVERSE;

    /** Bundle of the four mecanum drive motors, direction-configured and BRAKE-set. */
    public static final class DriveMotors {
        public final DcMotorEx leftFront;
        public final DcMotorEx rightFront;
        public final DcMotorEx leftRear;
        public final DcMotorEx rightRear;

        DriveMotors(DcMotorEx lf, DcMotorEx rf, DcMotorEx lr, DcMotorEx rr) {
            this.leftFront  = lf;
            this.rightFront = rf;
            this.leftRear   = lr;
            this.rightRear  = rr;
        }
    }

    public static DriveMotors getDriveMotors(HardwareMap hardwareMap) {
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, LEFT_REAR);
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, RIGHT_REAR);

        lf.setDirection(LEFT_FRONT_DIR);
        rf.setDirection(RIGHT_FRONT_DIR);
        lr.setDirection(LEFT_REAR_DIR);
        rr.setDirection(RIGHT_REAR_DIR);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return new DriveMotors(lf, rf, lr, rr);
    }

    /**
     * Intake configured for closed-loop velocity control with FLOAT zero-power
     * behavior so game pieces can free-wheel through the intake when unpowered.
     */
    public static DcMotorEx getIntake(HardwareMap hardwareMap) {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, INTAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return intake;
    }

    /**
     * Fully-configured Pinpoint odometry driver. Caller is responsible for the
     * initial {@code resetPosAndIMU()} at the pose they consider field-forward.
     */
    public static GoBildaPinpointDriver getPinpoint(HardwareMap hardwareMap) {
        GoBildaPinpointDriver pp = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);
        pp.setOffsets(PINPOINT_FORWARD_POD_Y_MM, PINPOINT_STRAFE_POD_X_MM, DistanceUnit.MM);
        pp.setEncoderResolution(PINPOINT_POD_TYPE);
        pp.setEncoderDirections(PINPOINT_FORWARD_DIR, PINPOINT_STRAFE_DIR);
        return pp;
    }
}
