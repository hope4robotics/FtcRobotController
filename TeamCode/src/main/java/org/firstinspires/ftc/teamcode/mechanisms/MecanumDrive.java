
package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

    // Positive offsets rotate field-forward counterclockwise from the nose.
    private static final double FIELD_FORWARD_OFFSET_DEGREES = 90.0;

    private static final double TURN_TOLERANCE_DEGREES = 2.0;
    private static final double TURN_POWER_PER_DEGREE = 0.02;
    private static final double MAX_TURN_POWER = 0.5;

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private IMU imu;


    public void init(HardwareMap hwMap) {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        frontLeftMotor = hwMap.get(DcMotor.class, "left_front_drive");
        frontRightMotor = hwMap.get(DcMotor.class, "right_front_drive");
        backLeftMotor = hwMap.get(DcMotor.class, "left_back_drive");
        backRightMotor = hwMap.get(DcMotor.class, "right_back_drive");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);

        /*
         * set up the IMU so we can field oriented drive
         */
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(frontLeftPower / maxPower);
        backLeftMotor.setPower(backLeftPower / maxPower);
        frontRightMotor.setPower(frontRightPower / maxPower);
        backRightMotor.setPower(backRightPower / maxPower);

    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        driveFieldRelative(forward, strafe, rotate,
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    private void driveFieldRelative(double forward, double strafe, double rotate, double yaw) {
        double theta = Math.atan2(forward, strafe) + Math.toRadians(FIELD_FORWARD_OFFSET_DEGREES);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - yaw);
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);

    }

    /** Runs a timed field-relative move while holding the specified yaw. */
    public boolean driveFieldRelativeHoldingHeading(double forward, double headingDegrees,
                                                     long durationMs, LinearOpMode opMode,
                                                     Runnable onDriveUpdate) {
        ElapsedTime timer = new ElapsedTime();
        try {
            while (opMode.opModeIsActive() && timer.milliseconds() < durationMs) {
                double yaw = getHeadingDegrees();
                if (!Double.isFinite(yaw) || !Double.isFinite(headingDegrees)) {
                    return false;
                }
                double error = AngleUnit.normalizeDegrees(headingDegrees - yaw);
                double rotate = Range.clip(-error * TURN_POWER_PER_DEGREE, -0.2, 0.2);
                driveFieldRelative(forward, 0, rotate, Math.toRadians(yaw));
                onDriveUpdate.run();
            }
            return opMode.opModeIsActive();
        } finally {
            drive(0, 0, 0);
        }
    }

    /** Sets the current robot heading as zero for field-relative driving. */
    public void resetHeading() {
        imu.resetYaw();
    }

    /** Returns the current IMU yaw in degrees, positive counterclockwise. */
    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Turns to an absolute IMU yaw in degrees. Positive yaw is counterclockwise.
     * Call after start. Returns true when the heading is within tolerance.
     */
    public boolean turnToAngle(double targetAngleDegrees, long timeoutMs, LinearOpMode opMode) {
        return turnToAngle(targetAngleDegrees, timeoutMs, opMode, () -> {});
    }

    public boolean turnToAngle(double targetAngleDegrees, long timeoutMs, LinearOpMode opMode,
                               Runnable onDriveUpdate) {
        ElapsedTime timer = new ElapsedTime();
        try {
            while (opMode.opModeIsActive() && timer.milliseconds() < timeoutMs) {
                double yawDegrees = getHeadingDegrees();
                if (!Double.isFinite(yawDegrees) || !Double.isFinite(targetAngleDegrees)) {
                    return false;
                }
                double errorDegrees = AngleUnit.normalizeDegrees(targetAngleDegrees - yawDegrees);
                if (Math.abs(errorDegrees) <= TURN_TOLERANCE_DEGREES) {
                    return true;
                }

                // Positive drive rotation is clockwise, opposite positive IMU yaw.
                double rotate = Range.clip(-errorDegrees * TURN_POWER_PER_DEGREE,
                        -MAX_TURN_POWER, MAX_TURN_POWER);
                drive(0, 0, rotate);
                onDriveUpdate.run();
            }
            return false;
        } finally {
            drive(0, 0, 0);
        }
    }

    /**
     * Drives relative to the field until the duration expires or Stop is pressed.
     * Call after start. Positive inputs drive forward, right, and clockwise.
     */
    public void driveFieldRelativeForMS(double forward, double strafe, double rotate,
                                        long durationMs, LinearOpMode opMode) {
        driveFieldRelativeForMS(forward, strafe, rotate, durationMs, opMode, () -> {});
    }

    public void driveFieldRelativeForMS(double forward, double strafe, double rotate,
                                        long durationMs, LinearOpMode opMode, Runnable onDriveUpdate) {
        ElapsedTime timer = new ElapsedTime();
        try {
            while (opMode.opModeIsActive() && timer.milliseconds() < durationMs) {
                driveFieldRelative(forward, strafe, rotate);
                onDriveUpdate.run();
            }
        } finally {
            drive(0, 0, 0);
        }
    }

}