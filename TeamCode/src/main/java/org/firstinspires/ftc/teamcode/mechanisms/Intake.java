
package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@Disabled
public class Intake {

    // Declare OpMode members.
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(HardwareMap hwMap) {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        intake = hwMap.get(DcMotor.class, "intake");
        leftIntakeServo = hwMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hwMap.get(CRServo.class, "right_intake_servo");

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        intake.setZeroPowerBehavior(BRAKE);
        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

    }

    public void intake(double in, double out) {
        intakePower = in - out;

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

    }

    public void intakeOn(){
        intake.setPower(1);
        leftIntakeServo.setPower(1);
        rightIntakeServo.setPower(1);
    }

    public void intakeOff(){
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    public void intakeOutOn(){
        intake.setPower(-1);
        leftIntakeServo.setPower(-1);
        rightIntakeServo.setPower(-1);
    }
}