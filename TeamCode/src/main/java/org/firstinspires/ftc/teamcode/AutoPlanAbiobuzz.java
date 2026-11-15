package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "AutoPlanAbiobuzz", group = "BioBuzz")
public class AutoPlanAbiobuzz extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private final Launch launcher = new Launch();

    // Edit the route here. Each move takes (direction, power from 0 to 1, seconds).
    // FORWARD is toward the robot's starting left side, just like in TeleOp.
    private void runRoute(double startingHeading) {
        launchFor(10.0);
        move(Direction.BACK_LEFT, 0.5, 1.0);
        intakeOn();
        move(Direction.FORWARD_LEFT, 0.5, 1.0);
        waitSeconds(1.0); // The intake stays on during this pause.
        intakeOff();
        move(Direction.BACK_LEFT, 0.5, 1.0);
        turnToAngle(startingHeading, 2.0);
    }

    // Turn angles are degrees from the startup yaw reset. Positive turns left.
    // Example: turnToAngle(90, 2.0) gives the turn up to two seconds.
    private void turnToAngle(double degrees, double timeoutSeconds) {
        drive.turnToAngle(degrees, milliseconds(timeoutSeconds), this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        drive.resetHeading();
        try {
            runRoute(drive.getHeadingDegrees());
        } finally {
            intakeOff();
        }
    }

    private enum Direction {
        FORWARD(1, 0), BACK(-1, 0), LEFT(0, -1), RIGHT(0, 1),
        FORWARD_LEFT(0.5, -0.5), FORWARD_RIGHT(0.5, 0.5),
        BACK_LEFT(-0.5, -0.5), BACK_RIGHT(-0.5, 0.5);

        final double forward;
        final double strafe;

        Direction(double forward, double strafe) {
            this.forward = forward;
            this.strafe = strafe;
        }
    }

    private void move(Direction direction, double power, double seconds) {
        double drivePower = Range.clip(power, 0.0, 1.0);
        drive.driveFieldRelativeForMS(direction.forward * drivePower,
                direction.strafe * drivePower, 0, milliseconds(seconds), this);
    }

    private void launchFor(double seconds) {
        launcher.launchForMS(milliseconds(seconds), intake, this);
    }

    private void intakeOn() {
        if (opModeIsActive()) {
            intake.intakeOn();
        }
    }

    private void intakeOff() {
        intake.intakeOff();
    }

    private void waitSeconds(double seconds) {
        if (opModeIsActive()) {
            sleep(milliseconds(seconds));
        }
    }

    private long milliseconds(double seconds) {
        return Math.max(0, Math.round(seconds * 1000));
    }
}
