package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "AutoLaunchOnly", group = "BioBuzz")
public class AutoLaunchOnly extends LinearOpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private final Launch launcher = new Launch();
    private String currentStep = "Waiting for Start";

    // Edit the route here. Each move takes (direction, power from 0 to 1, seconds).
    // FORWARD is toward the robot's starting left side, just like in TeleOp.
    private void runRoute(double startingHeading) {
        launchFor(30.0);
//        move(Direction.BACK_LEFT, 0.5, 1.0);
//        waitSeconds(0.2);
//        intakeOn();
//        move(Direction.BACK_RIGHT, 0.5, 1.35);
//        waitSeconds(1.0); // The intake stays on during this pause.
//        intakeOff();
//        move(Direction.FORWARD_LEFT, 0.5, 0.5);
//        turnToAngle(startingHeading, 2.0);
//        waitSeconds(1.0);
//        move(Direction.LEFT,0.5,1.3);
//        turnToAngle(180,1.0);
//        waitSeconds(5.0);
    }

    // Turn angles are degrees from the startup yaw reset. Positive turns left.
    // Example: turnToAngle(90, 2.0) gives the turn up to two seconds.
    private void turnToAngle(double degrees, double timeoutSeconds) {
        showStep(String.format("Turn to %.0f degrees", degrees));
        drive.turnToAngle(degrees, milliseconds(timeoutSeconds), this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);

        updateTelemetry();
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        drive.resetHeading();
        try {
            runRoute(drive.getHeadingDegrees());
        } finally {
            intake.intakeOff();
            showStep(opModeIsActive() ? "Route complete" : "Stopped");
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
        showStep(String.format("Move %s at %.0f%% for %.1f s",
                direction.name(), drivePower * 100, seconds));
        drive.driveFieldRelativeForMS(direction.forward * drivePower,
                direction.strafe * drivePower, 0, milliseconds(seconds), this);
    }

    private void launchFor(double seconds) {
        showStep(String.format("Launch for %.1f s", seconds));
        launcher.launchForMS(milliseconds(seconds), intake, this, this::updateTelemetry);
    }

    private void intakeOn() {
        if (opModeIsActive()) {
            intake.intakeOn();
            showStep("Intake on");
        }
    }

    private void intakeOff() {
        intake.intakeOff();
        showStep("Intake off");
    }

    private void waitSeconds(double seconds) {
        if (opModeIsActive()) {
            showStep(String.format("Wait %.1f s", seconds));
            sleep(milliseconds(seconds));
        }
    }

    private void showStep(String step) {
        currentStep = step;
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("Current step", currentStep);
        launcher.addTelemetry(telemetry);
        telemetry.update();
    }

    private long milliseconds(double seconds) {
        return Math.max(0, Math.round(seconds * 1000));
    }
}
