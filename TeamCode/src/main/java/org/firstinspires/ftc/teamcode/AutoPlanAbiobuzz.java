package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import java.util.Locale;

@SuppressWarnings({"unused", "SpellCheckingInspection"}) // FTC SDK entry point and registered name.
@Autonomous(name = "AutoPlanAbiobuzz", group = "BioBuzz")
public class AutoPlanAbiobuzz extends LinearOpMode {
    private static final long TELEMETRY_INTERVAL_NS = 100_000_000L;
    private static final double DRIVE_POWER = 0.5;
    private static final double LAUNCH_SECONDS = 7.0;

    private long lastTelemetryTimeNs;
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private final Launch launcher = new Launch();
    private String currentStep = "Waiting for Start";
    private String lastTurnResult = "Waiting for first turn";

    // Edit the route here. Each move takes a direction and duration in seconds.
    // FORWARD is toward the robot's starting left side, just like in TeleOp.
    private void runRoute(double startingHeading) {
        launch();
        turnToAngle(-10, 1.0);
        move(Direction.BACK, 1.0);
        intakeOn();
        intakeOff();
        //noinspection ConstantValue
        if (true) return; // Temporary route stop.
        move(Direction.FORWARD_LEFT, 0.5);
        turnToAngle(startingHeading, 2.0);
        waitSeconds(1.0);
        move(Direction.LEFT, 1.3);
        turnToAngle(180,1.0);
        waitSeconds(5.0);
    }

    // Turn angles are degrees from the startup yaw reset. Positive turns left.
    // Example: turnToAngle(90, 2.0) gives the turn up to two seconds.
    private void turnToAngle(double degrees, double timeoutSeconds) {
        showStep(String.format(Locale.ROOT, "Turn to %.0f degrees", degrees));
        boolean reachedTarget = drive.turnToAngle(degrees, milliseconds(timeoutSeconds), this,
                this::updateTelemetryDuringMotion);
        lastTurnResult = String.format(Locale.ROOT, "Target %.1f°, measured %.1f° (%s)",
                degrees, drive.getHeadingDegrees(), reachedTarget ? "reached" : "not reached");
    }

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        BACK(-1, 0), LEFT(0, -1), FORWARD_LEFT(0.5, -0.5);

        final double forward;
        final double strafe;

        Direction(double forward, double strafe) {
            this.forward = forward;
            this.strafe = strafe;
        }
    }

    private void move(Direction direction, double seconds) {
        showStep(String.format(Locale.ROOT, "Move %s at %.0f%% for %.1f s",
                direction.name(), DRIVE_POWER * 100, seconds));
        drive.driveFieldRelativeForMS(direction.forward * DRIVE_POWER,
                direction.strafe * DRIVE_POWER, 0, milliseconds(seconds), this,
                this::updateTelemetryDuringMotion);
    }

    private void launch() {
        showStep(String.format(Locale.ROOT, "Launch for %.1f s", LAUNCH_SECONDS));
        launcher.launchForMS(milliseconds(LAUNCH_SECONDS), intake, this, this::updateTelemetry);
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
            showStep(String.format(Locale.ROOT, "Wait %.1f s", seconds));
            sleep(milliseconds(seconds));
        }
    }

    private void showStep(String step) {
        currentStep = step;
        updateTelemetry();
    }

    private void updateTelemetryDuringMotion() {
        if (System.nanoTime() - lastTelemetryTimeNs >= TELEMETRY_INTERVAL_NS) {
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        lastTelemetryTimeNs = System.nanoTime();
        telemetry.addData("Current step", currentStep);
        telemetry.addData("Last turn", lastTurnResult);
        telemetry.addData("IMU yaw (deg)", drive.getHeadingDegrees());
        launcher.addTelemetry(telemetry);
        telemetry.update();
    }

    private long milliseconds(double seconds) {
        return Math.max(0, Math.round(seconds * 1000));
    }
}
