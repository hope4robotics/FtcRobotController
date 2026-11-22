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
    private static final double BACK_HEADING_DEGREES = -10.0;
    private static final double START_HEADING_DEGREES = 0.0;

    private long lastTelemetryTimeNs;
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private final Launch launcher = new Launch();
    private String currentStep = "Waiting for Start";
    private String lastTurnResult = "Waiting for first turn";

    // Edit the route here. Power is a fraction from 0 to 1. Durations are in seconds.
    private boolean runRoute() {
        launchFor(7.0);
        if (!turnToAngle(BACK_HEADING_DEGREES, 2.0)) {
            return false;
        }
        if (!driveFieldBack(0.10, 2.5, BACK_HEADING_DEGREES)) {
            return false;
        }
        if (!turnToAngle(START_HEADING_DEGREES, 2.0)) {
            return false;
        }
        move(Direction.FORWARD_LEFT, 0.20, 1.0);
        move(Direction.LEFT, 0.50, 2.0);
        return turnToAngle(START_HEADING_DEGREES, 2.0);
    }

    // Turn angles are degrees from the startup yaw reset. Positive turns left.
    // Example: turnToAngle(90, 2.0) gives the turn up to two seconds.
    private boolean turnToAngle(double degrees, double timeoutSeconds) {
        if (!opModeIsActive()) {
            return false;
        }
        showStep(String.format(Locale.ROOT, "Turn to %.0f degrees", degrees));
        boolean reachedTarget = drive.turnToAngle(degrees, milliseconds(timeoutSeconds), this,
                this::updateTelemetryDuringMotion);
        lastTurnResult = String.format(Locale.ROOT, "Target %.1f°, measured %.1f° (%s)",
                degrees, drive.getHeadingDegrees(), reachedTarget ? "reached" : "not reached");
        return reachedTarget;
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
        boolean completed = false;
        try {
            completed = runRoute();
        } finally {
            drive.drive(0, 0, 0);
            intake.intakeOff();
            showStep(isStopRequested() ? "Stopped" : completed ? "Route complete" : "Route interrupted");
        }
    }

    private enum Direction {
        FORWARD_LEFT(Math.sqrt(0.5), -Math.sqrt(0.5)), LEFT(0, -1);

        final double forward;
        final double strafe;

        Direction(double forward, double strafe) {
            this.forward = forward;
            this.strafe = strafe;
        }
    }

    private void move(Direction direction, double power, double seconds) {
        if (!opModeIsActive()) {
            return;
        }
        showStep(String.format(Locale.ROOT, "Move %s at %.0f%% for %.1f s",
                direction.name(), power * 100, seconds));
        drive.driveFieldRelativeForMS(direction.forward * power, direction.strafe * power,
                0, milliseconds(seconds), this, this::updateTelemetryDuringMotion);
    }

    private boolean driveFieldBack(double power, double seconds, double headingDegrees) {
        showStep(String.format(Locale.ROOT, "Move field BACK at %.0f%% for %.1f s",
                power * 100, seconds));
        return drive.driveFieldRelativeHoldingHeading(-power, headingDegrees,
                milliseconds(seconds), this, this::updateTelemetryDuringMotion);
    }

    private void launchFor(double seconds) {
        showStep(String.format(Locale.ROOT, "Launch for %.1f s", seconds));
        launcher.launchForMS(milliseconds(seconds), intake, this, this::updateTelemetryDuringMotion);
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
