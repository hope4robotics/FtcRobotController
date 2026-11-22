package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import java.util.Locale;

@SuppressWarnings("unused") // Discovered by the FTC SDK through @Autonomous.
@Autonomous(name = "AutoTesting", group = "BioBuzz")
public class AutoTesting extends LinearOpMode {
    private static final long TELEMETRY_INTERVAL_NS = 100_000_000L;
    private static final double TEST_HEADING_DEGREES = -10.0;
    private static final double DRIVE_POWER = 0.5;
    private static final long DRIVE_DURATION_MS = 1_000;

    private long lastTelemetryTimeNs;
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private final Launch launcher = new Launch();
    private String currentStep = "Waiting for Start";
    private String lastTurnResult = "Waiting for first turn";

    private String runRoute(double startingHeading) {
        showStep("Launch for 7 s");
        launcher.launchForMS(7_000, intake, this, this::updateTelemetryDuringMotion);
        if (!turnToAngle(TEST_HEADING_DEGREES, 1_000)) {
            return "Aborted: test heading not reached";
        }
        if (moveInterrupted("Back", -DRIVE_POWER) || moveInterrupted("Return", DRIVE_POWER)) {
            return "Aborted: timed move interrupted";
        }
        return turnToAngle(startingHeading, 2_000)
                ? "Route complete (timed return)"
                : "Return drive complete: starting heading not reached";
    }

    private boolean turnToAngle(double degrees, long timeoutMs) {
        if (!opModeIsActive()) {
            return false;
        }
        showStep(String.format(Locale.ROOT, "Turn to %.0f degrees", degrees));
        boolean reachedTarget = drive.turnToAngle(degrees, timeoutMs, this,
                this::updateTelemetryDuringMotion);
        lastTurnResult = String.format(Locale.ROOT, "Target %.1f°, measured %.1f° (%s)",
                degrees, drive.getHeadingDegrees(), reachedTarget ? "reached" : "not reached");
        return reachedTarget;
    }

    private boolean moveInterrupted(String step, double fieldForwardPower) {
        // Field-forward points toward the starting left side of the robot.
        showStep(step);
        return !drive.driveFieldRelativeHoldingHeading(fieldForwardPower, TEST_HEADING_DEGREES,
                DRIVE_DURATION_MS, this, this::updateTelemetryDuringMotion);
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
        String result = "Route interrupted";
        try {
            result = runRoute(drive.getHeadingDegrees());
        } finally {
            intake.intakeOff();
            showStep(isStopRequested() ? "Stopped" : result);
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

}
