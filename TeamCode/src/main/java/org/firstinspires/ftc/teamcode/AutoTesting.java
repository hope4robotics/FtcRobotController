package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

@SuppressWarnings("unused") // Discovered by the FTC SDK through @Autonomous.
@Autonomous(name = "AutoTesting", group = "BioBuzz")
public class AutoTesting extends LinearOpMode {
    private static final long TELEMETRY_INTERVAL_NS = 100_000_000L;
    private static final int TEST_HEADING_DEGREES = -10;
    private static final double MOVE_POWER = 0.10;
    private static final long MOVE_DURATION_MS = 1_000;
    private static final long SETTLE_DURATION_MS = 300;
    private static final long TURN_TIMEOUT_MS = 2_000;
    private static final int MAX_TURN_ATTEMPTS = 3;

    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private long lastTelemetryTimeNs;
    private long testStartNs;
    private StringBuilder samples;
    private String currentStep = "Waiting for Start";

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        intake.intakeOff();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        updateTelemetry();
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        drive.resetHeading();
        testStartNs = System.nanoTime();
        samples = new StringBuilder("Elapsed ms,Current step,IMU yaw (deg),Drive yaw command (CW positive),"
                + "Drive front left power,Drive back left power,Drive front right power,Drive back right power,"
                + "Drive front left velocity (ticks/s),Drive back left velocity (ticks/s),"
                + "Drive front right velocity (ticks/s),Drive back right velocity (ticks/s)\n");
        String result = "Test interrupted";
        try {
            if (!turnAndSettle(TEST_HEADING_DEGREES)) {
                result = "Test stopped before field-back move: -10 degrees not reached";
                return;
            }
            if (!opModeIsActive()) {
                return;
            }
            intake.intakeOn();
            try {
                if (!moveFieldRelative("Field back with intake", -MOVE_POWER)) {
                    return;
                }
            } finally {
                intake.intakeOff();
            }
            if (!turnAndSettle(TEST_HEADING_DEGREES)) {
                result = "Test stopped before return: -10 degrees not reached";
                return;
            }
            if (!moveFieldRelative("Field return", MOVE_POWER)) {
                return;
            }
            result = turnAndSettle(0)
                    ? "Field-relative test complete"
                    : "Moves complete: starting heading not reached";
        } finally {
            drive.drive(0, 0, 0);
            showStep(isStopRequested() ? "Stopped" : result);
            saveSamples();
        }
    }

    private boolean moveFieldRelative(String step, double forwardPower) {
        if (!opModeIsActive()) {
            return false;
        }
        showStep(step);
        return drive.driveFieldRelativeHoldingHeading(forwardPower, TEST_HEADING_DEGREES,
                MOVE_DURATION_MS, this, this::updateTelemetryDuringMotion);
    }

    private boolean turnAndSettle(int headingDegrees) {
        for (int attempt = 1; attempt <= MAX_TURN_ATTEMPTS && opModeIsActive(); attempt++) {
            showStep("Settling before turn");
            sleep(SETTLE_DURATION_MS);
            if (!opModeIsActive()) {
                return false;
            }
            showStep("Turn to " + headingDegrees + " degrees (attempt " + attempt + ")");
            boolean reached = drive.turnToAngle(headingDegrees, TURN_TIMEOUT_MS, this,
                    this::updateTelemetryDuringMotion);
            sleep(SETTLE_DURATION_MS);
            if (reached && opModeIsActive()
                    && Math.abs(drive.getHeadingDegrees() - headingDegrees)
                    <= MecanumDrive.TURN_TOLERANCE_DEGREES) {
                return true;
            }
        }
        return false;
    }

    private void saveSamples() {
        File directory = new File(AppUtil.FIRST_FOLDER, "matchlogs");
        File file = new File(directory, "AutoTesting-" + System.currentTimeMillis() + ".csv");
        try {
            if (!directory.isDirectory() && !directory.mkdirs()) {
                throw new IOException("Cannot create " + directory);
            }
            try (FileOutputStream output = new FileOutputStream(file)) {
                output.write(samples.toString().getBytes(StandardCharsets.UTF_8));
            }
            telemetry.addData("Saved CSV", file.getName());
        } catch (IOException | SecurityException error) {
            RobotLog.ee("AutoTesting", error, "Unable to save drivetrain CSV");
            telemetry.addData("CSV save failed", error.getMessage());
        }
        telemetry.update();
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
        double yaw = drive.getHeadingDegrees();
        telemetry.addData("IMU yaw (deg)", yaw);
        if (samples != null) {
            samples.append((lastTelemetryTimeNs - testStartNs) / 1_000_000L)
                    .append(',').append(currentStep).append(',').append(yaw);
        }
        drive.addCommandTelemetry(telemetry, samples);
        if (samples != null) {
            samples.append('\n');
        }
        telemetry.update();
    }
}
