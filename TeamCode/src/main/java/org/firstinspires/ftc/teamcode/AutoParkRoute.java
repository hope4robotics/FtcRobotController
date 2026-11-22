package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

abstract class AutoParkRoute extends LinearOpMode {
    private static final double DRIVE_POWER = 0.5;
    private static final long RIGHT_DURATION_MS = 1_000;
    private static final long BACK_DURATION_MS = 500;
    private static final long LAUNCH_DURATION_MS = 7_000;

    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private final Launch launcher = new Launch();
    private final boolean launchBeforeParking;
    private final double fieldDirectionSign;

    AutoParkRoute(boolean startsOppositePlanA, boolean launchBeforeParking) {
        // A yaw reset at a 180-degree offset reverses both field axes.
        fieldDirectionSign = startsOppositePlanA ? -1.0 : 1.0;
        this.launchBeforeParking = launchBeforeParking;
    }

    @Override
    public final void runOpMode() {
        drive.init(hardwareMap);
        if (launchBeforeParking) {
            intake.init(hardwareMap);
            launcher.init(hardwareMap);
        }

        showStep("Waiting for Start");
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }
        drive.resetHeading();
        if (launchBeforeParking) {
            showStep("Launch for 7 s");
            launcher.launchForMS(LAUNCH_DURATION_MS, intake, this);
        }

        if (opModeIsActive()) {
            showStep("Drive field right for 1 s");
            drive.driveFieldRelativeForMS(0, fieldDirectionSign * DRIVE_POWER, 0,
                    RIGHT_DURATION_MS, this);
        }
        if (opModeIsActive()) {
            showStep("Drive field back for 0.5 s");
            drive.driveFieldRelativeForMS(-fieldDirectionSign * DRIVE_POWER, 0, 0,
                    BACK_DURATION_MS, this);
        }
        showStep(isStopRequested() ? "Stopped" : "Park route complete");
    }

    private void showStep(String step) {
        telemetry.addData("Current step", step);
        telemetry.update();
    }
}
