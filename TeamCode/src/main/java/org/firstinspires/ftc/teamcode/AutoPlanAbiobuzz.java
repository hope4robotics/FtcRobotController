package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "AutoPlanAbiobuzz", group = "BioBuzz")
public class AutoPlanAbiobuzz extends LinearOpMode {
    private static final long LAUNCH_TIMEOUT_MS = 10000;
    private static final long DRIVE_DURATION_MS = 1000;
    private static final long INTAKE_PAUSE_MS = 1000;
    private static final long TURN_TIMEOUT_MS = 2000;
    // Forward and strafe add at the wheels. Two quarter-power inputs cap power at 0.5.
    private static final double DIAGONAL_AXIS_POWER = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive();
        Intake intake = new Intake();
        Launch launcher = new Launch();

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        drive.resetHeading();
        double startingHeading = drive.getHeadingDegrees();
        launcher.launchForMS(LAUNCH_TIMEOUT_MS, intake, this);
        drive.driveFieldRelativeForMS(-DIAGONAL_AXIS_POWER, -DIAGONAL_AXIS_POWER, 0,
                DRIVE_DURATION_MS, this);

        if (!opModeIsActive()) {
            return;
        }
        try {
            intake.intakeOn();
            drive.driveFieldRelativeForMS(DIAGONAL_AXIS_POWER, -DIAGONAL_AXIS_POWER, 0,
                    DRIVE_DURATION_MS, this);
            if (opModeIsActive()) {
                sleep(INTAKE_PAUSE_MS);
            }
        } finally {
            intake.intakeOff();
        }

        drive.driveFieldRelativeForMS(-DIAGONAL_AXIS_POWER, -DIAGONAL_AXIS_POWER, 0,
                DRIVE_DURATION_MS, this);

        drive.turnToAngle(startingHeading, TURN_TIMEOUT_MS, this);
    }
}
