package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "BioBuzz Field Oriented", group = "BioBuzz")
public class BioBuzzFieldOriented extends OpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private final Intake intake = new Intake();
    private final Launch launcher = new Launch();
    private boolean resetButtonWasDown;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
    }

    @Override
    public void start() {
        drive.resetHeading();
        resetButtonWasDown = gamepad1.a;
    }

    @Override
    public void loop() {
        boolean resetButtonDown = gamepad1.a;
        if (resetButtonDown && !resetButtonWasDown) {
            drive.resetHeading();
        }
        resetButtonWasDown = resetButtonDown;

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);

        double intakeAssist = launcher.launch(gamepad1.right_bumper);
        double intakePower = Range.clip(
                gamepad1.right_trigger - gamepad1.left_trigger + intakeAssist, -1.0, 1.0);
        intake.intake(intakePower, 0);

        String currentAction = intakeAssist > 0 ? "Feeding" :
                gamepad1.right_bumper ? "Launcher spinning up" : "Driver control";
        telemetry.addData("Current action", currentAction);
        launcher.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.drive(0, 0, 0);
        launcher.launch(false);
        intake.intakeOff();
    }
}
