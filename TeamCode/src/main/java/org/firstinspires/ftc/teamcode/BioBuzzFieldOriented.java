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
    public void loop() {
        boolean resetButtonDown = gamepad1.a;
        if (resetButtonDown && !resetButtonWasDown) {
            drive.resetHeading();
        }
        resetButtonWasDown = resetButtonDown;

        drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        double intakeAssist = launcher.launch(gamepad1.right_bumper);
        double intakePower = Range.clip(
                gamepad1.right_trigger - gamepad1.left_trigger + intakeAssist, -1.0, 1.0);
        intake.intake(intakePower, 0);
    }

    @Override
    public void stop() {
        drive.drive(0, 0, 0);
        launcher.launch(false);
        intake.intakeOff();
    }
}
