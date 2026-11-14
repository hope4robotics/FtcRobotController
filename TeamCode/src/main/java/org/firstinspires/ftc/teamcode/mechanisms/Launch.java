/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launch {
    private static final int TARGET_VELOCITY = 1300; // Encoder ticks per second
    private static final int MIN_FEED_VELOCITY = 1250;
    private static final double INTAKE_ASSIST_POWER = 0.5;

    private DcMotorEx launcher;
    private CRServo windmillServo;

    public void init(HardwareMap hwMap) {
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        windmillServo = hwMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        launch(false);
    }

    /**
     * Updates the launcher and feeder. Returns the power to add to the driver's intake command.
     * In teleop, call each loop with the current right bumper state.
     */
    public double launch(boolean requested) {
        launcher.setVelocity(requested ? TARGET_VELOCITY : 0);

        boolean feeding = requested && launcher.getVelocity() > MIN_FEED_VELOCITY;
        windmillServo.setPower(feeding ? 1.0 : 0.0);
        return feeding ? INTAKE_ASSIST_POWER : 0.0;
    }

    /**
     * Runs a timed launch with intake assist. The timeout includes spin-up.
     * Call after start with both mechanisms initialized. Returns true if the
     * feeder was enabled at least once.
     */
    public boolean launchForMS(long timeoutMs, Intake intake, LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();
        boolean fed = false;
        try {
            while (opMode.opModeIsActive() && timer.milliseconds() < timeoutMs) {
                double intakeAssist = launch(true);
                intake.intake(intakeAssist, 0);
                fed |= intakeAssist > 0;
            }
            return fed;
        } finally {
            try {
                launch(false);
            } finally {
                intake.intakeOff();
            }
        }
    }
}
