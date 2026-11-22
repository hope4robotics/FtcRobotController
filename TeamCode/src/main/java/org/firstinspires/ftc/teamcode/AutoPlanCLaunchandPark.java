package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@SuppressWarnings("unused") // Discovered by the FTC SDK through @Autonomous.
@Autonomous(name = "AutoPlanCLaunchandPark", group = "BioBuzz")
public class AutoPlanCLaunchandPark extends AutoParkRoute {
    public AutoPlanCLaunchandPark() {
        super(false, true);
    }
}
