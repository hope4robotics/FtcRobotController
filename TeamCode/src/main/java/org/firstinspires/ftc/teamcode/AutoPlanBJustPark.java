package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@SuppressWarnings("unused") // Discovered by the FTC SDK through @Autonomous.
@Autonomous(name = "AutoPlanBJustPark", group = "BioBuzz")
public class AutoPlanBJustPark extends AutoParkRoute {
    public AutoPlanBJustPark() {
        super(true, false);
    }
}
