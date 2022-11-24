package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestCompetitionAuto", group = "Test")
public class TestCompetitionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        AutoMain autoMain = new AutoMain();

        waitForStart();
        autoMain.driveSmart(0.5,0,0,1000);
    }
}
