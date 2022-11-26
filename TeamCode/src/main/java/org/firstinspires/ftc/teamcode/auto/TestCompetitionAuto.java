package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestCompetitionAuto", group = "Test")
@Disabled
public class TestCompetitionAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        AutoMain autoMain = new AutoMain();

        waitForStart();
        autoMain.driveDumb(0.5,0,0);
    }
}
