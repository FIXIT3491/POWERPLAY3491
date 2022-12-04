package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OldTestCompetitionAuto", group = "Test")
@Disabled
public class OldTestCompetitionAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        OldAutoMain oldAutoMain = new OldAutoMain();

        waitForStart();
        oldAutoMain.driveDumb(0.5,0,0);
    }
}
