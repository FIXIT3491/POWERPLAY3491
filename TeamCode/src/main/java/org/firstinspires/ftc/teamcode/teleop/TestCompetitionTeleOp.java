package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TestCompetitionTeleOp", group = "Test")
@Disabled
public class TestCompetitionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        TeleOpMain teleOpMain = new TeleOpMain();

        while (opModeIsActive()) {

            // driving control
            double axial   = -gamepad1.left_stick_y;
            double lateral = -gamepad1.right_stick_x;
            double yaw     = gamepad1.left_stick_x;
            teleOpMain.driveDumb(axial, lateral, yaw);

            // claw control
            teleOpMain.grabberMove();

            // viper slide control
            teleOpMain.viperMove();

        }
    }
}