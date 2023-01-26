package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "CompetitionTeleOp", group = "Competition")
public class CompetitionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int slidePosition = 0;

        telemetry.addData(">", "Ready!");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {

            //basic grabber claw control
            if (gamepad2.left_bumper || gamepad1.left_bumper) { //close
                drive.grabber(0);
            } else if (gamepad2.right_bumper || gamepad1.right_bumper) { //open
                drive.grabber(0.2);
            }

            drive.extenderMove(slidePosition);

            //pole specific control
            if (gamepad2.x) {
                slidePosition = 0;

            } else if (gamepad2.y) {
                slidePosition = -1630;

            } else if (gamepad2.b) {
                slidePosition = -2830;

            } else if (gamepad2.a)
                slidePosition = -4000;

            //driver 1 lower slide functionality
            if (gamepad1.left_trigger > 0) {
                slidePosition = slidePosition + 400;
                sleep(500);
            } else if (gamepad1.right_trigger > 0) {
                slidePosition = slidePosition - 400;
                sleep(500);
            }

            if (gamepad2.left_trigger > 0) {
                slidePosition = slidePosition + 400;
                sleep(500);
            } else if (gamepad2.right_trigger > 0) {
                slidePosition = slidePosition - 400;
                sleep(500);
            }

            //prevent the slide from over-extending/over-retracting
            if (slidePosition > 0) {
                slidePosition = 0;
            } else if (slidePosition < -4000) {
                slidePosition = -4000;
            }

            //variable slide control using joystick v1

//            //viper slide control - CTRL + / to uncomment
//            int floor = 0;
//            int lowPole = -1630;
//            int midPole = -2830;
//            int hiPole = -4000;
//
//            //pole specific control
//            if (gamepad2.x) {
//                drive.extenderMove(floor);
//
//            } else if (gamepad2.y) {
//                drive.extenderMove(lowPole);
//
//            } else if (gamepad2.b) {
//                drive.extenderMove(midPole);
//
//            } else if (gamepad2.a)
//                drive.extenderMove(hiPole);

            //drive control (from RR)
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
