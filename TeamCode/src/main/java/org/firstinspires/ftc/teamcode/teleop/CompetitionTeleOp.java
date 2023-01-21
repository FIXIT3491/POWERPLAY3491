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

        telemetry.addData(">", "Ready!");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {

            //basic grabber claw control
            if (gamepad2.left_bumper || gamepad1.left_bumper) { //close
                drive.grabber(0);
            } else if (gamepad2.right_bumper || gamepad1.right_bumper) {
                drive.grabber(0.2);
            }

            //viper slide control
            int floor = 0;
            int lowPole = -1630;
            int midPole = -2830;
            int hiPole = -4000;

            //pole specific control
            if (gamepad2.x) {
                drive.extenderMove(floor);

            } else if (gamepad2.y) {
                drive.extenderMove(lowPole);

            } else if (gamepad2.b) {
                drive.extenderMove(midPole);

            } else if (gamepad2.a)
                drive.extenderMove(hiPole);

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
