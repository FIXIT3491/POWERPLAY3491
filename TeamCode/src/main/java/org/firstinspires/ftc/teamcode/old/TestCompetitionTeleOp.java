package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestCompetitionTeleOp", group = "Test")
public class TestCompetitionTeleOp extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor slideExtender = null;
    private Servo grabberClaw = null;

    @Override
    public void runOpMode() {

        //000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
        //Init Stuff Here                                                                          0
        //000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slideExtender = hardwareMap.get(DcMotor.class, "slideExtender");
        grabberClaw = hardwareMap.get(Servo.class, "grabberClaw");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        slideExtender.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Calibrating");
        telemetry.update();

        slideExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        grabberClaw.setPosition(0);

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();
        waitForStart();

        double axial = 0;   //how fast forward the drivers want the robot to move
        double lateral = 0; //how fast sideways the drivers want the robot to move
        double yaw = 0;     //how fast the drivers want the robot to spin
        double direction = 0;
        boolean auto = false;
        while (opModeIsActive()) {

            telemetry.addData("status", "Running");
            telemetry.addData("axial", axial);
            telemetry.addData("lateral", lateral);
            telemetry.addData("yaw", yaw);
            telemetry.addData("direction", direction);
            telemetry.addData("autopilot", auto);
            telemetry.addData("slide value", slideExtender.getCurrentPosition());
            telemetry.addData("joystick y val 2", gamepad2.right_stick_y);
            telemetry.update();

            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000
            //Driving Control                                                                      0
            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000

            axial = gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            driveDumb(axial, lateral, yaw);

            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000
            //Mechanism Control                                                                    0
            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000

            //basic grabber claw control
            if (gamepad2.left_bumper || gamepad1.left_bumper) { //close
                grabberClaw.setPosition(0);
            }

            if (gamepad2.right_bumper || gamepad1.right_bumper) { //open
                grabberClaw.setPosition(0.2);
            }

            //viper slide control
            int floor = 0;
            int lowPole = -1630;
            int midPole = -2830;
            int hiPole = -4000;

            if (gamepad2.x) {
                extenderMove(floor);

            } else if (gamepad2.y) {
                extenderMove(lowPole);

            } else if (gamepad2.b) {
                extenderMove(midPole);

            } else if (gamepad2.a) {
                extenderMove(hiPole);

            } else if (gamepad2.left_stick_y > 0) {
                extenderMove(hiPole);

            } else if (gamepad2.left_stick_y < 0) {
                extenderMove(floor);
            }
        }
    }

    //slide control function
    public void extenderMove(int slidePosition) {
        slideExtender.setTargetPosition(slidePosition);
        slideExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideExtender.setPower(1);
    }

    //basic driving function
    public void driveDumb(double axial, double lateral, double yaw) {

        double max;
        double leftFrontPower = axial - lateral - yaw;
        double rightFrontPower = axial + lateral + yaw;
        double leftBackPower = axial + lateral - yaw;
        double rightBackPower = axial - lateral + yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

    }

    private void stopDrive(long ms) {
        driveDumb(0, 0, 0);
        sleep(ms);
    }

    private void stopDrive() {
        stopDrive(0);
    }
}