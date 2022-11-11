package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum (Blocks to Java)")
public class TeleOpMecanumDrive2 extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            while (opModeIsActive()) {
                leftBack.setPower(((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x);
                rightBack.setPower(((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) + gamepad1.right_stick_x);
                leftFront.setPower((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x);
                rightFront.setPower(((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) - gamepad1.right_stick_x);
            }
        }
    }
}
