package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TestTeleOp", group = "Test")
public class TestTeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;

    // very basic mecanum TeleOp *don't change, it just works*
    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        waitForStart();
        if (opModeIsActive()) {

            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive()) {
                leftBack.setPower(((0 + gamepad1.right_stick_y) - gamepad1.left_stick_x) + gamepad1.right_stick_x);
                rightBack.setPower(((0 + gamepad1.right_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x);
                leftFront.setPower((0 + gamepad1.right_stick_y) - gamepad1.left_stick_x - gamepad1.right_stick_x);
                rightFront.setPower(((0 + gamepad1.right_stick_y) + gamepad1.left_stick_x) + gamepad1.right_stick_x);
            }
        }
    }
}
