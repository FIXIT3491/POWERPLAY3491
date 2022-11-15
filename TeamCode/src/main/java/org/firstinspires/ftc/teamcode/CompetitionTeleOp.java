package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "CompetitionTeleOp")
public class CompetitionTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    Orientation lastAngles = new Orientation();

    double globalAngle = 0;

    BNO055IMU imu;

    Orientation angles;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor slideExtender = null;
    private Servo grabberClaw = null;

    @Override
    public void runOpMode(){

        //000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
        //Init Stuff Here                                                                          0
        //000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slideExtender = hardwareMap.get(DcMotor.class, "slideExtender");
        grabberClaw = hardwareMap.get(Servo.class, "grabberClaw");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        slideExtender.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Calibrating");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double axial = 0;   //how fast forward the drivers want the robot to move
        double lateral = 0; //how fast sideways the drivers want the robot to move
        double yaw = 0;     //how fast the drivers want the robot to spin
        double direction = 0;
        boolean auto = false;
        while(opModeIsActive()){

            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000
            //Driving Control                                                                      0
            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            //when gamepad1.a is pressed the robot now wants to always point in the direction it's currently facing
            if(gamepad1.a){
                auto = true;
                resetAngle();
                direction = 0;
            }

            if(gamepad1.b){
                auto = false;
            }

            if(auto){
                //If drivers are trying to turn, constantly set the desired direction to something out of reach to keep the robot turning
                if(yaw != 0){
                    direction = getAngle() - yaw * 20;
                }

                //driveSmart aimed at pointing at direction, in degrees
                driveSmart(axial, lateral, direction);

                //try to get rid of shimmy from robot overcorrecting after the drivers stop turning
                if(yaw != 0){
                    direction = getAngle();
                }
            }

            else{
                //Drivers get manual control of robot when robot be small brain
                driveDumb(axial, lateral, yaw);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("axial", axial);
            telemetry.addData("lateral", lateral);
            telemetry.addData("yaw", yaw);
            telemetry.addData("direction", direction);
            telemetry.addData("autopilot", auto);

            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000
            //Mechanism Control                                                                    0
            //00000000000000000000000000000000000000000000000000000000000000000000000000000000000000

            //basic viper slide control
            double extenderPower = gamepad2.left_stick_y;

            slideExtender.setPower(extenderPower);

            telemetry.addData("Extender Power", extenderPower);
            telemetry.update();

            //basic grabber claw control - change to trigger not bumper!!
            if(gamepad2.left_bumper = true){
                grabberClaw.setPosition(0);
            }
            if(gamepad2.right_bumper = true){
                grabberClaw.setPosition(1);
            }
        }
    }

    //basic driving function
    public void driveDumb(double axial, double lateral, double yaw) {

        double max;

        double leftFrontPower = - axial - lateral - yaw;
        double rightFrontPower = - axial + lateral + yaw;
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

    //advanced driving function, used for autonomous and also driver assistance
    public void driveSmart(double axial, double lateral, double yaw, double time) {

        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        double max, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        double correction, angle, gain = .05;

        do{
            angle = getAngle();

            //value may need tinkering
            correction = (-angle + yaw) * gain;

            leftFrontPower = - axial - lateral - correction;
            rightFrontPower = - axial + lateral + correction;
            leftBackPower = axial + lateral - correction;
            rightBackPower = axial - lateral + correction;

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

        while (timer.milliseconds() <= time && opModeIsActive());

    }

    public void driveSmart(double axial, double lateral, double yaw){
        driveSmart(axial, lateral, yaw, 0);
    }

    //Resets the cumulative angle tracking to zero.
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


    //Get current cumulative angle rotation from last reset.
    //Angle in degrees. + = left, - = right.
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void stopDrive(long ms){
        driveDumb(0, 0, 0);
        sleep(ms);
    }

    private void stopDrive(){
        stopDrive(0);
    }
}
