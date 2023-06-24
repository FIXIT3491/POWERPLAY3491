package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDriveBackup extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 1);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.0169491525;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront, slideExtender;
    private Servo grabberClaw;
    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    public SampleMecanumDriveBackup(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        slideExtender = hardwareMap.get(DcMotorEx.class, "slideExtender");
        grabberClaw = hardwareMap.get(Servo.class, "grabberClaw");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        slideExtender.setDirection(DcMotor.Direction.REVERSE);

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }
    //auton methods
    public void leftSide() throws InterruptedException {

        Pose2d startPose = new Pose2d(-32, -65, Math.toRadians(90));

        setPoseEstimate(startPose);

        Trajectory leftSideStrafeRight = trajectoryBuilder(startPose)
                .strafeRight(6.5)
                .build();

        Trajectory forward = trajectoryBuilder(leftSideStrafeRight.end())
                .forward(11)
                .build();

        Trajectory back = trajectoryBuilder(forward.end())
                .back(9)
                .build();

        Trajectory strafeRight = trajectoryBuilder(back.end())
                .strafeRight(11.625)
                .build();

        Trajectory approachStack1 = trajectoryBuilder(strafeRight.end())
                .forward(49)
                .build();

        Trajectory approachStack2 = trajectoryBuilder(approachStack1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(48)
                .build();

        Trajectory forwardStack = trajectoryBuilder(approachStack2.end())
                .forward(5)
                .build();

        Trajectory backStack = trajectoryBuilder(forwardStack.end())
                .back(17)
                .build();

        Trajectory forwardsPole = trajectoryBuilder(backStack.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(7)
                .build();

        Trajectory backPole = trajectoryBuilder(forwardsPole.end())
                .back(7)
                .build();

        Trajectory position = trajectoryBuilder(backPole.end())
                .strafeLeft(11.625)
                .build();





        grabberClaw.setPosition(0);
        followTrajectory(leftSideStrafeRight);
        extenderMove(-1630);
        sleep(500);
        followTrajectory(forward);
        sleep(500);
        grabberClaw.setPosition(0.15);
        followTrajectory(back);
        extenderMove(0);
        sleep(500);
        followTrajectory(strafeRight);
        followTrajectory(approachStack1);
        followTrajectory(approachStack2);
        extenderMove(-650);
        sleep(500);
        followTrajectory(forwardStack);
        grabberClaw.setPosition(0);
        sleep(500);
        extenderMove(-1650);
        sleep(500);
        followTrajectory(backStack);
        turn(Math.toRadians(90));
        followTrajectory(forwardsPole);
        grabberClaw.setPosition(0.15);
        sleep(500);
        followTrajectory(backPole);
        followTrajectory(position);
        extenderMove(0);
        sleep(500);



    }

    public void rightSide() throws InterruptedException {
        Pose2d startPose = new Pose2d(42, -65, Math.toRadians(90));

        setPoseEstimate(startPose);

        Trajectory leftSideStrafeLeft = trajectoryBuilder(startPose)
                .strafeLeft(17)
                .build();

        Trajectory forward = trajectoryBuilder(leftSideStrafeLeft.end())
                .forward(11)
                .build();

        Trajectory back = trajectoryBuilder(forward.end())
                .back(9)
                .build();

        Trajectory strafeLeft = trajectoryBuilder(back.end())
                .strafeLeft(11.625)
                .build();

        //auton part 2

        Trajectory approachStack1 = trajectoryBuilder(strafeLeft.end())
                .forward(49)
                .build();

        Trajectory approachStack2 = trajectoryBuilder(approachStack1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(47)
                .build();

        Trajectory forwardStack = trajectoryBuilder(approachStack2.end())
                .forward(5)
                .build();

        Trajectory backStack = trajectoryBuilder(forwardStack.end())
                .back(17)
                .build();

        Trajectory forwardsPole = trajectoryBuilder(backStack.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(7)
                .build();

        Trajectory backPole = trajectoryBuilder(forwardsPole.end())
                .back(7)
                .build();

        Trajectory position = trajectoryBuilder(backPole.end())
                .strafeRight(11.625)
                .build();





        grabberClaw.setPosition(0);
        followTrajectory(leftSideStrafeLeft);
        extenderMove(-1630);
        sleep(500);
        followTrajectory(forward);
        sleep(500);
        grabberClaw.setPosition(0.15);
        followTrajectory(back);
        extenderMove(0);
        sleep(500);
        followTrajectory(strafeLeft);
        followTrajectory(approachStack1);
        followTrajectory(approachStack2);
        extenderMove(-650);
        sleep(500);
        followTrajectory(forwardStack);
        grabberClaw.setPosition(0);
        sleep(500);
        extenderMove(-1650);
        sleep(500);
        followTrajectory(backStack);
        turn(Math.toRadians(-90)); //error is here
        followTrajectory(forwardsPole);
        grabberClaw.setPosition(0.15);
        sleep(500);
        followTrajectory(backPole);
        followTrajectory(position);
        extenderMove(0);
        sleep(500);
    }


    //zone methods
    public void one() {
        setPoseEstimate(new Pose2d());
        Trajectory right = trajectoryBuilder((new Pose2d()))
                .strafeRight(25)
                .build();
        followTrajectory(right);
    }

    public void two() throws InterruptedException {
        sleep(500);
    }

    public void three() {
        setPoseEstimate(new Pose2d());
        Trajectory left = trajectoryBuilder((new Pose2d()))
                .strafeLeft(25)
                .build();
        followTrajectory(left);
    }


    //claw control method
    public void grabber(double grabberPosition) {
        grabberClaw.setPosition(grabberPosition);
    }

    //slide control methods
    public void extenderZero() {
        slideExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extenderMove(int slidePosition) {
        slideExtender.setTargetPosition(slidePosition);
        slideExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideExtender.setPower(1);
    }

    public void extenderRetract(double speed) {
        slideExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideExtender.setPower(speed);
    }

    //RR methods
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
