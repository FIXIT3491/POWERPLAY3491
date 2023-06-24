package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class NewCompetitionAutoRight extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    //UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 3;

    int redCanada = 3;
    int greenFedora = 11;
    int yellowDuck = 18;

    //define specifiers
    int aprilCode = 0;
    String symbol;
    int zone;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //init le grabber
        drive.grabber(0);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == redCanada) {
                        telemetry.addData(">", "redCanada, 3, 1");
                        aprilCode = redCanada;
                        symbol = "redCanada";
                        zone = 1;

                    } else if (tag.id == greenFedora) {
                        telemetry.addData(">", "greenFedora, 11, 2");
                        aprilCode = greenFedora;
                        symbol = "greenFedora";
                        zone = 2;

                    } else if (tag.id == yellowDuck) {
                        telemetry.addData(">", "yellowDuck, 18, 3");
                        aprilCode = yellowDuck;
                        symbol = "yellowDuck";
                        zone = 3;

                    }
                }
            } else {
                telemetry.addLine("No tag in sight");
            }
            telemetry.update();
            sleep(20);
        }

        //auton code here

        if (aprilCode == 0) {
            telemetry.addLine("No tag in sight");
            telemetry.update();
            drive.rightSide();

        } else {
            telemetry.addData("April Code", aprilCode);
            telemetry.addData("Symbol", symbol);
            telemetry.addData("Zone", zone);
            telemetry.update();

            if (zone == 1) {
                drive.rightSide();
                drive.one();
            } else if (zone == 2) {
                drive.rightSide();
                drive.two();
            } else if (zone == 3) {
                drive.rightSide();
                drive.three();
            }
        }
    }
}

