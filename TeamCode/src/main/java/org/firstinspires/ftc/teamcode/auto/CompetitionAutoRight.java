package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "CompetitionAutoRight", group = "Competition")
public class CompetitionAutoRight extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Model1.tflite";
    private static final String[] LABELS = {"greenFedora", "redCanada", "yellowDuck"};
    private static final String VUFORIA_KEY = "ATU9MNz/////AAABmdp9yZ8JdEGjpiGfxU8g64YjAQPwRcIIIqytyWu9HmjEkTELwI1JsCtkFv/I4k2S8KXjgWFB61R+GwLPvY3T1EyQmpV/UFfaSEqcJLpT++NbMjv5JkXg3JG92Ga+RnHYS3WaTBgRZexhqar4QNK4exrzUQUJjy2ntF2Afb+ENqH4glLQW85aM0BA4+8WMjcplpZ5WbhJ82ruz0ikcpy8bffFnhd+pN1/xficoB/Szcx5lt1SmKzVjbYkktmVd8qS6qGd8yVH1DydPQlP6njcUDllIc1a3oAO5zmWTFoxfaknDOm2bXka6V2Qht6pD7pl1tSP3vgeCZPM0fKSowfy0MoFVzsuuBvwqloB4Obt4NDT";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 16.0/9.0);
        }

        drive.grabber(0);

        telemetry.addData(">", "Ready!");
        telemetry.update();

        waitForStart();

        Pose2d poseEstimate = drive.getPoseEstimate();

        if(isStopRequested()) return;


        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                            if (recognition.getLabel() == "redCanada") {
                                tfod.deactivate();
                                drive.rightSide();
                                //drive.one();
                            }

                            if (recognition.getLabel() == "greenFedora") {
                                tfod.deactivate();
                                drive.rightSide();
                                sleep(3000);
                                drive.two();
                            }

                            if (recognition.getLabel() == "yellowDuck") {
                                tfod.deactivate();
                                drive.rightSide();
                                //drive.three();
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

}
