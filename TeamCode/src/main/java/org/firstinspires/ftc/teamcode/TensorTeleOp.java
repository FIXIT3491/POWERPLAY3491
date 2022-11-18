package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@TeleOp(name = "TensorTeleOp")
public class TensorTeleOp extends LinearOpMode {

    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;

    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rightFront;

    Recognition recognition;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        //------------------------------------------------------------------------------------------
        // Driving Controls
        //------------------------------------------------------------------------------------------

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

                leftBack.setPower(((0 + gamepad1.right_stick_y) - gamepad1.left_stick_x) + gamepad1.right_stick_x);
                rightBack.setPower(((0 + gamepad1.right_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x);
                leftFront.setPower((0 + gamepad1.right_stick_y) - gamepad1.left_stick_x - gamepad1.right_stick_x);
                rightFront.setPower(((0 + gamepad1.right_stick_y) + gamepad1.left_stick_x) + gamepad1.right_stick_x);

                List<Recognition> recognitions;
                int index;

                vuforiaPOWERPLAY = new VuforiaCurrentGame();
                tfod = new Tfod();

                // Sample TFOD Op Mode using a Custom Model
                // Initialize Vuforia to provide TFOD with camera
                // images.
                // The following block uses the device's back camera.
                // The following block uses a webcam.
                vuforiaPOWERPLAY.initialize(
                        "", // vuforiaLicenseKey
                        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                        "", // webcamCalibrationFilename
                        false, // useExtendedTracking
                        false, // enableCameraMonitoring
                        VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                        0, // dx
                        0, // dy
                        0, // dz
                        AxesOrder.XZY, // axesOrder
                        90, // firstAngle
                        90, // secondAngle
                        0, // thirdAngle
                        true); // useCompetitionFieldTargetLocations
                // Initialize TFOD before waitForStart.
                // Use the Manage page to upload your custom model.
                // In the next block, replace
                // YourCustomModel.tflite with the name of your
                // custom model.
                // Set isModelTensorFlow2 to true if you used a TensorFlow
                // 2 tool, such as ftc-ml, to create the model.
                //
                // Set isModelQuantized to true if the model is
                // quantized. Models created with ftc-ml are quantized.
                //
                // Set inputSize to the image size corresponding to the model.
                // If your model is based on SSD MobileNet v2
                // 320x320, the image size is 300 (srsly!).
                // If your model is based on SSD MobileNet V2 FPNLite 320x320, the image size is 320.
                // If your model is based on SSD MobileNet V1 FPN 640x640 or
                // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
                tfod.useModelFromFile("CustomModelPowerplay.tflite", JavaUtil.createListWith("greenFedora", "redCanada", "yellowDuck"), true, true, 320);
                tfod.initialize(vuforiaPOWERPLAY, (float) 0.5, true, true);
                tfod.setClippingMargins(0, 80, 0, 0);
                tfod.activate();
                // Enable following block to zoom in on target.
                tfod.setZoom(1.5, 16 / 9);
                telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
                telemetry.addData(">", "Press Play to start");
                telemetry.update();
                // Wait for start command from Driver Station.
                waitForStart();
                if (opModeIsActive()) {
                    // Put run blocks here.
                    while (opModeIsActive()) {
                        // Put loop blocks here.
                        // Get a list of recognitions from TFOD.
                        recognitions = tfod.getRecognitions();
                        // If list is empty, inform the user. Otherwise, go
                        // through list and display info for each recognition.
                        if (JavaUtil.listLength(recognitions) == 0) {
                            telemetry.addData("TFOD", "No items detected.");
                        } else {
                            index = 0;
                            // Iterate through list and call a function to
                            // display info for each recognized object.
                            for (Recognition recognition_item : recognitions) {
                                recognition = recognition_item;
                                // Display info.
                                displayInfo(index);
                                // Increment index.
                                index = index + 1;
                            }
                        }
                        telemetry.update();
                    }
                }
                // Deactivate TFOD.
                tfod.deactivate();

                vuforiaPOWERPLAY.close();
                tfod.close();
            }
        }


    }

    private void displayInfo(int i) {
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Label: " + recognition.getLabel() + ", Confidence: " + recognition.getConfidence(), "X: " + Math.round(JavaUtil.averageOfList(JavaUtil.createListWith(Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)), Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0))))) + ", Y: " + Math.round(JavaUtil.averageOfList(JavaUtil.createListWith(Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)), Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0))))));
    }


}

