package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MLToolChain {

    private static String TFOD_MODEL_FILE;
    private static String[] LABELS;
    private static final String VUFORIA_KEY =
            "ATIZ5+n/////AAABmYR5zYS7b0fUh3KW5QXpeYpWedz730exbiG/c1eUwnbTLDy3S1zpCnndqs2H1bBRoRji6zODId1wNciirjMhrl3kb2xpxrhjWKGzfymlE48qjK1tSK9ctbVPDDEI7d3wiO1vISMt8XRVM0HJFumWl7snC0XtMEyTqy7IxrPv858Bm2voggonrgAx+WT/LcXxfsUQjd7SpX1AhzOEzksBRKfws0MP5hw7hLLbB+4QSb03hTA90Mh/7F78NxFG8yYmPrM0GMVg+3TRjteX6TWOHidLSmrIzsZNGFeqXCHOx4QIWIiMOIenqgau/XX2zCqvrMGCpOTb77oTjkvOlLNh9WizYdFPQjT/u7CeU1p3Y1ox";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private HardwareMap hardwareMap;
    private String telemetry = "TensorFlow not ready";
    public String sleeveNum;

    public MLToolChain(String filePath, String [] labels, HardwareMap hardwareMap){
        TFOD_MODEL_FILE = filePath;
        LABELS = labels;
        this.hardwareMap = hardwareMap;

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }

        telemetry = "TensorFlow is setup and ready!";
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
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    public String toString () {
        return telemetry;
    }

    public void getSleeveNum (double confidanceLevel) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    telemetry = "TF recognizes: " + recognition.getLabel() + "\nConfidance level: " + recognition.getConfidence();
                    if(recognition.getConfidence() > confidanceLevel){
                        sleeveNum = recognition.getLabel();
                    }
                }
            }
        }else {
            telemetry = "Nothing detected!";
        }
    }
}
