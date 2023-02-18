
package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class SignalSleeve {
    OpenCvCamera webcam;
    Threshold thresh;
    String telemetry = "waiting for input";
    DuckPos position;
    public static Rect redRect = new Rect(75,105,15,15);
    public static Rect blueRect = new Rect(90,105,15,15);
    public static Rect yellowRect = new Rect(60,105,15,15);
    public static int threshRed = 145, threshBlue = 150, threshYellow = 100;
    public static int filterVal = 10;


    public SignalSleeve(HardwareMap hardwareMap, String camName){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,  camName), cameraMonitorViewId);

        thresh = new Threshold();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(thresh);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry = "waiting for start";
    }

    class Threshold extends OpenCvPipeline {

        int stage = 0;
        Mat ycrcb = new Mat();
        Mat red = new Mat();
        Mat yellow = new Mat();
        Mat blue = new Mat();
        Mat thresholdRed = new Mat();
        Mat thresholdYellow = new Mat();
        Mat thresholdBlue = new Mat();


        double avg1, avg2, avg3;

        @Override
        public void onViewportTapped() {
            stage ++;

            if(stage > 6){
                stage = 0;
            }
        }

        @Override
        public Mat processFrame(Mat input){
//            telemetry = "pre-init";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

//            telemetry = "created submats";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(ycrcb, yellow, 0);
            Core.extractChannel(ycrcb, red, 1);
            Core.extractChannel(ycrcb, blue, 2);

//            telemetry = "calculated channels";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            Imgproc.threshold(yellow, thresholdYellow, threshYellow, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(red, thresholdRed, threshRed, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(blue, thresholdBlue, threshBlue, 255, Imgproc.THRESH_BINARY);

//            telemetry = "thresholded";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }


            Imgproc.rectangle(thresholdBlue, blueRect,new Scalar(255,0,0), 2);
            Imgproc.rectangle(thresholdRed, redRect,new Scalar(255,0,0), 2);
            Imgproc.rectangle(thresholdYellow, yellowRect,new Scalar(255,0,0), 2);

//            telemetry = "drew rectangles";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            Mat section1 = thresholdRed.submat(blueRect);
            Mat section2 = thresholdYellow.submat(redRect);
            Mat section3 = thresholdBlue.submat(yellowRect);

//            telemetry = "created sections";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }


            avg1 = Core.mean(section1).val[0];
            avg2 = Core.mean(section2).val[0];
            avg3 = Core.mean(section3).val[0];

//            telemetry = "calculated avgs";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            if(avg1 > avg2){
                if(avg1 > avg3){
                    position = DuckPos.ONE;
                }else{
                    position = DuckPos.THREE;
                }
            }else{
                if(avg2 > avg3){
                    position = DuckPos.TWO;
                }else{
                    position = DuckPos.THREE;
                }
            }

            switch (stage){
                case 0:
                    telemetry = "active stage is ycrcb" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(ycrcb, blueRect,new Scalar(255,0,0), 2);
                    Imgproc.rectangle(ycrcb, redRect,new Scalar(255,0,0), 2);
                    return ycrcb;
                case 1:
                    telemetry = "active stage is coi" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(blue, blueRect,new Scalar(255,0,0), 2);
                    return blue;
                case 2:
                    telemetry = "active stage is threshold" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(thresholdBlue, blueRect,new Scalar(255,0,0), 2);
                    return thresholdBlue;
                case 3:
                    telemetry = "active stage is input" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(red, redRect,new Scalar(255,0,0), 2);
                    return red;
                case 4:
                    Imgproc.rectangle(thresholdRed, redRect,new Scalar(255,0,0), 2);
                    return thresholdRed;
                case 5:
                    Imgproc.rectangle(yellow, yellowRect,new Scalar(255,0,0), 2);
                    return yellow;
                case 6:
                    Imgproc.rectangle(thresholdYellow, yellowRect,new Scalar(255,0,0), 2);
                    return thresholdYellow;
            }
            return input;
        }
    }

    public enum DuckPos{
        ONE,
        TWO,
        THREE
    }

    public DuckPos getPosition(){
        return position;
    }

    public String toString(){
        return telemetry;
    }
}
