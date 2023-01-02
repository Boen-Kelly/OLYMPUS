
package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
public class AutoAlignPipeline {
    WebcamName backCam;
    WebcamName frontCam;
    OpenCvSwitchableWebcam switchableWebcam;

    PoleDetector poleDetector;
    String telemetry = "waiting for input";
    public static Rect midBox = new Rect(185,115,25,25);
    public static int threshVal = 128;

    public static double LH = 90, LS = 170, LV = 160;
    public static double UH = 120, US = 255, UV = 255;

    public static int x = 10, y = 10;
    public static double boxWidth = 20;
    public static double frontPoint = .84, backPoint = .75;
    double startTime = 0;
    double distance = 0;
    double maxWidth = 0;
    public static double rate = .0005;

    boolean usingFrontCam = true;

    DcMotor bl, br, fl, fr;
    Servo front, back;

    ElapsedTime time = new ElapsedTime();

    DuckPos position;
    public static Rect redRect = new Rect(105,245,15,15);
    public static Rect blueRect = new Rect(105,245,15,15);
    public static Rect yellowRect = new Rect(105,245,15,15);
    Rect bigRect = new Rect();
    public static int threshRed = 145, threshBlue = 150, threshYellow = 160;
    SleeveDetector sleeveDetector;


    public AutoAlignPipeline(HardwareMap hardwareMap, String camName){

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");

        front = hardwareMap.get(Servo.class, "front");
        back = hardwareMap.get(Servo.class, "back");

        front.setPosition(frontPoint);
        back.setPosition(backPoint);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backCam = hardwareMap.get(WebcamName.class, "Webcam 2");
        frontCam = hardwareMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, frontCam, backCam);

        poleDetector = new PoleDetector();
        sleeveDetector = new SleeveDetector();
        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                switchableWebcam.setPipeline(sleeveDetector);
                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

        }
        });


            telemetry = "waiting for start";
    }

    class PoleDetector extends OpenCvPipeline {

        int stage = 0;
        Mat hsv = new Mat();
        Mat h = new Mat();
        Mat s = new Mat();
        Mat v = new Mat();
        Mat hierarchy = new Mat();
        Mat maskYellow = new Mat();
        Mat maskRed = new Mat();
        Mat maskBlue = new Mat();
        Mat filteredYellow = new Mat();
        Mat filteredRed = new Mat();
        Mat filteredBlue = new Mat();
        Mat output = new Mat();
        Mat openYellow = new Mat();
        Mat closedYellow = new Mat();
        Mat openRed = new Mat();
        Mat closedRed = new Mat();
        Mat openBlue = new Mat();
        Mat closedBlue = new Mat();

        Rect boundingRect = new Rect();

        Point top, bottom;

        Mat kernel = new Mat(12,12, CvType.CV_8UC1);

        double avg1, avg2;

        @Override
        public void onViewportTapped() {
            stage ++;

            if(stage > 4){
                stage = 0;
            }
        }

        @Override
        public Mat processFrame(Mat input){
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            final Scalar LOWER_BOUND_YELLOW = new Scalar(LH,LS,LV);
            final Scalar UPPER_BOUND_YELLOW = new Scalar(UH,US,UV);
            final Scalar LOWER_BOUND_RED = new Scalar(110,120,150);//110, 120, 150,
            final Scalar UPPER_BOUND_RED = new Scalar(150,255,255);
            Scalar LOWER_BOUND_BLUE = new Scalar(0,100,140);
            Scalar UPPER_BOUND_BLUE = new Scalar(30,255,255);


            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

//            Core.extractChannel(hsv, h, 0);
//            Core.extractChannel(hsv, s, 1);
//            Core.extractChannel(hsv, v, 2);
//
//            Imgproc.threshold(h,h,LH, 255, Imgproc.THRESH_BINARY);
//            Imgproc.threshold(s,s,LS, 255, Imgproc.THRESH_BINARY);
//            Imgproc.threshold(v,v,LV, 255, Imgproc.THRESH_BINARY);

            Core.inRange(hsv, LOWER_BOUND_RED, UPPER_BOUND_RED, maskRed);
            Core.inRange(hsv, LOWER_BOUND_YELLOW, UPPER_BOUND_YELLOW, maskYellow);
            Core.inRange(hsv, LOWER_BOUND_BLUE, UPPER_BOUND_BLUE, maskBlue);

            Imgproc.morphologyEx(maskYellow, openYellow, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskYellow, closedYellow, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(maskRed, openRed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskRed, closedRed, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(maskBlue, openBlue, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskBlue, closedBlue, Imgproc.MORPH_CLOSE, kernel);

            Core.add(openYellow, closedYellow, filteredYellow);
            Core.add(openRed, closedRed, filteredRed);
            Core.add(openBlue, closedBlue, filteredBlue);

            Core.add(filteredRed, filteredYellow, output);
            Core.add(output, filteredBlue, output);

            Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//            Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 1);

            List<MatOfPoint2f> newContours = new ArrayList<>();

            for (MatOfPoint mat : contours) {

                MatOfPoint2f newPoint = new MatOfPoint2f(mat.toArray());
                newContours.add(newPoint);

            }


            for(Mat box:contours) {
                boundingRect = Imgproc.boundingRect(box);

                if(boundingRect.width > maxWidth){
                    maxWidth = boundingRect.width;
                    bigRect = boundingRect;
                }
            }

            if(bigRect != null) {
                Imgproc.rectangle(input, bigRect, new Scalar(255, 0, 0), 2);
                top = new Point(bigRect.x + bigRect.width * .5, bigRect.y);
                bottom = new Point(bigRect.x + bigRect.width * .5, bigRect.y + bigRect.height);
                Imgproc.line(input, top, bottom, new Scalar(255, 0, 0));
            }

            if(top != null) {
                distance = top.x - 120;
                Imgproc.line(input, new Point(top.x, 10), new Point(120, 10), new Scalar(0, 255, 0), 2);
            }

            Imgproc.rectangle(input,new Point(120-boxWidth/2,5), new Point(120+boxWidth/2,15), new Scalar(0,0,255), 2);

            telemetry = "contours.length: " + contours.size() + "\nwidth: " + maxWidth + "\ndistance: " + distance;

            maxWidth = Double.MIN_VALUE;

            switch (stage){
                case 0:
//                    telemetry = "active stage is ycrcb" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    return input;
                case 1:
//                    telemetry = "active stage is coi" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.circle(maskYellow, new Point(x,y), 2, new Scalar(255,0,0),-1);
                    return output;
                case 2:
                    telemetry = "active stage is threshold" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.circle(hsv, new Point(x,y), 2, new Scalar(255,0,0),-1);
                    return filteredYellow;
                case 3:
                    telemetry = "active stage is input" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    return filteredRed;
                case 4:
                    return filteredBlue;
                default:
                    return output;
            }
        }
    }

    class SleeveDetector extends OpenCvPipeline {

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

    public DuckPos getSleevePosition(){
        return position;
    }


    public String toString(){
        return telemetry;
    }

    public void useFrontCam(){
        switchableWebcam.setActiveCamera(frontCam);
        switchableWebcam.setPipeline(sleeveDetector);
        usingFrontCam = true;
    }

    public void useBackCam() {
        switchableWebcam.setActiveCamera(backCam);
        switchableWebcam.setPipeline(poleDetector);
        usingFrontCam = false;
    }

    public void aimCam () {
        double speed = 0;

        if(distance > 10 || distance < -10) {
            speed = (distance / 120) * rate;
        }
        backPoint += speed;

        back.setPosition(backPoint);
    }

    public double align(double camPos, double maxPower){
        if(usingFrontCam){
            front.setPosition(camPos);
        }else{
            back.setPosition(camPos);
        }

        double speed = distance/120;

        return Math.min(maxPower,speed);
    }

    public double width(){
        if(bigRect == null){
            return 0;
        }else{
            return bigRect.width;
        }
    }

//    public double coneDistance(){
//        return
//    }
//
//    public double poleDistance(){
//
//    }
}
