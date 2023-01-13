
package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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
    public static double frontPoint = .84, backPoint = .8;
    double startTime = 0;
    double distance = 0;


    polePos pos = polePos.ON_POINT;

    DcMotor bl, br, fl, fr;
    Servo front, back;
    DistanceSensor backDistance;

    ElapsedTime time = new ElapsedTime();

    DuckPos position;
    public static Rect redRect = new Rect(105,245,15,15);
    public static Rect blueRect = new Rect(105,245,15,15);
    public static Rect yellowRect = new Rect(105,245,15,15);
    public static int threshRed = 145, threshBlue = 150, threshYellow = 160;
    SleeveDetector sleeveDetector;


    public AutoAlignPipeline(HardwareMap hardwareMap, String camName){

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");

        front = hardwareMap.get(Servo.class, "front");
        back = hardwareMap.get(Servo.class, "back");

        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");

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
        Mat yellow = new Mat();
        Mat red = new Mat();
        Mat blue = new Mat();
        Mat hierarchy = new Mat();
        Mat mask = new Mat();
        Mat filtered = new Mat();
        Mat output = new Mat();
        Mat open = new Mat();
        Mat closed = new Mat();

        Rect boundingRect = new Rect();

        Point top, bottom;

        Mat kernel = new Mat(12,12, CvType.CV_8UC1);

        double maxWidth = 0;


        double avg1, avg2;

        @Override
        public void onViewportTapped() {
            stage ++;

            if(stage > 6){
                stage = 0;
            }
        }

        @Override
        public Mat processFrame(Mat input){
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            final Scalar LOWER_BOUND_YELLOW = new Scalar(90,170,160);
            final Scalar UPPER_BOUND_YELLOW = new Scalar(120,255,255);

            Imgproc.cvtColor(input, yellow, Imgproc.COLOR_BGR2HSV);

//            Core.extractChannel(hsv, h, 0); //90-100
//            Core.extractChannel(hsv, s, 1); //170-255
//            Core.extractChannel(hsv, v, 2); //230-200
//            Core.inRange(v, LOWER_BOUND, UPPER_BOUND, v);

            Core.inRange(yellow, LOWER_BOUND_YELLOW, UPPER_BOUND_YELLOW, mask);

            Imgproc.morphologyEx(mask, open, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, closed, Imgproc.MORPH_OPEN, kernel);

            Core.add(open, closed, filtered);

            Imgproc.findContours(filtered, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//            Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 1);


            for(Mat box:contours) {
                boundingRect = Imgproc.boundingRect(box);

                if(boundingRect.width > maxWidth){
                    maxWidth = boundingRect.width;

                    Imgproc.rectangle(input, boundingRect, new Scalar(255, 0, 0), 2);

                    top = new Point(boundingRect.x + boundingRect.width * .5, boundingRect.y);
                    bottom = new Point(boundingRect.x + boundingRect.width * .5, boundingRect.y + boundingRect.height);

                    Imgproc.line(input, top, bottom, new Scalar(255, 0, 0));
                }
            }

            if(top != null) {
                distance = top.x - 120;
                Imgproc.line(input, new Point(top.x, 10), new Point(120, 10), new Scalar(0, 255, 0), 2);
            }

            Imgproc.rectangle(input,new Point(120-boxWidth/2,5), new Point(120+boxWidth/2,15), new Scalar(0,0,255), 2);

            if(distance > boxWidth/2){
                pos = polePos.RIGHT;
            }else if(distance < -boxWidth/2){
                pos = polePos.LEFT;
            }else{
                pos = polePos.ON_POINT;
            }

            telemetry = "contours.length: " + contours.size() + "\nwidth: " + maxWidth + "\ndistance: " + distance + "\npos: " + pos;

            maxWidth = 0;

            switch (stage){
                case 0:
//                    telemetry = "active stage is ycrcb" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(greyscale, midBox,new Scalar(255,0,0), 2);
                    return input;
                case 1:
//                    telemetry = "active stage is coi" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(greyscale, midBox,new Scalar(255,0,0), 2);
                    Imgproc.circle(mask, new Point(x,y), 2, new Scalar(255,0,0),-1);
                    return yellow;
                case 2:
                    telemetry = "active stage is threshold" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(threshold, midBox, new Scalar(255,0,0), 2);
                    Imgproc.circle(yellow, new Point(x,y), 2, new Scalar(255,0,0),-1);
                    return mask;
                case 3:
                    telemetry = "active stage is input" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(input, midBox,new Scalar(255,0,0), 2);
                    return open;
                case 4:
                    return closed;
                case 5:
                    return filtered;
                case 6:
                    return input;
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

    public enum polePos{
        LEFT,
        RIGHT,
        ON_POINT
    }

    public polePos getPolePos(){
        return pos;
    }

    public void align(){
        time.reset();
        while(time.milliseconds() < 5000 && (time.milliseconds() - startTime) < 500) {
            telemetry = "aligning, pos is: " + pos;
            if (pos.equals(polePos.RIGHT)) {
                startTime = time.milliseconds();
                bl.setPower(.1);
                fl.setPower(.1);
                br.setPower(-.1);
                fr.setPower(-.1);
            } else if (pos.equals(polePos.LEFT)) {
                startTime = time.milliseconds();
                bl.setPower(-.1);
                fl.setPower(-.1);
                br.setPower(.1);
                fr.setPower(.1);
            } else if(pos.equals(polePos.ON_POINT)){
                bl.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                fr.setPower(0);
            }
        }
    }

    public void useFrontCam(){
        switchableWebcam.setActiveCamera(frontCam);
        switchableWebcam.setPipeline(sleeveDetector);
    }

    public void useBackCam() {
        switchableWebcam.setActiveCamera(backCam);
        switchableWebcam.setPipeline(poleDetector);
    }

    public double align(double camPos, double maxPower, boolean usingFrontCam){
        if(usingFrontCam){
            front.setPosition(camPos);
        }else{
            back.setPosition(camPos);
        }

        double speed = distance/120;

        double output =  Math.min(maxPower,speed);
        output = Math.max(-maxPower, output);
        return output;
    }


    public void turnToAlign(double camPos, boolean usingFrontCam) {
        time.reset();
        while (time.milliseconds() < 5000 && (time.milliseconds() - startTime) < 500) {
            fl.setPower(align(camPos, .1, usingFrontCam));
            br.setPower(-align(camPos, .1, usingFrontCam));
            bl.setPower(align(camPos, .1, usingFrontCam));
            fr.setPower(-align(camPos, .1 , usingFrontCam));

            if(!pos.equals(polePos.ON_POINT)){
                startTime = time.milliseconds();
            }
        }
    }

    public void strafeToAlign(double camPos, boolean usingFrontCam){
        fl.setPower(-align(camPos, .5, usingFrontCam));
        br.setPower(-align(camPos, .5, usingFrontCam));
        bl.setPower(align(camPos, .5, usingFrontCam));
        fr.setPower(align(camPos, .5, usingFrontCam));
    }

    public double targetDistance(double distance, double maxPower){
        double speed = (backDistance.getDistance(DistanceUnit.CM) - distance)/30;

        speed = Math.min(maxPower, speed);

        return speed;
    }
}
