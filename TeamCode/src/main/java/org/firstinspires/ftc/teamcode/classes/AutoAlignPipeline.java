
package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
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

import java.util.ArrayList;
import java.util.List;

@Config
public class AutoAlignPipeline {
    OpenCvCamera webcam;
    Threshold thresh;
    String telemetry = "waiting for input";
    public static Rect midBox = new Rect(185,115,25,25);
    public static int threshVal = 128;

    public static double LH = 90, LS = 170, LV = 160;
    public static double UH = 120, US = 255, UV = 255;

    public static int x = 10, y = 10;
    public static double boxWidth = 40;
    public static double frontPoint = .75, backPoint = .75;

    polePos pos = polePos.ON_POINT;

    DcMotor bl, br, fl, fr;
    Servo front, back;


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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,  camName), cameraMonitorViewId);

        thresh = new Threshold();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(thresh);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

        }
        });

            telemetry = "waiting for start";
    }

    class Threshold extends OpenCvPipeline {

        int stage = 0;
        Mat hsv = new Mat();
        Mat threshold = new Mat();
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
        double distance = 0;


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

            Scalar LOWER_BOUND = new Scalar(LH,LS,LV);
            Scalar UPPER_BOUND = new Scalar(UH,US,UV);

//            output = input;

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

//            Core.extractChannel(hsv, h, 0); //90-100
//            Core.extractChannel(hsv, s, 1); //170-255
//            Core.extractChannel(hsv, v, 2); //230-200
//            Core.inRange(v, LOWER_BOUND, UPPER_BOUND, v);

            Core.inRange(hsv, LOWER_BOUND, UPPER_BOUND, mask);

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
                    return hsv;
                case 2:
                    telemetry = "active stage is threshold" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(threshold, midBox, new Scalar(255,0,0), 2);
                    Imgproc.circle(hsv, new Point(x,y), 2, new Scalar(255,0,0),-1);
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

    public String toString(){
        return telemetry;
    }

    public enum polePos{
        LEFT,
        RIGHT,
        ON_POINT
    }

    public polePos getPos(){
        return pos;
    }

    public void align(){
        while(!pos.equals(polePos.ON_POINT)) {
            telemetry = "aligning, pos is: " + pos;
            if (pos.equals(polePos.RIGHT)) {
                bl.setPower(.3);
                fl.setPower(.3);
                br.setPower(-.3);
                fr.setPower(-.3);
            } else if (pos.equals(polePos.LEFT)) {
                bl.setPower(-.3);
                fl.setPower(-.3);
                br.setPower(.3);
                fr.setPower(.3);
            } else {
                bl.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                fr.setPower(0);
            }
        }
    }
}
