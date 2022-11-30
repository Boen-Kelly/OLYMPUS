
package org.firstinspires.ftc.teamcode.classes;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

@Config
public class AutoAlignPipeline {
    OpenCvCamera webcam;
    Threshold thresh;
    String telemetry = "waiting for input";
    public static Rect midBox = new Rect(185,115,25,25);
    public static int threshVal = 128;

    public static double LH = 40, LS = 20, LV = 80;
    public static double UH = 60, US = 80, UV = 100;

    public static int x = 10, y = 10;


    public AutoAlignPipeline(HardwareMap hardwareMap, String camName){

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
        Mat greyscale = new Mat();
        Mat threshold = new Mat();
        Mat hierarchy = new Mat();
        Mat mask = new Mat();
        Mat filtered = new Mat();
        Mat output = new Mat();
        Mat open = new Mat();
        Mat closed = new Mat();

        Mat kernel = new Mat(5,5, CvType.CV_8UC1);


        double avg1, avg2;

        @Override
        public void onViewportTapped() {
            stage ++;

            if(stage > 1){
                stage = 0;
            }
        }

        @Override
        public Mat processFrame(Mat input){

            Scalar LOWER_BOUND = new Scalar(LH,LS,LV);
            Scalar UPPER_BOUND = new Scalar(UH,US,UV);

//            output = input;

//            Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_BGR2HSV);

            Core.inRange(input, LOWER_BOUND, UPPER_BOUND, mask);

            telemetry = "h: " + input.get(x,y)[0] + "\ns: " + input.get(x,y)[1] + "\nv: " + input.get(x,y)[2];

//            Imgproc.morphologyEx(mask, open, Imgproc.MORPH_OPEN, kernel);
//            Imgproc.morphologyEx(mask, closed, Imgproc.MORPH_OPEN, kernel);

//            Core.add(open, closed, filtered);

//            Core.bitwise_and(input, input, output, mask);

            switch (stage){
                case 0:
//                    telemetry = "active stage is ycrcb" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(greyscale, midBox,new Scalar(255,0,0), 2);
                    Imgproc.circle(input, new Point(x,y), 2, new Scalar(255,0,0),-1);
                    return input;
                case 1:
//                    telemetry = "active stage is coi" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(greyscale, midBox,new Scalar(255,0,0), 2);
                    Imgproc.circle(mask, new Point(x,y), 2, new Scalar(255,0,0),-1);
                    return mask;
                case 2:
                    telemetry = "active stage is threshold" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(threshold, midBox, new Scalar(255,0,0), 2);
                    return open;
                case 3:
                    telemetry = "active stage is input" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
//                    Imgproc.rectangle(input, midBox,new Scalar(255,0,0), 2);
                    return  open;
                case 4:
                    return open;
                case 5:
                    return input;
                case 6:
                    return output;
                default:
                    return output;
            }
        }
    }

    public String toString(){
        return telemetry;
    }
}
