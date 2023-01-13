
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
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
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
    AprilTagDetectionPipeline sleeveDetector;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


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
        sleeveDetector = new AprilTagDetectionPipeline(tagsize,fx,fy,cx,cy);
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

    public class AprilTagDetectionPipeline extends OpenCvPipeline {
        private long nativeApriltagPtr;
        private Mat grey = new Mat();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();

        private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
        private final Object detectionsUpdateSync = new Object();

        Mat cameraMatrix;

        Scalar blue = new Scalar(7,197,235,255);
        Scalar red = new Scalar(255,0,0,255);
        Scalar green = new Scalar(0,255,0,255);
        Scalar white = new Scalar(255,255,255,255);

        double fx;
        double fy;
        double cx;
        double cy;

        // UNITS ARE METERS
        double tagsize;
        double tagsizeX;
        double tagsizeY;

        private float decimation;
        private boolean needToSetDecimation;
        private final Object decimationSync = new Object();

        public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy)
        {
            this.tagsize = tagsize;
            this.tagsizeX = tagsize;
            this.tagsizeY = tagsize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            constructMatrix();

            // Allocate a native context object. See the corresponding deletion in the finalizer
            nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        public void finalize()
        {
            // Might be null if createApriltagDetector() threw an exception
            if(nativeApriltagPtr != 0)
            {
                // Delete the native context we created in the constructor
                AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
                nativeApriltagPtr = 0;
            }
            else
            {
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
            }
        }

        @Override
        public Mat processFrame(Mat input)
        {
            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync)
            {
                if(needToSetDecimation)
                {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

            synchronized (detectionsUpdateSync)
            {
                detectionsUpdate = detections;
            }

            // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
            // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
            for(AprilTagDetection detection : detections)
            {
                Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
            }

            return input;
        }

        public void setDecimation(float decimation)
        {
            synchronized (decimationSync)
            {
                this.decimation = decimation;
                needToSetDecimation = true;
            }
        }

        public ArrayList<AprilTagDetection> getLatestDetections()
        {
            return detections;
        }

        public ArrayList<AprilTagDetection> getDetectionsUpdate()
        {
            synchronized (detectionsUpdateSync)
            {
                ArrayList<AprilTagDetection> ret = detectionsUpdate;
                detectionsUpdate = null;
                return ret;
            }
        }

        void constructMatrix()
        {
            //     Construct the camera matrix.
            //
            //      --         --
            //     | fx   0   cx |
            //     | 0    fy  cy |
            //     | 0    0   1  |
            //      --         --
            //

            cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

            cameraMatrix.put(0,0, fx);
            cameraMatrix.put(0,1,0);
            cameraMatrix.put(0,2, cx);

            cameraMatrix.put(1,0,0);
            cameraMatrix.put(1,1,fy);
            cameraMatrix.put(1,2,cy);

            cameraMatrix.put(2, 0, 0);
            cameraMatrix.put(2,1,0);
            cameraMatrix.put(2,2,1);
        }

        /**
         * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
         *
         * @param buf the RGB buffer on which to draw the marker
         * @param length the length of each of the marker 'poles'
         * @param rvec the rotation vector of the detection
         * @param tvec the translation vector of the detection
         * @param cameraMatrix the camera matrix used when finding the detection
         */
        void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(0,0,0),
                    new Point3(length,0,0),
                    new Point3(0,length,0),
                    new Point3(0,0,-length)
            );

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Draw the marker!
            Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

            Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
        }

        void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
        {
            //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
            //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(-tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2,-tagHeight/2,-length),
                    new Point3(-tagWidth/2,-tagHeight/2,-length));

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Pillars
            for(int i = 0; i < 4; i++)
            {
                Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
            }

            // Base lines
            //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
            //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
            //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
            //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

            // Top lines
            Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
            Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
            Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
            Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
        }

        /**
         * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
         * original size of the tag.
         *
         * @param points the points which form the trapezoid
         * @param cameraMatrix the camera intrinsics matrix
         * @param tagsizeX the original width of the tag
         * @param tagsizeY the original height of the tag
         * @return the 6DOF pose of the camera relative to the tag
         */
        Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
        {
            // The actual 2d points of the tag detected in the image
            MatOfPoint2f points2d = new MatOfPoint2f(points);

            // The 3d points of the tag in an 'ideal projection'
            Point3[] arrayPoints3d = new Point3[4];
            arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
            arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
            MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

            // Using this information, actually solve for pose
            Pose pose = new Pose();
            Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

            return pose;
        }

        /*
         * A simple container to hold both rotation and translation
         * vectors, which together form a 6DOF pose.
         */
        class Pose
        {
            Mat rvec;
            Mat tvec;

            public Pose()
            {
                rvec = new Mat();
                tvec = new Mat();
            }

            public Pose(Mat rvec, Mat tvec)
            {
                this.rvec = rvec;
                this.tvec = tvec;
            }
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

    public double targetDistance(double distance, double maxPower) {
        double speed = (backDistance.getDistance(DistanceUnit.CM) - distance) / 30;

        speed = Math.min(maxPower, speed);

        return speed;
    }

    public int AprilTagID(){
        ArrayList<AprilTagDetection> detections = sleeveDetector.getLatestDetections();
        if(detections.size() != 0) {
            return detections.get(0).id;
        }else{
            return 6;
        }
    }
}
