
package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
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
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class AutoAlignPipeline {
    OpenCvWebcam backCam; //TODO:
    OpenCvWebcam frontCam;//TODO:

    String telemetry = "waiting for input";
    public static int threshVal = 128;

    public static double LH = 90, LS = 160, LV = 200;
    public static double UH = 105, US = 255, UV = 255;

    public static int x = 10, y = 10;
    public static double boxWidth = 20;

    DuckPos position;
    public static Rect redRect = new Rect(105,245,15,15);
    public static Rect blueRect = new Rect(105,245,15,15);
    public static Rect yellowRect = new Rect(105,245,15,15);
    public static int threshRed = 145, threshBlue = 150, threshYellow = 160;

    public AprilTagDetectionPipeline backSleeveDetector, frontSleeveDetector;
    public PoleDetector backPoleDetector, frontPoleDetector;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    ExposureControl frontExposureControl, backExposureControl;
    GainControl frontGainControl, backGainControl;
    WhiteBalanceControl frontWBControl, backWBControl;

    public AutoAlignPipeline(HardwareMap hardwareMap, String camName){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally

        backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        frontCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);

        backSleeveDetector = new AprilTagDetectionPipeline(tagsize,fx,fy,cx,cy);
        frontSleeveDetector = new AprilTagDetectionPipeline(tagsize,fx,fy,cx,cy);
        backPoleDetector = new PoleDetector();
        frontPoleDetector = new PoleDetector();

        backCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                backCam.setPipeline(backPoleDetector);
                backCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                frontCam.setPipeline(frontSleeveDetector);
                frontCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry = "waiting for start";
    }

    public class PoleDetector extends OpenCvPipeline {
        Rect bigRect = new Rect();
        public RotatedRect bigRotatedRect = new RotatedRect();

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
        RotatedRect rotatedBoundingRect = new RotatedRect();

        Point top, bottom;

        Mat kernel = new Mat(6,6, CvType.CV_8UC1);
        double maxWidth = 0;
        double maxRotatedWidth = 0;
        double avg1, avg2;
        double distance = 0;

        boolean yellow = true, red = true, blue = true;


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

            Imgproc.morphologyEx(maskYellow, openYellow, Imgproc.MORPH_OPEN, kernel, new Point(-1,-1), 1, Core.BORDER_REFLECT);
            Imgproc.morphologyEx(maskYellow, closedYellow, Imgproc.MORPH_CLOSE, kernel, new Point(-1,-1), 1, Core.BORDER_REFLECT);
            Imgproc.morphologyEx(maskRed, openRed, Imgproc.MORPH_OPEN, kernel, new Point(-1,-1), 1, Core.BORDER_REFLECT);
            Imgproc.morphologyEx(maskRed, closedRed, Imgproc.MORPH_CLOSE, kernel, new Point(-1,-1), 1, Core.BORDER_REFLECT);
            Imgproc.morphologyEx(maskBlue, openBlue, Imgproc.MORPH_OPEN, kernel, new Point(-1,-1), 1, Core.BORDER_REFLECT);
            Imgproc.morphologyEx(maskBlue, closedBlue, Imgproc.MORPH_CLOSE, kernel, new Point(-1,-1), 1, Core.BORDER_REFLECT);

            Core.add(openYellow, closedYellow, filteredYellow);
            Core.add(openRed, closedRed, filteredRed);
            Core.add(openBlue, closedBlue, filteredBlue);

            if(yellow){
                if(output == null || output.size() != filteredYellow.size()){
                    output = filteredYellow;
                }else{
                    Core.add(filteredYellow, output, output);
                }
            }
            if(red) {
                if(output == null || output.size() != filteredRed.size()){
                    output = filteredRed;
                }else{
                    Core.add(filteredRed, output, output);
                }
            }
            if(blue){
                if(output == null || output.size() != filteredBlue.size()){
                    output = filteredBlue;
                }else{
                    Core.add(filteredBlue, output, output);
                }
            }

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

            for(MatOfPoint2f box:newContours){
                rotatedBoundingRect = Imgproc.minAreaRect(box);

                if(width(rotatedBoundingRect) > maxRotatedWidth){
                    maxRotatedWidth = width(rotatedBoundingRect);
                    bigRotatedRect = rotatedBoundingRect;
                }
            }

            Point[] vertices = new Point[4];

            bigRotatedRect.points(vertices);
            for (int j = 0; j < 4; j++){
                Imgproc.line(input, vertices[j], vertices[(j+1)%4], new Scalar(0,255,0), 2);
            }

            if(bigRect != null) {
                Imgproc.rectangle(input, bigRect, new Scalar(255, 0, 0), 2);
                top = new Point(bigRect.x + bigRect.width * .5, bigRect.y);
                bottom = new Point(bigRect.x + bigRect.width * .5, bigRect.y + bigRect.height);
                Imgproc.line(input, top, bottom, new Scalar(255, 0, 0));
            }

            if(top != null) {
                distance = top.x - 160;
                Imgproc.line(input, new Point(top.x, 10), new Point(160, 10), new Scalar(0, 255, 0), 2);
            }

            Imgproc.rectangle(input,new Point(160-boxWidth/2,5), new Point(160+boxWidth/2,15), new Scalar(0,0,255), 2);

            telemetry = "contours.length: " + contours.size() + "\nwidth: " + maxWidth + "\ndistance: " + distance;

            maxWidth = Double.MIN_VALUE;
            maxRotatedWidth = Double.MIN_VALUE;

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

        public double width(RotatedRect rect){
            if(rect.size == null){
                return 0;
            }else{
                return Math.min(rect.size.width, rect.size.height);
            }
        }

        public double height(RotatedRect rect){
            if(rect.size == null){
                return 0;
            }else{
                return Math.max(rect.size.height, rect.size.width);
            }
        }

        public double angle(){
            if(bigRotatedRect == null){
                return 0;
            }else{
                return bigRotatedRect.angle;
            }
        }

        public double getDistance(){
            return distance;
        }

        public void setColors(boolean yellow, boolean red, boolean blue){
            this.yellow = yellow;
            this.red = red;
            this.blue = blue;
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

    public String setCamVals(double exposure, int gain, int WB){
        frontExposureControl = frontCam.getExposureControl();
        backExposureControl = backCam.getExposureControl();

        frontGainControl = frontCam.getGainControl();
        backGainControl = backCam.getGainControl();

        frontWBControl = frontCam.getWhiteBalanceControl();
        backWBControl = backCam.getWhiteBalanceControl();

        frontExposureControl.setAePriority(false);
        backExposureControl.setAePriority(false);

        frontExposureControl.setMode(ExposureControl.Mode.Manual);
        backExposureControl.setMode(ExposureControl.Mode.Manual);
        frontWBControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        backWBControl.setMode(WhiteBalanceControl.Mode.MANUAL);

        frontExposureControl.setExposure((long)exposure, TimeUnit.MILLISECONDS);
        backExposureControl.setExposure((long)exposure, TimeUnit.MILLISECONDS);

        frontGainControl.setGain(gain);
        backGainControl.setGain(gain);

        frontWBControl.setWhiteBalanceTemperature(WB);
        backWBControl.setWhiteBalanceTemperature(WB);

        return "exposure: " + frontExposureControl.getExposure(TimeUnit.MILLISECONDS) + "\ngain: " + frontGainControl.getGain() + "\nWB: " + frontWBControl.getWhiteBalanceTemperature();
    }

    public int AprilTagID(boolean isUsingFrontCam){
        ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();
        if(isUsingFrontCam) {
            detections = frontSleeveDetector.getLatestDetections();
        }else{
            detections = backSleeveDetector.getLatestDetections();
        }

        if(detections.size() != 0) {
            return detections.get(0).id;
        }else{
            return 6;
        }
    }

    public void setPipelines(String front, String back){
        if(front.equals("pole")){
            frontCam.setPipeline(frontPoleDetector);
        }else{
            frontCam.setPipeline(frontSleeveDetector);
        }

        if(back.equals("pole")){
            backCam.setPipeline(backPoleDetector);
        }else{
            backCam.setPipeline(backSleeveDetector);
        }
    }

    public double getMaxWidth(boolean usingFrontCam){
        if(usingFrontCam){
            return frontPoleDetector.width(frontPoleDetector.bigRotatedRect);
        }else{
            return backPoleDetector.width(backPoleDetector.bigRotatedRect);
        }
    }

    public double getMaxHeight(boolean usingFrontCam){
        if(usingFrontCam){
            return frontPoleDetector.height(frontPoleDetector.bigRotatedRect);
        }else{
            return backPoleDetector.height(backPoleDetector.bigRotatedRect);
        }
    }
}
