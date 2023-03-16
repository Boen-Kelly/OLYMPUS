package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class AlignThread implements Runnable{
    private boolean usingFrontCam;
    private boolean masterEngaged = false;
    boolean isPole = true;
    double distanceToTarget = 7;
    double robotHeading = 0;
    public static double headingError = 10;
    public static double xError = 5;
    public static double yError = 1.5;
    public static double kPAngle = .023222;
    public static double kDAngle = 0;
    public static double kPY = .05;
    public static double kPYDist = 0.035;
    public static double kPX = .05;
    public static double maxAngleSpeed = .3;
    public static double xMaxSpeed = 1;
    public static double yMaxSpeed = .5;
    public static double frontPoint = .84, backPoint = .8;
    public final double FRONT_CAM_X_OFFSET = -5.75;
    public final double FRONT_CAM_Y_OFFSET = -5.75;
    public static double BACK_CAM_X_OFFSET = 5.75;
    public static double BACK_CAM_Y_OFFSET = 0;
    public double xDist = 0, yDist = 0;
    double angleError = 0, lastAngleError = 0;
    double I = 0;
    public double straight = 0, strafe = 0, rotate = 0;
    double prevStraight = 0, prevStrafe = 0, prevRotate = 0;
    public double xSpeed = 0, ySpeed = 0;

    double angles = 0;

    public double frontDist, backDist;

    private AutoAlignPipeline pipeline;
    public CameraAimThread camera;
    Thread cameraThread;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double lastTime = 0;
    public AlignThread(HardwareMap hardwareMap, AutoAlignPipeline pipeline, BNO055IMU.Parameters parameters){
        this.pipeline = pipeline;;

        camera = new CameraAimThread(hardwareMap, pipeline);
        cameraThread = new Thread(camera);
        cameraThread.start();

        camera.pointCam(frontPoint, backPoint);
    }

    public void run() {
        timer.reset();
        while (!Thread.interrupted()){
            // master align code
            masterAlign(distanceToTarget, robotHeading, usingFrontCam, isPole);
            if(masterEngaged) {
                xSpeed = xDist * kPX;

                strafe = (.9 * Range.clip(xSpeed, -xMaxSpeed, xMaxSpeed)) + (.1 * prevStrafe);

                straight = (.9 * yAlign(distanceToTarget,usingFrontCam)) + (.1 * prevStraight);
                rotate = ( .9 * gyroAlign(robotHeading)) + (.1 * prevRotate);

                prevStraight = straight;
                prevStrafe = strafe;
                prevRotate = rotate;
            }else{
                straight = 0;
                strafe = 0;
                rotate = 0;
            }
            //----------------------------------------

            lastTime = timer.time();
        }
        cameraThread.interrupt();
    }

    public double angleToCam(){
        double ratio = pipeline.frontPoleDetector.getDistance() + 160.0 / 320.0;
        return (ratio*55)/2;
    }

    public double getRobotDistance(boolean usingFrontCam, boolean isPole){
        if(isPole) {
            return 539.8011184016967 / pipeline.getMaxWidth(usingFrontCam);
        }else{
            return 1934.0789619903978 / pipeline.getMaxWidth(usingFrontCam);
        }
    }

    public void masterAlign(double distance, double robotAngle, boolean usingFrontCam, boolean isPole){
        double r = getRobotDistance(usingFrontCam, isPole);
        double angle = camera.getAngle(usingFrontCam);

        if(usingFrontCam) {
            if(frontDist > 150) {
                yDist = (r * Math.sin(Math.toRadians(angle))) - FRONT_CAM_Y_OFFSET - distance;
                xDist = (r * Math.cos(Math.toRadians(angle))) - FRONT_CAM_X_OFFSET;
            }else{
                yDist = frontDist - distance;
                xDist = 0;
            }
        }else {
            if(backDist > 150) {
                yDist = (r * Math.sin(Math.toRadians(angle))) - BACK_CAM_Y_OFFSET - distance;
                xDist = (r * Math.cos(Math.toRadians(angle))) - BACK_CAM_X_OFFSET;
            }else{
                yDist = backDist - distance;
                xDist = 0;
            }
        }
    }

    public double align(double camPos, double maxPower, boolean usingFrontCam){
        double speed;
        if(usingFrontCam){
            camera.front.setPosition(camPos);
            speed = pipeline.frontPoleDetector.getDistance()/120;
        }else{
            camera.back.setPosition(camPos);
            speed = pipeline.backPoleDetector.getDistance()/120;
        }


        double output = Math.min(maxPower,speed);
        output = Math.max(-maxPower, output);
        return output;
    }

//    public void turnToAlign(double camPos, boolean usingFrontCam) {
//        time.reset();
//        while (time.milliseconds() < 5000 && (time.milliseconds() - startTime) < 500) {
//            fl.setPower(align(camPos, .1, usingFrontCam));
//            br.setPower(-align(camPos, .1, usingFrontCam));
//            bl.setPower(align(camPos, .1, usingFrontCam));
//            fr.setPower(-align(camPos, .1 , usingFrontCam));
//
//            if(!aligned()){
//                startTime = time.milliseconds();
//            }
//        }
//    }

//    public void strafeToAlign(double camPos, boolean usingFrontCam){
//        time.reset();
//        while(time.milliseconds() < 5000 && (time.milliseconds() - startTime) < 500) {
//            fl.setPower(align(camPos, .5, usingFrontCam));
//            br.setPower(align(camPos, .5, usingFrontCam));
//            bl.setPower(-align(camPos, .5, usingFrontCam));
//            fr.setPower(-align(camPos, .5, usingFrontCam));
//
//            if(!aligned()){
//                startTime = time.milliseconds();
//            }
//        }
//    }

    public boolean aligned(){
        return (robotHeading - headingError <= angles && angles <= robotHeading + headingError) && (-xError <= xDist && xDist <= xError) && (distanceToTarget - yError <= yDist && yDist <= distanceToTarget + yError);
    }

    public double gyroAlign(double robotAngle){
        angleError = angles - robotAngle;
        double P = angleError * kPAngle;
        double D = (angleError - lastAngleError) * kDAngle;

        lastAngleError = angleError;

        return Range.clip(P + D, -maxAngleSpeed,maxAngleSpeed);
    }

    public double yAlign(double distance, boolean usingFrontCam){
        ySpeed = yDist * kPY;
        ySpeed = Range.clip(ySpeed, -yMaxSpeed, yMaxSpeed);
        return ySpeed;
    }

    public void engageMaster(double distanceToTarget, boolean usingFrontCam, double robotHeading, boolean isPole){
        masterEngaged = true;
        this.distanceToTarget = distanceToTarget;
        this.usingFrontCam = usingFrontCam;
        this.robotHeading = robotHeading;
        this.isPole = isPole;
    }

    public void disengageMaster(){
        masterEngaged = false;
    }

    public double getCycleTime(){
        return timer.time() - lastTime;
    }

    public void updateHardware(double angle, double frontDist, double backDist){
        angles = angle;
        this.frontDist = frontDist;
        this.backDist = backDist;
    }
}
