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
    double distanceToTarget = 7;
    double robotHeading = 0;
    public static double headingError = 10;
    public static double xError = 5;
    public static double yError = 3;
    public static double kPAngle = .023222;
    public static double kDAngle = 0;
    public static double kPY = .01;
    public static double kPYDist = 0.035;
    public static double kPX = .025;
    public static double maxAngleSpeed = .3;
    public static double xMaxSpeed = 1;
    public static double yMaxSpeed = .5;
    public static double frontPoint = .84, backPoint = .8;
    public final double FRONT_CAM_X_OFFSET = -5.75;
    public final double FRONT_CAM_Y_OFFSET = -5.75;
    public final double BACK_CAM_X_OFFSET = -5.75;
    public final double BACK_CAM_Y_OFFSET = 5.75;
    public double xDist = 0, yDist = 0;
    double angleError = 0, lastAngleError = 0;
    double I = 0;
    public double straight = 0, strafe = 0, rotate = 0;
    public double xSpeed = 0, ySpeed = 0;
    double startTime = 0;

    double globalAngle = 0;

    Orientation lastAngles = new Orientation();
    ElapsedTime time = new ElapsedTime();
//    DcMotor bl, br, fl, fr;

    BNO055IMU imu;

    public DistanceSensor frontDist, backDist;

    private AutoAlignPipeline pipeline;
    public CameraAimThread camera;
    Thread cameraThread;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double lastTime = 0;
    public AlignThread(HardwareMap hardwareMap, AutoAlignPipeline pipeline, BNO055IMU.Parameters parameters){

//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fr = hardwareMap.get(DcMotor.class, "fr");
        this.pipeline = pipeline;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        backDist = hardwareMap.get(DistanceSensor.class, "backDist");

        imu.initialize(parameters);

//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        camera = new CameraAimThread(hardwareMap, pipeline);
        cameraThread = new Thread(camera);
        cameraThread.start();

        camera.pointCam(frontPoint, backPoint);
    }

    public void run() {
        timer.reset();
        while (!Thread.interrupted()){
            // master align code
            masterAlign(distanceToTarget, robotHeading, usingFrontCam);
            if(masterEngaged) {
                xSpeed = xDist * kPX;

                strafe = Range.clip(xSpeed, -xMaxSpeed, xMaxSpeed);

                straight = yAlign(distanceToTarget,usingFrontCam);
                rotate = gyroAlign(robotHeading);

//                if(usingFrontCam) {
//                    fl.setPower(gyroAlign(robotHeading) + xSpeed + yAlign(distanceToTarget, usingFrontCam));
//                    fr.setPower(-gyroAlign(robotHeading) - xSpeed + yAlign(distanceToTarget, usingFrontCam));
//                    bl.setPower(gyroAlign(robotHeading) - xSpeed + yAlign(distanceToTarget, usingFrontCam));
//                    br.setPower(-gyroAlign(robotHeading) + xSpeed + yAlign(distanceToTarget, usingFrontCam));
//                }else{
//                    fl.setPower(gyroAlign(robotHeading) - xSpeed - yAlign(distanceToTarget, usingFrontCam));
//                    fr.setPower(-gyroAlign(robotHeading) + xSpeed - yAlign(distanceToTarget, usingFrontCam));
//                    bl.setPower(gyroAlign(robotHeading) + xSpeed - yAlign(distanceToTarget, usingFrontCam));
//                    br.setPower(-gyroAlign(robotHeading) - xSpeed - yAlign(distanceToTarget, usingFrontCam));
//                }
//                masterEngaged = !aligned();
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

    public double getRobotDistance(boolean usingFrontCam){
        return 539.8011184016967/pipeline.getMaxWidth(usingFrontCam);
    }

    public void masterAlign(double distance, double robotAngle, boolean usingFrontCam){
        double r = getRobotDistance(usingFrontCam);
        double angle = camera.getAngle(usingFrontCam);

        if(usingFrontCam) {
            xDist = (r * Math.cos(Math.toRadians(angle))) - FRONT_CAM_X_OFFSET;
            yDist = (r * Math.sin(Math.toRadians(angle))) - FRONT_CAM_Y_OFFSET - distance;
        }else {
            xDist = (r * Math.cos(Math.toRadians(angle))) - BACK_CAM_X_OFFSET;
            yDist = (r * Math.sin(Math.toRadians(angle))) - BACK_CAM_Y_OFFSET - distance;
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
        return (robotHeading - headingError <= gyroHeading() && gyroHeading() <= robotHeading + headingError) && (-xError <= xDist && xDist <= xError) && (distanceToTarget - yError <= yDist && yDist <= distanceToTarget + yError);
    }

    public double gyroHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double gyroAlign(double robotAngle){
        angleError = gyroHeading() - robotAngle;
        double P = angleError * kPAngle;
        double D = (angleError - lastAngleError) * kDAngle;

        lastAngleError = angleError;

        return Range.clip(P + D, -maxAngleSpeed,maxAngleSpeed);
    }

    public double yAlign(double distance, boolean usingFrontCam){
        if(usingFrontCam){
            if(frontDist.getDistance(DistanceUnit.INCH) > 200){
                ySpeed = yDist * kPY;
            }else{
                ySpeed = (frontDist.getDistance(DistanceUnit.INCH) - distance) * kPYDist;
            }
        }else{
            if(backDist.getDistance(DistanceUnit.INCH) > 200){
                ySpeed = yDist * kPY;
            }else{
                ySpeed = (backDist.getDistance(DistanceUnit.INCH) - distance) * kPYDist;
            }
        }
        ySpeed = Range.clip(ySpeed, -yMaxSpeed, yMaxSpeed);
        return ySpeed;
    }

    public void engageMaster(double distanceToTarget, boolean usingFrontCam, double robotHeading){
        masterEngaged = true;
        this.distanceToTarget = distanceToTarget;
        this.usingFrontCam = usingFrontCam;
        this.robotHeading = robotHeading;
    }

    public void disengageMaster(){
        masterEngaged = false;
    }

    public double getCycleTime(){
        return timer.time() - lastTime;
    }
}
