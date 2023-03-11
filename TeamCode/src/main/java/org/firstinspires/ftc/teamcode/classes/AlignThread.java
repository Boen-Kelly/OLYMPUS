package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    private boolean firstLoop = true;
    public static double distanceToTarget = 0;
    double robotHeading = 0;
    public static double kPFrontCam = .03;
    public static double kIFrontCam = 0;
    public static double kDFrontCam = 0;
    public static double kPBackCam = 0;
    public static double kIBackCam = 0;
    public static double kDBackCam = 0;
    public static double headingError = 2;
    public static double xError = 3;
    public static double yError = 3;
    public static double kPAngle = .022222;
    public static double kDAngle = 0;
    public static double kPY = .01;
    public static double kPYDist = 0;
    public static double kPX = .05;
    public static double maxAngleSpeed = .1;
    public static double xMaxSpeed = .5;
    public static double yMaxSpeed = .5;
    public static double frontPoint = .84, backPoint = .8;
    public final double ROBOT_X = -5.75;
    public final double ROBOT_Y = -5.75;
    public double xDist = 0, yDist = 0, totalYDist = 0;
    double angleError = 0, lastAngleError = 0;
    double I = 0;
    public double xSpeed = 0, ySpeed = 0;
    double startTime = 0;

    double globalAngle = 0;
    double prevCamAngle = 0, prevTime = 0;

    Orientation lastAngles = new Orientation();
    ElapsedTime time = new ElapsedTime();
    DcMotor bl, br, fl, fr;

    BNO055IMU imu;

    public Servo front, back;

    public DistanceSensor frontDist, backDist;

    private AutoAlignPipeline pipeline;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double lastTime = 0;
    public AlignThread(HardwareMap hardwareMap, AutoAlignPipeline pipeline, BNO055IMU.Parameters parameters){

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        this.pipeline = pipeline;
        front = hardwareMap.get(Servo.class, "front");
        back = hardwareMap.get(Servo.class, "back");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        backDist = hardwareMap.get(DistanceSensor.class, "backDist");

        imu.initialize(parameters);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front.scaleRange(0, .63);
        back.scaleRange(.48,1);

        front.setPosition(frontPoint);
        back.setPosition(backPoint);
    }

    public void run() {
        timer.reset();
        while (!Thread.interrupted()){
            // master align code
            if(masterEngaged) {
                masterAlign(distanceToTarget, robotHeading, usingFrontCam);
                aimCam(usingFrontCam);

                if(firstLoop){
                    totalYDist = yDist;
                }

                xSpeed = xDist * kPX;

                xSpeed = Range.clip(xSpeed, -xMaxSpeed, xMaxSpeed);

                if(usingFrontCam) {
                    fl.setPower(gyroAlign(robotHeading) + xSpeed + yAlign(distanceToTarget, usingFrontCam));
                    fr.setPower(-gyroAlign(robotHeading) - xSpeed + yAlign(distanceToTarget, usingFrontCam));
                    bl.setPower(gyroAlign(robotHeading) - xSpeed + yAlign(distanceToTarget, usingFrontCam));
                    br.setPower(-gyroAlign(robotHeading) + xSpeed + yAlign(distanceToTarget, usingFrontCam));
                }else{
                    fl.setPower(gyroAlign(robotHeading) - xSpeed - yAlign(distanceToTarget, usingFrontCam));
                    fr.setPower(-gyroAlign(robotHeading) + xSpeed - yAlign(distanceToTarget, usingFrontCam));
                    bl.setPower(gyroAlign(robotHeading) + xSpeed - yAlign(distanceToTarget, usingFrontCam));
                    br.setPower(-gyroAlign(robotHeading) - xSpeed - yAlign(distanceToTarget, usingFrontCam));
                }
//                masterEngaged = !aligned();
                firstLoop = false;
            }else{
                fl.setPower(0);
                bl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
                firstLoop = true;
            }
            //----------------------------------------

            lastTime = timer.time();
        }
    }
    public void aimCam (boolean isFrontCam) {
        double P = 0;
        double D = 0;

        if(isFrontCam) {
            if(!(-20 < pipeline.frontPoleDetector.getDistance() && pipeline.frontPoleDetector.getDistance() < 20)) {
                P = (pipeline.frontPoleDetector.getDistance() / 160) * kPFrontCam;
                I += pipeline.frontPoleDetector.getDistance() * kIFrontCam;
                D = ((pipeline.frontPoleDetector.getDistance() - prevCamAngle)/ (time.milliseconds() - prevTime))* kDFrontCam;
            }

            frontPoint += P + I + D;

            frontPoint = Range.clip(frontPoint, 0,1);

            prevCamAngle = pipeline.frontPoleDetector.getDistance();

            front.setPosition(frontPoint);
        }else{
            if(!(-20 < pipeline.backPoleDetector.getDistance() && pipeline.backPoleDetector.getDistance() < 20)) {
                P = (pipeline.backPoleDetector.getDistance() / 120) * kPBackCam;
                I += pipeline.backPoleDetector.getDistance() * kIBackCam;
                D = ((pipeline.backPoleDetector.getDistance() - prevCamAngle)/ (time.milliseconds() - prevTime))* kDBackCam;
            }

            backPoint += P + I + D;

            backPoint = Range.clip(backPoint, 0,1);

            prevCamAngle = pipeline.frontPoleDetector.getDistance();

            back.setPosition(backPoint);
        }
        prevTime = time.milliseconds();
    }

    public double angleToCam(){
        double ratio = pipeline.frontPoleDetector.getDistance() + 160.0 / 320.0;
        return (ratio*55)/2;
    }

    public double getRobotDistance(boolean usingFrontCam){
        return 666.1332771378968/pipeline.getMaxWidth(usingFrontCam);
    }

    public double getAngle(boolean usingFrontCam){
        if(usingFrontCam){
            return 180 - (front.getPosition() * 180) - 50.4 + gyroHeading();
        }else{
            return 180 - back.getPosition() * 115;
        }
    }

    public void masterAlign(double distance, double robotAngle, boolean usingFrontCam){
        double r = getRobotDistance(usingFrontCam);
        double angle = getAngle(usingFrontCam);

        xDist = (r * Math.cos(Math.toRadians(angle))) - ROBOT_X;
        yDist = (r * Math.sin(Math.toRadians(angle))) - ROBOT_Y - distance;
    }

    public double align(double camPos, double maxPower, boolean usingFrontCam){
        double speed;
        if(usingFrontCam){
            front.setPosition(camPos);
            speed = pipeline.frontPoleDetector.getDistance()/120;
        }else{
            back.setPosition(camPos);
            speed = pipeline.backPoleDetector.getDistance()/120;
        }


        double output = Math.min(maxPower,speed);
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

            if(!aligned()){
                startTime = time.milliseconds();
            }
        }
    }

    public void strafeToAlign(double camPos, boolean usingFrontCam){
        time.reset();
        while(time.milliseconds() < 5000 && (time.milliseconds() - startTime) < 500) {
            fl.setPower(align(camPos, .5, usingFrontCam));
            br.setPower(align(camPos, .5, usingFrontCam));
            bl.setPower(-align(camPos, .5, usingFrontCam));
            fr.setPower(-align(camPos, .5, usingFrontCam));

            if(!aligned()){
                startTime = time.milliseconds();
            }
        }
    }

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
