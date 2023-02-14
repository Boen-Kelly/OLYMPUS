package org.firstinspires.ftc.teamcode.oldcode.tests.classes;

import static org.firstinspires.ftc.teamcode.oldcode.tests.classes.AutoAlignPipeline.boxWidth;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class AlignThread implements Runnable{
    private boolean usingFrontCam;
    private boolean masterEngaged = false;
    private boolean alignEngaged = false;
    private double distanceToTarget = 0;
    public static double rate = .0005;
    public static double frontPoint = .84, backPoint = .8;
    public final double ROBOT_X = -3.75;
    public final double ROBOT_Y = -5.75;
    public double xDist = 0, yDist = 0;
    double startTime = 0;
    ElapsedTime time = new ElapsedTime();
    DcMotor bl, br, fl, fr;


    Servo front, back;

    private AutoAlignPipeline pipeline;
    public AlignThread(HardwareMap hardwareMap, AutoAlignPipeline pipeline){

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        this.pipeline = pipeline;
        front = hardwareMap.get(Servo.class, "front");
        back = hardwareMap.get(Servo.class, "back");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front.scaleRange(.13, .85);
        back.scaleRange(.48,1);

        front.setPosition(frontPoint);
        back.setPosition(backPoint);
    }

    public void run() {
        while (!Thread.interrupted()){
            // master align code
            if(masterEngaged) {
                masterAlign(distanceToTarget, usingFrontCam);
                aimCam(usingFrontCam);
            }
            //----------------------------------------
        }
    }

    public void aimCam (boolean isFrontCam) {
        double speed = 0;

        if(isFrontCam) {
            if (pipeline.frontPoleDetector.getDistance() > 10 || pipeline.frontPoleDetector.getDistance() < -10){
                speed = (pipeline.frontPoleDetector.getDistance() / 120) * rate;
            }
            frontPoint += speed;

            frontPoint = Range.clip(frontPoint, 0,1);

            front.setPosition(frontPoint);
        }else{
            if (pipeline.backPoleDetector.getDistance() > 10 || pipeline.backPoleDetector.getDistance() < -10){
                speed = (pipeline.backPoleDetector.getDistance() / 120) * rate;
            }
            backPoint += speed;

            backPoint = Range.clip(backPoint, 0, 1);

            back.setPosition(backPoint);
        }
    }

    public double getRobotDistance(boolean usingFrontCam){
        return 666.1332771378968/pipeline.getMaxWidth(usingFrontCam);
    }

    public double getAngle(boolean usingFrontCam){
        if(usingFrontCam){
            return 180 - front.getPosition() * 180;
        }else{
            return 180 - back.getPosition() * 115;
        }
    }

    public void masterAlign(double distance, boolean usingFrontCam){
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

            if(!aligned(usingFrontCam)){
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

            if(!aligned(usingFrontCam)){
                startTime = time.milliseconds();
            }
        }
    }

    public boolean aligned(boolean isUsingFrontCam){
        if(isUsingFrontCam) {
            return -boxWidth / 2 < pipeline.frontPoleDetector.getDistance() && pipeline.frontPoleDetector.getDistance() < boxWidth / 2;
        }else{
            return -boxWidth / 2 < pipeline.backPoleDetector.getDistance() && pipeline.backPoleDetector.getDistance() < boxWidth / 2;
        }
    }

    public void engageMaster(double distanceToTarget, boolean usingFrontCam){
        masterEngaged = true;
        this.distanceToTarget = distanceToTarget;
        this.usingFrontCam = usingFrontCam;
    }

    public void disengageMaster(){
        masterEngaged = false;
    }

    public void engageAlign(){
        alignEngaged = true;
    }

    public void disengageAlign(){
        alignEngaged = false;
    }
}
