package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

@Config
public class CameraAimThread implements Runnable{
    AutoAlignPipeline pipeline;
    public Servo front;//TODO:
    public Servo back;//TODO:
    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double prevCamAngle = 0, prevTimeMs = 0;
    private boolean isFrontCam = false;
    private boolean frontEnabled = false;
    private boolean backEnabled = false;

    private double prevTimeS = 0;
    private double deltaTime = 0;
    public static double speedFactor = .25;

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double kPFrontCam = .002;
    public static double kIFrontCam = 0;
    public static double kDFrontCam = 0.0000025;
    public static double kPBackCam = 0.0015;
    public static double kIBackCam = 0;
    public static double kDBackCam = .00000001;
    public static double frontPoint = .6;
    public static double backPoint = .5;

    public CameraAimThread (HardwareMap hardwareMap, AutoAlignPipeline pipeline){
        front = hardwareMap.get(Servo.class, "front");
        back = hardwareMap.get(Servo.class, "back");
        this.pipeline = pipeline;

        front.scaleRange(0, .63);
        back.scaleRange(.22,.89);
    }

    @Override
    public void run() {
        while(!Thread.interrupted()) {
            deltaTime = time.time(TimeUnit.MILLISECONDS) - prevTimeS;
            prevTimeS = time.time(TimeUnit.MILLISECONDS);

            if (frontEnabled) {
                if (!(-10 < pipeline.frontPoleDetector.getDistance() && pipeline.frontPoleDetector.getDistance() < 10)) {
                    P = (pipeline.frontPoleDetector.getDistance() / 160) * kPFrontCam;
                    I += pipeline.frontPoleDetector.getDistance() * kIFrontCam;
                    D = ((pipeline.frontPoleDetector.getDistance() - prevCamAngle) / (time.milliseconds() - prevTimeMs)) * kDFrontCam;
                }

                frontPoint += (P + I + D) * deltaTime * speedFactor;
                frontPoint = Range.clip(frontPoint, 0, 1);

                prevCamAngle = pipeline.frontPoleDetector.getDistance();

                front.setPosition(frontPoint);
            } else if (backEnabled){
                if (!(-10 < pipeline.backPoleDetector.getDistance() && pipeline.backPoleDetector.getDistance() < 10)) {
                    P = (pipeline.backPoleDetector.getDistance() / 120) * kPBackCam;
                    I += pipeline.backPoleDetector.getDistance() * kIBackCam;
                    D = ((pipeline.backPoleDetector.getDistance() - prevCamAngle) / (time.milliseconds() - prevTimeMs)) * kDBackCam;
                }

                backPoint += (P + I + D) * deltaTime * speedFactor;

                backPoint = Range.clip(backPoint, .28, 1);

                prevCamAngle = pipeline.frontPoleDetector.getDistance();

                back.setPosition(backPoint);
            }

            prevTimeMs = time.milliseconds();
        }
    }

    public void enableCam(boolean isFrontCam){
        if(isFrontCam) {
            frontEnabled = true;
        }else{
            backEnabled = true;
        }
    }

    public void disableCam(boolean isFrontCam){
        if(isFrontCam){
            frontEnabled = false;
        }else{
            backEnabled = false;
        }
    }

    public void pointCam(double frontPoint, double backPoint){
        CameraAimThread.frontPoint = frontPoint;
        CameraAimThread.backPoint = backPoint;

        front.setPosition(frontPoint);
        back.setPosition(backPoint);
    }

    public double getAngle(boolean usingFrontCam){
        if(usingFrontCam){
            return 180 - (frontPoint * 180) - 50.4 + 90;
        }else{
            return 180 - (backPoint * 180);
        }
    }

    public double deltaTime(){
        return deltaTime;
    }
}
