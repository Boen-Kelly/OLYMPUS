package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class CameraAimThread implements Runnable{
    AutoAlignPipeline pipeline;
    public Servo front;
    public Servo back;
    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double prevCamAngle = 0, prevTime = 0;
    private boolean isFrontCam = false;
    private boolean frontEnabled = false;
    private boolean backEnabled = false;

    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static double kPFrontCam = .004;
    public static double kIFrontCam = 0;
    public static double kDFrontCam = 0.0000025;
    public static double kPBackCam = 0;
    public static double kIBackCam = 0;
    public static double kDBackCam = 0;
    public static double frontPoint = .6;
    public static double backPoint = .5;

    public CameraAimThread (HardwareMap hardwareMap, AutoAlignPipeline pipeline){
        front = hardwareMap.get(Servo.class, "front");
        back = hardwareMap.get(Servo.class, "back");
        this.pipeline = pipeline;

        front.scaleRange(0, .63);
        back.scaleRange(.48,1);
    }

    @Override
    public void run() {
        while(!Thread.interrupted()) {
            if (frontEnabled) {
                if (!(-20 < pipeline.frontPoleDetector.getDistance() && pipeline.frontPoleDetector.getDistance() < 20)) {
                    P = (pipeline.frontPoleDetector.getDistance() / 160) * kPFrontCam;
                    I += pipeline.frontPoleDetector.getDistance() * kIFrontCam;
                    D = ((pipeline.frontPoleDetector.getDistance() - prevCamAngle) / (time.milliseconds() - prevTime)) * kDFrontCam;
                }

                frontPoint += P + I + D;
                frontPoint = Range.clip(frontPoint, 0, 1);

                prevCamAngle = pipeline.frontPoleDetector.getDistance();

                front.setPosition(frontPoint);
            } else if (backEnabled){
                if (!(-20 < pipeline.backPoleDetector.getDistance() && pipeline.backPoleDetector.getDistance() < 20)) {
                    P = (pipeline.backPoleDetector.getDistance() / 120) * kPBackCam;
                    I += pipeline.backPoleDetector.getDistance() * kIBackCam;
                    D = ((pipeline.backPoleDetector.getDistance() - prevCamAngle) / (time.milliseconds() - prevTime)) * kDBackCam;
                }

                backPoint += P + I + D;

                backPoint = Range.clip(backPoint, 0, 1);

                prevCamAngle = pipeline.frontPoleDetector.getDistance();

                back.setPosition(backPoint);
            }else{
                front.setPosition(frontPoint);
                back.setPosition(backPoint);
            }
            prevTime = time.milliseconds();
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
    }

    public double getAngle(boolean usingFrontCam){
        if(usingFrontCam){
            return 180 - (front.getPosition() * 180) - 50.4 + 90;
        }else{
            return 180 - back.getPosition() * 115;
        }
    }
}
