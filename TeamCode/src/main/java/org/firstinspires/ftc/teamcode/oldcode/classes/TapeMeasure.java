package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class TapeMeasure implements Runnable{

    private Servo liftL, liftR, zAxis, xAxis;
    private CRServo  feed;
    public static double zInit = 0, xInit = 1;
    public static double xOpen = .35, targetZ, zOpen, liftPos = .45, liftLPos, liftRPos, aPosZB, aPosXB, aPosZR, aPosXR;
    public static double leftBound = 0, rightBound = 1, bottomBound = 0, topBound = .5, targetXAxis;
    private double lastAngle = 0, globalAngle = 0, baseAngle = 0, slowSpeed;
    private boolean toggle = false, contFeed = false, toggle2 = false;
    private boolean isOpen = false, controlsActive = false;
    private Side side;
    Gamepad gamepad1, gamepad2;
    String telemetry;

    public TapeMeasure(HardwareMap hardwareMap, Gamepad gamepadOne, Gamepad gamepadTwo, Side side){
        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        zAxis = hardwareMap.get(Servo.class, "zAxis");
        xAxis = hardwareMap.get(Servo.class, "xAxis");
        feed = hardwareMap.get(CRServo.class, "feed");

        zAxis.scaleRange(leftBound, rightBound);
        liftL.scaleRange(0, .37);
        liftR.scaleRange(0, .33);

        gamepad1 = gamepadOne;
        gamepad2 = gamepadTwo;

        liftL.setPosition(0);
        liftR.setPosition(1);
        zAxis.setPosition(zInit);
        xAxis.setPosition(xInit);

        this.side = side;

        if(side.equals(Side.BLUE)){
            zOpen = .7;
        }else if (side.equals(Side.RED)){
            zOpen = .96;
        }

        telemetry = "gyro is calibrated";
    }

    public void open (){
        liftL.setPosition(.96);
        liftR.setPosition(.05);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        xAxis.setPosition(xOpen);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        zAxis.setPosition(zOpen);
        feed.setPower(-1);
        targetXAxis = xAxis.getPosition();
        targetZ = zAxis.getPosition();
        contFeed = true;
        isOpen = true;
    }

    public void close(){
        zAxis.setPosition(zInit);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        xAxis.setPosition(xInit);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        liftL.setPosition(0.68);
        liftR.setPosition(1);
        targetZ = zAxis.getPosition();
        targetXAxis = xAxis.getPosition();
        contFeed = false;
        isOpen = false;
    }

    public void extend(double power){
        feed.setPower(power);
    }

    public boolean isOpen(){
        return isOpen;
    }

    public void alternate(){
        if(isOpen){
            close();
        }else{
            open();
        }
    }

    public boolean isControlsActive(){
        return controlsActive;
    }

    public String toString(){
        return telemetry;
    }

    public enum Side{
        RED,
        BLUE
    }

    public void run(){
        while(!Thread.interrupted()) {
            if(gamepad1.b){
                if (side.equals(Side.BLUE)) {
                    targetZ = aPosZB;
                    targetXAxis = aPosXB;
                } else if (side.equals(Side.RED)) {
                    targetZ = aPosZR;
                    targetXAxis = aPosXR;
                }
            }

            if(gamepad1.start){
                if(toggle2){
                    controlsActive = !controlsActive;
                    toggle2 = false;
                }
            }else{
                toggle2 = true;
            }

            if (gamepad1.back || gamepad2.start) {
                if (toggle) {
                    alternate();
                    toggle = false;
                }
            } else {
                toggle = true;
            }

            if (contFeed && isOpen) {
                extend(-1);
            }

            if(gamepad2.left_stick_button){
                extend(1);
            }

            if (controlsActive) {
                if(!gamepad2.left_stick_button) {
                    if (isOpen()) {
                        if (Math.abs(gamepad1.left_stick_y) > 0) {
                            extend(gamepad1.left_stick_y *slowSpeed);
                            contFeed = false;
                        } else if (!contFeed && gamepad1.left_stick_y == 0) {
                            extend(0);
                        }
                    } else {
                        extend(0);
                    }
                }else{
                    extend(1);
                }

                if (isOpen) {
                    targetZ += gamepad1.right_stick_x * .001 * slowSpeed;
                    targetXAxis += -gamepad1.right_stick_y * .0005 * slowSpeed;

                    if (gamepad1.left_trigger > 0) {
                        slowSpeed = .5;
                    } else {
                        slowSpeed = 1;
                    }

                    targetZ = Range.clip(targetZ, 0, 1);
                    targetXAxis = Range.clip(targetXAxis, 0, 1);

                    zAxis.setPosition(targetZ);
                    xAxis.setPosition(targetXAxis);
                }
            }
//            liftL.setPosition(liftLPos);
//            liftR.setPosition(liftRPos);
        }
    }
}
