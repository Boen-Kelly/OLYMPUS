package org.firstinspires.ftc.teamcode.oldcode.tests.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftArm implements Runnable{
    private DcMotor lift1, lift2, arm;
    private CRServo slurper;

    private boolean isLiftUp = false, intake = true;

    private int height = 0, bottom = 0;
    private double slurpPower = 0;

    public LiftArm(HardwareMap hardwareMap){
        lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        slurper = hardwareMap.get(CRServo.class, "Slurper");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setDirection(DcMotor.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slurper.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run() {
        while (!Thread.interrupted()){
            if(isLiftUp && (arm.getCurrentPosition() < -300 || intake)){
                lift1.setTargetPosition(height);
                lift2.setTargetPosition(lift1.getTargetPosition());
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(lift1.getMode());
                lift1.setPower(1);
                lift2.setPower(lift1.getPower());
            }else{
                lift1.setTargetPosition(bottom);
                lift2.setTargetPosition(lift1.getTargetPosition());
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(lift1.getMode());
                lift1.setPower(1);
                lift2.setPower(lift1.getPower());
            }

            if(!intake && isLiftUp){
                arm.setTargetPosition(-3400);
            }else{
                arm.setTargetPosition(0);
            }

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);

            slurper.setPower(slurpPower);
        }
    }

    public void lift (int height, boolean intake){
        isLiftUp = true;
        this.intake = intake;
        this.height = height;
    }

    public void lift (){
        isLiftUp = true;
    }

    public void drop(int bottom){
        isLiftUp = false;
        this.bottom = bottom;
    }

    public void drop () {
        isLiftUp = false;
        bottom = 0;
    }

    public void setSlurpPower(double power){
        slurpPower = power;
    }
}
