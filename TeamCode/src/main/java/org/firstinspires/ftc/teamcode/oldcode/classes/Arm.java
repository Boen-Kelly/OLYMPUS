package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private DcMotor armL, armR;
    private Servo gate, bottom;

    public Arm (HardwareMap hardwareMap){
        armL = hardwareMap.get(DcMotor.class, "arm");
        armR = hardwareMap.get(DcMotor.class, "arm2");
        gate = hardwareMap.get(Servo.class, "gate");
        bottom = hardwareMap.get(Servo.class, "bottom");

        armR.setDirection(DcMotorSimple.Direction.REVERSE);

        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gate.setPosition(.1);
        bottom.setPosition(1);

        armL.setTargetPosition(0);
        armR.setTargetPosition(armL.getTargetPosition());
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armR.setMode(armL.getMode());
        armL.setPower(1);
        armR.setPower(armL.getPower());
    }

    public void moveArm(int targetPos, double power){
        if (targetPos <= 0 && targetPos >= -400) {
            armL.setTargetPosition(targetPos);
            armR.setTargetPosition(armL.getTargetPosition());
            armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armR.setMode(armL.getMode());
            armL.setPower(power);
            armR.setPower(armL.getPower());
        }
    }

    public void open(){
        if(armL.getCurrentPosition() <= -300){
            gate.setPosition(.37);
            bottom.setPosition(1);
        }else if(armL.getCurrentPosition() >= -100){
            gate.setPosition(.37);
            bottom.setPosition(1);
        }else if(-100 > armL.getCurrentPosition() && armL.getCurrentPosition() > -300){
            bottom.setPosition(0);
            gate.setPosition(.1);
        }
    }

    public void open(double gatePos, double bottomPos){
        gate.setPosition(gatePos);
        bottom.setPosition(bottomPos);
    }

    public void close(){
        gate.setPosition(.1);
        bottom.setPosition(1);
    }
}
