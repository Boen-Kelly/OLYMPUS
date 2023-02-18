package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeThread extends Thread{
    CRServo Slurper;
    Gamepad gamepad1;

    public IntakeThread(HardwareMap hardwareMap, Gamepad gamepad1){
        Slurper = hardwareMap.get(CRServo.class, "Slurper");
        this.gamepad1 = gamepad1;
    }
    public void run(){
        while (!Thread.interrupted()) {
            if(gamepad1.a){
                Slurper.setPower(-1);

            }else if(gamepad1.y){
                Slurper.setPower(1);
            }else if(gamepad1.b){
                Slurper.setPower(0);
            }
        }
    }
}
