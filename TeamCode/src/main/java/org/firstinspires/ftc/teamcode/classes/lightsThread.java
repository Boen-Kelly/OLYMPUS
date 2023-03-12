package org.firstinspires.ftc.teamcode.classes;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lightsThread{

    RevBlinkinLedDriver leds;

    public lightsThread (HardwareMap hardwareMap){
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    public void blue(){
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void green(){
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

}
