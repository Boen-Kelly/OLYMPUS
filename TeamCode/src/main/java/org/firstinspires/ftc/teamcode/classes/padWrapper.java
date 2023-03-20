package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

public class padWrapper {
    private Gamepad gamepad1, gamepad2;
    private boolean toggle = true, toggleVal = false;
    private int num = 0;

    public padWrapper(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public int incrUp(String user, String button, int maxval){
        String pressed = user + "." + button;



        if(num > maxval){
            num = 0;
        }

        return num;
    }


    public enum button {
        A,
        B,
        X,
        Y,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        RIGHT_BUMPER,
        LEFT_BUMPER
    }
}
