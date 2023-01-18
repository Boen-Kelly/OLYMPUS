package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.DualCameraClass;

@TeleOp
public class DualCameraClassTest extends LinearOpMode {
    public void runOpMode(){
        DualCameraClass twoCameras = new DualCameraClass(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

        }
    }
}
