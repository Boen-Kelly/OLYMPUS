package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.SignalSleeve;

@TeleOp
public class SignalSleeveTest extends LinearOpMode {
    public void runOpMode(){
        SignalSleeve detector = new SignalSleeve(hardwareMap, "Webcam 1");

        while(!isStarted()) {
            telemetry.addLine(detector +"");
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("target is", detector.getPosition());
            telemetry.update();
        }

    }
}
