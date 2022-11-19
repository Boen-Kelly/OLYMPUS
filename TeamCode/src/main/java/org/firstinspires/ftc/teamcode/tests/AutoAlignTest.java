package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
public class AutoAlignTest extends LinearOpMode {
    public void runOpMode(){
        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 1");

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Pipeline says", pipeline);
            telemetry.update();
        }
    }
}
