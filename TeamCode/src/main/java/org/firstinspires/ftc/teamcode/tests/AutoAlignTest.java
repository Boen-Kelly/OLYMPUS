package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
public class AutoAlignTest extends LinearOpMode {
    public void runOpMode(){
        DcMotor bl, br, fl, fr;

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 1");

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            if(pipeline.getPos().equals(AutoAlignPipeline.polePos.LEFT)){
                bl.setPower(.2);
                fl.setPower(.2);
                br.setPower(-.2);
                fr.setPower(-.2);
            }else if(pipeline.getPos().equals(AutoAlignPipeline.polePos.RIGHT)){
                bl.setPower(-.2);
                fl.setPower(-.2);
                br.setPower(.2);
                fr.setPower(.2);
            }else{
                bl.setPower(0);
                fl.setPower(0);
                br.setPower(0);
                fr.setPower(0);
            }


            telemetry.addData("Pipeline says", pipeline);
            telemetry.update();
        }
    }
}
