package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
public class AutoAlignTest extends LinearOpMode {
    public void runOpMode(){
        DcMotor bl, br, fl, fr;

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fr = hardwareMap.get(DcMotor.class, "fr");
//
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()){

//            fl.setPower(pipeline.align(.75, .5));
//            br.setPower(-pipeline.align(.75, .5));
//            bl.setPower(pipeline.align(.75, .5));
//            fr.setPower(-pipeline.align(.75, .5));

//            if(pipeline.getPolePos().equals(AutoAlignPipeline.polePos.RIGHT)){
//                bl.setPower(.3);
//                fl.setPower(.3);
//                br.setPower(-.3);
//                fr.setPower(-.3);
//            }else if(pipeline.getPolePos().equals(AutoAlignPipeline.polePos.LEFT)){
//                bl.setPower(-.3);
//                fl.setPower(-.3);
//                br.setPower(.3);
//                fr.setPower(.3);
//            }else{
//                bl.setPower(0);
//                fl.setPower(0);
//                br.setPower(0);
//                fr.setPower(0);
//            }
//            pipeline.aimCam();

//            telemetry.addData("Pipeline says", pipeline);
//            telemetry.addData("width", pipeline.width());
//            telemetry.addData("height", pipeline.height());
//            telemetry.addData("angle", pipeline.angle());
//            telemetry.update();
        }
    }
}
