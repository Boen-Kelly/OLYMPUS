package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
@Config
public class AutoAlignTest extends LinearOpMode {
    public static double cameraPoint = .75;
    public void runOpMode(){
        DcMotor bl, br, fl, fr;
        DistanceSensor backDist, frontDist;

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        backDist = hardwareMap.get(DistanceSensor.class, "backDist");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        Rev2mDistanceSensor dist = (Rev2mDistanceSensor) backDist;

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!isStarted() && !isStopRequested()) {
            pipeline.setPipelines("pole", "pole");
            pipeline.frontPoleDetector.setColors(true, false, false);
            pipeline.backPoleDetector.setColors(true, false, false);
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()){

//            fl.setPower(pipeline.align(cameraPoint, .1, false));
//            br.setPower(-pipeline.align(cameraPoint, .1, false));
//            bl.setPower(pipeline.align(cameraPoint, .1, false));
//            fr.setPower(-pipeline.align(cameraPoint, .1, false));

            fl.setPower(pipeline.align(cameraPoint, .3, true));
            br.setPower(-pipeline.align(cameraPoint, .3, true));
            bl.setPower(pipeline.align(cameraPoint, .3, true));
            fr.setPower(-pipeline.align(cameraPoint, .3 , true));

//            pipeline.aimCam();


            telemetry.addData("Pipeline says", pipeline);
            telemetry.addData("back distance", backDist.getDistance(DistanceUnit.CM));
            telemetry.addData("front distance", frontDist.getDistance(DistanceUnit.CM));
            telemetry.addData("dist", dist.getDistance(DistanceUnit.CM));
            telemetry.addData("average", (backDist.getDistance(DistanceUnit.CM) + dist.getDistance(DistanceUnit.CM))/2);
//            telemetry.addData("width", pipeline.width());
//            telemetry.addData("height", pipeline.height());
//            telemetry.addData("angle", pipeline.angle());
//            telemetry.addData("distance speed", pipeline.targetDistance(.7,.5));
            telemetry.update();
        }
    }
}
